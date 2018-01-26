/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-vmalloc.h>
#include <uapi/video/v4l2_camera_ext_ctrls.h>
#include <uapi/video/v4l2_camera_ext_events.h>

#include "camera_ext.h"
#include "connection.h"
#include "kernel_ver.h"

#define CAM_EXT_CTRL_NUM_HINT   100
#define MAX_RETRY_TIMES         50

#define CAM_DEV_FROM_V4L2_CTRL(ctrl) \
	container_of(ctrl->handler, struct camera_ext, hdl_ctrls)

#define TO_CAMERA_EXT_FH(file_handle) \
	container_of(file_handle, struct camera_ext_fh, fh)

struct metastream_data {
	struct vb2_queue vb2_q;
	struct mutex list_lock;
	struct list_head available_buffers;
};

struct camera_ext_v4l2 {
	struct regulator *cdsi_reg;
	struct mutex mod_mutex;
	int mod_users; /* number of active driver users */
	int open_mode;
	bool buf_requested;
	bool streaming;
	struct metastream_data strm;
};

struct vb2_metadata_buffer {
	struct vb2_buffer vb;
	struct list_head list;
};

struct camera_ext_fh {
	struct v4l2_fh fh;
};

struct camera_ext_v4l2 *g_v4l2_data;

static int g_open_mode = CAMERA_EXT_BOOTMODE_NORMAL;

#define MOD_V4L2_DRIVER_VERSION 1

static DEFINE_SPINLOCK(camera_ext_lock);

static DEVICE_INT_ATTR(open_mode, 0644, g_open_mode);

static bool is_open_mode_valid(int mode)
{
	return mode >= 0 && mode < CAMERA_EXT_BOOTMODE_MAX;
}

static void camera_ext_kref_release(struct kref *kref)
{
	struct camera_ext *cam;

	cam = container_of(kref, struct camera_ext, kref);
	kfree(cam);
}

static inline void camera_ext_get(struct camera_ext *cam)
{
	unsigned long flags;

	gb_connection_get(cam->connection);
	spin_lock_irqsave(&camera_ext_lock, flags);
	kref_get(&cam->kref);
	spin_unlock_irqrestore(&camera_ext_lock, flags);
}

static inline void camera_ext_put(struct camera_ext *cam)
{
	unsigned long flags;
	struct gb_connection *conn = cam->connection;

	spin_lock_irqsave(&camera_ext_lock, flags);
	kref_put(&cam->kref, camera_ext_kref_release);
	spin_unlock_irqrestore(&camera_ext_lock, flags);
	gb_connection_put(conn);
}

static int query_cap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strlcpy(cap->driver, "mods v4l2 ctrl", sizeof(cap->driver));
	strlcpy(cap->card, "camera_ext mod", sizeof(cap->card));
	strlcpy(cap->bus_info, "greybus", sizeof(cap->card));
	cap->version = MOD_V4L2_DRIVER_VERSION;
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int input_enum(struct file *file, void *fh, struct v4l2_input *inp)
{
	struct camera_ext *cam_dev = video_drvdata(file);

	return gb_camera_ext_input_enum(cam_dev->connection, inp);
}

static int input_get(struct file *file, void *fh, unsigned int *i)
{
	struct camera_ext *cam_dev = video_drvdata(file);

	return gb_camera_ext_input_get(cam_dev->connection, i);
}

static int input_set(struct file *file, void *fh, unsigned int i)
{
	struct camera_ext *cam_dev = video_drvdata(file);

	return gb_camera_ext_input_set(cam_dev->connection, i);
}

static int fmt_enum(struct file *file, void *fh, struct v4l2_fmtdesc *fmt)
{
	struct camera_ext *cam_dev = video_drvdata(file);

	return gb_camera_ext_format_enum(cam_dev->connection, fmt);
}

static int fmt_get(struct file *file, void *fh,	struct v4l2_format *fmt)
{
	struct camera_ext *cam_dev;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		pr_err("%s: unsupport buffer type %d\n", __func__, fmt->type);
		return -EINVAL;
	}

	cam_dev = video_drvdata(file);
	return gb_camera_ext_format_get(cam_dev->connection, fmt);
}

static int fmt_set(struct file *file, void *fh,	struct v4l2_format *fmt)
{
	struct camera_ext *cam_dev;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		pr_err("%s: unsupport buffer type %d\n", __func__, fmt->type);
		return -EINVAL;
	}

	cam_dev = video_drvdata(file);
	return gb_camera_ext_format_set(cam_dev->connection, fmt);
}

static int frmsize_enum(struct file *file, void *fh,
			struct v4l2_frmsizeenum *frmsize)
{
	struct camera_ext *cam_dev = video_drvdata(file);

	return gb_camera_ext_frmsize_enum(cam_dev->connection, frmsize);
}

static int frmival_enum(struct file *file, void *fh,
			struct v4l2_frmivalenum *frmival)
{
	struct camera_ext *cam_dev = video_drvdata(file);

	return gb_camera_ext_frmival_enum(cam_dev->connection, frmival);
}

static int stream_on(struct file *file, void *fh,
			enum v4l2_buf_type buf_type)
{
	struct camera_ext *cam_dev = video_drvdata(file);
	struct metastream_data *strm = &g_v4l2_data->strm;
	struct vb2_queue *q = &strm->vb2_q;
	int ret;

	mutex_lock(&g_v4l2_data->mod_mutex);
	if (g_v4l2_data->streaming) {
		mutex_unlock(&g_v4l2_data->mod_mutex);
		return -EBUSY;
	}

	g_v4l2_data->streaming = true;

	if (g_v4l2_data->buf_requested)
		ret = vb2_streamon(q, buf_type);
	else
		ret = gb_camera_ext_stream_on(cam_dev->connection);
	mutex_unlock(&g_v4l2_data->mod_mutex);

	return ret;
}

static int stream_off(struct file *file, void *fh,
			enum v4l2_buf_type buf_type)
{
	struct camera_ext *cam_dev = video_drvdata(file);
	struct metastream_data *strm = &g_v4l2_data->strm;
	struct vb2_queue *q = &strm->vb2_q;
	int ret;

	mutex_lock(&g_v4l2_data->mod_mutex);
	g_v4l2_data->streaming = false;

	if (g_v4l2_data->buf_requested)
		ret = vb2_streamoff(q, buf_type);
	else
		ret = gb_camera_ext_stream_off(cam_dev->connection);
	mutex_unlock(&g_v4l2_data->mod_mutex);

	return ret;
}

static int stream_parm_get(struct file *file, void *fh,
			struct v4l2_streamparm *parm)
{
	struct camera_ext *cam_dev = video_drvdata(file);

	return gb_camera_ext_stream_parm_get(cam_dev->connection, parm);
}

static int stream_parm_set(struct file *file, void *fh,
			struct v4l2_streamparm *parm)
{
	struct camera_ext *cam_dev = video_drvdata(file);

	return gb_camera_ext_stream_parm_set(cam_dev->connection, parm);
}

static int subscribe_event(struct v4l2_fh *fh,
	const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_CAMERA_EXT_EVENT_TYPE:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	case V4L2_EVENT_CTRL:
		/* TODO: do we have ctrl event to report ? */
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

int camera_ext_mod_v4l2_buffer_notify(struct camera_ext *cam_dev,
				const char *desc, size_t size)
{
	struct vb2_metadata_buffer *available_buf;
	void *metadata;

	if (!vb2_is_streaming(&g_v4l2_data->strm.vb2_q))
		return 0;
	mutex_lock(&g_v4l2_data->strm.list_lock);
	available_buf = list_first_entry_or_null(
		&g_v4l2_data->strm.available_buffers,
		struct vb2_metadata_buffer, list);
	if (available_buf)
		list_del(&available_buf->list);
	mutex_unlock(&g_v4l2_data->strm.list_lock);

	if (available_buf) {
		metadata = vb2_plane_vaddr(&available_buf->vb, 0);
		if (metadata != NULL)
			memcpy(metadata, desc, size);
		vb2_set_plane_payload(&available_buf->vb, 0, size);
		vb2_buffer_done(&available_buf->vb, VB2_BUF_STATE_DONE);
	} else
		pr_warn("%s: buffer is not available\n", __func__);

	return 0;
}

static int _camera_ext_queue_setup(struct vb2_queue *q,
	const struct v4l2_format *fmt,
	unsigned int *num_buffers, unsigned int *num_planes,
	unsigned int sizes[], void *alloc_ctxs[])
{
	int size = CAMERA_EXT_EVENT_METADATA_DESC_LEN;

	*num_planes = 1;
	sizes[0] = size;

	return 0;
}

#ifdef V4L2_VIDEOBUF2_VOID_FORMAT
static int camera_ext_queue_setup(struct vb2_queue *q,
	const void *parg,
	unsigned int *num_buffers, unsigned int *num_planes,
	unsigned int sizes[], void *alloc_ctxs[])
{
	const struct v4l2_format *fmt = parg;

	return _camera_ext_queue_setup(q, fmt, num_buffers,
				num_planes, sizes, alloc_ctxs);
}
#else
#define camera_ext_queue_setup _camera_ext_queue_setup
#endif

static void camera_ext_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_metadata_buffer *buf = container_of(vb,
			struct vb2_metadata_buffer, vb);

	mutex_lock(&g_v4l2_data->strm.list_lock);
	list_add_tail(&buf->list, &g_v4l2_data->strm.available_buffers);
	mutex_unlock(&g_v4l2_data->strm.list_lock);
}

static int query_buf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct metastream_data *strm;
	struct vb2_queue *q;

	strm = &g_v4l2_data->strm;
	q = &strm->vb2_q;

	return vb2_querybuf(q, b);
}

static int camera_ext_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct camera_ext *cam_dev = vb2_get_drv_priv(q);

	return gb_camera_ext_stream_on(cam_dev->connection);
}

static void camera_ext_stop_streaming(struct vb2_queue *q)
{
	struct camera_ext *cam_dev = vb2_get_drv_priv(q);
	struct vb2_metadata_buffer *available_buf;

	if (g_v4l2_data->buf_requested) {
		mutex_lock(&g_v4l2_data->strm.list_lock);
		while ((available_buf = list_first_entry_or_null(
			&g_v4l2_data->strm.available_buffers,
			struct vb2_metadata_buffer, list)) != NULL) {
			list_del(&available_buf->list);
			vb2_buffer_done(&available_buf->vb,
				VB2_BUF_STATE_ERROR);
		}
		mutex_unlock(&g_v4l2_data->strm.list_lock);
		g_v4l2_data->buf_requested = false;
	}
	gb_camera_ext_stream_off(cam_dev->connection);
}

static struct vb2_ops camera_ext_vb2_ops = {

	.queue_setup = camera_ext_queue_setup,
	.buf_queue = camera_ext_buf_queue,
	.start_streaming = camera_ext_start_streaming,
	.stop_streaming = camera_ext_stop_streaming,
};

static int camera_ext_vb2_q_init(struct camera_ext *cam_dev,
					struct vb2_queue *q)
{
	memset(q, 0, sizeof(struct vb2_queue));
	q->drv_priv = cam_dev;
	q->mem_ops = &vb2_vmalloc_memops;
	q->ops = &camera_ext_vb2_ops;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->buf_struct_size = sizeof(struct vb2_metadata_buffer);

	return vb2_queue_init(q);
}


static int request_bufs(struct file *file, void *fh,
	struct v4l2_requestbuffers *req)
{
	struct metastream_data *strm = &g_v4l2_data->strm;
	struct vb2_queue *q = &strm->vb2_q;
	int ret;

	mutex_lock(&g_v4l2_data->mod_mutex);
	if (g_v4l2_data->streaming
		|| g_v4l2_data->buf_requested) {
		mutex_unlock(&g_v4l2_data->mod_mutex);
		return -EBUSY;
	}

	INIT_LIST_HEAD(&g_v4l2_data->strm.available_buffers);

	ret = vb2_reqbufs(q, req);
	g_v4l2_data->buf_requested = (req->count > 0 && ret == 0);

	mutex_unlock(&g_v4l2_data->mod_mutex);

	return ret;
}

static int queue_buf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct metastream_data *strm;
	struct vb2_queue *q;

	strm = &g_v4l2_data->strm;
	q = &strm->vb2_q;
	return vb2_qbuf(q, b);
}

static int dequeue_buf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct metastream_data *strm = &g_v4l2_data->strm;
	struct vb2_queue *q = &strm->vb2_q;

	return vb2_dqbuf(q, b, file->f_flags & O_NONBLOCK);
}

/* This device is used to query mod capabilities and config mod stream.
 * It does not support video buffer related operations.
 */
static const struct v4l2_ioctl_ops camera_ext_v4l2_ioctl_ops = {
	.vidioc_querycap		= query_cap,
	.vidioc_enum_input		= input_enum,
	.vidioc_g_input			= input_get,
	.vidioc_s_input			= input_set,
	.vidioc_enum_fmt_vid_cap	= fmt_enum,
	.vidioc_g_fmt_vid_cap		= fmt_get,
	.vidioc_s_fmt_vid_cap		= fmt_set,
	.vidioc_enum_framesizes		= frmsize_enum,
	.vidioc_enum_frameintervals	= frmival_enum,
	.vidioc_streamon		= stream_on,
	.vidioc_streamoff		= stream_off,
	.vidioc_g_parm			= stream_parm_get,
	.vidioc_s_parm			= stream_parm_set,
	.vidioc_querybuf		= query_buf,
	.vidioc_reqbufs			= request_bufs,
	.vidioc_qbuf			= queue_buf,
	.vidioc_dqbuf			= dequeue_buf,
	.vidioc_subscribe_event		= subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

static int mod_v4l2_reg_control(bool on)
{
	int rc;

	/* NO-OP if there is no platform device probed */
	if (!g_v4l2_data)
		return 0;

	if (on)
		rc = regulator_enable(g_v4l2_data->cdsi_reg);
	else
		rc = regulator_disable(g_v4l2_data->cdsi_reg);

	return rc;
}

void camera_ext_mod_v4l2_event_notify(struct camera_ext *cam_dev,
	struct v4l2_camera_ext_event *event)
{
	struct v4l2_event ev;

	memset(&ev, 0, sizeof(ev));
	ev.type = V4L2_CAMERA_EXT_EVENT_TYPE;
	if (sizeof(*event) <= sizeof(ev.u.data))
		memcpy(ev.u.data, event, sizeof(*event));
	else
		pr_warn("%s: incorrect event size\n", __func__);
	v4l2_event_queue(cam_dev->vdev_mod, &ev);
}

static void reset_ctrls_value(struct v4l2_ctrl_handler *hdl)
{
	struct v4l2_ctrl *ctrl;

	list_for_each_entry(ctrl, &hdl->ctrls, node) {
		if (ctrl->type == V4L2_CTRL_TYPE_BUTTON ||
			(ctrl->flags & V4L2_CTRL_FLAG_READ_ONLY))
				continue;
		if (ctrl->is_ptr) {
			struct camera_ext_v4l2_ctrl_priv *priv = ctrl->priv;

			if (priv->def != NULL && priv->def_size > 0) {
				memcpy(ctrl->p_cur.p_char, priv->def,
							priv->def_size);
				memcpy(ctrl->p_new.p_char, priv->def,
							priv->def_size);
			}
		} else if (ctrl->type == V4L2_CTRL_TYPE_INTEGER64)
			*ctrl->p_cur.p_s64 = *ctrl->p_new.p_s64 =
						ctrl->default_value;
		else
			ctrl->cur.val = ctrl->val = ctrl->default_value;
	}
}

static int mod_v4l2_open(struct file *file)
{
	int rc;
	struct camera_ext_fh *camera_fh;
	struct camera_ext *cam_dev = video_drvdata(file);
	int retry_count = 0;
	int mode = g_open_mode;

	if (!is_open_mode_valid(mode)) {
		mode = CAMERA_EXT_BOOTMODE_NORMAL;
	}

	camera_fh = kzalloc(sizeof(*camera_fh), GFP_KERNEL);
	if (!camera_fh)
		return -ENOMEM;

	camera_ext_get(cam_dev);
	v4l2_fh_init(&camera_fh->fh, cam_dev->vdev_mod);
	v4l2_fh_add(&camera_fh->fh);
	file->private_data = &camera_fh->fh;

	mutex_lock(&g_v4l2_data->mod_mutex);
	if (g_v4l2_data->mod_users) {
		if ((CAMERA_EXT_BOOTMODE_DFU == g_v4l2_data->open_mode) ||
		    (CAMERA_EXT_BOOTMODE_DFU == mode)) {
			rc = -EBUSY;
			goto err_open;
		}
		else if (g_v4l2_data->open_mode == mode) {
			goto user_present;
		}
		else if (CAMERA_EXT_BOOTMODE_PREVIEW == g_v4l2_data->open_mode) {
			// it is in preview mode and no change should be made
			pr_warn("%s: already in preview mode\n", __func__);
			rc = -EBUSY;
			goto err_open;
		}
		/* MOD has been opened by other user in different mode.
		 * Powered it off first and reopen it in the new mode.
		 */
		pr_warn("%s: close previous instance\n", __func__);
		gb_camera_ext_power_off(cam_dev->connection);
	}

	/* init MOD */
	rc = mod_v4l2_reg_control(true);
	if (rc < 0)
		goto err_open;

	g_v4l2_data->open_mode = mode;

	rc = gb_camera_ext_power_on(cam_dev->connection,
				    g_v4l2_data->open_mode);

	/* Open could fail if the mod is out of sync, retry once */
	if (rc < 0) {
		gb_camera_ext_power_off(cam_dev->connection);
		pr_err("%s - Camera power On failed, close and open\n",
								 __func__);
		rc = gb_camera_ext_power_on(cam_dev->connection,
					    g_v4l2_data->open_mode);
	}

	if (rc < 0)
		goto err_power_on;

	/* reset all none readonly controls */
	reset_ctrls_value(&cam_dev->hdl_ctrls);
	do {
		rc = v4l2_ctrl_handler_setup(&cam_dev->hdl_ctrls);
	} while (rc == -EAGAIN && ++retry_count < MAX_RETRY_TIMES);
	if (rc != 0) {
		v4l2_err(&cam_dev->v4l2_dev,
			 "failed to apply contrl default value\n");
		goto err_set_ctrl_def;
	}

	rc = camera_ext_vb2_q_init(cam_dev, &g_v4l2_data->strm.vb2_q);
	if (rc)
		goto err_set_ctrl_def;

	g_v4l2_data->buf_requested = false;
	g_v4l2_data->streaming = false;

user_present:
	++g_v4l2_data->mod_users;
	mutex_unlock(&g_v4l2_data->mod_mutex);

	return 0;

err_set_ctrl_def:
	gb_camera_ext_power_off(cam_dev->connection);
err_power_on:
	mod_v4l2_reg_control(false);
err_open:
	mutex_unlock(&g_v4l2_data->mod_mutex);
	v4l2_fh_del(&camera_fh->fh);
	v4l2_fh_exit(&camera_fh->fh);
	kfree(camera_fh);
	camera_ext_put(cam_dev);

	return rc;
}

static int mod_v4l2_close(struct file *file)
{
	int ret = 0;
	struct camera_ext_fh *camera_fh;
	struct camera_ext *cam_dev = video_drvdata(file);

	camera_fh = TO_CAMERA_EXT_FH(file->private_data);
	file->private_data = NULL;
	v4l2_fh_del(&camera_fh->fh);
	v4l2_fh_exit(&camera_fh->fh);
	kfree(camera_fh);

	mutex_lock(&g_v4l2_data->mod_mutex);
	--g_v4l2_data->mod_users;
	if (g_v4l2_data->mod_users == 0) {
		vb2_queue_release(&g_v4l2_data->strm.vb2_q);
		mod_v4l2_reg_control(false);
		g_open_mode = CAMERA_EXT_BOOTMODE_NORMAL;
		if (cam_dev->state == CAMERA_EXT_READY)
			ret = gb_camera_ext_power_off(cam_dev->connection);
	}
	mutex_unlock(&g_v4l2_data->mod_mutex);

	camera_ext_put(cam_dev);
	return ret;
}

static unsigned int mod_v4l2_poll(struct file *file,
		struct poll_table_struct *pll_table)
{
	int ret = 0;
	struct metastream_data *strm = &g_v4l2_data->strm;
	struct vb2_queue *q = &strm->vb2_q;
	struct v4l2_fh *fh = file->private_data;

	if (vb2_is_streaming(q)) {
		ret = vb2_poll(q, file, pll_table);
	} else {
		poll_wait(file, &fh->wait, pll_table);
		if (v4l2_event_pending(fh))
			ret = POLLPRI;
	}

	return ret;
}

static int mod_v4l2_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct metastream_data *strm = &g_v4l2_data->strm;
	struct vb2_queue *q = &strm->vb2_q;

	return vb2_mmap(q, vma);
}

static struct v4l2_file_operations camera_ext_mod_v4l2_fops = {
	.owner		= THIS_MODULE,
	.open		= mod_v4l2_open,
	.unlocked_ioctl	= video_ioctl2,
	.release	= mod_v4l2_close,
	.poll		= mod_v4l2_poll,
	.mmap		= mod_v4l2_mmap,
};

static int mod_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct camera_ext *cam_dev = CAM_DEV_FROM_V4L2_CTRL(ctrl);
	struct gb_connection *conn = cam_dev->connection;

	return gb_camera_ext_g_volatile_ctrl(conn, ctrl);
}

static int mod_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct camera_ext *cam_dev = CAM_DEV_FROM_V4L2_CTRL(ctrl);
	struct gb_connection *conn = cam_dev->connection;

	return gb_camera_ext_s_ctrl(conn, ctrl);
}

static int mod_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct camera_ext *cam_dev = CAM_DEV_FROM_V4L2_CTRL(ctrl);
	struct gb_connection *conn = cam_dev->connection;

	return gb_camera_ext_try_ctrl(conn, ctrl);
}

static const struct v4l2_ctrl_ops mod_ctrl_ops = {
	.g_volatile_ctrl = mod_g_volatile_ctrl,
	.s_ctrl = mod_s_ctrl,
	.try_ctrl = mod_try_ctrl,
};

static int custom_ctrl_register(
	struct camera_ext_predefined_ctrl_v4l2_cfg *mod_cfg, void *ctx)
{
	struct camera_ext_v4l2_ctrl_priv *priv;
	struct v4l2_ctrl *ctrl;
	struct camera_ext *cam_dev = ctx;
	struct v4l2_ctrl_config *cfg = camera_ext_get_ctrl_config(mod_cfg->id);

	if (cfg == NULL)
		return -EINVAL;

	cfg->ops = &mod_ctrl_ops;
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
	priv->idx = mod_cfg->idx;

	if (cfg->flags & CAMERA_EXT_CTRL_FLAG_NEED_MIN)
		cfg->min = mod_cfg->min;
	if (cfg->flags & CAMERA_EXT_CTRL_FLAG_NEED_MAX)
		cfg->max = mod_cfg->max;
	if (cfg->flags & CAMERA_EXT_CTRL_FLAG_NEED_STEP)
		cfg->step = mod_cfg->step;
	if (cfg->flags & CAMERA_EXT_CTRL_FLAG_NEED_DEF)
		switch (cfg->type) {
		case V4L2_CTRL_TYPE_INTEGER64:
			cfg->def = *(int64_t *)mod_cfg->p_def;
			break;
		case V4L2_CTRL_TYPE_INTEGER:
		case V4L2_CTRL_TYPE_BOOLEAN:
		case V4L2_CTRL_TYPE_INTEGER_MENU:
		case V4L2_CTRL_TYPE_MENU:
		case V4L2_CTRL_TYPE_BITMASK:
			cfg->def = *(int32_t *)mod_cfg->p_def;
			break;
		default:
			/* keep def as predefined */
			break;
		}

	if (cfg->flags & CAMERA_EXT_CTRL_FLAG_NEED_DIMS)
		memcpy(cfg->dims, mod_cfg->dims, sizeof(cfg->dims));
	if (cfg->flags & CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK)
		cfg->menu_skip_mask = mod_cfg->menu_skip_mask;

	if (cfg->flags & CAMERA_EXT_CTRL_FLAG_NEED_MENU_INT)
		memcpy((s64*)cfg->qmenu_int, mod_cfg->menu_int,
			sizeof(u64) * CAMERA_EXT_MAX_MENU_NUM);

	if (cfg->flags & CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT) {
		int i;
		camera_ext_ctrl_float *mem_pool =
			(camera_ext_ctrl_float *)cfg->qmenu[0];
		memcpy(mem_pool, mod_cfg->menu_float,
		sizeof(camera_ext_ctrl_float) * CAMERA_EXT_MAX_MENU_NUM);

		/* qmenu[0] always points to the mem pool */
		for (i = 1; i < CAMERA_EXT_MAX_MENU_NUM; i++) {
			/* cast it to writable memory */
			char **qmenu = (char**)&cfg->qmenu[i];
			*qmenu = mem_pool[i];
			if (cfg->qmenu[i][0] == 0) {
				/* reach the end. NULL is required by v4l2 */
				*qmenu = NULL;
				break;
			}
		}
	}

	/* If v4l2_ctrl_new_custom succeeds, priv will be free'd when cleanup all
	 * controls in release_ctrl_priv_mem. Otherwise, it should be free'd here.*/
	ctrl = v4l2_ctrl_new_custom(&cam_dev->hdl_ctrls, cfg, priv);

	if (ctrl == NULL) {
		pr_err("register id 0x%x failed\n", mod_cfg->id);
		kfree(priv);
		return -EINVAL;
	} else if (cfg->flags & CAMERA_EXT_CTRL_FLAG_NEED_DEF
		&& (ctrl->elems > 1
		|| (cfg->type == V4L2_CTRL_TYPE_STRING))) {
		/* update default value for array or string control */
		size_t size = ctrl->elem_size * ctrl->elems;

		if (size != mod_cfg->val_size) {
			pr_err("%s: wrong value size from mod %zu, expected %zu\n",
				__func__, size, mod_cfg->val_size);
			return -EINVAL;
		}
		memcpy(ctrl->p_cur.p_char, mod_cfg->p_def, size);
		memcpy(ctrl->p_new.p_char, mod_cfg->p_def, size);
		priv->def = kmalloc(size, GFP_KERNEL);
		if (priv->def == NULL)
			return -ENOMEM;
		memcpy(priv->def, mod_cfg->p_def, size);
		priv->def_size = size;
	}
	return 0;
}

static const struct v4l2_ctrl_config mod_ctrl_class = {
	.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_WRITE_ONLY,
	.id = MOD_CID_MOD_CLASS,
	.name = "Mod controls",
	.type = V4L2_CTRL_TYPE_CTRL_CLASS,
};

static void camera_ext_dev_release(struct video_device *dev)
{
	struct camera_ext *cam = video_get_drvdata(dev);

	video_device_release(dev);
	camera_ext_put(cam);
}

static void release_ctrl_priv_mem(struct v4l2_ctrl_handler *hdl)
{
	struct camera_ext_v4l2_ctrl_priv *priv;
	struct v4l2_ctrl *ctrl;

	list_for_each_entry(ctrl, &hdl->ctrls, node) {
		priv = ctrl->priv;
		if (priv == NULL) /* mod_ctrl_class ctrl */
			continue;
		if (priv->def != NULL)
			kfree(priv->def);
		kfree(priv);
	}
}

/* TODO: make this function asynchronously if it takes too long */
int camera_ext_mod_v4l2_init(struct camera_ext *cam_dev)
{
	int retval;
	struct gb_connection *conn = cam_dev->connection;
	g_open_mode = CAMERA_EXT_BOOTMODE_NORMAL;

	v4l2_ctrl_handler_init(&cam_dev->hdl_ctrls, CAM_EXT_CTRL_NUM_HINT);
	v4l2_ctrl_new_custom(&cam_dev->hdl_ctrls, &mod_ctrl_class, NULL);

	retval = gb_camera_ext_ctrl_process_all(conn, custom_ctrl_register,
			cam_dev);

	if (retval != 0 || cam_dev->hdl_ctrls.error != 0) {
		pr_err("%s: failed to process ctrl\n", __func__);
		goto error_ctrl_process;
	}

	snprintf(cam_dev->v4l2_dev.name, sizeof(cam_dev->v4l2_dev.name),
				"%s", CAMERA_EXT_DEV_NAME);
	retval = v4l2_device_register(NULL, &cam_dev->v4l2_dev);
	if (retval) {
		pr_err("failed to register v4l2 device\n");
		goto error_ctrl_process;
	}

	cam_dev->vdev_mod = video_device_alloc();
	if (cam_dev->vdev_mod == NULL) {
		pr_err("failed to allocate video_device\n");
		goto error_alloc_vdev;
	}

	snprintf(cam_dev->vdev_mod->name, sizeof(cam_dev->vdev_mod->name),
				"%s", CAMERA_EXT_DEV_NAME);
	cam_dev->vdev_mod->ctrl_handler = &cam_dev->hdl_ctrls;
	cam_dev->vdev_mod->v4l2_dev = &cam_dev->v4l2_dev;
	cam_dev->vdev_mod->release = camera_ext_dev_release;
	cam_dev->vdev_mod->fops = &camera_ext_mod_v4l2_fops;
	cam_dev->vdev_mod->ioctl_ops = &camera_ext_v4l2_ioctl_ops;
	cam_dev->vdev_mod->vfl_type = VFL_TYPE_MOT_GRABBER;

	video_set_drvdata(cam_dev->vdev_mod, cam_dev);
	retval = video_register_device(cam_dev->vdev_mod,
				VFL_TYPE_MOT_GRABBER, -1);
	if (retval) {
		pr_err("%s: failed to register video device. rc %d\n",
			__func__, retval);
		goto error_reg_vdev;
	}

	retval = device_create_file(&cam_dev->vdev_mod->dev,
				    &dev_attr_open_mode.attr);
	if (retval) {
		pr_err("failed to create open_mode sysfs entry\n");
		goto error_sysfs;
	}

	camera_ext_get(cam_dev);
	kobject_uevent(&cam_dev->vdev_mod->dev.kobj, KOBJ_ADD);

	return retval;
error_sysfs:
	video_unregister_device(cam_dev->vdev_mod);
error_reg_vdev:
	video_device_release(cam_dev->vdev_mod);
error_alloc_vdev:
	v4l2_device_unregister(&cam_dev->v4l2_dev);
error_ctrl_process:
	release_ctrl_priv_mem(&cam_dev->hdl_ctrls);
	v4l2_ctrl_handler_free(&cam_dev->hdl_ctrls);
	return retval;
}

void camera_ext_mod_v4l2_exit(struct camera_ext *cam_dev)
{
	video_unregister_device(cam_dev->vdev_mod);
	device_remove_file(cam_dev->v4l2_dev.dev, &dev_attr_open_mode.attr);
	v4l2_device_unregister(&cam_dev->v4l2_dev);
	release_ctrl_priv_mem(&cam_dev->hdl_ctrls);
	v4l2_ctrl_handler_free(&cam_dev->hdl_ctrls);
	camera_ext_put(cam_dev);
}

static int camera_ext_v4l2_probe(struct platform_device *pdev)
{
	struct camera_ext_v4l2 *data;

	if (!pdev->dev.of_node) {
		/* Platform data not currently supported */
		dev_err(&pdev->dev, "%s: of devtree not found\n", __func__);
		return -EINVAL;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cdsi_reg = devm_regulator_get(&pdev->dev, "camera_ext_cdsi");
	if (IS_ERR(data->cdsi_reg)) {
		dev_err(&pdev->dev, "%s: failed to get cdsi regulator.\n",
			__func__);
		return PTR_ERR(data->cdsi_reg);
	}

	mutex_init(&data->mod_mutex);
	mutex_init(&data->strm.list_lock);
	g_v4l2_data = data;

	return 0;
}

static int camera_ext_v4l2_remove(struct platform_device *pdev)
{
	mutex_destroy(&g_v4l2_data->strm.list_lock);
	g_v4l2_data = NULL;

	return 0;
}

static const struct of_device_id camera_ext_match[] = {
	{.compatible = "mmi,mods-camera-ext",},
	{},
};

static const struct platform_device_id camera_ext_id_table[] = {
	{"camera_ext", 0},
	{},
};

static struct platform_driver camera_ext_v4l2_driver = {
	.driver = {
		.name = "camera_ext_v4l2",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(camera_ext_match),
	},
	.probe = camera_ext_v4l2_probe,
	.remove = camera_ext_v4l2_remove,
	.id_table = camera_ext_id_table,
};

int camera_ext_v4l2_driver_init(void)
{
	return platform_driver_register(&camera_ext_v4l2_driver);
}

void camera_ext_v4l2_driver_exit(void)
{
	platform_driver_unregister(&camera_ext_v4l2_driver);
}
