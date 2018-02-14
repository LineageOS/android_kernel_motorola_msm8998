/*
 * Copyright (C) 2016 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/compat.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>

#include "v4l2_hal.h"
#include "v4l2_hal_internal.h"

#define V4L2_HAL_DRIVER_VERSION 1
#define HAL_DEV_NAME "v4l2_hal"

#define FH_TO_STREAM(hal_fh) \
	container_of(hal_fh, struct v4l2_stream_data, fh)

struct v4l2_buffer_data {
	int orig_fd;
	int mapped_fd;
};

struct v4l2_stream_data {
	unsigned int id;
	bool used;
	bool handled;
	struct vb2_queue vb2_q;
	size_t bcount;
	struct v4l2_buffer_data *bdata;
	struct v4l2_buffer_data cid_map[V4L2_HAL_MAX_NUM_MMAP_CID];
	struct v4l2_fh fh;
};

enum v4l2_hal_state {
	HAL_READY = 0,
	HAL_DESTROYED,
};

struct v4l2_hal_data {
	struct v4l2_device v4l2_dev;
	struct video_device *vdev;
	struct mutex lock;
	struct v4l2_stream_data strms[V4L2_HAL_MAX_STREAMS];

	struct kref kref;
	enum v4l2_hal_state state;
};

struct v4l2_format_compat {
	__u32 type;
	union {
		struct v4l2_pix_format	pix;
		__u8	raw_data[200];		  /* user-defined */
	} fmt;
};

#define VIDIOC_G_FMT32		_IOWR('V',	4, struct v4l2_format_compat)
#define VIDIOC_S_FMT32		_IOWR('V',	5, struct v4l2_format_compat)

static DEFINE_MUTEX(hal_data_lock);
static DEFINE_MUTEX(hal_state_lock);
static enum v4l2_hal_state hal_state = HAL_DESTROYED;

static void hal_data_kref_release(struct kref *kref)
{
	struct v4l2_hal_data *hal_data;

	hal_data = container_of(kref, struct v4l2_hal_data, kref);
	/* Reset the v4l2_dev pointer on video device node
	 * before calling kfree(hal_data).
	 */
	if(hal_data->vdev)
		hal_data->vdev->v4l2_dev = NULL;
	video_unregister_device(hal_data->vdev);
	v4l2_device_unregister(&hal_data->v4l2_dev);
	kfree(hal_data);
}

static inline void hal_data_get(struct v4l2_hal_data *data)
{
	mutex_lock(&hal_data_lock);
	kref_get(&data->kref);
	mutex_unlock(&hal_data_lock);
}

static inline void hal_data_put(struct v4l2_hal_data *data)
{
	mutex_lock(&hal_data_lock);
	kref_put(&data->kref, hal_data_kref_release);
	mutex_unlock(&hal_data_lock);
}

static int query_cap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strlcpy(cap->driver, HAL_DEV_NAME, sizeof(cap->driver));
	strlcpy(cap->card, HAL_DEV_NAME, sizeof(cap->card));
	strlcpy(cap->bus_info, "greybus", sizeof(cap->card));
	cap->version = V4L2_HAL_DRIVER_VERSION;
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int input_enum(struct file *file, void *fh, struct v4l2_input *inp)
{
	int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	ret = v4l2_misc_process_command(strm->id, VIDIOC_ENUMINPUT,
					sizeof(*inp), inp);

	return ret;
}

static int input_get(struct file *file, void *fh, unsigned int *i)
{
	int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	ret = v4l2_misc_process_command(strm->id, VIDIOC_G_INPUT,
					sizeof(*i), i);
	return ret;
}

static int input_set(struct file *file, void *fh,
			 unsigned int i)
{
	int ret;
	unsigned int tmp = i;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	ret = v4l2_misc_process_command(strm->id, VIDIOC_S_INPUT,
					sizeof(tmp), &tmp);

	return ret;
}

static int fmt_enum(struct file *file, void *fh,
			struct v4l2_fmtdesc *fmt)
{
	int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	ret = v4l2_misc_process_command(strm->id, VIDIOC_ENUM_FMT,
					sizeof(*fmt), fmt);

		return ret;
}

static int fmt_get(struct file *file, void *fh,
		   struct v4l2_format *fmt)
{
	int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	/* G_FMT required special handling due to incompatiblity
	   of v4l2_format structure */
	if (v4l2_misc_compat_mode()) {
		struct v4l2_format_compat cfmt;

		cfmt.type = fmt->type;
		memcpy(&cfmt.fmt, &fmt->fmt, sizeof(fmt->fmt));
		ret = v4l2_misc_process_command(strm->id, VIDIOC_G_FMT32,
						sizeof(cfmt), &cfmt);
		fmt->type = cfmt.type;
		memcpy(&fmt->fmt, &cfmt.fmt, sizeof(fmt->fmt));
	} else
		ret = v4l2_misc_process_command(strm->id, VIDIOC_G_FMT,
						sizeof(*fmt), fmt);

	return ret;
}

static int fmt_set(struct file *file, void *fh,
		   struct v4l2_format *fmt)
{
	int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	/* S_FMT required special handling due to incompatiblity
	   of v4l2_format structure */
	if (v4l2_misc_compat_mode()) {
		struct v4l2_format_compat cfmt;

		cfmt.type = fmt->type;
		memcpy(&cfmt.fmt, &fmt->fmt, sizeof(fmt->fmt));
		ret = v4l2_misc_process_command(strm->id, VIDIOC_S_FMT32,
						sizeof(cfmt), &cfmt);
		fmt->type = cfmt.type;
		memcpy(&fmt->fmt, &cfmt.fmt, sizeof(fmt->fmt));
	} else
		ret = v4l2_misc_process_command(strm->id, VIDIOC_S_FMT,
						sizeof(*fmt), fmt);

	return ret;
}

static int frmsize_enum(struct file *file, void *fh,
			struct v4l2_frmsizeenum *frmsize)
{
	int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	ret = v4l2_misc_process_command(strm->id, VIDIOC_ENUM_FRAMESIZES,
					sizeof(*frmsize), frmsize);

		return ret;
}

static int frmival_enum(struct file *file, void *fh,
			struct v4l2_frmivalenum *frmival)
{
	int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	ret = v4l2_misc_process_command(strm->id, VIDIOC_ENUM_FRAMEINTERVALS,
					sizeof(*frmival), frmival);

		return ret;
}


static int stream_parm_get(struct file *file, void *fh,
			   struct v4l2_streamparm *parm)
{
	int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	ret = v4l2_misc_process_command(strm->id, VIDIOC_G_PARM,
					sizeof(*parm), parm);

		return ret;
}

static int stream_parm_set(struct file *file, void *fh,
			   struct v4l2_streamparm *parm)
{
	int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	ret = v4l2_misc_process_command(strm->id, VIDIOC_S_PARM,
					sizeof(*parm), parm);

	return ret;
}

static int request_bufs(struct file *file, void *fh,
			struct v4l2_requestbuffers *req)
{
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	return vb2_reqbufs(&strm->vb2_q, req);
}

static int queue_buf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);
	unsigned int idx = b->index;
	int fd = b->m.userptr;

	if (!strm->bdata || idx >= strm->bcount) {
		pr_err("queue buffer in invalid state: bdata %p, idx %d",
			strm->bdata, idx);
		return -EINVAL;
	}

	/* Make sure existing mapped fd is not overwritten with new one.
	   REQ_BUF needs to be called when queuing a new set */
	if (strm->bdata && idx < strm->bcount &&
	    strm->bdata[idx].orig_fd != -1) {
		if (strm->bdata[idx].orig_fd != fd)
			return -EAGAIN;
	}

	return vb2_qbuf(&strm->vb2_q, b);
}

static int dequeue_buf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	return vb2_dqbuf(&strm->vb2_q, b, file->f_flags & O_NONBLOCK);
}

static int streamon(struct file *file, void *fh, enum v4l2_buf_type buf_type)
{
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	return vb2_streamon(&strm->vb2_q, buf_type);
}

static int streamoff(struct file *file, void *fh, enum v4l2_buf_type buf_type)
{
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	return vb2_streamoff(&strm->vb2_q, buf_type);
}

static __u32 v4l2_hal_mmap_cid_index(__u32 id) {
	return id - V4L2_CID_PRIVATE_BASE;
}

static bool validate_mapping(struct v4l2_stream_data *strm,
			     struct v4l2_control *ctrl) {
	__u32 idx;

	idx = v4l2_hal_mmap_cid_index(ctrl->id);

	if (strm->cid_map[idx].orig_fd != -1 &&
	    strm->cid_map[idx].orig_fd != ctrl->value) {
		return false;
	}

	return true;
}

static int get_ctrl(struct file *file, void *fh,
			struct v4l2_control *ctrl)
{
	int ret;
	__u32 idx;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	if (v4l2_hal_is_set_mapping_cid(ctrl->id))
		/* No "get" for setup mapping request */
		return -EINVAL;

	ret = v4l2_misc_process_command(strm->id, VIDIOC_G_CTRL,
					sizeof(*ctrl), ctrl);
	if (ret == 0)
		ctrl->value = strm->cid_map[idx].orig_fd;

	return ret;
}

static int set_ctrl(struct file *file, void *fh,
			struct v4l2_control *ctrl)
{
	int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	if (v4l2_hal_is_set_mapping_cid(ctrl->id)) {
		if (!validate_mapping(strm, ctrl)) {
			pr_err("%s: cannot override fd mapping\n", __func__);
			return -EINVAL;
		}
	}

	ret = v4l2_misc_process_command(strm->id, VIDIOC_S_CTRL,
					sizeof(*ctrl), ctrl);

	return ret;
}

static int get_ext_ctrls(struct file *file, void *fh,
			 struct v4l2_ext_controls *ctrls) {
	int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	ret = v4l2_misc_process_command(strm->id, VIDIOC_G_EXT_CTRLS,
					sizeof(*ctrls), ctrls);

	return ret;
}

static int set_ext_ctrls(struct file *file, void *fh,
			 struct v4l2_ext_controls *ctrls) {
	int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(fh);

	ret = v4l2_misc_process_command(strm->id, VIDIOC_S_EXT_CTRLS,
					sizeof(*ctrls), ctrls);

	return ret;
}

static int v4l2_hal_queue_setup(struct vb2_queue *q,
				const void *parg,
				unsigned int *num_buffers,
				unsigned int *num_planes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct v4l2_stream_data *strm = q->drv_priv;
	struct v4l2_hal_reqbufs_data data;
	struct v4l2_buffer_data *bdata;
	int i;

	*num_planes = 1;
	sizes[0] = 0;

	data.count = *num_buffers;

	bdata = kmalloc_array(data.count, sizeof(*bdata), GFP_KERNEL);
	if (bdata == NULL)
		return -ENOMEM;

	for (i = 0; i < data.count; i++) {
		bdata[i].orig_fd = -1;
		bdata[i].mapped_fd = -1;
	}

	/* free existing mapping data if any */
	kfree(strm->bdata);

	strm->bcount = data.count;
	strm->bdata = bdata;

	return v4l2_misc_process_command(strm->id, VIOC_HAL_STREAM_REQBUFS,
					 sizeof(data), &data);
}

static void v4l2_hal_buf_queue(struct vb2_buffer *vb)
{
	struct v4l2_stream_data *strm = vb->vb2_queue->drv_priv;
	struct v4l2_hal_qbuf_data data;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	data.index = vb->index;
	data.fd = (int)vbuf->vb2_buf.planes[0].m.userptr;
	data.length = vbuf->vb2_buf.planes[0].length;

	v4l2_misc_process_command(strm->id, VIOC_HAL_STREAM_QBUF,
				  sizeof(data), &data);
}

static int v4l2_hal_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct v4l2_stream_data *strm = q->drv_priv;

	return v4l2_misc_process_command(strm->id,
					 VIOC_HAL_STREAM_ON, 0, NULL);
}

static void v4l2_hal_stop_streaming(struct vb2_queue *q)
{
	struct v4l2_stream_data *strm = q->drv_priv;

	v4l2_misc_process_command(strm->id, VIOC_HAL_STREAM_OFF, 0, NULL);
}

static void *v4l2_hal_get_userptr(void *alloc_ctx,
				  unsigned long vaddr,
				  unsigned long size, enum dma_data_direction dma_dir)
{
	return (void *)vaddr;
}

static void v4l2_hal_put_userptr(void *buf_priv)
{
	/* Intentionally left empty. Required for user ptr mode mem operation */
}
static struct vb2_ops v4l2_hal_vb2_ops = {
	.queue_setup = v4l2_hal_queue_setup,
	.start_streaming = v4l2_hal_start_streaming,
	.stop_streaming = v4l2_hal_stop_streaming,
	.buf_queue = v4l2_hal_buf_queue,
};

static struct vb2_mem_ops v4l2_hal_vb2_mem_ops = {
	.get_userptr = v4l2_hal_get_userptr,
	.put_userptr = v4l2_hal_put_userptr,
};

static int v4l2_hal_vb2_q_init(struct v4l2_stream_data *strm,
				   struct vb2_queue *q)
{
	memset(q, 0, sizeof(struct vb2_queue));
	q->drv_priv = strm;
	q->mem_ops = &v4l2_hal_vb2_mem_ops;
	q->ops = &v4l2_hal_vb2_ops;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_USERPTR;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->buf_struct_size = sizeof(struct vb2_buffer);

	return vb2_queue_init(q);
}

int v4l2_hal_get_mapped_fd(void *hal_data, unsigned int stream, int index)
{
	struct v4l2_hal_data *data = hal_data;
	struct v4l2_stream_data *strm;

	strm = &data->strms[stream];
	return strm->bdata[index].mapped_fd;
}

void v4l2_hal_set_mapped_fd(void *hal_data, unsigned int stream, int index,
			    int orig_fd, int mapped_fd)
{
	struct v4l2_hal_data *data = hal_data;
	struct v4l2_stream_data *strm;

	strm = &data->strms[stream];
	strm->bdata[index].orig_fd = orig_fd;
	strm->bdata[index].mapped_fd = mapped_fd;
}

int v4l2_hal_get_mapped_fd_for_cid(void *hal_data, unsigned int stream, __u32 cid)
{
	struct v4l2_hal_data *data = hal_data;
	struct v4l2_stream_data *strm;
	__u32 idx = v4l2_hal_mmap_cid_index(cid);

	strm = &data->strms[stream];
	return strm->cid_map[idx].mapped_fd;
}

void v4l2_hal_set_mapped_fd_for_cid(void *hal_data, unsigned int stream, __u32 cid,
				    int orig_fd, int mapped_fd)
{
	struct v4l2_hal_data *data = hal_data;
	struct v4l2_stream_data *strm;
	__u32 idx = v4l2_hal_mmap_cid_index(cid);

	strm = &data->strms[stream];
	strm->cid_map[idx].orig_fd = orig_fd;
	strm->cid_map[idx].mapped_fd = mapped_fd;
}

int v4l2_hal_stream_set_handled(void *hal_data, unsigned int stream)
{
	int rc = 0;
	struct v4l2_hal_data *data = hal_data;

	if (stream >= V4L2_HAL_MAX_STREAMS) {
		pr_err("%s: invalid stream %u\n", __func__, stream);
		return -EINVAL;
	}

	mutex_lock(&data->lock);
	if (!data->strms[stream].used) {
		pr_err("%s: stream %u not start\n", __func__, stream);
		rc = -EINVAL;
	}

	if(data->strms[stream].handled) {
		pr_err("%s: stream %u already handled\n", __func__, stream);
		rc = -EINVAL;
	}

	data->strms[stream].handled = true;
	mutex_unlock(&data->lock);
	return rc;
}

int v4l2_hal_buffer_ready(void *hal_data, unsigned int stream, int index,
			unsigned int length, unsigned int seq,
			unsigned int ts_sec, unsigned int ts_usec,
			enum misc_buffer_state state)
{
	struct v4l2_hal_data *data = hal_data;
	struct v4l2_stream_data *strm;
	struct vb2_buffer *vb;
	enum vb2_buffer_state buffer_state;
	if (stream >= V4L2_HAL_MAX_STREAMS)
		return -EINVAL;

	switch(state) {
	case MISC_BUFFER_STATE_DONE:
		buffer_state = VB2_BUF_STATE_DONE;
		break;
	case MISC_BUFFER_STATE_ERROR:
		buffer_state = VB2_BUF_STATE_ERROR;
		break;
	case MISC_BUFFER_STATE_QUEUED:
		buffer_state = VB2_BUF_STATE_QUEUED;
		break;
	default:
		return -EINVAL;
	}

	strm = &data->strms[stream];
	mutex_lock(&data->lock);
	if (strm->used) {
		vb = strm->vb2_q.bufs[index];
		if (vb != NULL) {
			struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
			vbuf->timestamp.tv_sec = ts_sec;
			vbuf->timestamp.tv_usec = ts_usec;
			vbuf->sequence = seq;
			vbuf->vb2_buf.planes[0].bytesused = length;
			vb2_buffer_done(vb, buffer_state);
		} else
			pr_warn("%s: return buffer in error status\n", __func__);
	}
	mutex_unlock(&data->lock);

	return 0;
}

static int v4l2_hal_open(struct file *file)
{
	struct v4l2_hal_data *data = video_drvdata(file);
	unsigned long idx;
	int ret;

	mutex_lock(&hal_state_lock);
	if (hal_state != HAL_READY) { /* it's in freeing right now */
		mutex_unlock(&hal_state_lock);
		pr_err("hal_state is NOT HAL_READY. current hal_state: %d",
		       hal_state);
		return -ENODEV;
	}
	hal_data_get(data);
	mutex_unlock(&hal_state_lock);

	/* This driver is counting on memory copy of structures btween
	   tasks. Those task needs to be in same compat mode. */
	if (v4l2_misc_compat_mode() != is_compat_task()) {
		pr_err("compat mode mismatch. v4l2_hal: %d, misc: %d\n",
			   is_compat_task(), v4l2_misc_compat_mode());
		hal_data_put(data);
		return -EFAULT;
	}

	mutex_lock(&data->lock);
	for (idx = 0; idx < V4L2_HAL_MAX_STREAMS; idx++) {
		if (!data->strms[idx].used) {
			data->strms[idx].used = true;
			break;
		}
	}
	mutex_unlock(&data->lock);

	if (idx == V4L2_HAL_MAX_STREAMS) {
		pr_err("%s: No more stream can be opened\n", __func__);
		hal_data_put(data);
		return -EFAULT;
	}

	ret = v4l2_misc_process_command(idx, VIOC_HAL_STREAM_OPENED, 0, NULL);
	if (ret) {
		pr_err("%s: Stream open failed\n", __func__);
		goto release_stream;
	}

	ret = v4l2_hal_vb2_q_init(&data->strms[idx], &data->strms[idx].vb2_q);
	if (ret) {
		pr_err("%s: Failed to initialize buffer queue\n", __func__);
		goto close_misc;
	}

	v4l2_fh_init(&data->strms[idx].fh, data->vdev);
	v4l2_fh_add(&data->strms[idx].fh);
	file->private_data = &data->strms[idx].fh;

	return 0;

close_misc:
	v4l2_misc_process_command(idx, VIOC_HAL_STREAM_CLOSED, 0, NULL);

release_stream:
	mutex_lock(&data->lock);
	data->strms[idx].used = false;
	data->strms[idx].handled = false;
	mutex_unlock(&data->lock);
	hal_data_put(data);

	pr_err("%s ret: %d", __func__, ret);
	return ret;
}

static int v4l2_hal_close(struct file *file)
{
	int i;
	struct v4l2_hal_data *data = video_drvdata(file);
	struct v4l2_stream_data *strm = FH_TO_STREAM(file->private_data);
	unsigned int id = strm->id;

	vb2_queue_release(&strm->vb2_q);
	v4l2_misc_process_command(strm->id, VIOC_HAL_STREAM_CLOSED, 0, NULL);

	mutex_lock(&data->lock);
	kfree(strm->bdata);
	v4l2_fh_del(&strm->fh);
	v4l2_fh_exit(&strm->fh);
	memset(strm, 0, sizeof(*strm));
	strm->id = id;
	for (i = 0; i < V4L2_HAL_MAX_NUM_MMAP_CID; i++) {
		strm->cid_map[i].orig_fd = -1;
		strm->cid_map[i].mapped_fd = -1;
	}
	mutex_unlock(&data->lock);

	file->private_data = NULL;
	hal_data_put(data);
	return 0;
}

static unsigned int v4l2_hal_poll(struct file *file, poll_table *wait)
{
	unsigned int ret;
	struct v4l2_stream_data *strm = FH_TO_STREAM(file->private_data);
	ret = vb2_poll(&strm->vb2_q, file, wait);
	return ret;
}

static int subscribe_event(struct v4l2_fh *fh,
	const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_HAL_ERROR_EVENT_TYPE:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	default:
		return -EINVAL;
	}
}
/* Callout ioctls not passing through */
static const struct v4l2_ioctl_ops v4l2_hal_ioctl_ops = {
	.vidioc_querycap				= query_cap,
	.vidioc_enum_input				= input_enum,
	.vidioc_g_input					= input_get,
	.vidioc_s_input					= input_set,
	.vidioc_enum_fmt_vid_cap		= fmt_enum,
	.vidioc_g_fmt_vid_cap			= fmt_get,
	.vidioc_s_fmt_vid_cap			= fmt_set,
	.vidioc_enum_framesizes			= frmsize_enum,
	.vidioc_enum_frameintervals		= frmival_enum,
	.vidioc_g_parm					= stream_parm_get,
	.vidioc_s_parm					= stream_parm_set,

	.vidioc_reqbufs			= request_bufs,
	.vidioc_qbuf			= queue_buf,
	.vidioc_dqbuf			= dequeue_buf,

	.vidioc_streamon		= streamon,
	.vidioc_streamoff		= streamoff,

	.vidioc_g_ctrl			= get_ctrl,
	.vidioc_s_ctrl			= set_ctrl,

	.vidioc_g_ext_ctrls		= get_ext_ctrls,
	.vidioc_s_ext_ctrls		= set_ext_ctrls,

	.vidioc_subscribe_event		= subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

static struct v4l2_file_operations v4l2_hal_fops = {
	.owner	 = THIS_MODULE,
	.poll    = v4l2_hal_poll,
	.open	 = v4l2_hal_open,
	.unlocked_ioctl	 = video_ioctl2,
	.release = v4l2_hal_close,
};

void *v4l2_hal_init()
{
	int retval;
	int i, j;
	struct v4l2_hal_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;

	for (i = 0; i < V4L2_HAL_MAX_STREAMS; i++) {
		data->strms[i].id = i;
		for (j = 0; j < V4L2_HAL_MAX_NUM_MMAP_CID; j++) {
			data->strms[i].cid_map[j].orig_fd = -1;
			data->strms[i].cid_map[j].mapped_fd = -1;
		}
	}

	mutex_init(&data->lock);

	snprintf(data->v4l2_dev.name, sizeof(data->v4l2_dev.name),
		 "%s", HAL_DEV_NAME);

	retval = v4l2_device_register(NULL, &data->v4l2_dev);
	if (retval) {
		pr_err("failed to register v4l2 hal\n");
		goto error_reg_v4l2dev;
	}

	data->vdev = video_device_alloc();
	if (data->vdev == NULL) {
		pr_err("failed to allocate video_device hal\n");
		goto error_alloc_vdev;
	}

	strlcpy(data->vdev->name, HAL_DEV_NAME, sizeof(data->vdev->name));
	data->vdev->ctrl_handler = NULL;
	data->vdev->v4l2_dev = &data->v4l2_dev;
	data->vdev->release = video_device_release;
	data->vdev->fops = &v4l2_hal_fops;
	data->vdev->ioctl_ops = &v4l2_hal_ioctl_ops;
	data->vdev->vfl_type = VFL_TYPE_GRABBER;

	retval = video_register_device(data->vdev,
					   VFL_TYPE_GRABBER, -1);
	if (retval) {
		pr_err("%s: failed to register video device. rc %d\n",
			   __func__, retval);
		goto error_reg_vdev;
	}

	video_set_drvdata(data->vdev, data);

	hal_state = HAL_READY;
	kref_init(&data->kref);
	return data;

error_reg_vdev:
	video_device_release(data->vdev);
error_alloc_vdev:
	v4l2_device_unregister(&data->v4l2_dev);
error_reg_v4l2dev:
	kfree(data);

	return NULL;
}

void v4l2_hal_exit(void *hal_data)
{
	struct v4l2_hal_data *data = hal_data;

	if (data != NULL) {
		mutex_lock(&hal_state_lock);
		hal_state = HAL_DESTROYED;
		hal_data_put(data);
		mutex_unlock(&hal_state_lock);
	}
}

bool v4l2_hal_check_dev_ready(void)
{
	return hal_state == HAL_READY;
}

int v4l2_hal_report_error(void *hal_data, unsigned int code)
{
	struct v4l2_event ev;
	struct v4l2_hal_data *data = hal_data;
	struct misc_report_mod_error *err;

	if (data == NULL)
		return -ENODEV;

	memset(&ev, 0, sizeof(ev));
	ev.type = V4L2_HAL_ERROR_EVENT_TYPE;
	err = (struct misc_report_mod_error *) ev.u.data;
	err->code = code;
	v4l2_event_queue(data->vdev, &ev);

	return 0;
}
