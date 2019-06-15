/*
 * Copyright (C) 2016 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */
#include <linux/compat.h>
#include <linux/file.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <video/v4l2_camera_ext_defs.h>

#include "v4l2_hal.h"
#include "v4l2_hal_internal.h"

struct v4l2_misc_command {
	atomic_t pending_read;
	atomic_t pending_resp;
	atomic_t result_code;
	unsigned int stream;
	unsigned int cmd;
	size_t size;
	void *data;
	int orig_fd;
	struct file *ionfile;
	void *priv;
	wait_queue_head_t wait;
	struct completion comp;
	struct mutex lock;
};

struct v4l2_misc_data {
	struct miscdevice misc_dev;
	void *v4l2_hal_data;
	/* the last one is for monitor */
	struct v4l2_misc_command command[V4L2_HAL_MAX_STREAMS + 1];
	bool compat;
	unsigned int users; /* active users */
	struct mutex users_lock;
};

static struct v4l2_misc_data *g_data;

/* change to a smaller value after fix the timeout issue stopping axi stream */
#define V4L2_MISC_IOCTL_TIMEOUT	8000 /* ms */

static inline void set_cmd_queue_to_file(struct file *filp,
		unsigned int idx)
{
	if (idx <= V4L2_HAL_MAX_STREAMS)
		filp->private_data = &g_data->command[idx];
}

static inline struct v4l2_misc_command *get_cmd_queue_from_file(
	struct file *filp)
{
	return filp->private_data;
}

static int misc_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	if (is_compat_task())
		g_data->compat = true;
	else
		g_data->compat = false;

	/* a client is monitor as default until call VIOC_SET_HANDLER */
	mutex_lock(&g_data->users_lock);
	++g_data->users;
	if (g_data->users == 1)
		set_cmd_queue_to_file(filp, V4L2_HAL_MAX_STREAMS);
	mutex_unlock(&g_data->users_lock);
	return ret;
}

static ssize_t misc_dev_read(struct file *filp, char __user *ubuf,
				 size_t count, loff_t *f_pos)
{
	int ret;
	int copy_size;
	struct misc_read_cmd *misc_cmd;
	struct v4l2_hal_qbuf_data *qb;
	struct v4l2_control *ctrl;
	int tgt_fd;
	struct v4l2_misc_command *target_cmd;

	target_cmd = get_cmd_queue_from_file(filp);
	if (!target_cmd) {
		pr_err("%s: unrecoganized client app\n", __func__);
		return -EFAULT;
	}

	if (!atomic_read(&target_cmd->pending_read)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(
			target_cmd->wait,
			atomic_read(&target_cmd->pending_read));

		if (ret) {
			if (ret == -ERESTARTSYS)
				return -EINTR;
			else
				return -EIO;
		}
	}

	atomic_set(&target_cmd->pending_read, 0);
	atomic_set(&target_cmd->pending_resp, 1);

	/* make sure there is enough space to copy */
	if (target_cmd->cmd == VIDIOC_G_EXT_CTRLS ||
	    target_cmd->cmd == VIDIOC_S_EXT_CTRLS) {
		struct v4l2_ext_controls *ctrls = target_cmd->data;
		if (g_data->compat)
			copy_size = sizeof(struct misc_read_cmd) +
				v4l2_hal_get_required_size32(ctrls);
		else
			copy_size = sizeof(struct misc_read_cmd) +
				v4l2_hal_get_required_size(ctrls);
	} else
		copy_size = sizeof(struct misc_read_cmd) + target_cmd->size;

	if (copy_size > count) {
		pr_err("%s: No enough memory to copy data.\n", __func__);
		ret = -ENOMEM;
		goto errout;
	}

	misc_cmd = (struct misc_read_cmd *)ubuf;
	if (copy_to_user((void __user *)&misc_cmd->stream, &target_cmd->stream,
			 sizeof(target_cmd->stream))) {
		pr_err("%s: failed to copy stream id to user\n", __func__);
		goto errout;
	}

	if (copy_to_user((void __user *)&misc_cmd->cmd, &target_cmd->cmd,
			 sizeof(target_cmd->cmd))) {
		pr_err("%s: failed to copy cmd to user\n", __func__);
		goto errout;
	}

	if (target_cmd->data == NULL)
		return copy_size;

	if (target_cmd->cmd == VIOC_HAL_STREAM_QBUF &&
		target_cmd->ionfile) {
		qb = target_cmd->data;
		tgt_fd = v4l2_hal_get_mapped_fd(g_data->v4l2_hal_data,
						target_cmd->stream,
						qb->index);
		if (tgt_fd < 0) {
			tgt_fd = get_unused_fd_flags(O_CLOEXEC);

			if (tgt_fd < 0)
				goto errout;

			get_file(target_cmd->ionfile);
			fd_install(tgt_fd, target_cmd->ionfile);
			v4l2_hal_set_mapped_fd(g_data->v4l2_hal_data,
					       target_cmd->stream,
					       qb->index,
					       target_cmd->orig_fd, tgt_fd);
		}

		qb->fd = tgt_fd;
	} else if (target_cmd->cmd == VIDIOC_S_CTRL && target_cmd->ionfile) {
		ctrl = target_cmd->data;
		tgt_fd = v4l2_hal_get_mapped_fd_for_cid(g_data->v4l2_hal_data,
							target_cmd->stream,
							ctrl->id);
		if (tgt_fd < 0) {
			tgt_fd = get_unused_fd_flags(O_CLOEXEC);

			if (tgt_fd < 0)
				goto errout;

			get_file(target_cmd->ionfile);
			fd_install(tgt_fd, target_cmd->ionfile);
			v4l2_hal_set_mapped_fd_for_cid(g_data->v4l2_hal_data,
						       target_cmd->stream,
						       ctrl->id,
						       target_cmd->orig_fd,
						       tgt_fd);
		}
		ctrl->value = tgt_fd;
	}

	if (target_cmd->cmd == VIDIOC_G_EXT_CTRLS ||
	    target_cmd->cmd == VIDIOC_S_EXT_CTRLS) {
		struct v4l2_ext_controls *ctrls = target_cmd->data;
		if (g_data->compat) {
			unsigned int cmd;
			ret = v4l2_hal_put_ext_controls32(ctrls,
							  misc_cmd->data,
							  target_cmd->priv);

			if (target_cmd->cmd == VIDIOC_G_EXT_CTRLS)
				cmd = VIDIOC_G_EXT_CTRLS32;
			else
				cmd = VIDIOC_S_EXT_CTRLS32;

			if (copy_to_user((void __user *)&misc_cmd->cmd, &cmd,
					 sizeof(cmd))) {
				pr_err("%s: failed to copy cmd to user\n", __func__);
				goto errout;
			}
		} else
			ret =v4l2_hal_put_ext_controls(ctrls,
						       misc_cmd->data,
						       target_cmd->priv);

		if (ret)
			goto errout;
	} else {
		if (copy_to_user((void __user *)misc_cmd->data,
				 target_cmd->data, target_cmd->size)) {
			ret = -EFAULT;
			goto errout;
		}
	}

	return copy_size;

errout:
	/* unblock video device side */
	complete(&target_cmd->comp);

	return ret;
}

static int misc_dev_release(struct inode *inodep, struct file *filp)
{
	int i;

	mutex_lock(&g_data->users_lock);
	--g_data->users;
	if (g_data->users == 0) {
		if (g_data->v4l2_hal_data) {
			for (i = 0; i < V4L2_HAL_MAX_STREAMS + 1; i++) {
				if (!mutex_trylock(&g_data->command[i].lock)) {
					complete(&g_data->command[i].comp);
					mutex_lock(&g_data->command[i].lock);
				}
			}
			/* if someone is listening v4l2 event,
			   safe to report an error */
			v4l2_hal_report_error(g_data->v4l2_hal_data,
				CAMERA_EXT_ERROR_FATAL);

			v4l2_hal_exit(g_data->v4l2_hal_data);
			g_data->v4l2_hal_data = NULL;

			for (i = 0; i < V4L2_HAL_MAX_STREAMS + 1; i++)
				mutex_unlock(&g_data->command[i].lock);
		}
	}
	mutex_unlock(&g_data->users_lock);

	return 0;
}

static unsigned int misc_dev_poll(struct file *filp,
				  struct poll_table_struct *tbl)
{
	unsigned int mask = 0;
	struct v4l2_misc_command *target_cmd;

	target_cmd = get_cmd_queue_from_file(filp);
	if (!target_cmd)
		return 0;

	poll_wait(filp, &target_cmd->wait, tbl);

	if (atomic_read(&target_cmd->pending_read))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static int misc_copy_ioctl(struct v4l2_misc_command *target_cmd,
		unsigned int cmd, void __user *data)
{
	int size = _IOC_SIZE(cmd);

	if (_IOC_DIR(cmd) == _IOC_NONE)
		return 0;

	if (size != target_cmd->size) {
		pr_err("%s: data size mis-match\n", __func__);
		return -EINVAL;
	}

	if (cmd == VIDIOC_G_EXT_CTRLS ||
	    cmd == VIDIOC_S_EXT_CTRLS) {
		int ret;
		struct v4l2_ext_controls *ctrls = target_cmd->data;

		if (g_data->compat)
			ret = v4l2_hal_get_ext_controls32(ctrls, data,
							  target_cmd->priv);
		else
			ret = v4l2_hal_get_ext_controls(ctrls, data,
							target_cmd->priv);

		if (ret)
			return ret;
	} else {
		if (copy_from_user(target_cmd->data, data, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	return 0;
}

static int misc_process_v4l2_ioctl(struct v4l2_misc_command *target_cmd, void *arg)
{
	int ret;
	struct misc_ioctl_resp ioctl_resp;
	void __user *data_ptr;

	if (copy_from_user(&ioctl_resp, arg, sizeof(ioctl_resp)))
		return -EFAULT;

#ifdef CONFIG_COMPAT
	if (g_data->compat) {
		data_ptr = compat_ptr((compat_uptr_t)ioctl_resp.data);
		if (ioctl_resp.cmd == VIDIOC_G_EXT_CTRLS32)
			ioctl_resp.cmd = VIDIOC_G_EXT_CTRLS;
		else if (ioctl_resp.cmd == VIDIOC_S_EXT_CTRLS32)
			ioctl_resp.cmd = VIDIOC_S_EXT_CTRLS;
	} else
#endif
		data_ptr = (void *)(uintptr_t)ioctl_resp.data;

	if (!atomic_read(&target_cmd->pending_resp)) {
		pr_err("%s: No ioctl in progress\n", __func__);
		return -EINVAL;
	}

	if (target_cmd->stream != ioctl_resp.stream ||
		target_cmd->cmd != ioctl_resp.cmd) {
		pr_err("%s: Invalid ioctl response\n", __func__);
		return -EINVAL;
	}

	if (ioctl_resp.result_code == 0) {
		ret = misc_copy_ioctl(target_cmd, ioctl_resp.cmd, data_ptr);
		if (ret == 0)
			atomic_set(&target_cmd->result_code, 0);
		else
			atomic_set(&target_cmd->result_code, -EFAULT);
	} else {
		atomic_set(&target_cmd->result_code,
			   ioctl_resp.result_code);
	}

	complete(&target_cmd->comp);

	return 0;
}

static int misc_process_stream_command(struct v4l2_misc_command *target_cmd,
		unsigned int cmd, void *arg)
{
	struct misc_stream_resp stream_resp;

	if (copy_from_user(&stream_resp, (void *)arg, sizeof(stream_resp)))
		return -EFAULT;

	if (!atomic_read(&target_cmd->pending_resp)) {
		pr_err("%s: No ioctl in progress\n", __func__);
		return -EINVAL;
	}

	if (target_cmd->stream != stream_resp.stream ||
		target_cmd->cmd != cmd) {
		pr_err("%s: Invalid ioctl response\n", __func__);
		return -EINVAL;
	}

	atomic_set(&target_cmd->result_code, stream_resp.result_code);
	complete(&target_cmd->comp);

	return 0;
}

static int misc_process_dequeue_request(void *arg)
{
	void *hal_data;
	struct misc_dequeue_cmd dq_cmd;

	if (copy_from_user(&dq_cmd, (void *)arg, sizeof(dq_cmd)))
		return -EFAULT;

	if (!v4l2_hal_check_dev_ready())
		return -ENODEV;

	hal_data = g_data->v4l2_hal_data;
	return v4l2_hal_buffer_ready(hal_data,
				     dq_cmd.stream,
				     dq_cmd.index,
				     dq_cmd.length,
				     dq_cmd.seq,
				     dq_cmd.ts_sec,
				     dq_cmd.ts_usec,
				     dq_cmd.state);
}

static int misc_process_report_mod_error(void *arg)
{
	void *hal_data;
	struct misc_report_mod_error mod_err;

	if (copy_from_user(&mod_err, arg, sizeof(mod_err)))
		return -EFAULT;

	if (!v4l2_hal_check_dev_ready())
		return -ENODEV;

	hal_data = g_data->v4l2_hal_data;
	return v4l2_hal_report_error(hal_data, mod_err.code);
}

static int misc_process_set_handler(struct file *filp, void *arg)
{
	struct v4l2_misc_command *monitor_cmd_queue;
	struct misc_set_handler set_handler_cmd;

	if (copy_from_user(&set_handler_cmd, (void *)arg,
		sizeof(set_handler_cmd)))
		return -EFAULT;

	if (set_handler_cmd.stream >= V4L2_HAL_MAX_STREAMS)
		return -EFAULT;

	monitor_cmd_queue = &g_data->command[V4L2_HAL_MAX_STREAMS];
	if (!atomic_read(&monitor_cmd_queue->pending_resp)) {
		/*Set handler should be only called when handling OPEN STREAM*/
		pr_info("ignore invalid set handler or open stream timeout");
		return -EFAULT;
	}

	if (v4l2_hal_stream_set_handled(g_data->v4l2_hal_data,
				set_handler_cmd.stream) != 0)
		return -EFAULT;

	set_cmd_queue_to_file(filp, set_handler_cmd.stream);
	return 0;
}

static long misc_dev_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	long ret = 0;
	struct v4l2_misc_command *target_cmd_queue;

	target_cmd_queue = get_cmd_queue_from_file(filp);
	if (target_cmd_queue == NULL) {
		pr_err("%s: unrecoganized client app\n", __func__);
		return -EFAULT;
	}

	switch (cmd) {
	case VIOC_HAL_IFACE_START: {
		int i;
		void *data;

		for (i = 0; i < V4L2_HAL_MAX_STREAMS + 1; i++) {
			/* a req from last session could stuck there */
			if (!mutex_trylock(&g_data->command[i].lock)) {
				complete(&g_data->command[i].comp);
				mutex_lock(&g_data->command[i].lock);
			}
			init_completion(&g_data->command[i].comp);
		}

		data = v4l2_hal_init();
		if (data == NULL)
			ret = -EFAULT;

		g_data->v4l2_hal_data = data;

		for (i = 0; i < V4L2_HAL_MAX_STREAMS + 1; i++)
			mutex_unlock(&g_data->command[i].lock);
		break;
	}
	case VIOC_HAL_IFACE_STOP: {
		int i;

		if (g_data->v4l2_hal_data == NULL)
			break;

		for (i = 0; i < V4L2_HAL_MAX_STREAMS + 1; i++) {
			/* a req from current session could stuck there */
			if (!mutex_trylock(&g_data->command[i].lock)) {
				complete(&g_data->command[i].comp);
				mutex_lock(&g_data->command[i].lock);
			}
		}

		v4l2_hal_exit(g_data->v4l2_hal_data);
		g_data->v4l2_hal_data = NULL;

		for (i = 0; i < V4L2_HAL_MAX_STREAMS + 1; i++)
			mutex_unlock(&g_data->command[i].lock);
		break;
	}
	case VIOC_HAL_STREAM_OPENED:
	case VIOC_HAL_STREAM_CLOSED:
	case VIOC_HAL_STREAM_ON:
	case VIOC_HAL_STREAM_OFF:
	case VIOC_HAL_STREAM_REQBUFS:
	case VIOC_HAL_STREAM_QBUF:
		ret = misc_process_stream_command(target_cmd_queue, cmd,
				(void *)arg);
		break;
	case VIOC_HAL_STREAM_DQBUF:
		ret = misc_process_dequeue_request((void *)arg);
		break;
	case VIOC_HAL_V4L2_CMD:
		ret = misc_process_v4l2_ioctl(target_cmd_queue, (void *)arg);
		break;
	case VIOC_HAL_SET_STREAM_HANDLER:
		ret = misc_process_set_handler(filp, (void *)arg);
		break;
	case VIOC_HAL_REPORT_MOD_ERROR:
		ret = misc_process_report_mod_error((void *)arg);
		break;
	default:
		pr_err("%s: Unknown command %x\n", __func__, cmd);
		ret = -EINVAL;
	}

	return ret;
}

static const struct file_operations v4l2_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= misc_dev_open,
	.read		= misc_dev_read,
	.poll		= misc_dev_poll,
	.compat_ioctl	= misc_dev_ioctl,
	.unlocked_ioctl	= misc_dev_ioctl,
	.release	= misc_dev_release,
	.llseek		= noop_llseek,
};

bool v4l2_misc_compat_mode(void)
{
	return g_data->compat;
}

int v4l2_misc_process_command(unsigned int stream, unsigned int cmd,
			      size_t size, void *data)
{
	struct v4l2_misc_command *target_cmd_queue;
	int ret;
	struct v4l2_hal_qbuf_data *qb;
	struct v4l2_control *ctrl;
	void *priv = NULL;

	/* OPEN request will be sent to monitor.
	 * Other stream request will be sent to its handler.
	 */
	if (cmd == VIOC_HAL_STREAM_OPENED)
		target_cmd_queue = &g_data->command[V4L2_HAL_MAX_STREAMS];
	else
		target_cmd_queue = &g_data->command[stream];

	/* Need to save all ext control data existing in userspace */
	if (cmd == VIDIOC_G_EXT_CTRLS || cmd == VIDIOC_S_EXT_CTRLS) {
		if (v4l2_hal_ext_ctrl_save_private(data, &priv))
			return -EFAULT;
	}

	/* each stream only allow one command queued at a time */
	mutex_lock(&target_cmd_queue->lock);

	if (!v4l2_hal_check_dev_ready()) {
		mutex_unlock(&target_cmd_queue->lock);
		return -ENODEV;
	}

	target_cmd_queue->stream = stream;
	target_cmd_queue->cmd = cmd;
	target_cmd_queue->size = size;
	target_cmd_queue->data = data;
	target_cmd_queue->orig_fd = -1;
	target_cmd_queue->priv = priv;
	target_cmd_queue->ionfile = NULL;
	if (cmd == VIOC_HAL_STREAM_QBUF) {
		int tgt_fd;

		qb = data;
		tgt_fd = v4l2_hal_get_mapped_fd(g_data->v4l2_hal_data,
						target_cmd_queue->stream,
						qb->index);
		if (tgt_fd < 0) {
			target_cmd_queue->orig_fd = qb->fd;
			target_cmd_queue->ionfile = fget(qb->fd);
		}
	} else if (cmd == VIDIOC_S_CTRL) {
		int tgt_fd;

		ctrl = data;
		if (v4l2_hal_is_set_mapping_cid(ctrl->id)) {
		    tgt_fd = v4l2_hal_get_mapped_fd_for_cid(g_data->v4l2_hal_data,
							target_cmd_queue->stream,
							ctrl->id);
			if (tgt_fd < 0) {
			    target_cmd_queue->orig_fd = ctrl->value;
			    target_cmd_queue->ionfile = fget(ctrl->value);
			}
		}
	}

	atomic_set(&target_cmd_queue->result_code, -ETIME);
	atomic_set(&target_cmd_queue->pending_read, 1);
	wake_up(&target_cmd_queue->wait);

	/* wait for ioctl command response */
	if (!wait_for_completion_timeout(
			&target_cmd_queue->comp,
			msecs_to_jiffies(V4L2_MISC_IOCTL_TIMEOUT))) {
		pr_err("%s: timeout for ioctl\n", __func__);
	}

	atomic_set(&target_cmd_queue->pending_read, 0);
	atomic_set(&target_cmd_queue->pending_resp, 0);

	ret = atomic_read(&target_cmd_queue->result_code);

	if (target_cmd_queue->ionfile) {
		fput(target_cmd_queue->ionfile);
		target_cmd_queue->ionfile = NULL;
	}

	mutex_unlock(&target_cmd_queue->lock);

	if (cmd != VIDIOC_G_EXT_CTRLS && cmd != VIDIOC_S_EXT_CTRLS)
		return ret;

	if (!priv)
		return ret;

	if (cmd == VIDIOC_G_EXT_CTRLS)
		ret = v4l2_hal_ext_ctrl_restore_private(data, priv);

	kfree(priv);

	return ret;
}

static int v4l2_hal_probe(struct platform_device *pdev)
{
	int i;
	int ret;
	struct v4l2_misc_data *data;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	for (i = 0; i < V4L2_HAL_MAX_STREAMS + 1; i++) {
		init_waitqueue_head(&data->command[i].wait);
		mutex_init(&data->command[i].lock);
		init_completion(&data->command[i].comp);
	}
	mutex_init(&data->users_lock);

	data->misc_dev.minor = MISC_DYNAMIC_MINOR;
	data->misc_dev.name = "v4l2-hal-ctrl";
	data->misc_dev.fops = &v4l2_misc_fops;

	ret = misc_register(&data->misc_dev);
	if (ret) {
		pr_err("%s : misc_register failed.\n", __func__);
		return ret;
	}

	g_data = data;
	return 0;
}

static int v4l2_hal_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < V4L2_HAL_MAX_STREAMS + 1; i++)
		complete(&g_data->command[i].comp);

	v4l2_hal_exit(g_data->v4l2_hal_data);
	g_data->v4l2_hal_data = NULL;

	misc_deregister(&g_data->misc_dev);
	g_data = NULL;

	return 0;
}

static const struct of_device_id v4l2_hal_match[] = {
	{.compatible = "mmi,mods-v4l2_hal",},
	{},
};

static const struct platform_device_id v4l2_hal_id_table[] = {
	{"v4l2_hal", 0},
	{},
};

static struct platform_driver v4l2_hal_driver = {
	.driver = {
		.name = "v4l2_hal",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(v4l2_hal_match),
	},
	.probe = v4l2_hal_probe,
	.remove = v4l2_hal_remove,
	.id_table = v4l2_hal_id_table,
};

static int __init v4l2_hal_module_init(void)
{
	return platform_driver_register(&v4l2_hal_driver);
}

void __exit v4l2_hal_module_exit(void)
{
	platform_driver_unregister(&v4l2_hal_driver);
}

module_init(v4l2_hal_module_init);
module_exit(v4l2_hal_module_exit);

MODULE_LICENSE("GPL v2");
