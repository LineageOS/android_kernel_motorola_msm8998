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

#include "v4l2_hal.h"

struct v4l2_misc_command {
	atomic_t pending_read;
	atomic_t pending_resp;
	atomic_t result_code;
	unsigned int stream;
	unsigned int cmd;
	size_t size;
	void *data;
	struct file *ionfile;
};

struct v4l2_misc_data {
	struct miscdevice misc_dev;
	bool in_use;
	void *v4l2_hal_data;
	wait_queue_head_t wait;
	struct v4l2_misc_command command;
	struct mutex lock;
	struct completion comp;
	bool compat;
};

static struct v4l2_misc_data *g_data;

#define V4L2_MISC_IOCTL_TIMEOUT	2000 /* ms */

static int misc_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	mutex_lock(&g_data->lock);

	if (g_data->in_use) {
		pr_err("%s: device already in use\n", __func__);
		ret = -EACCES;
	} else
		g_data->in_use = true;

	mutex_unlock(&g_data->lock);

	if (is_compat_task())
		g_data->compat = true;
	else
		g_data->compat = false;

	return ret;
}

static ssize_t misc_dev_read(struct file *filp, char __user *ubuf,
				 size_t count, loff_t *f_pos)
{
	int ret;
	int copy_size;
	struct misc_read_cmd *misc_cmd;
	struct v4l2_hal_qbuf_data *qb;
	int tgt_fd;

	if (!atomic_read(&g_data->command.pending_read)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(
			g_data->wait,
			atomic_read(&g_data->command.pending_read));

		if (ret) {
			if (ret == -ERESTARTSYS)
				return -EINTR;
			else
				return -EIO;
		}
	}

	atomic_set(&g_data->command.pending_read, 0);
	atomic_set(&g_data->command.pending_resp, 1);

	/* make sure there is enough space to copy */
	copy_size = sizeof(struct misc_read_cmd) + g_data->command.size;
	if (copy_size > count) {
		pr_err("%s: No enough memory to copy data.\n", __func__);
		ret = -ENOMEM;
		goto errout;
	}

	misc_cmd = (struct misc_read_cmd *)ubuf;
	misc_cmd->stream = g_data->command.stream;
	misc_cmd->cmd = g_data->command.cmd;
	if (g_data->command.data == NULL)
		return copy_size;

	if (g_data->command.cmd == VIOC_HAL_STREAM_QBUF &&
		g_data->command.ionfile) {
		qb = g_data->command.data;
		tgt_fd = v4l2_hal_get_mapped_fd(g_data->v4l2_hal_data,
						g_data->command.stream,
						qb->index);
		if (tgt_fd < 0) {
			tgt_fd = get_unused_fd_flags(O_CLOEXEC);

			if (tgt_fd < 0)
				goto errout;

			fd_install(tgt_fd, g_data->command.ionfile);
			v4l2_hal_set_mapped_fd(g_data->v4l2_hal_data,
						   g_data->command.stream,
						   qb->index, tgt_fd);
		}

		qb->fd = tgt_fd;
	}

	if (copy_to_user((void __user *)misc_cmd->data,
			 g_data->command.data, g_data->command.size)) {
		ret = -EFAULT;
		goto errout;
	}

	return copy_size;

errout:
	/* unblock video device side */
	complete(&g_data->comp);

	return ret;
}

static int misc_dev_release(struct inode *inodep, struct file *filep)
{
	if (g_data->v4l2_hal_data)
		v4l2_hal_exit(g_data->v4l2_hal_data);
	g_data->v4l2_hal_data = NULL;

	mutex_lock(&g_data->lock);
	g_data->in_use = false;
	mutex_unlock(&g_data->lock);

	return 0;
}

static unsigned int misc_dev_poll(struct file *filp,
				   struct poll_table_struct *tbl)
{
	unsigned int mask = 0;

	poll_wait(filp, &g_data->wait, tbl);

	if (atomic_read(&g_data->command.pending_read))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static int misc_copy_ioctl(unsigned int cmd, void __user *data)
{
	int size = _IOC_SIZE(cmd);

	if (_IOC_DIR(cmd) == _IOC_NONE)
		return 0;

	if (size != g_data->command.size) {
		pr_err("%s: data size mis-match\n", __func__);
		return -EINVAL;
	}

	if (copy_from_user(g_data->command.data, data, _IOC_SIZE(cmd)))
		return -EFAULT;

	return 0;
}

static int misc_process_v4l2_ioctl(void *arg)
{
	int ret;
	struct misc_ioctl_resp ioctl_resp;
	void __user *data_ptr;

	if (copy_from_user(&ioctl_resp, arg, sizeof(ioctl_resp)))
		return -EFAULT;

	if (g_data->compat)
		data_ptr = compat_ptr((compat_uptr_t)ioctl_resp.data);
	else
		data_ptr = (void *)ioctl_resp.data;

	if (!atomic_read(&g_data->command.pending_resp)) {
		pr_err("%s: No ioctl in progress\n", __func__);
		return -EINVAL;
	}

	if (g_data->command.stream != ioctl_resp.stream ||
		g_data->command.cmd != ioctl_resp.cmd) {
		pr_err("%s: Invalid ioctl response\n", __func__);
		return -EINVAL;
	}

	if (ioctl_resp.result_code == 0) {
		ret = misc_copy_ioctl(ioctl_resp.cmd, data_ptr);
		if (ret == 0)
			atomic_set(&g_data->command.result_code, 0);
		else
			atomic_set(&g_data->command.result_code, -EFAULT);
	} else {
		atomic_set(&g_data->command.result_code,
			   ioctl_resp.result_code);
	}

	complete(&g_data->comp);

	return 0;
}

static int misc_process_stream_command(unsigned int cmd, void *arg)
{
	struct misc_stream_resp stream_resp;

	if (copy_from_user(&stream_resp, (void *)arg, sizeof(stream_resp)))
		return -EFAULT;

	if (!atomic_read(&g_data->command.pending_resp)) {
		pr_err("%s: No ioctl in progress\n", __func__);
		return -EINVAL;
	}

	if (g_data->command.stream != stream_resp.stream ||
		g_data->command.cmd != cmd) {
		pr_err("%s: Invalid ioctl response\n", __func__);
		return -EINVAL;
	}

	atomic_set(&g_data->command.result_code, 0);
	complete(&g_data->comp);

	return 0;
}

static int misc_process_dequeue_request(void *arg)
{
	struct misc_dequeue_cmd dq_cmd;

	if (copy_from_user(&dq_cmd, (void *)arg, sizeof(dq_cmd)))
		return -EFAULT;

	return v4l2_hal_buffer_ready(g_data->v4l2_hal_data,
				     dq_cmd.stream,
				     dq_cmd.index,
				     dq_cmd.length);
}

static long misc_dev_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	long ret = 0;

	switch (cmd) {
	case VIOC_HAL_IFACE_START: {
		/* enumerate v4l2 video device */
		void *data = v4l2_hal_init();

		if (data == NULL)
			ret = -EFAULT;

		g_data->v4l2_hal_data = data;
		break;
	}
	case VIOC_HAL_IFACE_STOP:
		v4l2_hal_exit(g_data->v4l2_hal_data);
		g_data->v4l2_hal_data = NULL;
		break;
	case VIOC_HAL_STREAM_OPENED:
	case VIOC_HAL_STREAM_CLOSED:
	case VIOC_HAL_STREAM_ON:
	case VIOC_HAL_STREAM_OFF:
	case VIOC_HAL_STREAM_REQBUFS:
	case VIOC_HAL_STREAM_QBUF:
		ret = misc_process_stream_command(cmd, (void *)arg);
		break;
	case VIOC_HAL_STREAM_DQBUF:
		ret = misc_process_dequeue_request((void *)arg);
		break;
	case VIOC_HAL_V4L2_CMD:
		ret = misc_process_v4l2_ioctl((void *)arg);
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
	int ret;
	struct v4l2_hal_qbuf_data *qb;

	/* only allow one command queued at a time */
	mutex_lock(&g_data->lock);

	g_data->command.stream = stream;
	g_data->command.cmd = cmd;
	g_data->command.size = size;
	g_data->command.data = data;

	if (cmd == VIOC_HAL_STREAM_QBUF) {
		qb = data;
		g_data->command.ionfile = fget(qb->fd);
	} else
		g_data->command.ionfile = NULL;

	atomic_set(&g_data->command.pending_read, 1);

	wake_up(&g_data->wait);

	atomic_set(&g_data->command.result_code, -ETIME);
	/* wait for ioctl command response */
	if (!wait_for_completion_timeout(
			&g_data->comp,
			msecs_to_jiffies(V4L2_MISC_IOCTL_TIMEOUT))) {
		pr_err("%s: timeout for ioctl\n", __func__);
	}

	atomic_set(&g_data->command.pending_read, 0);
	atomic_set(&g_data->command.pending_resp, 0);

	ret = atomic_read(&g_data->command.result_code);

	mutex_unlock(&g_data->lock);

	return ret;
}

static int v4l2_hal_probe(struct platform_device *pdev)
{
	int ret;
	struct v4l2_misc_data *data;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	init_waitqueue_head(&data->wait);
	init_completion(&data->comp);
	mutex_init(&data->lock);

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
	complete(&g_data->comp);

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
