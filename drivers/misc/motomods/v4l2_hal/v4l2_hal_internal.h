/*
 * copyright (C) 2016 Motorola Mobility LLC
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

#ifndef __V4L2_HAL_INTERNAL_H__
#define __V4L2_HAL_INTERNAL_H__

#include <linux/ioctl.h>
#include <linux/videodev2.h>
#include "v4l2_hal.h"

#ifdef CONFIG_COMPAT
struct v4l2_ext_controls32 {
	__u32 ctrl_class;
	__u32 count;
	__u32 error_idx;
	__u32 reserved[2];
	compat_caddr_t controls; /* actually struct v4l2_ext_control32 * */
};

#define VIDIOC_G_EXT_CTRLS32	_IOWR('V', 71, struct v4l2_ext_controls32)
#define VIDIOC_S_EXT_CTRLS32	_IOWR('V', 72, struct v4l2_ext_controls32)
#else
#define VIDIOC_G_EXT_CTRLS32 VIDIOC_G_EXT_CTRLS
#define VIDIOC_S_EXT_CTRLS32 VIDIOC_S_EXT_CTRLS
#endif

bool v4l2_misc_compat_mode(void);
int v4l2_misc_process_command(unsigned int stream, unsigned int cmd,
			      size_t size, void *data);
int v4l2_hal_buffer_ready(void *hal_data, unsigned int stream,
			  int fd, unsigned int length, unsigned int seq,
			  unsigned int ts_sec, unsigned int ts_usec,
			  enum misc_buffer_state state);
int v4l2_hal_get_mapped_fd(void *hal_data, unsigned int stream, int index);
void v4l2_hal_set_mapped_fd(void *hal_data, unsigned int stream,
			    int index, int orig_fd, int mapped_fd);
int v4l2_hal_get_mapped_fd_for_cid(void *hal_data, unsigned int stream,
				   __u32 cid);
void v4l2_hal_set_mapped_fd_for_cid(void *hal_data, unsigned int stream,
				    __u32 cid, int orig_fd, int mapped_fd);

int v4l2_hal_stream_set_handled(void *hal_data, unsigned int stream);


int v4l2_hal_ext_ctrl_save_private(void *data, void **priv);
int v4l2_hal_ext_ctrl_restore_private(void *data, void *priv);

size_t v4l2_hal_get_required_size32(struct v4l2_ext_controls *kp);
int v4l2_hal_put_ext_controls32(struct v4l2_ext_controls *kp,
				void *uctrls, void *priv);
int v4l2_hal_get_ext_controls32(struct v4l2_ext_controls *kp,
				void *uctrls, void *priv);

size_t v4l2_hal_get_required_size(struct v4l2_ext_controls *kp);
int v4l2_hal_put_ext_controls(struct v4l2_ext_controls *kp,
				void *uctrls, void *priv);
int v4l2_hal_get_ext_controls(struct v4l2_ext_controls *kp,
				void *uctrls, void *priv);

void *v4l2_hal_init(void);
void v4l2_hal_exit(void *);
bool v4l2_hal_check_dev_ready(void);
int v4l2_hal_report_error(void *hal_data, unsigned int code);
#endif /* __V4L2_HAL_INTERNAL_H__ */
