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

#ifndef __V4L2_HAL_H__
#define __V4L2_HAL_H__

#include <linux/ioctl.h>

/* structure used for misc -> v4l2_hal direction */
struct misc_stream_resp {
	__u32 stream;
	__s32 result_code;
} __packed;

struct misc_dequeue_cmd {
	__u32 stream;
	__u32 index;
	__u32 length;
} __packed;

struct misc_ioctl_resp {
	__u32 stream;
	__u32 cmd;
	__s32 result_code;
	__u32 pad;
	__u64 data;
} __packed;

#define VIOC_HAL_IFACE_START	_IO('H', 0)
#define VIOC_HAL_IFACE_STOP	_IO('H', 1)
#define VIOC_HAL_STREAM_OPENED	_IOW('H', 2, struct misc_stream_resp)
#define VIOC_HAL_STREAM_CLOSED	_IOW('H', 3, struct misc_stream_resp)
#define VIOC_HAL_STREAM_ON	_IOW('H', 4, struct misc_stream_resp)
#define VIOC_HAL_STREAM_OFF	_IOW('H', 5, struct misc_stream_resp)
#define VIOC_HAL_STREAM_REQBUFS	_IOW('H', 6, struct misc_stream_resp)
#define VIOC_HAL_STREAM_QBUF	_IOW('H', 7, struct misc_stream_resp)
#define VIOC_HAL_STREAM_DQBUF	_IOW('H', 8, struct misc_dequeue_cmd)
#define VIOC_HAL_V4L2_CMD	_IOW('H', 9, struct misc_ioctl_resp)

#define V4L2_HAL_MAX_STREAMS 6

/* structure used for V4L2_hal -> misc direction */
struct misc_read_cmd {
	unsigned int stream;
	unsigned int cmd;
	char data[0];
};

struct v4l2_hal_reqbufs_data {
	unsigned int count;
};

struct v4l2_hal_qbuf_data {
	unsigned int index;
	int fd;
	unsigned int length;
};

bool v4l2_misc_compat_mode(void);
int v4l2_misc_process_command(unsigned int stream, unsigned int cmd,
			      size_t size, void *data);
int v4l2_hal_buffer_ready(void *hal_data, unsigned int stream,
			  int fd, unsigned int length);
int v4l2_hal_get_mapped_fd(void *hal_data, unsigned int stream, int index);
void v4l2_hal_set_mapped_fd(void *hal_data, unsigned int stream,
			    int index, int fd);

void *v4l2_hal_init(void);
void v4l2_hal_exit(void *);


#endif /* __V4L2_HAL_H__ */
