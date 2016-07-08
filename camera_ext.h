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

#ifndef CAMERA_EXT_H
#define CAMERA_EXT_H

#include <linux/byteorder/generic.h>
#include <linux/types.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include "camera_ext_defs.h"

#define CAMERA_EXT_DEV_NAME "camera_ext"

enum camera_ext_state {
	CAMERA_EXT_READY = 0,
	CAMERA_EXT_DESTROYED,
};

struct camera_ext {
	struct gb_connection *connection;
	struct v4l2_device v4l2_dev;
	struct video_device *vdev_mod;
	struct v4l2_ctrl_handler hdl_ctrls;

	struct kref kref;
	enum camera_ext_state state;
};

/* gb functions */
int gb_camera_ext_power_on(struct gb_connection *conn, uint8_t mode);
int gb_camera_ext_power_off(struct gb_connection *conn);
int gb_camera_ext_stream_on(struct gb_connection *conn);
int gb_camera_ext_stream_off(struct gb_connection *conn);
int gb_camera_ext_input_enum(struct gb_connection *conn, struct v4l2_input *input);
int gb_camera_ext_input_get(struct gb_connection *conn, int32_t *index);
int gb_camera_ext_input_set(struct gb_connection *conn, int32_t index);
int gb_camera_ext_format_enum(struct gb_connection *conn,
		struct v4l2_fmtdesc *fmtdesc);
int gb_camera_ext_format_get(struct gb_connection *conn,
		struct v4l2_format *format);
int gb_camera_ext_format_set(struct gb_connection *conn,
		struct v4l2_format *format);
int gb_camera_ext_frmsize_enum(struct gb_connection *conn,
		struct v4l2_frmsizeenum *frmsize);
int gb_camera_ext_frmival_enum(struct gb_connection *conn,
		struct v4l2_frmivalenum *frmival);
int gb_camera_ext_stream_parm_get(struct gb_connection *conn,
		struct v4l2_streamparm *parm);
int gb_camera_ext_stream_parm_set(struct gb_connection *conn,
		struct v4l2_streamparm *parm);

struct camera_ext_predefined_ctrl_v4l2_cfg {
	uint32_t id;
	s64 min;
	s64 max;
	u64 step;
	/* point to default value (inside greybus message which
	 * should be released after the registration is done)
	 */
	void *p_def;
	size_t val_size; /* size of the default value */
	u64 menu_skip_mask;
	union {
		u32 dims[V4L2_CTRL_MAX_DIMS];
		s64 menu_int[CAMERA_EXT_MAX_MENU_NUM];
		/* reserve one more space to append a NULL string */
		/* TODO: use 4/8 bytes to transfer float/double over greybus */
		camera_ext_ctrl_float menu_float[CAMERA_EXT_MAX_MENU_NUM + 1];
	};
	unsigned int idx;
};

struct camera_ext_v4l2_ctrl_priv {
	unsigned int idx;
	/* default value for array or string type ctrl */
	void *def;
	size_t def_size;
};

typedef int (*register_custom_mod_ctrl_func_t)(
		struct camera_ext_predefined_ctrl_v4l2_cfg *cfg, void *ctx);

int gb_camera_ext_ctrl_process_all(struct gb_connection *conn,
	register_custom_mod_ctrl_func_t register_custom_mod_ctrl, void *ctx);
int gb_camera_ext_g_volatile_ctrl(struct gb_connection *conn, struct v4l2_ctrl *ctrl);
int gb_camera_ext_s_ctrl(struct gb_connection *conn, struct v4l2_ctrl *ctrl);
int gb_camera_ext_try_ctrl(struct gb_connection *conn, struct v4l2_ctrl *ctrl);

/* v4l2 functions */
int camera_ext_mod_v4l2_init(struct camera_ext *cam_dev);
void camera_ext_mod_v4l2_exit(struct camera_ext *cam_dev);
int camera_ext_v4l2_driver_init(void);
void camera_ext_v4l2_driver_exit(void);
struct v4l2_ctrl_config *camera_ext_get_ctrl_config(uint32_t id);
void camera_ext_mod_v4l2_event_notify(struct camera_ext *cam_dev,
		struct v4l2_camera_ext_event *event);
int camera_ext_mod_v4l2_buffer_notify(struct camera_ext *cam_dev,
		const char *desc, size_t size);
#endif /* CAMERA_EXT_H */
