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
#include "camera_ext_defs.h"

#define CAMERA_EXT_DEV_NAME "camera_ext"

struct camera_ext {
	struct gb_connection *connection;
	struct v4l2_device v4l2_dev;
	struct video_device vdev_mod;
};

/* gb functions */
int gb_camera_ext_power_on(struct device *dev);
int gb_camera_ext_power_off(struct device *dev);
int gb_camera_ext_stream_on(struct device *dev);
int gb_camera_ext_stream_off(struct device *dev);
int gb_camera_ext_input_enum(struct device *dev, struct v4l2_input *input);
int gb_camera_ext_input_get(struct device *dev, int32_t *index);
int gb_camera_ext_input_set(struct device *dev, int32_t index);
int gb_camera_ext_format_enum(struct device *dev,
		struct v4l2_fmtdesc *fmtdesc);
int gb_camera_ext_format_get(struct device *dev,
		struct v4l2_format *format);
int gb_camera_ext_format_set(struct device *dev,
		struct v4l2_format *format);
int gb_camera_ext_frmsize_enum(struct device *dev,
		struct v4l2_frmsizeenum *frmsize);
int gb_camera_ext_frmival_enum(struct device *dev,
		struct v4l2_frmivalenum *frmival);
int gb_camera_ext_stream_parm_get(struct device *dev,
		struct v4l2_streamparm *parm);
int gb_camera_ext_stream_parm_set(struct device *dev,
		struct v4l2_streamparm *parm);

/* v4l2 functions */
int camera_ext_mod_v4l2_init(struct camera_ext *cam_dev, struct device *gb_dev);
void camera_ext_mod_v4l2_exit(struct camera_ext *cam_dev);

int camera_ext_v4l2_driver_init(void);
void camera_ext_v4l2_driver_exit(void);
#endif /* CAMERA_EXT_H */
