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

#include "camera_ext.h"
#include "greybus.h"
#include "kernel_ver.h"

/* Version of the Greybus camera protocol we support */
#define GB_CAMERA_EXT_VERSION_MAJOR 0x00
#define GB_CAMERA_EXT_VERSION_MINOR 0x01

/* Greybus camera request types */
#define GB_CAMERA_EXT_TYPE_INVALID		0x00
#define GB_CAMERA_EXT_TYPE_PROTOCOL_VERSION	0x01

#define GB_CAMERA_EXT_TYPE_POWER_ON		0x02
#define GB_CAMERA_EXT_TYPE_POWER_OFF		0x03

#define GB_CAMERA_EXT_TYPE_INPUT_ENUM		0x04
#define GB_CAMERA_EXT_TYPE_INPUT_GET		0x05
#define GB_CAMERA_EXT_TYPE_INPUT_SET		0x06

#define GB_CAMERA_EXT_TYPE_FMT_ENUM		0x07
#define GB_CAMERA_EXT_TYPE_FMT_GET		0x08
#define GB_CAMERA_EXT_TYPE_FMT_SET		0x09

#define GB_CAMERA_EXT_TYPE_FMSIZE_ENUM		0x0A
#define GB_CAMERA_EXT_TYPE_FRMIVAL_ENUM		0x0B

#define GB_CAMERA_EXT_TYPE_STREAM_ON		0x0C
#define GB_CAMERA_EXT_TYPE_STREAM_OFF		0x0D

#define GB_CAMERA_EXT_TYPE_STREAM_PARM_SET	0x0E
#define GB_CAMERA_EXT_TYPE_STREAM_PARM_GET	0x0F

#define dev_to_conn(dev) container_of(dev, struct gb_connection, dev)

/* dev points to gb_connection->dev for all gb_camera_ext_XYZ functions */
int gb_camera_ext_power_on(struct device *dev)
{
	return gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_POWER_ON,
				NULL,
				0,
				NULL,
				0);
}

int gb_camera_ext_power_off(struct device *dev)
{
	return gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_POWER_OFF,
				NULL,
				0,
				NULL,
				0);
}

int gb_camera_ext_stream_on(struct device *dev)
{
	return gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_STREAM_ON,
				NULL,
				0,
				NULL,
				0);
}

int gb_camera_ext_stream_off(struct device *dev)
{
	return gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_STREAM_OFF,
				NULL,
				0,
				NULL,
				0);
}

int gb_camera_ext_input_enum(struct device *dev, struct v4l2_input *inp)
{
	int retval;
	struct camera_ext_input input;
	__le32 index = cpu_to_le32(inp->index);

	retval = gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_INPUT_ENUM,
				&index,
				sizeof(index),
				&input,
				sizeof(input));
	if (retval == 0) {
		memcpy(inp->name, input.name, sizeof(inp->name));
		inp->type = le32_to_cpu(input.type);
		inp->status = le32_to_cpu(input.status);
	}
	return retval;
}

int gb_camera_ext_input_get(struct device *dev, int32_t *i)
{

	int retval;
	__le32 index = 0;

	retval = gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_INPUT_GET,
				NULL,
				0,
				&index,
				sizeof(index));
	if (retval == 0)
		*i = le32_to_cpu(index);
	return retval;
}

int gb_camera_ext_input_set(struct device *dev, int32_t i)
{
	__le32 index = cpu_to_le32(i);

	return gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_INPUT_SET,
				&index,
				sizeof(index),
				NULL,
				0);
}

int gb_camera_ext_format_enum(struct device *dev, struct v4l2_fmtdesc *fmt)
{
	int retval;
	__le32 index;
	struct camera_ext_fmtdesc fmtdesc;

	memset(&fmtdesc, 0, sizeof(fmtdesc));
	index = cpu_to_le32(fmt->index);
	retval = gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_FMT_ENUM,
				&index,
				sizeof(index),
				&fmtdesc,
				sizeof(fmtdesc));
	if (retval == 0) {
		memcpy(fmt->description, fmtdesc.name,
			sizeof(fmt->description));
		fmt->pixelformat = le32_to_cpu(fmtdesc.fourcc);
	}
	return retval;
}

int gb_camera_ext_format_get(struct device *dev, struct v4l2_format *fmt)
{
	int retval;
	struct camera_ext_format format;

	retval = gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_FMT_GET,
				NULL,
				0,
				&format,
				sizeof(format));
	if (retval == 0) {
		fmt->fmt.pix.width = le32_to_cpu(format.width);
		fmt->fmt.pix.height = le32_to_cpu(format.height);
		fmt->fmt.pix.pixelformat = le32_to_cpu(format.pixelformat);
		fmt->fmt.pix.bytesperline = le32_to_cpu(format.bytesperline);
		fmt->fmt.pix.sizeimage = le32_to_cpu(format.sizeimage);
	}
	return retval;
}

int gb_camera_ext_format_set(struct device *dev, struct v4l2_format *fmt)
{
	int retval;
	struct camera_ext_format format;

	format.width = cpu_to_le32(fmt->fmt.pix.width);
	format.height = cpu_to_le32(fmt->fmt.pix.height);
	format.pixelformat = cpu_to_le32(fmt->fmt.pix.pixelformat);
	retval = gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_FMT_SET,
				&format,
				sizeof(format),
				NULL,
				0);
	return retval;
}

int gb_camera_ext_frmsize_enum(struct device *dev,
		struct v4l2_frmsizeenum *frmsize)
{
	int retval;
	struct camera_ext_frmsize mod_frmsize;

	mod_frmsize.index = cpu_to_le32(frmsize->index);
	mod_frmsize.pixelformat = cpu_to_le32(frmsize->pixel_format);
	retval = gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_FMSIZE_ENUM,
				&mod_frmsize,
				sizeof(mod_frmsize),
				&mod_frmsize,
				sizeof(mod_frmsize));
	if (retval == 0) {
		frmsize->type = le32_to_cpu(mod_frmsize.type);
		switch (frmsize->type) {
		case V4L2_FRMSIZE_TYPE_DISCRETE:
			frmsize->discrete.width = le32_to_cpu(
				mod_frmsize.discrete.width);
			frmsize->discrete.height = le32_to_cpu(
				mod_frmsize.discrete.height);
			break;

		case CAM_EXT_FRMSIZE_TYPE_STEPWISE:
			frmsize->stepwise.min_width = le32_to_cpu(
				mod_frmsize.stepwise.min_width);
			frmsize->stepwise.max_width = le32_to_cpu(
				mod_frmsize.stepwise.max_width);
			frmsize->stepwise.step_width = le32_to_cpu(
				mod_frmsize.stepwise.step_width);

			frmsize->stepwise.min_height = le32_to_cpu(
				mod_frmsize.stepwise.min_height);
			frmsize->stepwise.max_height = le32_to_cpu(
				mod_frmsize.stepwise.max_height);
			frmsize->stepwise.step_height = le32_to_cpu(
				mod_frmsize.stepwise.step_height);
			break;

		case CAM_EXT_FRMSIZE_TYPE_CONTINUOUS:
			break;
		}
	}
	return retval;
}

int gb_camera_ext_frmival_enum(struct device *dev,
		struct v4l2_frmivalenum *frmival)
{
	int retval;
	struct camera_ext_frmival mod_frmival;

	memset(&mod_frmival, 0, sizeof(mod_frmival));
	mod_frmival.index = cpu_to_le32(frmival->index);
	mod_frmival.pixelformat = cpu_to_le32(frmival->pixel_format);
	mod_frmival.width = cpu_to_le32(frmival->width);
	mod_frmival.height = cpu_to_le32(frmival->height);

	retval = gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_FRMIVAL_ENUM,
				&mod_frmival,
				sizeof(mod_frmival),
				&mod_frmival,
				sizeof(mod_frmival));
	if (retval == 0) {
		frmival->type = le32_to_cpu(mod_frmival.type);
		switch (frmival->type) {
		case V4L2_FRMIVAL_TYPE_DISCRETE:
			frmival->discrete.numerator = le32_to_cpu(
				mod_frmival.discrete.numerator);
			frmival->discrete.denominator = le32_to_cpu(
				mod_frmival.discrete.denominator);
			break;

		case V4L2_FRMIVAL_TYPE_STEPWISE:
			frmival->stepwise.min.numerator = le32_to_cpu(
				mod_frmival.stepwise.min.numerator);
			frmival->stepwise.min.denominator = le32_to_cpu(
				mod_frmival.stepwise.min.denominator);
			frmival->stepwise.max.numerator = le32_to_cpu(
				mod_frmival.stepwise.max.numerator);
			frmival->stepwise.max.denominator = le32_to_cpu(
				mod_frmival.stepwise.max.denominator);
			frmival->stepwise.step.numerator = le32_to_cpu(
				mod_frmival.stepwise.max.denominator);
			frmival->stepwise.step.denominator = le32_to_cpu(
				mod_frmival.stepwise.step.denominator);
			break;

		case V4L2_FRMIVAL_TYPE_CONTINUOUS:
			break;
		}
	}
	return retval;
}

int gb_camera_ext_stream_parm_set(struct device *dev,
		struct v4l2_streamparm *parm)
{
	struct camera_ext_streamparm mod_streamparm;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(&mod_streamparm, 0, sizeof(mod_streamparm));
	mod_streamparm.type = cpu_to_le32(parm->type);
	mod_streamparm.capture.timeperframe.numerator = cpu_to_le32(
			parm->parm.capture.timeperframe.numerator);
	mod_streamparm.capture.timeperframe.denominator = cpu_to_le32(
				parm->parm.capture.timeperframe.denominator);
	return gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_STREAM_PARM_SET,
				&mod_streamparm,
				sizeof(mod_streamparm),
				NULL,
				0);
}

int gb_camera_ext_stream_parm_get(struct device *dev,
		struct v4l2_streamparm *parm)
{
	int retval;
	struct camera_ext_streamparm mod_streamparm;

	memset(&mod_streamparm, 0, sizeof(mod_streamparm));
	retval = gb_operation_sync(dev_to_conn(dev),
				GB_CAMERA_EXT_TYPE_STREAM_PARM_GET,
				NULL,
				0,
				&mod_streamparm,
				sizeof(mod_streamparm));
	if (retval == 0) {
		parm->type = __le32_to_cpu(mod_streamparm.type);
		if (parm->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			struct camera_ext_captureparm *captureparm =
				&mod_streamparm.capture;
			struct camera_ext_fract *fps =
				&captureparm->timeperframe;

			parm->parm.capture.capability =
				le32_to_cpu(captureparm->capability);
			parm->parm.capture.capturemode =
				le32_to_cpu(captureparm->capturemode);
			parm->parm.capture.timeperframe.numerator =
				le32_to_cpu(fps->numerator);
			parm->parm.capture.timeperframe.denominator =
				le32_to_cpu(fps->denominator);
			parm->parm.capture.extendedmode =
				le32_to_cpu(captureparm->extendedmode);
			parm->parm.capture.readbuffers =
				le32_to_cpu(captureparm->readbuffers);
		} else {
			pr_err("unsupported type:%d\n", parm->type);
			retval = -EINVAL;
		}
	}
	return retval;
}

static int gb_camera_ext_connection_init(struct gb_connection *connection)
{
	int retval;
	struct camera_ext *cam;

	cam = kzalloc(sizeof(*cam), GFP_KERNEL);
	if (!cam)
		return -ENOMEM;

	cam->connection = connection;
	connection->private = cam;

	retval = camera_ext_mod_v4l2_init(cam, &connection->dev);
	if (retval) {
		pr_err("failed to init v4l2 for mod control\n");
		kfree(cam);
	}

	return retval;
}

static void gb_camera_ext_connection_exit(struct gb_connection *connection)
{
	struct camera_ext *cam = connection->private;

	camera_ext_mod_v4l2_exit(cam);
	kfree(cam);
}

static struct gb_protocol camera_ext_protocol = {
	.name			= "camera_ext",
	.id			= GREYBUS_PROTOCOL_CAMERA_EXT,
	.major			= GB_CAMERA_EXT_VERSION_MAJOR,
	.minor			= GB_CAMERA_EXT_VERSION_MINOR,
	.connection_init	= gb_camera_ext_connection_init,
	.connection_exit	= gb_camera_ext_connection_exit,
	.request_recv		= NULL, /* no incoming requests */
};

gb_protocol_driver(&camera_ext_protocol);

MODULE_LICENSE("GPL v2");
