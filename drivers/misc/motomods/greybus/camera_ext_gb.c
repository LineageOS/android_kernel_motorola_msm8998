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
#include <uapi/video/v4l2_camera_ext_events.h>

#include "camera_ext.h"
#include "greybus.h"
#include "kernel_ver.h"

#define MAX_CTRLS_SUPPORT 1000

int gb_camera_ext_power_on(struct gb_connection *conn, uint8_t mode)
{
	return gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_POWER_ON,
				&mode,
				sizeof(mode),
				NULL,
				0);
}

int gb_camera_ext_power_off(struct gb_connection *conn)
{
	return gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_POWER_OFF,
				NULL,
				0,
				NULL,
				0);
}

int gb_camera_ext_stream_on(struct gb_connection *conn)
{
	return gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_STREAM_ON,
				NULL,
				0,
				NULL,
				0);
}

int gb_camera_ext_stream_off(struct gb_connection *conn)
{
	return gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_STREAM_OFF,
				NULL,
				0,
				NULL,
				0);
}

int gb_camera_ext_input_enum(struct gb_connection *conn, struct v4l2_input *inp)
{
	int retval;
	struct camera_ext_input input;
	__le32 index = cpu_to_le32(inp->index);

	retval = gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_INPUT_ENUM,
				&index,
				sizeof(index),
				&input,
				sizeof(input));
	if (retval == 0) {
		if (input.index == GB_CAMERA_EXT_INVALID_INDEX)
			return -EFAULT;

		memcpy(inp->name, input.name, sizeof(inp->name));
		inp->type = le32_to_cpu(input.type);
		inp->status = le32_to_cpu(input.status);
		inp->capabilities = le32_to_cpu(input.capabilities);
	}
	return retval;
}

int gb_camera_ext_input_get(struct gb_connection *conn, int32_t *i)
{

	int retval;
	__le32 index = 0;

	retval = gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_INPUT_GET,
				NULL,
				0,
				&index,
				sizeof(index));
	if (retval == 0)
		*i = le32_to_cpu(index);
	return retval;
}

int gb_camera_ext_input_set(struct gb_connection *conn, int32_t i)
{
	__le32 index = cpu_to_le32(i);

	return gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_INPUT_SET,
				&index,
				sizeof(index),
				NULL,
				0);
}

int gb_camera_ext_format_enum(struct gb_connection *conn, struct v4l2_fmtdesc *fmt)
{
	int retval;
	__le32 index;
	struct camera_ext_fmtdesc fmtdesc;

	memset(&fmtdesc, 0, sizeof(fmtdesc));
	index = cpu_to_le32(fmt->index);
	retval = gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_FMT_ENUM,
				&index,
				sizeof(index),
				&fmtdesc,
				sizeof(fmtdesc));
	if (retval == 0) {
		if (fmtdesc.index == GB_CAMERA_EXT_INVALID_INDEX)
			return -EFAULT;

		memcpy(fmt->description, fmtdesc.name,
			sizeof(fmt->description));
		fmt->pixelformat = le32_to_cpu(fmtdesc.fourcc);
	}
	return retval;
}

int gb_camera_ext_format_get(struct gb_connection *conn, struct v4l2_format *fmt)
{
	int retval;
	struct camera_ext_format format;

	retval = gb_operation_sync(conn,
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

int gb_camera_ext_format_set(struct gb_connection *conn, struct v4l2_format *fmt)
{
	int retval;
	struct camera_ext_format format;

	format.width = cpu_to_le32(fmt->fmt.pix.width);
	format.height = cpu_to_le32(fmt->fmt.pix.height);
	format.pixelformat = cpu_to_le32(fmt->fmt.pix.pixelformat);
	retval = gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_FMT_SET,
				&format,
				sizeof(format),
				NULL,
				0);
	return retval;
}

int gb_camera_ext_frmsize_enum(struct gb_connection *conn,
		struct v4l2_frmsizeenum *frmsize)
{
	int retval;
	struct camera_ext_frmsize mod_frmsize;

	mod_frmsize.index = cpu_to_le32(frmsize->index);
	mod_frmsize.pixelformat = cpu_to_le32(frmsize->pixel_format);
	retval = gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_FMSIZE_ENUM,
				&mod_frmsize,
				sizeof(mod_frmsize),
				&mod_frmsize,
				sizeof(mod_frmsize));
	if (retval == 0) {
		if (mod_frmsize.index == GB_CAMERA_EXT_INVALID_INDEX)
			return -EFAULT;

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

int gb_camera_ext_frmival_enum(struct gb_connection *conn,
		struct v4l2_frmivalenum *frmival)
{
	int retval;
	struct camera_ext_frmival mod_frmival;

	memset(&mod_frmival, 0, sizeof(mod_frmival));
	mod_frmival.index = cpu_to_le32(frmival->index);
	mod_frmival.pixelformat = cpu_to_le32(frmival->pixel_format);
	mod_frmival.width = cpu_to_le32(frmival->width);
	mod_frmival.height = cpu_to_le32(frmival->height);

	retval = gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_FRMIVAL_ENUM,
				&mod_frmival,
				sizeof(mod_frmival),
				&mod_frmival,
				sizeof(mod_frmival));
	if (retval == 0) {
		if (mod_frmival.index == GB_CAMERA_EXT_INVALID_INDEX)
			return -EFAULT;

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

int gb_camera_ext_stream_parm_set(struct gb_connection *conn,
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
	return gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_STREAM_PARM_SET,
				&mod_streamparm,
				sizeof(mod_streamparm),
				NULL,
				0);
}

int gb_camera_ext_stream_parm_get(struct gb_connection *conn,
		struct v4l2_streamparm *parm)
{
	int retval;
	struct camera_ext_streamparm mod_streamparm;

	memset(&mod_streamparm, 0, sizeof(mod_streamparm));
	retval = gb_operation_sync(conn,
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

/* read s64 from *p and fill it to target */
static inline int get_s64(uint8_t **p, size_t *size, s64 *target)
{
	if (*size < sizeof(s64))
		return -EINVAL;

	*target = (s64)(le64_to_cpu(*(__le64*)*p));
	*p += sizeof(s64);
	*size -= sizeof(s64);

	return 0;
}

/* read list of s64 from *p and fill them to target */
static inline int get_s64s(uint8_t **p, size_t *size, s64 *target, size_t num)
{
	int retval = 0;
	size_t i;
	if (*size < sizeof(s64) * num)
		return -EINVAL;

	for (i = 0; i < num; i++) {
		retval = get_s64(p, size, target++);
		if (retval != 0)
			break;
	}

	return retval;
}

/* read u64 from *p and fill it to target */
static inline int get_u64(uint8_t **p, size_t *size, u64 *target)
{
	if (*size < sizeof(u64))
		return -EINVAL;

	*target = le64_to_cpu(*(__le64*)*p);
	*p += sizeof(u64);
	*size -= sizeof(u64);

	return 0;
}

/* read u32 from *p and fill it to target */
static inline int get_u32(uint8_t **p, size_t *size, u32 *target)
{
	if (*size < sizeof(u32))
		return -EINVAL;

	*target = le32_to_cpu(*(__le32*)*p);
	*p += sizeof(u32);
	*size -= sizeof(u32);

	return 0;
}

/* read list of u32 from *p and fill them to target */
static inline int get_u32s(uint8_t **p, size_t *size, u32 *target, size_t num)
{
	int retval = 0;
	size_t i;
	if (*size < sizeof(u32) * num) {
		pr_err("%s: expect %zu u32 from %zu bytes\n", __func__,
			num, *size);
		return -EINVAL;
	}

	for (i = 0; i < num; i++) {
		retval = get_u32(p, size, target++);
		if (retval != 0)
			break;
	}

	return retval;
}

static inline void camera_ext_ctrl_float_set_empty(camera_ext_ctrl_float *ff)
{
	(*ff)[0] = '\0';
}

static inline void camera_ext_ctrl_float_copy(camera_ext_ctrl_float *src,
						camera_ext_ctrl_float *dst)
{
	memcpy(*src, *dst, sizeof(camera_ext_ctrl_float));
}

static inline void camera_ext_ctrl_double_copy(camera_ext_ctrl_double *src,
						camera_ext_ctrl_double *dst)
{
	memcpy(*src, *dst, sizeof(camera_ext_ctrl_double));
}

static inline int get_camera_ext_ctrl_float(uint8_t **p, size_t *size,
					camera_ext_ctrl_float *target)
{
	if (*size < sizeof(camera_ext_ctrl_float)) {
		pr_err("%s: invalid camera_ext_ctrl_float size %zu\n",
			__func__, *size);
		return -EINVAL;
	}

	camera_ext_ctrl_float_copy(target, (camera_ext_ctrl_float*) *p);
	*p += sizeof(camera_ext_ctrl_float);
	*size -= sizeof(camera_ext_ctrl_float);

	return 0;
}

static inline int get_camera_ext_ctrl_double(uint8_t **p, size_t *size,
					camera_ext_ctrl_double *target)
{
	if (*size < sizeof(camera_ext_ctrl_double)) {
		pr_err("%s: invalid camera_ext_ctrl_double size %zu\n",
			__func__, *size);
		return -EINVAL;
	}

	camera_ext_ctrl_double_copy(target, (camera_ext_ctrl_double*) *p);
	*p += sizeof(camera_ext_ctrl_double);
	*size -= sizeof(camera_ext_ctrl_double);

	return 0;
}

/* read list of camera_ext_ctrl_float from *p and fill them to target */
static inline int get_camera_ext_ctrl_floats(uint8_t **p, size_t *size,
	camera_ext_ctrl_float *target, size_t num)
{
	if (*size < sizeof(camera_ext_ctrl_float) * num) {
		pr_err("%s: expect %zu floats from %zu bytes\n", __func__,
			num, *size);
		return -EINVAL;
	}

	memcpy(target, *p, sizeof(camera_ext_ctrl_float) * num);
	*p += sizeof(camera_ext_ctrl_float) * num;
	*size -= sizeof(camera_ext_ctrl_float) * num;

	target += num; /* we reserved MENU_MAX + 1 items */
	camera_ext_ctrl_float_set_empty(target); /* add a NULL string */
	return 0;
}

/* Save default value point. NOTE: greybus message must not be released before
 * the default value is consumed.
 */
static inline int get_def(uint8_t **p, size_t *size, void **q, size_t length,
			uint32_t type)
{
	size_t i;
	uint8_t *num;

	if (*size < length) {
		pr_err("%s: expect %zu bytes from %zu bytes\n", __func__,
			length, *size);
		return -EINVAL;
	}

	if (type != V4L2_CTRL_TYPE_STRING) {
		/* in place edian conversion for number */
		i = 0;
		num = *p;

		switch (type) {
		case V4L2_CTRL_TYPE_INTEGER:
		case V4L2_CTRL_TYPE_BOOLEAN:
		case V4L2_CTRL_TYPE_INTEGER_MENU:
		case V4L2_CTRL_TYPE_MENU:
		case V4L2_CTRL_TYPE_BITMASK:
			while (i < length) {
				*(uint32_t *)num = le32_to_cpu(*(__le32 *)num);
				num += sizeof(uint32_t);
				i += sizeof(uint32_t);
			}
			break;
		case V4L2_CTRL_TYPE_INTEGER64:
			while (i < length) {
				*(uint64_t *)num = le64_to_cpu(*(__le64 *)num);
				num += sizeof(uint64_t);
				i += sizeof(uint64_t);
			}
			break;
		default:
			pr_err("%s: unknown type %d\n", __func__, type);
			return -EINVAL;
		}
		if (i != length) {
			pr_err("%s: wrong value bytes: %zu\n", __func__, i);
			return -EINVAL;
		}
	} /* note: if use 4/8 bytes pass float/double, conversion is needed */

	*q = *p;
	*p += length;
	*size -= length;

	return 0;
}

/* Read a field from stream p and fill to cam_ext_v4l2_cfg.
 * This field is described by mod_flag_tag (CAMERA_EXT_CTRL_FLAG_NEED_XXX)
 *
 * predef_cfg - predefined config for this control
 * p - stream (from greybus) to read out the field. Advance *p after read.
 * reamain_size - size of stream. descrease size after read.
 * array_size - If this config has array data (dim, menu). It's the size.
 * cam_ext_v4l2_cfg - cfg to update.
 */
static int camera_ext_ctrl_process_one_field(
		struct v4l2_ctrl_config *predef_cfg,
		uint32_t mod_flag_tag,
		uint8_t **p,
		size_t *remain_size,
		uint32_t array_size,
		uint32_t val_size,
		struct camera_ext_predefined_ctrl_v4l2_cfg *cam_ext_v4l2_cfg)
{
	int retval = 0;

	switch (mod_flag_tag) {
	case CAMERA_EXT_CTRL_FLAG_NEED_MIN:
		retval = get_s64(p, remain_size, &cam_ext_v4l2_cfg->min);
		break;
	case CAMERA_EXT_CTRL_FLAG_NEED_MAX:
		retval = get_s64(p, remain_size, &cam_ext_v4l2_cfg->max);
		break;
	case CAMERA_EXT_CTRL_FLAG_NEED_STEP:
		retval = get_u64(p, remain_size, &cam_ext_v4l2_cfg->step);
		break;
	case CAMERA_EXT_CTRL_FLAG_NEED_DEF:
		retval = get_def(p, remain_size, &cam_ext_v4l2_cfg->p_def,
				val_size, predef_cfg->type);
		if (retval == 0)
			cam_ext_v4l2_cfg->val_size = val_size;
		break;
	case CAMERA_EXT_CTRL_FLAG_NEED_DIMS:
		if (unlikely(array_size > V4L2_CTRL_MAX_DIMS))
			pr_warn("%s: illegal dims %u\n", __func__, array_size);
		else
			retval = get_u32s(p, remain_size,
					cam_ext_v4l2_cfg->dims,
					array_size);
		if (array_size < V4L2_CTRL_MAX_DIMS)
			cam_ext_v4l2_cfg->dims[array_size] = 0;
		break;
	case CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK:
		retval = get_u64(p, remain_size,
				&cam_ext_v4l2_cfg->menu_skip_mask);
		break;
	case CAMERA_EXT_CTRL_FLAG_NEED_MENU_INT:
		if (unlikely(array_size > CAMERA_EXT_MAX_MENU_NUM))
			pr_warn("%s: illegal menu number %u\n", __func__,
				array_size);
		else
			retval = get_s64s(p, remain_size,
					cam_ext_v4l2_cfg->menu_int,
					array_size);
		break;
	case CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT:
		if (unlikely(array_size > CAMERA_EXT_MAX_MENU_NUM))
			pr_warn("%s: illegal menu number %u\n", __func__,
				array_size);
		else
			retval = get_camera_ext_ctrl_floats(p, remain_size,
					cam_ext_v4l2_cfg->menu_float,
					array_size);
		break;
	default:
		pr_err("unknow flag 0x%x\n", mod_flag_tag);
		retval = -EINVAL;
		break;
	}

	return retval;
}

/* Process one control config data from MOD.
 * ctrl_idx - current processing control index at mod side. This index will be
 * used to access mod control later.
 * mod_cfg - mod side control config data
 * mod_cfg_size - size in bytes of mod_cfg
 * array_size - if this config has array data (dim or menu). It's the size.
 * val_size - if this config has default value. It's the size.
 * register_custom_mod_ctrl - function to register the control to kernel.
 * ctx - parameter of register_custom_mod_ctrl
 */
static int camera_ext_ctrl_process_one(
	uint32_t ctrl_idx,
	struct camera_ext_predefined_ctrl_mod_cfg *mod_cfg,
	uint32_t mod_cfg_size,
	uint32_t array_size,
	uint32_t val_size,
	register_custom_mod_ctrl_func_t register_custom_mod_ctrl,
	void *ctx)
{
	int retval;
	uint8_t *p;
	size_t remain_size;
	uint32_t mod_flag_tag;
	struct v4l2_ctrl_config *predef_cfg;
	struct camera_ext_predefined_ctrl_v4l2_cfg cam_ext_v4l2_cfg;

	cam_ext_v4l2_cfg.id = (le32_to_cpu(mod_cfg->id) & CAM_EXT_CTRL_ID_MASK);
	predef_cfg = camera_ext_get_ctrl_config(cam_ext_v4l2_cfg.id);
	if (predef_cfg == NULL) {
		pr_err("%s: invalid config id %d from MOD\n", __func__,
			cam_ext_v4l2_cfg.id);
		return -EINVAL;
	}

	cam_ext_v4l2_cfg.idx = ctrl_idx;
	p = mod_cfg->data;
	remain_size = mod_cfg_size - sizeof(*mod_cfg);

	retval = 0;
	while (retval == 0 && remain_size > 0) {
		retval = get_u32(&p, &remain_size, &mod_flag_tag);
		if (retval != 0) {
			pr_err("failed to read flag tag\n");
			break;
		}

		/* MOD can only use flags used by phone */
		if (!(predef_cfg->flags & mod_flag_tag)) {
			pr_err("reject mod control %d with illegal flag %x\n",
				cam_ext_v4l2_cfg.id, mod_flag_tag);
			retval = -EINVAL;
			break;
		}
		retval = camera_ext_ctrl_process_one_field(predef_cfg,
				mod_flag_tag,
				&p, &remain_size, array_size, val_size,
				&cam_ext_v4l2_cfg);
	}

	if (retval == 0)
		retval = register_custom_mod_ctrl(&cam_ext_v4l2_cfg, ctx);
	else
		pr_err("failed to parse config data for control %d\n",
			cam_ext_v4l2_cfg.id);

	/* TODO: if CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER is set,
	 * the default control value will be spaces with number 'ctrl->min'.
	 * Need to call extra get_ctrl. (extra get_ctrl also needed for controls
	 * with array value.
	 */
	return retval;
}

static int camera_ext_ctrl_get_cfg_payload_size(size_t *data_size, uint32_t id,
		uint32_t next_array_size, uint32_t next_val_size)
{
	struct v4l2_ctrl_config *cfg = camera_ext_get_ctrl_config(id);
	uint32_t flags;
	*data_size = 0;
	if (cfg == NULL) {
		pr_err("%s: control id %x not found\n", __func__, id);
		return -EINVAL;
	}

	/* the config data has the format [FLAG0 DATA0 FLAG1 DATA1 ...] */
	flags = cfg->flags;
	if (flags & CAMERA_EXT_CTRL_FLAG_NEED_MIN)
		*data_size += sizeof(uint32_t) + sizeof(s64);

	if (flags & CAMERA_EXT_CTRL_FLAG_NEED_MAX)
		*data_size += sizeof(uint32_t) + sizeof(s64);

	if (flags & CAMERA_EXT_CTRL_FLAG_NEED_STEP)
		*data_size += sizeof(uint32_t) + sizeof(u64);

	if (flags & CAMERA_EXT_CTRL_FLAG_NEED_DEF) {
		if (next_val_size <= CAMERA_EXT_CTRL_MAX_VAL_SIZE)
			*data_size += sizeof(uint32_t) + next_val_size;
		else {
			pr_err("%s: id %x val size exceeds limit\n",
				__func__, next_val_size);
			return -EINVAL;
		}
	}

	if (flags & CAMERA_EXT_CTRL_FLAG_NEED_DIMS) {
		if (next_array_size <= V4L2_CTRL_MAX_DIMS)
			*data_size += sizeof(uint32_t)
				+ sizeof(u32) * next_array_size;
		else {
			pr_err("%s: wrong dim size %u\n", __func__,
				next_array_size);
			return -EINVAL;
		}
	}

	if (flags & CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK)
		*data_size += sizeof(uint32_t) + sizeof(u64);

	if (flags & CAMERA_EXT_CTRL_FLAG_NEED_MENU_INT) {
		if (next_array_size <= CAMERA_EXT_MAX_MENU_NUM)
			*data_size += sizeof(uint32_t)
				+ sizeof(s64) * next_array_size;
		else {
			pr_err("%s: wrong int menu size %u\n", __func__,
				next_array_size);
			return -EINVAL;
		}
	}

	if (flags & CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT) {
		if (next_array_size <= CAMERA_EXT_MAX_MENU_NUM)
			*data_size += sizeof(uint32_t)
					+ sizeof(camera_ext_ctrl_float)
					* next_array_size;
		else {
			pr_err("%s: wrong float menu size %u\n", __func__,
				next_array_size);
			return -EINVAL;
		}
	}
	return 0;
}

int gb_camera_ext_ctrl_process_all(struct gb_connection *conn,
	register_custom_mod_ctrl_func_t register_custom_mod_ctrl, void *ctx)
{
	int retval = 0;
	uint32_t idx;
	size_t data_size, last_array_size, last_val_size;
	struct camera_ext_predefined_ctrl_mod_cfg *mod_cfg;
	struct camera_ext_predefined_ctrl_mod_req req;

	idx = -1;
	/* -1 is invalid index at MOD side. The return mod_cfg will
	 * only have the header.
	 */
	data_size = sizeof(*mod_cfg);
	last_array_size = 0;
	last_val_size = 0;
	while (1) {
		mod_cfg = kmalloc(data_size, GFP_KERNEL);
		if (mod_cfg == NULL) {
			retval = -ENOMEM;
			break;
		}

		req.idx = cpu_to_le32(idx);
		req.data_size = cpu_to_le32(data_size);
		retval = gb_operation_sync(conn,
				GB_CAMERA_EXT_TYPE_CTRL_GET_CFG,
				&req,
				sizeof(req),
				mod_cfg,
				data_size);
		if (retval != 0) {
			pr_err("%s: get cfg failed\n", __func__);
			kfree(mod_cfg);
			break;
		}

		/* skip if no control for this idx (e.g. -1 index) */
		if (mod_cfg->id != (uint32_t) -1) {
			retval = camera_ext_ctrl_process_one(idx, mod_cfg,
					data_size, last_array_size,
					last_val_size,
					register_custom_mod_ctrl, ctx);
			if (retval != 0) {
				pr_err("failed to process control %x\n",
					mod_cfg->id);
				kfree(mod_cfg);
				break;
			}
		}

		if (mod_cfg->next.id != (uint32_t) -1) {
			/* save for later use when unpack
			 * next_id's config.
			 */
			last_array_size =
				le32_to_cpu(mod_cfg->next.array_size);
			last_val_size =
				le32_to_cpu(mod_cfg->next.val_size);
			/* calculate next response size */
			retval = camera_ext_ctrl_get_cfg_payload_size(
				&data_size,
				le32_to_cpu(mod_cfg->next.id),
				last_array_size, last_val_size);
			if (retval == 0)
				/* adding header */
				data_size += sizeof(*mod_cfg);
			else {
				pr_err("failed to calc next pkt size\n");
				kfree(mod_cfg);
				break;
			}
		} else {
			/* no more controls */
			kfree(mod_cfg);
			break;
		}

		kfree(mod_cfg);
		++idx;
		/* in case a misbehave mod cause a deadloop here */
		if (idx > MAX_CTRLS_SUPPORT) {
			retval = -EINVAL;
			break;
		}
	}

	return retval;
}

/* called by get_ctrl */
static int ctrl_val_mod_to_v4l2(uint8_t *mod_ctrl_val, struct v4l2_ctrl *ctrl)
{
	int retval = 0;
	uint32_t i;
	uint8_t *p = mod_ctrl_val;
	uint8_t *q = ctrl->p_new.p;

	for (i = 0; i < ctrl->elems; i++) {
		switch (ctrl->type) {
		case V4L2_CTRL_TYPE_INTEGER:
		case V4L2_CTRL_TYPE_BOOLEAN:
		case V4L2_CTRL_TYPE_MENU:
		case V4L2_CTRL_TYPE_INTEGER_MENU:
		case V4L2_CTRL_TYPE_BITMASK:
			*(s32 *)q = le32_to_cpu(*(__le32 *)p);
			break;
		case V4L2_CTRL_TYPE_INTEGER64:
			*(s64 *)q = le64_to_cpu(*(__le64 *)p);
			break;
		case V4L2_CTRL_TYPE_STRING:
			memcpy(q, p, ctrl->elem_size);
			break;

		default:
			retval = -EINVAL;
			pr_err("%s: error type %d\n", __func__, ctrl->type);
			break;
		}
		p += ctrl->elem_size;
		q += ctrl->elem_size;
	}
	return retval;
}

int gb_camera_ext_g_volatile_ctrl(struct gb_connection *conn,
		struct v4l2_ctrl *ctrl)
{
	int retval = -EINVAL;
	struct camera_ext_predefined_ctrl_mod_req req;
	size_t mod_ctrl_val_size;
	uint8_t *mod_ctrl_val;
	struct camera_ext_v4l2_ctrl_priv *priv = ctrl->priv;

	mod_ctrl_val_size = ctrl->elems * ctrl->elem_size;
	mod_ctrl_val = kmalloc(mod_ctrl_val_size, GFP_KERNEL);
	if (mod_ctrl_val == NULL)
		return -ENOMEM;

	req.idx = cpu_to_le32(priv->idx);
	req.data_size = cpu_to_le32(mod_ctrl_val_size);

	retval = gb_operation_sync(conn,
			GB_CAMERA_EXT_TYPE_CTRL_GET,
			&req,
			sizeof(req),
			mod_ctrl_val,
			mod_ctrl_val_size);
	if (retval == 0)
		retval = ctrl_val_mod_to_v4l2(mod_ctrl_val, ctrl);
	kfree(mod_ctrl_val);

	return retval;
}

/* called by set_ctrl */
static int ctrl_val_v4l2_to_mod(struct v4l2_ctrl *ctrl, uint8_t *mod_ctrl_val)
{
	int retval = 0;
	uint32_t i;
	uint8_t *p = mod_ctrl_val;
	uint8_t *q = ctrl->p_new.p;

	for (i = 0; i < ctrl->elems; i++) {
		switch (ctrl->type) {
		case V4L2_CTRL_TYPE_INTEGER:
		case V4L2_CTRL_TYPE_BOOLEAN:
		case V4L2_CTRL_TYPE_BUTTON:
		case V4L2_CTRL_TYPE_MENU:
		case V4L2_CTRL_TYPE_INTEGER_MENU:
		case V4L2_CTRL_TYPE_BITMASK:
			*(__le32 *)p = cpu_to_le32(*(s32*)q);
			break;
		case V4L2_CTRL_TYPE_STRING:
			memcpy(p, q, ctrl->elem_size);
			break;
		case V4L2_CTRL_TYPE_INTEGER64:
			*(__le64 *)p = cpu_to_le64(*(s64*)q);
			break;

		default:
			retval = -EINVAL;
			pr_err("%s: error type %d\n", __func__, ctrl->type);
			break;
		}
		p += ctrl->elem_size;
		q += ctrl->elem_size;
	}
	return retval;
}

static int gb_camera_ext_s_or_try_ctrl(struct gb_connection *conn,
		struct v4l2_ctrl *ctrl, int is_try)
{
	int retval = -EINVAL;
	uint32_t data_size;
	struct camera_ext_predefined_ctrl_mod_req *req;
	size_t mod_ctrl_val_size;
	struct camera_ext_v4l2_ctrl_priv *priv = ctrl->priv;
	int op = is_try ? GB_CAMERA_EXT_TYPE_CTRL_TRY
			: GB_CAMERA_EXT_TYPE_CTRL_SET;

	data_size = ctrl->elems * ctrl->elem_size;;
	mod_ctrl_val_size = sizeof(*req) + data_size;
	req = kmalloc(mod_ctrl_val_size, GFP_KERNEL);
	if (req == NULL) {
		pr_err("%s: failed to allocate %zu bytes\n", __func__,
			mod_ctrl_val_size);
		return -ENOMEM;
	}

	req->idx = cpu_to_le32(priv->idx);
	req->data_size = cpu_to_le32(data_size);
	retval = ctrl_val_v4l2_to_mod(ctrl, req->data);
	if (retval == 0)
		retval = gb_operation_sync(conn,
				op,
				req,
				mod_ctrl_val_size,
				NULL,
				0);
	kfree(req);
	return retval;
}

int gb_camera_ext_s_ctrl(struct gb_connection *conn, struct v4l2_ctrl *ctrl)
{
	return gb_camera_ext_s_or_try_ctrl(conn, ctrl, 0);
}

int gb_camera_ext_try_ctrl(struct gb_connection *conn, struct v4l2_ctrl *ctrl)
{
	return gb_camera_ext_s_or_try_ctrl(conn, ctrl, 1);
}

static int gb_error_msg_to_local(uint32_t type, void *data, size_t size,
	struct v4l2_camera_ext_event *event)
{
	struct camera_ext_event_error *err;

	memset(event, 0, sizeof(*event));
	event->type = type;

	if (size != sizeof(*err)) {
		pr_err("%s: invalid error msg\n", __func__);
		return -EINVAL;
	}
	err = (struct camera_ext_event_error *)data;
	event->error_code = le32_to_cpu(err->error_code);

	return 0;
}

static int gb_camera_ext_async_msg_receive(u8 type, struct gb_operation *op)
{
	struct camera_ext_event_hdr *msg_hdr;
	struct v4l2_camera_ext_event event;
	uint32_t msg_type;
	size_t msg_data_size;
	struct camera_ext *cam_dev;
	struct camera_ext_event_metadata *metadata;

	if (type != GB_CAMERA_EXT_ASYNC_MESSAGE) {
		pr_err("%s: unknown request type %u\n", __func__, type);
		return -EINVAL;
	}

	if (op->request->payload_size < sizeof(*msg_hdr)) {
		pr_err("%s: async msg payload too small(%zu < %zu)\n", __func__,
			op->request->payload_size, sizeof(*msg_hdr));
		return -EINVAL;
	}

	cam_dev = op->connection->private;
	if (!cam_dev) {
		pr_err("%s: cam_dev device not initialized\n", __func__);
		return -EAGAIN;
	}

	msg_hdr = op->request->payload;
	msg_type = le32_to_cpu(msg_hdr->type);
	msg_data_size = op->request->payload_size - sizeof(*msg_hdr);

	switch (msg_type) {
	case CAMERA_EXT_REPORT_ERROR:
		if (gb_error_msg_to_local(msg_type, msg_hdr->data,
				msg_data_size, &event) != 0) {
			pr_err("%s: failed to convert msg to v4l2 event\n",
				__func__);
			return -EINVAL;
		}
		camera_ext_mod_v4l2_event_notify(cam_dev, &event);
		break;
	case CAMERA_EXT_REPORT_METADATA:
		if (msg_data_size > sizeof(*metadata)) {
			pr_err("%s: incorrect metadata size %zu != %zu\n",
				__func__, msg_data_size, sizeof(*metadata));
			return -EINVAL;
		}
		metadata = (struct camera_ext_event_metadata *)msg_hdr->data;
		camera_ext_mod_v4l2_buffer_notify(cam_dev,
			metadata->desc, msg_data_size);
		break;
	default:
		pr_err("unsupported event type %d\n", msg_type);
		return -ENOTSUPP;
	}

	return 0;
}

static int gb_camera_ext_connection_init(struct gb_connection *connection)
{
	int retval;
	struct camera_ext *cam;

	cam = kzalloc(sizeof(*cam), GFP_KERNEL);
	if (!cam)
		return -ENOMEM;

	kref_init(&cam->kref);
	cam->connection = connection;
	gb_connection_get(cam->connection);
	cam->state = CAMERA_EXT_READY;

	retval = camera_ext_mod_v4l2_init(cam);
	if (retval) {
		pr_err("failed to init v4l2 for mod control\n");
		gb_connection_put(cam->connection);
		kfree(cam);
		goto exit;
	}

	connection->private = cam;

exit:
	return retval;
}

static void gb_camera_ext_connection_exit(struct gb_connection *connection)
{
	struct camera_ext *cam = connection->private;

	cam->state = CAMERA_EXT_DESTROYED;
	camera_ext_mod_v4l2_exit(cam);
}

static struct gb_protocol camera_ext_protocol = {
	.name			= "camera_ext",
	.id			= GREYBUS_PROTOCOL_CAMERA_EXT,
	.major			= GB_CAMERA_EXT_VERSION_MAJOR,
	.minor			= GB_CAMERA_EXT_VERSION_MINOR,
	.connection_init	= gb_camera_ext_connection_init,
	.connection_exit	= gb_camera_ext_connection_exit,
	.request_recv		= gb_camera_ext_async_msg_receive,
};

static int camera_ext_init(void)
{
	int rc;

	/*
	 * The function below should always success unless there are
	 * duplicate platform devices.
	 */
	rc = camera_ext_v4l2_driver_init();
	if (rc < 0)
		return rc;

	rc = gb_protocol_register(&camera_ext_protocol);
	if (rc < 0)
		camera_ext_v4l2_driver_exit();

	return rc;
}
module_init(camera_ext_init);

static void __exit camera_ext_exit(void)
{
	gb_protocol_deregister(&camera_ext_protocol);
	camera_ext_v4l2_driver_exit();
}
module_exit(camera_ext_exit);

MODULE_LICENSE("GPL v2");
