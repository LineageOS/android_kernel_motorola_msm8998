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
#include <uapi/video/v4l2_camera_ext_ctrls.h>
#include <media/v4l2-ctrls.h>
#include "camera_ext_defs.h" /* CAMERA_EXT_CTRL_FLAG_NEED_XXX flags */

#define CTRL_MAX_INT 0x7FFFFFFF
#define CTRL_MAX_INT64 0x7FFFFFFFFFFFFFFFLL

#define CTRL_STRING_MAX_LEN 64
#define CTRL_CUSTOM_PARAM_MAX_LEN 256

/* camera_ext predefined controls
 * These objects cannot be constant because they will be updated by data from
 * MOD. TODO: to support multiple MODS, need to allocate memory for menu and
 * menu_int from MOD (currently static allocated).
 */

static const u64 color_correction_aberration_mode_items[] = {
	CAM_EXT_COLOR_CORRECTION_ABERRATION_OFF,
	CAM_EXT_COLOR_CORRECTION_ABERRATION_FAST,
	CAM_EXT_COLOR_CORRECTION_ABERRATION_HQ,
};

static struct v4l2_ctrl_config color_correction_aberration_mode = {
	.id = CAM_EXT_CID_COLOR_CORRECTION_ABERRATION_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "color correction aberration mode",
	.max = CAM_EXT_COLOR_CORRECTION_ABERRATION_MAX,
	.qmenu_int = color_correction_aberration_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static const u64 ae_antibanding_mode_items[] = {
	CAM_EXT_AE_ANTIBANDING_OFF,
	CAM_EXT_AE_ANTIBANDING_50HZ,
	CAM_EXT_AE_ANTIBANDING_60HZ,
	CAM_EXT_AE_ANTIBANDING_AUTO,
};

static struct v4l2_ctrl_config ae_antibanding_mode = {
	.id = CAM_EXT_CID_AE_ANTIBANDING_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "ae antibanding mode",
	.max = CAM_EXT_AE_ANTIBANDING_MAX,
	.qmenu_int = ae_antibanding_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config ae_exposure_compensation = {
	.id = CAM_EXT_CID_AE_EXPOSURE_COMPENSATION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "ae exposure compensation",
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_STEP
		| CAMERA_EXT_CTRL_FLAG_NEED_MIN
		| CAMERA_EXT_CTRL_FLAG_NEED_MAX
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config ae_lock = {
	.id = CAM_EXT_CID_AE_LOCK,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "ae lock",
	.min = 0,
	.max = 1,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static const u64 ae_mode_items[] = {
	CAM_EXT_AE_MODE_OFF,
	CAM_EXT_AE_MODE_ON,
	CAM_EXT_AE_MODE_ON_AUTO_FLASH,
	CAM_EXT_AE_MODE_ON_ALWAYS_FLASH,
	CAM_EXT_AE_MODE_ON_AUTO_FLASH_REDEYE,
};

static struct v4l2_ctrl_config ae_mode = {
	.id = CAM_EXT_CID_AE_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "ae mode",
	.max = CAM_EXT_AE_MODE_MAX,
	.qmenu_int = ae_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static s64 ae_available_ranges_items[CAMERA_EXT_MAX_MENU_NUM];

/* int menu */
static struct v4l2_ctrl_config ae_target_fps_range = {
	.id = CAM_EXT_CID_AE_TARGET_FPS_RANGE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "ae target fps range",
	.qmenu_int = ae_available_ranges_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MAX
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_INT,
};

static const u64 af_mode_items[] = {
	CAM_EXT_AF_MODE_OFF,
	CAM_EXT_AF_MODE_AUTO,
	CAM_EXT_AF_MODE_MACRO,
	CAM_EXT_AF_MODE_CONTINUOUS_VIDEO,
	CAM_EXT_AF_MODE_CONTINUOUS_PICTURE,
	CAM_EXT_AF_MODE_EDOF,
};

static struct v4l2_ctrl_config af_mode = {
	.id = CAM_EXT_CID_AF_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "af mode",
	.max = CAM_EXT_AF_MODE_MAX,
	.qmenu_int = af_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static const u64 af_trigger_items[] = {
	CAM_EXT_AF_TRIGGER_IDLE,
	CAM_EXT_AF_TRIGGER_START,
	CAM_EXT_AF_TRIGGER_CANCEL,
};

static struct v4l2_ctrl_config af_trigger = {
	.id = CAM_EXT_CID_AF_TRIGGER,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "af trigger",
	.max = CAM_EXT_AF_TRIGGER_MAX,
	.qmenu_int = af_trigger_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static struct v4l2_ctrl_config awb_lock = {
	.id = CAM_EXT_CID_AWB_LOCK,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "awb lock",
	.max = 1,
	.min = 0,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static const u64 awb_mode_items[] = {
	CAM_EXT_AWB_MODE_OFF,
	CAM_EXT_AWB_MODE_AUTO,
	CAM_EXT_AWB_MODE_INCANDESCENT,
	CAM_EXT_AWB_MODE_FLUORESCENT,
	CAM_EXT_AWB_MODE_DAYLIGHT,
	CAM_EXT_AWB_MODE_CLOUDY_DAYLIGHT,
	CAM_EXT_AWB_MODE_TWILIGHT,
	CAM_EXT_AWB_MODE_SHADE,
};

static struct v4l2_ctrl_config awb_mode = {
	.id = CAM_EXT_CID_AWB_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "awb mode",
	.max = CAM_EXT_AWB_MODE_MAX,
	.qmenu_int = awb_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static const u64 effect_mode_items[] = {
	CAM_EXT_EFFECT_MODE_OFF,
	CAM_EXT_EFFECT_MODE_MONO,
	CAM_EXT_EFFECT_MODE_NEGATIVE,
	CAM_EXT_EFFECT_MODE_SOLARIZE,
	CAM_EXT_EFFECT_MODE_SEPIA,
	CAM_EXT_EFFECT_MODE_POSTERIZE,
	CAM_EXT_EFFECT_MODE_WHITEBOARD,
	CAM_EXT_EFFECT_MODE_BLACKBOARD,
	CAM_EXT_EFFECT_MODE_AQUA,
};

static struct v4l2_ctrl_config effect_mode = {
	.id = CAM_EXT_CID_EFFECT_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "effect mode",
	.max = CAM_EXT_EFFECT_MODE_MAX,
	.qmenu_int = effect_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static const u64 control_mode_items[] = {
	CAM_EXT_CONTROL_MODE_OFF,
	CAM_EXT_CONTROL_MODE_AUTO,
	CAM_EXT_CONTROL_MODE_USE_SCENE_MODE,
	CAM_EXT_CONTROL_MODE_OFF_KEEP_STATE,
};

static struct v4l2_ctrl_config control_mode = {
	.id = CAM_EXT_CID_CONTROL_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "control mode",
	.max = CAM_EXT_CONTROL_MODE_MAX,
	.qmenu_int = control_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static const u64 scene_mode_items[] = {
	CAM_EXT_SCENE_MODE_DISABLED,
	CAM_EXT_SCENE_MODE_FACE_PRIORITY,
	CAM_EXT_SCENE_MODE_ACTION,
	CAM_EXT_SCENE_MODE_PORTRAIT,
	CAM_EXT_SCENE_MODE_LANDSCAPE,
	CAM_EXT_SCENE_MODE_NIGHT,
	CAM_EXT_SCENE_MODE_NIGHT_PORTRAIT,
	CAM_EXT_SCENE_MODE_THEATRE,
	CAM_EXT_SCENE_MODE_BEACH,
	CAM_EXT_SCENE_MODE_SNOW,
	CAM_EXT_SCENE_MODE_SUNSET,
	CAM_EXT_SCENE_MODE_STEADYPHOTO,
	CAM_EXT_SCENE_MODE_FIREWORKS,
	CAM_EXT_SCENE_MODE_SPORTS,
	CAM_EXT_SCENE_MODE_PARTY,
	CAM_EXT_SCENE_MODE_CANDLELIGHT,
	CAM_EXT_SCENE_MODE_BARCODE,
	CAM_EXT_SCENE_MODE_HDR,
};

static struct v4l2_ctrl_config scene_mode = {
	.id = CAM_EXT_CID_SCENE_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "scene mode",
	.max = CAM_EXT_SCENE_MODE_MAX,
	.qmenu_int = scene_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static const s64 video_stabilization_items[] = {
	CAM_EXT_VIDEO_STABILIZATION_MODE_ON,
	CAM_EXT_VIDEO_STABILIZATION_MODE_OFF,
};

static struct v4l2_ctrl_config video_stabilization = {
	.id = CAM_EXT_CID_VIDEO_STABILIZATION_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "video stabilization",
	.max = CAM_EXT_VIDEO_STABILIZATION_MODE_MAX,
	.qmenu_int = video_stabilization_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

/* double [3] */
static struct v4l2_ctrl_config jpeg_gps_location = {
	.id = CAM_EXT_CID_JPEG_GPS_LOCATION,
	.type = V4L2_CTRL_TYPE_STRING,
	/* v4l2 will use max+1 for elem_size */
	.max = CAM_EXT_CTRL_DOUBLE_STR_LEN - 1,
	.name = "jpeg gps location",
	.dims = {3},
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
	/* no default value needed from MOD */
};

static const s64 jpeg_orientation_items[] = {
	CAM_EXT_JPEG_ORIENTATION_0,
	CAM_EXT_JPEG_ORIENTATION_90,
	CAM_EXT_JPEG_ORIENTATION_180,
	CAM_EXT_JPEG_ORIENTATION_270,
};

static struct v4l2_ctrl_config jpeg_orientation = {
	.id = CAM_EXT_CID_JPEG_ORIENTATION,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "jpeg orientation",
	.max = CAM_EXT_JPEG_ORIENTATION_MAX,
	.qmenu_int = jpeg_orientation_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static struct v4l2_ctrl_config jpeg_quality = {
	.id = CAM_EXT_CID_JPEG_QUALITY,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "jpeg quality",
	.min = 0,
	.max = 100,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config manual_focus_position = {
	.id = CAM_EXT_CID_MANUAL_FOCUS_POSITION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "manual focus position",
	.min = 0,
	.max = 100,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MIN
		| CAMERA_EXT_CTRL_FLAG_NEED_MAX,
};

static struct v4l2_ctrl_config lens_facing = {
	.id = CAM_EXT_CID_LENS_FACING,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "lens facing",
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| V4L2_CTRL_FLAG_READ_ONLY,
	.max = CAM_EXT_LENS_FACING_MAX,
	.step = 1,
};

static const s64 flash_mode_items[] = {
	CAM_EXT_FLASH_MODE_OFF,
	CAM_EXT_FLASH_MODE_SINGLE,
	CAM_EXT_FLASH_MODE_TORCH,
};

static struct v4l2_ctrl_config flash_mode = {
	.id = CAM_EXT_CID_FLASH_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "flash mode",
	.max = CAM_EXT_FLASH_MODE_MAX,
	.qmenu_int = flash_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

/* allocate enough space to hold menu items from MOD */
static camera_ext_ctrl_float focal_length_items_mem[CAMERA_EXT_MAX_MENU_NUM];

/* Must have one item at least, so focal_length_items[0] always point
 * to focal_length_items_mem.
 *
 * Camera_ext mod v4l2 driver will store float data (in string) into
 * focal_length_items_mem, then update focal_length_items to have a NULL after
 * last valid item.
 */
static const char *focal_length_items[CAMERA_EXT_MAX_MENU_NUM + 1] = {
        focal_length_items_mem[0],
};

/* float menu, floats from MOD */
static struct v4l2_ctrl_config focal_length = {
	.id = CAM_EXT_CID_FOCAL_LENGTH,
	.type = V4L2_CTRL_TYPE_MENU,
	.name = "focal length",
	.qmenu = focal_length_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MAX
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static const s64 capabilities_items[] = {
	CAM_EXT_CAPABILITIES_BACKWARD_COMPATIBLE,
	CAM_EXT_CAPABILITIES_MANUAL_SENSOR,
	CAM_EXT_CAPABILITIES_POST_PROCESSING,
	CAM_EXT_CAPABILITIES_RAW,
	CAM_EXT_CAPABILITIES_PRIVATE_REPROCESSING,
	CAM_EXT_CAPABILITIES_READ_SENSOR_SETTING,
	CAM_EXT_CAPABILITIES_BURST_CAPTURE,
	CAM_EXT_CAPABILITIES_YUV_REPROCESSING,
	CAM_EXT_CAPABILITIES_DEPTH_OUTPUT,
	CAM_EXT_CAPABILITIES_CONSTRAINED_HIGHT_SPEED_VIDEO,
};

static struct v4l2_ctrl_config capabilities = {
	.id = CAM_EXT_CID_CAPABILITIES,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "capabilities",
	.qmenu_int = capabilities_items,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
	.max = CAM_EXT_CAPABILITIES_MAX,
};

static struct v4l2_ctrl_config max_num_output_proc = {
	.id = CAM_EXT_CID_MAX_NUM_OUTPUT_PROC,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "max num output proc",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
	.max = CTRL_MAX_INT, /*legacy at least 2, full >= 3, limited >=2 */
	.step = 1,
};

static struct v4l2_ctrl_config max_num_output_proc_stalling = {
	.id = CAM_EXT_CID_MAX_NUM_OUTPUT_PROC_STALLING,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "max num output proc stalling",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
	.max = 1, /* legacy supports 1 */
	.step = 1,
};

static struct v4l2_ctrl_config max_num_output_raw = {
	.id = CAM_EXT_CID_MAX_NUM_OUTPUT_RAW,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "max num output raw",
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config pipleline_max_depth = {
	.id = CAM_EXT_CID_PIPLELINE_MAX_DEPTH,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "pipleline max depth",
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config scaler_max_digital_zoom = {
	.id = CAM_EXT_CID_SCALER_MAX_DIGITAL_ZOOM,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "scaler max digital zoom",
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
};

static const s64 scaler_cropping_type_items[] = {
	CAM_EXT_SCALER_CROPPING_TYPE_CENTER_ONLY,
	CAM_EXT_SCALER_CROPPING_TYPE_FREEFORM,
};

static struct v4l2_ctrl_config scaler_cropping_type = {
	.id = CAM_EXT_CID_SCALER_CROPPING_TYPE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "scaler cropping type",
	.max = CAM_EXT_SCALER_CROPPING_TYPE_MAX,
	.qmenu_int = scaler_cropping_type_items,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static struct v4l2_ctrl_config scaler_crop_region = {
	.id = CAM_EXT_CID_SCALER_CROP_REGION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "scaler crop region",
	.dims = {4},
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

/* float [2] */
static struct v4l2_ctrl_config sensor_info_physical_size = {
	.id = CAM_EXT_CID_SENSOR_INFO_PHYSICAL_SIZE,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "sensor info physical size",
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.dims = {2},
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
};

static struct v4l2_ctrl_config sensor_info_pixel_array_size = {
	.id = CAM_EXT_CID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "sensor info pixel array size",
	.dims = {2},
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config sensor_info_pre_correction_active_array_size = {
	.id = CAM_EXT_CID_SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "sensor info pre-correction active array size",
	.dims = {4},
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config sensor_info_active_array_size = {
	.id = CAM_EXT_CID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "sensor info active array size",
	.dims = {4},
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config sensor_info_timestamp_source = {
	.id = CAM_EXT_CID_SENSOR_INFO_TIMESTAMP_SOURCE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "sensor info timestamp source",
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config sensor_orientation = {
	.id = CAM_EXT_CID_SENSOR_ORIENTATION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "sensor orientation",
	.max = CAM_EXT_SENSOR_ORIENTATION_MAX,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static const s64 statistics_face_detect_mode_items[] = {
	CAM_EXT_STATISTICS_FACE_DETECT_MODE_OFF,
	CAM_EXT_STATISTICS_FACE_DETECT_MODE_SIMPLE,
	CAM_EXT_STATISTICS_FACE_DETECT_MODE_FULL,
};

static struct v4l2_ctrl_config statistics_face_detect_mode = {
	.id = CAM_EXT_CID_STATISTICS_FACE_DETECT_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "statistics face detect mode",
	.qmenu_int = statistics_face_detect_mode_items,
	.max = CAM_EXT_STATISTICS_FACE_DETECT_MODE_MAX,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config statistics_info_max_face_count = {
	.id = CAM_EXT_CID_STATISTICS_INFO_MAX_FACE_COUNT,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "statistics info max face count",
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config sync_max_latency = {
	.id = CAM_EXT_CID_SYNC_MAX_LATENCY,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "sync max latency",
	.max = CAM_EXT_SYNC_MAX_LATENCY_MAX,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static const s64 control_ae_precapture_trigger_items[] = {
	CAM_EXT_CONTROL_AE_PRECATURE_TRIGGER_IDLE,
	CAM_EXT_CONTROL_AE_PRECATURE_TRIGGER_START,
	CAM_EXT_CONTROL_AE_PRECATURE_TRIGGER_CANCEL,
};

static struct v4l2_ctrl_config control_ae_precapture_trigger = {
	.id = CAM_EXT_CID_CONTROL_AE_PRECATURE_TRIGGER,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "ae precapture trigger",
	.max = CAM_EXT_CONTROL_AE_PRECATURE_TRIGGER_MAX,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config lens_info_focus_distance_calibration = {
	.id = CAM_EXT_CID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "lens info focus distance calibration",
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

/* float */
static struct v4l2_ctrl_config lens_info_focus_hyperfocal_distance = {
	.id = CAM_EXT_CID_LENS_INFO_FOCUS_HYPERFOCAL_DISTANCE,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "lens info focus hyperfocal distance",
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
};

/* float */
static struct v4l2_ctrl_config lens_info_minimum_focus_distance = {
	.id = CAM_EXT_CID_LENS_INFO_MINIMUM_FOCUS_DISTANCE,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "lens info minimum focus distance",
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
};

/* float */
static struct v4l2_ctrl_config lens_focus_distance = {
	.id = CAM_EXT_CID_LENS_FOCUS_DISTANCE,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "lens focus distance",
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
};

static const s64 lens_optical_stabilization_mode_items[] = {
	CAM_EXT_CID_LENS_OPTICAL_STABILIZATION_MODE_OFF,
	CAM_EXT_CID_LENS_OPTICAL_STABILIZATION_MODE_ON,
};

static struct v4l2_ctrl_config lens_optical_stabilization_mode = {
	.id = CAM_EXT_CID_LENS_OPTICAL_STABILIZATION_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.qmenu_int = lens_optical_stabilization_mode_items,
	.name = "lens optical stabilization mode",
	.max = CAM_EXT_CID_LENS_OPTICAL_STABILIZATION_MODE_MAX,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

/* float */
static struct v4l2_ctrl_config reprocess_effective_exposure_factore = {
	.id = CAM_EXT_CID_REPROCESS_EFFECTIVE_EXPOSURE_FACTOR,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "reprocess effective exposure factore",
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
};

/* integer read only */
static struct v4l2_ctrl_config reprocess_max_capture_stall = {
	.id = CAM_EXT_CID_REPROCESS_MAX_CAPTURE_STALL,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "reprocess max capture stall",
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

/* boolean */
static struct v4l2_ctrl_config depth_depth_is_exclusive = {
	.id = CAM_EXT_CID_DEPTH_DEPTH_IS_EXCLUSIVE,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "depth depth is exclusive",
	.max = 1,
	.min = 0,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

/* boolean */
static struct v4l2_ctrl_config black_level_lock = {
	.id = CAM_EXT_CID_BLACK_LEVEL_LOCK,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "black level lock",
	.min = 0,
	.max = 1,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

/* int */
static struct v4l2_ctrl_config color_correction_mode = {
	.id = CAM_EXT_CID_COLOR_CORRECTION_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "color correction mode",
	.max = CAM_EXT_COLOR_CORRECTION_MODE_MAX,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

/* float [4] RGGB */
static struct v4l2_ctrl_config color_correction_gains = {
	.id = CAM_EXT_CID_COLOR_CORRECTION_GAINS,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "color correction gains",
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.dims = {4},
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
};

/* float [3][3] */
static struct v4l2_ctrl_config color_correction_transform = {
	.id = CAM_EXT_CID_COLOR_CORRECTION_TRANSFORM,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "color correction transform",
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN -  1,
	.step = 1,
	.dims = {3, 3},
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
};

static const s64 edge_mode_items[] = {
	CAM_EXT_EDGE_MODE_OFF,
	CAM_EXT_EDGE_MODE_FAST,
	CAM_EXT_EDGE_MODE_HIGH_QUALITY,
	CAM_EXT_EDGE_MODE_ZERO_SHUTTER_LAG,
};

static struct v4l2_ctrl_config edge_mode = {
	.id = CAM_EXT_CID_EDGE_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "edge mode",
	.qmenu_int = edge_mode_items,
	.max = CAM_EXT_EDGE_MODE_MAX,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

/* allocate enough space for menu items from MOD */
static camera_ext_ctrl_float lens_apertures_mem[CAMERA_EXT_MAX_MENU_NUM];

static const char *lens_apertures_items[CAMERA_EXT_MAX_MENU_NUM + 1] = {
	lens_apertures_mem[0],
};

static struct v4l2_ctrl_config lens_apertures = {
	.id = CAM_EXT_CID_LENS_APERTURES,
	.type = V4L2_CTRL_TYPE_MENU,
	.name = "lens apertures",
	.qmenu = lens_apertures_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MAX,
};

/* allocate enough space for menu items from MOD */
static camera_ext_ctrl_float lens_filter_density_mem[CAMERA_EXT_MAX_MENU_NUM];

static const char *lens_filter_density_items[CAMERA_EXT_MAX_MENU_NUM + 1] = {
	lens_filter_density_mem[0],
};

static struct v4l2_ctrl_config lens_filter_density = {
	.id = CAM_EXT_CID_LENS_FILTER_DENSITY,
	.type = V4L2_CTRL_TYPE_MENU,
	.name = "lens filter density",
	.qmenu = lens_filter_density_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MAX,
};

static const s64 noise_reduction_mode_items[] = {
	CAM_EXT_NOISE_REDUCTION_MODE_OFF,
	CAM_EXT_NOISE_REDUCTION_MODE_FAST,
	CAM_EXT_NOISE_REDUCTION_MODE_HIGH_QUALITY,
	CAM_EXT_NOISE_REDUCTION_MODE_MINIMAL,
	CAM_EXT_NOISE_REDUCTION_MODE_ZERO_SHUTTER_LAG,
};

static struct v4l2_ctrl_config noise_reduction_mode = {
	.id = CAM_EXT_CID_NOISE_REDUCTION_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "noise reduction mode",
	.qmenu_int = noise_reduction_mode_items,
	.max = CAM_EXT_NOISE_REDUCTION_MODE_MAX,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config request_max_num_input_stream = {
	.id = CAM_EXT_CID_REQUEST_MAX_NUM_INPUT_STREAM,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "request max num input stream",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
	.max = CTRL_MAX_INT,
	.step = 1,
};

static struct v4l2_ctrl_config rquest_partial_result_count = {
	.id = CAM_EXT_CID_REQUEST_PARTIAL_RESULT_COUNT,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "rquest partial result count",
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config sensor_exposure_time = {
	.id = CAM_EXT_CID_SENSOR_EXPOSURE_TIME,
	.type = V4L2_CTRL_TYPE_INTEGER64,
	.name = "sensor exposure time",
	.max = CTRL_MAX_INT64,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MIN
		| CAMERA_EXT_CTRL_FLAG_NEED_MAX
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config sensor_frame_duration = {
	.id = CAM_EXT_CID_SENSOR_FRAME_DURATION,
	.type = V4L2_CTRL_TYPE_INTEGER64,
	.name = "sensor frame duration",
	.max = CTRL_MAX_INT64,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MAX
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config sensor_sensitivity = {
	.id = CAM_EXT_CID_SENSOR_SENSITIVITY,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "sensor sensitivity",
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
	.max = CTRL_MAX_INT,
	.step = 1,
};

static struct v4l2_ctrl_config sensor_info_color_filter_arrangement = {
	.id = CAM_EXT_CID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "sensor info color filter arrangement",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
	.max = CTRL_MAX_INT,
	.step = 1,
};

static struct v4l2_ctrl_config sensor_max_analog_sensitivity = {
	.id = CAM_EXT_CID_SENSOR_MAX_ANALOG_SENSITIVITY,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "sensor max analog sensitivity",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
	.max = CTRL_MAX_INT,
	.step = 1,
};

static const s64 shading_mode_items[] = {
	CAM_EXT_SHADING_MODE_OFF,
	CAM_EXT_SHADING_MODE_FAST,
	CAM_EXT_SHADING_MODE_HIGH_QUALITY,
};

static struct v4l2_ctrl_config shading_mode = {
	.id = CAM_EXT_CID_SHADING_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "shading mode",
	.qmenu_int = shading_mode_items,
	.max = CAM_EXT_SHADING_MODE_MAX,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static const s64 statistics_lens_shading_map_mode_items[] = {
	CAM_EXT_CID_STATISTICS_LENS_SHADING_MAP_MODE_ON,
	CAM_EXT_CID_STATISTICS_LENS_SHADING_MAP_MODE_OFF,
};

static struct v4l2_ctrl_config statistics_lens_shading_map_mode = {
	.id = CAM_EXT_CID_STATISTICS_LENS_SHADING_MAP_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "statistics lens shading map mode",
	.max = CAM_EXT_CID_STATISTICS_LENS_SHADING_MAP_MODE_MAX,
	.qmenu_int = statistics_lens_shading_map_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

/* array of floats, dimentions are from MOD */
static struct v4l2_ctrl_config tonemap_curve = {
	.id = CAM_EXT_CID_TONEMAP_CURVE,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "tonemap curve",
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	/* MOD should have the dims = {2, 3, POINT_NUM}, */
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
};

/* float */
static struct v4l2_ctrl_config tonemap_gamma = {
	.id = CAM_EXT_CID_TONEMAP_GAMMA,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "tonemap gamma",
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN -  1,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
};

static const s64 tonemap_mode_items[] = {
	CAM_EXT_TONEMAP_MODE_CONTRAST_CURVE,
	CAM_EXT_TONEMAP_MODE_FAST,
	CAM_EXT_TONEMAP_MODE_HIGH_QUALITY,
	CAM_EXT_TONEMAP_MODE_GAMMA_VALUE,
	CAM_EXT_TONEMAP_MODE_PRESET_CURVE,
};

static struct v4l2_ctrl_config tonemap_mode = {
	.id = CAM_EXT_CID_TONEMAP_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "tonemap mode",
	.max = CAM_EXT_TONEMAP_MODE_MAX,
	.qmenu_int = tonemap_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config tonemap_preset_curve = {
	.id = CAM_EXT_CID_TONEMAP_PRESET_CURVE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "tonemap preset curve",
	.step = 1,
	.max = CAM_EXT_TONEMAP_PRESET_CURVE_MAX,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

/* integer */
static struct v4l2_ctrl_config tonemap_max_curve_points = {
	.id = CAM_EXT_CID_TONEMAP_MAX_CURVE_POINSTS,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "tonemap max curve points",
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config ae_regions = {
	.id = CAM_EXT_CID_AE_REGIONS,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "ae regions",
	/* .dims = {4, MAX_AE_REGIONS}, */
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
};

static struct v4l2_ctrl_config af_regions = {
	.id = CAM_EXT_CID_AF_REGIONS,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "af regions",
	.max = CTRL_MAX_INT,
	.step = 1,
	/* MOD should have the dims {4, MAX_AF_REGIONS}, */
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
};

static struct v4l2_ctrl_config awb_regions = {
	.id = CAM_EXT_CID_AWB_REGIONS,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "awb regions",
	.max = CTRL_MAX_INT,
	.step = 1,
	/* MOD should have the dims {4, MAX_AWB_REGIONS}, */
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
};

static const s64 hot_pixel_mode_items[] = {
	CAM_EXT_HOT_PIXEL_MODE_OFF,
	CAM_EXT_HOT_PIXEL_MODE_FAST,
	CAM_EXT_HOT_PIXEL_MODE_HIGH_QUALITY,
};

static struct v4l2_ctrl_config hot_pixel_mode = {
	.id = CAM_EXT_CID_HOT_PIXEL_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "hot pixel mode",
	.max = CAM_EXT_HOT_PIXEL_MODE_MAX,
	.qmenu_int = hot_pixel_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

/* float[5] f_x, f_y, c_x, c_y, s */
static struct v4l2_ctrl_config lens_intrinsic_calibration = {
	.id = CAM_EXT_CID_LENS_INTRINSIC_CALIBRATION,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "lens intrinsic calibration",
	.dims = {5},
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
};

/* float [7] x, y, z, w, a_x, a_y, a_z */
static struct v4l2_ctrl_config lens_pos_rotation = {
	.id = CAM_EXT_CID_LENS_POSE_ROTATION,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "lens pos rotation",
	.dims = {7},
	/* TODO: READ ONLY MUST HAVE DEF FROM MOD
	 * But for array data, need extra work to init control.
	 */
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
};

/* float [3] x, y, z */
static struct v4l2_ctrl_config lens_pos_translation = {
	.id = CAM_EXT_CID_LENS_POSE_TRANSLATION,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "lens pos translation",
	.dims = {3},
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
};

/* float [6] kappa_0, kappa_1, kappa_2, kappa_3, kappa_4, kappa_5 */
static struct v4l2_ctrl_config lens_radial_distortion = {
	.id = CAM_EXT_CID_LENS_RADIAL_DISTORTION,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "lens radial distortion",
	.dims = {6},
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
	.step = 1,
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
};

static struct v4l2_ctrl_config sensor_test_pattern_data = {
	.id = CAM_EXT_CID_SENSOR_TEST_PATTERN_DATA,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.max = CTRL_MAX_INT,
	.step = 1,
	.name = "sensor test pattern data",
	.dims = {4},
};

static const s64 sensor_test_pattern_mode_items[] = {
	CAM_EXT_SENSOR_TEST_PATTERN_MODE_OFF,
	CAM_EXT_SENSOR_TEST_PATTERN_MODE_SOLID_COLOR,
	CAM_EXT_SENSOR_TEST_PATTERN_MODE_COLOR_BARS,
	CAM_EXT_SENSOR_TEST_PATTERN_MODE_COLOR_BARS_FADE_TO_GRAY,
	CAM_EXT_SENSOR_TEST_PATTERN_MODE_PN9,
	CAM_EXT_SENSOR_TEST_PATTERN_MODE_CUSTOM1,
};

static struct v4l2_ctrl_config sensor_test_pattern_mode = {
	.id = CAM_EXT_CID_SENSOR_TEST_PATTERN_MODE,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "sensor test pattern mode",
	.max = CAM_EXT_SENSOR_TEST_PATTERN_MODE_MAX,
	.qmenu_int = sensor_test_pattern_mode_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static struct v4l2_ctrl_config sensor_black_level_pattern = {
	.id = CAM_EXT_CID_SENSOR_BLACK_LEVEL_PATTERN,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "sensor black level pattern",
	.max = CTRL_MAX_INT,
	.step = 1,
	.dims = {4},
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

/* float [3][3] */
static struct v4l2_ctrl_config sensor_calibration_transform1 = {
	.id = CAM_EXT_CID_SENSOR_CALIBRATION_TRANSFORM1,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "sensor calibration transform1",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
	.step = 1,
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.dims = {3, 3},
};

/* float [3][3] */
static struct v4l2_ctrl_config sensor_calibration_transform2 = {
	.id = CAM_EXT_CID_SENSOR_CALIBRATION_TRANSFORM2,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "sensor calibration transform2",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.dims = {3, 3},
};

/* float [3][3] */
static struct v4l2_ctrl_config sensor_color_transform1 = {
	.id = CAM_EXT_CID_SENSOR_COLOR_TRANSFORM1,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "sensor color transform1",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.dims = {3, 3},
};

/* float [3][3] */
static struct v4l2_ctrl_config sensor_color_transform2 = {
	.id = CAM_EXT_CID_SENSOR_COLOR_TRANSFORM2,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "sensor color transform2",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.dims = {3, 3},
};

/* float [3][3] */
static struct v4l2_ctrl_config sensor_forward_matrix1 = {
	.id = CAM_EXT_CID_SENSOR_FORWARD_MATRIX1,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "sensor forward matrix1",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.dims = {3, 3},
};

/* float [3][3] */
static struct v4l2_ctrl_config sensor_forward_matrix2 = {
	.id = CAM_EXT_CID_SENSOR_FORWARD_MATRIX2,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "sensor forward matrix2",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.dims = {3, 3},
};

static struct v4l2_ctrl_config sensor_info_lens_shading_applied = {
	.id = CAM_EXT_CID_SENSOR_INFO_LENS_SHADING_APPLIED,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "sensor info lens shading applied",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
	.min = 0,
	.max = 1,
	.step = 1,
};

static struct v4l2_ctrl_config sensor_info_white_level = {
	.id = CAM_EXT_CID_SENSOR_INFO_WHITE_LEVEL,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.max = CTRL_MAX_INT,
	.step = 1,
	.name = "sensor info white level",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config sensor_preference_illuminant1 = {
	.id = CAM_EXT_CID_SENSOR_PREFERENCE_ILLUMINANT1,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.max = CTRL_MAX_INT,
	.step = 1,
	.name = "sensor preference illuminant1",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config sensor_preference_illuminant2 = {
	.id = CAM_EXT_CID_SENSOR_PREFERENCE_ILLUMINANT2,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.max = CTRL_MAX_INT,
	.step = 1,
	.name = "sensor preference illuminant2",
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config statistics_hot_pixel_map_mode = {
	.id = CAM_EXT_CID_STATISTICS_HOT_PIXEL_MAP_MODE,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "statistics hot pixel map mode",
	.min = 0,
	.max = 1,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config hot_pixel_map = {
	.id = CAM_EXT_CID_HOT_PIXEL_MAP,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.max = CTRL_MAX_INT,
	.step = 1,
	.name = "hot pixel map",
	/* MOD should have the dims {2, NUM_HOT_PIXELS} */
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
};

static const u64 capture_items[] = {
	CAM_EXT_CID_CAPTURE_STILL_CAPTURE,
	CAM_EXT_CID_CAPTURE_VIDEO_SNAPSHOT,
	CAM_EXT_CID_CAPTURE_ZSL_CAPTURE,
	CAM_EXT_CID_CAPTURE_RAW,
	CAM_EXT_CID_CAPTURE_JPG,
	CAM_EXT_CID_CAPTURE_BURST,
};

static struct v4l2_ctrl_config start_capture = {
	.id = CAM_EXT_CID_CAPTURE,
	.type = V4L2_CTRL_TYPE_BITMASK,
	.name = "start capture",
	.max = CAM_EXT_CID_CAPTURE_MAX,
};

static const u64 af_mode_ext_items[] = {
	CAM_EXT_AF_MODE_EXT_NULL,
	CAM_EXT_AF_MODE_EXT_INFINITY,
	CAM_EXT_AF_MODE_EXT_FIXED,
};

static struct v4l2_ctrl_config af_mode_ext = {
	.id = CAM_EXT_CID_AF_MODE_EXT,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "af mode extenstion",
	.max = CAM_EXT_AF_MODE_EXT_MAX,
	.qmenu_int = af_mode_ext_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static const u64 iso_items[] = {
	CAM_EXT_ISO_AUTO,
	CAM_EXT_ISO_50,
	CAM_EXT_ISO_100,
	CAM_EXT_ISO_200,
	CAM_EXT_ISO_400,
	CAM_EXT_ISO_800,
	CAM_EXT_ISO_1600,
	CAM_EXT_ISO_3200,
};

static struct v4l2_ctrl_config iso = {
	.id = CAM_EXT_CID_ISO,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "iso",
	.max = CAM_EXT_ISO_MAX,
	.qmenu_int = iso_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static const u64 nd_filter_items[] = {
	CAM_EXT_ND_FILTER_AUTO,
	CAM_EXT_ND_FILTER_ON,
	CAM_EXT_ND_FILTER_OFF,
};

static struct v4l2_ctrl_config nd_filter = {
	.id = CAM_EXT_CID_ND_FILTER,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "neutral desity filter",
	.max = CAM_EXT_ND_FILTER_MAX,
	.qmenu_int = nd_filter_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config jpeg_sharpness = {
	.id = CAM_EXT_CID_JPEG_SHARPNESS,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "sharpness tunning for jpeg",
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
	.min = -2,
	.max = 2,
	.step = 1,
	.def = 0,
};

static struct v4l2_ctrl_config jpeg_contrast = {
	.id = CAM_EXT_CID_JPEG_CONTRAST,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "contrast tunning for jpeg",
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
	.min = -2,
	.max = 2,
	.step = 1,
	.def = 0,
};

static struct v4l2_ctrl_config jpeg_saturation = {
	.id = CAM_EXT_CID_JPEG_SATURATION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "saturation tunning for jpeg",
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
	.min = -2,
	.max = 2,
	.step = 1,
	.def = 0,
};

static struct v4l2_ctrl_config time_sync = {
	.id = CAM_EXT_CID_TIME_SYNC,
	.type = V4L2_CTRL_TYPE_INTEGER64,
	.name = "time sync",
	.min = 0,
	.max = CTRL_MAX_INT64,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MIN
		| CAMERA_EXT_CTRL_FLAG_NEED_MAX
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config jpeg_gps_timestamp = {
	.id = CAM_EXT_CID_JPEG_GPS_TIMESTAMP,
	.type = V4L2_CTRL_TYPE_INTEGER64,
	.name = "jpeg gps timestamp",
	.min = 0,
	.max = CTRL_MAX_INT64,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MIN
		| CAMERA_EXT_CTRL_FLAG_NEED_MAX
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config jpeg_gps_proc_method = {
	.id = CAM_EXT_CID_JPEG_GPS_PROC_METHOD,
	.type = V4L2_CTRL_TYPE_STRING,
	.max = CTRL_STRING_MAX_LEN - 1,
	.name = "jpeg gps location proc method",
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static const u64 face_detection_items[] = {
	CAM_EXT_CID_FACE_DETECTION_STOP,
	CAM_EXT_CID_FACE_DETECTION_START,
};

static struct v4l2_ctrl_config face_detection = {
	.id = CAM_EXT_CID_FACE_DETECTION,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "face detection operation",
	.max = CAM_EXT_ND_FILTER_MAX,
	.qmenu_int = face_detection_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config uvc_snapshot = {
	.id = CAM_EXT_CID_MOD_CAPS_UVC_SNAPSHOT,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "uvc snapshot",
	.dims = {3},
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config meta_data_path = {
	.id = CAM_EXT_CID_MOD_META_DATA_PATH,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "meta data path",
	.min = 0,
	.max = CAM_EXT_CID_MOD_META_DATA_PATH_MAX,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config meta_data_size = {
	.id = CAM_EXT_CID_MOD_META_DATA_SIZE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "meta data size",
	.min = 0,
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config zoom_lock_1x = {
	.id = CAM_EXT_CID_ZOOM_LOCK_1X,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "zoom lock 1x",
	.max = 1,
	.min = 0,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config shutter_lock = {
	.id = CAM_EXT_CID_SHUTTER_LOCK,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "shutter lock",
	.min = 0,
	.max = 1,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config model_number = {
	.id = CAM_EXT_CID_MODEL_NUMBER,
	.type = V4L2_CTRL_TYPE_STRING,
	.max = CTRL_STRING_MAX_LEN - 1,
	.name = "model number",
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config firmware_version = {
	.id = CAM_EXT_CID_FIRMWARE_VERSION,
	.type = V4L2_CTRL_TYPE_STRING,
	.max = CTRL_STRING_MAX_LEN - 1,
	.name = "firmware version",
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| V4L2_CTRL_FLAG_VOLATILE
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static const u64 ae_mode_ext_items[] = {
	CAM_EXT_AE_MODE_EXT_NULL,
	CAM_EXT_AE_MODE_EXT_OFF_FLASH,
	CAM_EXT_AE_MODE_EXT_OFF_FLASH_REDEYE,
};

static struct v4l2_ctrl_config ae_mode_ext = {
	.id = CAM_EXT_CID_AE_MODE_EXT,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "ae mode extension",
	.max = CAM_EXT_AE_MODE_EXT_MAX,
	.qmenu_int = ae_mode_ext_items,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static const u64 scene_mode_items_ext[] = {
	CAM_EXT_SCENE_MODE_EXT_NULL,
	CAM_EXT_SCENE_MODE_EXT_AUTO_HDR,
	CAM_EXT_SCENE_MODE_EXT_BACKLIGHT_PORTRAIT,
	CAM_EXT_SCENE_MODE_EXT_CLOSEUP,
	CAM_EXT_SCENE_MODE_EXT_DUSK_DAWN,
	CAM_EXT_SCENE_MODE_EXT_FOOD,
	CAM_EXT_SCENE_MODE_EXT_NIGHT_LANDSCAPE,
	CAM_EXT_SCENE_MODE_EXT_PET_PORTRAIT,
};

static struct v4l2_ctrl_config scene_mode_ext = {
	.id = CAM_EXT_CID_SCENE_MODE_EXT,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "scene mode extension",
	.max = CAM_EXT_SCENE_MODE_EXT_MAX,
	.qmenu_int = scene_mode_items_ext,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static struct v4l2_ctrl_config zoom_limit = {
	.id = CAM_EXT_CID_ZOOM_LIMIT,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "zoom limit",
	.min = 100,
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config focus_key_lock = {
	.id = CAM_EXT_CID_FOCUS_KEY_LOCK,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "focus key lock",
	.min = 0,
	.max = 1,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config eis_frmsize_map = {
	.id = CAM_EXT_CID_EIS_FRAME_SIZE_MAP,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "eis frame size map",
	.max = CTRL_MAX_INT,
	.step = 1,
	/* MOD should have dims {4, EIS_FRAME_SIZE_NUM} */
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_DIMS
		| V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config jpeg_available_thumbnail_sizes = {
	.id = CAM_EXT_CID_JPEG_AVAILABLE_THUMBNAIL_SIZES,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "jpeg thumbnail size",
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_DIMS
		| V4L2_CTRL_FLAG_READ_ONLY,
};

static struct v4l2_ctrl_config jpeg_thumbnail_size_index = {
	.id = CAM_EXT_CID_JPEG_THUMBNAIL_SIZE_INDEX,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "jpeg thumbnail size index",
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config phone_version = {
	.id = CAM_EXT_CID_PHONE_VERSION,
	.type = V4L2_CTRL_TYPE_STRING,
	.max = CTRL_STRING_MAX_LEN - 1,
	.name = "phone version",
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config supplemental_key_mask = {
	.id = CAM_EXT_CID_SUPPLEMENTAL_KEY_MASK,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "supplemental key mask",
	.max = CAM_EXT_HW_KEY_MAX,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config group_ind = {
	.id = CAM_EXT_CID_GROUP_IND,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "group indicator",
	.min = CAM_EXT_CID_GROUP_IND_BEGIN,
	.max = CAM_EXT_CID_GROUP_IND_MAX,
	.step = 1,
};

static struct v4l2_ctrl_config video_record_hint = {
	.id = CAM_EXT_CID_VIDEO_RECORD_HINT,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "video record hint",
	.min = 0,
	.max = 1,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config raw_to_yuv_gain = {
	.id = CAM_EXT_CID_RAW_TO_YUV_GAIN,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "raw to yuv conversion gain",
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN - 1,
	.step = 1,
	.dims = {3},
	.flags = V4L2_CTRL_FLAG_READ_ONLY
			| CAMERA_EXT_CTRL_FLAG_NEED_DEF
			| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
};

static const u64 effect_mode_items_ext[] = {
	CAM_EXT_EFFECT_MODE_EXT_NULL,
	CAM_EXT_EFFECT_MODE_EXT_BLACK_BLUE,
	CAM_EXT_EFFECT_MODE_EXT_PURE,
	CAM_EXT_EFFECT_MODE_EXT_MIRROR,
	CAM_EXT_EFFECT_MODE_EXT_BUBBLE,
	CAM_EXT_EFFECT_MODE_EXT_NEON,
	CAM_EXT_EFFECT_MODE_EXT_CARTOON,
	CAM_EXT_EFFECT_MODE_EXT_SOFT,
	CAM_EXT_EFFECT_MODE_EXT_DIORAMA,

};

static struct v4l2_ctrl_config effect_mode_ext = {
	.id = CAM_EXT_CID_EFFECT_MODE_EXT,
	.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	.name = "effect mode extension",
	.max = CAM_EXT_EFFECT_MODE_EXT_MAX,
	.qmenu_int = effect_mode_items_ext,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
};

static struct v4l2_ctrl_config zsl_buffer_depth = {
	.id = CAM_EXT_CID_ZSL_BUFFER_DEPTH,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "zsl buffer depth",
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_STEP
		| CAMERA_EXT_CTRL_FLAG_NEED_MIN
		| CAMERA_EXT_CTRL_FLAG_NEED_MAX
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config custom_parameter = {
	.id = CAM_EXT_CID_CUSTOM_PARAMETER,
	.type = V4L2_CTRL_TYPE_STRING,
	.max = CTRL_CUSTOM_PARAM_MAX_LEN - 1,
	.name = "custom end-to-end parameter string",
	.step = 1,
	.flags = CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config frag_uvc_cfg = {
	.id = CAM_EXT_CID_UVC_FRAG_CFG,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "frag uvc cfg",
	.dims = {2},
	.max = CTRL_MAX_INT,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
			| CAMERA_EXT_CTRL_FLAG_NEED_DEF,
};

static struct v4l2_ctrl_config continuous_snapshot_fps = {
	.id = CAM_EXT_CID_CONTINUOUS_SNAPSHOT_FPS,
	.type = V4L2_CTRL_TYPE_STRING,
	.name = "continuous snapshot fps",
	.max = CAM_EXT_CTRL_FLOAT_STR_LEN -  1,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY
		| CAMERA_EXT_CTRL_FLAG_NEED_DEF
		| CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER,
};

#define __ITEM(ID, item) \
	[CAM_EXT_CID_##ID - CID_CAM_EXT_CLASS_BASE] = &item

/* predefind android controls' index db, index == id - id0 */
static struct v4l2_ctrl_config* const ctrls[] = {
__ITEM(COLOR_CORRECTION_ABERRATION_MODE, color_correction_aberration_mode),
__ITEM(AE_ANTIBANDING_MODE, ae_antibanding_mode),
__ITEM(AE_EXPOSURE_COMPENSATION, ae_exposure_compensation),
__ITEM(AE_LOCK, ae_lock),
__ITEM(AE_MODE, ae_mode),
__ITEM(AE_TARGET_FPS_RANGE, ae_target_fps_range),
__ITEM(AF_MODE, af_mode),
__ITEM(AF_TRIGGER, af_trigger),
__ITEM(AWB_LOCK, awb_lock),
__ITEM(AWB_MODE, awb_mode),
__ITEM(EFFECT_MODE, effect_mode),
__ITEM(CONTROL_MODE, control_mode),
__ITEM(SCENE_MODE, scene_mode),
__ITEM(VIDEO_STABILIZATION_MODE, video_stabilization),
__ITEM(JPEG_GPS_LOCATION, jpeg_gps_location),
__ITEM(JPEG_ORIENTATION, jpeg_orientation),
__ITEM(JPEG_QUALITY, jpeg_quality),
__ITEM(LENS_FACING, lens_facing),
__ITEM(FLASH_MODE, flash_mode),
__ITEM(FOCAL_LENGTH, focal_length),
__ITEM(CAPABILITIES, capabilities),
__ITEM(MAX_NUM_OUTPUT_PROC, max_num_output_proc),
__ITEM(MAX_NUM_OUTPUT_PROC_STALLING, max_num_output_proc_stalling),
__ITEM(MAX_NUM_OUTPUT_RAW, max_num_output_raw),
__ITEM(PIPLELINE_MAX_DEPTH, pipleline_max_depth),
__ITEM(SCALER_MAX_DIGITAL_ZOOM, scaler_max_digital_zoom),
__ITEM(SCALER_CROPPING_TYPE, scaler_cropping_type),
__ITEM(SCALER_CROP_REGION, scaler_crop_region),
__ITEM(SENSOR_INFO_PHYSICAL_SIZE, sensor_info_physical_size),
__ITEM(SENSOR_INFO_PIXEL_ARRAY_SIZE, sensor_info_pixel_array_size),
__ITEM(SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE,
		sensor_info_pre_correction_active_array_size),
__ITEM(SENSOR_INFO_ACTIVE_ARRAY_SIZE, sensor_info_active_array_size),
__ITEM(SENSOR_INFO_TIMESTAMP_SOURCE, sensor_info_timestamp_source),
__ITEM(SENSOR_ORIENTATION, sensor_orientation),
__ITEM(STATISTICS_FACE_DETECT_MODE, statistics_face_detect_mode),
__ITEM(STATISTICS_INFO_MAX_FACE_COUNT, statistics_info_max_face_count),
__ITEM(SYNC_MAX_LATENCY, sync_max_latency),
__ITEM(CONTROL_AE_PRECATURE_TRIGGER, control_ae_precapture_trigger),
__ITEM(LENS_INFO_FOCUS_DISTANCE_CALIBRATION,
		lens_info_focus_distance_calibration),
__ITEM(LENS_INFO_FOCUS_HYPERFOCAL_DISTANCE,
		lens_info_focus_hyperfocal_distance),
__ITEM(LENS_INFO_MINIMUM_FOCUS_DISTANCE, lens_info_minimum_focus_distance),
__ITEM(LENS_FOCUS_DISTANCE, lens_focus_distance),
__ITEM(LENS_OPTICAL_STABILIZATION_MODE, lens_optical_stabilization_mode),
__ITEM(MANUAL_FOCUS_POSITION, manual_focus_position),
__ITEM(REPROCESS_EFFECTIVE_EXPOSURE_FACTOR,
		reprocess_effective_exposure_factore),
__ITEM(REPROCESS_MAX_CAPTURE_STALL, reprocess_max_capture_stall),
__ITEM(DEPTH_DEPTH_IS_EXCLUSIVE, depth_depth_is_exclusive),
__ITEM(BLACK_LEVEL_LOCK, black_level_lock),
__ITEM(COLOR_CORRECTION_MODE, color_correction_mode),
__ITEM(COLOR_CORRECTION_GAINS, color_correction_gains),
__ITEM(COLOR_CORRECTION_TRANSFORM, color_correction_transform),
__ITEM(EDGE_MODE, edge_mode),
__ITEM(LENS_APERTURES, lens_apertures),
__ITEM(LENS_FILTER_DENSITY, lens_filter_density),
__ITEM(NOISE_REDUCTION_MODE, noise_reduction_mode),
__ITEM(REQUEST_MAX_NUM_INPUT_STREAM, request_max_num_input_stream),
__ITEM(REQUEST_PARTIAL_RESULT_COUNT, rquest_partial_result_count),
__ITEM(SENSOR_EXPOSURE_TIME, sensor_exposure_time),
__ITEM(SENSOR_FRAME_DURATION, sensor_frame_duration),
__ITEM(SENSOR_SENSITIVITY, sensor_sensitivity),
__ITEM(SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,
		sensor_info_color_filter_arrangement),
__ITEM(SENSOR_MAX_ANALOG_SENSITIVITY,
		sensor_max_analog_sensitivity),
__ITEM(SHADING_MODE, shading_mode),
__ITEM(STATISTICS_LENS_SHADING_MAP_MODE, statistics_lens_shading_map_mode),
__ITEM(TONEMAP_CURVE, tonemap_curve),
__ITEM(TONEMAP_GAMMA, tonemap_gamma),
__ITEM(TONEMAP_MODE, tonemap_mode),
__ITEM(TONEMAP_PRESET_CURVE, tonemap_preset_curve),
__ITEM(TONEMAP_MAX_CURVE_POINSTS, tonemap_max_curve_points),
__ITEM(AE_REGIONS, ae_regions),
__ITEM(AF_REGIONS, af_regions),
__ITEM(AWB_REGIONS, awb_regions),
__ITEM(HOT_PIXEL_MODE, hot_pixel_mode),
__ITEM(LENS_INTRINSIC_CALIBRATION, lens_intrinsic_calibration),
__ITEM(LENS_POSE_ROTATION, lens_pos_rotation),
__ITEM(LENS_POSE_TRANSLATION, lens_pos_translation),
__ITEM(LENS_RADIAL_DISTORTION, lens_radial_distortion),
__ITEM(SENSOR_TEST_PATTERN_DATA, sensor_test_pattern_data),
__ITEM(SENSOR_TEST_PATTERN_MODE, sensor_test_pattern_mode),
__ITEM(SENSOR_BLACK_LEVEL_PATTERN, sensor_black_level_pattern),
__ITEM(SENSOR_CALIBRATION_TRANSFORM1, sensor_calibration_transform1),
__ITEM(SENSOR_CALIBRATION_TRANSFORM2, sensor_calibration_transform2),
__ITEM(SENSOR_COLOR_TRANSFORM1, sensor_color_transform1),
__ITEM(SENSOR_COLOR_TRANSFORM2, sensor_color_transform2),
__ITEM(SENSOR_FORWARD_MATRIX1, sensor_forward_matrix1),
__ITEM(SENSOR_FORWARD_MATRIX2, sensor_forward_matrix2),
__ITEM(SENSOR_INFO_LENS_SHADING_APPLIED, sensor_info_lens_shading_applied),
__ITEM(SENSOR_INFO_WHITE_LEVEL, sensor_info_white_level),
__ITEM(SENSOR_PREFERENCE_ILLUMINANT1, sensor_preference_illuminant1),
__ITEM(SENSOR_PREFERENCE_ILLUMINANT2, sensor_preference_illuminant2),
__ITEM(STATISTICS_HOT_PIXEL_MAP_MODE, statistics_hot_pixel_map_mode),
__ITEM(HOT_PIXEL_MAP, hot_pixel_map),
__ITEM(CAPTURE, start_capture),
__ITEM(AF_MODE_EXT, af_mode_ext),
__ITEM(ISO, iso),
__ITEM(ND_FILTER, nd_filter),
__ITEM(JPEG_SHARPNESS, jpeg_sharpness),
__ITEM(JPEG_CONTRAST, jpeg_contrast),
__ITEM(JPEG_SATURATION, jpeg_saturation),
__ITEM(TIME_SYNC, time_sync),
__ITEM(JPEG_GPS_TIMESTAMP, jpeg_gps_timestamp),
__ITEM(JPEG_GPS_PROC_METHOD, jpeg_gps_proc_method),
__ITEM(FACE_DETECTION, face_detection),
__ITEM(MOD_CAPS_UVC_SNAPSHOT, uvc_snapshot),
__ITEM(MOD_META_DATA_PATH, meta_data_path),
__ITEM(MOD_META_DATA_SIZE, meta_data_size),
__ITEM(ZOOM_LOCK_1X, zoom_lock_1x),
__ITEM(SHUTTER_LOCK, shutter_lock),
__ITEM(MODEL_NUMBER, model_number),
__ITEM(FIRMWARE_VERSION, firmware_version),
__ITEM(AE_MODE_EXT, ae_mode_ext),
__ITEM(SCENE_MODE_EXT, scene_mode_ext),
__ITEM(ZOOM_LIMIT, zoom_limit),
__ITEM(FOCUS_KEY_LOCK, focus_key_lock),
__ITEM(EIS_FRAME_SIZE_MAP, eis_frmsize_map),
__ITEM(JPEG_AVAILABLE_THUMBNAIL_SIZES, jpeg_available_thumbnail_sizes),
__ITEM(JPEG_THUMBNAIL_SIZE_INDEX, jpeg_thumbnail_size_index),
__ITEM(PHONE_VERSION, phone_version),
__ITEM(SUPPLEMENTAL_KEY_MASK, supplemental_key_mask),
__ITEM(GROUP_IND, group_ind),
__ITEM(VIDEO_RECORD_HINT, video_record_hint),
__ITEM(RAW_TO_YUV_GAIN, raw_to_yuv_gain),
__ITEM(EFFECT_MODE_EXT, effect_mode_ext),
__ITEM(ZSL_BUFFER_DEPTH, zsl_buffer_depth),
__ITEM(CUSTOM_PARAMETER, custom_parameter),
__ITEM(UVC_FRAG_CFG, frag_uvc_cfg),
__ITEM(CONTINUOUS_SNAPSHOT_FPS, continuous_snapshot_fps),
};

struct v4l2_ctrl_config *camera_ext_get_ctrl_config(uint32_t id)
{
	uint32_t idx = id - CID_CAM_EXT_CLASS_BASE;

	if (idx >= sizeof(ctrls)/sizeof(ctrls[0]))
		return NULL;
	return ctrls[idx];
}
