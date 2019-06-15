/*
 * Greybus audio
 *
 * Copyright 2015 Google Inc.
 * Copyright 2015 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#ifndef __GB_AUDIO_H
#define __GB_AUDIO_H
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/soc.h>

#include "greybus.h"

#define GB_RATES (SNDRV_PCM_RATE_5512 | SNDRV_PCM_RATE_8000 |\
		SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 |\
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |\
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
		SNDRV_PCM_RATE_64000 | SNDRV_PCM_RATE_96000 |\
		SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000)

#define GB_FMTS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

#define MODS_VOL_STEP		50
#define MODS_MIN_VOL		-12750

#define CONFIG_COUNT_MAX		5

/*
 * This codec structure will be passed as platform data
 * to mods codec when physical I2S interface is used
 * instead of pcm tunneling.
 */
struct gb_snd_codec {
	struct platform_device		codec_dev;
	struct gb_audio_get_volume_db_range_response *vol_range;
	struct gb_audio_get_supported_usecases_response *use_cases;
	struct gb_audio_get_devices_response *aud_devices;
	struct gb_i2s_mgmt_get_supported_configurations_response
			*i2s_configs; /* table of i2s configurations*/
	struct gb_i2s_mgmt_get_config_masks_response
			*i2s_cfg_masks; /* bit mask of i2s configurations */
	struct gb_audio_get_speaker_preset_eq_response *spkr_preset;
	struct gb_connection *mods_aud_connection;
	struct gb_connection *mgmt_connection;
	uint32_t playback_use_case;
	uint32_t capture_use_case;
	int sys_vol_mb;
	int mods_vol_step;
	struct mutex lock;
	struct kref mods_aud_kref;
	struct kref mods_i2s_kref;
	int (*report_devices)(struct gb_snd_codec *);
};

/* kref resource counting */
void gb_mods_audio_get(struct gb_snd_codec *codec);
void gb_mods_audio_put(struct gb_snd_codec *codec);
void gb_mods_i2s_get(struct gb_snd_codec *codec);
void gb_mods_i2s_put(struct gb_snd_codec *codec);

/* Version check utilily */
bool gb_i2s_audio_is_ver_supported(struct gb_connection *conn,
				uint32_t major, uint32_t minor);
/*
 * GB I2S cmd functions
 */
int gb_i2s_mgmt_activate_cport(struct gb_connection *connection,
				      uint16_t cport);
int gb_i2s_mgmt_deactivate_cport(struct gb_connection *connection,
					uint16_t cport);
int gb_i2s_mgmt_get_supported_configurations(
	struct gb_connection *connection,
	struct gb_i2s_mgmt_get_supported_configurations_response *get_cfg,
	size_t size);
int gb_i2s_mgmt_set_configuration(struct gb_connection *connection,
			struct gb_i2s_mgmt_set_configuration_request *set_cfg);
int gb_i2s_mgmt_set_samples_per_message(struct gb_connection *connection,
					uint16_t samples_per_message);
int gb_i2s_mgmt_get_cfgs(struct gb_snd_codec *snd_codec,
			 struct gb_connection *connection);
void gb_i2s_mgmt_free_cfgs(struct gb_snd_codec *snd_codec);
int gb_i2s_mgmt_set_cfg(struct gb_snd_codec *snd_codec, uint32_t rate,
			uint8_t chans, uint32_t format,
			int bytes_per_chan, int is_le);
int gb_i2s_mgmt_activate_port(struct gb_connection *connection,
				uint8_t port_type);
int gb_i2s_mgmt_deactivate_port(struct gb_connection *connection,
				uint8_t port_type);
int gb_i2s_mgmt_send_start(struct gb_snd_codec *snd_codec, uint32_t port_type,
			bool start);

/* GB Mods Audio Cmd functions */
int gb_mods_aud_get_vol_range(
			struct gb_audio_get_volume_db_range_response *vol,
			struct gb_connection *connection);
int gb_mods_aud_get_supported_usecase(
		struct gb_audio_get_supported_usecases_response *usecase,
		struct gb_connection *connection);
int gb_mods_aud_set_vol(struct gb_connection *connection,
			uint32_t vol_step);
int gb_mods_aud_set_sys_vol(struct gb_connection *connection,
			int vol_db);
int gb_mods_aud_set_playback_usecase(struct gb_connection *connection,
			uint32_t usecase);
int gb_mods_aud_set_capture_usecase(struct gb_connection *connection,
			uint32_t usecase);
int gb_mods_aud_enable_devices(struct gb_connection *connection,
			uint32_t in_devices, uint32_t out_devices);
int gb_mods_aud_get_devices(
		struct gb_audio_get_devices_response *get_devices,
		struct gb_connection *connection);
int gb_mods_aud_get_speaker_preset_eq(
		struct gb_audio_get_speaker_preset_eq_response *get_preset,
		struct gb_connection *connection);
int gb_mods_aud_get_mic_params(
		struct gb_audio_get_mic_params_response *get_params,
		struct gb_connection *connection);
/*
 * Platform drivers
 */
extern struct platform_driver gb_audio_mods_driver;


#endif /* __GB_AUDIO_H */
