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

#define GB_SAMPLE_RATE				48000
#define GB_RATES				SNDRV_PCM_RATE_48000
#define GB_FMTS					SNDRV_PCM_FMTBIT_S16_LE
#define PREALLOC_BUFFER				(32 * 1024)
#define PREALLOC_BUFFER_MAX			(32 * 1024)

/* assuming 1 ms samples @ 48KHz */
#define CONFIG_SAMPLES_PER_MSG			48L
#define CONFIG_PERIOD_NS			1000000 /* send msg every 1ms */

#define CONFIG_COUNT_MAX			5

/* Switch between dummy spdif and jetson rt5645 codec */
#define USE_RT5645				0

#define SAMPLE_SIZE				4
#define MAX_SEND_DATA_LEN (CONFIG_SAMPLES_PER_MSG * SAMPLE_SIZE)
#define SEND_DATA_BUF_LEN (sizeof(struct gb_i2s_send_data_request) + \
				MAX_SEND_DATA_LEN)


/*
 * This is the gb_snd structure which ties everything together
 * and fakes DMA interrupts via a timer.
 */
struct gb_snd {
	struct platform_device		card;
	struct platform_device		cpu_dai;
	struct platform_device		*codec;
	struct platform_device		*dai;
	struct asoc_simple_card_info	*simple_card_info;
	struct i2c_client		*rt5647;
	struct gb_connection		*mgmt_connection;
	struct gb_connection		*i2s_tx_connection;
	struct gb_connection		*i2s_rx_connection;
	struct gb_i2s_mgmt_get_supported_configurations_response
					*i2s_configs;
	char				*send_data_req_buf;
	long				send_data_sample_count;
	int				gb_bundle_id;
	int				device_count;
	struct snd_pcm_substream	*substream;
	struct hrtimer			timer;
	atomic_t			running;
	bool				cport_active;
	struct workqueue_struct		*workqueue;
	struct work_struct		work;
	int				hwptr_done;
	int				transfer_done;
	struct list_head		list;
	spinlock_t			lock;
};

/*
 * This codec structure will be passed as platform data
 * to mods codec when physical I2S interface is used
 * instead of pcm tunneling.
 */
struct gb_snd_codec {
	struct platform_device		codec_dev;
	struct list_head		*gb_snd_devs;
	struct gb_audio_get_volume_db_range_response *vol_range;
	struct gb_audio_get_supported_usecases_response *use_cases;
	struct gb_audio_get_devices_response *aud_devices;
	struct gb_connection *mods_aud_connection;
	struct mutex lock;
	int (*report_devices)(struct gb_snd_codec *);
};

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
int gb_i2s_mgmt_get_cfgs(struct gb_snd *snd_dev,
			 struct gb_connection *connection);
void gb_i2s_mgmt_free_cfgs(struct gb_snd *snd_dev);
int gb_i2s_mgmt_set_cfg(struct gb_snd *snd_dev, int rate, int chans,
			int bytes_per_chan, int is_le);
int gb_i2s_send_data(struct gb_connection *connection, void *req_buf,
				void *source_addr, size_t len, int sample_num);

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
/*
 * GB PCM hooks
 */
void gb_pcm_hrtimer_start(struct gb_snd *snd_dev);
void gb_pcm_hrtimer_stop(struct gb_snd *snd_dev);

/*
 * Platform drivers
 */
extern struct platform_driver gb_audio_pcm_driver;
extern struct platform_driver gb_audio_plat_driver;
extern struct platform_driver gb_audio_mods_driver;


#endif /* __GB_AUDIO_H */
