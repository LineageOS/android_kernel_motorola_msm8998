/*
 * Greybus audio commands
 *
 * Copyright 2015 Google Inc.
 * Copyright 2015 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#include <linux/kernel.h>

#include "greybus.h"
#include "audio.h"

#define AUDIO_GB_CMD_TIME_OUT  2000

/* mods audio will be using standard I2S protocol always */
static const struct gb_i2s_mgmt_config_masks mods_i2s_cfg = {
	.protocol = GB_I2S_MGMT_PROTOCOL_I2S,
	.wclk_polarity = GB_I2S_MGMT_POLARITY_NORMAL,
	.wclk_change_edge = GB_I2S_MGMT_EDGE_RISING,
	.wclk_tx_edge = GB_I2S_MGMT_EDGE_FALLING,
	.wclk_rx_edge = GB_I2S_MGMT_EDGE_RISING,
};

/***********************************
 * GB I2S helper functions
 ***********************************/
int gb_i2s_mgmt_activate_cport(struct gb_connection *connection,
					  uint16_t cport)
{
	struct gb_i2s_mgmt_activate_cport_request request;

	memset(&request, 0, sizeof(request));
	request.cport = cpu_to_le16(cport);

	return gb_operation_sync(connection, GB_I2S_MGMT_TYPE_ACTIVATE_CPORT,
				 &request, sizeof(request), NULL, 0);
}

int gb_i2s_mgmt_deactivate_cport(struct gb_connection *connection,
					uint16_t cport)
{
	struct gb_i2s_mgmt_deactivate_cport_request request;

	memset(&request, 0, sizeof(request));
	request.cport = cpu_to_le16(cport);

	return gb_operation_sync(connection, GB_I2S_MGMT_TYPE_DEACTIVATE_CPORT,
				 &request, sizeof(request), NULL, 0);
}

int gb_i2s_mgmt_activate_port(struct gb_connection *connection,
				      uint8_t port_type)
{
	struct gb_i2s_mgmt_activate_port_request request;

	memset(&request, 0, sizeof(request));
	request.port_type = port_type;

	return gb_operation_sync(connection, GB_I2S_MGMT_TYPE_ACTIVATE_PORT,
				 &request, sizeof(request), NULL, 0);
}

int gb_i2s_mgmt_deactivate_port(struct gb_connection *connection,
					uint8_t port_type)
{
	struct gb_i2s_mgmt_deactivate_port_request request;

	memset(&request, 0, sizeof(request));
	request.port_type = port_type;

	return gb_operation_sync(connection, GB_I2S_MGMT_TYPE_DEACTIVATE_PORT,
				 &request, sizeof(request), NULL, 0);
}

int gb_i2s_mgmt_get_supported_configurations(
	struct gb_connection *connection,
	struct gb_i2s_mgmt_get_supported_configurations_response *get_cfg,
	size_t size)
{
	return gb_operation_sync(connection,
				 GB_I2S_MGMT_TYPE_GET_SUPPORTED_CONFIGURATIONS,
				 NULL, 0, get_cfg, size);
}

int gb_i2s_mgmt_set_configuration(struct gb_connection *connection,
			struct gb_i2s_mgmt_set_configuration_request *set_cfg)
{
	return gb_operation_sync_timeout(connection,
				GB_I2S_MGMT_TYPE_SET_CONFIGURATION, set_cfg,
				sizeof(*set_cfg), NULL, 0,
				AUDIO_GB_CMD_TIME_OUT);
}

int gb_i2s_mgmt_set_samples_per_message(
				struct gb_connection *connection,
				uint16_t samples_per_message)
{
	struct gb_i2s_mgmt_set_samples_per_message_request request;

	memset(&request, 0, sizeof(request));
	request.samples_per_message = cpu_to_le16(samples_per_message);

	return gb_operation_sync(connection,
				 GB_I2S_MGMT_TYPE_SET_SAMPLES_PER_MESSAGE,
				 &request, sizeof(request), NULL, 0);
}

bool gb_i2s_audio_is_ver_supported(struct gb_connection *conn,
				uint32_t major, uint32_t minor)
{
	if (conn->module_major < major) {
		pr_debug("%s() mod fw does'nt support cfg masks!\n", __func__);
		return false;
	}
	if (conn->module_minor < minor && conn->module_major == major) {
		pr_debug("%s() mod fw minor version does'nt support cfg masks!\n",
			__func__);
		return false;
	}

	return true;
}

int gb_i2s_mgmt_get_cfgs(struct gb_snd_codec *snd_codec,
			 struct gb_connection *connection)
{
	struct gb_i2s_mgmt_get_supported_configurations_response *get_cfg;
	struct gb_i2s_mgmt_get_config_masks_response *get_cfg_mask;
	size_t size;
	int ret;

	if (gb_i2s_audio_is_ver_supported(snd_codec->mgmt_connection,
					GB_I2S_MGMT_VERSION_CFG_MASK_MAJOR,
					GB_I2S_MGMT_VERSION_CFG_MASK_MINOR)) {
		size = sizeof(*get_cfg_mask);

		get_cfg_mask = kzalloc(size, GFP_KERNEL);
		if (!get_cfg_mask)
			return -ENOMEM;

		ret = gb_operation_sync(connection,
				 GB_I2S_MGMT_TYPE_GET_SUPPORTED_CONFIGURATIONS,
				 NULL, 0, get_cfg_mask, size);
		if (ret) {
			pr_err("get_supported_config maska failed: %d\n", ret);
			kfree(get_cfg_mask);
			return ret;
		}

		snd_codec->i2s_cfg_masks = get_cfg_mask;
	} else {
		 size = sizeof(*get_cfg) +
				(CONFIG_COUNT_MAX * sizeof(get_cfg->config[0]));
		get_cfg = kzalloc(size, GFP_KERNEL);
		if (!get_cfg)
			return -ENOMEM;

		ret = gb_i2s_mgmt_get_supported_configurations(
					snd_codec->mgmt_connection, get_cfg,
							size);
		if (ret) {
			pr_err("get_supported_config failed: %d\n", ret);
			kfree(get_cfg);
			return ret;
		}

		snd_codec->i2s_configs = get_cfg;
	}

	return 0;
}

void gb_i2s_mgmt_free_cfgs(struct gb_snd_codec *snd_codec)
{
	kfree(snd_codec->i2s_configs);
	snd_codec->i2s_configs = NULL;
	kfree(snd_codec->i2s_cfg_masks);
	snd_codec->i2s_cfg_masks = NULL;
}

static int gb_i2s_mgmt_convert_rate_to_gb_i2s(uint32_t rate)
{
	switch (rate) {
	case 16000:
		return GB_I2S_MGMT_PCM_RATE_16000;
	case 48000:
		return GB_I2S_MGMT_PCM_RATE_48000;
	case 96000:
		return GB_I2S_MGMT_PCM_RATE_96000;
	case 8000:
		return GB_I2S_MGMT_PCM_RATE_8000;
	case 44100:
		return GB_I2S_MGMT_PCM_RATE_44100;
	case 88200:
		return GB_I2S_MGMT_PCM_RATE_88200;
	case 192000:
		return GB_I2S_MGMT_PCM_RATE_192000;
	case 11025:
		return GB_I2S_MGMT_PCM_RATE_11025;
	case 5512:
		return GB_I2S_MGMT_PCM_RATE_5512;
	case 22050:
		return GB_I2S_MGMT_PCM_RATE_22050;
	case 32000:
		return GB_I2S_MGMT_PCM_RATE_32000;
	case 64000:
		return GB_I2S_MGMT_PCM_RATE_64000;
	case 176400:
		return GB_I2S_MGMT_PCM_RATE_176400;
	default:
		return -EINVAL;
	}
	return -EINVAL;
}

static int gb_i2s_mgmt_convert_format_to_gb_i2s(uint32_t format)
{
	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		return GB_I2S_MGMT_PCM_FMT_16;
	case SNDRV_PCM_FORMAT_S24_LE:
		return GB_I2S_MGMT_PCM_FMT_24;
	case SNDRV_PCM_FORMAT_S32_LE:
		return GB_I2S_MGMT_PCM_FMT_32;
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int gb_i2s_mgmt_is_cfg_supported(struct gb_snd_codec *snd_codec,
		uint32_t rate, uint8_t chans, uint32_t format)
{
	struct gb_i2s_mgmt_config_masks *cfg;
	int gb_rate = gb_i2s_mgmt_convert_rate_to_gb_i2s(rate);
	int gb_format = gb_i2s_mgmt_convert_format_to_gb_i2s(format);

	if (gb_rate < 0) {
		pr_err("%s gb rate invalid\n", __func__);
		return -EINVAL;
	}
	if (gb_format < 0) {
		pr_err("%s gb format invalid\n", __func__);
		return -EINVAL;
	}

	if (!snd_codec->i2s_cfg_masks) {
		pr_err("%s i2s cfg masks is not init'd\n", __func__);
		return -EINVAL;
	}
	cfg = &snd_codec->i2s_cfg_masks->config;

	if (((cfg->sample_frequency & cpu_to_le32(gb_rate)) !=
		cpu_to_le32(gb_rate)) ||
					(cfg->num_channels < chans) ||
		((cfg->format & cpu_to_le32(gb_format)) !=
					cpu_to_le32(gb_format)) ||
		((cfg->protocol & mods_i2s_cfg.protocol) !=
					mods_i2s_cfg.protocol) ||
		((cfg->wclk_polarity & mods_i2s_cfg.wclk_polarity) !=
					mods_i2s_cfg.wclk_polarity) ||
		((cfg->wclk_change_edge & mods_i2s_cfg.wclk_change_edge) !=
					mods_i2s_cfg.wclk_change_edge) ||
		((cfg->wclk_rx_edge & mods_i2s_cfg.wclk_rx_edge) !=
					mods_i2s_cfg.wclk_rx_edge) ||
		((cfg->wclk_tx_edge & mods_i2s_cfg.wclk_tx_edge) !=
					mods_i2s_cfg.wclk_tx_edge)) {
		pr_err("%s() config (fmt 0x%x, %uHz, %i channel) not supported by mods codec",
			 __func__, gb_format, rate, chans);
		return -EINVAL;
	}

	return 0;
}

static int gb_i2s_mgmt_set_cfg_masks(struct gb_snd_codec *snd_codec,
			uint32_t rate, uint8_t chans, uint32_t format)
{
	struct gb_i2s_mgmt_set_config_masks_request set_cfg;
	int ret;
	int gb_rate = gb_i2s_mgmt_convert_rate_to_gb_i2s(rate);
	int gb_format = gb_i2s_mgmt_convert_format_to_gb_i2s(format);

	pr_debug("%s gb rate %d gb format %d\n", __func__, gb_rate, gb_format);
	memset(&set_cfg, 0, sizeof(set_cfg));
	set_cfg.config.num_channels = chans;
	set_cfg.config.format = cpu_to_le32(gb_format);
	set_cfg.config.sample_frequency = cpu_to_le32(gb_rate);
	set_cfg.config.protocol = mods_i2s_cfg.protocol;
	set_cfg.config.wclk_polarity = mods_i2s_cfg.wclk_polarity;
	set_cfg.config.wclk_change_edge = mods_i2s_cfg.wclk_change_edge;
	set_cfg.config.wclk_tx_edge = mods_i2s_cfg.wclk_tx_edge;
	set_cfg.config.wclk_rx_edge = mods_i2s_cfg.wclk_rx_edge;

	ret = gb_operation_sync_timeout(snd_codec->mgmt_connection,
				GB_I2S_MGMT_TYPE_SET_CONFIGURATION,
				&set_cfg, sizeof(set_cfg), NULL, 0,
				AUDIO_GB_CMD_TIME_OUT);
	if (ret)
		pr_err("set_configuration failed: %d\n", ret);

	return ret;
}
int gb_i2s_mgmt_set_cfg(struct gb_snd_codec *snd_codec, uint32_t rate,
			uint8_t chans, uint32_t format,
			int bytes_per_chan, int is_le)
{
	struct gb_i2s_mgmt_set_configuration_request set_cfg;
	struct gb_i2s_mgmt_configuration *cfg;
	int i, ret;
	u8 byte_order = GB_I2S_MGMT_BYTE_ORDER_NA;

	if (gb_i2s_audio_is_ver_supported(snd_codec->mgmt_connection,
					GB_I2S_MGMT_VERSION_CFG_MASK_MAJOR,
					GB_I2S_MGMT_VERSION_CFG_MASK_MINOR)) {
		ret = gb_i2s_mgmt_is_cfg_supported(snd_codec,
						rate, chans, format);
		if (!ret)
			ret = gb_i2s_mgmt_set_cfg_masks(snd_codec,
						rate, chans, format);
		return ret;
	}

	if (bytes_per_chan > 1) {
		if (is_le)
			byte_order = GB_I2S_MGMT_BYTE_ORDER_LE;
		else
			byte_order = GB_I2S_MGMT_BYTE_ORDER_BE;
	}

	for (i = 0, cfg = snd_codec->i2s_configs->config;
		 i < CONFIG_COUNT_MAX;
		 i++, cfg++) {
		if ((cfg->sample_frequency == cpu_to_le32(rate)) &&
			(cfg->num_channels == chans) &&
			(cfg->bytes_per_channel == bytes_per_chan) &&
			(cfg->byte_order & byte_order) &&
			(cfg->ll_protocol &
					cpu_to_le32(mods_i2s_cfg.protocol)) &&
			(cfg->ll_mclk_role & GB_I2S_MGMT_ROLE_MASTER) &&
			(cfg->ll_bclk_role & GB_I2S_MGMT_ROLE_MASTER) &&
			(cfg->ll_wclk_role & GB_I2S_MGMT_ROLE_MASTER) &&
			(cfg->ll_wclk_polarity & mods_i2s_cfg.wclk_polarity) &&
			(cfg->ll_wclk_change_edge &
					mods_i2s_cfg.wclk_change_edge) &&
			(cfg->ll_wclk_tx_edge & mods_i2s_cfg.wclk_tx_edge) &&
			(cfg->ll_wclk_rx_edge & mods_i2s_cfg.wclk_rx_edge) &&
			(cfg->ll_data_offset == 1))
			break;
	}

	if (i >= CONFIG_COUNT_MAX) {
		pr_err("No valid configuration\n");
		return -EINVAL;
	}

	memcpy(&set_cfg, cfg, sizeof(set_cfg));
	set_cfg.config.byte_order = byte_order;
	set_cfg.config.ll_protocol = cpu_to_le32(mods_i2s_cfg.protocol);
	set_cfg.config.ll_mclk_role = GB_I2S_MGMT_ROLE_MASTER;
	set_cfg.config.ll_bclk_role = GB_I2S_MGMT_ROLE_MASTER;
	set_cfg.config.ll_wclk_role = GB_I2S_MGMT_ROLE_MASTER;
	set_cfg.config.ll_wclk_polarity = mods_i2s_cfg.wclk_polarity;
	set_cfg.config.ll_wclk_change_edge = mods_i2s_cfg.wclk_change_edge;
	set_cfg.config.ll_wclk_tx_edge = mods_i2s_cfg.wclk_tx_edge;
	set_cfg.config.ll_wclk_rx_edge = mods_i2s_cfg.wclk_rx_edge;

	ret = gb_i2s_mgmt_set_configuration(snd_codec->mgmt_connection,
					&set_cfg);
	if (ret)
		pr_err("set_configuration failed: %d\n", ret);

	return ret;
}

int gb_i2s_mgmt_send_start(struct gb_snd_codec *snd_codec, uint32_t port_type,
			bool start)
{
	int ret;
	struct gb_i2s_mgmt_start_request req_start;
	struct gb_i2s_mgmt_stop_request req_stop;

	if (!gb_i2s_audio_is_ver_supported(snd_codec->mgmt_connection,
					GB_I2S_MGMT_VERSION_START_MSG_MAJOR,
					GB_I2S_MGMT_VERSION_START_MSG_MINOR)) {
		pr_warn("gb i2s start and stop messages not supported by mod\n");
		return -ENOTSUPP;
	}

	if (start) {
		req_start.port_type = port_type;
		ret = gb_operation_sync_timeout(snd_codec->mgmt_connection,
					GB_I2S_MGMT_TYPE_START,
					&req_start, sizeof(req_start), NULL, 0,
					AUDIO_GB_CMD_TIME_OUT);
		if (ret)
			pr_err("%s(): gb i2s start failed: %d\n",
					__func__, ret);
	} else {
		req_stop.port_type = port_type;
		ret = gb_operation_sync_timeout(snd_codec->mgmt_connection,
					GB_I2S_MGMT_TYPE_STOP,
					&req_stop, sizeof(req_stop), NULL, 0,
					AUDIO_GB_CMD_TIME_OUT);
		if (ret)
			pr_err("%s(): gb i2s stop failed: %d\n", __func__, ret);
	}

	return ret;
}

int gb_mods_aud_get_vol_range(
			struct gb_audio_get_volume_db_range_response *get_vol,
			struct gb_connection *connection)
{
	int ret;
	size_t size = sizeof(*get_vol);

	ret = gb_operation_sync(connection,
				 GB_AUDIO_GET_VOLUME_DB_RANGE,
				 NULL, 0, get_vol, size);
	if (ret) {
		pr_err("get vol failed: %d\n", ret);
		return ret;
	}

	return 0;
}


int gb_mods_aud_get_supported_usecase(
		struct gb_audio_get_supported_usecases_response *get_usecase,
		struct gb_connection *connection)
{
	int ret;
	size_t size = sizeof(*get_usecase);

	ret = gb_operation_sync(connection,
				 GB_AUDIO_GET_SUPPORTED_USE_CASES,
				 NULL, 0, get_usecase, size);
	if (ret) {
		pr_err("get usecase failed: %d\n", ret);
		return ret;
	}

	return 0;
}

int gb_mods_aud_set_vol(struct gb_connection *connection,
			uint32_t vol_step)
{
	struct gb_audio_set_volume_db_request request;

	request.vol_step = cpu_to_le32(vol_step);

	return gb_operation_sync(connection, GB_AUDIO_SET_VOLUME,
				 &request, sizeof(request), NULL, 0);

}

int gb_mods_aud_set_sys_vol(struct gb_connection *connection,
			int vol_db)
{
	struct gb_audio_set_system_volume_db_request request;

	request.vol_db = cpu_to_le32(vol_db);

	return gb_operation_sync(connection, GB_AUDIO_SET_SYSTEM_VOLUME,
				 &request, sizeof(request), NULL, 0);

}

int gb_mods_aud_set_playback_usecase(struct gb_connection *connection,
			uint32_t usecase)
{
	struct gb_audio_set_use_case_request request;

	request.use_case = cpu_to_le32(usecase);

	return gb_operation_sync(connection, GB_AUDIO_SET_PLAYBACK_USE_CASE,
				 &request, sizeof(request), NULL, 0);
}

int gb_mods_aud_set_capture_usecase(struct gb_connection *connection,
			uint32_t usecase)
{
	struct gb_audio_set_use_case_request request;

	request.use_case = cpu_to_le32(usecase);

	return gb_operation_sync(connection, GB_AUDIO_SET_CAPTURE_USE_CASE,
				 &request, sizeof(request), NULL, 0);
}

int gb_mods_aud_get_devices(
		struct gb_audio_get_devices_response *get_devices,
		struct gb_connection *connection)
{
	int ret;
	size_t size = sizeof(*get_devices);

	ret = gb_operation_sync(connection,
				GB_AUDIO_GET_SUPPORTED_DEVICES,
				NULL, 0, get_devices, size);
	if (ret) {
		pr_err("get supported devices failed: %d\n", ret);
		return ret;
	}

	return 0;
}

int gb_mods_aud_enable_devices(struct gb_connection *connection,
			uint32_t in_devices, uint32_t out_devices)
{
	struct gb_audio_enable_devices_request request;

	request.devices.in_devices = cpu_to_le32(in_devices);
	request.devices.out_devices = cpu_to_le32(out_devices);

	return gb_operation_sync(connection, GB_AUDIO_ENABLE_DEVICES,
				 &request, sizeof(request), NULL, 0);
}

int gb_mods_aud_get_speaker_preset_eq(
		struct gb_audio_get_speaker_preset_eq_response *get_preset,
		struct gb_connection *connection)
{
	int ret;
	size_t size = sizeof(*get_preset);

	ret = gb_operation_sync(connection,
				 GB_AUDIO_GET_SPEAKER_PRESET_EQ,
				 NULL, 0, get_preset, size);
	if (ret) {
		pr_err("get speaker preset eq failed: %d\n", ret);
		return ret;
	}

	return 0;
}

int gb_mods_aud_get_mic_params(
		struct gb_audio_get_mic_params_response *get_params,
		struct gb_connection *connection)
{
	int ret;
	size_t size = sizeof(*get_params);

	ret = gb_operation_sync(connection,
				 GB_AUDIO_GET_MIC_PARAMS,
				 NULL, 0, get_params, size);
	if (ret) {
		pr_err("get mic tuning params failed: %d\n", ret);
		return ret;
	}

	return 0;
}
