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
	return gb_operation_sync(connection, GB_I2S_MGMT_TYPE_SET_CONFIGURATION,
				 set_cfg, sizeof(*set_cfg), NULL, 0);
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

int gb_i2s_mgmt_get_cfgs(struct gb_snd_codec *snd_codec,
			 struct gb_connection *connection)
{
	struct gb_i2s_mgmt_get_supported_configurations_response *get_cfg;
	size_t size;
	int ret;

	size = sizeof(*get_cfg) +
	       (CONFIG_COUNT_MAX * sizeof(get_cfg->config[0]));

	get_cfg = kzalloc(size, GFP_KERNEL);
	if (!get_cfg)
		return -ENOMEM;

	ret = gb_i2s_mgmt_get_supported_configurations(connection, get_cfg,
						       size);
	if (ret) {
		pr_err("get_supported_config failed: %d\n", ret);
		goto err_free_get_cfg;
	}

	snd_codec->i2s_configs = get_cfg;

	return 0;

err_free_get_cfg:
	kfree(get_cfg);
	return ret;
}

void gb_i2s_mgmt_free_cfgs(struct gb_snd_codec *snd_codec)
{
	kfree(snd_codec->i2s_configs);
	snd_codec->i2s_configs = NULL;
}

int gb_i2s_mgmt_set_cfg(struct gb_snd_codec *snd_codec, int rate, int chans,
			int bytes_per_chan, int is_le)
{
	struct gb_i2s_mgmt_set_configuration_request set_cfg;
	struct gb_i2s_mgmt_configuration *cfg;
	int i, ret;
	u8 byte_order = GB_I2S_MGMT_BYTE_ORDER_NA;

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
			     cpu_to_le32(GB_I2S_MGMT_PROTOCOL_I2S)) &&
		    (cfg->ll_mclk_role & GB_I2S_MGMT_ROLE_MASTER) &&
		    (cfg->ll_bclk_role & GB_I2S_MGMT_ROLE_MASTER) &&
		    (cfg->ll_wclk_role & GB_I2S_MGMT_ROLE_MASTER) &&
		    (cfg->ll_wclk_polarity & GB_I2S_MGMT_POLARITY_NORMAL) &&
		    (cfg->ll_wclk_change_edge & GB_I2S_MGMT_EDGE_FALLING) &&
		    (cfg->ll_wclk_tx_edge & GB_I2S_MGMT_EDGE_RISING) &&
		    (cfg->ll_wclk_rx_edge & GB_I2S_MGMT_EDGE_FALLING) &&
		    (cfg->ll_data_offset == 1))
			break;
	}

	if (i >= CONFIG_COUNT_MAX) {
		pr_err("No valid configuration\n");
		return -EINVAL;
	}

	memcpy(&set_cfg, cfg, sizeof(set_cfg));
	set_cfg.config.byte_order = byte_order;
	set_cfg.config.ll_protocol = cpu_to_le32(GB_I2S_MGMT_PROTOCOL_I2S);
	set_cfg.config.ll_mclk_role = GB_I2S_MGMT_ROLE_MASTER;
	set_cfg.config.ll_bclk_role = GB_I2S_MGMT_ROLE_MASTER;
	set_cfg.config.ll_wclk_role = GB_I2S_MGMT_ROLE_MASTER;
	set_cfg.config.ll_wclk_polarity = GB_I2S_MGMT_POLARITY_NORMAL;
	set_cfg.config.ll_wclk_change_edge = GB_I2S_MGMT_EDGE_FALLING;
	set_cfg.config.ll_wclk_tx_edge = GB_I2S_MGMT_EDGE_RISING;
	set_cfg.config.ll_wclk_rx_edge = GB_I2S_MGMT_EDGE_FALLING;

	ret = gb_i2s_mgmt_set_configuration(snd_codec->mgmt_connection, &set_cfg);
	if (ret)
		pr_err("set_configuration failed: %d\n", ret);

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
