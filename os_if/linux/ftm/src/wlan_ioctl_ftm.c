/*
 * Copyright (c) 2018 The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC: implementation of the driver FTM functions interfacing with linux kernel
 */

#include <qdf_util.h>
#include <wlan_objmgr_pdev_obj.h>
#include <wlan_ftm_ucfg_api.h>
#include <qdf_types.h>
#include <qdf_module.h>
#include <wlan_cfg80211_ftm.h>
#include <wlan_ioctl_ftm.h>

static QDF_STATUS
wlan_process_ftm_ioctl_cmd(struct wlan_objmgr_pdev *pdev,
		uint8_t *userdata, uint32_t length)
{
	uint8_t *buffer;
	QDF_STATUS error;

	if (get_user(length, (uint32_t *)userdata) != 0)
		return QDF_STATUS_E_FAILURE;

	if (length > WLAN_FTM_DATA_MAX_LEN)
		return QDF_STATUS_E_FAILURE;

	buffer = qdf_mem_malloc(length);
	if (!buffer)
		return QDF_STATUS_E_NOMEM;

	if (copy_from_user(buffer, &userdata[sizeof(length)], length))
		error = QDF_STATUS_E_FAILURE;
	else
		error = ucfg_wlan_ftm_testmode_cmd(pdev, buffer, length);

	qdf_mem_free(buffer);

	return error;
}

static QDF_STATUS
wlan_process_ftm_ioctl_rsp(struct wlan_objmgr_pdev *pdev,
		uint8_t *userdata, uint32_t length)
{
	uint8_t *buffer;
	QDF_STATUS error;

	length = WLAN_FTM_DATA_MAX_LEN + sizeof(u_int32_t);

	buffer = qdf_mem_malloc(length);
	if (!buffer)
		return QDF_STATUS_E_NOMEM;

	error = ucfg_wlan_ftm_testmode_rsp(pdev, buffer);
	if (!error)
		error = copy_to_user((userdata - sizeof(int)), buffer, length);
	else
		error = QDF_STATUS_E_AGAIN;

	qdf_mem_free(buffer);

	return error;
}

int
wlan_ioctl_ftm_testmode_cmd(struct wlan_objmgr_pdev *pdev, int cmd,
		uint8_t *userdata, uint32_t length)
{
	QDF_STATUS error;
	struct wifi_ftm_pdev_priv_obj *ftm_pdev_obj;

	ftm_pdev_obj = wlan_objmgr_pdev_get_comp_private_obj(pdev,
			WLAN_UMAC_COMP_FTM);
	if (!ftm_pdev_obj) {
		ftm_err("Failed to get ftm pdev component");
		return QDF_STATUS_E_FAILURE;
	}

#ifdef WLAN_WHITE_LIST
	//Check if the tcmd data for the FTM access is whitelisted
	if (vos_is_tcmd_data_white_listed(userdata, length) != QDF_STATUS_SUCCESS) {
		ftm_err("No permission to execute FTM command");
		return QDF_STATUS_E_PERM;
	}
#endif

	ftm_pdev_obj->cmd_type = WIFI_FTM_CMD_IOCTL;

	switch (cmd) {
	case FTM_IOCTL_UNIFIED_UTF_CMD:
		error = wlan_process_ftm_ioctl_cmd(pdev,
				userdata, length);
		break;
	case FTM_IOCTL_UNIFIED_UTF_RSP:
		error = wlan_process_ftm_ioctl_rsp(pdev,
				userdata, length);
		break;
	default:
		ftm_err("FTM Unknown cmd - not supported");
		error = QDF_STATUS_E_NOSUPPORT;
	}

	return qdf_status_to_os_return(error);
}

qdf_export_symbol(wlan_ioctl_ftm_testmode_cmd);

void getHexDump(char *s0, char *s1, int len)
{
    int i = 0, j = 0;
    printk(KERN_EMERG "%s\n :", s0);

    if (len > 8)
    {
        for (j = 0; j < len/8; j++)
        {
            printk(KERN_EMERG "%02x %02x %02x %02x %02x %02x %02x %02x",
                    s1[j*8], s1[j*8+1], s1[j*8+2], s1[j*8+3], s1[j*8+4],
                    s1[j*8+5],s1[j*8+6],s1[j*8+7] );
        }
        len = len - j*8;
    }
    for (i = 0; i< len; i++) {
        printk(KERN_EMERG "%02x ", s1[j*8+i]);
    }
    printk("\n");
}

//Function to check the data, and confirm if its whitelisted.
//05 00 00 00 - ID
//02 00 00 00 - version
//00 00 00 00 - header
//28 00 00 00 - length
//24 FD 00 00 - checksum
//00 00 00 00 - headerDepValue
//00 00 00 00 - headerExtended
//above raw data needs to be parsed, and to be reached for the
//Tx code.
//0E 00 00 00  --- > Tx ON code is 4 bytes and value is 14 (TLV 2).
//E4 00 00 00  --- > PHY ON code is 4 bytes and value is 224 (TLV 2).
//data recieved to the wlandriver starts for the TLV2 Stream Header
//Whose value is for 28 bytes, the first byte is 5 (fixed)
QDF_STATUS vos_is_tcmd_data_white_listed(uint8_t *data, uint32_t len)
{
    //CMD_TX  = Tx command
    //CMD_PHY = Phy command
    uint8_t whitelist_read_tx[]     = {0x0E, 0x00, 0x00, 0x00};
    uint8_t whitelist_read_tx_phy[] = {0xE4, 0x00, 0x00, 0x00};
    uint8_t whitelist_read_tx_phy_off[] = {0xA6, 0x00, 0x00, 0x00};
    uint8_t whitelist_read_tx_off[]     = {0x35, 0x00, 0x00, 0x00};
    uint8_t whitelist_read_Rx_on[]      = {0x70, 0x00, 0x00, 0x00};
    uint8_t whitelist_read_Rx_stop[]    = {0x86, 0x00, 0x00, 0x00};

    //enable phy command
    if (!qdf_mem_cmp((data + WLAN_FTM_OPCODE_PHY),
                          whitelist_read_tx_phy, sizeof(whitelist_read_tx_phy)))
        return QDF_STATUS_SUCCESS;

    //enable Txon command
    if (!qdf_mem_cmp((data + WLAN_FTM_OPCODE_TX_ON),
                          whitelist_read_tx, sizeof(whitelist_read_tx)))
        return QDF_STATUS_SUCCESS;

    if ((!qdf_mem_cmp((data + WLAN_FTM_OPCODE_PHY),
                  whitelist_read_tx_phy_off, sizeof(whitelist_read_tx_phy_off)))
                    && (!qdf_mem_cmp((data + WLAN_FTM_OPCODE_DATA),
                         whitelist_read_tx_off, sizeof(whitelist_read_tx_off))))
        return QDF_STATUS_SUCCESS;

    if (!qdf_mem_cmp((data + WLAN_FTM_OPCODE_PHY),
                          whitelist_read_tx_phy, sizeof(whitelist_read_tx_phy)))
        return QDF_STATUS_SUCCESS;

    if ((!qdf_mem_cmp((data + WLAN_FTM_OPCODE_PHY),
                  whitelist_read_tx_phy_off, sizeof(whitelist_read_tx_phy_off)))
                  && (!qdf_mem_cmp((data + WLAN_FTM_OPCODE_DATA),
                           whitelist_read_Rx_on, sizeof(whitelist_read_Rx_on))))
        return QDF_STATUS_SUCCESS;

    if ((!qdf_mem_cmp((data + WLAN_FTM_OPCODE_PHY),
                  whitelist_read_tx_phy_off, sizeof(whitelist_read_tx_phy_off)))
                  && (!qdf_mem_cmp((data + WLAN_FTM_OPCODE_DATA),
                       whitelist_read_Rx_stop, sizeof(whitelist_read_Rx_stop))))
        return QDF_STATUS_SUCCESS;

    //black list all other commands
    return QDF_STATUS_E_PERM;
}

