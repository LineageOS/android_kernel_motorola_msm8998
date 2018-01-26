/*
 * Copyright (C) 2016 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/workqueue.h>

#include "apba.h"
#include "kernel_ver.h"
#include "cust_kernel_ver.h"
#include "mhb_protocol.h"
#include "mods_nw.h"
#include "mods_protocols.h"
#include "mods_uart.h"
#include "mods_uart_pm.h"
#include "muc.h"

#define MAX_PARTITION_NAME           (16)

#define APBA_FIRMWARE_PARTITION      ("apba")

#define APBA_FIRMWARE_UNIPRO_MID     (0x00000126)
#define APBA_FIRMWARE_UNIPRO_PID_ES2 (0x00001000)
#define APBA_FIRMWARE_UNIPRO_PID     (0x00001001)
#define APBA_FIRMWARE_ARA_VID        (0xfed70128)
#define APBA_FIRMWARE_ARA_PID        (0xfffe0001)
#define APBA_FIRMWARE_STAGE          (2)
#define APBA_FIRMWARE_NAME_LEN       (48)

#define FFFF_HEADER_SIZE          (4096)
#define FFFF_SENTINEL             "FlashFormatForFW"
#define FFFF_SENTINEL_LENGTH      (16)
#define FFFF_TIMESTAMP_LENGTH     (16)
#define FFFF_NAME_LENGTH          (48)
#define FFFF_RESERVED_LENGTH      (16)
#define FFFF_ELEMENT_CLASS_LENGTH (3)
#define FFFF_ELEMENT_LENGTH       (20)
#define FFFF_ELEMENTS             (198)

#define FFFF_ELEMENT_TYPE_STAGE_2_FW  (1)
#define FFFF_ELEMENT_TYPE_END         (0xFE)

#if FFFF_HEADER_SIZE > PAGE_SIZE
#error 'This should not happen'
#endif

#pragma pack(push, 1)
typedef struct {
	u8 type;
	u8 class[FFFF_ELEMENT_CLASS_LENGTH];
	u32 id;
	u32 length;
	u32 offset;
	u32 generation;
} ffff_element;

typedef struct {
	u8 leading_sentinel[FFFF_SENTINEL_LENGTH];
	u8 timestamp[FFFF_TIMESTAMP_LENGTH];
	char name[FFFF_NAME_LENGTH];
	u32 flash_capacity;
	u32 erase_size;
	u32 header_size;
	u32 flash_image_length;
	u32 header_generation;
	u8 reserved[FFFF_RESERVED_LENGTH];
	ffff_element element[FFFF_ELEMENTS];
	u32 padding;
	u8 trailing_sentinel[FFFF_SENTINEL_LENGTH];
} ffff_header;
#pragma pack(pop)

#define TFTF_OFFSET           (FFFF_HEADER_SIZE * 2)
#define TFTF_HEADER_SIZE      (512)
#define TFTF_SENTINEL         "TFTF"
#define TFTF_SENTINEL_LENGTH  (4)
#define TFTF_TIMESTAMP_LENGTH (16)
#define TFTF_NAME_LENGTH      (48)
#define TFTF_RESERVED_LENGTH  (12)
#define TFTF_SECTION_LENGTH   (20)
#define TFTF_SECTION_CLASS_LENGTH (3)
#define TFTF_SECTIONS         (20)

#pragma pack(push, 1)
typedef struct {
	u8 sentinel[TFTF_SENTINEL_LENGTH];
	u32 header_size;
	u8 timestamp[TFTF_TIMESTAMP_LENGTH];
	char name[TFTF_NAME_LENGTH];
	u32 package_type;
	u32 start_offset;
	u32 unipro_mid;
	u32 unipro_pid;
	u32 ara_vid;
	u32 ara_pid;
	u8 reserved[TFTF_RESERVED_LENGTH];
	u32 version;
	u8 reserved_sections[TFTF_SECTIONS][TFTF_SECTION_LENGTH];
} tftf_header;
#pragma pack(pop)

#define APBA_NUM_GPIOS (8)
#define APBA_MAX_SEQ   (APBA_NUM_GPIOS*3*2)

#define APBE_RESET_DELAY (250)

struct apba_seq {
	u32 val[APBA_MAX_SEQ];
	size_t len;
};

struct apba_ctrl {
	struct device *dev;
	struct clk *mclk;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state_default;
	struct pinctrl_state *pinctrl_state_active;
	u32 unipro_mid;
	u32 unipro_pid;
	u32 ara_vid;
	u32 ara_pid;
	u32 fw_version;
	char fw_package_name[TFTF_NAME_LENGTH];
	char firmware_name[APBA_FIRMWARE_NAME_LEN];
	int gpio_cnt;
	int gpios[APBA_NUM_GPIOS];
	const char *gpio_labels[APBA_NUM_GPIOS];
	int int_index;
	int irq;
	struct apba_seq enable_preclk_seq;
	struct apba_seq enable_postclk_seq;
	struct apba_seq disable_seq;
	struct apba_seq wake_assert_seq;
	struct apba_seq wake_deassert_seq;
	struct apba_seq flash_start_seq;
	struct apba_seq flash_end_seq;
	void *mods_uart;
	int desired_on;
	int on;
	struct mutex log_mutex;
	struct completion comp;
	struct completion apbe_log_comp;
	struct completion baud_comp;
	struct completion mode_comp;
	struct completion unipro_comp;
	struct completion unipro_stats_comp;
	struct completion fw_callback;
	uint8_t master_intf;
	uint8_t mode;
	bool flash_dev_populated;
	uint32_t apbe_status;
	uint32_t last_unipro_value;
	uint32_t last_unipro_status;
	uint32_t unipro_stats[32];
	struct notifier_block attach_nb;
	unsigned long present;
	struct workqueue_struct *wq;
	struct mhb_diag_id_not apba_ids;
	struct mhb_diag_id_not apbe_ids;
} *g_ctrl;

#define APBA_LOG_SIZE	SZ_16K
static DEFINE_KFIFO(apba_log_fifo, char, APBA_LOG_SIZE);

#define APBE_LOG_SIZE	SZ_16K
static DEFINE_KFIFO(apbe_log_fifo, char, APBE_LOG_SIZE);

/* used as temporary buffer to pop out content from FIFOs */
static char fifo_overflow[MHB_MAX_MSG_SIZE];

#define APBA_LOG_REQ_TIMEOUT	1000 /* ms */
#define APBE_LOG_REQ_TIMEOUT	1000 /* ms */
#define APBA_MODE_REQ_TIMEOUT	1000 /* ms */
#define APBA_BAUD_REQ_TIMEOUT	1000 /* ms */
#define APBA_UNIPRO_REQ_TIMEOUT	1000 /* ms */

static struct work_struct apba_disable_work;
static struct work_struct apba_enable_work;
static struct work_struct apba_dettach_work;

static void apba_send_kobj_uevent(const char *event)
{
	struct kobj_uevent_env *env;

	env = kzalloc(sizeof(*env), GFP_KERNEL);
	if (!env)
		return;

	add_uevent_var(env, event);
	kobject_uevent_env(&g_ctrl->dev->kobj, KOBJ_CHANGE, env->envp);
	kfree(env);
}

static void apba_handle_pm_status_not(struct mhb_hdr *hdr, uint8_t *payload,
		size_t len)
{
	struct mhb_pm_status_not *not;

	if (!g_ctrl || !g_ctrl->master_intf)
		return;

	if (len != sizeof(*not))
		return;

	not = (struct mhb_pm_status_not *)payload;
	g_ctrl->apbe_status = le32_to_cpu(not->status);

	switch (g_ctrl->apbe_status) {
	case MHB_PM_STATUS_PEER_NONE:
		/* ignore */
		break;
	case MHB_PM_STATUS_PEER_ON:
		pr_info("APBE: on\n");
		mods_slave_ctrl_power(g_ctrl->master_intf,
			MB_CONTROL_SLAVE_POWER_ON, MB_CONTROL_SLAVE_MASK_APBE);
		apba_send_kobj_uevent("APBA_EVENT=BOOT_COMPLETED");
		break;
	case MHB_PM_STATUS_PEER_RESET:
		pr_info("APBE: reset\n");
		mods_slave_ctrl_power(g_ctrl->master_intf,
			MB_CONTROL_SLAVE_POWER_OFF, MB_CONTROL_SLAVE_MASK_APBE);
		msleep(APBE_RESET_DELAY);
		mods_slave_ctrl_power(g_ctrl->master_intf,
			MB_CONTROL_SLAVE_POWER_ON, MB_CONTROL_SLAVE_MASK_APBE);
		break;
	case MHB_PM_STATUS_PEER_CONNECTED:
		pr_info("APBE: connected\n");
		break;
	case MHB_PM_STATUS_PEER_DISCONNECTED:
		pr_info("APBE: disconnected\n");
		mods_slave_ctrl_power(g_ctrl->master_intf,
			MB_CONTROL_SLAVE_POWER_OFF, MB_CONTROL_SLAVE_MASK_APBE);
		apba_send_kobj_uevent("APBA_EVENT=PEER_DISCONNECTED");
		break;
	default:
		pr_err("%s: Invalid reason=%d.\n", __func__, not->status);
		break;
	}
}

/* PM */
int apba_send_pm_wake_rsp(void)
{
	int ret;
	struct mhb_hdr rsp_hdr;

	if (!g_ctrl || !g_ctrl->mods_uart)
		return -ENODEV;

	memset(&rsp_hdr, 0, sizeof(rsp_hdr));
	rsp_hdr.addr = MHB_ADDR_PM;
	rsp_hdr.type = MHB_TYPE_PM_WAKE_RSP;

	ret = mods_uart_send(g_ctrl->mods_uart, &rsp_hdr, NULL, 0,
		UART_PM_FLAG_WAKE_ACK);
	if (ret)
		pr_err("%s: failed to send\n", __func__);

	return ret;
}

int apba_send_pm_sleep_req(void)
{
	int ret;
	struct mhb_hdr req_hdr;

	if (!g_ctrl || !g_ctrl->mods_uart)
		return -ENODEV;

	memset(&req_hdr, 0, sizeof(req_hdr));
	req_hdr.addr = MHB_ADDR_PM;
	req_hdr.type = MHB_TYPE_PM_SLEEP_REQ;

	ret = mods_uart_send(g_ctrl->mods_uart, &req_hdr, NULL, 0,
		UART_PM_FLAG_SLEEP_IND);
	if (ret)
		pr_err("%s: failed to send\n", __func__);

	return ret;
}

static int apba_send_pm_sleep_rsp(void)
{
	int ret;
	struct mhb_hdr rsp_hdr;

	if (!g_ctrl || !g_ctrl->mods_uart)
		return -ENODEV;

	memset(&rsp_hdr, 0, sizeof(rsp_hdr));
	rsp_hdr.addr = MHB_ADDR_PM;
	rsp_hdr.type = MHB_TYPE_PM_SLEEP_RSP;

	ret = mods_uart_send(g_ctrl->mods_uart, &rsp_hdr, NULL, 0,
		UART_PM_FLAG_SLEEP_ACK);
	if (ret)
		pr_err("%s: failed to send\n", __func__);

	return ret;
}

static void apba_handle_pm_message(struct mhb_hdr *hdr, uint8_t *payload,
		size_t len)
{
	switch (hdr->type) {
	case MHB_TYPE_PM_WAKE_RSP:
		/* APBA had acknowledged wake interrupt */
		mods_uart_pm_handle_pm_wake_rsp(g_ctrl->mods_uart);
		break;
	case MHB_TYPE_PM_SLEEP_RSP:
		/* APBA had acknowledged to our sleep indication */
		break;
	case MHB_TYPE_PM_SLEEP_REQ:
		/* APBA is going to sleep */
		apba_send_pm_sleep_rsp();
		break;
	case MHB_TYPE_PM_STATUS_NOT:
		apba_handle_pm_status_not(hdr, payload, len);
		break;
	default:
		pr_err("%s: Invalid type=0x%02x.\n", __func__, hdr->type);
		break;
	}
}

/* UART */
static void apba_handle_uart_config_rsp(struct mhb_hdr *hdr, uint8_t *payload,
		size_t len)
{
	int ret;
	struct mhb_uart_config_rsp *rsp;

	if (len != sizeof(*rsp))
		goto done;

	rsp = (struct mhb_uart_config_rsp *)payload;
	rsp->baud = le32_to_cpu(rsp->baud);

	if (hdr->result != MHB_RESULT_SUCCESS)
		goto done;

	ret = mods_uart_set_baud(g_ctrl->mods_uart, rsp->baud);
	if (ret)
		pr_err("%s: baud update failed: %d\n", __func__, ret);

done:
	complete(&g_ctrl->baud_comp);
}

static void apba_handle_uart_message(struct mhb_hdr *hdr, uint8_t *payload,
		size_t len)
{
	switch (hdr->type) {
	case MHB_TYPE_UART_CONFIG_RSP:
		apba_handle_uart_config_rsp(hdr, payload, len);
		break;
	default:
		pr_err("%s: Invalid type=0x%02x.\n", __func__, hdr->type);
		break;
	}
}

/* UniPro */
static int apba_send_unipro_read_attr_req(uint16_t attribute, uint16_t selector,
		uint8_t peer)
{
	int ret;
	struct mhb_hdr req_hdr;
	struct mhb_unipro_read_attr_req req;

	if (!g_ctrl->mods_uart)
		return -ENODEV;

	memset(&req_hdr, 0, sizeof(req_hdr));
	req_hdr.addr = MHB_ADDR_UNIPRO;
	req_hdr.type = MHB_TYPE_UNIPRO_READ_ATTR_REQ;

	req.attribute = cpu_to_le16(attribute);
	req.selector = cpu_to_le16(selector);
	req.peer = peer;

	ret = mods_uart_send(g_ctrl->mods_uart, &req_hdr, (uint8_t *)&req,
		sizeof(req), 0);
	if (ret)
		pr_err("%s: failed to send\n", __func__);

	return ret;
}

static int apba_send_unipro_write_attr_req(uint16_t attribute, uint16_t selector,
		uint8_t peer, uint32_t value)
{
	int ret;
	struct mhb_hdr req_hdr;
	struct mhb_unipro_write_attr_req req;

	if (!g_ctrl->mods_uart)
		return -ENODEV;

	memset(&req_hdr, 0, sizeof(req_hdr));
	req_hdr.addr = MHB_ADDR_UNIPRO;
	req_hdr.type = MHB_TYPE_UNIPRO_WRITE_ATTR_REQ;

	req.attribute = cpu_to_le16(attribute);
	req.selector = cpu_to_le16(selector);
	req.peer = peer;
	req.value = cpu_to_le32(value);

	ret = mods_uart_send(g_ctrl->mods_uart, &req_hdr, (uint8_t *)&req,
		sizeof(req), 0);
	if (ret)
		pr_err("%s: failed to send\n", __func__);

	return ret;
}

static int apba_send_unipro_gear_req(uint8_t tx, uint8_t rx, uint8_t pwrmode,
		uint8_t series)
{
	int ret;
	struct mhb_hdr req_hdr;
	struct mhb_unipro_control_req req;

	if (!g_ctrl->mods_uart)
		return -ENODEV;

	memset(&req_hdr, 0, sizeof(req_hdr));
	req_hdr.addr = MHB_ADDR_UNIPRO;
	req_hdr.type = MHB_TYPE_UNIPRO_CONTROL_REQ;

	req.gear.tx = tx;
	req.gear.rx = rx;
	req.gear.pwrmode = pwrmode;
	req.gear.series = series;

	ret = mods_uart_send(g_ctrl->mods_uart, &req_hdr, (uint8_t *)&req,
		sizeof(req), 0);
	if (ret)
		pr_err("%s: failed to send\n", __func__);

	return ret;
}

static void apba_handle_unipro_gear_rsp(struct mhb_hdr *hdr,
		uint8_t *payload, size_t len)
{
	g_ctrl->last_unipro_status = hdr->result;
	complete(&g_ctrl->unipro_comp);
}

static void apba_handle_unipro_read_attr_rsp(struct mhb_hdr *hdr,
		uint8_t *payload, size_t len)
{
	struct mhb_unipro_read_attr_rsp *rsp;

	if (len != sizeof(*rsp))
		return;

	rsp = (struct mhb_unipro_read_attr_rsp *)payload;

	g_ctrl->last_unipro_status = hdr->result;
	g_ctrl->last_unipro_value = (hdr->result == MHB_RESULT_SUCCESS) ?
		le32_to_cpu(rsp->value) : 0;

	complete(&g_ctrl->unipro_comp);
}

static void apba_handle_unipro_write_attr_rsp(struct mhb_hdr *hdr,
		uint8_t *payload, size_t len)
{
	g_ctrl->last_unipro_status = hdr->result;
	complete(&g_ctrl->unipro_comp);
}

static int apba_send_unipro_stats_req(void)
{
	int ret;
	struct mhb_hdr req_hdr;

	if (!g_ctrl->mods_uart)
		return -ENODEV;

	memset(&req_hdr, 0, sizeof(req_hdr));
	req_hdr.addr = MHB_ADDR_UNIPRO;
	req_hdr.type = MHB_TYPE_UNIPRO_STATS_REQ;

	ret = mods_uart_send(g_ctrl->mods_uart, &req_hdr, NULL,	0, 0);
	if (ret)
		pr_err("%s: failed to send\n", __func__);

	return ret;
}

static void apba_handle_unipro_stats_not(struct mhb_hdr *hdr,
                uint8_t *payload, size_t len)
{
        uint32_t *src = (uint32_t *)payload;
        uint32_t *dst = g_ctrl->unipro_stats;

        len = min(len, sizeof(g_ctrl->unipro_stats));
        len /= sizeof(uint32_t);

        while (len--)
                *dst++ += le32_to_cpu(*src++);
}

static void apba_handle_unipro_stats_rsp(struct mhb_hdr *hdr,
		uint8_t *payload, size_t len)
{
	if (hdr->result == MHB_RESULT_SUCCESS)
		apba_handle_unipro_stats_not(hdr, payload, len);

	if (!completion_done(&g_ctrl->unipro_stats_comp))
		complete(&g_ctrl->unipro_stats_comp);
}

static void apba_handle_unipro_message(struct mhb_hdr *hdr, uint8_t *payload,
		size_t len)
{
	switch (hdr->type) {
	case MHB_TYPE_UNIPRO_CONTROL_RSP:
		apba_handle_unipro_gear_rsp(hdr, payload, len);
		break;
	case MHB_TYPE_UNIPRO_READ_ATTR_RSP:
		apba_handle_unipro_read_attr_rsp(hdr, payload, len);
		break;
	case MHB_TYPE_UNIPRO_WRITE_ATTR_RSP:
		apba_handle_unipro_write_attr_rsp(hdr, payload, len);
		break;
	case MHB_TYPE_UNIPRO_CONFIG_RSP:
	case MHB_TYPE_UNIPRO_STATUS_RSP:
		/* ignore */
		break;
	case MHB_TYPE_UNIPRO_STATS_RSP:
		apba_handle_unipro_stats_rsp(hdr, payload, len);
		break;
	case MHB_TYPE_UNIPRO_STATS_NOT:
		apba_handle_unipro_stats_not(hdr, payload, len);
		break;
	default:
		pr_err("%s: Invalid type=0x%02x.\n", __func__, hdr->type);
		break;
	}
}

/* Diag */
static void save_log_data(struct kfifo *fifo, uint8_t *payload, size_t len)
{
	int overflow;

	mutex_lock(&g_ctrl->log_mutex);

	overflow = kfifo_len(fifo) + len - kfifo_size(fifo);
	if (overflow > 0) {
		/* pop out from older content if buffer is full */
		overflow = kfifo_out(fifo, fifo_overflow,
				     min(overflow, MHB_MAX_MSG_SIZE));
	}
	kfifo_in(fifo, payload, len);

	mutex_unlock(&g_ctrl->log_mutex);
}

static void apba_handle_diag_mode_rsp(struct mhb_hdr *hdr, uint8_t *payload,
		size_t len)
{
	complete(&g_ctrl->mode_comp);
}

static void apba_handle_diag_id_not(struct mhb_hdr *hdr, uint8_t *payload,
		size_t len)
{
	struct mhb_diag_id_not *src = (struct mhb_diag_id_not *)payload;
        struct mhb_diag_id_not *dest =
	    (hdr->addr == MHB_ADDR_DIAG ? &g_ctrl->apba_ids : &g_ctrl->apbe_ids);

	if (len == sizeof(struct mhb_diag_id_not)) {
		dest->unipro_mid = le32_to_cpu(src->unipro_mid);
		dest->unipro_pid = le32_to_cpu(src->unipro_pid);
		dest->vid = le32_to_cpu(src->vid);
		dest->pid = le32_to_cpu(src->pid);
		dest->major_version = le16_to_cpu(src->major_version);
		dest->minor_version = le16_to_cpu(src->minor_version);
		memcpy(dest->build, src->build, sizeof(src->build));
	}
}

static void apba_handle_diag_message(struct mhb_hdr *hdr, uint8_t *payload,
		size_t len)
{
	switch (hdr->type) {
	case MHB_TYPE_DIAG_LOG_RSP:
	case MHB_TYPE_DIAG_LOG_NOT:
		if (hdr->addr == MHB_ADDR_DIAG) {
			save_log_data((struct kfifo *)&apba_log_fifo,
				      payload, len);
			if (!completion_done(&g_ctrl->comp))
				complete(&g_ctrl->comp);
		} else if (hdr->addr == MHB_ADDR_PEER_DIAG) {
			save_log_data((struct kfifo *)&apbe_log_fifo,
				      payload, len);
			if (!completion_done(&g_ctrl->apbe_log_comp))
				complete(&g_ctrl->apbe_log_comp);
		}
		break;
	case MHB_TYPE_DIAG_MODE_RSP:
		apba_handle_diag_mode_rsp(hdr, payload, len);
		break;
	case MHB_TYPE_DIAG_ID_NOT:
		apba_handle_diag_id_not(hdr, payload, len);
		break;
	default:
		pr_err("%s: Invalid type=0x%02x.\n", __func__, hdr->type);
		break;
	}
}

void apba_handle_message(struct mhb_hdr *hdr, uint8_t *payload, size_t len)
{
	__u8 func = (hdr->addr & MHB_FUNC_MASK) >> MHB_FUNC_SHIFT;

	if (!g_ctrl)
		return;

	switch (func) {
	case MHB_FUNC_PM:
		apba_handle_pm_message(hdr, payload, len);
		break;
	case MHB_FUNC_UART:
		apba_handle_uart_message(hdr, payload, len);
		break;
	case MHB_FUNC_UNIPRO:
		apba_handle_unipro_message(hdr, payload, len);
		break;
	case MHB_FUNC_DIAG:
		apba_handle_diag_message(hdr, payload, len);
		break;
	default:
		pr_err("%s: Invalid func=0x%02x.\n", __func__, func);
		break;
	}
}

static int apba_send_uart_config_req(unsigned long val)
{
	int ret;
	struct mhb_hdr req_hdr;
	struct mhb_uart_config_req req;

	if (!g_ctrl->mods_uart)
		return -ENODEV;

	memset(&req_hdr, 0, sizeof(req_hdr));
	req_hdr.addr = MHB_ADDR_UART;
	req_hdr.type = MHB_TYPE_UART_CONFIG_REQ;

	req.baud = cpu_to_le32(val);

	ret = mods_uart_send(g_ctrl->mods_uart, &req_hdr,
		(uint8_t *)&req, sizeof(req), 0);
	if (ret)
		pr_err("%s: failed to send req\n", __func__);

	return ret;
}

static int apba_send_diag_mode_req(__u32 mode)
{
	int ret;
	struct mhb_hdr req_hdr;
	struct mhb_diag_mode_req req;

	if (!g_ctrl->mods_uart)
		return -ENODEV;

	memset(&req_hdr, 0, sizeof(req_hdr));
	req_hdr.addr = MHB_ADDR_DIAG;
	req_hdr.type = MHB_TYPE_DIAG_MODE_REQ;

	req.mode = cpu_to_le32(mode);

	ret = mods_uart_send(g_ctrl->mods_uart, &req_hdr,
		(uint8_t *)&req, sizeof(req), 0);
	if (ret)
		pr_err("%s: failed to send req\n", __func__);

	return ret;
}

static int apba_send_diag_log_req(uint8_t addr)
{
	int ret;
	struct mhb_hdr req_hdr;

	if (!g_ctrl->mods_uart)
		return -ENODEV;

	memset(&req_hdr, 0, sizeof(req_hdr));
	req_hdr.addr = addr;
	req_hdr.type = MHB_TYPE_DIAG_LOG_REQ;

	ret = mods_uart_send(g_ctrl->mods_uart, &req_hdr,
		NULL, 0, 0);
	if (ret)
		pr_err("%s: failed to send log req\n", __func__);

	return ret;
}

static int apba_mtd_erase(struct mtd_info *mtd_info,
	 unsigned int start, unsigned int len)
{
	int err;
	struct erase_info ei = {0};

	ei.addr = start;
	ei.len = len;
	ei.mtd = mtd_info;
	err = mtd_erase(mtd_info, &ei);
	return err;
}

static struct mtd_info * apba_init_mtd_module(const char *partition_name)
{
	struct mtd_info *mtd_info;
	int num;

	for (num = 0; num < 16; num++) {
		mtd_info = get_mtd_device(NULL, num);
		if (IS_ERR(mtd_info)) {
			pr_debug("%s: No device for num %d\n", __func__, num);
			continue;
		}

		if (mtd_info->type == MTD_ABSENT) {
			put_mtd_device(mtd_info);
			continue;
		}

		if (strcmp(mtd_info->name, partition_name)) {
			put_mtd_device(mtd_info);
			continue;
		}

		pr_debug("%s: MTD name: %s\n", __func__, mtd_info->name);
		pr_debug("%s: MTD type: %d\n", __func__, mtd_info->type);
		pr_debug("%s: MTD total size : %ld bytes\n", __func__,
			 (long)mtd_info->size);
		pr_debug("%s: MTD erase size : %ld bytes\n", __func__,
			 (long)mtd_info->erasesize);

		return mtd_info;
	}

	return NULL;
}

static int apba_parse_seq(struct device *dev, const char *name,
	struct apba_seq *seq)
{
	int ret;
	int cnt = 0;
	struct property *pp = of_find_property(dev->of_node, name, &cnt);

	cnt /= sizeof(u32);
	if (!pp || cnt == 0 || cnt > seq->len || cnt % 3) {
		pr_err("%s: error reading property %s, cnt = %d\n",
			__func__, name, cnt);
		ret = -EINVAL;
	} else {
		ret = of_property_read_u32_array(dev->of_node, name,
			seq->val, cnt);
		if (ret) {
			pr_err("%s: unable to read %s, ret = %d\n",
				__func__, name, ret);
		} else {
			seq->len = cnt;
		}
	}

	return ret;
}

static void apba_seq(struct apba_ctrl *ctrl, struct apba_seq *seq)
{
	size_t i;

	for (i = 0; i < seq->len; i += 3) {
		u32 index = seq->val[i];
		int value = (int)seq->val[i+1];
		unsigned long delay = (unsigned long)seq->val[i+2];

		/* Set a gpio (if valid). */
		if (index < ARRAY_SIZE(ctrl->gpios)) {
			int gpio = ctrl->gpios[index];

			if (gpio_is_valid(gpio)) {
				pr_debug("%s: set gpio=%d, value=%u\n",
					__func__, gpio, value);
				gpio_set_value(gpio, value);
			}
		}

		/* Delay (if valid). */
		if (delay) {
			usleep_range(delay * 1000, delay * 1000);
			pr_debug("%s: delay=%lu\n",
				__func__, delay);
		}
	}
}

static void apba_on(struct apba_ctrl *ctrl, bool on)
{
	if (on == ctrl->on) {
		if (on)
			pr_warn("%s: already  on\n", __func__);

		return;
	}

	pr_info("%s: %s\n", __func__, on ? "on" : "off");

	if (on) {
		mods_ext_bus_vote(true);
		apba_seq(ctrl, &ctrl->enable_preclk_seq);
		if (clk_prepare_enable(g_ctrl->mclk)) {
			dev_err(g_ctrl->dev, "%s: failed to prepare clock.\n",
				__func__);
			apba_seq(ctrl, &ctrl->disable_seq);
			mods_ext_bus_vote(false);
			return;
		}

		if (ctrl->mods_uart)
			mods_uart_open(ctrl->mods_uart);

		apba_seq(ctrl, &ctrl->enable_postclk_seq);
		enable_irq(ctrl->irq);
	} else {
		ctrl->mode = 0;
		disable_irq(ctrl->irq);
		if (ctrl->mods_uart)
			mods_uart_close(ctrl->mods_uart);

		clk_disable_unprepare(g_ctrl->mclk);
		apba_seq(ctrl, &ctrl->disable_seq);
		mods_ext_bus_vote(false);
	}
	ctrl->on = on;
}

static int apba_attach_notifier(struct notifier_block *nb,
		unsigned long now_present, void *not_used)
{
	struct apba_ctrl *ctrl = container_of(nb, struct apba_ctrl, attach_nb);

	if (now_present != ctrl->present) {
		ctrl->present = now_present;

		flush_work(&apba_dettach_work);
		if (!now_present) {
			pr_debug("%s: disable apba\n", __func__);
			queue_work(ctrl->wq, &apba_dettach_work);
		}
	}

	return NOTIFY_OK;
}

static void populate_transports_node(struct apba_ctrl *ctrl)
{
	struct device_node *np;
	struct device_node *spi_np;
	struct platform_device *pdev;

	np = of_find_node_by_name(ctrl->dev->of_node, "transports");
	if (!np) {
		dev_warn(ctrl->dev, "transports node not present\n");
		return;
	}

	spi_np = of_find_compatible_node(np, NULL, "moto,apba-spi-transfer");
	if (!spi_np) {
		dev_warn(ctrl->dev, "SPI transport device not present\n");
		goto put_np;
	}

	dev_dbg(ctrl->dev, "%s: creating platform device\n", __func__);
	pdev = of_platform_device_create(spi_np, NULL, ctrl->dev);
	if (!pdev) {
		dev_warn(ctrl->dev, "failed to populate transport devices\n");
		goto put_spi_np;
	}

	of_node_set_flag(ctrl->dev->of_node, OF_POPULATED_BUS);
	ctrl->flash_dev_populated = true;

put_spi_np:
	of_node_put(spi_np);
put_np:
	of_node_put(np);
}

/*
 * for flash on:
 *   Configure SPI interface pin control for use of the SPI interface to
 *    the flash part.
 *   Set the PMIC GPIO and any other GPIOs needed for AP to interface with
 *    the flash part on the SPI interface.
 *   Search device tree entry for a transport node which holds the SPI
 *    interface info.
 *   If the transport node is found, probe the flash device on the SPI
 *    interface.
 *
 * for flash off:
 *   reverse the above conditions.
 */
static void apba_flash_on(struct apba_ctrl *ctrl, bool on)
{
	int ret;

	if (on) {
		if (!IS_ERR(ctrl->pinctrl_state_active)) {
			dev_dbg(ctrl->dev, "%s: Pinctrl set active\n", __func__);
			ret = pinctrl_select_state(ctrl->pinctrl,
						   ctrl->pinctrl_state_active);
			if (ret)
				dev_err(ctrl->dev,
					"%s: Pinctrl set failed %d\n",
					__func__, ret);
		}

		apba_seq(ctrl, &ctrl->flash_start_seq);

		/* Register SPI transport for shared muc_spi and spi_flash */
		muc_register_spi_flash();

		populate_transports_node(ctrl);
	} else {
		if (ctrl->flash_dev_populated) {
			of_platform_depopulate(ctrl->dev);
			of_node_clear_flag(ctrl->dev->of_node,
						OF_POPULATED_BUS);
			ctrl->flash_dev_populated = false;
		}

		apba_seq(ctrl, &ctrl->flash_end_seq);

		muc_deregister_spi_flash();

		dev_dbg(ctrl->dev, "%s: Pinctrl set default\n", __func__);
		ret = pinctrl_select_state(ctrl->pinctrl,
					   ctrl->pinctrl_state_default);
		if (ret)
			dev_err(ctrl->dev,
				"%s: Pinctrl set default failed %d\n",
				__func__, ret);
	}
}

static int apba_erase_partition(struct apba_ctrl *ctrl, const char *partition)
{
	struct mtd_info *mtd_info;
	int err;

	if (!ctrl)
		return -EINVAL;

	/* Disable the APBA so that it does not access the flash. */
	apba_on(ctrl, false);

	apba_flash_on(ctrl, true);

	mtd_info = apba_init_mtd_module(partition);
	if (!mtd_info) {
		pr_err("%s: mtd init module failed for %s\n",
			__func__, partition);
		err = -ENODEV;
		goto no_mtd;
	}

	/* Erase the flash */
	err = apba_mtd_erase(mtd_info, 0, mtd_info->size);
	if (err < 0) {
		pr_err("%s: mtd erase failed for %s, err=%d\n",
			__func__, partition, err);
		goto cleanup;
	}

	pr_debug("%s: %s complete\n", __func__, partition);

cleanup:
	put_mtd_device(mtd_info);

no_mtd:
	apba_flash_on(ctrl, false);
	if (ctrl->desired_on)
		apba_on(ctrl, true);

	return err;
}

static ssize_t erase_partition_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct apba_ctrl *ctrl = platform_get_drvdata(pdev);
	char partition[MAX_PARTITION_NAME + 1];
	size_t partition_name_sz;
	int err;

	partition_name_sz = count;
	if (partition_name_sz && buf[partition_name_sz - 1] == '\n')
		partition_name_sz--;

	if (!partition_name_sz || (partition_name_sz >= sizeof(partition))) {
		pr_err("%s: partition name too large %s\n",
			__func__, buf);
		return -EINVAL;
	}

	memcpy(partition, buf, partition_name_sz);
	partition[partition_name_sz] = 0;
	pr_debug("%s: partition=%s\n", __func__, partition);

	err = apba_erase_partition(ctrl, partition);
	if (err < 0)
		pr_err("%s: flashing erase err=%d\n", __func__, err);

	return err ? err : count;
}

static DEVICE_ATTR_WO(erase_partition);

/*
 * This function will compare the entire contents of the firmware file
 * described by fw against the flash contents read from the mtd device,
 * comparing one page at a time.
 * The flash contents will have two FFFF headers prepended before the TFTF
 * header, so this function reads from the flash after the FFFF headers.
 *
 * Return 0 if the firmware does not need to be flashed
 * Return !0 otherwise
 */
static int apba_compare_partition(struct apba_ctrl *ctrl,
	struct mtd_info *mtd_info,
	const struct firmware *fw)
{
	int err;
	tftf_header *fsfw = (tftf_header *)fw->data;
	unsigned long fwsize = fw->size;
	tftf_header *tftf;
	void *buf;
	size_t retlen = 0;
	/* Assume different */
	int compare_result = 1;
	unsigned long cur = 0;
	size_t readlen;

	tftf = kmalloc(PAGE_SIZE, GFP_KERNEL | GFP_DMA);
	if (!tftf)
		goto skip_compare;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL | GFP_DMA);
	if (!buf)
		goto cleanup;

	while (cur < fwsize) {
		readlen = min(fwsize - cur, PAGE_SIZE);

		compare_result = 1;
		err = mtd_read(mtd_info, TFTF_OFFSET + cur, readlen,
			       &retlen, buf);
		if (err < 0 || retlen < readlen) {
			pr_err("%s: mtd_read failure. err:%d, retlen:%zd\n",
			       __func__, err, retlen);
			goto cleanup;
		}

		/* Store away the page w/ TFTF header for later check */
		if (cur == 0)
			memcpy(tftf, buf, readlen);

		compare_result = memcmp(buf, fw->data + cur, readlen);

		if (compare_result)
			break;

		cur += readlen;
	}

	/* log header values only if header sentinal is valid */
	if (memcmp(tftf->sentinel, TFTF_SENTINEL, TFTF_SENTINEL_LENGTH)) {
		pr_debug("%s: flashed sentinel value not valid\n", __func__);
	} else {
		pr_debug("%s: flashed ts %s name %s type %d mid: %08x pid: %08x"\
			 " ara vid: %08x ara pid: %08x version %08x\n", __func__,
			 tftf->timestamp, tftf->name, tftf->package_type,
			 tftf->unipro_mid, tftf->unipro_pid,
			 tftf->ara_vid, tftf->ara_pid, tftf->version);

		ctrl->fw_version = tftf->version;
		strlcpy(ctrl->fw_package_name, tftf->name, TFTF_NAME_LENGTH);

		/* don't overwrite newer fw with older fw */
		if (tftf->version > fsfw->version) {
			compare_result = 0;
			goto cleanup;
		}
	}

	pr_debug("%s: file ts %s name %s type %d mid: %08x pid: %08x"\
		 " ara vid: %08x ara pid: %08x version: %08x comp: %d\n",
		__func__, fsfw->timestamp, fsfw->name, fsfw->package_type,
		fsfw->unipro_mid, fsfw->unipro_pid,
		fsfw->ara_vid, fsfw->ara_pid, fsfw->version, compare_result);

cleanup:
	kfree(tftf);
	kfree(buf);

skip_compare:
	return compare_result;
}

static void construct_ffff_header(ffff_header *header,
				  const struct mtd_info *mtd_info,
				  const struct firmware *fw)
{
	tftf_header *tftf_hdr = (tftf_header *)fw->data;

	memcpy(&header->leading_sentinel, FFFF_SENTINEL,
		sizeof(header->leading_sentinel));
	memcpy(&header->timestamp, tftf_hdr->timestamp,
		sizeof(header->timestamp));
	memcpy(&header->name, tftf_hdr->name, sizeof(header->name));

	header->flash_capacity = mtd_info->size;
	header->erase_size = mtd_info->erasesize;

	header->header_size = sizeof(*header);
	/* flash_image_length is used in the bootloader when validating
	 * element locations */
	header->flash_image_length = 2 * sizeof(*header) + fw->size;
	header->header_generation = 1;
	header->element[0].type = FFFF_ELEMENT_TYPE_STAGE_2_FW;
	/* currently assume that element class 0 (the default) is fine. */
	header->element[0].id = 1;
	header->element[0].length = fw->size;
	header->element[0].offset = TFTF_OFFSET;
	/* Boot rom and tools will use the latest generation of a given
	 * element and type. */
	header->element[0].generation = 0;

	header->element[1].type = FFFF_ELEMENT_TYPE_END;

	memcpy(&header->trailing_sentinel, FFFF_SENTINEL,
		sizeof(header->trailing_sentinel));
}

static int apba_flash_partition(struct apba_ctrl *ctrl,
	const char *partition, const struct firmware *fw)
{
	struct mtd_info *mtd_info;
	int err;
	size_t retlen = 0;
	int compare_result;
	void *buffer;
	const void *data;
	int offset;
	size_t count;

	if (!fw || !ctrl)
		return -EINVAL;

	if (memcmp(fw->data, TFTF_SENTINEL, TFTF_SENTINEL_LENGTH) ||
	    fw->size < PAGE_SIZE) {
		pr_err("%s: firmware invalid\n", __func__);
		return -EINVAL;
	}

	/* Disable the APBA so that it does not access the flash. */
	apba_on(ctrl, false);

	apba_flash_on(ctrl, true);

	mtd_info = apba_init_mtd_module(partition);
	if (!mtd_info) {
		pr_err("%s: mtd init module failed for %s\n",
			__func__, partition);
		err = -ENODEV;
		goto no_mtd;
	}

	/* Before erasing and flashing, compare the firmware and the partition
	 * If they match, skip the process.  If anything fails during the
	 * comparison, then flash.
	 */
	compare_result = apba_compare_partition(ctrl, mtd_info, fw);
	if (compare_result == 0) {
		pr_info("%s: firmware unchanged or newer, skipping flash\n",
			__func__);
		err = 0;
		goto cleanup;
	}

	/* Erase the flash */
	err = apba_mtd_erase(mtd_info, 0, mtd_info->size);
	if (err < 0) {
		pr_err("%s: mtd flash failed for %s, err=%d\n",
			__func__, partition, err);
		goto cleanup;
	}

	buffer = kzalloc(PAGE_SIZE, GFP_KERNEL | GFP_DMA);
	if (!buffer) {
		err = -ENOMEM;
		goto cleanup;
	}

	construct_ffff_header((ffff_header *)buffer, mtd_info, fw);

	/* Write the first copy of the FFFF header to the flash */
	err = mtd_write(mtd_info, 0, FFFF_HEADER_SIZE,
			&retlen, (const u_char *)buffer);
	if (err < 0) {
		pr_err("%s: write error %d\n", __func__, err);
		goto free_mem;
	}

	/* Write the second copy of the FFFF header to the flash */
	err = mtd_write(mtd_info, FFFF_HEADER_SIZE, FFFF_HEADER_SIZE,
			&retlen, (const u_char *)buffer);
	if (err < 0) {
		pr_err("%s: write error %d\n", __func__, err);
		goto free_mem;
	}

	/* Write the TFTF header and firmware from the firmware file
	   one page at a time to allow DMA to be used. */
	data = fw->data;
	offset = TFTF_OFFSET;
	count = fw->size;
	while (count) {
		size_t s = count > PAGE_SIZE ? PAGE_SIZE : count;

		memcpy(buffer, data, s);

		err = mtd_write(mtd_info, offset, s, &retlen, buffer);
		if (err < 0) {
			pr_err("%s: write error %d\n", __func__, err);
			goto free_mem;
		}

		data += s;
		offset += s;
		count -= s;
	}

	pr_debug("%s: %s write complete\n", __func__, partition);

	/* Since fw file version is now in flash, store that version */
	ctrl->fw_version = ((tftf_header *)fw->data)->version;
	strlcpy(ctrl->fw_package_name, ((tftf_header *)fw->data)->name,
		TFTF_NAME_LENGTH);

free_mem:
	kfree(buffer);

cleanup:
	put_mtd_device(mtd_info);

no_mtd:
	apba_flash_on(ctrl, false);
	if (ctrl->desired_on && !err)
		apba_on(ctrl, true);

	return err;
}

/*
 * Try to load the specified firmware file and flash it into the specified
 * partition.
 */
static int request_fw_and_flash(struct apba_ctrl *ctrl,
				const char *name, const char *partition)
{
	const struct firmware *fw = NULL;
	int err;

	pr_debug("%s: request firmware %s for parition %s\n",
		__func__, name, partition);
	err = request_firmware(&fw, name, ctrl->dev);
	if (err < 0) {
		pr_debug("%s: request firmware failed for %s, err=%d\n",
			__func__, name, err);
		return err;
	}

	if (!fw || !fw->size) {
		pr_err("%s: firmware invalid for %s\n",
			__func__, name);
		return -EINVAL;
	}

	/* Support loading fw for ES2 into es2_apba partition */
	err = apba_flash_partition(ctrl,
		((tftf_header *)fw->data)->unipro_pid ==
		  APBA_FIRMWARE_UNIPRO_PID_ES2 ? "es2_apba" : partition,
		fw);
	if (err < 0)
		pr_err("%s: flashing failed for %s partition %s, err=%d\n",
			__func__, name, partition, err);

	release_firmware(fw);

	return err;
}

/*
 * Given a specific firmware file name as input to this sys fs file,
 * attempt to load that firmware file and flash it in the apba
 * flash partition.
 */
static ssize_t flash_file_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct apba_ctrl *ctrl = platform_get_drvdata(pdev);
	/* Null-termination accounted for. */
	char fw_name[APBA_FIRMWARE_NAME_LEN];
	size_t fw_name_sz = count;
	int err;

	if (fw_name_sz && buf[fw_name_sz - 1] == '\n')
		fw_name_sz--;

	if (!fw_name_sz || (fw_name_sz >= sizeof(fw_name))) {
		pr_err("%s: firmware name too large %s\n",
			__func__, buf);
		return -EINVAL;
	}

	memcpy(fw_name, buf, fw_name_sz);
	fw_name[fw_name_sz] = 0;

	err = request_fw_and_flash(ctrl, fw_name, APBA_FIRMWARE_PARTITION);

	return err ? err : count;
}

static DEVICE_ATTR_WO(flash_file);

static ssize_t apba_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", g_ctrl->desired_on);
}

static ssize_t apba_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;
	else if (val != 0 && val != 1)
		return -EINVAL;

	/* serialize on/off operations through the work queue */
	if (val)
		queue_work(g_ctrl->wq, &apba_enable_work);
	else
		queue_work(g_ctrl->wq, &apba_disable_work);

	return count;
}

static DEVICE_ATTR_RW(apba_enable);

static ssize_t flash_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", (int)g_ctrl->flash_dev_populated);
}

static ssize_t flash_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;
	else if (val != 0 && val != 1)
		return -EINVAL;

	apba_flash_on(g_ctrl, val ? true : false);

	return count;
}

static DEVICE_ATTR_RW(flash_enable);

static ssize_t apba_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return 0;

	return scnprintf(buf, 4, "%d\n", g_ctrl->mode);
}

static ssize_t apba_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (!g_ctrl || !g_ctrl->mods_uart)
		return -ENODEV;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (apba_send_diag_mode_req(val)) {
		pr_err("%s: failed to send MODE\n", __func__);
		return -EIO;
	}

	if (!wait_for_completion_timeout(
		    &g_ctrl->mode_comp,
		    msecs_to_jiffies(APBA_MODE_REQ_TIMEOUT))) {
		pr_err("%s: timeout for MODE\n", __func__);
		return -ETIMEDOUT;
	}

	g_ctrl->mode = val;

	return count;
}

static DEVICE_ATTR_RW(apba_mode);

static ssize_t apba_baud_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl || !g_ctrl->mods_uart)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", mods_uart_get_baud(g_ctrl->mods_uart));
}

static ssize_t apba_baud_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (!g_ctrl || !g_ctrl->mods_uart)
		return -ENODEV;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (apba_send_uart_config_req(val))
		return -EIO;

	/* Prevent further transmissions until we receive the baud
	 * change ACK and change the baud rate.
	 */
	mods_uart_lock_tx(g_ctrl->mods_uart, true);
	if (!wait_for_completion_timeout(
		    &g_ctrl->baud_comp,
		    msecs_to_jiffies(APBA_BAUD_REQ_TIMEOUT))) {
		pr_err("%s: timeout for BAUD\n", __func__);
		mods_uart_lock_tx(g_ctrl->mods_uart, false);
		return -ETIMEDOUT;
	}
	mods_uart_lock_tx(g_ctrl->mods_uart, false);

	return count;
}

static DEVICE_ATTR_RW(apba_baud);

static ssize_t print_ids(char *buf, struct mhb_diag_id_not *not)
{
	return scnprintf(buf, PAGE_SIZE,
			 "%08x:%08x:%08x:%08x %d.%d %s\n",
			 not->unipro_mid,
			 not->unipro_pid,
			 not->vid,
			 not->pid,
			 not->major_version,
			 not->minor_version,
			 not->build);
}

static ssize_t apba_ids_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return -ENODEV;

	return print_ids(buf, &g_ctrl->apba_ids);
}

static DEVICE_ATTR_RO(apba_ids);

static ssize_t apbe_ids_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return -ENODEV;

	return print_ids(buf, &g_ctrl->apbe_ids);
}

static DEVICE_ATTR_RO(apbe_ids);

static ssize_t apba_log_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int count;

	if (!g_ctrl)
		return -ENODEV;

	reinit_completion(&g_ctrl->comp);
	if (apba_send_diag_log_req(MHB_ADDR_DIAG) == 0) {
		if (!wait_for_completion_timeout(
		    &g_ctrl->comp,
		    msecs_to_jiffies(APBA_LOG_REQ_TIMEOUT))) {
			pr_err("%s: timeout from LOG REQUEST\n", __func__);
		}
	}

	mutex_lock(&g_ctrl->log_mutex);
	count = kfifo_out(&apba_log_fifo, buf, PAGE_SIZE - 1);
	mutex_unlock(&g_ctrl->log_mutex);

	return count;
}

static DEVICE_ATTR_RO(apba_log);

static ssize_t apbe_log_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int count;

	if (!g_ctrl)
		return -ENODEV;

	reinit_completion(&g_ctrl->apbe_log_comp);
	if (apba_send_diag_log_req(MHB_ADDR_PEER_DIAG) == 0) {
		if (!wait_for_completion_timeout(
		    &g_ctrl->apbe_log_comp,
		    msecs_to_jiffies(APBE_LOG_REQ_TIMEOUT))) {
			pr_err("%s: timeout from LOG REQUEST\n", __func__);
		}
	}

	mutex_lock(&g_ctrl->log_mutex);
	count = kfifo_out(&apbe_log_fifo, buf, PAGE_SIZE - 1);
	mutex_unlock(&g_ctrl->log_mutex);

	return count;
}

static DEVICE_ATTR_RO(apbe_log);

static ssize_t apbe_power_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (!g_ctrl)
		return -EINVAL;

	if (!g_ctrl->master_intf)
		return -EINVAL;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;
	else if (val != 0 && val != 1)
		return -EINVAL;

	mods_slave_ctrl_power(g_ctrl->master_intf, val,
		MB_CONTROL_SLAVE_MASK_APBE);

	return count;
}

static DEVICE_ATTR_WO(apbe_power);

static ssize_t apbe_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d\n", g_ctrl->apbe_status);
}

static DEVICE_ATTR_RO(apbe_status);

static ssize_t apba_read_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	uint16_t attribute = 0;
	uint16_t selector = 0;

	if (!g_ctrl)
		return -ENODEV;

	ret = sscanf(buf, "%hx %hu", &attribute, &selector);
	if (ret < 1) /* one required parameter */
		return -EINVAL;

	if (apba_send_unipro_read_attr_req(attribute, selector, 0))
		return -EIO;

	if (!wait_for_completion_timeout(
		    &g_ctrl->unipro_comp,
		    msecs_to_jiffies(APBA_UNIPRO_REQ_TIMEOUT))) {
		pr_err("%s: timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return count;
}

static ssize_t apba_read_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "value=%08x, status=%08x\n",
		g_ctrl->last_unipro_value, g_ctrl->last_unipro_status);
}

static DEVICE_ATTR_RW(apba_read);

static ssize_t apba_write_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	uint16_t attribute = 0;
	uint32_t value = 0;
	uint16_t selector = 0;

	if (!g_ctrl)
		return -ENODEV;

	ret = sscanf(buf, "%hx %x %hu", &attribute, &value, &selector);
	if (ret < 2) /* two required parameters */
		return -EINVAL;

	if (apba_send_unipro_write_attr_req(attribute, selector, 0, value))
		return -EIO;

	if (!wait_for_completion_timeout(
		    &g_ctrl->unipro_comp,
		    msecs_to_jiffies(APBA_UNIPRO_REQ_TIMEOUT))) {
		pr_err("%s: timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return count;
}

static DEVICE_ATTR_WO(apba_write);

static ssize_t apbe_read_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int attribute = 0;
	unsigned int selector = 0;

	if (!g_ctrl)
		return -ENODEV;

	ret = sscanf(buf, "%x %u", &attribute, &selector);
	if (ret < 1) /* one required parameter */
		return -EINVAL;

	if (apba_send_unipro_read_attr_req(attribute, selector, 1))
		return -EIO;

	if (!wait_for_completion_timeout(
		    &g_ctrl->unipro_comp,
		    msecs_to_jiffies(APBA_UNIPRO_REQ_TIMEOUT))) {
		pr_err("%s: timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return count;
}

static ssize_t apbe_read_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "value=%08x, status=%08x\n",
		g_ctrl->last_unipro_value, g_ctrl->last_unipro_status);
}

static DEVICE_ATTR_RW(apbe_read);

static ssize_t apbe_write_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	uint16_t attribute = 0;
	uint32_t value = 0;
	uint16_t selector = 0;

	if (!g_ctrl)
		return -ENODEV;

	ret = sscanf(buf, "%hx %x %hu", &attribute, &value, &selector);
	if (ret < 2) /* two required parameters */
		return -EINVAL;

	if (apba_send_unipro_write_attr_req(attribute, selector, 1, value))
		return -EIO;

	if (!wait_for_completion_timeout(
		    &g_ctrl->unipro_comp,
		    msecs_to_jiffies(APBA_UNIPRO_REQ_TIMEOUT))) {
		pr_err("%s: timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return count;
}

static DEVICE_ATTR_WO(apbe_write);

static ssize_t gear_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int tx = 0;
	unsigned int rx = 0;
	int pwrmode = 0;
	unsigned int series = 0;

	if (!g_ctrl)
		return -ENODEV;

	ret = sscanf(buf, "%u %u %x %u", &tx, &rx, &pwrmode, &series);
	if (ret < 4)
		return -EINVAL;

	if (apba_send_unipro_gear_req(tx, rx, pwrmode, series))
		return -EIO;

	if (!wait_for_completion_timeout(
		    &g_ctrl->unipro_comp,
		    msecs_to_jiffies(APBA_UNIPRO_REQ_TIMEOUT))) {
		pr_err("%s: timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return count;
}

static DEVICE_ATTR_WO(gear);

static ssize_t unipro_mid_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%08x\n", g_ctrl->unipro_mid);
}

static DEVICE_ATTR_RO(unipro_mid);

static ssize_t unipro_pid_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%08x\n", g_ctrl->unipro_pid);
}

static DEVICE_ATTR_RO(unipro_pid);

static ssize_t vid_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%08x\n", g_ctrl->ara_vid);
}

static DEVICE_ATTR_RO(vid);

static ssize_t pid_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%08x\n", g_ctrl->ara_pid);
}

static DEVICE_ATTR_RO(pid);

static ssize_t fw_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%08x\n", g_ctrl->fw_version);
}

static DEVICE_ATTR_RO(fw_version);

static ssize_t fw_version_str_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (!g_ctrl)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d.%d %s\n",
			 g_ctrl->fw_version >> 16,
			 g_ctrl->fw_version & 0x0000FFFF,
			 g_ctrl->fw_package_name);
}

static DEVICE_ATTR_RO(fw_version_str);

static ssize_t unipro_stats_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	int count = 0;

	if (!g_ctrl)
		return -ENODEV;

	reinit_completion(&g_ctrl->unipro_stats_comp);
	if (apba_send_unipro_stats_req() == 0) {
		if (!wait_for_completion_timeout(
		    &g_ctrl->unipro_stats_comp,
		    msecs_to_jiffies(APBA_UNIPRO_REQ_TIMEOUT))) {
			pr_err("%s: timeout\n", __func__);
		}
	}

	for (i = 0; i < ARRAY_SIZE(g_ctrl->unipro_stats); i++) {
		count += scnprintf(buf + count, PAGE_SIZE, "%08x\n",
				   g_ctrl->unipro_stats[i]);

		/* Clear counter */
		g_ctrl->unipro_stats[i] = 0;
	}

	return count;
}

static DEVICE_ATTR_RO(unipro_stats);

static struct attribute *apba_attrs[] = {
	&dev_attr_erase_partition.attr,
	&dev_attr_flash_enable.attr,
	&dev_attr_flash_file.attr,
	&dev_attr_apba_enable.attr,
	&dev_attr_apba_baud.attr,
	&dev_attr_apba_ids.attr,
	&dev_attr_apba_log.attr,
	&dev_attr_apba_mode.attr,
	&dev_attr_apbe_ids.attr,
	&dev_attr_apbe_log.attr,
	&dev_attr_apbe_power.attr,
	&dev_attr_apbe_status.attr,
	&dev_attr_apba_read.attr,
	&dev_attr_apba_write.attr,
	&dev_attr_apbe_read.attr,
	&dev_attr_apbe_write.attr,
	&dev_attr_gear.attr,
	&dev_attr_unipro_mid.attr,
	&dev_attr_unipro_pid.attr,
	&dev_attr_vid.attr,
	&dev_attr_pid.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_fw_version_str.attr,
	&dev_attr_unipro_stats.attr,
	NULL,
};

ATTRIBUTE_GROUPS(apba);

static void apba_firmware_callback(const struct firmware *fw,
					 void *context)
{
	struct apba_ctrl *ctrl = (struct apba_ctrl *)context;
	int err;

	if (!ctrl) {
		pr_err("%s: invalid ctrl\n", __func__);
		release_firmware(fw);
		if (g_ctrl)
			complete(&g_ctrl->fw_callback);
		return;
	}

	if (!fw) {
		pr_err("%s: no firmware available\n", __func__);
		apba_flash_on(ctrl, false);
		if (ctrl->desired_on)
			apba_on(ctrl, true);
	} else {
		pr_debug("%s: size=%zu data=%p\n", __func__, fw->size,
			fw->data);

		err = apba_flash_partition(ctrl, APBA_FIRMWARE_PARTITION, fw);
		if (err < 0)
			pr_err("%s: flashing failed err=%d\n", __func__, err);

		/* TODO: notify system, in case of error */
		release_firmware(fw);
	}

	/* Flashing is done, let's let muc core probe finish. */
	muc_enable_det();
	complete(&g_ctrl->fw_callback);
}

static irqreturn_t apba_isr(int irq, void *data)
{
	struct apba_ctrl *ctrl = (struct apba_ctrl *)data;
	int value = gpio_get_value(ctrl->gpios[ctrl->int_index]);

	pr_debug("%s: ctrl=%p, value=%d\n", __func__, ctrl, value);


	if (!ctrl->desired_on || !ctrl->mods_uart || value) {
		pr_err("%s: int ignored\n", __func__);
		return IRQ_HANDLED;
	}

	mods_uart_pm_handle_wake_interrupt(ctrl->mods_uart);

	return IRQ_HANDLED;
}

static int apba_int_setup(struct apba_ctrl *ctrl,
	struct device *dev)
{
	int ret;
	int gpio;
	unsigned int flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

	ret = of_property_read_u32(dev->of_node,
		"mmi,int-index", &ctrl->int_index);
	if (ret) {
		dev_err(dev, "failed to read int index.\n");
		return ret;
	}

	if (ctrl->int_index < 0 || ctrl->int_index >= ctrl->gpio_cnt) {
		dev_err(dev, "int index out of range: %d\n", ctrl->int_index);
		return -EINVAL;
	}

	gpio = ctrl->gpios[ctrl->int_index];
	ctrl->irq = gpio_to_irq(gpio);
	dev_dbg(dev, "irq: gpio=%d irq=%d\n", gpio, ctrl->irq);

	irq_set_status_flags(ctrl->irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, ctrl->irq, NULL /* handler */,
		apba_isr, flags, "apba_ctrl", ctrl);
	if (ret) {
		dev_err(dev, "irq request failed: %d\n", ret);
		return ret;
	}

	enable_irq_wake(ctrl->irq);

	return ret;
}

static void apba_gpio_free(struct apba_ctrl *ctrl, struct device *dev)
{
	int i;

	for (i = 0; i < ctrl->gpio_cnt; i++) {
		sysfs_remove_link(&dev->kobj, ctrl->gpio_labels[i]);
		gpio_unexport(ctrl->gpios[i]);
	}
}

static int apba_gpio_setup(struct apba_ctrl *ctrl, struct device *dev)
{
	int i;
	int gpio_cnt = of_gpio_count(dev->of_node);
	const char *label_prop = "mmi,gpio-labels";
	int label_cnt = of_property_count_strings(dev->of_node, label_prop);
	int ret;

	if (gpio_cnt <= 0) {
		dev_err(dev, "No GPIOs were defined\n");
		return -EINVAL;
	}

	if (gpio_cnt > ARRAY_SIZE(ctrl->gpios)) {
		dev_err(dev, "%s: gpio count is greater than %zu.\n",
			__func__, ARRAY_SIZE(ctrl->gpios));
		return -EINVAL;
	}

	if (label_cnt != gpio_cnt) {
		dev_err(dev, "%s: label count does not match gpio count.\n",
			__func__);
		return -EINVAL;
	}

	for (i = 0; i < gpio_cnt; i++) {
		enum of_gpio_flags flags = 0;
		int gpio;
		const char *label = NULL;

		gpio = of_get_gpio_flags(dev->of_node, i, &flags);
		if (!gpio_is_valid(gpio)) {
			dev_err(dev, "of_get_gpio failed: %d index: %d\n",
				gpio, i);
			ret = -EINVAL;
			goto gpio_cleanup;
		}

		ret = of_property_read_string_index(dev->of_node,
					label_prop, i, &label);
		if (ret) {
			dev_err(dev, "reading label failed: %d index: %d\n",
				ret, i);
			goto gpio_cleanup;
		}

		ret = devm_gpio_request_one(dev, gpio, flags, label);
		if (ret) {
			dev_err(dev, "failed request gpio: %d index: %d\n",
				gpio, i);
			goto gpio_cleanup;
		}

		ret = gpio_export(gpio, true);
		if (ret) {
			dev_err(dev, "failed to export gpio: %d index: %d\n",
				gpio, i);
			goto gpio_cleanup;
		}

		ret = gpio_export_link(dev, label, gpio);
		if (ret) {
			dev_err(dev, "failed to link gpio: %d index: %d\n",
				gpio, i);
			gpio_unexport(gpio);
			goto gpio_cleanup;
		}

		dev_dbg(dev, "%s: gpio=%d, flags=0x%x, label=%s\n",
			__func__, gpio, flags, label);

		ctrl->gpios[i] = gpio;
		ctrl->gpio_labels[i] = label;
		ctrl->gpio_cnt++;
	}

	return 0;

gpio_cleanup:
	apba_gpio_free(ctrl, dev);

	return ret;
}

void apba_wake_assert(bool assert)
{
	if (!g_ctrl)
		return;

	if (assert)
		apba_seq(g_ctrl, &g_ctrl->wake_assert_seq);
	else
		apba_seq(g_ctrl, &g_ctrl->wake_deassert_seq);
}

int apba_uart_register(void *mods_uart)
{
	if (!g_ctrl)
		return -ENODEV;

	g_ctrl->mods_uart = mods_uart;
	return 0;
}

static void apba_disable_work_func(struct work_struct *work)
{
	apba_disable();
	apba_send_kobj_uevent("APBA_EVENT=SLAVE_DISABLED");
}

static void apba_enable_work_func(struct work_struct *work)
{
	apba_enable();
	apba_send_kobj_uevent("APBA_EVENT=SLAVE_ENABLED");
}

static void apba_dettach_work_func(struct work_struct *work)
{
	if (!g_ctrl)
		return;

	g_ctrl->desired_on = 0;
	g_ctrl->apbe_status = MHB_PM_STATUS_PEER_NONE;

	apba_on(g_ctrl, false);
}

/*
 * muc is informing us through this callback that it has a slave present,
 * likely APBE. If it is an APBE, we should enable the APBA so that the two
 * can communicate.
 */
static void apba_slave_notify(uint8_t master_intf, uint32_t slave_mask,
				uint32_t slave_state)
{
	if (!g_ctrl)
		return;

	pr_debug("%s: master_intf=%d, slave_mask=0x%x, slave_state=0x%x\n",
		__func__, master_intf, slave_mask, slave_state);

	if (slave_mask != MB_CONTROL_SLAVE_MASK_APBE) {
		pr_debug("%s: ignore\n", __func__);
		return;
	}

	g_ctrl->master_intf = master_intf;

	switch (slave_state) {
	case SLAVE_STATE_DISABLED:
		/* don't call apba_disable() here, causes greybus
		 * operation failures as we are not done handling
		 * slave state gb message yet.
		 */
		queue_work(g_ctrl->wq, &apba_disable_work);
		break;
	case SLAVE_STATE_ENABLED:
		queue_work(g_ctrl->wq, &apba_enable_work);
		break;
	default:
		pr_err("%s: Invalid slave state=%d.\n", __func__, slave_state);
		break;
	}
}

static struct mods_slave_ctrl_driver apbe_ctrl_drv = {
	.slave_notify = apba_slave_notify,
};

int apba_enable(void)
{
	if (!g_ctrl)
		return -ENODEV;

	g_ctrl->desired_on = 1;

	apba_on(g_ctrl, true);

	return 0;
}

void apba_disable(void)
{
	if (!g_ctrl || !g_ctrl->desired_on)
		return;

	mods_slave_ctrl_power(g_ctrl->master_intf,
		MB_CONTROL_SLAVE_POWER_OFF, MB_CONTROL_SLAVE_MASK_APBE);
	g_ctrl->desired_on = 0;
	g_ctrl->apbe_status = MHB_PM_STATUS_PEER_NONE;

	apba_on(g_ctrl, false);
}

static int apba_ctrl_probe(struct platform_device *pdev)
{
	struct apba_ctrl *ctrl;
	int ret;

	/* we depend on the muc_core for transports and pinctrls */
	if (!muc_core_probed())
		return -EPROBE_DEFER;

	if (!pdev->dev.of_node) {
		/* Platform data not currently supported */
		dev_err(&pdev->dev, "%s: of devtree not found\n", __func__);
		return -EINVAL;
	}

	ctrl = devm_kzalloc(&pdev->dev, sizeof(*ctrl),
		GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->dev = &pdev->dev;

	ctrl->mclk = devm_clk_get(&pdev->dev, "apba_mclk");
	if (IS_ERR(ctrl->mclk)) {
		dev_err(&pdev->dev, "%s: failed to get clock.\n", __func__);
		return PTR_ERR(ctrl->mclk);
	}

	ret = apba_gpio_setup(ctrl, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to read gpios.\n");
		return ret;
	}

	ret = apba_int_setup(ctrl, &pdev->dev);
	if (ret)
		goto free_gpios;

	ctrl->enable_preclk_seq.len = ARRAY_SIZE(ctrl->enable_preclk_seq.val);
	ret = apba_parse_seq(&pdev->dev, "mmi,enable-preclk-seq",
		&ctrl->enable_preclk_seq);
	if (ret)
		goto disable_irq;


	ctrl->enable_postclk_seq.len =
				     ARRAY_SIZE(ctrl->enable_postclk_seq.val);
	ret = apba_parse_seq(&pdev->dev, "mmi,enable-postclk-seq",
		&ctrl->enable_postclk_seq);
	if (ret)
		goto disable_irq;

	ctrl->disable_seq.len = ARRAY_SIZE(ctrl->disable_seq.val);
	ret = apba_parse_seq(&pdev->dev, "mmi,disable-seq",
		&ctrl->disable_seq);
	if (ret)
		goto disable_irq;

	ctrl->wake_assert_seq.len = ARRAY_SIZE(ctrl->wake_assert_seq.val);
	ret = apba_parse_seq(&pdev->dev, "mmi,wake-assert-seq",
		&ctrl->wake_assert_seq);
	if (ret)
		goto disable_irq;

	ctrl->wake_deassert_seq.len = ARRAY_SIZE(ctrl->wake_deassert_seq.val);
	ret = apba_parse_seq(&pdev->dev, "mmi,wake-deassert-seq",
		&ctrl->wake_deassert_seq);
	if (ret)
		goto disable_irq;

	ctrl->flash_start_seq.len = ARRAY_SIZE(ctrl->flash_start_seq.val);
	ret = apba_parse_seq(&pdev->dev, "mmi,flash-start-seq",
		&ctrl->flash_start_seq);
	if (ret)
		goto disable_irq;

	ctrl->flash_end_seq.len = ARRAY_SIZE(ctrl->flash_end_seq.val);
	ret = apba_parse_seq(&pdev->dev, "mmi,flash-end-seq",
		&ctrl->flash_end_seq);
	if (ret)
		goto disable_irq;

	/* A default pinctrl state (at least) is expected */
	ctrl->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(ctrl->pinctrl)) {
		dev_err(&pdev->dev, "Pinctrl not defined\n");
		ret = PTR_ERR(ctrl->pinctrl);
		goto disable_irq;
	}

	ctrl->pinctrl_state_default = pinctrl_lookup_state(ctrl->pinctrl,
							   PINCTRL_STATE_DEFAULT);

	if (IS_ERR(ctrl->pinctrl_state_default)) {
		dev_err(&pdev->dev, "Pinctrl lookup failed for default\n");
		ret = PTR_ERR(ctrl->pinctrl_state_default);
		goto disable_irq;
	}

	/* The spi_active pinctrl state is optional */
	ctrl->pinctrl_state_active = pinctrl_lookup_state(ctrl->pinctrl,
							  "spi_active");
	if (IS_ERR(ctrl->pinctrl_state_active)) {
		dev_warn(&pdev->dev, "Pinctrl lookup failed for spi_active\n");
	}

	mutex_init(&ctrl->log_mutex);
	init_completion(&ctrl->comp);
	init_completion(&ctrl->apbe_log_comp);
	init_completion(&ctrl->baud_comp);
	init_completion(&ctrl->mode_comp);
	init_completion(&ctrl->unipro_comp);
	init_completion(&ctrl->unipro_stats_comp);

	ctrl->unipro_mid = APBA_FIRMWARE_UNIPRO_MID;
	ctrl->unipro_pid = APBA_FIRMWARE_UNIPRO_PID;
	ctrl->ara_vid = APBA_FIRMWARE_ARA_VID;
	ctrl->ara_pid = APBA_FIRMWARE_ARA_PID;

	if (of_property_read_u32(pdev->dev.of_node,
				 "mmi,apba-unipro-mid", &ctrl->unipro_mid))
		dev_warn(&pdev->dev, "unipro-mid missing\n");
	if (of_property_read_u32(pdev->dev.of_node,
				 "mmi,apba-unipro-pid", &ctrl->unipro_pid))
		dev_warn(&pdev->dev, "unipro-pid missing\n");
	if (of_property_read_u32(pdev->dev.of_node,
				 "mmi,apba-ara-vid", &ctrl->ara_vid))
		dev_warn(&pdev->dev, "ara-vid missing\n");
	if (of_property_read_u32(pdev->dev.of_node,
				 "mmi,apba-ara-pid", &ctrl->ara_pid))
		dev_warn(&pdev->dev, "ara-pid missing\n");

	ret = sysfs_create_groups(&pdev->dev.kobj, apba_groups);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs attr\n");
		goto disable_irq;
	}

	/* start with APBA turned OFF */
	apba_seq(ctrl, &ctrl->disable_seq);

	g_ctrl = ctrl;

	platform_set_drvdata(pdev, ctrl);

	ctrl->wq = create_singlethread_workqueue("apba");
	if (!ctrl->wq) {
		dev_err(&pdev->dev, "Failed to create workqueue\n");
		ret = -ENOMEM;
		goto reset_global;
	}

	INIT_WORK(&apba_disable_work, apba_disable_work_func);
	INIT_WORK(&apba_enable_work, apba_enable_work_func);
	INIT_WORK(&apba_dettach_work, apba_dettach_work_func);

	ret = mods_register_slave_ctrl_driver(&apbe_ctrl_drv);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register slave driver\n");
		goto remove_wq;
	}

	snprintf(ctrl->firmware_name, sizeof(ctrl->firmware_name),
		 "upd-%08x-%08x-%08x-%08x-%02x.tftf",
		 ctrl->unipro_mid, ctrl->unipro_pid,
		 ctrl->ara_vid, ctrl->ara_pid,
		 APBA_FIRMWARE_STAGE);
	pr_debug("%s: requesting fw %s\n", __func__, ctrl->firmware_name);

	init_completion(&ctrl->fw_callback);
	ret = request_firmware_nowait(THIS_MODULE, true, ctrl->firmware_name,
				      g_ctrl->dev, GFP_KERNEL, g_ctrl,
				      apba_firmware_callback);
	if (ret) {
		dev_err(g_ctrl->dev, "failed to request firmware.\n");
		goto unregister_slave_ctrl;
	}

	ctrl->attach_nb.notifier_call = apba_attach_notifier;
	register_muc_attach_notifier(&ctrl->attach_nb);

	kobject_uevent(&pdev->dev.kobj, KOBJ_ADD);

	return 0;

unregister_slave_ctrl:
	mods_unregister_slave_ctrl_driver(&apbe_ctrl_drv);
remove_wq:
	destroy_workqueue(ctrl->wq);
reset_global:
	sysfs_remove_groups(&pdev->dev.kobj, apba_groups);
	g_ctrl = NULL;
disable_irq:
	disable_irq_wake(ctrl->irq);
free_gpios:
	apba_gpio_free(ctrl, &pdev->dev);

	of_platform_depopulate(ctrl->dev);
	of_node_clear_flag(ctrl->dev->of_node, OF_POPULATED_BUS);

	/* Let muc core finish probe even if we bombed out. */
	muc_enable_det();
	return ret;
}

static int apba_ctrl_remove(struct platform_device *pdev)
{
	struct apba_ctrl *ctrl = platform_get_drvdata(pdev);

	wait_for_completion(&ctrl->fw_callback);

	unregister_muc_attach_notifier(&ctrl->attach_nb);
	flush_work(&apba_dettach_work);

	sysfs_remove_groups(&pdev->dev.kobj, apba_groups);

	mods_unregister_slave_ctrl_driver(&apbe_ctrl_drv);

	cancel_work_sync(&apba_disable_work);
	cancel_work_sync(&apba_enable_work);
	cancel_work_sync(&apba_dettach_work);
	destroy_workqueue(ctrl->wq);

	disable_irq_wake(ctrl->irq);
	apba_disable();
	apba_gpio_free(ctrl, &pdev->dev);

	of_platform_depopulate(ctrl->dev);
	of_node_clear_flag(ctrl->dev->of_node, OF_POPULATED_BUS);

	g_ctrl = NULL;

	return 0;
}

static const struct of_device_id apba_ctrl_match[] = {
	{.compatible = "mmi,apba-ctrl",},
	{},
};

static const struct platform_device_id apba_ctrl_id_table[] = {
	{"apba_ctrl", 0},
	{},
};

static struct platform_driver apba_ctrl_driver = {
	.driver = {
		.name = "apba_ctrl",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(apba_ctrl_match),
	},
	.probe = apba_ctrl_probe,
	.remove = apba_ctrl_remove,
	.id_table = apba_ctrl_id_table,
};

int __init apba_ctrl_init(void)
{
	return platform_driver_register(&apba_ctrl_driver);
}

void apba_ctrl_exit(void)
{
	platform_driver_unregister(&apba_ctrl_driver);
}
