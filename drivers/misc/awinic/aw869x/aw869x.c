/*
 * aw869x.c   aw869x haptic module
 *
 * Version: v1.4.0
 *
 * Copyright (c) 2017 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include "aw869x.h"
#include "aw869x_reg.h"

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW869X_I2C_NAME "aw869x_haptic"
#define AW869X_HAPTIC_NAME "aw869x_haptic"

#define AW869X_VERSION "v1.4.0"


//#define AWINIC_I2C_REGMAP
//#define AWINIC_RAM_UPDATE_DELAY

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2

#define AW869X_MAX_DSP_START_TRY_COUNT    10


//#define AW869X_REPEAT_RTP_PLAYING
#define AW869X_MAX_FIRMWARE_LOAD_CNT 20
#define AW869X_SEQ_NO_RTP_BASE 102
/******************************************************
 *
 * variable
 *
 ******************************************************/
#define AW869X_RTP_NAME_MAX        64
static char *aw869x_ram_name = "aw869x_haptic.bin";
static char aw869x_rtp_name[][AW869X_RTP_NAME_MAX] = {
    {"aw869x_rtp.bin"},
    {"aw869x_rtp_Argo_Navis.bin"},
    {"aw869x_rtp_Attentive.bin"},
    {"aw869x_rtp_Awake.bin"},
    {"aw869x_rtp_Bird_Loop.bin"},
    {"aw869x_rtp_Brilliant_Times.bin"},
    {"aw869x_rtp_Chimey_Phone.bin"},
    {"aw869x_rtp_Complex.bin"},
    {"aw869x_rtp_Crazy_Dream.bin"},
    {"aw869x_rtp_Curve_Ball_Blend.bin"},
    {"aw869x_rtp_Digital_Phone.bin"},
    {"aw869x_rtp_Electrovision.bin"},
    {"aw869x_rtp_Ether_Shake.bin"},
    {"aw869x_rtp_Fateful_Words.bin"},
    {"aw869x_rtp_Flutey_Phone.bin"},
    {"aw869x_rtp_Future_Funk.bin"},
    {"aw869x_rtp_Future_Hi_Tech.bin"},
    {"aw869x_rtp_Girtab.bin"},
    {"aw869x_rtp_Hello.bin"},
    {"aw869x_rtp_Hexagon.bin"},
    {"aw869x_rtp_Hydra.bin"},
    {"aw869x_rtp_Insert_Coin.bin"},
    {"aw869x_rtp_Jumping_Dots.bin"},
    {"aw869x_rtp_Keys.bin"},
    {"aw869x_rtp_Loopy.bin"},
    {"aw869x_rtp_Loopy_Lounge.bin"},
    {"aw869x_rtp_Modular.bin"},
    {"aw869x_rtp_Momentum.bin"},
    {"aw869x_rtp_Morning.bin"},
    {"aw869x_rtp_Moto.bin"},
    {"aw869x_rtp_Natural.bin"},
    {"aw869x_rtp_New_Player.bin"},
    {"aw869x_rtp_Onward.bin"},
    {"aw869x_rtp_Organ_Dub.bin"},
    {"aw869x_rtp_Overclocked.bin"},
    {"aw869x_rtp_Pegasus.bin"},
    {"aw869x_rtp_Pyxis.bin"},
    {"aw869x_rtp_Regrade.bin"},
    {"aw869x_rtp_Scarabaeus.bin"},
    {"aw869x_rtp_Sceptrum.bin"},
    {"aw869x_rtp_Simple.bin"},
    {"aw869x_rtp_Solarium.bin"},
    {"aw869x_rtp_Sparse.bin"},
    {"aw869x_rtp_Terrabytes.bin"},
    {"aw869x_rtp_TJINGLE.bin"},
};

struct aw869x_container *aw869x_rtp;
struct aw869x *g_aw869x;

/******************************************************
 *
 * functions
 *
 ******************************************************/
static void aw869x_interrupt_clear(struct aw869x *aw869x);
static void aw869x_vibrate(struct aw869x *aw869x, int value);

 /******************************************************
 *
 * aw869x i2c write/read
 *
 ******************************************************/
static int aw869x_i2c_write(struct aw869x *aw869x, 
         unsigned char reg_addr, unsigned char reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < AW_I2C_RETRIES) {
#ifdef AWINIC_I2C_REGMAP
        ret = regmap_write(aw869x->regmap, reg_addr, reg_data);
        if(ret < 0) {
            pr_err("%s: regmap_write cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            break;
        }
#else
        ret = i2c_smbus_write_byte_data(aw869x->i2c, reg_addr, reg_data);
        if(ret < 0) {
            pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            break;
        }
#endif
        cnt ++;
        msleep(AW_I2C_RETRY_DELAY);
    }

    return ret;
}

static int aw869x_i2c_read(struct aw869x *aw869x, 
        unsigned char reg_addr, unsigned char *reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < AW_I2C_RETRIES) {
#ifdef AWINIC_I2C_REGMAP
        ret = regmap_read(aw869x->regmap, reg_addr, reg_data);
        if(ret < 0) {
            pr_err("%s: regmap_read cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            break;
        }
#else
        ret = i2c_smbus_read_byte_data(aw869x->i2c, reg_addr);
        if(ret < 0) {
            pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            *reg_data = ret;
            break;
        }
#endif
        cnt ++;
        msleep(AW_I2C_RETRY_DELAY);
    }

    return ret;
}

static int aw869x_i2c_write_bits(struct aw869x *aw869x, 
         unsigned char reg_addr, unsigned int mask, unsigned char reg_data)
{
    unsigned char reg_val = 0;

    aw869x_i2c_read(aw869x, reg_addr, &reg_val);
    reg_val &= mask;
    reg_val |= reg_data;
    aw869x_i2c_write(aw869x, reg_addr, reg_val);

    return 0;
}

static int aw869x_i2c_writes(struct aw869x *aw869x,
        unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
    int ret = -1;
    unsigned char *data;

    data = kmalloc(len+1, GFP_KERNEL);
    if (data == NULL) {
        pr_err("%s: can not allocate memory\n", __func__);
        return  -ENOMEM;
    }

    data[0] = reg_addr;
    memcpy(&data[1], buf, len);

    ret = i2c_master_send(aw869x->i2c, data, len+1);
    if (ret < 0) {
        pr_err("%s: i2c master send error\n", __func__);
    }

    kfree(data);

    return ret;
}

/*****************************************************
 *
 * ram update
 *
 *****************************************************/
static void aw869x_rtp_loaded(const struct firmware *cont, void *context)
{
    struct aw869x *aw869x = context;
    pr_debug("%s enter\n", __func__);

    if (!cont) {
        pr_err("%s: failed to read %s\n", __func__, aw869x_rtp_name[aw869x->rtp_file_num]);
        release_firmware(cont);
        return;
    }

    pr_debug("%s: loaded %s - size: %zu\n", __func__, aw869x_rtp_name[aw869x->rtp_file_num],
                    cont ? cont->size : 0);

    /* aw869x rtp update */
    aw869x_rtp = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
    if (!aw869x_rtp) {
        release_firmware(cont);
        pr_err("%s: Error allocating memory\n", __func__);
        return;
    }
    aw869x_rtp->len = cont->size;
    pr_debug("%s: rtp size = %d\n", __func__, aw869x_rtp->len);
    memcpy(aw869x_rtp->data, cont->data, cont->size);
    release_firmware(cont);

    aw869x->rtp_init = 1;
    pr_debug("%s: rtp update complete\n", __func__);

    /* Vibrate for 1 second as TI drv2624's auto-calibration does */
    aw869x_vibrate(aw869x, 1000);
}

static int aw869x_rtp_update(struct aw869x *aw869x)
{
    pr_debug("%s enter\n", __func__);

    return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
                aw869x_rtp_name[aw869x->rtp_file_num], aw869x->dev, GFP_KERNEL,
                aw869x, aw869x_rtp_loaded);
}


 static void aw869x_container_update(struct aw869x *aw869x, 
        struct aw869x_container *aw869x_cont)
{
    int i = 0;
    unsigned int shift = 0;

    pr_debug("%s enter\n", __func__);

    aw869x->ram.baseaddr_shift = 2;
    aw869x->ram.ram_shift = 4;

    /* RAMINIT Enable */
    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL, 
            AW869X_BIT_SYSCTRL_RAMINIT_MASK, AW869X_BIT_SYSCTRL_RAMINIT_EN);

    /* base addr */
    shift = aw869x->ram.baseaddr_shift;
    aw869x->ram.base_addr = (unsigned int)((aw869x_cont->data[0+shift]<<8) | 
            (aw869x_cont->data[1+shift]));
    pr_debug("%s: base_addr=0x%4x\n", __func__, aw869x->ram.base_addr);
    
    aw869x_i2c_write(aw869x, AW869X_REG_BASE_ADDRH, aw869x_cont->data[0+shift]);
    aw869x_i2c_write(aw869x, AW869X_REG_BASE_ADDRL, aw869x_cont->data[1+shift]);
    
    aw869x_i2c_write(aw869x, AW869X_REG_FIFO_AEH, 
                    (unsigned char)((aw869x->ram.base_addr>>2)>>8));
    aw869x_i2c_write(aw869x, AW869X_REG_FIFO_AEL, 
                    (unsigned char)((aw869x->ram.base_addr>>2)&0x00FF));
    aw869x_i2c_write(aw869x, AW869X_REG_FIFO_AFH, 
                    (unsigned char)((aw869x->ram.base_addr-(aw869x->ram.base_addr>>2))>>8));
    aw869x_i2c_write(aw869x, AW869X_REG_FIFO_AFL, 
                    (unsigned char)((aw869x->ram.base_addr-(aw869x->ram.base_addr>>2))&0x00FF));

    /* ram */
    shift = aw869x->ram.baseaddr_shift;
    aw869x_i2c_write(aw869x, AW869X_REG_RAMADDRH, aw869x_cont->data[0+shift]);
    aw869x_i2c_write(aw869x, AW869X_REG_RAMADDRL, aw869x_cont->data[1+shift]);
    shift = aw869x->ram.ram_shift;
    for(i=shift; i<aw869x_cont->len; i++) {
        aw869x_i2c_write(aw869x, AW869X_REG_RAMDATA, aw869x_cont->data[i]);
    }

#if 0
    /* ram check */
    shift = aw869x->ram.baseaddr_shift;
    aw869x_i2c_write(aw869x, AW869X_REG_RAMADDRH, aw869x_cont->data[0+shift]);
    aw869x_i2c_write(aw869x, AW869X_REG_RAMADDRL, aw869x_cont->data[1+shift]);
    shift = aw869x->ram.ram_shift;
    for(i=shift; i<aw869x_cont->len; i++) {
        aw869x_i2c_read(aw869x, AW869X_REG_RAMDATA, &reg_val);
        if(reg_val != aw869x_cont->data[i]) {
            pr_err("%s: ram check error addr=0x%04x, file_data=0x%02x, ram_data=0x%02x\n",
                        __func__, i, aw869x_cont->data[i], reg_val);
            return;
        }
    }
#endif

    /* RAMINIT Disable */
    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL, 
            AW869X_BIT_SYSCTRL_RAMINIT_MASK, AW869X_BIT_SYSCTRL_RAMINIT_OFF);

    pr_debug("%s exit\n", __func__);
}


static void aw869x_ram_loaded(const struct firmware *cont, void *context)
{
    struct aw869x *aw869x = context;
    struct aw869x_container *aw869x_fw;
    int i = 0;
    unsigned short check_sum = 0;

    pr_debug("%s enter\n", __func__);

    if (!cont) {
        pr_err("%s: failed to read %s\n", __func__, aw869x_ram_name);
        release_firmware(cont);
        return;
    }

    pr_debug("%s: loaded %s - size: %zu\n", __func__, aw869x_ram_name,
                    cont ? cont->size : 0);
/*
    for(i=0; i<cont->size; i++) {
        pr_info("%s: addr:0x%04x, data:0x%02x\n", __func__, i, *(cont->data+i));
    }
*/

    /* check sum */
    for(i=2; i<cont->size; i++) {
        check_sum += cont->data[i];
    }
    if(check_sum != (unsigned short)((cont->data[0]<<8)|(cont->data[1]))) {
        pr_err("%s: check sum err: check_sum=0x%04x\n", __func__, check_sum);
        return;
    } else {
        pr_debug("%s: check sum pass : 0x%04x\n", __func__, check_sum);
        aw869x->ram.check_sum = check_sum;
    }

    /* aw869x ram update */
    aw869x_fw = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
    if (!aw869x_fw) {
        release_firmware(cont);
        pr_err("%s: Error allocating memory\n", __func__);
        return;
    }
    aw869x_fw->len = cont->size;
    memcpy(aw869x_fw->data, cont->data, cont->size);
    release_firmware(cont);

    aw869x_container_update(aw869x, aw869x_fw);

    aw869x->ram.len = aw869x_fw->len;

    kfree(aw869x_fw);

    aw869x->ram_init = 1;
    pr_debug("%s: fw update complete\n", __func__);
    
    aw869x_rtp_update(aw869x);
}

static int aw869x_ram_update(struct aw869x *aw869x)
{
    aw869x->ram_init = 0;
    aw869x->rtp_init = 0;
    return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
                aw869x_ram_name, aw869x->dev, GFP_KERNEL,
                aw869x, aw869x_ram_loaded);
}

#ifdef AWINIC_RAM_UPDATE_DELAY
static enum hrtimer_restart aw869x_ram_timer_func(struct hrtimer *timer)
{
    struct aw869x *aw869x = container_of(timer, struct aw869x, ram_timer);

    pr_info("%s enter\n", __func__);

    schedule_work(&aw869x->ram_work);
    
    return HRTIMER_NORESTART;
}

static void aw869x_ram_work_routine(struct work_struct *work)
{
    struct aw869x *aw869x = container_of(work, struct aw869x, ram_work);

    pr_info("%s enter\n", __func__);
    
    aw869x_ram_update(aw869x);

}
#endif

static int aw869x_ram_init(struct aw869x *aw869x)
{
#ifdef AWINIC_RAM_UPDATE_DELAY
    int ram_timer_val = 5000;

    hrtimer_init(&aw869x->ram_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    aw869x->ram_timer.function = aw869x_ram_timer_func;
    INIT_WORK(&aw869x->ram_work, aw869x_ram_work_routine);
    hrtimer_start(&aw869x->ram_timer, 
            ktime_set(ram_timer_val/1000, (ram_timer_val%1000)*1000000), 
            HRTIMER_MODE_REL);
#else
    aw869x_ram_update(aw869x);
#endif
    return 0;
}



/*****************************************************
 *
 * haptic control
 *
 *****************************************************/
static int aw869x_haptic_play_mode(struct aw869x *aw869x, unsigned char play_mode)
{
    switch(play_mode) {
        case AW869X_HAPTIC_STANDBY_MODE:
            aw869x->play_mode = AW869X_HAPTIC_STANDBY_MODE;
            break;
        case AW869X_HAPTIC_RAM_MODE:
            aw869x->play_mode = AW869X_HAPTIC_RAM_MODE;
            aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL, 
                    AW869X_BIT_SYSCTRL_PLAY_MODE_MASK, AW869X_BIT_SYSCTRL_PLAY_MODE_RAM);
            break;
        case AW869X_HAPTIC_RTP_MODE:
            aw869x->play_mode = AW869X_HAPTIC_RTP_MODE;
            aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL, 
                    AW869X_BIT_SYSCTRL_PLAY_MODE_MASK, AW869X_BIT_SYSCTRL_PLAY_MODE_RTP);
            break;
        case AW869X_HAPTIC_TRIG_MODE:
            aw869x->play_mode = AW869X_HAPTIC_TRIG_MODE;
            aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL, 
                    AW869X_BIT_SYSCTRL_PLAY_MODE_MASK, AW869X_BIT_SYSCTRL_PLAY_MODE_RAM);
            break;
        default:
            dev_err(aw869x->dev, "%s: play mode %d err",
                    __func__, play_mode);
            break;
    }
    return 0;
}

static int aw869x_haptic_stop(struct aw869x *aw869x)
{
    aw869x_haptic_play_mode(aw869x, AW869X_HAPTIC_STANDBY_MODE);
    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL, 
            AW869X_BIT_SYSCTRL_WORK_MODE_MASK, AW869X_BIT_SYSCTRL_STANDBY);
    msleep(1);
    /* RAMINIT Disable */
    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL,
        AW869X_BIT_SYSCTRL_RAMINIT_MASK, AW869X_BIT_SYSCTRL_RAMINIT_OFF);
    /* RAMINIT Disable End */
    return 0;
}

static int aw869x_haptic_start(struct aw869x *aw869x)
{
    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL, 
            AW869X_BIT_SYSCTRL_WORK_MODE_MASK, AW869X_BIT_SYSCTRL_ACTIVE);
    /* RAMINIT Enable */
    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL,
            AW869X_BIT_SYSCTRL_RAMINIT_MASK, AW869X_BIT_SYSCTRL_RAMINIT_EN);
    /* RAMINIT Enable End */
    aw869x_interrupt_clear(aw869x);
    aw869x_i2c_write(aw869x, AW869X_REG_PROC_CTRL,
            AW869X_BIT_PROC_CTRL_GO);
    return 0;
}

static int aw869x_haptic_set_repeat_seq(struct aw869x *aw869x, unsigned char flag)
{
    if(flag) {
        aw869x_i2c_write_bits(aw869x, AW869X_REG_DATACTRL,
                AW869X_BIT_DATACTRL_WAV_DBG_MASK, AW869X_BIT_DATACTRL_WAV_DBG);
    } else {
        aw869x_i2c_write_bits(aw869x, AW869X_REG_DATACTRL,
                AW869X_BIT_DATACTRL_WAV_DBG_MASK, AW869X_BIT_DATACTRL_WAV_NORMAL);
    }

    return 0;
}

static int aw869x_haptic_set_repeat_que_seq(struct aw869x *aw869x, unsigned char seq)
{
    unsigned char i;
    for(i=0; i<AW869X_SEQUENCER_SIZE; i++) {
        aw869x_i2c_write(aw869x, AW869X_REG_QUE_SEQ1+i,
                seq);
    }
    return 0;
}

static int aw869x_haptic_set_que_seq(struct aw869x *aw869x, unsigned int seq)
{
    unsigned char i = 0;
    struct aw869x_que_seq que_seq;

    for(i=0; i<AW869X_SEQUENCER_SIZE; i++) {
        que_seq.index[i] = (seq>>((AW869X_SEQUENCER_SIZE-i-1)*8))&0xFF;
    }

    for(i=0; i<AW869X_SEQUENCER_SIZE; i++) {
        aw869x_i2c_write(aw869x, AW869X_REG_QUE_SEQ1+i,
                que_seq.index[i]);
    }
    return 0;
}

static int aw869x_haptic_set_que_loop(struct aw869x *aw869x, unsigned int loop)
{
    unsigned char i = 0;
    unsigned char tmp[2] = {0, 0 };

    tmp[0] |= (loop>>(24-4))&0xF0;
    tmp[0] |= (loop>>(16-0))&0x0F;
    tmp[1] |= (loop>>( 8-4))&0xF0;
    tmp[1] |= (loop>>( 0-0))&0x0F;

    for(i=0; i<AW869X_SEQUENCER_LOOP_SIZE; i++) {
        aw869x_i2c_write(aw869x, AW869X_REG_SEQ_LOOP1+i, tmp[i]);
    }

    return 0;
}


static int aw869x_set_que_seq(struct aw869x *aw869x, unsigned long arg)
{
    int ret = 0;
    unsigned char i = 0;
    struct aw869x_que_seq que_seq;

    if (copy_from_user(&que_seq,
        (void __user *)arg, sizeof(struct aw869x_que_seq)))
        return -EFAULT;

    for(i=0; i<AW869X_SEQUENCER_SIZE; i++) {
        aw869x_i2c_write(aw869x, AW869X_REG_QUE_SEQ1+i, que_seq.index[i]);
    }

    return ret;
}

static int aw869x_set_seq_loop(struct aw869x *aw869x, unsigned long arg)
{
    struct aw869x_seq_loop seq_loop;
    int ret = 0;
    unsigned char i = 0;
    unsigned char loop[2] = {0, 0 };

    if (copy_from_user(&seq_loop, 
        (void __user *)arg, sizeof(struct aw869x_seq_loop)))
        return -EFAULT;

    for(i=0; i<AW869X_SEQUENCER_SIZE; i++) {
        if(seq_loop.loop[i] & 0xF0) {
            dev_err(aw869x->dev, "%s: seq_loop err loop[%d]=0x%02x\n",
                    __func__, i, seq_loop.loop[i]);
            return -1;
        }
    }
    loop[0] |= (seq_loop.loop[0]<<4);
    loop[0] |= (seq_loop.loop[1]<<0);
    loop[1] |= (seq_loop.loop[2]<<4);
    loop[1] |= (seq_loop.loop[3]<<0);

    for(i=0; i<(AW869X_SEQUENCER_SIZE>>1); i++) {
        aw869x_i2c_write(aw869x, AW869X_REG_SEQ_LOOP1+i, loop[i]);
    }

    return ret;
}

static int aw869x_haptic_set_bst_vol(struct aw869x *aw869x, unsigned char bst_vol)
{
    aw869x_i2c_write_bits(aw869x, AW869X_REG_BSTCFG,
            AW869X_BIT_BSTCFG_BSTVOL_MASK, bst_vol);
    return 0;
}

static int aw869x_haptic_set_bst_peak_cur(struct aw869x *aw869x, unsigned char peak_cur)
{
    aw869x_i2c_write_bits(aw869x, AW869X_REG_BSTCFG,
            AW869X_BIT_BSTCFG_PEAKCUR_MASK, peak_cur);
    return 0;
}

static int aw869x_haptic_set_gain(struct aw869x *aw869x, unsigned char gain)
{
    if(gain & 0x80) {
        gain = 0x7F;
    }

    aw869x_i2c_write(aw869x, AW869X_REG_DATDBG, gain);

    return 0;
}

#ifdef AW869X_HAPTIC_VBAT_MONITOR
static int aw869x_haptic_set_bst_mode(struct aw869x *aw869x, unsigned char mode)
{
    if(mode == AW869X_HAPTIC_BYPASS_MODE) {
        aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL,
                AW869X_BIT_SYSCTRL_BST_MODE_MASK, AW869X_BIT_SYSCTRL_BST_MODE_BYPASS);
    } else if (mode == AW869X_HAPTIC_BOOST_MODE){
        aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL,
                AW869X_BIT_SYSCTRL_BST_MODE_MASK, AW869X_BIT_SYSCTRL_BST_MODE_BOOST);
    }

    return 0;
}
#endif

static int aw869x_haptic_play_que_seq(struct aw869x *aw869x, unsigned char flag)
{
    aw869x_haptic_stop(aw869x);
    if(flag) {
        aw869x_haptic_play_mode(aw869x, AW869X_HAPTIC_RAM_MODE);
        aw869x_haptic_set_repeat_seq(aw869x, 0);
        aw869x_haptic_start(aw869x);
    }
    return 0;
}

static int aw869x_haptic_play_repeat_seq(struct aw869x *aw869x, unsigned char flag)
{
    aw869x_haptic_stop(aw869x);
    if(flag) {
        aw869x_haptic_play_mode(aw869x, AW869X_HAPTIC_RAM_MODE);
        aw869x_haptic_set_repeat_seq(aw869x, 1);
        aw869x_haptic_start(aw869x);
    }

    return 0;
}

static void aw869x_haptic_set_rtp_aei(struct aw869x *aw869x, bool flag)
{
    if(flag) {
        aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSINTM,
                AW869X_BIT_SYSINTM_FF_AE_MASK, AW869X_BIT_SYSINTM_FF_AE_EN);
    } else {
        aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSINTM,
                AW869X_BIT_SYSINTM_FF_AE_MASK, AW869X_BIT_SYSINTM_FF_AE_OFF);        
    }
}
/*
static void aw869x_haptic_set_rtp_afi(struct aw869x *aw869x, bool flag)
{
    if(flag) {
        aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSINTM,
                AW869X_BIT_SYSINTM_FF_AF_MASK, AW869X_BIT_SYSINTM_FF_AF_EN);
    } else {
        aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSINTM,
                AW869X_BIT_SYSINTM_FF_AF_MASK, AW869X_BIT_SYSINTM_FF_AF_OFF);        
    }
}
*/
/*
static unsigned char aw869x_haptic_rtp_get_fifo_aei(struct aw869x *aw869x)
{
    unsigned char ret;
    unsigned char reg_val = 0;

    aw869x_i2c_read(aw869x, AW869X_REG_SYSINT, &reg_val);
    reg_val &= AW869X_BIT_SYSINT_FF_AEI;
    ret = reg_val>>4;

    return ret;
}
*/
#ifdef AW869X_HAPTIC_VBAT_MONITOR
static int aw869x_get_sys_battery_info(char *dev)
{
#ifdef AWINIC_GET_BATTERY_READ_NODE
    int fd;
    int eCheck;
    int nReadSize;
    char buf[64],*pvalue;
    mm_segment_t oldfs;

    oldfs = get_fs();
    set_fs(KERNEL_DS);
    fd = sys_open(dev, O_RDONLY, 0);
    if (fd < 0) {
        pr_err("%s: open fail dev:%s fd:%d\n", __func__, dev, fd);
        set_fs(oldfs);
        return fd;
    }

    nReadSize = sys_read(fd, buf, sizeof(buf) - 1);
    pr_debug("%s: nReadSize:%d\n", __func__, nReadSize);

    eCheck = simple_strtoul(buf,&pvalue,10);
    pr_debug("%s: eCheck = %d\n",eCheck);

    set_fs(oldfs);
    sys_close(fd);

    if (eCheck > 0)
        return eCheck;
    else
        return 0;
#else
    struct power_supply *batt_psy;
    union power_supply_propval prop = {0, };
    int rc = -1;

    batt_psy = power_supply_get_by_name("battery");
    rc = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
                        &prop);
	power_supply_put(batt_psy);
    if (rc < 0) {
        pr_err("Error in getting charging status, rc=%d\n", rc);
        return 0;
    }
	return prop.intval;
#endif
}
#endif

static unsigned char aw869x_haptic_rtp_get_fifo_afi(struct aw869x *aw869x)
{
    unsigned char ret = 0;
    unsigned char reg_val = 0;

    aw869x_i2c_read(aw869x, AW869X_REG_SYSINT, &reg_val);
    reg_val &= AW869X_BIT_SYSINT_FF_AFI;
    ret = reg_val>>3;

    return ret;
}

static int aw869x_haptic_rtp_init(struct aw869x *aw869x)
{
    unsigned int buf_len = 0;

    pr_debug("%s enter\n", __func__);

    aw869x->rtp_cnt = 0;

    while((!aw869x_haptic_rtp_get_fifo_afi(aw869x)) && 
            (aw869x->play_mode == AW869X_HAPTIC_RTP_MODE)) {
        pr_debug("%s rtp cnt = %d\n", __func__, aw869x->rtp_cnt);
        if((aw869x_rtp->len-aw869x->rtp_cnt) < (aw869x->ram.base_addr>>3)) {
            buf_len = aw869x_rtp->len-aw869x->rtp_cnt;
        } else {
            buf_len = (aw869x->ram.base_addr>>3);
        }
        aw869x_i2c_writes(aw869x, AW869X_REG_RTP_DATA,
                &aw869x_rtp->data[aw869x->rtp_cnt], buf_len);
        aw869x->rtp_cnt += buf_len;
        if(aw869x->rtp_cnt == aw869x_rtp->len) {
            pr_debug("%s: rtp update complete\n", __func__);
            aw869x->rtp_cnt = 0;
#ifdef AW869X_HAPTIC_VBAT_MONITOR
            aw869x_haptic_set_bst_mode(aw869x, AW869X_HAPTIC_BOOST_MODE);
#endif
            return 0;
        }
    }

    if(aw869x->play_mode == AW869X_HAPTIC_RTP_MODE) {
        aw869x_haptic_set_rtp_aei(aw869x, true);
    }

    pr_debug("%s exit\n", __func__);

    return 0;
}

static void aw869x_rtp_work_routine(struct work_struct *work)
{
#ifdef AW869X_HAPTIC_VBAT_MONITOR
    int i = 0;
    int tmp = 0;
    unsigned int vbat = 0;
#endif
    const struct firmware *rtp_file;
    int ret = -1;
    struct aw869x *aw869x = container_of(work, struct aw869x, rtp_work);

    pr_debug("%s enter\n", __func__);

    /* fw loaded */
    ret = request_firmware(&rtp_file,
            aw869x_rtp_name[aw869x->rtp_file_num],
            aw869x->dev);
    if(ret < 0)
    {
        pr_err("%s: failed to read %s\n", __func__,
                aw869x_rtp_name[aw869x->rtp_file_num]);
        return ;
    }
    aw869x->rtp_init = 0;
    kfree(aw869x_rtp);
    aw869x_rtp = kzalloc(rtp_file->size+sizeof(int), GFP_KERNEL);
    if (!aw869x_rtp) {
        release_firmware(rtp_file);
        pr_err("%s: error allocating memory\n", __func__);
        return;
    }
    aw869x_rtp->len = rtp_file->size;
    pr_debug("%s: rtp file [%s] size = %d\n", __func__,
            aw869x_rtp_name[aw869x->rtp_file_num], aw869x_rtp->len);
    memcpy(aw869x_rtp->data, rtp_file->data, rtp_file->size);
    release_firmware(rtp_file);

    aw869x->rtp_init = 1;

    /* rtp mode config */
    aw869x_haptic_play_mode(aw869x, AW869X_HAPTIC_RTP_MODE);
    aw869x_i2c_write_bits(aw869x, AW869X_REG_PWMDBG,
           AW869X_BIT_PWMDBG_PWMCLK_MODE_MASK, AW869X_BIT_PWMDBG_PWMCLK_MODE_12KB);
    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL,
            AW869X_BIT_SYSCTRL_WORK_MODE_MASK, AW869X_BIT_SYSCTRL_ACTIVE);
    msleep(2);
#ifdef AW869X_HAPTIC_VBAT_MONITOR
    vbat = aw869x_get_sys_battery_info(SYS_BAT_DEV);
    pr_debug("%s: get sys battery = %d\n", __func__, vbat);
    if((vbat > AW869X_SYS_VBAT_MIN) && (vbat < AW869X_SYS_VBAT_MAX)) {
        for(i=0; i<aw869x_rtp->len; i++) {
            tmp = (int)aw869x_rtp->data[i];
            tmp = tmp*AW869X_SYS_VBAT_REFERENCE/vbat;
            aw869x_rtp->data[i] = (unsigned char)tmp;
        }
    }
#endif
    aw869x_haptic_rtp_init(aw869x);
}


/*****************************************************
 *
 * haptic fops
 *
 *****************************************************/
static int aw869x_file_open(struct inode *inode, struct file *file)
{
    if (!try_module_get(THIS_MODULE)) 
        return -ENODEV;

    file->private_data = (void*)g_aw869x;

    return 0;
}

static int aw869x_file_release(struct inode *inode, struct file *file)
{
    file->private_data = (void*)NULL;

    module_put(THIS_MODULE);

    return 0;
}

static long aw869x_file_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct aw869x *aw869x = (struct aw869x *)file->private_data;

    int ret = 0;
    unsigned int temp = 0;
 
    dev_info(aw869x->dev, "%s: cmd=0x%x, arg=0x%lx\n",
              __func__, cmd, arg);

    mutex_lock(&aw869x->lock);
   
    if(_IOC_TYPE(cmd) != AW869X_HAPTIC_IOCTL_MAGIC) {
        dev_err(aw869x->dev, "%s: cmd magic err\n",
                __func__);
        return -EINVAL;
    }
 
    switch (cmd) {
        case AW869X_HAPTIC_SET_QUE_SEQ:
            ret = aw869x_set_que_seq(aw869x, arg);
            break;
        case AW869X_HAPTIC_SET_SEQ_LOOP:
            ret = aw869x_set_seq_loop(aw869x, arg);
            break;
        case AW869X_HAPTIC_PLAY_QUE_SEQ:
            if (copy_from_user(&temp, (void __user *)arg, sizeof(int)))
                return -EFAULT;
            aw869x_haptic_play_que_seq(aw869x, (unsigned char)temp);
            break;
        case AW869X_HAPTIC_SET_BST_VOL:
            if (copy_from_user(&temp, (void __user *)arg, sizeof(int)))
                return -EFAULT;
            aw869x_haptic_set_bst_vol(aw869x, (unsigned char)(temp<<3));
            break;
        case AW869X_HAPTIC_SET_BST_PEAK_CUR:
            if (copy_from_user(&temp, (void __user *)arg, sizeof(int)))
                return -EFAULT;
            aw869x_haptic_set_bst_peak_cur(aw869x, (unsigned char)(temp));
            break;
        case AW869X_HAPTIC_SET_GAIN:
            if (copy_from_user(&temp, (void __user *)arg, sizeof(int)))
                return -EFAULT;
            aw869x_haptic_set_gain(aw869x, (unsigned char)(temp));
            break;
        case AW869X_HAPTIC_PLAY_REPEAT_SEQ:
            if (copy_from_user(&temp, (void __user *)arg, sizeof(int)))
                return -EFAULT;
            aw869x_haptic_play_repeat_seq(aw869x, (unsigned char)(temp));
            break;
        default:
            dev_err(aw869x->dev, "%s, unknown cmd\n", __func__);
            break;
    }

    mutex_unlock(&aw869x->lock);
    
    return ret;
}

static ssize_t aw869x_file_read(struct file* filp, char* buff, size_t len, loff_t* offset)
{
    struct aw869x *aw869x = (struct aw869x *)filp->private_data;
    int ret = 0;
    int i = 0;
    unsigned char reg_val = 0;
    unsigned char *pbuff = NULL;
    
    mutex_lock(&aw869x->lock);

    dev_info(aw869x->dev, "%s: len=%zu\n", __func__, len);

    switch(aw869x->fileops.cmd)
    {
        case AW869X_HAPTIC_CMD_READ_REG:
            pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
            if(pbuff != NULL) {
                for(i=0; i<len; i++) {
                    aw869x_i2c_read(aw869x, aw869x->fileops.reg+i, &reg_val);
                    pbuff[i] = reg_val;
                }
                ret = copy_to_user(buff, pbuff, len);
                if(ret) {
                    dev_err(aw869x->dev, "%s: copy to user fail\n", __func__);
                }
                kfree(pbuff);
            } else {
                dev_err(aw869x->dev, "%s: alloc memory fail\n", __func__);
            }
            break;
        case AW869X_HAPTIC_CMD_READ_FIRMWARE:
            pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
            if(pbuff != NULL) {
                aw869x_haptic_stop(aw869x);
                /* RAMINIT Enable */
                aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL,
                        AW869X_BIT_SYSCTRL_RAMINIT_MASK, AW869X_BIT_SYSCTRL_RAMINIT_EN);
 
                aw869x_i2c_write(aw869x, AW869X_REG_RAMADDRH, aw869x->fileops.ram_addrh);
                aw869x_i2c_write(aw869x, AW869X_REG_RAMADDRL, aw869x->fileops.ram_addrl);
                for(i=0; i<len; i++) {
                    aw869x_i2c_read(aw869x, AW869X_REG_RAMDATA, &reg_val);
                    pbuff[i] = reg_val;
                }

                /* RAMINIT Disable */
                aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL,
                        AW869X_BIT_SYSCTRL_RAMINIT_MASK, AW869X_BIT_SYSCTRL_RAMINIT_OFF);

                ret = copy_to_user(buff, pbuff, len);
                if(ret) {
                    dev_err(aw869x->dev, "%s: copy to user fail\n", __func__);
                }
                kfree(pbuff);
            } else {
                dev_err(aw869x->dev, "%s: alloc memory fail\n", __func__);                
            }
            break;
        default:
            dev_err(aw869x->dev, "%s, unknown cmd %d \n", __func__, aw869x->fileops.cmd);
            break;
    }

    mutex_unlock(&aw869x->lock);

    for(i=0; i<len; i++) {
        dev_info(aw869x->dev, "%s: buff[%d]=0x%02x\n", 
                __func__, i, buff[i]);
    }

    return len;
}

static ssize_t aw869x_file_write(struct file* filp, const char* buff, size_t len, loff_t* off)
{
    struct aw869x *aw869x = (struct aw869x *)filp->private_data;
    int i = 0;
    int ret = 0;
    unsigned char *pbuff = NULL;

    for(i=0; i<len; i++) {
        dev_info(aw869x->dev, "%s: buff[%d]=0x%02x\n", 
                __func__, i, buff[i]);
    }

    mutex_lock(&aw869x->lock);
    
    aw869x->fileops.cmd = buff[0];
    
    switch(aw869x->fileops.cmd)
    {
        case AW869X_HAPTIC_CMD_READ_REG:
            if(len == 2) {
                aw869x->fileops.reg = buff[1];
            } else {
                dev_err(aw869x->dev, "%s: read cmd len %zu err\n", __func__, len);
            }
            break;
        case AW869X_HAPTIC_CMD_WRITE_REG:
            if(len > 2) {
                pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
                if(pbuff != NULL) {
                    ret = copy_from_user(pbuff, buff, len);
                    if(ret) {
                        dev_err(aw869x->dev, "%s: copy from user fail\n", __func__);
                    } else {
                        for(i=0; i<len-2; i++) {
                            dev_info(aw869x->dev, "%s: write reg0x%02x=0x%02x\n", 
                                    __func__, pbuff[1]+i, pbuff[i+2]);
                            aw869x_i2c_write(aw869x, pbuff[1]+i, pbuff[2+i]);
                        }
                    }
                    kfree(pbuff);
                } else {
                    dev_err(aw869x->dev, "%s: alloc memory fail\n", __func__);                
                }
            } else {
                dev_err(aw869x->dev, "%s: write cmd len %zu err\n", __func__, len);
            }
            break;
        case AW869X_HAPTIC_CMD_UPDATE_FIRMWARE:
            pbuff = (unsigned char *)kzalloc(len-1, GFP_KERNEL);
            if(pbuff != NULL) {
                ret = copy_from_user(pbuff, &buff[1], len-1);
                if(ret) {
                    dev_err(aw869x->dev, "%s: copy from user fail\n", __func__);
                } else {
                    __pm_stay_awake(aw869x->ws);

                    aw869x_haptic_stop(aw869x);
                    /* RAMINIT Enable */
                    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL,
                            AW869X_BIT_SYSCTRL_RAMINIT_MASK, AW869X_BIT_SYSCTRL_RAMINIT_EN);

                    aw869x_i2c_write(aw869x, AW869X_REG_RAMADDRH, aw869x->fileops.ram_addrh);
                    aw869x_i2c_write(aw869x, AW869X_REG_RAMADDRL, aw869x->fileops.ram_addrl);
                    for(i=0; i<len-1; i++) {
                        aw869x_i2c_write(aw869x, AW869X_REG_RAMDATA, pbuff[i]);
                    }

                    /* RAMINIT Disable */
                    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL,
                            AW869X_BIT_SYSCTRL_RAMINIT_MASK, AW869X_BIT_SYSCTRL_RAMINIT_OFF);

                    __pm_relax(aw869x->ws);
                }
                kfree(pbuff);
            } else {
                dev_err(aw869x->dev, "%s: alloc memory fail\n", __func__);                
            }

            break;
        case AW869X_HAPTIC_CMD_READ_FIRMWARE:
            if(len == 3) {
                aw869x->fileops.ram_addrh = buff[1];
                aw869x->fileops.ram_addrl = buff[2];
            } else {
                dev_err(aw869x->dev, "%s: read firmware len %zu err\n", __func__, len);
            }
            break;
        default:
            dev_err(aw869x->dev, "%s, unknown cmd %d \n", __func__, aw869x->fileops.cmd);
        break;
    }

    mutex_unlock(&aw869x->lock);

    return len;
}

static struct file_operations fops =
{
    .owner = THIS_MODULE,
    .read = aw869x_file_read,
    .write = aw869x_file_write,
    .unlocked_ioctl = aw869x_file_unlocked_ioctl,
    .open = aw869x_file_open,
    .release = aw869x_file_release,
};

static struct miscdevice aw869x_haptic_misc =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = AW869X_HAPTIC_NAME,
    .fops = &fops,
};

static int aw869x_haptic_init(struct aw869x *aw869x)
{
    int ret = 0;

    pr_info("%s enter\n", __func__);

    ret = misc_register(&aw869x_haptic_misc);
    if(ret) {
        dev_err(aw869x->dev,  "%s: misc fail: %d\n", __func__, ret);
        return ret;
    }

    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL, 
            AW869X_BIT_SYSCTRL_WORK_MODE_MASK, AW869X_BIT_SYSCTRL_STANDBY);
    aw869x_i2c_write(aw869x, AW869X_REG_ANACTRL, 0x01);
    //aw869x_i2c_write_bits(aw869x, AW869X_REG_PWMDBG,
    //        AW869X_BIT_PWMDBG_PWMCLK_MODE_MASK, AW869X_BIT_PWMDBG_PWMCLK_MODE_24KB);
    return ret;
}



/*****************************************************
 *
 * vibrator
 *
 *****************************************************/
static void aw869x_rtp_play(struct aw869x *aw869x, int value)
{
	aw869x_haptic_stop(aw869x);
	aw869x_haptic_set_rtp_aei(aw869x, false);
	aw869x_interrupt_clear(aw869x);
#if 0
#ifdef AW869X_HAPTIC_VBAT_MONITOR
	aw869x_haptic_set_bst_mode(aw869x, AW869X_HAPTIC_BYPASS_MODE);
#endif
#else
	aw869x_haptic_set_bst_vol(aw869x, AW869X_BIT_BSTCFG_BSTVOL_8V);
#endif
	if(value < (sizeof(aw869x_rtp_name)/AW869X_RTP_NAME_MAX)) {
		aw869x->rtp_file_num = value;
		if(value) {
			schedule_work(&aw869x->rtp_work);
		}
	} else {
		pr_err("%s: rtp_file_num 0x%02x over max value \n", __func__, aw869x->rtp_file_num);
	}
}

static void aw869x_haptic_context(struct aw869x *aw869x, enum aw869x_haptic_mode cmd)
{
	int t_top = 0;
	if (!gpio_is_valid(aw869x->haptic_context_gpio)) {
		pr_debug("%s haptic context gpio is invalid \n", __func__);
		return;
	}

	t_top = gpio_get_value(aw869x->haptic_context_gpio);
	if (t_top) {
		switch (cmd) {
		case HAPTIC_RTP:
			aw869x->gain = 0x20;
			break;
		case HAPTIC_SHORT:
			aw869x->gain = 0x20;
			break;
		case HAPTIC_LONG:
			aw869x->gain = 0x06;
			break;
		default:
			break;
		}
	}
}

static void aw869x_vibrate(struct aw869x *aw869x, int value)
{
	int seq;
	mutex_lock(&aw869x->lock);

	aw869x_haptic_stop(aw869x);
	seq = (aw869x->seq >> ((AW869X_SEQUENCER_SIZE - 1) * 8)) & 0xFF;
	pr_debug("%s: value=%d, seq=%d\n", __FUNCTION__, value, seq);

	if (value > 0 || seq > 2) {

		if (seq >= AW869X_SEQ_NO_RTP_BASE) {
			aw869x->haptic_mode = HAPTIC_RTP;
			aw869x->gain = 0x20;
		} else if (value < 100 || seq > 2) {
			aw869x->haptic_mode = HAPTIC_SHORT;
			aw869x->gain = 0x20;
		} else {
			aw869x->haptic_mode = HAPTIC_LONG;
			aw869x->gain = 0x0e;
		}

		if(!aw869x->factory_mode)
			aw869x_haptic_context(aw869x,aw869x->haptic_mode);

		if (aw869x->debugfs_debug)
			aw869x_haptic_set_gain(aw869x, aw869x->gain_debug);
		else
			aw869x_haptic_set_gain(aw869x, aw869x->gain);

		switch (aw869x->haptic_mode) {
		case HAPTIC_RTP:
			aw869x_rtp_play(aw869x, seq - AW869X_SEQ_NO_RTP_BASE);
			break;
		case HAPTIC_SHORT:
			aw869x_i2c_write_bits(aw869x, AW869X_REG_PWMDBG,
				AW869X_BIT_PWMDBG_PWMCLK_MODE_MASK,
				AW869X_BIT_PWMDBG_PWMCLK_MODE_12KB);
			aw869x_haptic_set_bst_vol(aw869x, AW869X_BIT_BSTCFG_BSTVOL_8P75V);
			//aw869x_haptic_set_peak_cur(aw869x, AW869X_BIT_BSTCFG_PEAKCUR_3P5A);

			if (aw869x->seq == 0)
				aw869x->seq = 0x01000000;

			aw869x_haptic_set_que_seq(aw869x, aw869x->seq);
			//aw869x_haptic_set_repeat_seq(aw869x, 0);
			//aw869x->index = 0x01;
			aw869x_haptic_play_que_seq(aw869x, 0x01);
		/*
			value = (value>HAPTIC_MAX_TIMEOUT)? HAPTIC_MAX_TIMEOUT:value;
			hrtimer_start(&aw869x->timer,
			ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		*/
			break;
		case HAPTIC_LONG:
			aw869x_i2c_write_bits(aw869x, AW869X_REG_PWMDBG,
				AW869X_BIT_PWMDBG_PWMCLK_MODE_MASK,
				AW869X_BIT_PWMDBG_PWMCLK_MODE_12KB);
			aw869x_haptic_set_bst_vol(aw869x, AW869X_BIT_BSTCFG_BSTVOL_6P25V);

			aw869x->duration = value;
			/* wav index config */
			aw869x->index = 0x02;
			aw869x_haptic_set_repeat_que_seq(aw869x, aw869x->index);

			__pm_wakeup_event(aw869x->ws, value + 100);
			/* run ms timer */
			hrtimer_cancel(&aw869x->timer);
			aw869x->state = 0x01;
			if (aw869x->state)
			{
			hrtimer_start(&aw869x->timer,
				ktime_set(aw869x->duration / 1000, (value % 1000) * 1000000),
				HRTIMER_MODE_REL);
			}
			schedule_work(&aw869x->vibrator_work);
			break;
		default:
			break;
		}

		/* Restore to default short waveform */
		if (seq > 2)
			aw869x->seq = 0;
	}

    mutex_unlock(&aw869x->lock);
}

#ifdef TIMED_OUTPUT
static int aw869x_vibrator_get_time(struct timed_output_dev *dev)
{
    struct aw869x *aw869x = container_of(dev, struct aw869x, to_dev);

    if (hrtimer_active(&aw869x->timer)) {
        ktime_t r = hrtimer_get_remaining(&aw869x->timer);
        return ktime_to_ms(r);
    }

    return 0;
}

static void aw869x_vibrator_enable( struct timed_output_dev *dev, int value)
{
    struct aw869x *aw869x = container_of(dev, struct aw869x, to_dev);

    pr_debug("%s enter, value=%d\n", __func__, value);
    aw869x_vibrate(aw869x, value);
    pr_debug("%s exit\n", __func__);
}

#else
static enum led_brightness aw869x_haptic_brightness_get(struct led_classdev *cdev)
{
    struct aw869x *aw869x =
        container_of(cdev, struct aw869x, cdev);

    return aw869x->amplitude;
}

static void aw869x_haptic_brightness_set(struct led_classdev *cdev,
                enum led_brightness level)
{
    struct aw869x *aw869x =
        container_of(cdev, struct aw869x, cdev);

    aw869x->amplitude = level;

    mutex_lock(&aw869x->lock);

    aw869x_haptic_stop(aw869x);
    if (aw869x->amplitude > 0) {
        aw869x_haptic_play_que_seq(aw869x, aw869x->amplitude);
    }

    mutex_unlock(&aw869x->lock);

}
#endif

static ssize_t aw869x_extra_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t aw869x_extra_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t aw869x_state_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif

    return snprintf(buf, PAGE_SIZE, "%d\n", aw869x->state);
}

static ssize_t aw869x_state_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t aw869x_duration_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    ktime_t time_rem;
    s64 time_ms = 0;

    if (hrtimer_active(&aw869x->timer)) {
        time_rem = hrtimer_get_remaining(&aw869x->timer);
        time_ms = ktime_to_ms(time_rem);
    }

    return snprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t aw869x_duration_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    /* setting 0 on duration is NOP for now */
    if (val <= 0)
        return count;

    aw869x->duration = val;

    return count;
}

static ssize_t aw869x_activate_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif

    /* For now nothing to show */
    return snprintf(buf, PAGE_SIZE, "%d\n", aw869x->state);
}

static ssize_t aw869x_activate_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    if (val != 0 && val != 1)
        return count;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    aw869x->state = val;

    if (aw869x->state)
        aw869x_vibrate(aw869x, aw869x->duration);
    else
        aw869x_vibrate(aw869x, 0);

    return count;
}

static ssize_t aw869x_index_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned char reg_val = 0;
    aw869x_i2c_read(aw869x, AW869X_REG_QUE_SEQ1, &reg_val);
    aw869x->index = reg_val;

    return snprintf(buf, PAGE_SIZE, "%d\n", aw869x->index);
}

static ssize_t aw869x_index_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    mutex_lock(&aw869x->lock);
    aw869x->index = val;
    aw869x_haptic_set_repeat_que_seq(aw869x, aw869x->index);
    mutex_unlock(&aw869x->lock);
    return count;
}

static ssize_t aw869x_vmax_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif

    return snprintf(buf, PAGE_SIZE, "%d\n", aw869x->vmax);
}

static ssize_t aw869x_vmax_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    mutex_lock(&aw869x->lock);
    aw869x->vmax = val;
    aw869x_haptic_set_bst_vol(aw869x, (unsigned char)(aw869x->vmax<<3));
    mutex_unlock(&aw869x->lock);
    return count;
}

static ssize_t aw869x_gain_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif

    return snprintf(buf, PAGE_SIZE, "%d\n", aw869x->gain_debug);
}

static ssize_t aw869x_gain_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    mutex_lock(&aw869x->lock);
	if (val > 0)
		aw869x->debugfs_debug = true;
	else
		aw869x->debugfs_debug = false;
    aw869x->gain_debug = val;
    aw869x_haptic_set_gain(aw869x, (unsigned char)aw869x->gain_debug);
    mutex_unlock(&aw869x->lock);
    return count;
}

static ssize_t aw869x_seq_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    size_t count = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;

    aw869x->seq = 0;
    for(i=0; i<AW869X_SEQUENCER_SIZE; i++) {
        aw869x_i2c_read(aw869x, AW869X_REG_QUE_SEQ1+i, &reg_val);
        count += snprintf(buf+count, PAGE_SIZE-count,
                "seq%d: 0x%02x\n", i+1, reg_val);
        aw869x->seq |= (reg_val<<((AW869X_SEQUENCER_SIZE-i-1)*8));
    }
    return count;
}

static ssize_t aw869x_seq_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%x\n", __FUNCTION__, val);

    mutex_lock(&aw869x->lock);
    aw869x->seq = val;
    aw869x_haptic_set_que_seq(aw869x, aw869x->seq);
    mutex_unlock(&aw869x->lock);
    return count;
}

static ssize_t aw869x_loop_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    size_t count = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;
    unsigned char tmp[2];
    unsigned char loop_tmp = 0;

    aw869x->loop = 0;
    for(i=0; i<AW869X_SEQUENCER_LOOP_SIZE; i++) {
        aw869x_i2c_read(aw869x, AW869X_REG_SEQ_LOOP1+i, &reg_val);
        tmp[0] = reg_val>>4;
        tmp[1] = reg_val&0x0F;
        loop_tmp = (tmp[0]<<8)|tmp[1];
        count += snprintf(buf+count, PAGE_SIZE-count,
                "seq%d loop: 0x%02x\n", i*2+1, tmp[0]);
        count += snprintf(buf+count, PAGE_SIZE-count,
                "seq%d loop: 0x%02x\n", i*2+2, tmp[1]);
        aw869x->loop |= (loop_tmp<<((AW869X_SEQUENCER_LOOP_SIZE-i-1)*8));
    }
    return count;
}

static ssize_t aw869x_loop_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%x\n", __FUNCTION__, val);

    mutex_lock(&aw869x->lock);
    aw869x->loop = val;
    aw869x_haptic_set_que_loop(aw869x, aw869x->loop);
    mutex_unlock(&aw869x->lock);
    return count;
}

static ssize_t aw869x_reg_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    ssize_t len = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;
    for(i = 0; i < AW869X_REG_MAX; i ++) {
        if(!(aw869x_reg_access[i]&REG_RD_ACCESS))
           continue;
        aw869x_i2c_read(aw869x, i, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
    }
    return len;
}

static ssize_t aw869x_reg_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int databuf[2] = {0, 0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw869x_i2c_write(aw869x, (unsigned char)databuf[0], (unsigned char)databuf[1]);
    }

    return count;
}

static ssize_t aw869x_rtp_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    //struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    //struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    //struct led_classdev *cdev = dev_get_drvdata(dev);
    //struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "rtp mode\n");
    return len;
}

static ssize_t aw869x_rtp_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    aw869x_rtp_play(aw869x, val);

    return count;
}

static ssize_t aw869x_ram_update_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    //struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    //struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    //struct led_classdev *cdev = dev_get_drvdata(dev);
    //struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "ram/rtp mode\n");
    return len;
}

static ssize_t aw869x_ram_update_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    if(val) {
        aw869x_ram_update(aw869x);
    }
    return count;
}

#ifdef CONFIG_INPUT_AWINIC_HAPTIC
static ssize_t aw869x_pwk_p_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned char reg_val = 0;
    aw869x_i2c_read(aw869x, AW869X_REG_TRG1_WAV_P, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw869x_pwk_p_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int databuf = 0;
    if(1 == sscanf(buf, "%x", &databuf)) {
        aw869x_i2c_write(aw869x, AW869X_REG_TRG1_WAV_P, (unsigned char)databuf);
    }

    return count;
}

static ssize_t aw869x_pwk_n_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned char reg_val = 0;
    aw869x_i2c_read(aw869x, AW869X_REG_TRG1_WAV_N, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw869x_pwk_n_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int databuf = 0;
    if(1 == sscanf(buf, "%x", &databuf)) {
        aw869x_i2c_write(aw869x, AW869X_REG_TRG1_WAV_N, (unsigned char)databuf);
    }

    return count;
}

static ssize_t aw869x_voldown_p_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned char reg_val = 0;
    aw869x_i2c_read(aw869x, AW869X_REG_TRG2_WAV_P, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw869x_voldown_p_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int databuf = 0;
    if(1 == sscanf(buf, "%x", &databuf)) {
        aw869x_i2c_write(aw869x, AW869X_REG_TRG2_WAV_P, (unsigned char)databuf);
    }

    return count;
}

static ssize_t aw869x_voldown_n_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned char reg_val = 0;
    aw869x_i2c_read(aw869x, AW869X_REG_TRG2_WAV_N, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw869x_voldown_n_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int databuf = 0;
    if(1 == sscanf(buf, "%x", &databuf)) {
        aw869x_i2c_write(aw869x, AW869X_REG_TRG2_WAV_N, (unsigned char)databuf);
    }

    return count;
}
static ssize_t aw869x_volup_p_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned char reg_val = 0;
    aw869x_i2c_read(aw869x, AW869X_REG_TRG3_WAV_P, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw869x_volup_p_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int databuf = 0;
    if(1 == sscanf(buf, "%x", &databuf)) {
        aw869x_i2c_write(aw869x, AW869X_REG_TRG3_WAV_P, (unsigned char)databuf);
    }

    return count;
}

static ssize_t aw869x_volup_n_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned char reg_val = 0;
    aw869x_i2c_read(aw869x, AW869X_REG_TRG3_WAV_N, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw869x_volup_n_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int databuf = 0;
    if(1 == sscanf(buf, "%x", &databuf)) {
        aw869x_i2c_write(aw869x, AW869X_REG_TRG3_WAV_N, (unsigned char)databuf);
    }

    return count;
}
static ssize_t aw869x_reset_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned char reg_val = 0;
    aw869x_i2c_read(aw869x, AW869X_REG_ID, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw869x_reset_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(to_dev, struct aw869x, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw869x *aw869x = container_of(cdev, struct aw869x, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    if(1 == val) {
        aw869x_i2c_write(aw869x, AW869X_REG_ID, 0xAA);
    }

    return count;
}
#endif

static DEVICE_ATTR(extra, S_IWUSR | S_IRUGO, aw869x_extra_show, aw869x_extra_store);
static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, aw869x_state_show, aw869x_state_store);
static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, aw869x_duration_show, aw869x_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, aw869x_activate_show, aw869x_activate_store);
static DEVICE_ATTR(index, S_IWUSR | S_IRUGO, aw869x_index_show, aw869x_index_store);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO, aw869x_vmax_show, aw869x_vmax_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO, aw869x_gain_show, aw869x_gain_store);
static DEVICE_ATTR(seq, S_IWUSR | S_IRUGO, aw869x_seq_show, aw869x_seq_store);
static DEVICE_ATTR(loop, S_IWUSR | S_IRUGO, aw869x_loop_show, aw869x_loop_store);
static DEVICE_ATTR(register, S_IWUSR | S_IRUGO, aw869x_reg_show, aw869x_reg_store);
static DEVICE_ATTR(rtp, S_IWUSR | S_IRUGO, aw869x_rtp_show, aw869x_rtp_store);
static DEVICE_ATTR(ram_update, S_IWUSR | S_IRUGO, aw869x_ram_update_show, aw869x_ram_update_store);
#ifdef CONFIG_INPUT_AWINIC_HAPTIC
static DEVICE_ATTR(pwk_p, S_IWUSR | S_IRUGO, aw869x_pwk_p_show, aw869x_pwk_p_store);/* Power key for trig1(0x0d) */
static DEVICE_ATTR(voldown_p, S_IWUSR | S_IRUGO, aw869x_voldown_p_show, aw869x_voldown_p_store);/* Vol down for trig2(0x0e) */
static DEVICE_ATTR(volup_p, S_IWUSR | S_IRUGO, aw869x_volup_p_show, aw869x_volup_p_store);/* Vol up key for trig3(0x0f) */
static DEVICE_ATTR(pwk_n, S_IWUSR | S_IRUGO, aw869x_pwk_n_show, aw869x_pwk_n_store);/* Power key for trig1_n(0x10) */
static DEVICE_ATTR(voldown_n, S_IWUSR | S_IRUGO, aw869x_voldown_n_show, aw869x_voldown_n_store);/* Vol down for trig2_n(0x11) */
static DEVICE_ATTR(volup_n, S_IWUSR | S_IRUGO, aw869x_volup_n_show, aw869x_volup_n_store);/* Vol up key for trig3_n(0x12) */
static DEVICE_ATTR(reset, S_IWUSR | S_IRUGO, aw869x_reset_show, aw869x_reset_store);/* Reset device */
#endif

static struct attribute *aw869x_vibrator_attributes[] = {
    &dev_attr_extra.attr,
    &dev_attr_state.attr,
    &dev_attr_duration.attr,
    &dev_attr_activate.attr,
    &dev_attr_index.attr,
    &dev_attr_vmax.attr,
    &dev_attr_gain.attr,
    &dev_attr_seq.attr,
    &dev_attr_loop.attr,
    &dev_attr_register.attr,
    &dev_attr_rtp.attr,
    &dev_attr_ram_update.attr,
#ifdef CONFIG_INPUT_AWINIC_HAPTIC
    &dev_attr_pwk_p.attr,
    &dev_attr_voldown_p.attr,
    &dev_attr_volup_p.attr,
    &dev_attr_reset.attr,
    &dev_attr_pwk_n.attr,
    &dev_attr_voldown_n.attr,
    &dev_attr_volup_n.attr,
#endif
    NULL
};

static struct attribute_group aw869x_vibrator_attribute_group = {
    .attrs = aw869x_vibrator_attributes
};

static enum hrtimer_restart aw869x_vibrator_timer_func(struct hrtimer *timer)
{
    struct aw869x *aw869x = container_of(timer, struct aw869x, timer);

    pr_debug("%s enter\n", __func__);
    aw869x->state = 0;
    schedule_work(&aw869x->vibrator_work);
    
    return HRTIMER_NORESTART;
}

static void aw869x_vibrator_work_routine(struct work_struct *work)
{
    struct aw869x *aw869x = container_of(work, struct aw869x, vibrator_work);

    pr_debug("%s enter\n", __func__);

    mutex_lock(&aw869x->lock);
    
    if(aw869x->state) {
        //aw869x_haptic_set_repeat_que_seq(aw869x, aw869x->index);
        aw869x_haptic_play_mode(aw869x, AW869X_HAPTIC_RAM_MODE);
        aw869x_haptic_set_repeat_seq(aw869x, 1);
        aw869x_haptic_start(aw869x);
    } else {
        aw869x_haptic_stop(aw869x);
    }
    mutex_unlock(&aw869x->lock);
}

static int aw869x_vibrator_init(struct aw869x *aw869x)
{
    int ret = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;

    pr_info("%s enter\n", __func__);

    ret = aw869x_i2c_read(aw869x, AW869X_REG_QUE_SEQ1, &reg_val);
    aw869x->index = reg_val & 0x7F;
    ret = aw869x_i2c_read(aw869x, AW869X_REG_DATDBG, &reg_val);
    aw869x->gain = reg_val & 0x7F;
    ret = aw869x_i2c_read(aw869x, AW869X_REG_BSTCFG, &reg_val);
    aw869x->vmax = (reg_val >> 3);
    for(i=0; i<AW869X_SEQUENCER_SIZE; i++) {
        ret = aw869x_i2c_read(aw869x, AW869X_REG_QUE_SEQ1+i, &reg_val);
        aw869x->seq |= (reg_val<<((AW869X_SEQUENCER_SIZE-i-1)*8));
    }
#ifdef TIMED_OUTPUT
    aw869x->to_dev.name = "vibrator";
    aw869x->to_dev.get_time = aw869x_vibrator_get_time;
    aw869x->to_dev.enable = aw869x_vibrator_enable;

    ret = timed_output_dev_register(&(aw869x->to_dev));
    if ( ret < 0){
        dev_err(aw869x->dev, "%s: fail to create timed output dev\n",
                __func__);
        return ret;
    }
    ret = sysfs_create_group(&aw869x->to_dev.dev->kobj, &aw869x_vibrator_attribute_group);
    if (ret < 0) {
        dev_err(aw869x->dev, "%s error creating sysfs attr files\n", __func__);
        return ret;
    }
#else
    aw869x->cdev.name = "vibrator";
    aw869x->cdev.brightness_get = aw869x_haptic_brightness_get;
    aw869x->cdev.brightness_set = aw869x_haptic_brightness_set;
    
    ret = devm_led_classdev_register(&aw869x->i2c->dev, &aw869x->cdev);
    if (ret < 0){
        dev_err(aw869x->dev, "%s: fail to create led dev\n",
                __func__);
        return ret;
    }
    ret = sysfs_create_group(&aw869x->cdev.dev->kobj, &aw869x_vibrator_attribute_group);
    if (ret < 0) {
        dev_err(aw869x->dev, "%s error creating sysfs attr files\n", __func__);
        return ret;
     }
#endif
    hrtimer_init(&aw869x->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    aw869x->timer.function = aw869x_vibrator_timer_func;
    INIT_WORK(&aw869x->vibrator_work, aw869x_vibrator_work_routine);

    INIT_WORK(&aw869x->rtp_work, aw869x_rtp_work_routine);

    aw869x->ws = wakeup_source_register("vibrator");
    if (!aw869x->ws)
        return -ENOMEM;

    mutex_init(&aw869x->lock);

    return 0;
}



/*****************************************************
 *
 * regmap
 *
 *****************************************************/
static bool aw869x_writeable_register(struct device *dev, unsigned int reg)
{
    /* enable read access for all registers */
    return true;
}

static bool aw869x_readable_register(struct device *dev, unsigned int reg)
{
    /* enable read access for all registers */
    return true;
}

static bool aw869x_volatile_register(struct device *dev, unsigned int reg)
{
    /* enable read access for all registers */
    return true;
}

static const struct regmap_config aw869x_regmap = {
    .reg_bits = 8,
    .val_bits = 8,

    .max_register = AW869X_REG_MAX,
    .writeable_reg = aw869x_writeable_register,
    .readable_reg = aw869x_readable_register,
    .volatile_reg = aw869x_volatile_register,
    .cache_type = REGCACHE_RBTREE,
};



/******************************************************
 *
 * irq 
 *
 ******************************************************/
static void aw869x_interrupt_clear(struct aw869x *aw869x)
{
    unsigned char reg_val = 0;
    pr_debug("%s enter\n", __func__);
    aw869x_i2c_read(aw869x, AW869X_REG_SYSINT, &reg_val);
    pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
}

static void aw869x_interrupt_setup(struct aw869x *aw869x)
{
    unsigned char reg_val = 0;

    pr_info("%s enter\n", __func__);

    aw869x_i2c_read(aw869x, AW869X_REG_SYSINT, &reg_val);
    pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);
/*
    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSINTM,
            AW869X_BIT_SYSINTM_UVLO_MASK, AW869X_BIT_SYSINTM_UVLO_EN);
    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSINTM,
            AW869X_BIT_SYSINTM_OCD_MASK, AW869X_BIT_SYSINTM_OCD_EN);
    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSINTM,
            AW869X_BIT_SYSINTM_OT_MASK, AW869X_BIT_SYSINTM_OT_EN);
*/
}

static irqreturn_t aw869x_irq(int irq, void *data)
{
    struct aw869x *aw869x = data;
    unsigned char reg_val = 0;
    unsigned int buf_len = 0;

    pr_debug("%s enter\n", __func__);

    aw869x_i2c_read(aw869x, AW869X_REG_SYSINT, &reg_val);
    pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);

    if(reg_val & AW869X_BIT_SYSINT_UVLI) {
        pr_err("%s chip uvlo int error\n", __func__);
    }
    if(reg_val & AW869X_BIT_SYSINT_OCDI) {
        pr_err("%s chip over current int error\n", __func__);
    }
    if(reg_val & AW869X_BIT_SYSINT_OTI) {
        pr_err("%s chip over temperature int error\n", __func__);
    }
    if(reg_val & AW869X_BIT_SYSINT_DONEI) {
        pr_debug("%s chip playback done\n", __func__);
    }

    if(reg_val & AW869X_BIT_SYSINT_FF_AEI) {
        pr_debug("%s: aw869x rtp fifo almost empty int\n", __func__);
        if(aw869x->rtp_init) {
            while((!aw869x_haptic_rtp_get_fifo_afi(aw869x)) && 
                    (aw869x->play_mode == AW869X_HAPTIC_RTP_MODE)) {
                pr_debug("%s: aw869x rtp mode fifo update, cnt=%d\n",
                        __func__, aw869x->rtp_cnt);
                if((aw869x_rtp->len-aw869x->rtp_cnt) < (aw869x->ram.base_addr>>3)) {
                    buf_len = aw869x_rtp->len-aw869x->rtp_cnt;
                } else {
                    buf_len = (aw869x->ram.base_addr>>3);
                }
                aw869x_i2c_writes(aw869x, AW869X_REG_RTP_DATA,
                        &aw869x_rtp->data[aw869x->rtp_cnt], buf_len);
                aw869x->rtp_cnt += buf_len;
                if(aw869x->rtp_cnt == aw869x_rtp->len) {
                    pr_debug("%s: rtp update complete\n", __func__);
                    aw869x_haptic_set_rtp_aei(aw869x, false);
                    aw869x->rtp_cnt = 0;
                    aw869x->rtp_init = 0;
#ifdef AW869X_HAPTIC_VBAT_MONITOR
                    aw869x_haptic_set_bst_mode(aw869x, AW869X_HAPTIC_BOOST_MODE);
#endif
#ifdef AW869X_REPEAT_RTP_PLAYING
                    aw869x_rtp_play(aw869x, aw869x->rtp_file_num);
#endif
                    break;
                }
            }
        } else {
            pr_err("%s: aw869x rtp init = %d, init error\n", __func__, aw869x->rtp_init);
        }
    }

    if(reg_val & AW869X_BIT_SYSINT_FF_AFI) {
        pr_debug("%s: aw869x rtp mode fifo full empty\n", __func__);
    }

    if(aw869x->play_mode != AW869X_HAPTIC_RTP_MODE) {
        aw869x_haptic_set_rtp_aei(aw869x, false);
    }

    aw869x_i2c_read(aw869x, AW869X_REG_SYSINT, &reg_val);
    pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
    aw869x_i2c_read(aw869x, AW869X_REG_SYSST, &reg_val);
    pr_debug("%s: reg SYSST=0x%x\n", __func__, reg_val);

    pr_debug("%s exit\n", __func__);

    return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw869x_parse_dt(struct device *dev, struct aw869x *aw869x,
        struct device_node *np) {
    aw869x->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
    if (aw869x->reset_gpio < 0) {
        dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
        return -1;
    } else {
        dev_info(dev, "%s: reset gpio provided ok\n", __func__);
    }
    aw869x->irq_gpio =  of_get_named_gpio(np, "irq-gpio", 0);
    if (aw869x->irq_gpio < 0) {
        dev_err(dev, "%s: no irq gpio provided.\n", __func__);
    } else {
        dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
    }

    aw869x->haptic_context_gpio = of_get_named_gpio(np, "haptic-context-gpio", 0);
    if (aw869x->haptic_context_gpio < 0) {
        dev_err(dev, "%s: no haptic context gpio provided.\n", __func__);
    } else {
        dev_info(dev, "%s: haptic context gpio provided ok.\n", __func__);
    }

    return 0;
}

static int aw869x_hw_reset(struct aw869x *aw869x)
{
    pr_info("%s enter\n", __func__);

    if (aw869x && gpio_is_valid(aw869x->reset_gpio)) {
        gpio_set_value_cansleep(aw869x->reset_gpio, 0);
        msleep(1);
        gpio_set_value_cansleep(aw869x->reset_gpio, 1);
        msleep(1);
    } else {
        dev_err(aw869x->dev, "%s:  failed\n", __func__);
    }
    return 0;
}


/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw869x_read_chipid(struct aw869x *aw869x)
{
    int ret = -1;
    unsigned char cnt = 0;
    unsigned char reg = 0;

    while(cnt < AW_READ_CHIPID_RETRIES) {
        /* hardware reset */
        aw869x_hw_reset(aw869x);

        ret = aw869x_i2c_read(aw869x, AW869X_REG_ID, &reg);
        if (ret < 0) {
            dev_err(aw869x->dev, "%s: failed to read register AW869X_REG_ID: %d\n", __func__, ret);
        }
        switch (reg) {
        case 0x91:
            pr_info("%s aw8691 detected\n", __func__);
            aw869x->chipid = AW8691_ID;
            //aw869x->flags |= AW869X_FLAG_SKIP_INTERRUPTS;
            return 0;
        default:
            pr_info("%s unsupported device revision (0x%x)\n", __func__, reg );
            break;
        }
        cnt ++;

        msleep(AW_READ_CHIPID_RETRY_DELAY);
    }

    return -EINVAL;
}


/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw869x_i2c_reg_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct aw869x *aw869x = dev_get_drvdata(dev);

    unsigned int databuf[2] = {0, 0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw869x_i2c_write(aw869x, (unsigned char)databuf[0], (unsigned char)databuf[1]);
    }

    return count;
}

static ssize_t aw869x_i2c_reg_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    struct aw869x *aw869x = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;
    for(i = 0; i < AW869X_REG_MAX; i ++) {
        if(!(aw869x_reg_access[i]&REG_RD_ACCESS))
           continue;
        aw869x_i2c_read(aw869x, i, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
    }
    return len;
}
static ssize_t aw869x_i2c_ram_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct aw869x *aw869x = dev_get_drvdata(dev);

    unsigned int databuf[1] = {0};

    if(1 == sscanf(buf, "%x", &databuf[0])) {
        if(1 == databuf[0]) {
            aw869x_ram_update(aw869x);
        }
    }

    return count;
}

static ssize_t aw869x_i2c_ram_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    struct aw869x *aw869x = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned int i = 0;
    unsigned char reg_val = 0;

    aw869x_haptic_stop(aw869x);
    /* RAMINIT Enable */
    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL,
            AW869X_BIT_SYSCTRL_RAMINIT_MASK, AW869X_BIT_SYSCTRL_RAMINIT_EN);

    aw869x_i2c_write(aw869x, AW869X_REG_RAMADDRH, (unsigned char)(aw869x->ram.base_addr>>8));
    aw869x_i2c_write(aw869x, AW869X_REG_RAMADDRL, (unsigned char)(aw869x->ram.base_addr&0x00ff));
    len += snprintf(buf+len, PAGE_SIZE-len, "aw869x_haptic_ram:\n");
    for(i=0; i<aw869x->ram.len; i++) {
        aw869x_i2c_read(aw869x, AW869X_REG_RAMDATA, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len, "0x%02x,", reg_val);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    /* RAMINIT Disable */
    aw869x_i2c_write_bits(aw869x, AW869X_REG_SYSCTRL,
            AW869X_BIT_SYSCTRL_RAMINIT_MASK, AW869X_BIT_SYSCTRL_RAMINIT_OFF);

    return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw869x_i2c_reg_show, aw869x_i2c_reg_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw869x_i2c_ram_show, aw869x_i2c_ram_store);

static struct attribute *aw869x_attributes[] = {
    &dev_attr_reg.attr,
    &dev_attr_ram.attr,
    NULL
};

static struct attribute_group aw869x_attribute_group = {
    .attrs = aw869x_attributes
};


static bool mmi_factory_check(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;

	if (np)
		factory = of_property_read_bool(np, "mmi,factory-cable");

	of_node_put(np);

	return factory;
}

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw869x_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    struct aw869x *aw869x;
    struct device_node *np = i2c->dev.of_node;
    int irq_flags = 0;
    int ret = -1;

    pr_info("%s enter\n", __func__);

    if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
        dev_err(&i2c->dev, "check_functionality failed\n");
        return -EIO;
    }

    aw869x = devm_kzalloc(&i2c->dev, sizeof(struct aw869x), GFP_KERNEL);
    if (aw869x == NULL)
        return -ENOMEM;

    aw869x->dev = &i2c->dev;
    aw869x->i2c = i2c;

    /* aw869x regmap */
    aw869x->regmap = devm_regmap_init_i2c(i2c, &aw869x_regmap);
    if (IS_ERR(aw869x->regmap)) {
        ret = PTR_ERR(aw869x->regmap);
        dev_err(&i2c->dev, "%s: failed to allocate register map: %d\n", __func__, ret);
        goto err;
    }

    i2c_set_clientdata(i2c, aw869x);

    /* aw869x rst & int */
    if (np) {
        ret = aw869x_parse_dt(&i2c->dev, aw869x, np);
        if (ret) {
            dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
            goto err;
        }
    } else {
        aw869x->reset_gpio = -1;
        aw869x->irq_gpio = -1;
    }

    if (gpio_is_valid(aw869x->reset_gpio)) {
        ret = devm_gpio_request_one(&i2c->dev, aw869x->reset_gpio,
            GPIOF_OUT_INIT_LOW, "aw869x_rst");
        if (ret){
            dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
            goto err;
        }
    }

    if (gpio_is_valid(aw869x->irq_gpio)) {
        ret = devm_gpio_request_one(&i2c->dev, aw869x->irq_gpio,
            GPIOF_DIR_IN, "aw869x_int");
        if (ret){
            dev_err(&i2c->dev, "%s: int request failed\n", __func__);
            goto err;
        }
    }

    /* aw869x chip id */
    ret = aw869x_read_chipid(aw869x);
    if (ret < 0) {
        dev_err(&i2c->dev, "%s: aw869x_read_chipid failed ret=%d\n", __func__, ret);
        goto err_id;
    }

    /* aw869x irq */
    if (gpio_is_valid(aw869x->irq_gpio) &&
        !(aw869x->flags & AW869X_FLAG_SKIP_INTERRUPTS)) {
        /* register irq handler */
        aw869x_interrupt_setup(aw869x);
        irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
        ret = devm_request_threaded_irq(&i2c->dev,
                    gpio_to_irq(aw869x->irq_gpio),
                    NULL, aw869x_irq, irq_flags,
                    "aw869x", aw869x);
        if (ret != 0) {
            dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
                    __func__, gpio_to_irq(aw869x->irq_gpio), ret);
            goto err_irq;
        }
    } else {
        dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
        /* disable feature support if gpio was invalid */
        aw869x->flags |= AW869X_FLAG_SKIP_INTERRUPTS;
    }

    dev_set_drvdata(&i2c->dev, aw869x);

    ret = sysfs_create_group(&i2c->dev.kobj, &aw869x_attribute_group);
    if (ret < 0) {
        dev_info(&i2c->dev, "%s error creating sysfs attr files\n", __func__);
        goto err_sysfs;
    }

    g_aw869x = aw869x;

    aw869x->factory_mode = mmi_factory_check();

    aw869x_vibrator_init(aw869x);

    aw869x_haptic_init(aw869x);

    aw869x_ram_init(aw869x);

    pr_info("%s probe completed successfully!\n", __func__);

    return 0;

err_sysfs:
    sysfs_remove_group(&i2c->dev.kobj, &aw869x_attribute_group);
err_irq:
err_id:
err:
    return ret;
}

static int aw869x_i2c_remove(struct i2c_client *i2c)
{
    struct aw869x *aw869x = i2c_get_clientdata(i2c);

    pr_info("%s enter\n", __func__);

    sysfs_remove_group(&i2c->dev.kobj, &aw869x_attribute_group);

    misc_deregister(&aw869x_haptic_misc);

    if (gpio_is_valid(aw869x->irq_gpio))
        devm_gpio_free(&i2c->dev, aw869x->irq_gpio);
    if (gpio_is_valid(aw869x->reset_gpio))
        devm_gpio_free(&i2c->dev, aw869x->reset_gpio);

    wakeup_source_unregister(aw869x->ws);

    return 0;
}

static const struct i2c_device_id aw869x_i2c_id[] = {
    { AW869X_I2C_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, aw869x_i2c_id);

static struct of_device_id aw869x_dt_match[] = {
    { .compatible = "awinic,aw869x_haptic" },
    { },
};

static struct i2c_driver aw869x_i2c_driver = {
    .driver = {
        .name = AW869X_I2C_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(aw869x_dt_match),
    },
    .probe = aw869x_i2c_probe,
    .remove = aw869x_i2c_remove,
    .id_table = aw869x_i2c_id,
};


static int __init aw869x_i2c_init(void)
{
    int ret = 0;

    pr_info("aw869x driver version %s\n", AW869X_VERSION);

    ret = i2c_add_driver(&aw869x_i2c_driver);
    if(ret){
        pr_err("fail to add aw869x device into i2c\n");
        return ret;
    }

    return 0;
}
module_init(aw869x_i2c_init);


static void __exit aw869x_i2c_exit(void)
{
    i2c_del_driver(&aw869x_i2c_driver);
}
module_exit(aw869x_i2c_exit);


MODULE_DESCRIPTION("AW869X Haptic Driver");
MODULE_LICENSE("GPL v2");
