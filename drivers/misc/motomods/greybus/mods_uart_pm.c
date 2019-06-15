/*
 * Copyright (C) 2015 Motorola Mobility LLC
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

#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#include "apba.h"
#include "mods_uart.h"
#include "mods_uart_pm.h"
#include "mhb_protocol.h"

struct mods_uart_pm_data {
	void *mods_uart_data;
	bool on;
	bool pm_state_local;
	atomic_t pm_state_remote;
	struct mutex pm_handshake_mutex;
	struct completion pm_handshake_comp;
	struct work_struct idle_timer_work;
	struct timer_list idle_timer;
};

#define MODS_UART_PM_HANDSHAKE_TIMEOUT	1000 /* ms */
#define MODS_UART_PM_IDLE_TIMEOUT	2000 /* msec */

void mods_uart_pm_update_idle_timer(void *uart_pm_data)
{
	struct mods_uart_pm_data *data =
		(struct mods_uart_pm_data *)uart_pm_data;

	if (data && data->on)
		mod_timer(&data->idle_timer,
			  jiffies +
			    msecs_to_jiffies(MODS_UART_PM_IDLE_TIMEOUT));
}

static void idle_timer_callback(unsigned long timer_data)
{
	struct mods_uart_pm_data *data;

	data = (struct mods_uart_pm_data *)timer_data;

	pr_debug("%s: idle timeout\n", __func__);

	if (!data->on)
		return;

	/* Need to switch out from atomic context */
	schedule_work(&data->idle_timer_work);
}

static void consider_remote_awake(struct mods_uart_pm_data *data)
{
	atomic_set(&data->pm_state_remote, 1);
	apba_wake_assert(false);
	if (!completion_done(&data->pm_handshake_comp))
		complete(&data->pm_handshake_comp);
}

/*
 * Called from Wake ISR handler thread. Send out an ack with special flag
 * so that PM status is updated in context of UART TX.
 */
void mods_uart_pm_handle_wake_interrupt(void *uart_data)
{
	struct mods_uart_pm_data *data;

	pr_debug("%s: wake interrupt\n", __func__);

	data = (struct mods_uart_pm_data *)mods_uart_get_pm_data(uart_data);
	if (data && data->on) {
		/* Getting an interrupt from APBA is a sign that it's alive.
		   This can complete a pending wake handshake completion. */
		consider_remote_awake(data);

		apba_send_pm_wake_rsp();

		/* Start idle time in case wake ack is dropped. */
		mods_uart_pm_update_idle_timer(data);
	}
}

/*
 * Function to update physical usart state.
 * Always executed while tx context is locked.
 */
static void local_pm_update(struct mods_uart_pm_data *data, bool on)
{
	/* No action if state remain unchanged. */
	if (data->pm_state_local == on)
		return;

	if (mods_uart_do_pm(data->mods_uart_data, on))
		return;

	data->pm_state_local = on;
}

/*
 * Wakes up handshake procedure.
 */
static void apba_pm_wake_handshake(struct mods_uart_pm_data *data)
{
	pr_debug("%s: wake handshake\n", __func__);
	if (data && data->on) {
		mutex_lock(&data->pm_handshake_mutex);

		/* must reinit because the interrupt can call complete */
		reinit_completion(&data->pm_handshake_comp);

		/* Wake INT then wait for an ack message. */
		apba_wake_assert(true);

		if (!wait_for_completion_timeout(
			 &data->pm_handshake_comp,
			 msecs_to_jiffies(MODS_UART_PM_HANDSHAKE_TIMEOUT))) {
			pr_err("%s: WAKE HANDSHAKE () timeout\n", __func__);
			apba_wake_assert(false);
		}

		mutex_unlock(&data->pm_handshake_mutex);
		pr_debug("%s: wake attempt done\n", __func__);
	}
}

/*
 * Called in context of UART TX, before TX chunks is sent out.
 * All UART TX request has been blocked in caller function while
 * executing this function.
 */
void mods_uart_pm_pre_tx(void *uart_pm_data, int flag)
{
	struct mods_uart_pm_data *data;

	data = (struct mods_uart_pm_data *)uart_pm_data;

	if (data && data->on) {
		/*
		 * Responding to sleep ind from APBA. Consder APBA went
		 * into sleep so next TX will kick off wake handshake.
		 */
		if (flag == UART_PM_FLAG_SLEEP_ACK) {
			atomic_set(&data->pm_state_remote, 0);

			/*
			 * Do not continue. Otherwise wake handshake will be
			 * done to send this SLEEP ACK.
			 */
			return;
		}

		/*
		 * Responding to wake interrupt from APBA. Consider APBA
		 * awake so we don't start wake up handshake.
		 */
		if (flag == UART_PM_FLAG_WAKE_ACK)
			atomic_set(&data->pm_state_remote, 1);

		/* Make sure we are on. Try fail through. */
		local_pm_update(data, true);

		/* If remote is up, no further action. */
		if (atomic_read(&data->pm_state_remote))
			return;

		/* If APBA is in sleep, wake it up first. */
		apba_pm_wake_handshake(data);
	}
}

/*
 * Called in context of UART TX, after TX chunks is sent out.
 * All UART TX request has been blocked in caller function while
 * executing this function.
 */
void mods_uart_pm_post_tx(void *uart_pm_data, int flag)
{
	struct mods_uart_pm_data *data;

	data = (struct mods_uart_pm_data *)uart_pm_data;
	if (data && data->on) {
		if (flag == UART_PM_FLAG_SLEEP_IND)
			atomic_set(&data->pm_state_remote, 0);

		mods_uart_pm_update_idle_timer(data);
	}
}

static void idle_timeout_work_func(struct work_struct *work)
{
	struct mods_uart_pm_data *data;

	data = container_of(work, struct mods_uart_pm_data, idle_timer_work);
	if (!data->on)
		return;

	pr_debug("%s: idle timeout\n", __func__);
	/*
	 * No need to tell peer that we are going to sleep if they are
	 * already in sleep.
	 */
	if (!atomic_read(&data->pm_state_remote)) {
		/* Change UART PM state while TX context is blocked */
		mods_uart_lock_tx(data->mods_uart_data, true);
		local_pm_update(data, false);
		mods_uart_lock_tx(data->mods_uart_data, false);
		return;
	}

	if (apba_send_pm_sleep_req())
		mods_uart_pm_update_idle_timer(data);
}

void mods_uart_pm_handle_pm_wake_rsp(void *uart_data)
{
	struct mods_uart_pm_data *data;

	data = (struct mods_uart_pm_data *)mods_uart_get_pm_data(uart_data);

	pr_debug("%s: wake_ack\n", __func__);

	if (data && data->on)
		consider_remote_awake(data);
}

/* assume tx is locked.  don't do anything that would send a message */
void mods_uart_pm_on(void *uart_data)
{
	struct mods_uart_pm_data *data;

	data = (struct mods_uart_pm_data *)mods_uart_get_pm_data(uart_data);

	if (data) {
		local_pm_update(data, true);
		atomic_set(&data->pm_state_remote, 1);
		data->on = true;
	}
}

void  mods_uart_pm_off(void *uart_data)
{
	struct mods_uart_pm_data *data;
	data = (struct mods_uart_pm_data *)mods_uart_get_pm_data(uart_data);

	if (data) {
		data->on = false;

		/* In case we happened to be waiting for a handshake,
		   finish the completion early. */
		complete_all(&data->pm_handshake_comp);
	}
}

void *mods_uart_pm_initialize(void *uart_data)
{
	struct mods_uart_pm_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;

	data->mods_uart_data = uart_data;

	mutex_init(&data->pm_handshake_mutex);
	init_completion(&data->pm_handshake_comp);

	INIT_WORK(&data->idle_timer_work, idle_timeout_work_func);

	setup_timer(&data->idle_timer, idle_timer_callback,
		    (unsigned long)data);

	return data;
}

void mods_uart_pm_cancel_timer(void *uart_pm_data)
{
	struct mods_uart_pm_data *data;

	data = (struct mods_uart_pm_data *)uart_pm_data;

	if (data) {
		cancel_work_sync(&data->idle_timer_work);
		del_timer(&data->idle_timer);
	}
}

void mods_uart_pm_uninitialize(void *uart_pm_data)
{
	kfree(uart_pm_data);
}
