/*
 * Copyright (C) 2016 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */
#include <linux/compat.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/v4l2-ctrls.h>
#include "v4l2_hal_internal.h"

#define V4L2_HAL_SIZE_ALIGN	4

int v4l2_hal_ext_ctrl_save_private(void *data, void **priv)
{
	int idx;
	size_t size = 0;
	struct v4l2_ext_controls *ctrls = data;
	void *kmem;
	char *ptr;

	*priv = NULL;

	for (idx = 0; idx < ctrls->count; idx++) {
		size += ctrls->controls[idx].size;
	}

	if (!size)
		return 0;

	kmem = kmalloc(size, GFP_KERNEL);
	if (!kmem)
		return -ENOMEM;

	ptr = kmem;
	for (idx = 0; idx < ctrls->count; idx++) {
		if (ctrls->controls[idx].size) {
			if (!access_ok(VERIFY_READ,
				       ctrls->controls[idx].ptr,
				       ctrls->controls[idx].size) ||
			    copy_from_user(ptr,
					   ctrls->controls[idx].ptr,
					   ctrls->controls[idx].size)) {
				kfree(kmem);
				return -EFAULT;
			}
			ptr += ctrls->controls[idx].size;
		}
	}
	*priv = kmem;

	return 0;
}

int v4l2_hal_ext_ctrl_restore_private(void *data, void *priv)
{
	int idx;
	struct v4l2_ext_controls *ctrls = data;
	char *ptr;

	ptr = priv;
	for (idx = 0; idx < ctrls->count; idx++) {
		if (ctrls->controls[idx].size) {
			if (!access_ok(VERIFY_WRITE,
				       ctrls->controls[idx].ptr,
				       ctrls->controls[idx].size) ||
			    copy_to_user(ctrls->controls[idx].ptr,
					 ptr,
					 ctrls->controls[idx].size))
				return -EFAULT;

			ptr += ctrls->controls[idx].size;
		}
	}

	return 0;
}

#ifdef CONFIG_COMPAT
/* compat APIs */
struct v4l2_ext_control32 {
	__u32 id;
	__u32 size;
	__u32 reserved2[1];
	union {
		__s32 value;
		__s64 value64;
		compat_caddr_t ptr;
	};
} __attribute__ ((packed));

size_t v4l2_hal_get_required_size32(struct v4l2_ext_controls *kp)
{
	int idx;
	size_t size;

	size = sizeof(struct v4l2_ext_controls32);

	for (idx = 0; idx < kp->count; idx++) {
		size += sizeof(struct v4l2_ext_control32);

		/* pointer type should have non-zero size */
		if (kp->controls[idx].size) {
			size_t aligned = kp->controls[idx].size;
			aligned = (aligned + V4L2_HAL_SIZE_ALIGN) &
				~(V4L2_HAL_SIZE_ALIGN - 1);
			size += aligned;
		}
	}

	return size;
}

static int v4l2_hal_put_ext_control32(struct v4l2_ext_control *kp,
				      struct v4l2_ext_control32 __user *up) {
	if (!access_ok(VERIFY_WRITE, up, sizeof(struct v4l2_ext_control32)) ||
	    put_user(kp->id, &up->id) ||
	    put_user(kp->size, &up->size) ||
	    copy_to_user(up->reserved2, kp->reserved2, sizeof(up->reserved2)) ||
	    put_user(kp->value64, &up->value64))
		return -EFAULT;

	return 0;
}

int v4l2_hal_put_ext_controls32(struct v4l2_ext_controls *kp,
				void __user *uctrls, void *priv)
{
	struct v4l2_ext_controls32 __user *up = uctrls;
	struct v4l2_ext_control32 __user *ucontrols;
	struct v4l2_ext_control *kcontrols = kp->controls;
	char __user *ucontrol_data;
	int n = kp->count;
	char *ptr = priv;

	if (!access_ok(VERIFY_WRITE, up, sizeof(struct v4l2_ext_controls32)) ||
	    put_user(kp->ctrl_class, &up->ctrl_class) ||
	    put_user(kp->count, &up->count) ||
	    put_user(kp->error_idx, &up->error_idx) ||
	    copy_to_user(up->reserved, kp->reserved, sizeof(up->reserved)))
		return -EFAULT;

	if (!kp->count)
		return 0;

	ucontrols = (struct v4l2_ext_control32 __user *)&up[1];

	if (!access_ok(VERIFY_WRITE, ucontrols,
		       n * sizeof(struct v4l2_ext_control32)))
		return -EFAULT;

	if (put_user(ptr_to_compat(ucontrols), &up->controls))
		return -EFAULT;

	ucontrol_data = (char *)ucontrols +
		n * sizeof(struct v4l2_ext_control32);

	while (--n >= 0) {
		 if (v4l2_hal_put_ext_control32(kcontrols, ucontrols))
			return -EFAULT;

		if (kcontrols->size) {
			size_t aligned = kcontrols->size;

			aligned = (aligned + V4L2_HAL_SIZE_ALIGN) &
				~(V4L2_HAL_SIZE_ALIGN - 1);

			if (!ptr)
				return -EFAULT;

			if (!access_ok(VERIFY_WRITE, ucontrol_data,
				       kcontrols->size))
				return -EFAULT;

			if (copy_to_user(ucontrol_data, ptr,
					 kcontrols->size))
				return -EFAULT;

			if (put_user(ptr_to_compat(ucontrol_data),
				     &ucontrols->ptr))
				return -EFAULT;

			ucontrol_data += aligned;
			ptr += kcontrols->size;
		}

		ucontrols++;
		kcontrols++;

	}

	return 0;
}

static int v4l2_hal_get_ext_control32(struct v4l2_ext_control *kp,
				      struct v4l2_ext_control32 __user *up) {
	if (!access_ok(VERIFY_READ, up, sizeof(struct v4l2_ext_control32)) ||
	    get_user(kp->id, &up->id) ||
	    get_user(kp->size, &up->size) ||
	    copy_from_user(kp->reserved2, up->reserved2, sizeof(kp->reserved2)))
		return -EFAULT;

	if (!kp->size && get_user(kp->value64, &up->value64))
		return -EFAULT;

	return 0;
}

int v4l2_hal_get_ext_controls32(struct v4l2_ext_controls *kp,
				void __user *uctrls, void *priv)
{
	struct v4l2_ext_controls32 __user *up = uctrls;
	struct v4l2_ext_control32 __user *ucontrols;
	struct v4l2_ext_control __user *kcontrols;
	int n;
	compat_caddr_t p;
	char *ptr = priv;

	if (!access_ok(VERIFY_READ, up, sizeof(struct v4l2_ext_controls32)) ||
	    get_user(kp->ctrl_class, &up->ctrl_class) ||
	    get_user(kp->count, &up->count) ||
	    get_user(kp->error_idx, &up->error_idx) ||
	    copy_from_user(kp->reserved, up->reserved, sizeof(kp->reserved)))
		return -EFAULT;

	n = kp->count;
	if (n == 0) {
		kp->controls = NULL;
		return 0;
	}
	if (get_user(p, &up->controls))
		return -EFAULT;
	ucontrols = compat_ptr(p);
	if (!access_ok(VERIFY_READ, ucontrols,
		       n * sizeof(struct v4l2_ext_control32)))
		return -EFAULT;

	kcontrols = kp->controls;
	while (--n >= 0) {
		if (v4l2_hal_get_ext_control32(kcontrols, ucontrols))
			return -EFAULT;

		if (kcontrols->size) {
			void __user *s;

			if (!ptr)
				return -EFAULT;

			if (get_user(p, &ucontrols->ptr))
				return -EFAULT;

			s = compat_ptr(p);
			if (copy_from_user(ptr, s, kcontrols->size))
				return -EFAULT;

			ptr += kcontrols->size;
		}
		ucontrols++;
		kcontrols++;
	}

	return 0;
}

#else /* CONFIG_COMPAT */
size_t v4l2_hal_get_required_size32(struct v4l2_ext_controls *kp)
{
	 return v4l2_hal_get_required_size(kp);
}

int v4l2_hal_put_ext_controls32(struct v4l2_ext_controls *kp,
				void __user *uctrls, void *priv)
{
	return v4l2_hal_put_ext_controls(kp, uctrls, priv);
}

int v4l2_hal_get_ext_controls32(struct v4l2_ext_controls *kp,
			      void __user *uctrls, void *priv)
{
	return v4l2_hal_get_ext_controls(kp, uctrls, priv);
}
#endif

/* non-compat APIs */
size_t v4l2_hal_get_required_size(struct v4l2_ext_controls *kp)
{
	int idx;
	size_t size;

	size = sizeof(struct v4l2_ext_controls);

	for (idx = 0; idx < kp->count; idx++) {
		size += sizeof(struct v4l2_ext_control);

		/* pointer type should have non-zero size */
		if (kp->controls[idx].size) {
			size_t aligned = kp->controls[idx].size;
			aligned = (aligned + V4L2_HAL_SIZE_ALIGN) &
				~(V4L2_HAL_SIZE_ALIGN - 1);
			size += aligned;
		}
	}

	return size;
}

int v4l2_hal_put_ext_controls(struct v4l2_ext_controls *kp,
				void __user *uctrls, void *priv)
{
	struct v4l2_ext_controls __user *up = uctrls;
	struct v4l2_ext_control __user *ucontrols;
	struct v4l2_ext_control *kcontrols = kp->controls;
	char __user *ucontrol_data;
	int n = kp->count;
	char *ptr = priv;

	if (!access_ok(VERIFY_WRITE, up, sizeof(struct v4l2_ext_controls)) ||
	    copy_to_user(up, kp, sizeof(struct v4l2_ext_controls)))
		return -EFAULT;

	if (!kp->count)
		return 0;

	ucontrols = (struct v4l2_ext_control __user *)&up[1];

	if (!access_ok(VERIFY_WRITE, ucontrols,
		       n * sizeof(struct v4l2_ext_control)))
		return -EFAULT;

	if (put_user(ucontrols, &up->controls))
		return -EFAULT;

	ucontrol_data = (char *)ucontrols +
		n * sizeof(struct v4l2_ext_control);

	while (--n >= 0) {
		if (!access_ok(VERIFY_WRITE, ucontrols,
			       sizeof(struct v4l2_ext_control)) ||
		    copy_to_user(ucontrols, kcontrols,
				 sizeof(struct v4l2_ext_controls)))
			return -EFAULT;

		if (kcontrols->size) {
			size_t aligned = kcontrols->size;

			aligned = (aligned + V4L2_HAL_SIZE_ALIGN) &
				~(V4L2_HAL_SIZE_ALIGN - 1);

			if (!ptr)
				return -EFAULT;

			if (!access_ok(VERIFY_WRITE, ucontrol_data,
				       kcontrols->size))
				return -EFAULT;

			if (copy_to_user(ucontrol_data, ptr,
					 kcontrols->size))
				return -EFAULT;

			if (put_user(ucontrol_data, &ucontrols->ptr))
				return -EFAULT;

			ucontrol_data += aligned;
			ptr += kcontrols->size;
		}

		ucontrols++;
		kcontrols++;

	}

	return 0;
}

static int v4l2_hal_get_ext_control(struct v4l2_ext_control *kp,
				    struct v4l2_ext_control __user *up) {
	if (!access_ok(VERIFY_READ, up, sizeof(struct v4l2_ext_control)) ||
	    get_user(kp->id, &up->id) ||
	    get_user(kp->size, &up->size) ||
	    copy_from_user(kp->reserved2, up->reserved2, sizeof(kp->reserved2)))
		return -EFAULT;

	if (!kp->size && get_user(kp->value64, &up->value64))
		return -EFAULT;

	return 0;
}

int v4l2_hal_get_ext_controls(struct v4l2_ext_controls *kp,
			      void __user *uctrls, void *priv)
{
	struct v4l2_ext_controls __user *up = uctrls;
	struct v4l2_ext_control __user *ucontrols;
	struct v4l2_ext_control __user *kcontrols;
	int n;
	char *ptr = priv;

	if (!access_ok(VERIFY_READ, up, sizeof(struct v4l2_ext_controls)) ||
	    get_user(kp->ctrl_class, &up->ctrl_class) ||
	    get_user(kp->count, &up->count) ||
	    get_user(kp->error_idx, &up->error_idx) ||
	    copy_from_user(kp->reserved, up->reserved, sizeof(kp->reserved)))
		return -EFAULT;

	n = kp->count;
	if (n == 0) {
		kp->controls = NULL;
		return 0;
	}

	if (get_user(ucontrols, &up->controls))
		return -EFAULT;

	if (!access_ok(VERIFY_READ, ucontrols,
		       n * sizeof(struct v4l2_ext_control)))
		return -EFAULT;

	kcontrols = kp->controls;
	while (--n >= 0) {
		if (v4l2_hal_get_ext_control(kcontrols, ucontrols))
			return -EFAULT;

		if (kcontrols->size) {
			if (!ptr)
				return -EFAULT;

			if (copy_from_user(ptr, ucontrols->ptr,
					   kcontrols->size))
				return -EFAULT;

			ptr += kcontrols->size;
		}
		ucontrols++;
		kcontrols++;
	}

	return 0;
}
