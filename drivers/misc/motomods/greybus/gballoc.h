/*
 * Copyright (C) 2016 Motorola Mobility, Inc.
 * Copyright (c) 2008-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __GBALLOC_H
#define __GBALLOC_H

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

/**
 * gballoc() - Use either kzalloc or vzalloc to allocate memory
 *
 * Allocate a block of memory - if it is small try to allocate it
 * from kmalloc (fast!) otherwise we need to go with vmalloc (safe!)
 */
static inline void *gballoc(size_t size, gfp_t gfp_flags)
{
	if (size <= PAGE_SIZE)
		return kzalloc(size, gfp_flags);

	return vzalloc(size);
}

/**
 * gbfree() - Free memory allocated by gballoc()
 *
 * Free the memory be it in vmalloc or kmalloc space
 */
static inline void gbfree(void *ptr)
{
	if (ptr != NULL && is_vmalloc_addr(ptr))
		return vfree(ptr);

	kfree(ptr);
}

#endif /* __GBALLOC_H */
