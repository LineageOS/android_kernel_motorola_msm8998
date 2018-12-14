/*
 * Copyright (C) 2015 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM mods

#if !defined(_TRACE_MODS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MODS_H

#include <linux/tracepoint.h>

struct gb_operation_msg_hdr;

TRACE_EVENT(mods_switch,

	TP_PROTO(struct gb_operation_msg_hdr *hdr, u8 from_intf,
		u16 from_cport, u8 to_intf, u16 to_cport),

	TP_ARGS(hdr, from_intf, from_cport, to_intf, to_cport),

	TP_STRUCT__entry(
		__field(unsigned int, op_id)
		__field(unsigned int, op_type)
		__field(unsigned int, op_result)
		__field(unsigned int, from_intf)
		__field(unsigned int, from_cport)
		__field(unsigned int, to_intf)
		__field(unsigned int, to_cport)
	),

	TP_fast_assign(
		__entry->op_id = hdr->operation_id;
		__entry->op_type = hdr->type;
		__entry->op_result = hdr->result;
		__entry->from_intf = from_intf;
		__entry->from_cport = from_cport;
		__entry->to_intf = to_intf;
		__entry->to_cport = to_cport;
	),

	TP_printk(
		"%04x:%04x -> %04x:%04x op=%04x type=%02x res=%04x",
		__entry->from_intf, __entry->from_cport,
		__entry->to_intf, __entry->to_cport,
		__entry->op_id, __entry->op_type, __entry->op_result
	)
);
#endif /* _TRACE_MODS_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE mods_trace
#include <trace/define_trace.h>
