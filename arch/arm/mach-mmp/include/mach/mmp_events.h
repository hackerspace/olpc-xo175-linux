#undef TRACE_SYSTEM
#define TRACE_SYSTEM mmp_events

#if !defined(_TRACE_MMP_EVENTS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MMP_EVENTS_H

#include <linux/tracepoint.h>

TRACE_EVENT(dfc,

	TP_PROTO(union trace_dfc_log *tgt),

	TP_ARGS(tgt),

	TP_STRUCT__entry(
		__field(u32, dat)
	),

	TP_fast_assign(
		__entry->dat = tgt->val;
	),

	TP_printk("DFC 0x%08x", __entry->dat)
);


TRACE_EVENT(idle_entry,

	TP_PROTO(int mode),

	TP_ARGS(mode),

	TP_STRUCT__entry(
		__field(int, mode)
	),

	TP_fast_assign(
		__entry->mode	= mode;
	),

	TP_printk("enter idle, type=%d", __entry->mode)
);

TRACE_EVENT(idle_exit,

	TP_PROTO(u32 reason),

	TP_ARGS(reason),

	TP_STRUCT__entry(
		__field(u32, reason)
	),

	TP_fast_assign(
		__entry->reason = reason;
	),

	TP_printk("exit idle, reason=0x%x", __entry->reason)
);

#endif /*  _TRACE_MMP_EVENTS_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH mach

#define TRACE_INCLUDE_FILE mmp_events
/* This part must be outside protection */
#include <trace/define_trace.h>
