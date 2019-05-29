#!/bin/sh
# This script is for enable ftrace config in already defined defconfig
SOURCE=

x=`echo $0 | grep "^/"`
cur=`pwd`
if test "${x}"; then
	SOURCE=`dirname $0`
else
	SOURCE=`dirname $cur/$0`
fi


$SOURCE/config -d ENABLE_DEFAULT_TRACERS
$SOURCE/config -e BINARY_PRINTF
$SOURCE/config -e FUNCTION_TRACER
$SOURCE/config -e IRQSOFF_TRACER
$SOURCE/config -e SCHED_TRACER
$SOURCE/config -e CONTEXT_SWITCH_TRACER
$SOURCE/config -e DYNAMIC_FTRACE
$SOURCE/config -e EVENT_POWER_TRACING_DEPRECATED
$SOURCE/config -e EVENT_TRACING
$SOURCE/config -e FRAME_POINTER
$SOURCE/config -e FTRACE_MCOUNT_RECORD
$SOURCE/config -d FTRACE_STARTUP_TEST
$SOURCE/config -e FUNCTION_GRAPH_TRACER
$SOURCE/config -e FUNCTION_PROFILER
$SOURCE/config -e GENERIC_TRACER
$SOURCE/config -d NET_DROP_MONITOR
$SOURCE/config -e NOP_TRACER
$SOURCE/config -e OLD_MCOUNT
$SOURCE/config -e RING_BUFFER
$SOURCE/config -e RING_BUFFER_ALLOW_SWAP
$SOURCE/config -d RING_BUFFER_BENCHMARK
$SOURCE/config -e TRACEPOINTS
$SOURCE/config -e TRACER_MAX_TRACE
$SOURCE/config -e TRACE_IRQFLAGS
$SOURCE/config -e TRACING

make oldconfig
