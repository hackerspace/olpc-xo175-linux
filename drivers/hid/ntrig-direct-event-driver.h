/*
 * N-Trig direct event character driver header
 * drivers/hid/ntrig-direct-event-driver.h
 *
 * Copyright (C) 2011, N-Trig.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef _NTRIG_DIRECT_EVENT_DRIVER_H
#define _NTRIG_DIRECT_EVENT_DRIVER_H

/** 
 * External Interface API
 */
int bus_init_direct_events(void);
void bus_exit_direct_events(void);

#endif /* _NTRIG_DIRECT_EVENT_DRIVER_H */
