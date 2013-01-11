#ifndef __ENEEC_H
#define __ENEEC_H

#define ENEEC_CAP_KEYBOARD	(1 << 0)
#define ENEEC_CAP_PS2MOUSE	(1 << 1)

struct eneec_platform_data {
	unsigned int capabilities;
};

#endif /* __ENEEC_H */
