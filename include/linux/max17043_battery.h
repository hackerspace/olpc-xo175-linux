#define MAX17043_VCELL		0x02
#define MAX17043_SOC		0x04
#define MAX17043_MODE		0x06
#define MAX17043_VERSION	0x08
#define MAX17043_CONFIG		0x0C
#define MAX17043_COMMAND	0xFE
#define MAX17043_ALERT		(1 << 5)
#define MAX17043_sleep		(1 << 7)

struct max17043_battery_pdata {
	bool gpio_en;
	int gpio;
	int interval;
};
