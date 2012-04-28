#ifndef _ECS_H_
#define _ECS_H_

/* Essential Camera Sensor: Header file */
/* Essential Camera Sensor driver (A.K.A. ECS_driver or x_driver) is a abstract
 * way to describe camera sensors. It provides an option to create a camera
 * sensor driver in an easy way: simply list all the sensor setting, register
 * them in a reasonable way, and use the helper function to implement all
 * nessarry function of any popular sensor driver interface (like v4l2-subdev),
 * and then, you get a driver */

#include <linux/types.h>
#include <media/v4l2-common.h>

#include <media/soc_camera.h>

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

struct ecs_setting {
	char	*name;
	/* As for property, each setting is refered to by a id number */
	int	id;
	/* Sensor config table */
	void	*cfg_tab;
	/* Config table size, in bytes */
	int	cfg_sz;
	/* A pointer to save the feed back information location. The information
	 * tells camera controller how to adapt to this sensor config */
	void	*info;
};

struct ecs_property {
	char	*name;
	/* As for sensor, each property is refered to by a id number */
	int	id;
	/* a hook to the list that links all properties of a sensor */
	struct ecs_setting *stn_tab;
	int	stn_num;
	/* settings for this property must in a register range of
	 * [reg_low, reg_high), if reg_low==reg_high, range check is ignored */
	int	reg_low;
	int	reg_high;
	/* cam we just ignore it, when change property to its current value? */
	int	speculate;
	int	value_now;
	int	value_type;
	/* Following handler will download register values into sensor,
	 * and return the number of downloaded values */
	int (*cfg_handler)(void *hw_ctx, void *table, int size);
	int (*value_equal)(void *a, void *b);
};

struct ecs_state_cfg {
	int	prop_id;
	int	prop_val;
};

struct ecs_state {
	char	*name;
	/* As for sensor, each state is refered to by a id number */
	int	id;
	/* a list to link all settings for this state,
	 * list member is struct ecs_state_cfg*/
	struct ecs_state_cfg *cfg_tab;
	int	cfg_num;
};

enum {
	ECS_HWIF_NONE	= 0,
	ECS_HWIF_I2C,
	ECS_HWIF_END,
};

enum {
	ECS_IF_NONE	= 0,
	ECS_IF_GENERIC,
	ECS_IF_SUBDEV,
	ECS_IF_END,
};

struct ecs_sensor {
	char	*name;
	/* a list to link all properties, list member is struct ecs_prop */
	struct ecs_property	*prop_tab;
	int	prop_num;
	/* a list to link all states, list member is struct ecs_state */
	struct ecs_state	*state_tab;
	int	state_num;
	int	state_now;
	int	speculate;

	void	*hw_ctx;
	/* Following handler will download register values into sensor,
	 * and return the number of downloaded values */
	int	(*cfg_handler)(void *hw_ctx, void *list, int size);

	/* Interface */
	int	swif_type;
	void	*swif;	/* pointer to controller programming interface*/
};

/* Data structures to support ECS interface to sensor driver */
struct ecs_if_subdev {
	int	*state_list;
	int	state_cnt;
	struct v4l2_mbus_framefmt	*state_map;
	int	*enum_map;
	int	fmt_id;
	int	res_id;
	int	str_id;
	void	(*get_fmt_code)(const struct ecs_sensor *snsr, \
				int fmt_value, struct v4l2_mbus_framefmt *mf);
	void	(*get_res_desc)(const struct ecs_sensor *snsr, \
				int res_value, struct v4l2_mbus_framefmt *mf);
};

/* recommended content of format setting info*/
struct ecs_default_fmt_info {
	enum	v4l2_mbus_pixelcode code;
	__u32	field;
	__u32	clrspc;
	int	fourcc;
};

/* recommended content of resolution setting info*/
struct ecs_default_res_info {
	int h_act;	/* output width */
	int v_act;	/* output height */
	int h_blk;	/* line blank */
	int v_blk;	/* frame blank */
};

int ecs_sensor_reset(struct ecs_sensor *snsr);
int ecs_sensor_plat(struct ecs_sensor *snsr, char *name, int speculate, \
			void *hw_ctx, int (*cfg_handler)(void *, void *, int), \
			int swif_type, void *swif);
int ecs_setting_override(struct ecs_sensor *snsr, int prop, int stn, \
				void *cfg_tab, int cfg_sz);
int ecs_property_override(struct ecs_sensor *snsr, int prop_id, \
			struct ecs_setting *stn_tab, \
			int (*cfg_handler)(void *, void *, int));
int ecs_get_info(struct ecs_sensor *snsr, int prop_id, void **feedback);

int ecs_subdev_set_stream(struct ecs_sensor *snsr, int enable);
int ecs_subdev_set_fmt(struct ecs_sensor *snsr, struct v4l2_mbus_framefmt *mf);
int ecs_subdev_get_fmt(struct ecs_sensor *snsr, struct v4l2_mbus_framefmt *mf);
int ecs_subdev_try_fmt(struct ecs_sensor *snsr, struct v4l2_mbus_framefmt *mf);
int ecs_subdev_enum_fmt(struct ecs_sensor *snsr, int idx, \
						enum v4l2_mbus_pixelcode *code);
int ecs_subdev_enum_fsize(struct ecs_sensor *snsr, \
						struct v4l2_frmsizeenum *fsize);

void ecs_subdev_default_get_fmt_code(const struct ecs_sensor *snsr, \
				int fmt_value, struct v4l2_mbus_framefmt *mf);
void ecs_subdev_default_get_res_desc(const struct ecs_sensor *snsr, \
				int res_value, struct v4l2_mbus_framefmt *mf);

int ecs_orig_set_state(struct ecs_sensor *snsr, int state);
int ecs_orig_set_list(struct ecs_sensor *snsr, \
					struct ecs_state_cfg *list, int len);

#define __DECLARE_SETTING(SENSOR, sensor, PROP, prop, VAL, val) \
	[SENSOR##_##PROP##_##VAL] = { \
		.name		= #VAL, \
		.id		= SENSOR##_##PROP##_##VAL, \
		.cfg_tab	= sensor##_##prop##_##val, \
		.cfg_sz		= ARRAY_SIZE(sensor##_##prop##_##val), \
		.info		= NULL, \
	}

#define __DECLARE_SETTING_VS_INFO(SENSOR, sensor, PROP, prop, VAL, val) \
	[SENSOR##_##PROP##_##VAL] = { \
		.name		= #VAL, \
		.id		= SENSOR##_##PROP##_##VAL, \
		.cfg_tab	= sensor##_##prop##_##val, \
		.cfg_sz		= ARRAY_SIZE(sensor##_##prop##_##val), \
		.info		= &sensor##_##prop##_##info##_table \
					[SENSOR##_##PROP##_##VAL], \
	}

#define __DECLARE_STATE(SENSOR, sensor, VAL, val) \
	[SENSOR##_##ST##_##VAL] = { \
		.name		= #VAL, \
		.id		= SENSOR##_ST_##VAL, \
		.cfg_tab	= sensor##_state_##val, \
		.cfg_num	= ARRAY_SIZE(sensor##_state_##val) ,\
	}

#define ECS_DEFAULT_SNSR_CFG_HANDLER	(&ecs_reg_array_dump)
#define ECS_DEFAULT_CTLR_CFG_HANDLER	NULL

#endif
