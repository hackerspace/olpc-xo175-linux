/*
 * Essential Camera Sensor Driver
 *
 * Copyright (c) 2012 Marvell Ltd.
 * Jiaquan Su <jqsu@marvell.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* basic sensor */
/* echo sensor */
/* plat sensor */
/* HW audit sensor */
/* SW audit sensor */

#include <linux/slab.h>
#include "ecs.h"

/* trace_level: */
/* 0 - only actual errors */
/* 1 - ecs internal error included */
/* 2 - all ecs log */
/* 3 - all ecs log and sensor behavior, performance impactable */
static int trace_level;
module_param(trace_level, int, 0644);
#define x_printk(level, printk_level, fmt, arg...) \
	do { \
		if (trace_level >= level) \
			printk(printk_level fmt, ##arg); \
	} while (0)
#define x_inf(level, fmt, arg...) \
	x_printk(level, KERN_NOTICE, "x: " fmt "\n", ##arg)
#define x_log(level, fmt, arg...) \
	x_printk(level, KERN_DEBUG, "x: " fmt "\n", ##arg)
#define xinf(level, fmt, arg...) x_printk(level, KERN_NOTICE, fmt, ##arg)
#define xlog(level, fmt, arg...) x_printk(level, KERN_DEBUG, fmt, ##arg)

#define ECS_DEFAULT_STN_NAME	"setting_name_unknown"
#define ECS_DEFAULT_PROP_NAME	"property name unknown"
#define ECS_DEFAULT_STATE_NAME	"state name unknown"
#define ECS_DEFAULT_SENSOR_NAME	"X-style sensor"
#define ECS_DEFAULT_SENSOR_BEHAVIOR	soc_subdev_ops

static void ecs_dump_subdev_map(struct ecs_sensor *snsr);
static int ecs_reg_array_dump(void *useless, void *table, int len);
static void ecs_sensor_dump(struct ecs_sensor *snsr);

inline int ecs_register_snsr_handler(struct ecs_sensor *snsr, \
					int (*handler)(void *, int))
{
	return 0;
}

inline int ecs_register_ctlr_handler(struct ecs_sensor *snsr, \
					int (*handler)(void *, int))
{
	return 0;
}

/* ecs core functions */
/* ecs_find_xxx */
static inline struct ecs_property *ecs_find_property(struct ecs_sensor *snsr, \
								int prop_id)
{
	struct ecs_property *prop = NULL;

	if (unlikely((prop_id < 0) || (prop_id >= snsr->prop_num))) {
		x_log(2, "property %d not found", prop_id);
		return NULL;
	}
	prop = snsr->prop_tab + prop_id;
	if (unlikely(prop->id == prop_id))
		return prop;
	x_log(3, "property '%s' is the %dth property, but it's id is %d, " \
		"suggest move it to the %dth in property list to avoid bug", \
		prop->name, prop_id, prop->id, prop->id);
	return NULL;
}

static inline struct ecs_state *ecs_find_state(struct ecs_sensor *snsr, \
								int state_id)
{
	struct ecs_state *state = NULL;

	if (unlikely((state_id < 0) || (state_id >= snsr->state_num))) {
		x_log(2, "state %d not found", state_id);
		return NULL;
	}
	state = snsr->state_tab + state_id;
	if (unlikely(state_id == state->id))
		return state;
	x_log(3, "setting '%s' is the %dth state, but it's id is %d, " \
		"suggest move it to the %dth in state list to avoid bug", \
		state->name, state_id, state->id, state->id);
	return NULL;
}

static inline struct ecs_setting *ecs_find_setting(struct ecs_property *prop, \
								int value)
{
	struct ecs_setting *stn = NULL;

	if (unlikely(prop == NULL))
		return NULL;
	if (unlikely((value < 0) || (value >= prop->stn_num))) {
		x_log(2, "setting %d not found", value);
		return NULL;
	}

	stn = prop->stn_tab + value;
	if (value == stn->id)
		return stn;
	x_log(3, "setting '%s' is the %dth setting, but it's id is %d, " \
		"suggest move it to the %dth in setting list to avoid bug", \
		stn->name, value, stn->id, stn->id);
	return NULL;
}

/* get all */
inline int ecs_get_state(struct ecs_sensor *snsr)
{
	return snsr->state_now;
}

inline int ecs_get_value(struct ecs_property *prop)
{
	return prop->value_now;
}

/* set all */
inline int ecs_set_value(struct ecs_sensor *snsr, \
			const struct ecs_state_cfg *cfg)
{
	struct ecs_property *prop = NULL;
	struct ecs_setting *stn = NULL;
	int ret;

	prop = ecs_find_property(snsr, cfg->prop_id);
	if (prop == NULL)
		return -EINVAL;
	/* interpret property value according to its type */
	if (prop->value_type == 0) {
		stn = ecs_find_setting(prop, cfg->prop_val);
		if (stn == NULL)
			return -EINVAL;
		if (prop->speculate && (prop->value_now == cfg->prop_val)) {
			x_inf(2, "%10s == %s", prop->name, stn->name);
			return 0;
		}

		x_inf(2, "%10s := %s", prop->name, stn->name);
		ret = (*prop->cfg_handler)(snsr->hw_ctx, \
						stn->cfg_tab, stn->cfg_sz);
		if (ret != stn->cfg_sz) {
			x_inf(0, "%s: error applying setting %s = %s ", \
				snsr->name, prop->name, stn->name);
			return -EIO;
		}
	} else {
		/* the value is not integral, sensor driver knows what to do*/
		if ((prop->speculate) && (prop->value_equal) && \
			(*prop->value_equal)((void *)prop->value_now, \
						(void *)cfg->prop_val)) {
			x_inf(0, "%10s needs no change\n", prop->name);
			return 0;
		}
		/* Here ECS assumes as soon as sensor driver mark a property
		 * value as none-integral, there is no way to convert property
		 * value to integral, even if there is someway, ECS don't want
		 * to envolve into this complexity, but cfg_handler shall */
		x_inf(2, "%10s will be changed\n", prop->name);
		ret = (*prop->cfg_handler)(snsr->hw_ctx, \
				prop->stn_tab[0].cfg_tab, cfg->prop_val);
		if (ret < 0) {
			x_inf(0, "error changing property '%s'", prop->name);
			return -EIO;
		}
	}

	prop->value_now = cfg->prop_val;
	return ret;
}

static inline int ecs_set_state(struct ecs_sensor *snsr, int state_id)
{
	struct ecs_state_cfg *cfg;
	struct ecs_state *state;
	int i, ret = 0;

	state = snsr->state_tab + state_id;
	for (i = 0; i < state->cfg_num; i++) {
		int cnt;
		cfg = &(state->cfg_tab[i]);
		cnt = ecs_set_value(snsr, cfg);
		if (cnt < 0) {
			x_inf(0, "change to state '%s' failed, when changing " \
				"property %d", state->name, cfg->prop_id);
			return -EIO;
		}
		ret += cnt;
	}

	snsr->state_now = state->id;
	return ret;
}

int ecs_get_info(struct ecs_sensor *snsr, int prop_id, void **feedback)
{
	struct ecs_property *prop = NULL;
	struct ecs_setting *stn = NULL;

	if (unlikely(feedback == NULL))
		return -EINVAL;
	prop = ecs_find_property(snsr, prop_id);
	if (unlikely(prop == NULL))
		return -EINVAL;
	stn = ecs_find_setting(prop, prop->value_now);
	if (unlikely(stn == NULL))
		return -EINVAL;

	*feedback = stn->info;
	return 0;
}
EXPORT_SYMBOL(ecs_get_info);

int ecs_setting_override(struct ecs_sensor *snsr, int prop_id, int stn_id, \
				void *cfg_tab, int cfg_sz)
{
	struct ecs_property *_prop = ecs_find_property(snsr, prop_id);
	struct ecs_setting *_stn = ecs_find_setting(_prop, stn_id);

	if (unlikely(_prop == NULL || _stn == NULL))
		return -EINVAL;

	_stn->cfg_tab = cfg_tab;
	_stn->cfg_sz = cfg_sz;
	return 0;
}
EXPORT_SYMBOL(ecs_setting_override);

int ecs_property_override(struct ecs_sensor *snsr, int prop_id, \
			struct ecs_setting *stn_tab, \
			int (*cfg_handler)(void *, void *, int))
{
	struct ecs_property *prop = ecs_find_property(snsr, prop_id);

	if (unlikely(prop == NULL))
		return -EINVAL;

	if (stn_tab != NULL)
		prop->stn_tab = stn_tab;
	if (cfg_handler != NULL)
		prop->cfg_handler = cfg_handler;
	return 0;
}
EXPORT_SYMBOL(ecs_property_override);

int ecs_sensor_reset(struct ecs_sensor *snsr)
{
	struct ecs_property *prop;
	int i;

	if (snsr == NULL)
		return -EINVAL;
	for (i = 0; i < snsr->prop_num; i++) {
		prop = ecs_find_property(snsr, i);
		prop->value_now = UNSET;
	}
	snsr->state_now = UNSET;
	return 0;
}
EXPORT_SYMBOL(ecs_sensor_reset);

static int ecs_subdev_init(struct ecs_sensor *snsr);

int ecs_sensor_plat(struct ecs_sensor *snsr, char *name, int speculate, \
			void *hw_ctx, int (*cfg_handler)(void *, void *, int), \
						int swif_type, void *swif)
{
	int i, j, ret;

	if (unlikely(snsr == NULL))
		return -EINVAL;

	snsr->state_now = UNSET;
	if (name == NULL)
		snsr->name = ECS_DEFAULT_SENSOR_NAME;
	else
		snsr->name = name;

	snsr->hw_ctx = hw_ctx;
	snsr->cfg_handler = cfg_handler;
	snsr->speculate = speculate;

	/* by this time, sensor driver should have line up all the settings and
	 * propertys. ECS driver will check if all of them are well organized */
	for (i = 0; i < snsr->prop_num; i++) {
		struct ecs_property *prop = ecs_find_property(snsr, i);
		if (unlikely(prop == NULL)) {
			x_inf(0, "error checking property %d", i);
			return -EINVAL;
		}

		/* copy handlers by the way. If no specific handler defined for
		 * this property, use global default handler */
		if (prop->cfg_handler == NULL)
			prop->cfg_handler = cfg_handler;

		for (j = 0; j < prop->stn_num; j++) {
			struct ecs_setting *stn = ecs_find_setting(prop, j);
			if (unlikely(stn == NULL)) {
				x_inf(0, "error checking '%s' setting %d", \
								prop->name, j);
				return -EINVAL;
			}
		}
		prop->speculate = prop->speculate && speculate;
	}

	for (i = 1; i < snsr->state_num; i++) {
		struct ecs_state *state = ecs_find_state(snsr, i);
		if (unlikely(state == NULL)) {
			x_inf(0, "error checking state %d", i);
			return -EINVAL;
		}
	}
	/* from this point on, ecs can directly access property/setting/state
	 * rather than calling ecs_find_property/setting/state */

	/* now deal with interfaces */
	snsr->swif_type = swif_type;
	snsr->swif = swif;
	ret = 0;
	switch (swif_type) {
	case ECS_IF_SUBDEV:
		ret = ecs_subdev_init(snsr);
		break;
	}
	return 0;
}
EXPORT_SYMBOL(ecs_sensor_plat);

/* Interfacing funtions */
/* subdev interface */
static int ecs_subdev_init(struct ecs_sensor *snsr)
{
	struct ecs_property *prop_fmt = NULL, *prop_res = NULL;
	struct ecs_if_subdev *subdev;
	int *enum_map = NULL;
	struct v4l2_mbus_framefmt *state_map = NULL;
	int tab_sz, line_sz, i, j, idx_st = 0, idx_ft = 0;

	subdev = snsr->swif;
	if (unlikely(subdev == NULL))
		return -EINVAL;

	if (unlikely((subdev->get_fmt_code == NULL)
		|| (subdev->get_res_desc == NULL)))
		return -EINVAL;

	prop_fmt = snsr->prop_tab + subdev->fmt_id;
	prop_res = snsr->prop_tab + subdev->res_id;

	/* Setup format-resolution to state mapping table */
	line_sz = prop_res->stn_num;
	tab_sz = prop_fmt->stn_num * line_sz;
	state_map = subdev->state_map;
	enum_map = subdev->enum_map;

	{
	int temp[tab_sz];

	if ((state_map == NULL) || (enum_map == NULL))
		return -ENOMEM;
	memset(temp, 0, sizeof(int) * tab_sz);

	/* Establish state map */
	for (i = 0; i < subdev->state_cnt; i++) {
		struct ecs_state *cstate;
		int fmt_val = UNSET, res_val = UNSET;
		/* for state_id_list[i], find "RES" and "FMT" property, and fill
		 * into enum table */
		cstate = snsr->state_tab + subdev->state_list[i];
		if (cstate == NULL) {
			printk(KERN_ERR "x: state id %d not found\n", \
				subdev->state_list[i]);
			return -EINVAL;
		}
		for (j = 0; j < cstate->cfg_num; j++) {
			if (cstate->cfg_tab[j].prop_id == subdev->fmt_id)
				fmt_val = cstate->cfg_tab[j].prop_val;
			if (cstate->cfg_tab[j].prop_id == subdev->res_id)
				res_val = cstate->cfg_tab[j].prop_val;
		}
		if ((fmt_val == UNSET) || (res_val == UNSET))
			return -EPERM;
		if (temp[fmt_val*line_sz + res_val] != 0) {
			printk(KERN_ERR "x: duplicate state: %d and %d\n", \
				temp[fmt_val*line_sz + res_val], \
				subdev->state_list[i]);
			return -EPERM;
		}
		temp[fmt_val*line_sz + res_val] = subdev->state_list[i];
		/* Mark this format 'active' in format table */
		enum_map[fmt_val] = 1;
	};

	/* Initialize the mf table, zero one more slot to establish end sign */
	memset(state_map, 0, sizeof(struct v4l2_mbus_framefmt)*snsr->state_num);
	/* Initialize format enum map */
	memset(enum_map, 0, sizeof(int) * prop_fmt->stn_num * 2);
	/* Establish state map triplets(code, width, height) in state_map */
	/* Establish enumerate map pairs(code, index_of_start) in enum_map */
	idx_st = 0;	/* index for state_map */
	idx_ft = 0;	/* index for enum_map */
	for (i = 0; i < prop_fmt->stn_num; i++) {
		int fmt_en = 0;
		for (j = 0; j < prop_res->stn_num; j++) {
			int state_id = temp[i*line_sz + j];
			if (state_id <= 0)
				continue;
			fmt_en = 1;
			/* get mbus_code and width,height from sensor driver */
			state_map[idx_st].reserved[0] = state_id;
			(*subdev->get_fmt_code)(snsr, i, state_map + idx_st);
			(*subdev->get_res_desc)(snsr, j, state_map + idx_st);
			idx_st++;
		}
		/* If at least one resolution is decleared for this format */
		if (fmt_en) {
			/* save format code in enum_map */
			enum_map[idx_ft++] = state_map[idx_st-1].code;
			/* save index of 1st item for NEXT format, will
			 * put them to the right position later */
			enum_map[idx_ft++] = idx_st;
		}
	}

	}
	/* move the 1st item index to the right position */
	for (idx_ft--; idx_ft > 1; idx_ft -= 2)
		enum_map[idx_ft] = enum_map[idx_ft-2];
	enum_map[idx_ft] = 0;
	/*ecs_dump_subdev_map(snsr);*/
	return 0;
}
EXPORT_SYMBOL(ecs_subdev_init);

int ecs_subdev_enum_fmt(struct ecs_sensor *snsr, int idx, \
			enum v4l2_mbus_pixelcode *code)
{
	struct ecs_property *prop_fmt = NULL;
	struct ecs_if_subdev *subdev;
	int *enum_map;

	if ((snsr == NULL) || (snsr->swif == NULL))
		return -EINVAL;
	subdev = snsr->swif;
	if (subdev->enum_map == NULL)
		return -EINVAL;

	enum_map = subdev->enum_map;
	prop_fmt = snsr->prop_tab + subdev->fmt_id;
	if ((idx >= prop_fmt->stn_num) || (enum_map[idx*2] == 0))
		return -EINVAL;
	*code = enum_map[idx*2];
	return 0;
}
EXPORT_SYMBOL(ecs_subdev_enum_fmt);

int ecs_subdev_enum_fsize(struct ecs_sensor *snsr, \
				struct v4l2_frmsizeenum *fsize)
{
	struct ecs_if_subdev *subdev;
	int *enum_map;
	struct v4l2_mbus_framefmt *state_map = NULL;
	int i, code, index;

	if ((snsr == NULL) || (snsr->swif == NULL))
		return -EINVAL;
	subdev = snsr->swif;
	if ((subdev->enum_map == NULL) || (subdev->state_map == NULL))
		return -EINVAL;

	state_map = subdev->state_map;
	enum_map = subdev->enum_map;
	code = fsize->pixel_format;
	index = fsize->index;

	for (i = 0; enum_map[i] != 0; i += 2)
		if (enum_map[i] == code)
			goto code_found;
	return -EINVAL;
code_found:
	state_map += (enum_map[i+1] + index);
	if (state_map->code != code)
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = state_map->width;
	fsize->discrete.height = state_map->height;
	return 0;
}
EXPORT_SYMBOL(ecs_subdev_enum_fsize);

int ecs_subdev_try_fmt(struct ecs_sensor *snsr, struct v4l2_mbus_framefmt *mf)
{
	struct ecs_if_subdev *subdev;
	int *enum_map;
	struct v4l2_mbus_framefmt *state_map = NULL;
	int i, code;

	if ((snsr == NULL) || (snsr->swif_type != ECS_IF_SUBDEV)
		|| (snsr->swif == NULL))
		return -EINVAL;
	subdev = snsr->swif;
	if ((subdev->enum_map == NULL) || (subdev->state_map == NULL))
		return -EINVAL;
	state_map = subdev->state_map;
	enum_map = subdev->enum_map;
	code = mf->code;

	for (i = 0; enum_map[i] != 0; i += 2)
		if (enum_map[i] == code)
			goto code_found;
	return -EINVAL;
code_found:
	/* Start searching from the 1st item for this format*/
	for (i = enum_map[i+1]; state_map[i].code == code; i++) {
		if ((state_map[i].width == mf->width)
		&& (state_map[i].height == mf->height))
			return state_map[i].reserved[0];
	}
	return -EPERM;
}
EXPORT_SYMBOL(ecs_subdev_try_fmt);

int ecs_subdev_set_fmt(struct ecs_sensor *snsr, struct v4l2_mbus_framefmt *mf)
{
	int state = ecs_subdev_try_fmt(snsr, mf);

	if (state < 0)
		return state;

	if (state == snsr->state_now)
		return 0;
	else
		return ecs_set_state(snsr, state);
}
EXPORT_SYMBOL(ecs_subdev_set_fmt);

int ecs_subdev_get_fmt(struct ecs_sensor *snsr, struct v4l2_mbus_framefmt *mf)
{
	const struct ecs_property *prop;
	struct ecs_if_subdev *subdev;

	if ((snsr == NULL) || (snsr->swif_type != ECS_IF_SUBDEV)
		|| (snsr->swif == NULL))
		return -EINVAL;
	subdev = snsr->swif;

	prop = ecs_find_property(snsr, subdev->fmt_id);
	if (prop == NULL)
		return -EINVAL;
	(*subdev->get_fmt_code)(snsr, prop->value_now, mf);

	prop = ecs_find_property(snsr, subdev->res_id);
	if (prop == NULL)
		return -EINVAL;
	(*subdev->get_res_desc)(snsr, prop->value_now, mf);
	return 0;
}
EXPORT_SYMBOL(ecs_subdev_get_fmt);

int ecs_subdev_set_stream(struct ecs_sensor *snsr, int enable)
{
	struct ecs_if_subdev *subdev;
	struct ecs_state_cfg cfg;

	if ((snsr == NULL) || (snsr->swif_type != ECS_IF_SUBDEV)
		|| (snsr->swif == NULL))
		return -EINVAL;
	subdev = snsr->swif;

	cfg.prop_id = subdev->str_id;
	cfg.prop_val = enable;

	return ecs_set_value(snsr, &cfg);
}
EXPORT_SYMBOL(ecs_subdev_set_stream);

void ecs_subdev_default_get_fmt_code(const struct ecs_sensor *snsr, \
				int fmt_value, struct v4l2_mbus_framefmt *mf)
{
	struct ecs_if_subdev *subdev = snsr->swif;
	struct ecs_property *prop = snsr->prop_tab + subdev->fmt_id;
	struct ecs_default_fmt_info *info = prop->stn_tab[fmt_value].info;

	mf->code = info->code;
	mf->colorspace = info->clrspc;
	mf->field = info->field;
}
EXPORT_SYMBOL(ecs_subdev_default_get_fmt_code);

void ecs_subdev_default_get_res_desc(const struct ecs_sensor *snsr, \
				int res_value, struct v4l2_mbus_framefmt *mf)
{
	struct ecs_if_subdev *subdev = snsr->swif;
	struct ecs_property *prop = snsr->prop_tab + subdev->res_id;
	struct ecs_default_res_info *info = prop->stn_tab[res_value].info;

	mf->width = info->h_act;
	mf->height = info->v_act;
}
EXPORT_SYMBOL(ecs_subdev_default_get_res_desc);

int ecs_orig_set_state(struct ecs_sensor *snsr, int state)
{
	if (unlikely((snsr == NULL) || (ecs_find_state(snsr, state) == NULL)))
		return -EINVAL;

	return ecs_set_state(snsr, state);
}
EXPORT_SYMBOL(ecs_orig_set_state);

int ecs_orig_set_list(struct ecs_sensor *snsr, \
					struct ecs_state_cfg *list, int len)
{
	int i, ret = 0;

	if ((snsr == NULL) || (list == NULL) || (len == 0))
		return -EINVAL;

	for (i = 0; i < len; i++) {
		int cnt;
		cnt = ecs_set_value(snsr, list);
		if (cnt < 0) {
			x_log(2, "set state failed");
			return -EIO;
		}
		ret += cnt;
		list++;
	}

	return ret;
}
EXPORT_SYMBOL(ecs_orig_set_list);

static __attribute__((unused)) void ecs_dump_subdev_map(struct ecs_sensor *snsr)
{
	struct ecs_if_subdev *subdev = snsr->swif;
	int *enum_map = NULL;
	struct v4l2_mbus_framefmt *state_map = NULL;
	int i;

	state_map = subdev->state_map;
	enum_map = subdev->enum_map;
	for (i = 0; state_map[i].code != 0; i++)
		printk(KERN_INFO "state=%2d, code=%04X, [w,h] = [%4d, %4d]\n", \
			state_map[i].reserved[0], state_map[i].code, \
			state_map[i].width, state_map[i].height);
	for (i = 0; enum_map[i] != 0; i += 2)
		printk(KERN_INFO "code=%04X, idx = %4d\n", \
			enum_map[i], enum_map[i+1]);

}

static __attribute__((unused)) void ecs_dump_property(struct ecs_property *prop)
{
	struct ecs_setting *stn;
	int i;
	printk(KERN_INFO "%15s: ", prop->name);
	for (i = 0; i < prop->stn_num; i++) {
		stn = prop->stn_tab + i;
		printk(KERN_INFO "%5s ", stn->name);
	}
	printk(KERN_INFO "\n");
};

static __attribute__((unused)) void ecs_sensor_dump(struct ecs_sensor *snsr)
{
	struct ecs_property *prop;
	int i;

	printk("--------------------------------\n" \
		".name = %s\n.prop:\n", snsr->name);
	for (i = 0; i < snsr->prop_num; i++) {
		prop = snsr->prop_tab + i;
		ecs_dump_property(prop);
	}
	printk("--------------------------------\n");
}

static __attribute__((unused)) int ecs_reg_array_dump(void *useless, \
							void *table, int len)
{
	int i;
	char *ch = table;

	if ((table == NULL) || (len == 0))
		return 0;

	for (i = 0; i < len; i += 4) {
		printk(KERN_INFO "%02X %02X %02X %02X\n", \
			ch[i], ch[i+1], ch[i+2], ch[i+3]);
	}
	return i;
}
