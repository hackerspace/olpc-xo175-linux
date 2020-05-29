// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Noralf Trønnes
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/lz4.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include <drm/drm_client.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_mode_object.h>
#include <drm/drm_plane.h>
#include <drm/drm_rect.h>
#include <drm/gud_drm.h>

#include "gud_drm_internal.h"

/*
 * Concurrency:
 * Calls into this module from f_gud_drm are serialized and run in process
 * context except gud_drm_gadget_ctrl_get() which is run in interrupt context.
 *
 * Termination:
 * A DRM client can not release itself, only the DRM driver which resources the
 * client uses can do that.
 * This means that there are 2 paths to stop the gadget function:
 * - Unregistering the DRM driver (module unload)
 * - Disabling the USB gadget (configfs unbind)
 *
 * A use counter protects the gadget should the client go away. A kref is used
 * to control the release of the gud_drm_gadget structure shared by the 2 actors.
 *
 * Backlight:
 * If there's a backlight device it's attached to the first connector.
 */

struct gud_drm_gadget_connector {
	struct drm_connector *connector;

	const struct gud_drm_property *properties;
	unsigned int num_properties;
	const char *tv_mode_enum_names;
	unsigned int num_tv_mode_enum_names;
	struct backlight_device *backlight;

	spinlock_t lock; /* Protects the following members: */
	enum drm_connector_status status;
	unsigned int width_mm;
	unsigned int height_mm;
	struct gud_drm_display_mode *modes;
	unsigned int num_modes;
	void *edid;
	size_t edid_len;
	bool changed;
};

struct gud_drm_gadget {
	struct kref refcount;
	refcount_t usecnt;
	struct drm_client_dev client;
	struct backlight_device *backlight;

	const u32 *formats;
	unsigned int format_count;

	const struct gud_drm_property *properties;
	unsigned int num_properties;

	struct gud_drm_gadget_connector *connectors;
	unsigned int connector_count;

	struct drm_rect set_buffer_rect;
	u32 set_buffer_length;
	u8 set_buffer_compression;
	u32 set_buffer_compressed_length;

	struct drm_client_buffer *buffer;
	struct drm_client_buffer *buffer_check;
	u8 brightness;
	bool check_ok;

	size_t max_buffer_size;
	void *work_buf;
};

static int gud_drm_gadget_probe_connector(struct gud_drm_gadget_connector *gconn)
{
	struct drm_connector *connector = gconn->connector;
	struct gud_drm_display_mode *modes = NULL;
	struct drm_device *drm = connector->dev;
	struct drm_display_mode *mode;
	void *edid_data, *edid = NULL;
	unsigned int num_modes = 0;
	size_t edid_len = 0;
	unsigned long flags;
	unsigned int i = 0;
	int ret = 0;

	mutex_lock(&drm->mode_config.mutex);

	connector->funcs->fill_modes(connector,
				     drm->mode_config.max_width,
				     drm->mode_config.max_height);

	list_for_each_entry(mode, &connector->modes, head)
		num_modes++;

	if (!num_modes)
		goto update;

	modes = kmalloc_array(num_modes, sizeof(*modes), GFP_KERNEL);
	if (!modes) {
		ret = -ENOMEM;
		num_modes = 0;
		goto update;
	}

	list_for_each_entry(mode, &connector->modes, head)
		gud_drm_from_display_mode(&modes[i++], mode);

	if (!connector->edid_blob_ptr)
		goto update;

	edid_data = connector->edid_blob_ptr->data;
	edid_len = connector->edid_blob_ptr->length;
	if (!edid_data || !edid_len) {
		edid_len = 0;
		goto update;
	}

	edid = kmemdup(edid_data, edid_len, GFP_KERNEL);
	if (!edid) {
		ret = -ENOMEM;
		edid_len = 0;
	}

update:
	spin_lock_irqsave(&gconn->lock, flags);
	if (gconn->status != connector->status || gconn->num_modes != num_modes ||
	    gconn->edid_len != edid_len ||
	    (gconn->modes && modes && memcmp(gconn->modes, modes, num_modes * sizeof(*modes))) ||
	    (gconn->edid && edid && memcmp(gconn->edid, edid, edid_len)))
		gconn->changed = true;
	swap(gconn->modes, modes);
	gconn->num_modes = num_modes;
	swap(gconn->edid, edid);
	gconn->edid_len = edid_len;
	gconn->width_mm = connector->display_info.width_mm;
	gconn->height_mm = connector->display_info.height_mm;
	gconn->status = connector->status;
	spin_unlock_irqrestore(&gconn->lock, flags);

	mutex_unlock(&drm->mode_config.mutex);

	kfree(edid);
	kfree(modes);

	return ret;
}

static void gud_drm_gadget_probe_connectors(struct gud_drm_gadget *gdg)
{
	unsigned int i;

	for (i = 0; i < gdg->connector_count; i++)
		gud_drm_gadget_probe_connector(&gdg->connectors[i]);
}

static bool gud_drm_gadget_check_buffer(struct gud_drm_gadget *gdg,
					struct drm_client_buffer *buffer,
					struct drm_display_mode *mode,
					u32 format)
{
	struct drm_framebuffer *fb;

	if (!buffer)
		return false;

	fb = buffer->fb;

	return fb->format->format == format &&
	       fb->width == mode->hdisplay && fb->height == mode->vdisplay;
}

static bool gud_drm_gadget_set_connector_property(struct drm_client_dev *client,
						  struct drm_connector *connector,
						  u16 prop, u64 val, int *ret)
{
	struct drm_mode_config *config = &connector->dev->mode_config;
	struct drm_property *property;

	switch (prop) {
	case GUD_DRM_PROPERTY_TV_SELECT_SUBCONNECTOR:
		property = config->tv_select_subconnector_property;
		break;
	case GUD_DRM_PROPERTY_TV_LEFT_MARGIN:
		property = config->tv_left_margin_property;
		break;
	case GUD_DRM_PROPERTY_TV_RIGHT_MARGIN:
		property = config->tv_right_margin_property;
		break;
	case GUD_DRM_PROPERTY_TV_TOP_MARGIN:
		property = config->tv_top_margin_property;
		break;
	case GUD_DRM_PROPERTY_TV_BOTTOM_MARGIN:
		property = config->tv_bottom_margin_property;
		break;
	case GUD_DRM_PROPERTY_TV_MODE:
		property = config->tv_mode_property;
		break;
	case GUD_DRM_PROPERTY_TV_BRIGHTNESS:
		property = config->tv_brightness_property;
		break;
	case GUD_DRM_PROPERTY_TV_CONTRAST:
		property = config->tv_contrast_property;
		break;
	case GUD_DRM_PROPERTY_TV_FLICKER_REDUCTION:
		property = config->tv_flicker_reduction_property;
		break;
	case GUD_DRM_PROPERTY_TV_OVERSCAN:
		property = config->tv_overscan_property;
		break;
	case GUD_DRM_PROPERTY_TV_SATURATION:
		property = config->tv_saturation_property;
		break;
	case GUD_DRM_PROPERTY_TV_HUE:
		property = config->tv_hue_property;
		break;
	default:
		return false;
	}

	*ret = drm_client_modeset_set_property(client, &connector->base, property, val);

	return true;
}

static int gud_drm_gadget_check(struct gud_drm_gadget *gdg, struct gud_drm_req_set_state *req,
				size_t size)
{
	struct drm_client_dev *client = &gdg->client;
	u32 format = le32_to_cpu(req->format);
	struct drm_client_buffer *buffer;
	struct drm_connector *connector;
	struct drm_display_mode mode;
	unsigned int i;
	void *vaddr;
	int ret;

	if (size < sizeof(struct gud_drm_req_set_state))
		return -EINVAL;

	if (size != struct_size(req, properties, req->num_properties))
		return -EINVAL;

	memset(&mode, 0, sizeof(mode));
	gud_drm_to_display_mode(&mode, &req->mode);

	gdg->check_ok = false;

	if (!mode.hdisplay || !format)
		return -EINVAL;

	if (req->connector >= gdg->connector_count)
		return -EINVAL;

	connector = gdg->connectors[req->connector].connector;

	if (gdg->buffer_check) {
		drm_client_framebuffer_delete(gdg->buffer_check);
		gdg->buffer_check = NULL;
	}

	if (!gud_drm_gadget_check_buffer(gdg, gdg->buffer, &mode, format)) {
		buffer = drm_client_framebuffer_create(client, mode.hdisplay, mode.vdisplay,
						       format);
		if (IS_ERR(buffer))
			return PTR_ERR(buffer);

		vaddr = drm_client_buffer_vmap(buffer);
		if (IS_ERR(vaddr)) {
			drm_client_framebuffer_delete(buffer);
			return PTR_ERR(vaddr);
		}

		gdg->buffer_check = buffer;
	} else {
		buffer = gdg->buffer;
	}

	ret = drm_client_modeset_set(client, connector, &mode, buffer->fb);
	if (ret)
		return ret;

	for (i = 0; i < req->num_properties; i++) {
		u16 prop = le16_to_cpu(req->properties[i].prop);
		u64 val = le64_to_cpu(req->properties[i].val);

		if (gud_drm_gadget_set_connector_property(client, connector, prop, val, &ret)) {
			if (ret)
				return ret;
			continue;
		}

		switch (prop) {
		case GUD_DRM_PROPERTY_BACKLIGHT_BRIGHTNESS:
			if (val > 100)
				return -EINVAL;
			gdg->brightness = val;
			break;
		case GUD_DRM_PROPERTY_ROTATION:
			ret = drm_client_modeset_set_rotation(client, val);
			break;
		default:
			pr_err("%s: Unknown property: %u\n", __func__, prop);
			continue;
		}

		if (ret)
			return ret;
	}

	ret = drm_client_modeset_check(&gdg->client);
	if (ret)
		return ret;

	gdg->check_ok = true;

	return 0;
}

static int gud_drm_gadget_commit(struct gud_drm_gadget *gdg)
{
	int ret;

	if (!gdg->check_ok)
		return -EINVAL;

	if (gdg->backlight) {
		int val, max_brightness = gdg->backlight->props.max_brightness;

		val = DIV64_U64_ROUND_UP(gdg->brightness * max_brightness, 100);
		ret = backlight_device_set_brightness(gdg->backlight, val);
		if (ret)
			return ret;
	}

	ret = drm_client_modeset_commit(&gdg->client);
	if (ret)
		return ret;

	if (gdg->buffer_check) {
		drm_client_framebuffer_delete(gdg->buffer);
		gdg->buffer = gdg->buffer_check;
		gdg->buffer_check = NULL;
	}

	return 0;
}

static size_t gud_drm_gadget_write_buffer_memcpy(struct drm_client_buffer *buffer,
						 const void *src, size_t len,
						 struct drm_rect *rect)
{
	unsigned int cpp = buffer->fb->format->cpp[0];
	size_t dst_pitch = buffer->fb->pitches[0];
	size_t src_pitch = drm_rect_width(rect) * cpp;
	unsigned int y;
	void *dst;

	/* Get the address, it's already mapped */
	dst = drm_client_buffer_vmap(buffer);
	dst += rect->y1 * dst_pitch;
	dst += rect->x1 * cpp;

	for (y = 0; y < drm_rect_height(rect) && len; y++) {
		src_pitch = min(src_pitch, len);
		memcpy(dst, src, src_pitch);
		src += src_pitch;
		dst += dst_pitch;
		len -= src_pitch;
	}

	return len;
}

static bool gud_drm_gadget_check_rect(struct drm_client_buffer *buffer, struct drm_rect *rect)
{
	return buffer->fb && rect->x1 < rect->x2 && rect->y1 < rect->y2 &&
	       rect->x2 <= buffer->fb->width && rect->y2 <= buffer->fb->height;
}

int gud_drm_gadget_write_buffer(struct gud_drm_gadget *gdg, const void *buf, size_t len)
{
	struct drm_client_buffer *buffer = gdg->buffer ? gdg->buffer : gdg->buffer_check;
	struct drm_rect *rect = &gdg->set_buffer_rect;
	u8 compression = gdg->set_buffer_compression;
	struct drm_framebuffer *fb;
	size_t remain;
	int ret;

	pr_debug("%s: len=%zu compression=0x%x\n", __func__, len, compression);

	if (WARN_ON_ONCE(!buffer))
		return -ENOMEM;

	if (!gud_drm_gadget_check_rect(buffer, rect)) {
		pr_err("%s: Rectangle doesn't fit: " DRM_RECT_FMT "\n",
		       __func__, DRM_RECT_ARG(rect));
		return -EINVAL;
	}

	fb = buffer->fb;

	if (compression & GUD_DRM_COMPRESSION_LZ4) {
		if (len != gdg->set_buffer_compressed_length) {
			pr_err("%s: Buffer compressed len differs: %zu != %zu\n",
			       __func__, len, gdg->set_buffer_compressed_length);
			return -EINVAL;
		}

		ret = LZ4_decompress_safe(buf, gdg->work_buf, len, gdg->max_buffer_size);
		if (ret < 0) {
			pr_err("%s: Failed to decompress buffer\n", __func__);
			return -EIO;
		}

		buf = gdg->work_buf;
		len = ret;
	}

	if (len != gdg->set_buffer_length) {
		pr_err("%s: Buffer len differs: %zu != %zu\n",
		       __func__, len, gdg->set_buffer_length);
		return -EINVAL;
	}

	if (len > (drm_rect_width(rect) * drm_rect_height(rect) * fb->format->cpp[0])) {
		pr_err("%s: Buffer is too big for rectangle: " DRM_RECT_FMT " len=%zu\n",
		       __func__, DRM_RECT_ARG(rect), len);
		return -EINVAL;
	}

	remain = gud_drm_gadget_write_buffer_memcpy(buffer, buf, len, rect);
	if (remain) {
		pr_err("%s: Failed to write buffer: remain=%zu\n", __func__, remain);
		return -EIO;
	}

	return drm_client_framebuffer_flush(buffer, rect);
}
EXPORT_SYMBOL(gud_drm_gadget_write_buffer);

int gud_drm_gadget_set_buffer(struct gud_drm_gadget *gdg, struct gud_drm_req_set_buffer *req)
{
	u32 compressed_length = le32_to_cpu(req->compressed_length);
	u32 length = le32_to_cpu(req->length);
	struct drm_client_buffer *buffer;
	struct drm_rect rect;
	int ret = 0;

	if (!refcount_inc_not_zero(&gdg->usecnt))
		return -ENODEV;

	buffer = gdg->buffer ? gdg->buffer : gdg->buffer_check;
	if (!buffer) {
		ret = -ENOENT;
		goto out;
	}

	drm_rect_init(&rect, le32_to_cpu(req->x), le32_to_cpu(req->y),
		      le32_to_cpu(req->width), le32_to_cpu(req->height));

	pr_debug("%s: " DRM_RECT_FMT "\n", __func__, DRM_RECT_ARG(&rect));

	if (!gud_drm_gadget_check_rect(buffer, &rect)) {
		ret = -EINVAL;
		goto out;
	}

	if (req->compression & ~GUD_DRM_COMPRESSION_LZ4) {
		ret = -EINVAL;
		goto out;
	}

	gdg->set_buffer_rect = rect;
	gdg->set_buffer_length = length;

	if (req->compression) {
		if (!compressed_length) {
			ret = -EINVAL;
			goto out;
		}
		gdg->set_buffer_compression = req->compression;
		gdg->set_buffer_compressed_length = compressed_length;
		length = compressed_length;
	} else {
		gdg->set_buffer_compression = 0;
		gdg->set_buffer_compressed_length = 0;
	}
out:
	refcount_dec(&gdg->usecnt);

	return ret ? ret : length;
}
EXPORT_SYMBOL(gud_drm_gadget_set_buffer);

static void gud_drm_gadget_delete_buffers(struct gud_drm_gadget *gdg)
{
	drm_client_framebuffer_delete(gdg->buffer_check);
	drm_client_framebuffer_delete(gdg->buffer);
	gdg->buffer_check = NULL;
	gdg->buffer = NULL;
}

int gud_drm_gadget_disable_pipe(struct gud_drm_gadget *gdg)
{
	int ret;

	ret = drm_client_modeset_disable(&gdg->client);
	gud_drm_gadget_delete_buffers(gdg);

	return ret;
}
EXPORT_SYMBOL(gud_drm_gadget_disable_pipe);

static int gud_drm_gadget_ctrl_get_display_descriptor(struct gud_drm_gadget *gdg, u16 value,
						      void *data, size_t size)
{
	struct drm_device *drm = gdg->client.dev;
	struct gud_drm_display_descriptor desc;
	u8 type = value >> 8;
	u8 index = value & 0xff;

	if (type != GUD_DRM_USB_DT_DISPLAY || index)
		return -EINVAL;

	desc.bLength = sizeof(desc);
	desc.bDescriptorType = GUD_DRM_USB_DT_DISPLAY;

	desc.bVersion = 1;
	desc.bMaxBufferSizeOrder = ilog2(gdg->max_buffer_size);
	desc.bmFlags = 0;
	desc.bCompression = GUD_DRM_COMPRESSION_LZ4;

	desc.dwMinWidth = cpu_to_le32(drm->mode_config.min_width);
	desc.dwMaxWidth = cpu_to_le32(drm->mode_config.max_width);
	desc.dwMinHeight = cpu_to_le32(drm->mode_config.min_height);
	desc.dwMaxHeight = cpu_to_le32(drm->mode_config.max_height);
	desc.bNumFormats = gdg->format_count;
	desc.bNumProperties = gdg->num_properties;
	desc.bNumConnectors = gdg->connector_count;

	size = min(size, sizeof(desc));
	memcpy(data, &desc, size);

	return size;
}

static void gud_drm_gadget_ctrl_get_formats(struct gud_drm_gadget *gdg, __le32 *formats)
{
	unsigned int i;

	for (i = 0; i < gdg->format_count; i++)
		formats[i] = cpu_to_le32(gdg->formats[i]);
}

static int gud_drm_gadget_ctrl_get_connector(struct gud_drm_gadget *gdg, unsigned int index,
					     struct gud_drm_req_get_connector *desc)
{
	struct gud_drm_gadget_connector *gconn;

	if (index >= gdg->connector_count)
		return -EINVAL;

	memset(desc, 0, sizeof(*desc));

	gconn = &gdg->connectors[index];

	desc->connector_type = gconn->connector->connector_type;
	desc->flags = cpu_to_le32(GUD_DRM_CONNECTOR_FLAGS_POLL);
	desc->num_properties = gconn->num_properties;

	return 0;
}

static struct gud_drm_gadget_connector *
gud_drm_gadget_get_gconn(struct gud_drm_gadget *gdg, unsigned int index)
{
	if (index >= gdg->connector_count)
		return NULL;

	return &gdg->connectors[index];
}

static int gud_drm_gadget_ctrl_get_connector_status(struct gud_drm_gadget *gdg, unsigned int index,
						    struct gud_drm_req_get_connector_status *status)
{
	struct gud_drm_gadget_connector *gconn;
	unsigned long flags;

	gconn = gud_drm_gadget_get_gconn(gdg, index);
	if (!gconn)
		return -ENOENT;

	memset(status, 0, sizeof(*status));

	spin_lock_irqsave(&gconn->lock, flags);
	status->status = gconn->status;
	if (gconn->changed) {
		status->status |= GUD_DRM_CONNECTOR_STATUS_CHANGED;
		gconn->changed = false;
	}
	if (gconn->num_modes)
		status->num_modes = cpu_to_le16(gconn->num_modes);
	if (gconn->edid_len)
		status->edid_len = cpu_to_le16(gconn->edid_len);
	spin_unlock_irqrestore(&gconn->lock, flags);

	return 0;
}

/* This runs in interrupt context */
int gud_drm_gadget_ctrl_get(struct gud_drm_gadget *gdg, u8 request,
			    u16 index, void *data, size_t size)
{
	struct gud_drm_gadget_connector *gconn;
	unsigned long flags;
	int ret = -EINVAL;

	pr_debug("%s: request=0x%x index=%u size=%zu\n", __func__, request, index, size);

	if (!refcount_inc_not_zero(&gdg->usecnt))
		return -ENODEV;

	if (!size)
		goto out;

	switch (request) {
	case USB_REQ_GET_DESCRIPTOR:
		ret = gud_drm_gadget_ctrl_get_display_descriptor(gdg, index, data, size);
		break;
	case GUD_DRM_USB_REQ_GET_FORMATS:
		if (!index && size == gdg->format_count * sizeof(u32)) {
			gud_drm_gadget_ctrl_get_formats(gdg, data);
			ret = 0;
		}
		break;
	case GUD_DRM_USB_REQ_GET_PROPERTIES:
		if (!index && size == gdg->num_properties * sizeof(*gdg->properties)) {
			memcpy(data, gdg->properties, size);
			ret = 0;
		}
		break;
	case GUD_DRM_USB_REQ_GET_CONNECTOR:
		if (size == sizeof(struct gud_drm_req_get_connector))
			ret = gud_drm_gadget_ctrl_get_connector(gdg, index, data);
		break;
	case GUD_DRM_USB_REQ_GET_CONNECTOR_PROPERTIES:
		gconn = gud_drm_gadget_get_gconn(gdg, index);
		if (gconn && size == gconn->num_properties * sizeof(*gconn->properties)) {
			memcpy(data, gconn->properties, size);
			ret = 0;
		}
		break;
	case GUD_DRM_USB_REQ_GET_CONNECTOR_TV_MODE_VALUES:
		gconn = gud_drm_gadget_get_gconn(gdg, index);
		if (gconn && size == gconn->num_tv_mode_enum_names * DRM_PROP_NAME_LEN) {
			memcpy(data, gconn->tv_mode_enum_names, size);
			ret = 0;
		}
		break;
	case GUD_DRM_USB_REQ_GET_CONNECTOR_STATUS:
		if (size == sizeof(struct gud_drm_req_get_connector_status))
			ret = gud_drm_gadget_ctrl_get_connector_status(gdg, index, data);
		break;
	case GUD_DRM_USB_REQ_GET_CONNECTOR_MODES:
		gconn = gud_drm_gadget_get_gconn(gdg, index);
		spin_lock_irqsave(&gconn->lock, flags);
		if (gconn && size == gconn->num_modes * sizeof(*gconn->modes)) {
			memcpy(data, gconn->modes, size);
			ret = 0;
		}
		spin_unlock_irqrestore(&gconn->lock, flags);
		break;
	case GUD_DRM_USB_REQ_GET_CONNECTOR_EDID:
		gconn = gud_drm_gadget_get_gconn(gdg, index);
		spin_lock_irqsave(&gconn->lock, flags);
		if (gconn && size == gconn->edid_len) {
			memcpy(data, gconn->edid, size);
			ret = 0;
		}
		spin_unlock_irqrestore(&gconn->lock, flags);
		break;
	}
out:
	refcount_dec(&gdg->usecnt);

	return !ret ? size : ret;
}
EXPORT_SYMBOL(gud_drm_gadget_ctrl_get);

int gud_drm_gadget_ctrl_set(struct gud_drm_gadget *gdg, u8 request,
			    u16 index, void *data, size_t size)
{
	struct gud_drm_gadget_connector *gconn;
	int dpms, ret = -EINVAL;

	pr_debug("%s: request=0x%x index=%u size=%zu\n", __func__, request, index, size);

	if (!refcount_inc_not_zero(&gdg->usecnt))
		return -ENODEV;

	switch (request) {
	case GUD_DRM_USB_REQ_SET_CONNECTOR_FORCE_DETECT:
		gconn = gud_drm_gadget_get_gconn(gdg, index);
		if (gconn)
			ret = gud_drm_gadget_probe_connector(gconn);
		break;
	case GUD_DRM_USB_REQ_SET_STATE_CHECK:
		ret = gud_drm_gadget_check(gdg, data, size);
		break;
	case GUD_DRM_USB_REQ_SET_STATE_COMMIT:
		if (!size)
			ret = gud_drm_gadget_commit(gdg);
		break;
	case GUD_DRM_USB_REQ_SET_CONTROLLER_ENABLE:
		if (size == sizeof(u8)) {
			if (*(u8 *)data == 0)
				ret = gud_drm_gadget_disable_pipe(gdg);
			else
				ret = 0;
		}
		break;
	case GUD_DRM_USB_REQ_SET_DISPLAY_ENABLE:
		if (size == sizeof(u8)) {
			dpms = *(u8 *)data ? DRM_MODE_DPMS_ON : DRM_MODE_DPMS_OFF;
			ret = drm_client_modeset_dpms(&gdg->client, dpms);
		}
		break;
	}

	refcount_dec(&gdg->usecnt);

	return ret;
}
EXPORT_SYMBOL(gud_drm_gadget_ctrl_set);

static int gud_drm_gadget_get_formats(struct gud_drm_gadget *gdg, u8 *max_cpp)
{
	struct drm_device *drm = gdg->client.dev;
	struct drm_plane *plane;
	unsigned int i;
	u32 *formats;
	int ret;

	*max_cpp = 0;

	drm_for_each_plane(plane, drm) {
		if (plane->type == DRM_PLANE_TYPE_PRIMARY)
			break;
	}

	formats = kcalloc(plane->format_count, sizeof(u32), GFP_KERNEL);
	if (!formats)
		return -ENOMEM;

	for (i = 0; i < plane->format_count; i++) {
		const struct drm_format_info *fmt;

		fmt = drm_format_info(plane->format_types[i]);
		if (fmt->num_planes != 1)
			continue;

		/*
		 * Supporting 24-bit bpp would add complexity so don't bother.
		 * It's hardly used and compression removes much of the gain.
		 */
		if (fmt->cpp[0] == 3)
			continue;

		if (*max_cpp < fmt->cpp[0])
			*max_cpp = fmt->cpp[0];

		formats[gdg->format_count++] = plane->format_types[i];
	}

	if (!gdg->format_count) {
		ret = -ENOENT;
		goto err_free;
	}

	gdg->formats = formats;

	return 0;

err_free:
	kfree(formats);

	return ret;
}

static int gud_drm_gadget_get_rotation_property(struct drm_device *drm, u16 *prop, u64 *val)
{
	struct drm_property_enum *prop_enum;
	struct drm_plane *plane;
	unsigned int num_props = 0;
	u16 bitmask = 0;

	drm_for_each_plane(plane, drm) {
		if (plane->type == DRM_PLANE_TYPE_PRIMARY)
			break;
	}

	if (!plane->rotation_property)
		return 0;

	list_for_each_entry(prop_enum, &plane->rotation_property->enum_list, head) {
		num_props++;
		bitmask |= BIT(prop_enum->value);
	}

	*prop = GUD_DRM_PROPERTY_ROTATION;
	*val = bitmask;

	return 1;
}

static int gud_drm_gadget_get_properties(struct gud_drm_gadget *gdg)
{
	struct gud_drm_property *properties;
	u16 prop;
	u64 val;
	int ret;

	ret = gud_drm_gadget_get_rotation_property(gdg->client.dev, &prop, &val);
	if (ret <= 0)
		return ret;

	properties = kcalloc(1, sizeof(*properties), GFP_KERNEL);
	if (!properties)
		return -ENOMEM;

	gdg->properties = properties;
	gdg->num_properties++;

	properties[0].prop = cpu_to_le16(prop);
	properties[0].val = cpu_to_le64(val);

	return 0;
}

static int gud_drm_gadget_get_connector_properties(struct gud_drm_gadget *gdg,
						   struct gud_drm_gadget_connector *gconn)
{
	struct drm_device *drm = gdg->client.dev;
	struct drm_mode_config *config = &drm->mode_config;
	struct drm_connector *connector = gconn->connector;
	struct drm_object_properties *conn_props = connector->base.properties;
	struct gud_drm_property *properties;
	unsigned int i, ret = 0;
	u16 prop;
	u64 val;

	mutex_lock(&drm->mode_config.mutex);

	if (!conn_props->count)
		goto unlock;

	/* Add room for possible backlight */
	properties = kcalloc(conn_props->count + 1, sizeof(*properties), GFP_KERNEL);
	if (!properties) {
		ret = -ENOMEM;
		goto unlock;
	}

	gconn->properties = properties;

	for (i = 0; i < conn_props->count; i++) {
		struct drm_property *property = conn_props->properties[i];

		if (property == config->tv_select_subconnector_property) {
			prop = GUD_DRM_PROPERTY_TV_SELECT_SUBCONNECTOR;
			val = connector->state->tv.subconnector;
		} else if (property == config->tv_left_margin_property) {
			prop = GUD_DRM_PROPERTY_TV_LEFT_MARGIN;
			val = connector->state->tv.margins.left;
		} else if (property == config->tv_right_margin_property) {
			prop = GUD_DRM_PROPERTY_TV_RIGHT_MARGIN;
			val = connector->state->tv.margins.right;
		} else if (property == config->tv_top_margin_property) {
			prop = GUD_DRM_PROPERTY_TV_TOP_MARGIN;
			val = connector->state->tv.margins.top;
		} else if (property == config->tv_bottom_margin_property) {
			prop = GUD_DRM_PROPERTY_TV_BOTTOM_MARGIN;
			val = connector->state->tv.margins.bottom;
		} else if (property == config->tv_mode_property) {
			struct drm_property_enum *prop_enum;
			char *buf;

			list_for_each_entry(prop_enum, &property->enum_list, head)
				gconn->num_tv_mode_enum_names++;

			if (WARN_ON(!gconn->num_tv_mode_enum_names)) {
				ret = -EINVAL;
				goto unlock;
			}

			buf = kcalloc(gconn->num_tv_mode_enum_names, DRM_PROP_NAME_LEN, GFP_KERNEL);
			if (!buf) {
				ret = -ENOMEM;
				goto unlock;
			}

			gconn->tv_mode_enum_names = buf;

			list_for_each_entry(prop_enum, &property->enum_list, head) {
				strncpy(buf, prop_enum->name, DRM_PROP_NAME_LEN);
				buf += DRM_PROP_NAME_LEN;
			}

			prop = GUD_DRM_PROPERTY_TV_MODE;
			val = connector->state->tv.mode;
			val |= gconn->num_tv_mode_enum_names << GUD_DRM_USB_CONNECTOR_TV_MODE_NUM_SHIFT;
		} else if (property == config->tv_brightness_property) {
			prop = GUD_DRM_PROPERTY_TV_BRIGHTNESS;
			val = connector->state->tv.brightness;
		} else if (property == config->tv_contrast_property) {
			prop = GUD_DRM_PROPERTY_TV_CONTRAST;
			val = connector->state->tv.contrast;
		} else if (property == config->tv_flicker_reduction_property) {
			prop = GUD_DRM_PROPERTY_TV_FLICKER_REDUCTION;
			val = connector->state->tv.flicker_reduction;
		} else if (property == config->tv_overscan_property) {
			prop = GUD_DRM_PROPERTY_TV_OVERSCAN;
			val = connector->state->tv.overscan;
		} else if (property == config->tv_saturation_property) {
			prop = GUD_DRM_PROPERTY_TV_SATURATION;
			val = connector->state->tv.saturation;
		} else if (property == config->tv_hue_property) {
			prop = GUD_DRM_PROPERTY_TV_HUE;
			val = connector->state->tv.hue;
		} else {
			continue;
		}

		properties[gconn->num_properties].prop = cpu_to_le16(prop);
		properties[gconn->num_properties++].val = cpu_to_le64(val);
	}

	if (!connector->index && gdg->backlight) {
		struct backlight_properties *props = &gdg->backlight->props;

		prop = GUD_DRM_PROPERTY_BACKLIGHT_BRIGHTNESS;
		val = DIV64_U64_ROUND_UP(props->brightness * 100, props->max_brightness);
		properties[gconn->num_properties].prop = cpu_to_le16(prop);
		properties[gconn->num_properties++].val = cpu_to_le64(val);
		gconn->backlight = gdg->backlight;
	}
unlock:
	mutex_unlock(&drm->mode_config.mutex);

	return ret;
}

static int gud_drm_gadget_get_connectors(struct gud_drm_gadget *gdg)
{
	struct gud_drm_gadget_connector *connectors = NULL;
	struct drm_connector_list_iter conn_iter;
	struct drm_device *drm = gdg->client.dev;
	unsigned int connector_count = 0;
	struct drm_connector *connector;
	int ret = 0;

	drm_connector_list_iter_begin(drm, &conn_iter);
	drm_client_for_each_connector_iter(connector, &conn_iter) {
		struct gud_drm_gadget_connector *tmp, *gconn;

		tmp = krealloc(connectors, (connector_count + 1) * sizeof(*connectors),
			       GFP_KERNEL | __GFP_ZERO);
		if (!tmp) {
			ret = -ENOMEM;
			break;
		}

		connectors = tmp;
		drm_connector_get(connector);
		gconn = &connectors[connector_count++];
		gconn->connector = connector;
		spin_lock_init(&gconn->lock);

		ret = gud_drm_gadget_get_connector_properties(gdg, gconn);
		if (ret)
			break;
	}
	drm_connector_list_iter_end(&conn_iter);

	gdg->connectors = connectors;
	gdg->connector_count = connector_count;

	return ret;
}

static void gud_drm_gadget_release(struct kref *kref)
{
	struct gud_drm_gadget *gdg = container_of(kref, struct gud_drm_gadget, refcount);

	kfree(gdg);
}

static void gud_drm_gadget_put(struct gud_drm_gadget *gdg)
{
	kref_put(&gdg->refcount, gud_drm_gadget_release);
}

static void gud_drm_gadget_client_unregister(struct drm_client_dev *client)
{
	struct gud_drm_gadget *gdg = container_of(client, struct gud_drm_gadget, client);
	int timeout = 10000 / 50;
	unsigned int i;

	/*
	 * If usecnt doesn't drop to zero, try waiting for the gadget, but we
	 * can't block the DRM driver forever. The worst case wait the gadget side
	 * can hit are tens of seconds through the call to drm_client_modeset_commit().
	 */
	if (refcount_dec_and_test(&gdg->usecnt)) {
		for (; timeout && refcount_read(&gdg->usecnt); timeout--)
			msleep(50);
	}

	if (!timeout) {
		pr_err("%s: Timeout waiting for gadget side, will leak memory\n", __func__);
		return;
	}

	vfree(gdg->work_buf);
	kfree(gdg->formats);
	kfree(gdg->properties);

	for (i = 0; i < gdg->connector_count; i++) {
		struct gud_drm_gadget_connector *gconn = &gdg->connectors[i];

		drm_connector_put(gconn->connector);
		kfree(gconn->properties);
		kfree(gconn->tv_mode_enum_names);
		kfree(gconn->modes);
		kfree(gconn->edid);
	}
	kfree(gdg->connectors);

	gud_drm_gadget_delete_buffers(gdg);
	drm_client_release(client);
	gud_drm_gadget_put(gdg);
}

static int gud_drm_gadget_client_hotplug(struct drm_client_dev *client)
{
	struct gud_drm_gadget *gdg = container_of(client, struct gud_drm_gadget, client);

	gud_drm_gadget_probe_connectors(gdg);

	return 0;
}

static const struct drm_client_funcs gdg_client_funcs = {
	.owner		= THIS_MODULE,
	.unregister	= gud_drm_gadget_client_unregister,
	.hotplug	= gud_drm_gadget_client_hotplug,
};

struct gud_drm_gadget *gud_drm_gadget_init(unsigned int minor_id, const char *backlight,
					   size_t *max_buffer_size)
{
	struct gud_drm_gadget *gdg;
	u8 max_cpp;
	int ret;

	gdg = kzalloc(sizeof(*gdg), GFP_KERNEL);
	if (!gdg)
		return ERR_PTR(-ENOMEM);

	ret = drm_client_init_from_id(minor_id, &gdg->client, "gud-drm-gadget", &gdg_client_funcs);
	if (ret) {
		pr_err("Failed to aquire minor=%u\n", minor_id);
		kfree(gdg);
		return ERR_PTR(ret);
	}

	refcount_set(&gdg->usecnt, 1);
	/* The DRM driver (through the client) and f_gud_drm need one ref each */
	kref_init(&gdg->refcount);
	kref_get(&gdg->refcount);

	if (backlight) {
		gdg->backlight = backlight_device_get_by_name(backlight);
		if (!gdg->backlight) {
			pr_err("Failed to find backlight: %s\n", backlight);
			ret = -ENODEV;
			goto error_release;
		}
	}

	ret = gud_drm_gadget_get_formats(gdg, &max_cpp);
	if (ret) {
		pr_err("Failed to get formats\n");
		goto error_release;
	}

	*max_buffer_size = gdg->client.dev->mode_config.max_width *
			   gdg->client.dev->mode_config.max_height * max_cpp;
	/* f_gud_drm will kmalloc a buffer of this size */
	*max_buffer_size = min_t(size_t, *max_buffer_size, KMALLOC_MAX_SIZE);

	gdg->max_buffer_size = *max_buffer_size;
	gdg->work_buf = vmalloc(gdg->max_buffer_size);
	if (!gdg->work_buf) {
		ret = -ENOMEM;
		goto error_release;
	}

	ret = gud_drm_gadget_get_properties(gdg);
	if (ret) {
		pr_err("Failed to get properties\n");
		goto error_release;
	}

	ret = gud_drm_gadget_get_connectors(gdg);
	if (ret) {
		pr_err("Failed to get connectors\n");
		goto error_release;
	}

	if (!drm_client_register(&gdg->client)) {
		pr_err("DRM device is gone\n");
		ret = -ENODEV;
		goto error_release;
	}

	gud_drm_gadget_probe_connectors(gdg);

	return gdg;

error_release:
	gud_drm_gadget_client_unregister(&gdg->client);
	gud_drm_gadget_fini(gdg);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL(gud_drm_gadget_init);

void gud_drm_gadget_fini(struct gud_drm_gadget *gdg)
{
	backlight_put(gdg->backlight);
	gud_drm_gadget_put(gdg);
}
EXPORT_SYMBOL(gud_drm_gadget_fini);

MODULE_AUTHOR("Noralf Trønnes");
MODULE_LICENSE("GPL");
