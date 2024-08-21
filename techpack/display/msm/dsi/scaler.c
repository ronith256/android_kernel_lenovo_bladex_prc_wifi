#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/mutex.h>
#include "../../../../sound/soc/codecs/tfa98xx/inc/tfa_device.h"
#include "../../../../sound/soc/codecs/tfa98xx/inc/tfa.h"
#include <drm/drm_panel.h>
#include "../../../../drivers/input/touchscreen/goodix_gt738x/goodix_ts_core.h"

#include "scaler.h"
#include "dsi86-edp-mux.h"
#include "dsi_panel.h"

static struct drm_panel *active_panel;
#define INPUT_POWER_KEY

#define TOUCH_VOL_DOWN_LEFT	945
#define TOUCH_VOL_DOWN_RIGHT	1030
#define TOUCH_VOL_DOWN_TOP	1295
#define TOUCH_VOL_UP_LEFT		1120
#define TOUCH_VOL_UP_RIGHT		1210
#define TOUCH_VOL_UP_TOP		1295

#define MAX_BRIGHTNESS_STEPS	16	// 0~15
static u8 brightness_mapping_table[MAX_BRIGHTNESS_STEPS] = {5, 17, 34, 51, 68, 85, 102, 119, 136, 153, 170, 187, 204, 221, 238, 255};
#define MAX_TRY_FOR_LONG_PRESS	2

#ifdef CONFIG_HW_BOOT_INFO
extern int register_hardware_info(const char *name,const char *model);
#endif

static int notify_scaler_brightness_and_volume(struct scaler_data *data);
static int notify_scaler_battery_status(struct scaler_data *data);
static int notify_scaler_to_init_panel(struct scaler_data *data);

static struct scaler_data *g_data;
static int scaler_i2c_read(struct i2c_client *client, uint8_t slave_addr,
			uint8_t reg_offset, uint8_t *read_buf)
{
	struct i2c_msg msgs[1];
	int ret = -1;
	uint8_t regs[5] = {0x51, 0x82, 0x01, 0, 0x7C};
	u8 data[READ_BUF_LEN];
	u8 retry = 5;
	u8 i = 0;

	pr_debug("%s: reading from slave_addr=[%x] and offset=[%x]\n",
		 __func__, slave_addr, reg_offset);

	regs[3] = reg_offset;

	while(i < retry) {
		msgs[0].addr = slave_addr >> 1;
		msgs[0].flags = 0;
		msgs[0].buf = regs;
		msgs[0].len = 5;

		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 1) {
			pr_err("%s: I2C WRITE FAILED=[%d]\n", __func__, ret);
			return -EACCES;
		}

		msleep(100);

		msgs[0].addr = slave_addr >> 1;
		msgs[0].flags = I2C_M_RD;
		msgs[0].buf = &data[0];
		msgs[0].len = READ_BUF_LEN;

		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 1) {
			pr_err("%s: I2C READ FAILED=[%d]\n", __func__, ret);
			return -EACCES;
		}

		pr_info("scaler_i2c_read data = [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]",
			data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10]);

		if(data[0] == 0x6E)
			break;

		i++;
	}

	if(data[0] == 0x6E) {
		memcpy(read_buf, data, READ_BUF_LEN);
		return 0;
	} else {
		return -1;
	}
}

static int scaler_i2c_write(struct i2c_client *client, uint8_t slave_addr,
			uint8_t reg_offset, uint8_t *value)
{
	struct i2c_msg msgs[1];
	uint8_t data[7];
	int status = -EACCES;

	pr_debug("%s: writing from slave_addr=[%x] and offset=[%x]\n",
		 __func__, slave_addr, reg_offset);

	data[0] = 0x51;
	data[1] = 0x84;
	data[2] = 0x03;
	data[3] = reg_offset;
	data[4] = value[1];	/* MSB */
	data[5] = value[0];	/* LSB */
	data[6] = (slave_addr >> 1 ) ^ data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5];	// checksum, xor

	pr_info("scaler_i2c_write data = [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]",
		data[0], data[1], data[2], data[3], data[4], data[5], data[6]);

	msgs[0].addr = slave_addr >> 1;
	msgs[0].flags = 0;
	msgs[0].len = 7;
	msgs[0].buf = data;

	status = i2c_transfer(client->adapter, msgs, 1);

	if (status < 1) {
		pr_err("I2C WRITE FAILED=[%d]\n", status);
		return -EACCES;
	}

	pr_debug("%s: I2C write status=%x\n", __func__, status);
	return status;
}

static ssize_t bl_control_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count) {
	struct scaler_data *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret = -1;

        ret = kstrtouint(buf, 10, &val);
        if (ret)
                return ret;

	pr_info("%s val = %d\n", __func__, val);

	mutex_lock(&data->mutex);
	if(1 == val) {
		if(gpio_is_valid(data->pdata->bl_up_gpio)) {
			gpio_direction_output(data->pdata->bl_up_gpio, 0);
			msleep(50);
			gpio_direction_output(data->pdata->bl_up_gpio, 1);
		}
	} else if(2 == val) {
		if(gpio_is_valid(data->pdata->bl_down_gpio)) {
			gpio_direction_output(data->pdata->bl_down_gpio, 0);
			msleep(50);
			gpio_direction_output(data->pdata->bl_down_gpio, 1);
		}
	}
	mutex_unlock(&data->mutex);

        return count;
}

static ssize_t pa_on_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count) {
	struct scaler_data *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret = -1;

        ret = kstrtouint(buf, 10, &val);
        if (ret)
                return ret;

	pr_info("%s val = %d\n", __func__, val);
	mutex_lock(&data->mutex);
	if(1 == val)
		tfa98xx_set_pa_state(true);
	else if(0 == val)
		tfa98xx_set_pa_state(false);

	mutex_unlock(&data->mutex);
        return count;
}


static ssize_t panel_init_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count) {
	struct scaler_data *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret = -1;

        ret = kstrtouint(buf, 10, &val);
        if (ret)
                return ret;

	pr_info("%s val = %d\n", __func__, val);

	mutex_lock(&data->mutex);
	notify_scaler_to_init_panel(data);
	mutex_unlock(&data->mutex);

        return count;
}

static ssize_t battery_status_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count) {
	struct scaler_data *data = dev_get_drvdata(dev);
	unsigned int capacity;
	int temp;
	int ret = -1;
	const char* sep = ",";
	char* str;
	char* buffer = kmalloc(count, GFP_KERNEL);

	strcpy(buffer, buf);

	str = strsep(&buffer, sep);
	if(!str) {
		ret = -EINVAL;
		goto exit;
	}

        ret = kstrtouint(str, 10, &capacity);
        if (ret)
                goto exit;

	str = strsep(&buffer, sep);
	if(!str) {
		ret = -EINVAL;
		goto exit;
	}

        ret = kstrtoint(str, 10, &temp);
        if (ret)
                goto exit;

	pr_info("%s capacity = %d,  temp = %d\n", __func__, capacity,  temp);

	if((capacity < 0) || (capacity > 100) ){
		pr_err("%s, Wrong value, capacity should be in [0, 100]\n", __func__);
		ret = -1;
		goto exit;
	}

	data->battery_capacity = capacity;
	data->battery_temp = temp < 0 ? TEMP_FROZEN : (temp > 50 ? TEMP_HOT : TEMP_NORMAL);	/* temp < 0: Frozen, temp > 50: Hot */

	mutex_lock(&data->mutex);
	notify_scaler_battery_status(data);
	mutex_unlock(&data->mutex);

	kfree(buffer);
	return count;

exit:
	kfree(buffer);
	return ret;
}

static ssize_t battery_charging_status_show(struct device *dev,
				     struct device_attribute *attr, char *buf) {
	struct scaler_data *data = dev_get_drvdata(dev);
	int ret = -1;

	if(data) {
		return scnprintf(buf, PAGE_SIZE, "%d\n", data->battery_charging);
	} else {
		SCALER_ERROR("data is null\n");
		return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
	}
}

static ssize_t battery_charging_status_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count) {
	struct scaler_data *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret = -1;

        ret = kstrtouint(buf, 10, &val);
        if (ret)
                return ret;

	pr_info("%s battery_charging = %d\n", __func__, val);

	mutex_lock(&data->mutex);
	data->battery_charging = (val == 1);
	notify_scaler_show_osd(data->battery_charging ? BATTERY_CHARGING : BATTERY_NOCHARGE);
	mutex_unlock(&data->mutex);

        return count;
}

/* Map brightness-value to [0, 15] */
static int get_brightness_step(u8 value) {
	u8 i = 0;
	u8 step;

	if((value < 0) || (value > 255)) {
		SCALER_ERROR("wrong value.");
		return -1;
	}

	if(value < brightness_mapping_table[0])
		return 0;

	if(value >= brightness_mapping_table[MAX_BRIGHTNESS_STEPS - 1])
		return 15;

	for(i = 0; i < MAX_BRIGHTNESS_STEPS - 1; i++) {
		if((value >= brightness_mapping_table[i]) && (value < brightness_mapping_table[i + 1])) {
			step = i + 1;
			break;
		}
	}

	return step;
}

static ssize_t brightness_volume_show(struct device *dev,
				     struct device_attribute *attr, char *buf) {
	struct scaler_data *data = dev_get_drvdata(dev);
	int ret = -1;

	if(data) {
		return scnprintf(buf, PAGE_SIZE, "brightness = %d, volume_level = %d\n", data->brightness, data->volume_level);
	} else {
		SCALER_ERROR("data is null\n");
		return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
	}
}

static ssize_t brightness_volume_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count) {
	struct scaler_data *data = dev_get_drvdata(dev);
	unsigned int brightness, volume;
	int ret = -1;
	const char* sep = ",";
	char* str;
	char* buffer = kmalloc(count, GFP_KERNEL);

	strcpy(buffer, buf);

	str = strsep(&buffer, sep);
	if(!str) {
		ret = -EINVAL;
		goto exit;
	}

        ret = kstrtouint(str, 10, &brightness);
        if (ret)
                goto exit;

	str = strsep(&buffer, sep);
	if(!str) {
		ret = -EINVAL;
		goto exit;
	}

        ret = kstrtouint(str, 10, &volume);
        if (ret)
                goto exit;

	pr_info("%s brightness = %d,  volume = %d\n", __func__, brightness,  volume);

	if(((brightness < 0) || (brightness > 255)) || ((volume < 0) || (volume > 15))){
		pr_err("%s, Wrong value, brightness should be in [0, 255], volume should be in [0, 15]\n", __func__);
		ret = -1;
		goto exit;
	}

	data->brightness = brightness;
	data->volume_level = volume;

	mutex_lock(&data->mutex);
	data->brightness_step = get_brightness_step(brightness);
	notify_scaler_brightness_and_volume(data);
	mutex_unlock(&data->mutex);

	kfree(buffer);
	return count;

exit:
	kfree(buffer);
	return ret;
}

static ssize_t scaler_connection_status_show(struct device *dev,
				     struct device_attribute *attr, char *buf) {
	struct scaler_data *data = dev_get_drvdata(dev);
	int ret = -1;

	return scnprintf(buf, PAGE_SIZE, "%d\n", data ? data->connection_status: ret);
}

static ssize_t scaler_regs_show(struct device *dev,
				     struct device_attribute *attr, char *buf) {
	struct scaler_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	u8 values[READ_BUF_LEN];
	int ret = -1;

	if(!data) {
		SCALER_ERROR("data is null\n");
		goto exit;
	}

	ret = scaler_i2c_read(client, SCALER_I2C_ADDR, GET_STATUS_REG, values);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to read reg 0xB1 : %d\n", __func__, ret);
		goto exit;
	}

	SCALER_INFO("values = [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]",
		values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9], values[10]);

	return scnprintf(buf, PAGE_SIZE, "values = [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]\n",
		values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9], values[10]);
exit:
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret );
}

static ssize_t osd_show_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count) {
	struct scaler_data *data = dev_get_drvdata(dev);
	unsigned int type;
	int ret = -1;

        ret = kstrtouint(buf, 10, &type);
        if (ret)
                return ret;

	pr_info("%s type = %d\n", __func__, type);

	if(type < MAX_OSD_TYPE) {
		mutex_lock(&data->mutex);
		notify_scaler_show_osd(type);
		mutex_unlock(&data->mutex);
	} else
		return ret;

        return count;
}

static ssize_t scaler_hdcp_key_status_show(struct device *dev,
				     struct device_attribute *attr, char *buf) {
	struct scaler_data *data = dev_get_drvdata(dev);
	int ret = -1;

	if(data) {
		return scnprintf(buf, PAGE_SIZE, "%d\n", data->hdcp_key_status);
	} else {
		SCALER_ERROR("data is null\n");
		return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
	}
}

static ssize_t scaler_fw_version_show(struct device *dev,
				     struct device_attribute *attr, char *buf) {
	struct scaler_data *data = dev_get_drvdata(dev);
	int ret = -1;

	if(data) {
		return scnprintf(buf, PAGE_SIZE, "%d\n", data->fw_version);
	} else {
		SCALER_ERROR("data is null\n");
		return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
	}
}

static ssize_t request_switch_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count) {
	struct scaler_data *data = dev_get_drvdata(dev);
	unsigned int val;
	int ret = -1;

        ret = kstrtouint(buf, 10, &val);
        if (ret)
                return ret;

	pr_info("%s request switch = %d\n", __func__, val);

	mutex_lock(&data->mutex);
	if(val)
		schedule_work(&data->hpd_irq_work);
	mutex_unlock(&data->mutex);

        return count;
}

static ssize_t cable_connected_show(struct device *dev,
				     struct device_attribute *attr, char *buf) {
	struct scaler_data *data = dev_get_drvdata(dev);
	int ret = -1;

	if(data) {
		return scnprintf(buf, PAGE_SIZE, "%d\n", data->cable_connected);
	} else {
		SCALER_ERROR("data is null\n");
		return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
	}
}

static DEVICE_ATTR(connection_status, 0644, scaler_connection_status_show, NULL);
static DEVICE_ATTR(regs, 0644, scaler_regs_show, NULL);
static DEVICE_ATTR(brightness_volume, 0664, brightness_volume_show, brightness_volume_store);
static DEVICE_ATTR(battery_status, 0664, NULL, battery_status_store);
static DEVICE_ATTR(battery_charging, 0664, battery_charging_status_show, battery_charging_status_store);
static DEVICE_ATTR(init, 0664, NULL, panel_init_store);
static DEVICE_ATTR(pa_on, 0664, NULL, pa_on_store);
static DEVICE_ATTR(bl_control, 0664, NULL, bl_control_store);
static DEVICE_ATTR(osd_show, 0664, NULL, osd_show_store);
static DEVICE_ATTR(fw_version, 0644, scaler_fw_version_show, NULL);
static DEVICE_ATTR(hdcp_key_status, 0644, scaler_hdcp_key_status_show, NULL);
static DEVICE_ATTR(request_switch, 0664, NULL, request_switch_store);
static DEVICE_ATTR(cable_connected, 0644, cable_connected_show, NULL);


static struct attribute *scaler_attributes[] = {
	&dev_attr_connection_status.attr,
	&dev_attr_regs.attr,
	&dev_attr_brightness_volume.attr,
	&dev_attr_battery_status.attr,
	&dev_attr_battery_charging.attr,
	&dev_attr_init.attr,
	&dev_attr_pa_on.attr,
	&dev_attr_bl_control.attr,
	&dev_attr_osd_show.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_hdcp_key_status.attr,
	&dev_attr_request_switch.attr,
	&dev_attr_cable_connected.attr,
	NULL,
};

static const struct attribute_group scaler_attr_group = {
	.attrs = scaler_attributes,
};

enum CONNECTION_STATUS get_connection_status() {
	return g_data ? g_data->connection_status : -1;
}

static int update_scaler_status(struct scaler_data *data) {
	struct i2c_client *client = data->client;
	u8 values[READ_BUF_LEN];
	int ret = 0;

	ret = scaler_i2c_read(client, SCALER_I2C_ADDR, GET_STATUS_REG, values);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to read reg 0xB1 : %d\n", __func__, ret);
		return ret;
	}

	pr_info("%s, values = [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]", __func__,
		values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9], values[10]);

	data->reg_status.request_PA_on = values[6];
	data->reg_status.scaler_init_status = values[2];
	data->reg_status.request_scaler_off = values[3];

	data->brightness_step = values[4];	/* Need to restore brightness and volume-level from tv-mode. */
	data->volume_level = values[5];

	data->hdcp_key_status = (values[7] == 2) ? PASS : FAIL;
	data->fw_version = values[8];

	return 0;
}

/* Notify scaler to config panel and dp-mux. */
static int notify_scaler_to_init_panel(struct scaler_data *data) {
	struct i2c_client *client = data->client;
	uint8_t value[2] = {1, 0};		// {LSB, MSB}----ori
	int ret;

	pr_info("%s\n", __func__);
	ret = scaler_i2c_write(client, SCALER_I2C_ADDR, PANEL_INIT_REG, value);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to set reg 0x%x : %d\n", __func__, PANEL_INIT_REG, ret);
		return ret;
	}

	return 0;
}

/* Nofity scaler the initial brightness and volume. */
static int notify_scaler_brightness_and_volume(struct scaler_data *data) {
	struct i2c_client *client = data->client;
	uint8_t value[2];
	int ret;

	pr_info("%s\n", __func__);

	value[0] = data->volume_level;	// LSB
	value[1] = data->brightness_step;		// MSB

	ret = scaler_i2c_write(client, SCALER_I2C_ADDR, INITIAL_STATUS_REG, value);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to set reg 0x%x : %d\n", __func__, INITIAL_STATUS_REG, ret);
		return ret;
	}

	return 0;
}

static int notify_scaler_battery_status(struct scaler_data *data) {
	struct i2c_client *client = data->client;
	uint8_t value[2];
	int ret;

	pr_info("%s\n", __func__);

	value[0] = data->battery_temp;			// LSB
	value[1] = data->battery_capacity;		// MSB

	ret = scaler_i2c_write(client, SCALER_I2C_ADDR, BATTERY_STATUS_REG, value);	/* Notify scaler the battery status, such as capacity, battery. */
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to set reg 0x%x : %d\n", __func__, BATTERY_STATUS_REG, ret);
		return ret;
	}

	return 0;
}

int notify_scaler_show_osd(enum OSD_TYPE type) {
	struct i2c_client *client;
	uint8_t value[2] = {1, 0};		// {LSB, MSB}
	int ret;

	pr_info("%s, type = %d\n", __func__, type);

	if(!g_data) {
		pr_err("g_data = NULL\n");
		return -1;
	}

	value[0] = type;
	client = g_data->client;
	ret = scaler_i2c_write(client, SCALER_I2C_ADDR, SHOW_OSD_REG, value);	/* Notify scaler to show the OSD. */
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to set reg 0x%x : %d\n", __func__, SHOW_OSD_REG, ret);
		return ret;
	}

	return 0;
}

int notify_scaler_backlight_up_or_down(bool up, bool long_press) {
	int i, j = 0;
	int ret = 0;
	u8 values[READ_BUF_LEN];
	pr_info("%s\n", __func__);

	if(!g_data) {
		pr_err("g_data = NULL\n");
		return -1;
	}

	pm_stay_awake(&g_data->client->dev);
	mutex_lock(&g_data->mutex);
	pr_info("xxxx %s\n", __func__);

	if(up) {
		if(gpio_is_valid(g_data->pdata->bl_up_gpio)) {
			if (long_press) {
				do {
					SCALER_INFO("brightness: %d\n", g_data->brightness_step);
					for (i = 0; i < MAX_BRIGHTNESS_STEPS - 1 - g_data->brightness_step; i++) {
						gpio_direction_output(g_data->pdata->bl_up_gpio, 0);
						mdelay(20);
						gpio_direction_output(g_data->pdata->bl_up_gpio, 1);
						mdelay(20);
					}
					ret = scaler_i2c_read(g_data->client, SCALER_I2C_ADDR, GET_STATUS_REG, values);
					if (ret < 0) {
						dev_err(&g_data->client->dev, "%s: failed to read reg 0xB1 : %d\n", __func__, ret);
						goto exit;
					}
					g_data->brightness_step = values[4];
					if (g_data->brightness_step > MAX_BRIGHTNESS_STEPS - 1) {
						g_data->brightness_step = MAX_BRIGHTNESS_STEPS - 1;
						goto exit;
					}

					if (j++ > MAX_TRY_FOR_LONG_PRESS) {
						SCALER_INFO("fail to set brightness after trying 3 times: %d\n", g_data->brightness_step);
						goto exit;
					}
				} while (g_data->brightness_step != MAX_BRIGHTNESS_STEPS - 1);
			} else {
				gpio_direction_output(g_data->pdata->bl_up_gpio, 0);
				mdelay(20);
				gpio_direction_output(g_data->pdata->bl_up_gpio, 1);
				if (g_data->brightness_step < MAX_BRIGHTNESS_STEPS - 1)
					g_data->brightness_step++;
			}
		}
	} else  {
		if(gpio_is_valid(g_data->pdata->bl_down_gpio)) {
			if (long_press) {
				do {
					SCALER_INFO("brightness: %d\n", g_data->brightness_step);
					for (i = 0; i < g_data->brightness_step; i++) {
						gpio_direction_output(g_data->pdata->bl_down_gpio, 0);
						mdelay(20);
						gpio_direction_output(g_data->pdata->bl_down_gpio, 1);
						mdelay(20);
					}
					ret = scaler_i2c_read(g_data->client, SCALER_I2C_ADDR, GET_STATUS_REG, values);
					if (ret < 0) {
						dev_err(&g_data->client->dev, "%s: failed to read reg 0xB1 : %d\n", __func__, ret);
						goto exit;
					}

					g_data->brightness_step = values[4];
					if (j++ > MAX_TRY_FOR_LONG_PRESS) {
						SCALER_INFO("fail to set brightness after trying 3 times: %d\n", g_data->brightness_step);
						goto exit;
					}
				} while (g_data->brightness_step != 0);
			} else {
				gpio_direction_output(g_data->pdata->bl_down_gpio, 0);
				mdelay(20);
				gpio_direction_output(g_data->pdata->bl_down_gpio, 1);
				if (g_data->brightness_step > 0)
					g_data->brightness_step--;
			}
		}
	}

exit:
	mutex_unlock(&g_data->mutex);
	pm_relax(&g_data->client->dev);

	return ret;
}

static void touch_irq_work(struct work_struct *work) {
    int x, y, ret;
    bool long_press;
	struct scaler_data *data = container_of(work,
			struct scaler_data, touch_irq_work);

    x = data->touch_info.x;
    y = data->touch_info.y;
    long_press = data->touch_info.long_press;

    if ((x >= TOUCH_VOL_UP_LEFT) && (x <= TOUCH_VOL_UP_RIGHT) && (y >= TOUCH_VOL_UP_TOP)) {
		ret = notify_scaler_backlight_up_or_down(true, long_press);
		if(ret)
			SCALER_INFO("failed to notify scaler to turn backlight up or down\n");
	} else if ((x >= TOUCH_VOL_DOWN_LEFT) && (x <= TOUCH_VOL_DOWN_RIGHT) && (y >= TOUCH_VOL_DOWN_TOP)) {
		ret = notify_scaler_backlight_up_or_down(false, long_press);
		if(ret)
			SCALER_INFO("failed to notify scaler to turn backlight up or down\n");
	} else {
		ret = notify_scaler_show_osd(BRIGHTNESS_ON);	// toggle osd on/off
		if(ret)
			SCALER_INFO("failed to notify scaler to show osd\n");
	}
}

int notify_scaler_touch_point(int x, int y, bool long_press) {
	int ret = true;

    g_data->touch_info.x = x;
    g_data->touch_info.y = y;
    g_data->touch_info.long_press = long_press;
	schedule_work(&g_data->touch_irq_work);

	return ret;
}

static int scaler_power_on(struct scaler_data *data) {
	struct scaler_pdata *pdata = data->pdata;
	struct i2c_client *client = data->client;

	int ret = 0;
	SCALER_INFO("enter %s, %d\n", __func__, __LINE__);

	if (gpio_is_valid(pdata->vcc3v3_on_gpio)) {
		ret = gpio_direction_output(pdata->vcc3v3_on_gpio, 1);
		if (ret) {
			dev_err(&client->dev, "Fail to output 1 for vcc3v3_on gpio [%d]\n", pdata->vcc3v3_on_gpio);
			return ret;
		}
	} else {
		SCALER_ERROR("vcc3v3_on_gpio invalid\n");
	}

	if (gpio_is_valid(pdata->vdd_1v2_gpio)) {
		ret = gpio_direction_output(pdata->vdd_1v2_gpio, 1);
		if (ret) {
			dev_err(&client->dev, "Fail to output 1 for vdd_1v2 gpio [%d]\n", pdata->vdd_1v2_gpio);
			return ret;
		}
	} else {
		SCALER_ERROR("vdd_1v2_gpio invalid\n");
	}

	mdelay(2);

	if (gpio_is_valid(pdata->v11s_on_gpio)) {
		ret = gpio_direction_output(pdata->v11s_on_gpio, 1);
		if (ret) {
			dev_err(&client->dev, "Fail to output 1 for v11s_on gpio [%d]\n", pdata->v11s_on_gpio);
			return ret;
		}
	} else {
		SCALER_ERROR("v11s_on_gpio invalid\n");
	}

	mdelay(1);

	if (gpio_is_valid(pdata->sdram_en_gpio)) {
		ret = gpio_direction_output(pdata->sdram_en_gpio, 1);
		if (ret) {
			dev_err(&client->dev, "Fail to output 1 for sdram_en gpio [%d]\n", pdata->sdram_en_gpio);
			return ret;
		}
	} else {
		SCALER_ERROR("sdram_en_gpio invalid\n");
	}


	return ret;
}

static int scaler_power_off(struct scaler_data *data) {
	struct scaler_pdata *pdata = data->pdata;
	struct i2c_client *client = data->client;
	int ret = 0;

	SCALER_INFO("enter %s, %d\n", __func__, __LINE__);

	if (gpio_is_valid(pdata->sdram_en_gpio)) {
		ret = gpio_direction_output(pdata->sdram_en_gpio, 0);
		if (ret) {
			dev_err(&client->dev, "Fail to output 0 for sdram_en gpio [%d]\n", pdata->sdram_en_gpio);
			return ret;
		}
	} else {
		SCALER_ERROR("sdram_en_gpio invalid\n");
	}

	if (gpio_is_valid(pdata->v11s_on_gpio)) {
		ret = gpio_direction_output(pdata->v11s_on_gpio, 0);
		if (ret) {
			dev_err(&client->dev, "Fail to output 0 for v11s_on gpio [%d]\n", pdata->v11s_on_gpio);
			return ret;
		}
	} else {
		SCALER_ERROR("v11s_on_gpio invalid\n");
	}

	if (gpio_is_valid(pdata->vdd_1v2_gpio)) {
		ret = gpio_direction_output(pdata->vdd_1v2_gpio, 0);
		if (ret) {
			dev_err(&client->dev, "Fail to output 0 for vdd_1v2 gpio [%d]\n", pdata->vdd_1v2_gpio);
			return ret;
		}
	} else {
		SCALER_ERROR("vdd_1v2_gpio invalid\n");
	}

	if (gpio_is_valid(pdata->vcc3v3_on_gpio)) {
		ret = gpio_direction_output(pdata->vcc3v3_on_gpio, 0);
		if (ret) {
			dev_err(&client->dev, "Fail to output 0 for vcc3v3_on gpio [%d]\n", pdata->vcc3v3_on_gpio);
			return ret;
		}
	} else {
		SCALER_ERROR("vcc3v3_on_gpio invalid\n");
	}

	return 0;
}

static int scaler_power_init(struct scaler_data *data) {
	return 0;
}

static int scaler_power_deinit(struct scaler_data *data) {
	return 0;
}

static int scaler_request_gpio(struct scaler_data *data)
{
	struct i2c_client *client = data->client;
	struct scaler_pdata *pdata = data->pdata;
	int ret;

	if (gpio_is_valid(pdata->hpd_irq_gpio)) {		/* gpio8*/
		ret = gpio_request_one(pdata->hpd_irq_gpio, GPIOF_DIR_IN, "hpd-irq");
		if (ret) {
			pdata->hpd_irq_gpio = -1;
			dev_err(&client->dev, "Fail to request hpd_irq gpio [%d]\n",
				pdata->hpd_irq_gpio);
			goto exit;
		}
	} else {
		dev_err(&client->dev, "Invalid hpd_irq gpio [%d]!\n", pdata->hpd_irq_gpio);
		ret = -EINVAL;
		goto exit;
	}

	if (gpio_is_valid(pdata->scaler_irq_gpio)) {		/* gpio93*/
		ret = gpio_request_one(pdata->scaler_irq_gpio, GPIOF_DIR_IN, "scaler_irq");
		if (ret) {
			pdata->scaler_irq_gpio = -1;
			dev_err(&client->dev, "Fail to request scaler_irq gpio [%d]\n",
				pdata->scaler_irq_gpio);
			goto exit_free_hpd_irq_gpio;
		}
	} else {
		dev_err(&client->dev, "Invalid scaler_irq gpio [%d]!\n", pdata->scaler_irq_gpio);
		ret = -EINVAL;
		goto exit_free_hpd_irq_gpio;
	}

	if (gpio_is_valid(pdata->bl_up_gpio)) {			/* gpio99*/
		ret = gpio_request(pdata->bl_up_gpio, "scaler_bl_up");
		if (ret) {
			pdata->bl_up_gpio = -1;
			dev_err(&client->dev, "Fail to request bl_up gpio [%d]\n",
				pdata->bl_up_gpio);
			goto exit_free_scaler_irq_gpio;
		} else {
			ret = gpio_direction_input(pdata->bl_up_gpio);
			if (ret) {
				pr_err("Fail to set bl_up gpio as input, ret = %d\n", ret);
				//return ret;
			}
		}
	} else {
		dev_err(&client->dev, "Invalid bl_up gpio [%d]!\n", pdata->bl_up_gpio);
		ret = -EINVAL;
		goto exit_free_scaler_irq_gpio;
	}

	if (gpio_is_valid(pdata->bl_down_gpio)) {		/* gpio82 */
		ret = gpio_request(pdata->bl_down_gpio, "scaler_bl_down");
		if (ret) {
			pdata->bl_down_gpio = -1;
			dev_err(&client->dev, "Fail to request bl_down gpio [%d]\n",
				pdata->bl_down_gpio);
			goto exit_free_bl_up_gpio;
		} else {
			ret = gpio_direction_input(pdata->bl_down_gpio);
			if (ret) {
				pr_err("Fail to set bl_down gpio as input, ret = %d\n", ret);
				//return ret;
			}
		}
	} else {
		dev_err(&client->dev, "Invalid bl_down gpio [%d]!\n", pdata->bl_down_gpio);
		ret = -EINVAL;
		goto exit_free_bl_up_gpio;
	}

	if (gpio_is_valid(pdata->fm_update_gpio)) {	/* gpio90 */
		ret = gpio_request_one(pdata->fm_update_gpio, GPIOF_DIR_OUT | GPIOF_EXPORT, "scaler_fm_update");
		if (ret) {
			dev_err(&client->dev, "Fail to request fm_update gpio [%d]\n",
				pdata->fm_update_gpio);
			pdata->fm_update_gpio = -1;
			goto exit_free_bl_down_gpio;
		}
	} else {
		dev_err(&client->dev, "Invalid fm_update gpio [%d]!\n", pdata->fm_update_gpio);
		ret = -EINVAL;
		goto exit_free_bl_down_gpio;
	}

	if (gpio_is_valid(pdata->sdram_en_gpio)) {		/* gpio9 */
		ret = gpio_request_one(pdata->sdram_en_gpio, GPIOF_DIR_OUT | GPIOF_EXPORT, "scaler_sdram_en");
		if (ret) {
			pdata->sdram_en_gpio = -1;
			dev_err(&client->dev, "Fail to request sdram_en gpio [%d]\n",
				pdata->sdram_en_gpio);
			goto exit_free_fm_update_gpio;
		}
	} else {
		dev_err(&client->dev, "Invalid sdram_en gpio [%d]!\n", pdata->sdram_en_gpio);
		ret = -EINVAL;
		goto exit_free_fm_update_gpio;
	}

	if (gpio_is_valid(pdata->vcc3v3_on_gpio)) {	/* gpio97 */
		ret = gpio_request_one(pdata->vcc3v3_on_gpio, GPIOF_DIR_OUT | GPIOF_EXPORT, "scaler_vcc3v3_on");
		if (ret) {
			pdata->vcc3v3_on_gpio = -1;
			dev_err(&client->dev, "Fail to request vcc3v3_on gpio [%d]\n",
				pdata->vcc3v3_on_gpio);
			goto exit_free_sdram_en_gpio;
		}
	} else {
		dev_err(&client->dev, "Invalid vcc3v3_on gpio [%d]!\n", pdata->vcc3v3_on_gpio);
		ret = -EINVAL;
		goto exit_free_sdram_en_gpio;
	}

	if (gpio_is_valid(pdata->vdd_1v2_gpio)) {		/* gpio42 */
		ret = gpio_request_one(pdata->vdd_1v2_gpio, GPIOF_DIR_OUT | GPIOF_EXPORT, "scaler_vdd1v2_en");
		if (ret) {
			pdata->vdd_1v2_gpio = -1;
			dev_err(&client->dev, "Fail to request vdd_1v2 gpio [%d]\n", pdata->vdd_1v2_gpio);
			goto exit_free_vcc3v3_on_gpio;
		}
	} else {
		dev_err(&client->dev, "Invalid vdd_1v2 gpio [%d]!\n", pdata->vdd_1v2_gpio);
		ret = -EINVAL;
		goto exit_free_vcc3v3_on_gpio;
	}

	if (gpio_is_valid(pdata->v11s_on_gpio)) {		/* gpio92 */
		ret = gpio_request_one(pdata->v11s_on_gpio, GPIOF_DIR_OUT | GPIOF_EXPORT, "scaler_v11s_on");
		if (ret) {
			dev_err(&client->dev, "Fail to request v11s_on gpio [%d]\n", pdata->v11s_on_gpio);
			pdata->v11s_on_gpio = -1;
			goto exit_free_vdd_1v2_gpio;
		}
	} else {
		dev_err(&client->dev, "Invalid v11s_on gpio [%d]!\n", pdata->v11s_on_gpio);
		ret = -EINVAL;
		goto exit_free_vdd_1v2_gpio;
	}

	return 0;

exit_free_vdd_1v2_gpio:
	if (gpio_is_valid(pdata->vdd_1v2_gpio))
		gpio_free(pdata->vdd_1v2_gpio);

exit_free_vcc3v3_on_gpio:
	if (gpio_is_valid(pdata->vcc3v3_on_gpio))
		gpio_free(pdata->vcc3v3_on_gpio);

exit_free_sdram_en_gpio:
	if (gpio_is_valid(pdata->sdram_en_gpio))
		gpio_free(pdata->sdram_en_gpio);

exit_free_fm_update_gpio:
	if (gpio_is_valid(pdata->fm_update_gpio))
		gpio_free(pdata->fm_update_gpio);

exit_free_bl_down_gpio:
	if (gpio_is_valid(pdata->bl_down_gpio))
		gpio_free(pdata->bl_down_gpio);

exit_free_bl_up_gpio:
	if (gpio_is_valid(pdata->bl_up_gpio))
		gpio_free(pdata->bl_up_gpio);

exit_free_scaler_irq_gpio:
	if (gpio_is_valid(pdata->scaler_irq_gpio))
		gpio_free(pdata->scaler_irq_gpio);

exit_free_hpd_irq_gpio:
	if (gpio_is_valid(pdata->hpd_irq_gpio))
		gpio_free(pdata->hpd_irq_gpio);

exit:
	return ret;
}


static void scaler_free_gpio(struct scaler_pdata *pdata) {
	if (gpio_is_valid(pdata->hpd_irq_gpio))
		gpio_free(pdata->hpd_irq_gpio);
	if (gpio_is_valid(pdata->scaler_irq_gpio))
		gpio_free(pdata->scaler_irq_gpio);
	if (gpio_is_valid(pdata->bl_up_gpio))
		gpio_free(pdata->bl_up_gpio);
	if (gpio_is_valid(pdata->bl_down_gpio))
		gpio_free(pdata->bl_down_gpio);
	if (gpio_is_valid(pdata->fm_update_gpio))
		gpio_free(pdata->fm_update_gpio);
	if (gpio_is_valid(pdata->sdram_en_gpio))
		gpio_free(pdata->sdram_en_gpio);
	if (gpio_is_valid(pdata->vcc3v3_on_gpio))
		gpio_free(pdata->vcc3v3_on_gpio);
	if (gpio_is_valid(pdata->vdd_1v2_gpio))
		gpio_free(pdata->vdd_1v2_gpio);
	if (gpio_is_valid(pdata->v11s_on_gpio))
		gpio_free(pdata->v11s_on_gpio);
}

static int scaler_parse_dt(struct device *dev,
			struct scaler_pdata *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->hpd_irq_gpio = of_get_named_gpio(np, "scaler,hpd-irq", 0);
	if (!gpio_is_valid(pdata->hpd_irq_gpio)) {
		SCALER_ERROR("hpd_irq gpio is not specified.");
		return -ENODEV;
	}

	pdata->scaler_irq_gpio = of_get_named_gpio(np, "scaler,scaler-irq", 0);
	if (!gpio_is_valid(pdata->scaler_irq_gpio)) {
		SCALER_ERROR("scaler_irq gpio is not specified.");
		return -ENODEV;
	}

	pdata->bl_up_gpio = of_get_named_gpio(np, "scaler,bl-up", 0);
	if (!gpio_is_valid(pdata->bl_up_gpio)) {
		SCALER_ERROR("bl_up gpio is not specified.");
		return -ENODEV;
	}

	pdata->bl_down_gpio = of_get_named_gpio(np, "scaler,bl-down", 0);
	if (!gpio_is_valid(pdata->bl_down_gpio)) {
		SCALER_ERROR("bl_down gpio is not specified.");
		return -ENODEV;
	}

	pdata->fm_update_gpio = of_get_named_gpio(np, "scaler,fm-update", 0);
	if (!gpio_is_valid(pdata->fm_update_gpio)) {
		SCALER_ERROR("fm_update gpio is not specified.");
		return -ENODEV;
	}

	pdata->sdram_en_gpio = of_get_named_gpio(np, "scaler,sdram-en", 0);
	if (!gpio_is_valid(pdata->sdram_en_gpio)) {
		SCALER_ERROR("sdram_en gpio is not specified.");
		return -ENODEV;
	}

	pdata->vcc3v3_on_gpio = of_get_named_gpio(np, "scaler,vcc3v3-on", 0);
	if (!gpio_is_valid(pdata->vcc3v3_on_gpio)) {
		SCALER_ERROR("vcc3v3_on gpio is not specified.");
		return -ENODEV;
	}

	pdata->vdd_1v2_gpio = of_get_named_gpio(np, "scaler,vdd-1v2-en", 0);
	if (!gpio_is_valid(pdata->vdd_1v2_gpio)) {
		SCALER_ERROR("vdd_1v2 gpio is not specified.");
		return -ENODEV;
	}

	pdata->v11s_on_gpio = of_get_named_gpio(np, "scaler,v11s-on-en", 0);
	if (!gpio_is_valid(pdata->v11s_on_gpio)) {
		SCALER_ERROR("v11s_on gpio is not specified.");
		return -ENODEV;
	}

	return 0;
}

static int scaler_config_input(struct scaler_data *data)
{
	struct i2c_client *client = data->client;
	int ret;

	data->input_dev = devm_input_allocate_device(&client->dev);
	if (!data->input_dev)
		return -ENOMEM;

	data->input_dev->name = "scaler";
	data->input_dev->phys = "scaler/input0";

	input_set_capability(data->input_dev, EV_KEY, KEY_POWER);

	ret =  input_register_device(data->input_dev);
	if (ret < 0) {
		SCALER_ERROR("Unable to register input device");
		input_free_device(data->input_dev);
		return ret;
	}

	return 0;
}
static void scaler_notify_service(struct scaler_data *data, u8 status)
{
	char* disconnected[2] = { "SCALER_STATE=DISCONNECTED", NULL };
	char* connecting[2] = { "SCALER_STATE=CONNECTING", NULL };
	char* connected[2] = { "SCALER_STATE=CONNECTED", NULL };
	struct device *scaler_dev = data->scaler_dev;

	SCALER_INFO("scaler_notify_service: send uevent to FW. status = %d", status);

	switch (status) {
		case DISCONNECTED:
			kobject_uevent_env(&scaler_dev->kobj, KOBJ_CHANGE, disconnected);
			break;
		case CONNECTING:
			kobject_uevent_env(&scaler_dev->kobj, KOBJ_CHANGE, connecting);
			break;
		case CONNECTED:
			kobject_uevent_env(&scaler_dev->kobj, KOBJ_CHANGE, connected);
			break;
		default:
			SCALER_INFO("scaler_notify_service, status = %d", status);
			break;
	}
}

static void update_status_delayed_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct scaler_data *data = container_of(dwork,
			struct scaler_data, update_status_work);
	int ret;
#ifdef CONFIG_HW_BOOT_INFO
	char str[64];
#endif

	SCALER_INFO("update_status_delayed_work");
	ret = update_scaler_status(data);
	if(ret)
		pr_err("update scaler status failed.\n");
	else {
		pr_info("fw_version = %d, hdcp_key_status = %d\n", data->fw_version, data->hdcp_key_status);

#ifdef CONFIG_HW_BOOT_INFO
		snprintf(str, 10, "0x%x", data->fw_version);
		register_hardware_info("Scaler_FW_Version", str);

		switch(data->hdcp_key_status) {
			case 0:
				register_hardware_info("Scaler_HDCP_Key", "Unkonwn");
				break;
			case 1:
				register_hardware_info("Scaler_HDCP_Key", "Fail");
				break;
			case 2:
				register_hardware_info("Scaler_HDCP_Key", "Pass");
				break;
			default:
				register_hardware_info("Scaler_HDCP_Key", "Unkonwn");
				break;
		}
#endif
	}

	scaler_power_off(data);
}

static void hpd_irq_work(struct work_struct *work)
{
	struct scaler_data *data = container_of(work,
			struct scaler_data, hpd_irq_work);
	struct scaler_pdata *pdata = data->pdata;
	u8 value;

	SCALER_INFO("hpd_irq_work.");

	value = gpio_get_value(pdata->hpd_irq_gpio);
	if(data->hpd_gpio_status != value) {
		SCALER_ERROR("Need a stable state in debouce time.\n");
		return;
	}

	if(!gpio_get_value(pdata->hpd_irq_gpio)) {	// plug in
		scaler_power_init(data);
		scaler_power_on(data);
	} else {	// plug out
		SCALER_INFO("hdmi cable removed");
//		tfa98xx_set_pa_state(false);
	}
}

static void cable_detection_delayed_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct scaler_data *data = container_of(dwork,
			struct scaler_data, cable_detection_work);
	struct scaler_pdata *pdata = data->pdata;
	u8 value;
	int ret;

	SCALER_INFO("cable_detection_delayed_work.");

	value = gpio_get_value(pdata->hpd_irq_gpio);
	if(data->hpd_gpio_status != value) {
		SCALER_ERROR("Need a stable state in debouce time.\n");
		return;
	}

	data->cable_connected = value ?  false : true;
	SCALER_INFO("cable_connected = %d.", data->cable_connected);

	if(data->cable_connected) {
		if(gpio_is_valid(data->pdata->bl_up_gpio)) {
			ret = gpio_direction_output(pdata->bl_up_gpio, 1);
			if (ret) {
				pr_err("Fail to output bl_up gpio 1, ret = %d\n", ret);
			}
		}

		if(gpio_is_valid(data->pdata->bl_down_gpio)) {
			ret = gpio_direction_output(pdata->bl_down_gpio, 1);
			if (ret) {
				pr_err("Fail to output bl_down gpio 1, ret = %d\n", ret);
			}
		}

		data->connection_status = CONNECTING;
		scaler_notify_service(data, CONNECTING);
	} else {
		if(gpio_is_valid(data->pdata->bl_up_gpio)) {
			ret = gpio_direction_input(pdata->bl_up_gpio);
			if (ret) {
				pr_err("Fail to input bl_up gpio 1, ret = %d\n", ret);
			}
		}

		if(gpio_is_valid(data->pdata->bl_down_gpio)) {
			ret = gpio_direction_input(pdata->bl_down_gpio);
			if (ret) {
				pr_err("Fail to input bl_down gpio 1, ret = %d\n", ret);
			}
		}

		if(data->reg_status.scaler_init_status != SCALER_ACTIVE) {
			/* If scaler_init_status == SCALER_ACTIVE, need to wait until scaler deinit panel. */
			scaler_power_off(data);
			scaler_power_deinit(data);

			data->connection_status = DISCONNECTED;
			scaler_notify_service(data, DISCONNECTED);
		}
	}
}

static irqreturn_t hpd_irq_thread(int irq, void *dev_id)
{
	struct scaler_data *data = dev_id;
	struct scaler_pdata *pdata = data->pdata;

	SCALER_INFO("hpd_irq_thread\n");

	pm_stay_awake(&data->client->dev);

	/* Recheck the cable status in 200ms. */
	data->hpd_gpio_status = gpio_get_value(pdata->hpd_irq_gpio);

	schedule_delayed_work(&data->cable_detection_work, msecs_to_jiffies(200));	/* debounce time increace to 200ms requested by hw.*/

	return IRQ_HANDLED;
}

static void scaler_irq_work(struct work_struct *work)
{
	struct scaler_data *data = container_of(work,
			struct scaler_data, scaler_irq_work);
	struct scaler_pdata *pdata = data->pdata;
	int ret;

	SCALER_INFO("scaler_irq_work.");

	if(data != NULL) {
		if(gpio_get_value(pdata->hpd_irq_gpio)) {	// already plug out
			SCALER_INFO("Cable removed. Current data->connection_status = %d", data->connection_status);
			if(data->connection_status == CONNECTING || data->connection_status == CONNECTED) {	// cable inserted before
				mdelay(150);

				/* hdmi cable plug out, check if scaler-ic had deinited panel or not */
				ret = update_scaler_status( data);
				if(!ret) {
					if(1 == data->reg_status.request_scaler_off) {	/* scaler deinit panel done, request for  */
						SCALER_INFO("plug out, scaler deinit panel done.");

						SCALER_INFO("turn off PA");
						tfa98xx_set_pa_state(false);

						/* turn off scaler */
						scaler_power_off(data);
						scaler_power_deinit(data);

						/* notify service */
						data->connection_status = DISCONNECTED;
						scaler_notify_service(data, DISCONNECTED);
					}
				}
			}
		} else {
			mdelay(150);
			ret = update_scaler_status(data);
			if(!ret) {
				SCALER_INFO("Cable in. connection_status = %d, scaler_init_status = %d", data->connection_status, data->reg_status.scaler_init_status);
				if(data->connection_status == CONNECTING) {
					if(data->reg_status.scaler_init_status == SCALER_ACTIVE) {
						data->connection_status = CONNECTED;
						scaler_notify_service(data, CONNECTED);

						if( 1 == data->reg_status.request_PA_on) {
							SCALER_INFO("turn on PA");
							tfa98xx_set_pa_state(true);
						} else if ( 2 == data->reg_status.request_PA_on) {
							SCALER_INFO("turn off PA");
							tfa98xx_set_pa_state(false);
						}

						if (data->panel_off) {
							tv_touchpoint.brightness_plug_in = 0;
							tp_resume_call_by_panel();
						}
					}
				}
			}
		}
	} else {
		SCALER_ERROR("data is null");
	}

	msleep(10);

	pm_relax(&data->client->dev);
}


static irqreturn_t scaler_irq_thread(int irq, void *dev_id)
{
	struct scaler_data *data = dev_id;

	SCALER_INFO("scaler_irq_thread");

	schedule_work(&data->scaler_irq_work);

	return IRQ_HANDLED;
}

static int scaler_register_interrupts(struct scaler_data *data)
{
	struct scaler_pdata *pdata = data->pdata;
	 int ret;

	if(gpio_is_valid(pdata->hpd_irq_gpio)) {
		 data->hpd_irq = gpio_to_irq(pdata->hpd_irq_gpio);
		 SCALER_INFO("hpd_irq = %d", data->hpd_irq);

	        ret = devm_request_threaded_irq(&data->client->dev, data->hpd_irq, NULL,
	                        hpd_irq_thread,
	                        IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
	                        "hpd_irq", data);
	}

	if(gpio_is_valid(pdata->scaler_irq_gpio)) {
		 data->scaler_irq = gpio_to_irq(pdata->scaler_irq_gpio);
		 SCALER_INFO("scaler_irq = %d", data->scaler_irq);

	        ret = devm_request_threaded_irq(&data->client->dev, data->scaler_irq, NULL,
	                        scaler_irq_thread,
	                        IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	                        "scaler_irq", data);
	}

        return ret;
}

static int scaler_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct scaler_data *data = dev_get_drvdata(dev);
	int ret = 0;

	if (!data) {
		dev_dbg(dev, "No scaler yet\n");
		return ret;
	}

	ret = add_uevent_var(env, "SUBSYSTEM=%s", "scaler");
	if (ret)
		return ret;

	return 0;
}

static int scaler_create_device_class(struct scaler_data *data)
{
	dev_t dev_num;
	int ret;

	if (data->scaler_class != NULL)
		return 0;

	data->scaler_class = class_create(THIS_MODULE, DEVICE_CLASS_NAME);
	if (IS_ERR(data->scaler_class)) {
		pr_err("%s: Failed to create class\n", __func__);
		return -ENODEV;
	}

	 ret = alloc_chrdev_region(&dev_num, 0, 1, "scaler");
	 if(ret) {
		pr_err("%s: alloc_chrdev_region failed: %d\n", __func__, ret);
		goto exit_class_destroy;
	 }

	data->scaler_devt = dev_num;
	data->scaler_class->dev_uevent = scaler_uevent;

	data->scaler_dev = device_create(data->scaler_class, NULL, dev_num, "scaler_drvdata", "scaler");
	if (IS_ERR(data->scaler_dev)) {
		pr_err("%s: Failed to create device\n", __func__);
		goto exit_class_destroy;
	}

	dev_set_drvdata(data->scaler_dev, data);

	ret = sysfs_create_group(&data->scaler_dev->kobj, &scaler_attr_group);
        if (ret < 0) {
                pr_err("%s: Failed to create sysfs attributes\n", __func__);
                goto exit_device_destroy;
        }

	return 0;

exit_device_destroy:
	device_destroy(data->scaler_class, dev_num);
	unregister_chrdev_region(dev_num, 1);
exit_class_destroy:
	class_destroy(data->scaler_class);

	return ret;
}

#ifdef CONFIG_DRM
static int scaler_check_dt(struct device_node *np)
{
        int i;
        int count;
        struct device_node *node;
        struct drm_panel *panel;

        count = of_count_phandle_with_args(np, "panel", NULL);
        if (count <= 0)
                return 0;

        for (i = 0; i < count; i++) {
                node = of_parse_phandle(np, "panel", i);
                panel = of_drm_find_panel(node);
                of_node_put(node);
                if (!IS_ERR(panel)) {
                        active_panel = panel;
                        pr_info(" %s:find\n", __func__);
                        return 0;
                }
        }

        pr_err(" %s: not find\n", __func__);
        return -ENODEV;
}

static int scaler_suspend(struct scaler_data *data)
{
        int ret = 0;

        SCALER_INFO("scaler_suspend enter\n");

        if(data->connection_status == CONNECTING) {
                switch_to_tv_mode(true);

                /* notify scaler to init panel */
                ret = notify_scaler_to_init_panel(data);
                if(ret)
                        SCALER_ERROR("failed to notify_scaler_to_init_panel");
        }

	enable_irq_wake(data->hpd_irq);
	enable_irq_wake(data->scaler_irq);

        return ret;
}

static int scaler_resume(struct scaler_data *data)
{
        SCALER_INFO("scaler_resume enter\n");
        return 0;
}

static int scaler_drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct scaler_data *scaler_data = container_of(self, struct scaler_data, drm_notifier);
	struct drm_panel_notifier *evdata = data;

        SCALER_INFO("scaler_drm_notifier_callback");

        if (evdata && evdata->data && scaler_data) {
                int *blank = evdata->data;
                if (event == DRM_PANEL_EVENT_BLANK) {
                        if (*blank == DRM_PANEL_BLANK_POWERDOWN)
                                scaler_suspend(scaler_data);

                        /* before fb blank */
                        if (*blank == DRM_PANEL_BLANK_UNBLANK)
                                scaler_resume(scaler_data);
                }
        }

        return 0;
}
#endif

static int scaler_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct scaler_pdata *pdata;
	struct scaler_data *data;
	int ret;

	SCALER_INFO("enter %s\n\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SCALER_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct scaler_pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		ret = scaler_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "Failed to parse dts.\n");
			return -EINVAL;
		}

		ret = scaler_check_dt(client->dev.of_node);
		if (ret) {
			dev_err(&client->dev, "Failed to check dts.\n");
			return -EINVAL;
		}
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		dev_err(&client->dev, "scaler invalid pdata\n");
		return -EINVAL;
	}

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	memset(data, 0, sizeof(*data));
	data->client = client;
	data->pdata = pdata;
	data->connection_status = DISCONNECTED;
	g_data = data;

	data->hdcp_key_status = UNKONWN;
	data->fw_version = 0;

	mutex_init(&data->mutex);

	dev_set_drvdata(&client->dev, data);

	ret = scaler_request_gpio(data);
	if (ret) {
		dev_err(&client->dev, "scaler request IO port failed.\n");
		goto exit_free_client_data;
	}

	INIT_WORK(&data->hpd_irq_work, hpd_irq_work);
	INIT_WORK(&data->scaler_irq_work, scaler_irq_work);
	INIT_WORK(&data->touch_irq_work, touch_irq_work);
	INIT_DELAYED_WORK(&data->update_status_work, update_status_delayed_work);
	INIT_DELAYED_WORK(&data->cable_detection_work, cable_detection_delayed_work);

	/* Power on scaler to get fw_ver and hdcp_key status. */
	ret = scaler_power_init(data);
	ret = scaler_power_on(data);
	if(ret) {
		dev_err(&client->dev, "scaler power on failed.\n");
	} else {
		schedule_delayed_work(&data->update_status_work, 2*HZ);
	}

	ret = scaler_config_input(data);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register input device. err: %d\n", ret);
		goto exit_free_gpio;
	}

#ifdef CONFIG_DRM
	data->drm_notifier.notifier_call = scaler_drm_notifier_callback;
        if (active_panel &&
                drm_panel_notifier_register(active_panel,
                        &data->drm_notifier) < 0) {
                dev_err(&client->dev, "register notifier failed!\n");
                goto exit_free_input;
        }
#endif

	ret = scaler_create_device_class(data);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Failed to create device class\n", ret);
		goto exit_unregister_drm_notifier;
	}

	device_init_wakeup(&data->client->dev, true);

	ret = scaler_register_interrupts(data);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register interrupts. err: %d\n", ret);
		goto exit_free_class;
	}

	if(!gpio_get_value(pdata->hpd_irq_gpio)) {	// already plug in
		dev_info(&client->dev, "hdmi cable is connected.\n");
		data->cable_connected = true;
		data->connection_status = CONNECTING;

		if(gpio_is_valid(data->pdata->bl_up_gpio)) {
			ret = gpio_direction_output(pdata->bl_up_gpio, 1);
			if (ret) {
				pr_err("Fail to output bl_up gpio 1, ret = %d\n", ret);
			}
		}

		if(gpio_is_valid(data->pdata->bl_down_gpio)) {
			ret = gpio_direction_output(pdata->bl_down_gpio, 1);
			if (ret) {
				pr_err("Fail to output bl_down gpio 1, ret = %d\n", ret);
			}
		}
	} else
		data->cable_connected = false;

	return 0;

exit_free_class:
	if (data->scaler_class != NULL) {
		class_destroy(data->scaler_class);
		data->scaler_class = NULL;
	}
	device_init_wakeup(&data->client->dev, false);
exit_unregister_drm_notifier:
#ifdef CONFIG_DRM
        if (active_panel)
                drm_panel_notifier_unregister(active_panel, &data->drm_notifier);
#endif
exit_free_input:
	input_free_device(data->input_dev);
exit_free_gpio:
	scaler_free_gpio(pdata);
exit_free_client_data:
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
	return ret;
 }

static int scaler_remove(struct i2c_client *client)
{
	struct scaler_data *data = i2c_get_clientdata(client);

	device_init_wakeup(&data->client->dev, false);
	free_irq(data->hpd_irq, data);
	free_irq(data->scaler_irq, data);
	scaler_free_gpio(data->pdata);
	input_free_device(data->input_dev);
#ifdef CONFIG_DRM
        if (active_panel)
                drm_panel_notifier_unregister(active_panel, &data->drm_notifier);
#endif
	dev_set_drvdata(data->scaler_dev, NULL);
	device_destroy(data->scaler_class, data->scaler_devt);
	class_destroy(data->scaler_class);
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
	kfree(data);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id scaler_match_table[] = {
		{.compatible = "realtek,scaler",},
		{ },
};
#endif

static const struct i2c_device_id scaler_id[] = {
    { SCALER_I2C_NAME, 0 },
    { }
};

static struct i2c_driver scaler_driver = {
    .probe      = scaler_probe,
    .remove     = scaler_remove,
    .id_table   = scaler_id,
    .driver = {
        .name     = SCALER_I2C_NAME,
        .owner    = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = scaler_match_table,
#endif
    },
};

static int __init scaler_init(void)
{
    s32 ret;
    ret = i2c_add_driver(&scaler_driver);
    SCALER_INFO("i2c_add_driver, ret = %d", ret);

    return ret;
}

static void __exit scaler_exit(void)
{
    SCALER_INFO("scaler driver exited.");
    i2c_del_driver(&scaler_driver);
}

module_init(scaler_init);
module_exit(scaler_exit);

MODULE_DESCRIPTION("Realtek scaler Driver");
MODULE_LICENSE("GPL");
