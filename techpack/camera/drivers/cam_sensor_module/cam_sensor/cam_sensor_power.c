#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <cam_sensor_cmn_header.h>
#include <cam_sensor_util.h>
#include <cam_sensor_io.h>
#include <cam_sensor_dev.h>
#include "cam_soc_util.h"
#include <cam_req_mgr_util.h>
#include "cam_req_mgr_dev.h"
#include "cam_debug_util.h"
#include <linux/platform_device.h>
#include <linux/of_device.h>

#define WL2864_SLAVE (0x29)
#define MIPI_REDRIVER_SLAVE (0x6c)
typedef union {
	unsigned unsigned char chipids[8];
	unsigned int chipid[2];
} mipi_redriver_chipid_s;

struct cam_power_device {
	struct camera_io_master io_master;
	struct cam_sensor_cci_client cci_client;
	unsigned char wl2864_addr;
	unsigned char mipi_addr;
	unsigned char wl2864_probed;
	unsigned char mipi_probed;
	int first_powerup;
	int en_gpio;
	int power_probed;
};

static struct cam_power_device cpd;

static struct cam_sensor_i2c_reg_array cam_reg_on_setting[] = {
	{0x03, 0x34, 0x5, 0x0},		//cam1 dvdd 1.2v LDO1
	{0x05, 0x82, 0x1, 0x0},		//cam1 avdd 2.8v LDO3
	{0x01, 0x3F, 0x1, 0x0},		//Higher current limit
	{0x0E, 0x05, 0x1, 0x0},		//enable ldo1,ldo3
};

static struct cam_sensor_i2c_reg_array cam_reg_off_setting[] = {
	{0x0E, 0x00, 0x2, 0x0},		//disable all
};

static struct cam_sensor_i2c_reg_array cam_mipi_reg_off_setting[] = {
	{0x09, 0x00, 0xf, 0x0},		//set mipi redriver
};

static struct cam_sensor_i2c_reg_setting cam_reg_on_cci_setting = {
	.reg_setting = cam_reg_on_setting,
	.size = 4,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting cam_reg_off_cci_setting = {
	.reg_setting = cam_reg_off_setting,
	.size = 1,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static struct cam_sensor_i2c_reg_setting cam_mipi_reg_off_cci_setting = {
	.reg_setting = cam_mipi_reg_off_setting,
	.size = 1,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

static int inline is_s5k4h7_sensor(struct cam_sensor_ctrl_t *s_ctrl)
{
	int ret = (s_ctrl->sensordata->slave_info.sensor_slave_addr == 0x20 &&
			   s_ctrl->sensordata->slave_info.sensor_id == 0x487b) ? 1 : 0;
	CAM_INFO(CAM_SENSOR, "ret = %d", ret);

	return ret;
}

static void cam_power_device_detect(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	if (!cpd.wl2864_probed && !cpd.mipi_probed) {

		struct cam_power_device *extern_cam = &cpd;
		unsigned int rdata;
		mipi_redriver_chipid_s mipi_redriver_chipid;
		CAM_INFO(CAM_SENSOR, "enter ");

		memcpy(&extern_cam->io_master, &(s_ctrl->io_master_info),
			   sizeof(extern_cam->io_master));
		memcpy(&extern_cam->cci_client, s_ctrl->io_master_info.cci_client,
			   sizeof(extern_cam->cci_client));

		extern_cam->cci_client.sid = cpd.mipi_addr;
		extern_cam->io_master.cci_client = &extern_cam->cci_client;
		rc = camera_io_dev_read_seq(&extern_cam->io_master,
									0, mipi_redriver_chipid.chipids,
									CAMERA_SENSOR_I2C_TYPE_BYTE,
									CAMERA_SENSOR_I2C_TYPE_BYTE, 8);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "fail to read mipiredirver chipid");
		} else {
			int i = 0;
			for (i = 0; i < 8; i++)
				CAM_DBG(CAM_SENSOR, "mipiredirver chipid[%d] = 0x%x", i,
						mipi_redriver_chipid.chipids[i]);

			CAM_INFO(CAM_SENSOR, "mipiredirver chipid = 0x%x 0x%x",
					 mipi_redriver_chipid.chipid[0],
					 mipi_redriver_chipid.chipid[1]);

			if (0x59485044 == mipi_redriver_chipid.chipid[0]
				&& 0x20303031 == mipi_redriver_chipid.chipid[1]) {
				extern_cam->mipi_probed = 1;
			}

			extern_cam->cci_client.sid = cpd.wl2864_addr;
			extern_cam->io_master.cci_client = &extern_cam->cci_client;

			rc = camera_io_dev_read(&extern_cam->io_master,
									0, &rdata,
									CAMERA_SENSOR_I2C_TYPE_BYTE,
									CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "fail to read wl2864 chipid");

			} else
				CAM_INFO(CAM_SENSOR, "wl2864 chipid = 0x%x", rdata);

			if (rdata == 1) {
				extern_cam->wl2864_probed = 1;
			}

		}

	}
}

static void cam_power_device_get_reg(struct camera_io_master *io_master,
									 int reg)
{
	int rc;
	unsigned char rval;
	rc = camera_io_dev_read_seq(io_master,
								reg, &rval,
								CAMERA_SENSOR_I2C_TYPE_BYTE,
								CAMERA_SENSOR_I2C_TYPE_BYTE, 1);
	if (rc) {
		CAM_ERR(CAM_SENSOR, " failed ");
	}
	CAM_INFO(CAM_SENSOR, " get reg(0x%x)=0x%x ", reg, rval);
}

void cam_power_device_set_power(struct cam_sensor_ctrl_t *s_ctrl, int on)
{
	struct cam_power_device *extern_cam = &cpd;

	if (!extern_cam->power_probed) {
		CAM_ERR(CAM_SENSOR, "device probe failed, return");
		return;
	}

	if (is_s5k4h7_sensor(s_ctrl)) {
		CAM_INFO(CAM_SENSOR, " set power:%d", on);

		gpio_set_value(cpd.en_gpio, 0);
		CAM_INFO(CAM_SENSOR, "set gpio %d 0 ", cpd.en_gpio);

		msleep(10);

		cam_power_device_detect(s_ctrl);

		if (!extern_cam->wl2864_probed || !extern_cam->mipi_probed) {
			CAM_ERR(CAM_SENSOR,
					"wl2864_exist or mipi redriver is not exist, return");
			return;
		}

		/* config wl2864 registers and mipi-redriver */
		if (on) {

			extern_cam->cci_client.sid = cpd.wl2864_addr;
			extern_cam->io_master.cci_client = &extern_cam->cci_client;
			camera_io_dev_write(&extern_cam->io_master,
								&cam_reg_on_cci_setting);

			cam_power_device_get_reg(&extern_cam->io_master, 0x3);
			cam_power_device_get_reg(&extern_cam->io_master, 0x5);
			cam_power_device_get_reg(&extern_cam->io_master, 0x1);
			cam_power_device_get_reg(&extern_cam->io_master, 0xE);

			extern_cam->cci_client.sid = cpd.mipi_addr;
			extern_cam->io_master.cci_client = &extern_cam->cci_client;
			camera_io_dev_write(&extern_cam->io_master,
								&cam_mipi_reg_off_cci_setting);
		} else {				/*power off wl2864 */

			extern_cam->cci_client.sid = cpd.wl2864_addr;
			extern_cam->io_master.cci_client = &extern_cam->cci_client;
			camera_io_dev_write(&extern_cam->io_master,
								&cam_reg_off_cci_setting);
		}

	}
}

static int32_t cam_power_platform_driver_probe(struct platform_device *pdev)
{
	int32_t rc = 0;

	struct cam_power_device *ecs = &cpd;
	memset(ecs, 0, sizeof(struct cam_power_device));

	ecs->en_gpio = of_get_named_gpio(pdev->dev.of_node, "en-gpio", 0);
	if (ecs->en_gpio < 0) {
		CAM_ERR(CAM_SENSOR,
				"platform_driver_register get gpio failed, err:%d",
				ecs->en_gpio);
		rc = -ENODEV;
		goto out;
	}

	if (of_property_read_u8(pdev->dev.of_node, "wl2864-addr",
							&ecs->wl2864_addr) < 0) {
		CAM_INFO(CAM_SENSOR, "Invalid wl2864_addrl, use default");
		ecs->wl2864_addr = WL2864_SLAVE;
	}

	if (of_property_read_u8(pdev->dev.of_node, "mipi-addr",
							&ecs->mipi_addr) < 0) {
		CAM_INFO(CAM_SENSOR, "Invalid mipi_addr, use default");
		ecs->mipi_addr = MIPI_REDRIVER_SLAVE;
	}

	CAM_INFO(CAM_SENSOR,
			 "  get gpio %d wl2864_addrl:0x%x mipi_addr:0x%x",
			 ecs->en_gpio, ecs->wl2864_addr, ecs->mipi_addr);

	ecs->power_probed = true;
	return 0;

  out:
	CAM_ERR(CAM_SENSOR, "probe failed");
	return rc;
}

static int cam_power_platform_driver_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id cam_power_dt_match[] = {
	{.compatible = "qcom,ic_power"},
	{}
};

MODULE_DEVICE_TABLE(of, cam_power_dt_match);

static struct platform_driver cam_power_platform_driver = {
	.driver = {
			   .name = "qcom,ic_power",
			   .owner = THIS_MODULE,
			   .of_match_table = cam_power_dt_match,
			   },
	.probe = cam_power_platform_driver_probe,
	.remove = cam_power_platform_driver_remove,
};

static int __init cam_power_driver_init(void)
{
	int rc = 0;
	CAM_INFO(CAM_SENSOR, "enter");

	rc = platform_driver_register(&cam_power_platform_driver);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, " failed rc = %d", rc);
		return rc;
	}
	return rc;
}

static void __exit cam_power_driver_exit(void)
{
	platform_driver_unregister(&cam_power_platform_driver);
}

subsys_initcall(cam_power_driver_init);
module_exit(cam_power_driver_exit);
MODULE_DESCRIPTION("CAM POWER driver");
MODULE_LICENSE("GPL v2");
