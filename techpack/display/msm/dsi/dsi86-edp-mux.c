#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/util_macros.h>
#include <linux/regmap.h>
#include <linux/pwm.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/semaphore.h>
#include <linux/i2c.h>
#include <trace/events/smbus.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include "dsi86-edp-mux.h"
#include "linux/bootinfo.h"

#define EDP_MUX_I2C_ADDR	0x33
#define DSI86_I2C_ADDR		0x2C
#define DSI86_POWER_OFF_TIME    500

extern int sde_i2c_byte_read(struct i2c_client *client, uint8_t slave_addr,
                        uint8_t reg_offset, uint8_t *read_buf);
extern int sde_i2c_byte_write(struct i2c_client *client, uint8_t slave_addr,
                        uint8_t reg_offset, uint8_t *value);
static int switch_to_port1(bool port1);

struct semaphore dsi86_sem;
static const struct of_device_id edp_mux_of_match[] = {
	{
        .compatible = "eDP-MUX-8331"
	},
};



static struct edp_gpio_info gpio_arrays[32];

struct i2c_client* g_i2c_client = NULL;
static struct regulator *vcc_1p2_vreg;
static struct regulator *vcc_1p5_vreg;
static struct regulator *vcc_1p8_vreg;
static struct regulator *vcc_3p3_vreg;
static struct clk *refclk_clk;

static bool dsi86_edp2mipi_on;
static bool dsi86_edp_mux_on;

MODULE_DEVICE_TABLE(of, dsi86_edp_mux_of_match);

static int first_probe = 0;
static int board_id = -1;
static ktime_t g_last_stime = 0;

int i2c_write(u8 slave_addr, u8 addr, u8 val)
{
#if 0
	u8 buf[] = {addr, val};
        int ret;

        struct i2c_msg msg = {
                .addr = slave_addr,
                .len = 2,
		.flags = 0;
                .buf = buf,
        };

	if (g_i2c_client == NULL)
	{
		pr_err("g_i2c_client is null");
		return -1;
	}

        ret = i2c_transfer(g_i2c_client->adapter, &msg, 1);
        if (ret < 0)
                dev_err(&g_i2c_client->dev, "Error %d writing to cec:0x%x\n",
                        ret, addr);
#endif
	int ret;

	if (g_i2c_client == NULL)
	{
		pr_err("g_i2c_client is null");
		return -1;
	}
	ret  = sde_i2c_byte_write(g_i2c_client, slave_addr<<1, addr, &val);
	if (ret < 0)
               dev_err(&g_i2c_client->dev, "Error %d writing to cec:0x%x\n",
                        ret, addr);
	return ret;

}

extern s32 i2c_smbus_read_byte_data(const struct i2c_client *client, u8 command);

int i2c_read(u8 slave_addr, u8 addr, u8* val)
{
        int reg_value;

	if (g_i2c_client == NULL)
	{
		pr_err("g_i2c_client is null");
		return -1;
	}
	g_i2c_client->addr = slave_addr;
	reg_value = i2c_smbus_read_byte_data(g_i2c_client, addr);
	if (reg_value < 0) {
		pr_err("%s read addr=0x%xfailed !!! \n", __func__, addr);
	} else {
		*val = reg_value;
	}

	return reg_value;

}

struct dsi_i2c_cmd{
	u8 reg;
	u8 value[8];
	int len; //reg + value
};

/*
static struct dsi_i2c_cmd dsi86_init_assr[] = {
	{0x0A, {0x02}, 2}, //19.2M REFCLK
	{0xE0, {0x01}, 2},
	{0x10, {0x80}, 2},
	{0x12, {0x36}, 2},
	{0x13, {0x36}, 2},
	{0x94, {0x80}, 2},
	{0x0d, {0x01}, 2},
	{0x64, {0x01}, 2},
	{0x74, {0x00}, 2},
	{0x75, {0x01}, 2},
	{0x76, {0x0A}, 2},
	{0x77, {0x01}, 2},
	{0x78, {0x81}, 2},
	{0xEE, {0x0A}, 0}, //sleep 10 ms

	{0x5A, {0x05}, 2}, //sleep 10 ms
	{0x93, {0x20}, 2},
	//prem start
	{0xB0, {0x04}, 2},
	{0xB1, {0x08}, 2},
	{0xB2, {0x0C}, 2},

	{0xB3, {0x24}, 2},
	{0xB4, {0x28}, 2},
	{0xB5, {0x2C}, 2},

	{0xB6, {0x34}, 2},
	{0xB7, {0x38}, 2},
	{0xB8, {0x3C}, 2},

	{0xB9, {0x44}, 2},
	{0xBA, {0x48}, 2},
	{0xBB, {0x4C}, 2},

	{0xBC, {0x54}, 2},
	{0xBD, {0x58}, 2},
	{0xBE, {0x5C}, 2},
	{0xBF, {0x7C}, 2},

	{0xC0, {0xFF}, 2},
	{0xC1, {0xFF}, 2},
	{0xC2, {0xFF}, 2},
	{0xC3, {0xFF}, 2},
	{0x95, {0x00}, 2},
	//prem end

	{0x96, {0x0A}, 2},

	{0x20, {0x38}, 2},
	{0x21, {0x04}, 2},
	{0x22, {0x38}, 2},
	{0x23, {0x04}, 2},
	{0x24, {0x46}, 2},
	{0x25, {0x05}, 2},
	{0x2c, {0x10}, 2},
	{0x2D, {0x00}, 2},
	{0x30, {0x01}, 2},
	{0x31, {0x00}, 2},

	{0x34, {0x0c}, 2},
	{0x36, {0x08}, 2},
	{0x38, {0x10}, 2},
	{0x3a, {0x08}, 2},
	{0x5b, {0x00}, 2},
	{0x3c, {0x07}, 2},
	{0x5a, {0x0d}, 2},
	{0xDD, {0},    0} //end
};*/

static struct dsi_i2c_cmd dsi86_init[] = {
	{0xFF, {0x07}, 2},
	{0x16, {0x01}, 2}, //ASSR RW EN
	{0xFF, {0x00}, 2},
	{0x0A, {0x02}, 2}, //19.2M REFCLK
	{0xE0, {0x01}, 2}, //irq
	{0x5c, {0x01}, 2}, //disable HPD
	{0x10, {0x80}, 2},
	{0x12, {0x36}, 2},
	{0x13, {0x36}, 2},
	{0x94, {0x80}, 2},
	{0x0d, {0x01}, 2},
	{0x5A, {0x04}, 2}, // NO ASSR(SDSS)
	{0x93, {0x20}, 2},
	//prem start
	{0xB0, {0x04}, 2},
	{0xB1, {0x08}, 2},
	{0xB2, {0x0C}, 2},

	{0xB3, {0x24}, 2},
	{0xB4, {0x28}, 2},
	{0xB5, {0x2C}, 2},

	{0xB6, {0x34}, 2},
	{0xB7, {0x38}, 2},
	{0xB8, {0x3C}, 2},

	{0xB9, {0x44}, 2},
	{0xBA, {0x48}, 2},
	{0xBB, {0x4C}, 2},

	{0xBC, {0x54}, 2},
	{0xBD, {0x58}, 2},
	{0xBE, {0x5C}, 2},
	{0xBF, {0x7C}, 2},

	{0xC0, {0xFF}, 2},
	{0xC1, {0xFF}, 2},
	{0xC2, {0xFF}, 2},
	{0xC3, {0xFF}, 2},
	{0x95, {0x00}, 2},
	//prem end

	{0x96, {0x09}, 2},

	{0x20, {0x38}, 2},
	{0x21, {0x04}, 2},
	{0x22, {0x38}, 2},
	{0x23, {0x04}, 2},
	{0x24, {0x46}, 2},
	{0x25, {0x05}, 2},
	{0x2c, {0x10}, 2},
	{0x2D, {0x00}, 2},
	{0x30, {0x01}, 2},
	{0x31, {0x00}, 2},

	{0x34, {0x0c}, 2},
	{0x36, {0x08}, 2},
	{0x38, {0x10}, 2},
	{0x3a, {0x08}, 2},
	{0x5b, {0x00}, 2},
	{0x3c, {0x07}, 2},
	{0x5a, {0x0c}, 2}, //NO ASSR
	{0xDD, {0},    0} //end
};

int dsi86_edp_regs_init(void)
{
	int i = 0;
	int ret = 0;
	u8 reg_value = 0;
	int count = 0;
	//int dp_tx_swing = 0;
	//int dp_pre_emphasis = 0;

	pr_err("%s enter ", __func__);

	if (g_i2c_client == NULL) 	{
		pr_err("g_i2c_client is null");
		return -1;
	}

	while(dsi86_init[i].reg != 0xDD) {

		if (dsi86_init[i].reg == 0xEE) {
			mdelay(dsi86_init[i].value[0]); /* Delay after writing 0x78 */
			i ++;
			continue;
		}

		ret = sde_i2c_byte_write(g_i2c_client, DSI86_I2C_ADDR<<1,
                        dsi86_init[i].reg, dsi86_init[i].value);
        	if (ret < 0) {
                	dev_err(&g_i2c_client->dev, "Error %d writing to address:0x%x\n",
                        ret, dsi86_init[i].reg);
			pr_err("%s i2c_write failed %x \n", __func__, dsi86_init[i].value[0]);
			return -1;
		}

		if(dsi86_init[i].reg == 0x0D) {
			pr_err("%s need check 0x0D register \n", __func__);
			count = 0;
			mdelay(10); /* Delay after writing 0x0d */
			while( count < 10) {
				reg_value = 0;
				ret = i2c_read(DSI86_I2C_ADDR, 0x0A, &reg_value);
				if (ret < 0) {
					pr_err("\n dsi86 Semi-Auto Training failed !!! reg_value=0x%x \n", reg_value);
					return -1;
				}
				if (reg_value == 0x82) {
					pr_err("\n 0x0A value is reg_value=0x%x  DP_PLL_LOCK success\n", reg_value);
					break;
				} else if (reg_value != 0x82) {
					pr_err("\n count =%d 0x0D value is reg_value=0x%x \n", count, reg_value);
					mdelay(10); /* Delay in case of unexpected value */
				}
				count ++;
			}
		}
		if(dsi86_init[i].reg == 0x96) {
			pr_err("%s need check 0x96 register \n", __func__);
			count = 0;
			mdelay(20); /* Delay after wrting 0x96 */
			while( count < 10) {
				reg_value = 0;
				ret = i2c_read(DSI86_I2C_ADDR, 0x96, &reg_value);
				if (ret < 0) {
					pr_err("\n dsi86 Semi-Auto Training failed !!! reg_value=0x%x \n", reg_value);
					return -1;
				}

				if (reg_value == 0x01) {
					pr_err("\n 0x96 value is reg_value=0x%x Linking-Training success\n", reg_value);
					break;
				} else if (reg_value == 0x0) {
					pr_err("\n count =%d 0x96 value is reg_value=0x%x Semi-Auto-Training need restart \n", count, reg_value);
					//msleep(20);
/*
					dp_pre_emphasis ++;
					if (dp_pre_emphasis > 3) {
						dp_pre_emphasis = 3;
					}
					pr_err("dp_pre_emphasis = 0x%x \n", dp_pre_emphasis << 6);
					ret = i2c_write(DSI86_I2C_ADDR, 0x93, 0x20 + (dp_pre_emphasis << 6));
					if (ret < 0) {
						pr_err("\n dsi86 Restart-Semi-Auto Training failed !!! ret=0x%x \n", ret);
						return -1;
					}

					dp_tx_swing ++;
					if(dp_tx_swing > 2)
						dp_tx_swing = 2;

					ret = i2c_write(DSI86_I2C_ADDR, 0x94, (0x80 + dp_tx_swing));
					if (ret < 0) {
						pr_err("\n dsi86 Restart-Semi-Auto Training failed !!! ret=0x%x \n", ret);
						return -1;
					}
*/
					//Restart Semi-Auto-Trainning
					ret = i2c_write(DSI86_I2C_ADDR, 0x96, 0x02);
					if (ret < 0) {
						pr_err("\n dsi86 set tps1 failed !!! ret=0x%x \n", ret);
						return -1;
					}
					ret = i2c_write(DSI86_I2C_ADDR, 0x96, 0x09);
					if (ret < 0) {
						pr_err("\n dsi86 Restart-Semi-Auto Training failed !!! ret=0x%x \n", ret);
						return -1;
					}
					mdelay(10); /* Delay after Link Re Training */
				}
				count ++;
			}
			#if 0
			reg_value = 0;
			ret = i2c_read(DSI86_I2C_ADDR, 0x93, &reg_value);
			pr_err("\n 0x93 value is reg_value=0x%x \n", reg_value);
			ret = i2c_read(DSI86_I2C_ADDR, 0x94, &reg_value);
			pr_err("\n 0x94 value is reg_value=0x%x \n", reg_value);
			ret = i2c_read(DSI86_I2C_ADDR, 0x95, &reg_value);
			pr_err("\n 0x95 value is reg_value=0x%x \n", reg_value);
			ret = i2c_read(DSI86_I2C_ADDR, 0xB1, &reg_value);
			pr_err("\n 0xB1 value is reg_value=0x%x \n", reg_value);
			#endif
		}

		i ++;
	}
#if 0
	pr_err("read badck registers \n");
	i = 0;
	while(dsi86_init[i].reg != 0xDD) {
		if (dsi86_init[i].reg == 0xEE) {
			 i ++;
			continue;
		}
		reg_value = 0;
		//ret = sde_i2c_byte_read(g_i2c_client, (DSI86_I2C_ADDR<<1), dsi86_init[i].reg, &reg_value);
		ret = i2c_read(DSI86_I2C_ADDR, dsi86_init[i].reg, &reg_value);
		if (ret < 0) {
			pr_err("\n dsi86 Semi-Auto Training failed !!! reg_value=0x%x \n", reg_value);
			return -1;
		}
		pr_err("\n\t reg[0x%x]=0x%x \n", dsi86_init[i].reg, reg_value);
		i ++;
	}

	i = 0;
	for(i=0; i < 9; i++) {
		//ret = sde_i2c_byte_read(g_i2c_client, (DSI86_I2C_ADDR<<1), 0x0+i, &device_name[i]);
		ret = i2c_read(DSI86_I2C_ADDR, i, &device_name[i]);
		if(ret < 0) {
			dev_err(&g_i2c_client->dev, "Error %d writing to address:0x%x\n",
                        ret, i);
			return -1;
		}

	}
	device_name[i] = '\0';
	pr_err("device_name=%s \n",device_name);
#endif
	reg_value = 0;
	ret = i2c_read(DSI86_I2C_ADDR, 0x93, &reg_value);
	pr_err("reg=0x%x:value=0x%x \n", 0x93, reg_value);
	ret = i2c_read(DSI86_I2C_ADDR, 0x94, &reg_value);
	pr_err("reg=0x%x:value=0x%x \n", 0x94, reg_value);
	ret = i2c_read(DSI86_I2C_ADDR, 0x95, &reg_value);
	pr_err("reg=0x%x:value=0x%x \n", 0x95, reg_value);
/*
	i = 0;
	reg_value = 0;
	pr_err("dump error registers:\n");
	for(i = 0; i < 9; i++) {
		ret = i2c_read(DSI86_I2C_ADDR, 0xf0 + i, &reg_value);
		pr_err("reg=0x%x:value=x%x \n", (0xF0 + i), reg_value);
	}
	i = 0;
	for(i = 0; i < 3; i++) {
		ret = i2c_read(DSI86_I2C_ADDR, 0xb0 + i, &reg_value);
		pr_err("reg=0x%x:value=x%x \n", (0xb0 + i), reg_value);
	}
*/
	pr_err("dis86-edp-init init done");

	return 0;
}

int dsi86_edp_mux_init(void)
{
	int ret = 0;
	//u8 reg_value = 0;
	pr_err("%s enter \n",__func__);

	//For  dsi86-edp mux
	ret = i2c_write(EDP_MUX_I2C_ADDR, 0, 0);
	if (ret < 0)
		pr_err("write 00 register error!");

	ret = i2c_write(EDP_MUX_I2C_ADDR, 0x3, 0x38);
	if (ret < 0) {
		pr_err("write 0x3 register error!");
	}
/*
	ret = i2c_write(EDP_MUX_I2C_ADDR, 0x09, 0x40);
	if (ret < 0)
		pr_err("write 0x09 register error!");
*/
	ret = i2c_write(EDP_MUX_I2C_ADDR, 0x10, 0x0A);
	if (ret < 0) {
		pr_err("write 0x10 register error!");
	}

	ret = i2c_write(EDP_MUX_I2C_ADDR, 0x11, 0x02);
	if (ret < 0) {
		pr_err("write 0x11 register error!");
	}
/*
	ret = i2c_write(EDP_MUX_I2C_ADDR, 0x12, 0x04);
	if (ret < 0)
		pr_err("write 0x12 register error!");

	ret = i2c_write(EDP_MUX_I2C_ADDR, 0x13, 0x04);
	if (ret < 0)
		pr_err("write 0x13 register error!");
*/
	pr_err("dsi86 edp mux init done!!!");
/*
	mdelay(10);

	ret = i2c_read(EDP_MUX_I2C_ADDR, 0x0C, &reg_value);
	if (ret < 0) {
		pr_err("read 0x0c register error!");
	}
	pr_err("0x0c=0x%x \n", reg_value);
*/
	return 0;
}

/*
int dsi86_boost_bypass_init(void)
{
	int ret = 0;
	pr_err("%s enter \n",__func__);

	//For  dsi86-edp mux
	ret = i2c_write(0x75, 1, 0x43);
	if (ret < 0)
        	pr_err("write 00 register error!");
	ret = i2c_write(0x75, 2, 0x13);
	if (ret < 0) {
        	pr_err("write 00 register error!");
	}
	ret = i2c_write(0x75, 3, 0x13);
	if (ret < 0) {
        	pr_err("write 00 register error!");
	}
	ret = i2c_write(0x75, 4, 0x0b);
	if (ret < 0) {
        	pr_err("write 00 register error!");
	}
	pr_err("dsi86 backlight init done!!!");

	return 0;
}
*/
/*
struct pwm_setting {
        u64     pre_period_ns;
        u64     period_ns;
        u64     duty_ns;
}pwm_setting;

struct pwm_device       *pwm_dev = NULL;
#define PWM_PERIOD_DEFAULT_NS           (1000*1000)
struct pwm_args pargs;
static int __tri_led_config_pwm(struct pwm_setting *pwm)
{
        struct pwm_state pstate;
        int rc;
	pr_err("%s enter\n", __func__);
	if(pwm_dev == NULL)
	{
		pr_err("%s:pwm_dev is null!!!\n");
		return -1;
	}
        pwm_get_state(pwm_dev, &pstate);
        pstate.enabled = 1;
        pstate.period = 1000*1000;
        pstate.duty_cycle = 500*1000;
        pstate.output_type = PWM_OUTPUT_FIXED;
        //Use default pattern in PWM device
        pstate.output_pattern = NULL;
        rc = pwm_apply_state(pwm_dev, &pstate);

	pr_err("%s rc=%d \n", __func__, rc);
        if (rc < 0)
                pr_err("Apply PWM state led failed, rc=%d\n", rc);

	pr_err("%s exit\n", __func__);
        return rc;
}

int lcd_backlight_pwm_out()
{
	return __tri_led_config_pwm(&pwm_setting);
}
*/
static irqreturn_t dsi86_irq(int irq, void *ptr)
{

	printk("dsi86_irq is working!!! \n");
	up(&dsi86_sem);

	return IRQ_HANDLED;
}

int turn_on_lcd_vcc(bool on)
{
	int ret;
	u8 value = on ? 1 : 0;

	if(gpio_is_valid(gpio_arrays[LCD_BOB_EN].gpio_num)) {
		ret = gpio_direction_output(gpio_arrays[LCD_BOB_EN].gpio_num, value);
		if (ret) {
			pr_err("unable to set %s gpio %d,  ret = %d\n", gpio_arrays[LCD_BOB_EN].gpio_name, value, ret);
			return ret;
		}
	} else {
		pr_err("LCD_BOB_EN is not valid, LCD_BOB_EN = %d\n", gpio_arrays[LCD_BOB_EN].gpio_num);
	}

	return 0;
}

int turn_on_led_vss(bool on)
{
	int ret;
	u8 value = on ? 1 : 0;
	if(gpio_is_valid(gpio_arrays[BOOST_BYPASS_EN].gpio_num)) {
		ret = gpio_direction_output(gpio_arrays[BOOST_BYPASS_EN].gpio_num, value);
		if (ret) {
			pr_err("unable to set %s gpio %d, ret = %d\n", gpio_arrays[BOOST_BYPASS_EN].gpio_name, value,  ret);
			return ret;
		}
	} else {
		pr_err("BOOST_BYPASS_EN is not valid, BOOST_BYPASS_EN = %d\n", gpio_arrays[BOOST_BYPASS_EN].gpio_num);
	}

	if(gpio_is_valid(gpio_arrays[BOOST_VSEL_EN].gpio_num)) {
		ret = gpio_direction_output(gpio_arrays[BOOST_VSEL_EN].gpio_num, value);
		if (ret) {
			pr_err("unable to set %s gpio %d, ret = %d\n", gpio_arrays[BOOST_VSEL_EN].gpio_name, value, ret);
			return ret;
		}
	} else {
		pr_err("BOOST_VSEL_EN is not valid, BOOST_VSEL_EN = %d\n", gpio_arrays[BOOST_VSEL_EN].gpio_num);
	}

	if(gpio_is_valid(gpio_arrays[BOOST_NBYP_EN].gpio_num)) {
		ret = gpio_direction_output(gpio_arrays[BOOST_NBYP_EN].gpio_num, value);
		if (ret) {
			pr_err("unable to set %s gpio %d,  ret = %d\n", gpio_arrays[BOOST_NBYP_EN].gpio_name, value, ret);
			return ret;
		}
	} else {
		pr_err("BOOST_NBYP_EN is not valid, BOOST_NBYP_EN = %d\n", gpio_arrays[BOOST_NBYP_EN].gpio_num);
	}

	return 0;
}

int turn_on_io_mux(bool on)
{
	int rc = 0;
	u8 value = on ? 0 : 1;
	//IO MUX
	if(gpio_is_valid(gpio_arrays[PANEL_GPIO_MUX_EN].gpio_num)) {
		rc = gpio_direction_output(gpio_arrays[PANEL_GPIO_MUX_EN].gpio_num, value);
		if (rc) {
			pr_err("unable to set %s %d,  rc=%d\n", gpio_arrays[PANEL_GPIO_MUX_EN].gpio_name, value,  rc);
			return rc;
		}
	} else {
		pr_err("%s is invalid, gpio = %d\n", gpio_arrays[PANEL_GPIO_MUX_EN].gpio_name, gpio_arrays[PANEL_GPIO_MUX_EN].gpio_num);
	}

	if(gpio_is_valid(gpio_arrays[IO_MUX_EN].gpio_num)) {
		rc = gpio_direction_output(gpio_arrays[IO_MUX_EN].gpio_num, value);
		if (rc) {
			pr_err("unable to set %s %d,  rc=%d\n", gpio_arrays[IO_MUX_EN].gpio_name, value,  rc);
			return rc;
		}
	} else {
		pr_err("%s is invalid, gpio = %d\n", gpio_arrays[IO_MUX_EN].gpio_name, gpio_arrays[IO_MUX_EN].gpio_num);
	}

	/*if(gpio_is_valid(gpio_arrays[ENP_MUX_EN].gpio_num)) {
		rc = gpio_direction_output(gpio_arrays[ENP_MUX_EN].gpio_num, value);
		if (rc) {
			pr_err("unable to set %s %d,  rc=%d\n", gpio_arrays[ENP_MUX_EN].gpio_name, value,  rc);
			return rc;
		}
	} else {
		pr_err("%s is invalid, gpio = %d\n", gpio_arrays[ENP_MUX_EN].gpio_name, gpio_arrays[ENP_MUX_EN].gpio_num);
	}*/

	if(gpio_is_valid(gpio_arrays[I2S_MUX_EN].gpio_num)) {
		rc = gpio_direction_output(gpio_arrays[I2S_MUX_EN].gpio_num, value);
		if (rc) {
			pr_err("unable to set %s %d,  rc=%d\n", gpio_arrays[I2S_MUX_EN].gpio_name, value,  rc);
			return rc;
		}
	} else {
		pr_err("%s is invalid, gpio = %d\n", gpio_arrays[I2S_MUX_EN].gpio_name, gpio_arrays[I2S_MUX_EN].gpio_num);
	}

	return 0;
}

int turn_on_dp_mux(bool on)
{
	int rc = 0;
	u8 value = on ? 1 : 0;

	if (dsi86_edp_mux_on == on) {
		pr_info("PS8331 Already  %d\n", on);
		return 0;
	}

	if(board_id <= HW_PLATFORM_EVT) {
		rc = gpio_direction_output(gpio_arrays[EDP_MUX_PWR_EN].gpio_num, value);
		if (rc) {
			pr_err("unable to set %s gpio to %d,  rc=%d\n", gpio_arrays[EDP_MUX_PWR_EN].gpio_name, value, rc);
			return rc;
		}
	} else {
		if(IS_ERR_OR_NULL(vcc_3p3_vreg)) {
			pr_err("PSA8331 VCC3P3 VREG NO FOUND\n");
			return rc;
		}

		/* 3V3 EN */
		if (on) {
			if (!regulator_is_enabled(vcc_3p3_vreg)) {
				regulator_set_voltage(vcc_3p3_vreg, 3300000, 3300000);
				rc = regulator_enable(vcc_3p3_vreg);
			}
		} else {
			if (regulator_is_enabled(vcc_3p3_vreg))
				rc = regulator_disable(vcc_3p3_vreg);
		}
		if (rc)
			pr_err("PS8331A unable to set VDD3V3\n");
	}

	if(IS_ERR_OR_NULL(vcc_1p5_vreg)) {
		pr_err("PSA8331 VCC1P5 VREG NO FOUND\n");
		return rc;
	} 
	
	/* 1V5 EN */
	if (on) {
		regulator_set_voltage(vcc_1p5_vreg, 1500000, 1500000);
		rc = regulator_enable(vcc_1p5_vreg);
	} else
		rc = regulator_disable(vcc_1p5_vreg);	
	if (rc)
			pr_err("PS8331A unable to set VDD15\n");

	dsi86_edp_mux_on = on;

	return 0;
}

int turn_on_lcd_vcc_loadswitch(bool on)
{
	int rc = 0;
	u8 value = on ? 1 : 0;
	ktime_t current_stime = 0;
	long long power_off_time = 0;

	if (first_probe == 0) {
		g_last_stime = ktime_get_boottime();
		if (board_id != HW_PLATFORM_EVB)
			first_probe = 1;
	} else {
		if (on) {
			current_stime = ktime_get_boottime();
			power_off_time = ktime_to_ms(ktime_sub(current_stime, g_last_stime));
			if (power_off_time < DSI86_POWER_OFF_TIME) {
				pr_err("panel 3V3 power time:%lld \n", power_off_time);
				mdelay(DSI86_POWER_OFF_TIME - power_off_time);
			}
		} else {
			g_last_stime = ktime_get_boottime();
		}
	}

	rc = gpio_direction_output(gpio_arrays[SM865_LCD_POWER_ON].gpio_num, value);
	if (rc) {
		pr_err("unable to set %s gpio to %d,  rc=%d\n", gpio_arrays[SM865_LCD_POWER_ON].gpio_name, value, rc);
		return rc;
	}

	return 0;
}

int turn_on_led_vss_loadswitch(bool on)
{
	int rc = 0;
	u8 value = on ? 1 : 0;
	//Load Swith U3211, BLK1_PWR_CONN, POWER ON GPIO37 HIGH
	rc = gpio_direction_output(gpio_arrays[SM865_BL_PWR_EN].gpio_num, value);
	if (rc) {
		pr_err("unable%s gpio rc=%d\n", gpio_arrays[SM865_BL_PWR_EN].gpio_name, rc);
		return rc;
	}

	return 0;
}

int turn_off_dsi86_by_cmd(void)
{
	int rc = 0;

	pr_err("%s enter \n", __func__);

	rc = i2c_write(DSI86_I2C_ADDR, 0x5A, 0x04);
	if (rc < 0) {
		pr_err("unable to write dsi86 reg 0x5A, rc=%d\n", rc);
		//return rc;
	}

	rc = i2c_write(DSI86_I2C_ADDR, 0x96, 0x00);
	if (rc < 0) {
		pr_err("unable to write dsi86 reg 0x96, rc=%d\n", rc);
		//return rc;
	}

	rc = i2c_write(DSI86_I2C_ADDR, 0x93, 0x00);
	if (rc < 0) {
		pr_err("unable to write dsi86 reg 0x93, rc=%d\n", rc);
		//return rc;
	}

	rc = i2c_write(DSI86_I2C_ADDR, 0x0D, 0x00);
	if (rc < 0) {
		pr_err("unable to write dsi86 reg 0x0D, rc=%d\n", rc);
		//return rc;
	}
	return rc;
}

int turn_on_dsi86(bool on)
{
	int rc = 0;
        if (dsi86_edp2mipi_on == on) {
		pr_info("TN DSI86 MIPI2EDP Already %d\n", on);
		return 0;
	}

	if(IS_ERR_OR_NULL(vcc_1p8_vreg)) {
		pr_err("TN DSI86 PSA8331 VCC1P8 VREG NO FOUND\n");
		return rc;
	}
	if(IS_ERR_OR_NULL(vcc_1p2_vreg)) {
		pr_err("TN DSI86 PSA8331 VCC1P2 VREG NO FOUND\n");
		return rc;
	}
	/* 1V8 EN */
	regulator_set_voltage(vcc_1p8_vreg, 1800000, 1800000);
	/* 1V2 EN */
	regulator_set_voltage(vcc_1p2_vreg, 1200000, 1200000);
	if(on) {
		rc = regulator_enable(vcc_1p2_vreg);
		if (rc)
			pr_err("PS8331A unable to set vcc_1p2_vreg\n");
#if 0
		rc = gpio_direction_output(gpio_arrays[MIPI_TO_EDP_1V2_EN].gpio_num, 1);
		if (rc) {
			pr_err("unable%d gpio rc=%d\n", gpio_arrays[MIPI_TO_EDP_1V2_EN].gpio_num, rc);
			return rc;
		}
#endif

		udelay(100);

		rc = regulator_enable(vcc_1p8_vreg);
		if (rc)
			pr_err("PS8331A unable to set vcc_1p8_vreg\n");
#if 0
		rc = gpio_direction_output(gpio_arrays[MIPI_TO_EDP_1V8_EN].gpio_num, 1);
		if (rc) {
			pr_err("unable%d gpio rc=%d\n", gpio_arrays[MIPI_TO_EDP_1V8_EN].gpio_num, rc);
			return rc;
		}
#endif
		if(!IS_ERR(refclk_clk)) {
			clk_prepare_enable(refclk_clk);
			pr_err("PS8331A enable bbclk2\n");
		}		
		

		mdelay(1);
		rc = gpio_direction_output(gpio_arrays[MIPI_TO_EDP_EN].gpio_num, 1);
		if (rc) {
			pr_err("unable to set %s gpio to %d,  rc=%d\n", gpio_arrays[MIPI_TO_EDP_EN].gpio_name, 1, rc);
			return rc;
		}

	} else {
		rc = gpio_direction_output(gpio_arrays[MIPI_TO_EDP_EN].gpio_num, 0);
		if (rc) {
			pr_err("unable to set %s gpio to %d,  rc=%d\n", gpio_arrays[MIPI_TO_EDP_EN].gpio_name, 0, rc);
			return rc;
		}

		if( !IS_ERR(refclk_clk)) {
			clk_disable_unprepare(refclk_clk);
			pr_err("PS8331A disable bbclk2\n");
		}

#if 0
		rc = gpio_direction_output(gpio_arrays[MIPI_TO_EDP_1V8_EN].gpio_num, 0);
		if (rc) {
			pr_err("unable%d gpio rc=%d\n", gpio_arrays[MIPI_TO_EDP_1V8_EN].gpio_num, rc);
			return rc;
		}

		rc = gpio_direction_output(gpio_arrays[MIPI_TO_EDP_1V2_EN].gpio_num, 0);
		if (rc) {
			pr_err("unable%d gpio rc=%d\n", gpio_arrays[MIPI_TO_EDP_1V2_EN].gpio_num, rc);
			return rc;
		}
#endif

		rc = regulator_disable(vcc_1p8_vreg);
		if (rc)
			pr_err("PS8331A unable to disable vcc1p8\n");
		mdelay(2);

		rc = regulator_disable(vcc_1p2_vreg);
		if (rc)
			pr_err("PS8331A unable to set VDD12\n");
	}

	dsi86_edp2mipi_on = on;

	return 0;
}

int switch_to_tv_mode(bool tv_on)
{
	int rc = 0;
	if(tv_on) {
		/* turn on dp mux loadswitch */
		rc = turn_on_dp_mux(true);
		if(rc) {
			pr_err("failed to turn on dp mux\n");
		}
		mdelay(10);

		rc = turn_on_lcd_vcc_loadswitch(false);
		if(rc) {
			pr_err("failed to turn on lcd vcc load-switch\n");
		}

		rc = turn_on_led_vss_loadswitch(false);
		if(rc) {
			pr_err("failed to turn on led vss load-switch\n");
		}

		/* turn on led_vss, lcd_vcc */
		rc = turn_on_lcd_vcc(true);
		if(rc) {
			pr_err("failed to turn on lcd_vcc\n");
		}

		rc = turn_on_led_vss(true);
		if(rc) {
			pr_err("failed to turn on led_vss\n");
		}

		/* switch io_mux, dp_mux to  port2 */
		rc = switch_to_port1(false);
		if(rc) {
			pr_err("failed to switch to port1\n");
		}

		/* enable io-mux */
		rc = turn_on_io_mux(true);
		if(rc) {
			pr_err("failed to turn on io-mux\n");
		}
	}

	return rc;
}
EXPORT_SYMBOL(switch_to_tv_mode);

static int dsi86_pwr_parse_gpios(struct device *dev)
{
	int index = 0;
	int rc = 0;
	struct device_node *np = dev->of_node;
	//memset((void*)gpio_arrays,sizeof(gpio_arrays),0);

	gpio_arrays[LCD_BOB_EN].gpio_name = "lcd-bob-en";
	gpio_arrays[EDP_MUX_PWR_EN].gpio_name = "edp-mux-pwr-en";
	gpio_arrays[EDP_AUX_PORT_SEL].gpio_name = "edp-aux-port-sel";
	gpio_arrays[PANEL_GPIO_MUX_EN].gpio_name = "panel-gpio-mux-en";

	gpio_arrays[IO_MUX_EN].gpio_name = "io-mux-en";
	gpio_arrays[IO_MUX_EN_SELECT].gpio_name = "io-mux-en-select";
	/*gpio_arrays[ENP_MUX_EN].gpio_name = "enp-mux-en";*/
	gpio_arrays[I2S_MUX_EN].gpio_name = "i2s-mux-en";
#if 0
	gpio_arrays[MIPI_TO_EDP_1V2_EN].gpio_name = "mipi-to-edp-1v2-en";
	gpio_arrays[MIPI_TO_EDP_1V8_EN].gpio_name = "mipi-to-edp-1v8-en";
#endif
	gpio_arrays[MIPI_TO_EDP_EN].gpio_name = "mipi-to-edp-en";

	gpio_arrays[BOOST_BYPASS_EN].gpio_name = "boost-bypass-en";
	gpio_arrays[BOOST_VSEL_EN].gpio_name = "boost-vsel-en";
	gpio_arrays[BOOST_NBYP_EN].gpio_name = "boost-nbyp-en";

	gpio_arrays[SM865_LCD_POWER_ON].gpio_name = "865-lcd-power-on";
	gpio_arrays[SM865_BL_PWR_EN].gpio_name = "865-bl-pwr-en";
	gpio_arrays[SM865_BL_EN].gpio_name = "865-bl-en";
	gpio_arrays[I2S_MUX_SELECT].gpio_name = "i2s-mux-select";
	gpio_arrays[DSI86_PWR_STATUS].gpio_name = "dsi86-pwr-status";

	for( index = 0; index < LCD_PWR_GPIO_MAX; index ++) {
		gpio_arrays[index].gpio_num = of_get_named_gpio(np,
						gpio_arrays[index].gpio_name, 0);
		if (!gpio_is_valid(gpio_arrays[index].gpio_num) ) {
			rc = gpio_arrays[index].gpio_num;
			pr_err("[%s] failed get gpio from dts, rc=%d\n", __func__, rc);
        	} else {
			rc = gpio_request(gpio_arrays[index].gpio_num, gpio_arrays[index].gpio_name);
			if (rc) {
				pr_err("request for %s gpio:%d failed, rc=%d\n", gpio_arrays[index].gpio_name, gpio_arrays[index].gpio_num, rc);
			} else {
				pr_err("names:%s gpio:%d requested success\n", gpio_arrays[index].gpio_name, gpio_arrays[index].gpio_num);
			}
		}
	}

	return rc;
}

int tp_get_io_mux_en_value(void) {
	if (gpio_is_valid(gpio_arrays[IO_MUX_EN_SELECT].gpio_num))
		return gpio_get_value(gpio_arrays[IO_MUX_EN_SELECT].gpio_num);
	return 0;
}

static int switch_to_port1(bool port1)
{
	int ret = 0;

	if(gpio_is_valid(gpio_arrays[IO_MUX_EN_SELECT].gpio_num)) {
		ret = gpio_direction_output(gpio_arrays[IO_MUX_EN_SELECT].gpio_num, port1 ? 0 : 1);
		if (ret) {
			pr_err("unable to set IO_MUX_EN_SELECT 1, ret = %d\n", ret);
			return ret;
		}
	} else {
		pr_err("IO_MUX_EN_SELECT is not valid, IO_MUX_EN_SELECT = %d\n", gpio_arrays[IO_MUX_EN_SELECT].gpio_num);
	}

	if(gpio_is_valid(gpio_arrays[EDP_AUX_PORT_SEL].gpio_num)) {
		ret = gpio_direction_output(gpio_arrays[EDP_AUX_PORT_SEL].gpio_num,  port1 ? 0 : 1);
		if (ret) {
			pr_err("unable to set EDP_AUX_PORT_SEL 0, ret = %d\n", ret);
			return ret;
		}
	} else {
		pr_err("EDP_AUX_PORT_SEL is not valid, EDP_AUX_PORT_SEL = %d\n", gpio_arrays[EDP_AUX_PORT_SEL].gpio_num);
	}

	if(gpio_is_valid(gpio_arrays[I2S_MUX_SELECT].gpio_num)) {
		ret = gpio_direction_output(gpio_arrays[I2S_MUX_SELECT].gpio_num, port1 ? 0 : 1);
		if (ret) {
			pr_err("unable to set I2S_MUX_SELECT 1, ret = %d\n", ret);
			return ret;
		}
	} else {
		pr_err("I2S_MUX_SELECT is not valid, I2S_MUX_SELECT = %d\n", gpio_arrays[I2S_MUX_SELECT].gpio_num);
	}

	return ret;
}

int  dsi86_power_on_pre_ulps()
{
	int rc = 0;

	pr_err("%s enter", __func__);

	tp_resume_call_by_panel();

	rc = turn_on_lcd_vcc(true);
	if (rc) {
		pr_err("failed to turn on lcd_vcc, rc = %d\n", rc);
		return rc;
	}

	//enable LCD_3V3 load switch U3205 , LCD_VCC_PANEL_3V3 POWER ON
	rc = turn_on_lcd_vcc_loadswitch(true);
	if (rc) {
		pr_err("unable to turn on vcc_loadswitch, rc = %d\n", rc);
		return rc;
	}

	rc = turn_on_dp_mux(true);
	if (rc) {
		pr_err("unable to turn on dp-mux, rc = %d\n", rc);
		return rc;
	}

	rc = turn_on_io_mux(true);
	if (rc) {
		pr_err("failed to turn on io-mux, rc = %d\n", rc);
		return rc;
	}

	rc = switch_to_port1(true);
	if (rc) {
		pr_err("failed to switch to port1, rc = %d\n", rc);
		return rc;
	}

	rc = turn_on_led_vss(true);
	if(rc) {
		pr_err("failed to turn on led_vss, rc=%d\n", rc);
		return rc;
	}

	pr_err("%s exit", __func__);
	return rc;
}

int dsi86_set_to_tps1(void)
{
	int rc = 0;

	rc = i2c_write(DSI86_I2C_ADDR, 0x96, 0x2);
	if (rc < 0) {
		pr_err("unable to write dsi86 reg 0x96, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int dsi86_power_on_post_ulps()
{
	int rc = 0;
	int i = 0;
	u8 reg_value = 0;
	int dsi86_pwr = -1;

	pr_err("%s enter", __func__);

	rc = turn_on_dsi86(true);
	if (rc) {
		pr_err("unable to turn off dsi86, rc=%d\n", rc);
		return rc;
	}

	if (board_id == HW_PLATFORM_EVB) {
		if (first_probe == 0) {
			mdelay(200);
			first_probe = 1;
		} else {
			for (i = 0; i < 20; i++)
			{
				rc = i2c_read(DSI86_I2C_ADDR, 0xF5, &reg_value);
				if (rc < 0)
					pr_err("dsi86 read F5 failed !!! \n");
				else {
					pr_err("dsi86 F5:%d, count:%d\n", reg_value, i);
					if (reg_value & 0x02) {
						rc = 0;
						break;
					}
				}
				mdelay(10);
			}
		}
	} else {
		if(gpio_is_valid(gpio_arrays[DSI86_PWR_STATUS].gpio_num)) {
			rc = gpio_direction_input(gpio_arrays[DSI86_PWR_STATUS].gpio_num);
			if (rc) {
				pr_err("unable to set gpio_direction_input, ret = %d\n", rc);
				mdelay(200);
			} else {
				for (i = 0; i < 40; i++)
				{
					dsi86_pwr = gpio_get_value(gpio_arrays[DSI86_PWR_STATUS].gpio_num);
					if (dsi86_pwr == 0) {
						pr_err("dsi86_pwr:%d gpio_num=%d i=%d\n", dsi86_pwr,gpio_arrays[DSI86_PWR_STATUS].gpio_num,i);
						break;
					}
					mdelay(5);
				}
			}
		} else {
			pr_err("DSI86_PWR_STATUS is not valid, DSI86_PWR_STATUS = %d\n", gpio_arrays[DSI86_PWR_STATUS].gpio_num);
			mdelay(200);
		}
	}
	rc = dsi86_set_to_tps1();
	if (rc) {
		pr_err("unable to set dsi86 to tps1, rc=%d\n", rc);
	}
	//msleep(200);
	//enable BOOST_BY_PASS chipset U3202 BLKT_PWR OUT

	//mdelay(20);
	// I2C2 CONFIG MIPI TO EDP ,EDP MUX, BOOST_BY_PASS
	pr_err("%s exit", __func__);
	return rc;
}

int turn_on_bl_en(bool on)
{
	int rc = 0;
	u8 value = on ? 1 : 0;

	rc = gpio_direction_output(gpio_arrays[SM865_BL_EN].gpio_num, value);
	if (rc) {
		pr_err("unable%s gpio rc=%d\n",gpio_arrays[SM865_BL_EN].gpio_name, rc);
		return rc;
	}

	return 0;
}

static int lcd_power_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
	int rc = 0;
	int irq = 0;
	struct edp_pinctrl_info *edp_pin = NULL;

	pr_err("xutao dsi86_probe enter \n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
                dev_warn(&client->dev, "xutao adapter does not support I2C\n");
                return -EIO;
        } else {
		g_i2c_client = client;
	}

	vcc_1p2_vreg = devm_regulator_get(&client->dev, "vdd-1p2");
	if (IS_ERR_OR_NULL(vcc_1p2_vreg)) {
		pr_err("PSA8331A 1p2_reg get failed\n");
		vcc_1p2_vreg = NULL;
	}

	vcc_1p5_vreg = devm_regulator_get(&client->dev, "vdd-1p5");
	if (IS_ERR_OR_NULL(vcc_1p5_vreg)) {
		pr_err("PSA8331A 1p5_reg get failed\n");
		vcc_1p5_vreg = NULL;
	}

	vcc_1p8_vreg = devm_regulator_get(&client->dev, "vdd-1p8");
	if (IS_ERR_OR_NULL(vcc_1p8_vreg)) {
		pr_err("PSA8331nA 1p8_reg get failed\n");
		vcc_1p8_vreg = NULL;
	}

	vcc_3p3_vreg = devm_regulator_get(&client->dev, "vdd-3p3");
	if (IS_ERR_OR_NULL(vcc_3p3_vreg)) {
		pr_err("PSA8331nA 3p3_reg get failed\n");
		vcc_3p3_vreg = NULL;
	} else {
		regulator_set_voltage(vcc_3p3_vreg, 3300000, 3300000);
		rc = regulator_enable(vcc_3p3_vreg);
		if (rc)
			pr_err("PSA8331A unable to set VDD3V3\n");
	}

	refclk_clk = devm_clk_get(&client->dev, "bb_clk2");
	if (IS_ERR(refclk_clk)) {
		pr_err("PSA8331A bb_clk2 get failed\n");
		refclk_clk = NULL;	
	}

	if (client->dev.of_node) {
		rc = dsi86_pwr_parse_gpios(&client->dev);
		if (rc) {
			dev_err(&client->dev, "Failed to parse dts.\n");
			return -EINVAL;
		}
	}
/*
	pwm_dev = devm_of_pwm_get(&client->dev, node, NULL);
	if (IS_ERR(pwm_dev)) {
		rc = PTR_ERR(pwm_dev);
		if (rc != -EPROBE_DEFER)
			dev_err(&client->dev, "Get pwm device for failed, rc=%d\n", rc);
                        return rc;

		dev_err(&client->dev, "Get pwm device for failed, rc=%d\n", rc);
	}
	pwm_get_args(pwm_dev, &pargs);
*/
        //if (pargs.period == 0)
		//pwm_setting.pre_period_ns = PWM_PERIOD_DEFAULT_NS;
	//else
		//pwm_setting.pre_period_ns = pargs.period;
	//pr_err("dsi86_probe success %d pre_period=%ld \n", rc, pwm_setting.pre_period_ns);

	irq = irq_of_parse_and_map(client->dev.of_node, 0);
        if (irq < 0) {
                dev_err(&client->dev, "failed to get irq: %d\n", -1);
                return -1;
        }

	pr_err("found irq = %d \n", irq);

        rc = devm_request_irq(&client->dev, irq,
                        dsi86_irq, IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
                        "dsi86_isr", NULL);
        if (rc < 0) {
                dev_err(&client->dev, "failed to request IRQ%u: %d\n",
                                irq, rc);
                return rc;
        } else {
		pr_err("devm_request_irq success \n");
	}

	if(gpio_is_valid(gpio_arrays[DSI86_PWR_STATUS].gpio_num)) {
		rc = gpio_direction_input(gpio_arrays[DSI86_PWR_STATUS].gpio_num);
		if (rc) {
			pr_err("unable to set gpio_direction_input, ret = %d\n", rc);
			goto error;
		}
		edp_pin = kzalloc(sizeof(*edp_pin), GFP_KERNEL);
		if (!edp_pin)
			return -1;

		edp_pin->pinctrl = devm_pinctrl_get(&client->dev);
		if (IS_ERR_OR_NULL(edp_pin->pinctrl)) {
			rc = PTR_ERR(edp_pin->pinctrl);
			pr_err("dsi86 failed to get pinctrl, rc=%d\n", rc);
			goto error;
		}

		edp_pin->active = pinctrl_lookup_state(edp_pin->pinctrl,
							       "dsi86_pwr_active");
		if (IS_ERR_OR_NULL(edp_pin->active)) {
			rc = PTR_ERR(edp_pin->active);
			pr_err("dsi86 failed to get pinctrl active state, rc=%d\n", rc);
			goto error;
		}

		edp_pin->suspend =
			pinctrl_lookup_state(edp_pin->pinctrl, "dsi86_pwr_suspend");

		if (IS_ERR_OR_NULL(edp_pin->suspend)) {
			rc = PTR_ERR(edp_pin->suspend);
			pr_err("dsi86 failed to get pinctrl suspend state, rc=%d\n", rc);
			goto error;
		}

		rc = pinctrl_select_state(edp_pin->pinctrl, edp_pin->active);
		if (rc)
			pr_err("dsi86 failed to set pin state, rc=%d\n", rc);
	} else {
		pr_err("dsi86 power status pin not defined\n");
	}
error:
	return rc;
}

static struct i2c_driver lcd_power_driver = {
    .driver = {
        .name    = "dsi86-edp-mux",
        .of_match_table = of_match_ptr(edp_mux_of_match),
    },
    .probe        = lcd_power_probe,
};

static int __init lcd_power_init(void)
{
	sema_init(&dsi86_sem, 0);
	if (board_id == -1) {
		board_id = hw_get_platform_type();
	}

	return i2c_add_driver(&lcd_power_driver);
}

subsys_initcall(lcd_power_init);

static void __exit lcd_power_exit(void)
{
	i2c_del_driver(&lcd_power_driver);
}
module_exit(lcd_power_exit);

MODULE_AUTHOR("some one  <l-felten@ti.com>");
MODULE_DESCRIPTION("dsi86 driver");
MODULE_LICENSE("GPL");
