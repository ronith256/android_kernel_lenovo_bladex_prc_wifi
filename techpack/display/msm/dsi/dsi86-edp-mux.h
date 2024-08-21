#ifndef _DSI86_EDP_MUX_H_

enum POWER_GPIOS{
	LCD_BOB_EN,		//GPIO71
	EDP_MUX_PWR_EN, 	//GPIO42 
	EDP_AUX_PORT_SEL, 	//GPIO40
	PANEL_GPIO_MUX_EN,	//GPIO23
	IO_MUX_EN, 		//GPIO67
	IO_MUX_EN_SELECT, 	//GPIO108
	I2S_MUX_SELECT,		//GPIO104
/*	ENP_MUX_EN, 		//GPIO27*/
	I2S_MUX_EN, 		// GPIO14
#if 0
	MIPI_TO_EDP_1V2_EN, 	//GPIO98
	MIPI_TO_EDP_1V8_EN, 	//GPIO13
#endif
	MIPI_TO_EDP_EN, 	//GPIO12
	BOOST_BYPASS_EN, 	//GPIO46
	SM865_LCD_POWER_ON, 	//GPIO101
	SM865_BL_PWR_EN, 	//GPIO9
	SM865_BL_EN, 		//GPIO102
	BOOST_VSEL_EN, 		//GPIO48
	BOOST_NBYP_EN, 		//GPIO49
	DSI86_PWR_STATUS,       //GPIO72
	LCD_PWR_GPIO_MAX
};

struct edp_gpio_info {
	char* gpio_name;
	int gpio_num;
};

struct edp_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *active;
	struct pinctrl_state *suspend;
};

int turn_on_dp_mux(bool on);
int turn_on_lcd_vcc_loadswitch(bool on);
int turn_on_led_vss_loadswitch(bool on);
int turn_on_bl_en(bool on);

int turn_on_led_vss(bool on);
int turn_on_io_mux(bool on);
int turn_on_dp_mux(bool on);
int turn_off_dsi86_by_cmd(void);
int turn_on_dsi86(bool on);
int turn_on_lcd_vcc(bool on);

int switch_to_tv_mode(bool tv_on);
int dsi86_power_on_pre_ulps(void);
int dsi86_power_on_post_ulps(void);
int dsi86_backlight_power(int flags);
int dsi86_edp_regs_init(void);
int dsi86_edp_mux_init(void);

int switch_to_tv_mode(bool tv_on);
extern void tp_resume_call_by_panel(void);
extern void tp_suspend_call_by_panel(void);
extern int tp_get_io_mux_en_value(void);
#endif
