/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spmi.h>
#include <linux/bootinfo.h>

#define QPNP_VREG_DEBUG_DRIVER_NAME "qcom,qpnp-vreg-debug"

/* To-Do:Pass VREG addr/cnt via dts */
#define QPNP_VREG_BASE_ADDR		0x1400
#define QPNP_VREG_END_ADDR		0xED00
#define QPNP_VREG_PHY_OFFS		0x100
#define QPNP_VREG_REGS_COUNT		8 /* Indeed 7 + 1 */
#define QPNP_VREG_PMIC_COUNT		3 /* Indeed 2 + 1 */

enum qpnp_regulator_type {
        QPNP_REG_TYPE_BUCK	= 0x03,
        QPNP_REG_TYPE_LDO	= 0x04,
        QPNP_REG_TYPE_VS	= 0x05,
        QPNP_REG_TYPE_BOOST	= 0x1b,
        QPNP_REG_TYPE_FTS	= 0x1c,
        QPNP_REG_TYPE_BOOST_BYP = 0x1f,
	QPNP_REG_TYPE_IBB	= 0x20,
        QPNP_REG_TYPE_ULT_LDO   = 0x21,
        QPNP_REG_TYPE_ULT_BUCK  = 0x22,
	QPNP_REG_TYPE_AB	= 0x24,
        QPNP_REG_TYPE_BOB	= 0x29,
	QPNP_REG_TYPE_OLEDB	= 0x2c,
	QPNP_REG_TYPE_LCDB	= 0x2d,
};


enum qpnp_regulator_registers {
        QPNP_REG_TYPES		= 0x04,
        QPNP_REG_STATUS		= 0x08,
        QPNP_REG_VOUT_LB	= 0x40,
        QPNP_REG_VOUT_UB	= 0x41,
        QPNP_REG_MODE_CTRL	= 0x45,
        QPNP_REG_ENABLE		= 0x46,
        QPNP_REG_FOLLOW_HW	= 0x47,
};


struct qpnp_vreg_debug {
	struct device		*dev;
        struct regmap           *regmap;
	u8			pmic_uid;
	u8     			regs[QPNP_VREG_REGS_COUNT];
};


static int vreg_pmic_idx;
static struct qpnp_vreg_debug *qpnp_vreg_dev[QPNP_VREG_PMIC_COUNT];

static int qpnp_vreg_debug_read(struct qpnp_vreg_debug *chip, u16 addr, u8 *value, u8 count)
{
	int rc = 0;

	rc = regmap_bulk_read(chip->regmap, addr, value, count);
	if (rc < 0)
		pr_debug("Failed to read from addr=0x%02x rc=%d\n", addr, rc);

	return rc;
}

static bool qpnp_vreg_type_valid(u8 vreg_type)
{
	bool vreg_valid = false;

	switch (vreg_type) {
	case QPNP_REG_TYPE_BUCK:
	case QPNP_REG_TYPE_LDO:
	case QPNP_REG_TYPE_VS:
	case QPNP_REG_TYPE_BOOST:
	case QPNP_REG_TYPE_FTS:
	case QPNP_REG_TYPE_BOOST_BYP:
	case QPNP_REG_TYPE_IBB:
	case QPNP_REG_TYPE_ULT_LDO:
	case QPNP_REG_TYPE_ULT_BUCK:
	case QPNP_REG_TYPE_AB:
	case QPNP_REG_TYPE_BOB:
	case QPNP_REG_TYPE_OLEDB:
	case QPNP_REG_TYPE_LCDB:
		vreg_valid = true;
		break;
	default:
		break;
	}

	return vreg_valid;
}

static int qpnp_vreg_dump_vreg_sts(struct qpnp_vreg_debug *chip, u16 base_addr)
{
	int rc = 0;
	/* ATTENTION: Max IDX = QPNP_VREG_REGS_COUNT - 1 */
	rc = qpnp_vreg_debug_read(chip, (base_addr + QPNP_REG_TYPES), &chip->regs[0], 1);
	if (rc)
		return rc;

	if (!qpnp_vreg_type_valid(chip->regs[0]))
		return -EINVAL;

	rc = qpnp_vreg_debug_read(chip, (base_addr + QPNP_REG_STATUS), &chip->regs[1], 1);
	if (rc)
		return rc;

	rc = qpnp_vreg_debug_read(chip, (base_addr + QPNP_REG_VOUT_LB), &chip->regs[2], 1);
	if (rc)
		return rc;

	rc = qpnp_vreg_debug_read(chip, (base_addr + QPNP_REG_VOUT_UB), &chip->regs[3], 1);
	if (rc)
		return rc;

	rc = qpnp_vreg_debug_read(chip, (base_addr + QPNP_REG_MODE_CTRL), &chip->regs[4], 1);
	if (rc)
		return rc;

	rc = qpnp_vreg_debug_read(chip, (base_addr + QPNP_REG_ENABLE), &chip->regs[5], 1);
	if (rc)
		return rc;

	rc = qpnp_vreg_debug_read(chip, (base_addr + QPNP_REG_FOLLOW_HW), &chip->regs[6], 1);
	return rc;
}

/* SMPS/LDO Status Raw Data Printf to Sync with NON_HLOS */
void qpnp_vreg_dump_vregs(void)
{
	int i = 0;
	u16 vreg_addr = 0;
	struct qpnp_vreg_debug *chip = NULL;

	if (!hw_pm_debug_enable())
		return;

	for (i = 0; i < QPNP_VREG_PMIC_COUNT; i++) {
		chip = qpnp_vreg_dev[i];
		if (!chip)
			continue;

		pr_info("DUMP PMIC SID@0x%x VREG Status -------\n", chip->pmic_uid);
		for (vreg_addr = QPNP_VREG_BASE_ADDR; vreg_addr < QPNP_VREG_END_ADDR;
			vreg_addr +=  QPNP_VREG_PHY_OFFS) {

			if (qpnp_vreg_dump_vreg_sts(chip, vreg_addr))
				continue;

			pr_info("ADDR:0x%4x TYP: 0x%2x STS:0x%2x LB:0x%2x UB:0x%2x MDE:0x%2x ENB:0x%2x FHW:0x%2x\n",
				vreg_addr, chip->regs[0], chip->regs[1], chip->regs[2], chip->regs[3],
				chip->regs[4], chip->regs[5], chip->regs[6]);
		}
	}
}
EXPORT_SYMBOL(qpnp_vreg_dump_vregs);

static int qpnp_vreg_debug_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device_node *node = NULL;
	struct qpnp_vreg_debug *chip = NULL;


	node = pdev->dev.of_node;
	if (!node) {
		pr_err("No nodes defined\n");
		return -ENODEV;
	}

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		dev_err(&pdev->dev, "Failed to get the regmap handle\n");
		rc = -EINVAL;
		goto error;
	}

	chip->pmic_uid = to_spmi_device(pdev->dev.parent)->usid;

	dev_set_drvdata(&pdev->dev, chip);

	if (vreg_pmic_idx >= QPNP_VREG_PMIC_COUNT) {
		dev_err(&pdev->dev, "vreg_pmic_idx %d too large\n", vreg_pmic_idx);
		rc = -EINVAL;
		goto error;
	}

	qpnp_vreg_dev[vreg_pmic_idx] = chip;

	pr_info("PMIC ID:0x%2x SID:0x%02x VREG DBG Registered Successfully!\n",
			vreg_pmic_idx, qpnp_vreg_dev[vreg_pmic_idx]->pmic_uid);

	vreg_pmic_idx++;

error:
	return rc;
}

static int qpnp_vreg_debug_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id vreg_debug_table[] = {
	{ .compatible = QPNP_VREG_DEBUG_DRIVER_NAME, },
	{ },
};

static struct platform_driver qpnp_vreg_debug_driver = {
	.driver		= {
		.name		= QPNP_VREG_DEBUG_DRIVER_NAME,
		.of_match_table	= vreg_debug_table,
	},
	.probe		= qpnp_vreg_debug_probe,
	.remove		= qpnp_vreg_debug_remove,
};

static int __init qpnp_vreg_debug_init(void)
{
	return platform_driver_register(&qpnp_vreg_debug_driver);
}

static void __exit qpnp_vreg_debug_exit(void)
{
	platform_driver_unregister(&qpnp_vreg_debug_driver);
}

MODULE_DESCRIPTION("QPNP HW VREG Debug driver");
MODULE_LICENSE("GPL v2");

module_init(qpnp_vreg_debug_init);
module_exit(qpnp_vreg_debug_exit);
