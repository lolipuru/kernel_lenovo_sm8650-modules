// SPDX-License-Identifier: GPL-2.0-only
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "cam_cci_dev.h"
#include "cam_power_dev.h"
#include "cam_req_mgr.h"
#include "cam_sensor_io.h"
#include "cam_subdev.h"

#define CAM_POWER_OUTPUTS 7
#define CAM_POWER_LOW_VOLTAGE_OUTPUT 1
#define CAM_POWER_WL_ADDRESS 0x2f
#define CAM_POWER_ET_ADDRESS 0x35

enum cam_power_provider {
	CAM_POWER_KIRBY_WL_I2C,
	CAM_POWER_KIRBY_ET_I2C,
	CAM_POWER_KIRBY_CCI,
	CAM_POWER_LAPIS_I2C,
	CAM_POWER_PROVIDER_COUNT,
};

struct cam_power_chip {
	u8 voltage_base;
	u8 voltage[CAM_POWER_OUTPUTS];
	u8 low_voltage;
	u8 enable_reg;
	u8 idle_enable;
};

static const struct cam_power_chip cam_power_wl = {
	.voltage_base = 0x03,
	.voltage = { 0x5a, 0x5a, 0xa2, 0xa2, 0xa2, 0xa2, 0x2c },
	.low_voltage = 0x4d,
	.enable_reg = 0x0e,
	.idle_enable = 0x80,
};

static const struct cam_power_chip cam_power_et = {
	.voltage_base = 0x04,
	.voltage = { 0x66, 0x66, 0xa0, 0xa0, 0xa0, 0xa0, 0x41 },
	.low_voltage = 0x55,
	.enable_reg = 0x03,
};

struct cam_power {
	struct device *dev;
	struct i2c_client *i2c;
	struct cam_sensor_cci_client cci;
	struct camera_io_master io;
	struct cam_subdev subdev;
	struct gpio_desc *enable_gpio;
	struct regulator *vin1;
	struct regulator *vin2;
	struct regulator *vin3;
	const struct cam_power_chip *chip;
	enum cam_power_provider provider;
	u32 generation;
	unsigned int users[CAM_POWER_OUTPUTS];
	u8 enabled;
	bool enable_dirty;
	bool inputs_enabled;
	bool pullup_enabled;
	bool low_voltage;
};

static DEFINE_MUTEX(cam_power_lock);
static struct cam_power *cam_power_providers[CAM_POWER_PROVIDER_COUNT];
static struct cam_power *cam_power_active;
static u32 cam_power_generation;

static int cam_power_read(struct cam_power *power, u8 reg, u8 *value)
{
	u32 data;
	int rc;

	if (power->i2c) {
		rc = i2c_smbus_read_byte_data(power->i2c, reg);
		if (rc < 0)
			return rc;

		*value = rc;
		return 0;
	}

	rc = camera_io_dev_read(&power->io, reg, &data,
		CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE,
		false);
	if (!rc)
		*value = data;

	return rc;
}

static int cam_power_write(struct cam_power *power, u8 reg, u8 value)
{
	struct cam_sensor_i2c_reg_array entry = {
		.reg_addr = reg,
		.reg_data = value,
	};
	struct cam_sensor_i2c_reg_setting setting = {
		.reg_setting = &entry,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	};

	if (power->i2c)
		return i2c_smbus_write_byte_data(power->i2c, reg, value);

	return camera_io_dev_write(&power->io, &setting);
}

static int cam_power_enable_input(struct regulator *supply, int uv)
{
	int rc;

	if (!supply)
		return 0;

	rc = regulator_set_voltage(supply, uv, uv);
	if (rc)
		return rc;

	rc = regulator_enable(supply);
	if (rc)
		regulator_set_voltage(supply, 0, uv);

	return rc;
}

static void cam_power_disable_input(struct cam_power *power,
	struct regulator *supply, int uv)
{
	int rc;

	if (!supply)
		return;

	rc = regulator_disable(supply);
	if (!rc)
		rc = regulator_set_voltage(supply, 0, uv);
	if (rc)
		dev_warn(power->dev, "Failed to release LDO input: %d\n", rc);
}

static int cam_power_inputs_on(struct cam_power *power)
{
	int rc;

	if (power->inputs_enabled)
		return 0;

	rc = cam_power_enable_input(power->vin1, 1856000);
	if (rc)
		return rc;

	rc = cam_power_enable_input(power->vin2, 3296000);
	if (rc) {
		cam_power_disable_input(power, power->vin1, 1856000);
		return rc;
	}

	power->inputs_enabled = true;
	return 0;
}

static void cam_power_inputs_off(struct cam_power *power)
{
	if (!power->inputs_enabled)
		return;

	cam_power_disable_input(power, power->vin2, 3296000);
	cam_power_disable_input(power, power->vin1, 1856000);
	power->inputs_enabled = false;
}

static void cam_power_pullup_off(struct cam_power *power)
{
	if (power->pullup_enabled) {
		cam_power_disable_input(power, power->vin3, 1800000);
		power->pullup_enabled = false;
	}
}

static int cam_power_open(struct cam_power *power)
{
	int rc;

	if (power->i2c)
		return 0;

	if (!power->pullup_enabled) {
		rc = cam_power_enable_input(power->vin3, 1800000);
		if (rc)
			return rc;

		power->pullup_enabled = true;
		usleep_range(100, 110);
	}

	rc = camera_io_init(&power->io);
	if (rc) {
		if (!power->enabled && !power->enable_dirty)
			cam_power_pullup_off(power);
		return rc;
	}

	usleep_range(500, 510);
	return 0;
}

static void cam_power_close(struct cam_power *power)
{
	if (!power->i2c) {
		camera_io_release(&power->io);
		if (!power->enabled && !power->enable_dirty)
			cam_power_pullup_off(power);
	}
}

static int cam_power_write_outputs(struct cam_power *power, u8 outputs)
{
	int rc, retries = 3;

	do {
		rc = cam_power_write(power, power->chip->enable_reg,
			power->chip->idle_enable | outputs);
	} while (rc && --retries);

	power->enable_dirty = rc != 0;
	return rc;
}

static int cam_power_identify(struct cam_power *power,
	const struct cam_power_chip *chip)
{
	u8 id;
	int rc;

	rc = cam_power_read(power, 0, &id);
	if (rc)
		return rc;

	if (chip == &cam_power_et)
		return id == 0x01 ? 0 : -ENODEV;

	return id == 0x33 || id == 0x82 ? 0 : -ENODEV;
}

static int cam_power_init_chip(struct cam_power *power,
	const struct cam_power_chip *chip)
{
	int output, rc;

	rc = cam_power_write(power, chip->enable_reg, chip->idle_enable);
	if (rc)
		return rc;

	for (output = 0; output < CAM_POWER_OUTPUTS; output++) {
		rc = cam_power_write(power, chip->voltage_base + output,
			chip->voltage[output]);
		if (rc)
			return rc;

		/* Stock WL2868C sets these registers on Lapis and CCI. */
		if (chip == &cam_power_wl &&
		    power->provider != CAM_POWER_KIRBY_WL_I2C) {
			rc = cam_power_write(power, 0x11 + 2 * output, 0x80);
			if (rc)
				return rc;
		}
	}

	if (power->provider == CAM_POWER_KIRBY_WL_I2C ||
	    power->provider == CAM_POWER_KIRBY_ET_I2C) {
		rc = cam_power_write(power,
			chip->voltage_base + CAM_POWER_LOW_VOLTAGE_OUTPUT,
			chip->low_voltage);
		if (rc)
			return rc;
	}

	power->chip = chip;
	dev_info(power->dev, "%s camera LDO ready over %s\n",
		chip == &cam_power_et ? "ET5907" : "WL286x",
		power->i2c ? "I2C" : "CCI");
	return 0;
}

static void cam_power_reset_gpio(struct cam_power *power)
{
	if (!power->enable_gpio)
		return;

	gpiod_set_value_cansleep(power->enable_gpio, 0);
	if (power->i2c)
		usleep_range(2000, 3000);
	else
		usleep_range(200, 210);
	gpiod_set_value_cansleep(power->enable_gpio, 1);
	if (power->i2c)
		usleep_range(3000, 4000);
	else
		usleep_range(1000, 1110);
}

static int cam_power_get_supply(struct cam_power *power, const char *name,
	struct regulator **supply)
{
	struct regulator *regulator;

	if (*supply)
		return 0;

	regulator = devm_regulator_get(power->dev, name);
	if (IS_ERR(regulator))
		return PTR_ERR(regulator);

	*supply = regulator;
	return 0;
}

static int cam_power_prepare_cci(struct cam_power *power)
{
	const struct cam_power_chip *chip = &cam_power_wl;
	int rc;

	if (!cam_cci_get_subdev(power->cci.cci_device))
		return -EPROBE_DEFER;

	rc = cam_power_get_supply(power, "vin1", &power->vin1);
	if (rc)
		return rc;
	rc = cam_power_get_supply(power, "vin2", &power->vin2);
	if (rc)
		return rc;
	rc = cam_power_get_supply(power, "vin3", &power->vin3);
	if (rc)
		return rc;

	if (!power->enable_gpio) {
		struct gpio_desc *gpio;

		gpio = devm_gpiod_get_optional(power->dev, "en", GPIOD_OUT_HIGH);
		if (IS_ERR(gpio))
			return PTR_ERR(gpio);
		power->enable_gpio = gpio;
	}

	rc = cam_power_inputs_on(power);
	if (rc)
		return rc;

	cam_power_reset_gpio(power);
	power->cci.sid = CAM_POWER_WL_ADDRESS;
	rc = cam_power_open(power);
	if (rc)
		goto disable_inputs;

	rc = cam_power_identify(power, chip);
	if (rc) {
		power->cci.sid = CAM_POWER_ET_ADDRESS;
		chip = &cam_power_et;
		rc = cam_power_identify(power, chip);
	}
	if (!rc)
		rc = cam_power_init_chip(power, chip);

	cam_power_close(power);
	if (!rc)
		return 0;

disable_inputs:
	cam_power_inputs_off(power);
	return rc;
}

static int cam_power_add_locked(struct cam_power *power)
{
	if (cam_power_providers[power->provider])
		return -EBUSY;

	if (!++cam_power_generation)
		++cam_power_generation;
	power->generation = cam_power_generation;
	cam_power_providers[power->provider] = power;
	return 0;
}

static void cam_power_remove(struct cam_power *power)
{
	int rc;

	mutex_lock(&cam_power_lock);
	if (cam_power_providers[power->provider] != power)
		goto unlock;

	if (power->enabled || power->enable_dirty) {
		power->enabled = 0;
		rc = cam_power_open(power);
		if (!rc) {
			rc = cam_power_write_outputs(power, 0);
			cam_power_close(power);
		}
		if (rc)
			dev_warn(power->dev, "Failed to shut down camera LDO: %d\n", rc);
	}
	power->enable_dirty = false;
	memset(power->users, 0, sizeof(power->users));
	cam_power_providers[power->provider] = NULL;
	if (cam_power_active == power)
		cam_power_active = NULL;
	cam_power_pullup_off(power);
	cam_power_inputs_off(power);
	if (power->provider == CAM_POWER_LAPIS_I2C)
		gpiod_set_value_cansleep(power->enable_gpio, 0);

unlock:
	mutex_unlock(&cam_power_lock);
}

static struct cam_power *cam_power_select(void)
{
	int provider;

	if (cam_power_active)
		return cam_power_active;

	for (provider = 0; provider < CAM_POWER_PROVIDER_COUNT; provider++)
		if (cam_power_providers[provider])
			return cam_power_providers[provider];

	return NULL;
}

int cam_power_rail_enable(struct cam_sensor_power_setting *setting)
{
	bool low_voltage = setting->seq_type == SENSOR_WL2868C_LDO1_1P1;
	unsigned int output = low_voltage ? CAM_POWER_LOW_VOLTAGE_OUTPUT :
		setting->seq_type - SENSOR_WL2868C_LDO1;
	struct cam_power *power;
	const struct cam_power_chip *chip;
	u8 voltage;
	int rc = 0;

	mutex_lock(&cam_power_lock);
	power = cam_power_select();
	if (!power) {
		rc = -ENODEV;
		goto unlock;
	}

	if (setting->ldo_generation == power->generation)
		goto unlock;

	if (!power->chip) {
		rc = cam_power_prepare_cci(power);
		if (rc)
			goto unlock;
	}

	/* A failed shutdown may have left a rail enabled without any owner. */
	if (power->enable_dirty) {
		rc = cam_power_open(power);
		if (rc)
			goto unlock;
		rc = cam_power_write_outputs(power, power->enabled);
		cam_power_close(power);
		if (rc)
			goto unlock;
		if (!power->enabled)
			cam_power_active = NULL;
	}

	if (power->users[output]) {
		if (output == CAM_POWER_LOW_VOLTAGE_OUTPUT &&
		    power->low_voltage != low_voltage) {
			rc = -EBUSY;
			goto unlock;
		}
		goto acquire;
	}

	rc = cam_power_open(power);
	if (rc)
		goto unlock;

	chip = power->chip;
	voltage = low_voltage ? chip->low_voltage : chip->voltage[output];
	rc = cam_power_write(power, chip->voltage_base + output, voltage);
	if (!rc) {
		rc = cam_power_write_outputs(power, power->enabled | BIT(output));
		if (!rc)
			power->enabled |= BIT(output);
		else if (cam_power_write_outputs(power, power->enabled))
			cam_power_active = power;
	}
	cam_power_close(power);
	if (rc)
		goto unlock;

	if (output == CAM_POWER_LOW_VOLTAGE_OUTPUT)
		power->low_voltage = low_voltage;

acquire:
	power->users[output]++;
	setting->ldo_generation = power->generation;
	cam_power_active = power;
unlock:
	mutex_unlock(&cam_power_lock);
	return rc;
}

int cam_power_rail_disable(struct cam_sensor_power_setting *setting)
{
	unsigned int output = setting->seq_type == SENSOR_WL2868C_LDO1_1P1 ?
		CAM_POWER_LOW_VOLTAGE_OUTPUT :
		setting->seq_type - SENSOR_WL2868C_LDO1;
	struct cam_power *power;
	int rc = 0;

	mutex_lock(&cam_power_lock);
	power = cam_power_active;
	if (!setting->ldo_generation || !power ||
	    setting->ldo_generation != power->generation) {
		setting->ldo_generation = 0;
		goto unlock;
	}

	setting->ldo_generation = 0;
	if (!--power->users[output]) {
		power->enabled &= ~BIT(output);
		power->enable_dirty = true;
		rc = cam_power_open(power);
		if (rc)
			goto unlock;

		rc = cam_power_write_outputs(power, power->enabled);
		cam_power_close(power);
	}

	if (!power->enabled && !power->enable_dirty)
		cam_power_active = NULL;

unlock:
	mutex_unlock(&cam_power_lock);
	return rc;
}

static const struct of_device_id cam_power_i2c_match[] = {
	{ .compatible = "vendor,wl2868c", .data = &cam_power_wl },
	{ .compatible = "vendor,et5907", .data = &cam_power_et },
	{ .compatible = "qcom,ic_wl2868c", .data = &cam_power_wl },
	{ }
};
MODULE_DEVICE_TABLE(of, cam_power_i2c_match);

static const struct i2c_device_id cam_power_i2c_ids[] = {
	{ "vendor,wl2868c", 0 },
	{ "vendor,et5907", 0 },
	{ "qcom,ic_wl2868c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cam_power_i2c_ids);

static int cam_power_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	const struct of_device_id *match;
	const struct cam_power_chip *chip;
	struct cam_power *power;
	bool lapis;
	int rc;

	match = of_match_device(cam_power_i2c_match, dev);
	if (!match)
		return -ENODEV;

	lapis = of_device_is_compatible(dev->of_node, "qcom,ic_wl2868c");
	if ((lapis && !IS_ENABLED(CONFIG_ARCH_LAPIS)) ||
	    (!lapis && !IS_ENABLED(CONFIG_ARCH_KIRBY)))
		return -ENODEV;

	power = devm_kzalloc(dev, sizeof(*power), GFP_KERNEL);
	if (!power)
		return -ENOMEM;

	chip = match->data;
	power->dev = dev;
	power->i2c = client;
	if (lapis) {
		power->provider = CAM_POWER_LAPIS_I2C;
		power->enable_gpio = devm_gpiod_get(dev, "en", GPIOD_OUT_HIGH);
		if (IS_ERR(power->enable_gpio))
			return dev_err_probe(dev, PTR_ERR(power->enable_gpio),
				"Failed to get camera LDO enable GPIO\n");
	} else {
		power->provider = chip == &cam_power_et ?
			CAM_POWER_KIRBY_ET_I2C : CAM_POWER_KIRBY_WL_I2C;
		rc = cam_power_get_supply(power, "vin1", &power->vin1);
		if (rc)
			return dev_err_probe(dev, rc, "Failed to get camera LDO vin1\n");
		if (chip == &cam_power_wl) {
			rc = cam_power_get_supply(power, "vin2", &power->vin2);
			if (rc)
				return dev_err_probe(dev, rc, "Failed to get camera LDO vin2\n");
		}
	}

	mutex_lock(&cam_power_lock);
	if (cam_power_providers[power->provider]) {
		rc = -EBUSY;
		goto unlock;
	}

	/* The two Kirby transports may address the same physical chip. */
	if (cam_power_active && !cam_power_active->i2c) {
		rc = -EPROBE_DEFER;
		goto unlock;
	}
	rc = cam_power_inputs_on(power);
	if (rc)
		goto unlock;

	cam_power_reset_gpio(power);
	rc = cam_power_identify(power, chip);
	if (!rc)
		rc = cam_power_init_chip(power, chip);
	if (!rc)
		rc = cam_power_add_locked(power);
	if (rc)
		cam_power_inputs_off(power);

unlock:
	mutex_unlock(&cam_power_lock);
	if (rc) {
		if (lapis)
			gpiod_set_value_cansleep(power->enable_gpio, 0);
		return dev_err_probe(dev, rc, "Failed to probe camera LDO\n");
	}

	i2c_set_clientdata(client, power);
	return 0;
}

static void cam_power_i2c_remove(struct i2c_client *client)
{
	cam_power_remove(i2c_get_clientdata(client));
}

static struct i2c_driver cam_power_i2c_driver = {
	.driver = {
		.name = "lenovo-camera-ldo",
		.of_match_table = cam_power_i2c_match,
	},
	.probe_new = cam_power_i2c_probe,
	.remove = cam_power_i2c_remove,
	.id_table = cam_power_i2c_ids,
};

static const struct v4l2_subdev_ops cam_power_subdev_ops;

static int cam_power_cci_bind(struct device *dev, struct device *master,
	void *data)
{
	struct cam_power *power = dev_get_drvdata(dev);
	int rc;

	power->subdev.pdev = to_platform_device(dev);
	power->subdev.ops = &cam_power_subdev_ops;
	power->subdev.name = "wl2868c";
	power->subdev.sd_flags = V4L2_SUBDEV_FL_HAS_DEVNODE |
		V4L2_SUBDEV_FL_HAS_EVENTS;
	power->subdev.token = power;
	power->subdev.ent_function = CAM_POWER_DEVICE_TYPE;
	power->subdev.close_seq_prior = CAM_SD_CLOSE_LOW_PRIORITY;

	mutex_lock(&cam_power_lock);
	rc = cam_power_add_locked(power);
	mutex_unlock(&cam_power_lock);
	if (rc)
		return rc;

	rc = cam_register_subdev(&power->subdev);
	if (rc)
		cam_power_remove(power);

	return rc;
}

static void cam_power_cci_unbind(struct device *dev, struct device *master,
	void *data)
{
	struct cam_power *power = dev_get_drvdata(dev);

	cam_unregister_subdev(&power->subdev);
	cam_power_remove(power);
	power->chip = NULL;
}

static const struct component_ops cam_power_component_ops = {
	.bind = cam_power_cci_bind,
	.unbind = cam_power_cci_unbind,
};

static int cam_power_cci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cam_power *power;
	u32 cci_device = CCI_DEVICE_1;

	power = devm_kzalloc(dev, sizeof(*power), GFP_KERNEL);
	if (!power)
		return -ENOMEM;

	power->dev = dev;
	power->provider = CAM_POWER_KIRBY_CCI;
	if (dev->of_node->parent)
		of_property_read_u32(dev->of_node->parent, "cell-index", &cci_device);
	if (cci_device >= CCI_DEVICE_MAX)
		return -EINVAL;
	power->cci.cci_device = cci_device;
	power->cci.cci_i2c_master = MASTER_1;
	power->cci.i2c_freq_mode = I2C_FAST_MODE;
	power->cci.retries = 3;
	power->io.master_type = CCI_MASTER;
	power->io.cci_client = &power->cci;

	platform_set_drvdata(pdev, power);
	return component_add(dev, &cam_power_component_ops);
}

static int cam_power_cci_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_power_component_ops);
	return 0;
}

static const struct of_device_id cam_power_cci_match[] = {
	{ .compatible = "lenovo,wl2868c" },
	{ }
};
MODULE_DEVICE_TABLE(of, cam_power_cci_match);

struct platform_driver cam_power_platform_driver = {
	.driver = {
		.name = "lenovo-camera-ldo-cci",
		.of_match_table = cam_power_cci_match,
	},
	.probe = cam_power_cci_probe,
	.remove = cam_power_cci_remove,
};

int cam_power_driver_init(void)
{
	int rc;

	rc = i2c_add_driver(&cam_power_i2c_driver);
	if (rc || !IS_ENABLED(CONFIG_ARCH_KIRBY))
		return rc;

	rc = platform_driver_register(&cam_power_platform_driver);
	if (rc)
		i2c_del_driver(&cam_power_i2c_driver);

	return rc;
}

void cam_power_driver_exit(void)
{
	if (IS_ENABLED(CONFIG_ARCH_KIRBY))
		platform_driver_unregister(&cam_power_platform_driver);

	i2c_del_driver(&cam_power_i2c_driver);
}
