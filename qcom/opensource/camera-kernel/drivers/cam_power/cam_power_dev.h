/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CAM_POWER_DEV_H_
#define _CAM_POWER_DEV_H_

struct cam_sensor_power_setting;
struct platform_driver;

extern struct platform_driver cam_power_platform_driver;

int cam_power_driver_init(void);
void cam_power_driver_exit(void);
int cam_power_rail_enable(struct cam_sensor_power_setting *setting);
int cam_power_rail_disable(struct cam_sensor_power_setting *setting);

#endif
