#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static WbDeviceTag touchsensor_a;

int main() {
  wb_robot_init();
  touchsensor_a = wb_robot_get_device("touch-01");
  wb_touch_sensor_enable(touchsensor_a, 1);
  int time_step = wb_robot_get_basic_time_step();
  while (wb_robot_step(time_step) != 1) {
    double value1 = wb_touch_sensor_get_value(touchsensor_a);
	if(value1 != 1.000000) printf("Robot at Depot 1");
  }
  return 0;
}
