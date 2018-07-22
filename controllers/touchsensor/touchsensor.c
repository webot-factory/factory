#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#define SENSOR_COUNT 3

static WbDeviceTag touchsensors[SENSOR_COUNT];


int main() {
  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  int tmp_values[SENSOR_COUNT];
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
      char sensor_name[8];
      *sensor_name = *"touch-0";
      sensor_name[7] = (i+1) + '0';
      touchsensors[i] = wb_robot_get_device(sensor_name);
      wb_touch_sensor_enable(touchsensors[i], 1);
      tmp_values[i] = 0;
  }
  
  
  while (wb_robot_step(time_step) != 1) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      int value = wb_touch_sensor_get_value(touchsensors[i]);
      if(value != tmp_values[i] && value == 1) printf("Robot at Depot %d\n",i+1);
      tmp_values[i] = value;
    }
  }
  return 0;
 
}
