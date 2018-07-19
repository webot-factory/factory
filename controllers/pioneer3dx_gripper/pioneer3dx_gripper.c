/*
 * Description:  A controller moving the Pioneer3DX and its gripper.
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define GRIPPER_MOTOR_MAX_SPEED 0.1
#define MAIN_COLOR 1044480
#define SPEED 1
#define LEFT 0
#define RIGHT 1

static WbDeviceTag wheel_motors[3];
static WbDeviceTag gripper_motors[3];
static WbDeviceTag ds, camera, camera_left, camera_right, camera_front, camera_back;
static int time_step = 0;

static char *colors[5] = {"red", "green", "blue", "yellow", "none"};

static void initialize()
{
    wb_robot_init();

    time_step = wb_robot_get_basic_time_step();

    gripper_motors[0] = wb_robot_get_device("lift motor");
    gripper_motors[1] = wb_robot_get_device("left finger motor");
    gripper_motors[2] = wb_robot_get_device("right finger motor");
    wheel_motors[0] = wb_robot_get_device("left wheel");
    wheel_motors[1] = wb_robot_get_device("right wheel");
    // Specify velocity control mode
    wb_motor_set_position(wheel_motors[0], INFINITY);
    wb_motor_set_position(wheel_motors[1], INFINITY);
    wb_motor_set_velocity(wheel_motors[0], 0.0);
    wb_motor_set_velocity(wheel_motors[1], 0.0);
    wb_motor_set_position(gripper_motors[0], 0.05);
    wb_motor_set_position(gripper_motors[1], 0.1);
    wb_motor_set_position(gripper_motors[2], 0.1);
    // Camera
    camera = wb_robot_get_device("camera");
    camera_back = wb_robot_get_device("camera_back");
    camera_front = wb_robot_get_device("camera_front");
    camera_right = wb_robot_get_device("camera_right");
    camera_left = wb_robot_get_device("camera_left");
    wb_camera_enable(camera, time_step * 2);
    wb_camera_enable(camera_back, time_step* 2);
    wb_camera_enable(camera_front, time_step); 
    wb_camera_enable(camera_left, time_step);
    wb_camera_enable(camera_right, time_step * 2);
    // Distance sensor
    ds = wb_robot_get_device("ds");
    wb_distance_sensor_enable(ds, 2* time_step);
}

static char *getColor(WbDeviceTag device){
  int red = 0, green = 0, blue = 0;
  int width = wb_camera_get_width(device);
  int height = wb_camera_get_height(device);
  const unsigned char *image = wb_camera_get_image(device);    
  for (int i = 0; i < width; i++) {
      for (int u = 0; u < height; u++) {
        red += wb_camera_image_get_red(image, width, i, u);
        blue += wb_camera_image_get_blue(image, width, i, u);
        green += wb_camera_image_get_green(image, width, i, u);
      }
  }
  //printf("%d,%d,%d\n", red, green, blue);
  if (red == MAIN_COLOR && green == MAIN_COLOR)
    return colors[3];
  else if (red == MAIN_COLOR)
    return colors[0];
  else if (green == MAIN_COLOR)
    return colors[1];
  else if (blue == MAIN_COLOR)
    return colors[2];
  else
    return colors[4];
}

static char *getColora(WbDeviceTag device){
  int red = 0, green = 0, blue = 0;
  int width = wb_camera_get_width(device);
  int height = wb_camera_get_height(device);
  const unsigned char *image = wb_camera_get_image(device);    
  for (int i = 0; i < width; i++) {
      for (int u = 0; u < height; u++) {
        red += wb_camera_image_get_red(image, width, i, u);
        blue += wb_camera_image_get_blue(image, width, i, u);
        green += wb_camera_image_get_green(image, width, i, u);
      }
  }
  printf("%d,%d,%d\n", red, green, blue);
  if (red == MAIN_COLOR && green == MAIN_COLOR)
    return colors[3];
  else if (red == MAIN_COLOR)
    return colors[0];
  else if (green == MAIN_COLOR)
    return colors[1];
  else if (blue == MAIN_COLOR)
    return colors[2];
  else
    return colors[4];
}

void step(double seconds){
  const double ms = seconds * 1000.0;
  int elapsed_time = 0;
  while (elapsed_time < ms){
    wb_robot_step(time_step);
    elapsed_time += time_step;
  }
}

void forward() {
  wb_motor_set_velocity(wheel_motors[0], SPEED);
  wb_motor_set_velocity(wheel_motors[1], SPEED);
}

void reverse() {
  wb_motor_set_velocity(wheel_motors[0], -SPEED);
  wb_motor_set_velocity(wheel_motors[1], -SPEED);
}

void turn(int arg) {
 double l_speed = 0, r_speed = 0;
  switch (arg) {
    case LEFT:
      l_speed = -2 * SPEED;
      r_speed = 2 * SPEED;
      break;
    case RIGHT:
      l_speed = 2 * SPEED;
      r_speed = -2 * SPEED;
      break;
  }
  wb_motor_set_velocity(wheel_motors[0], l_speed);
  wb_motor_set_velocity(wheel_motors[1], r_speed);
  step(1.355);
  forward();
  step(0.5);
}

void stop() {
  wb_motor_set_velocity(wheel_motors[0], 0.0);
  wb_motor_set_velocity(wheel_motors[1], 0.0);
  step(1.0);
}

void pick() {
  wb_motor_set_velocity(gripper_motors[1], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_velocity(gripper_motors[2], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[1], 0.07);
  wb_motor_set_position(gripper_motors[2], 0.07);
  step(1.0);
  wb_motor_set_velocity(gripper_motors[0], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[0], 0.0);
  step(1.0);
  reverse();
}

void release() {
  wb_motor_set_velocity(gripper_motors[0], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[0], 0.05);
  step(1.0);
  wb_motor_set_velocity(gripper_motors[1], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_velocity(gripper_motors[2], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[1], 0.1);
  wb_motor_set_position(gripper_motors[2], 0.1);
  step(1.0);
  reverse();
}


int main() {
  char *tmp_color = colors[4];
  int is_going_back = 0;
  
  initialize();
  step(2.0);
  forward();
  while (wb_robot_step(time_step) != 1) {
  //getColor(camera);
    if (tmp_color == colors[4]) {
      if (wb_distance_sensor_get_value(ds) < 400) {
        stop();
        pick();
        tmp_color = getColor(camera);
        printf("%s\n",tmp_color);
      }
    } else {
      if (is_going_back == 1 && wb_distance_sensor_get_value(ds) < 400) {
        is_going_back = 0;
        stop();
        tmp_color = getColor(camera);
        pick();
      }
      if (getColor(camera_front) == colors[3]) {
        stop();
        is_going_back = 1;
        release();
        step(1.0);
        
      } else if (getColor(camera_left) == tmp_color) {
        turn(LEFT);
      } else if (getColor(camera_right) == tmp_color) {
        stop();
        turn(RIGHT);
      }
    }
  }
  
  

  
  /*

  moveForwards(1.75);
  step(2.0);
  stop(0.5);
  
  turn(2.0);
  step(2.720001);
  turn(0);
  step(1);
  moveForwards(1.25);
  step(5.5);
  step(2.0);
  stop(0.5);
  moveFingers(0.01);
  step(0.5);
  lift(0.0);
  step(0.5);
  turn(2.0);
  step(1.0);
  moveForwards(1.25);
  step(5.5);
  turn(-1.5);
  step(1.5);
  stop(0.5);
  lift(0.05);
  step(0.5);
  moveFingers(0.04);
  step(0.5);
  stop(0.5);
  */
  return 0;
}
