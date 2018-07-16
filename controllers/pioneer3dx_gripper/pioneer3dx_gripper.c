/*
 * Description:  A controller moving the Pioneer3DX and its gripper.
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/motor.h>
#include <webots/camera.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define GRIPPER_MOTOR_MAX_SPEED 0.1

static WbDeviceTag wheel_motors[3];
static WbDeviceTag gripper_motors[3];
static WbDeviceTag camera, camera_left, camera_right, camera_front, camera_back;
static int time_step = 0;

static char *colors[5] = {"red", "green", "blue", "purple", "none"};

static int is_going_back = 0;

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
    // Camera
    camera = wb_robot_get_device("camera");
    camera_back = wb_robot_get_device("camera_back");
    camera_front = wb_robot_get_device("camera_front");
    camera_left = wb_robot_get_device("camera_right");
    camera_right = wb_robot_get_device("camera_left");
    wb_camera_enable(camera, 2 * time_step);
    wb_camera_enable(camera_back, 2 * time_step);
    wb_camera_enable(camera_front, 2 * time_step); 
    wb_camera_enable(camera_left, 2 * time_step);
    wb_camera_enable(camera_right, 2 * time_step);
}

static int *getColor(WbDeviceTag device){
  int tmp_colors[3];
  tmp_colors[0] = tmp_colors[1] = tmp_colors[2] = 0;
  int width = wb_camera_get_width(device);
  int height = wb_camera_get_height(device);
  const unsigned char *image = wb_camera_get_image(device);    
  for (int i = width / 3; i < 2 * width / 3; i++) {
      for (int u = height / 2; u < 3 * height / 4; u++) {
        tmp_colors[0] += wb_camera_image_get_red(image, width, i, u);
        tmp_colors[2] += wb_camera_image_get_blue(image, width, i, u);
        tmp_colors[1] += wb_camera_image_get_green(image, width, i, u);
      }
  }
  return int{tmp_colors[0],tmp_colors[0],tmp_colors[0]};
}

void step(double seconds){
  const double ms = seconds * 1000.0;
  int elapsed_time = 0;
  while (elapsed_time < ms){
    wb_robot_step(time_step);
    elapsed_time += time_step;
  }
}

void lift(double position) {
  wb_motor_set_velocity(gripper_motors[0], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[0], position);
}

void moveFingers(double position) {
  wb_motor_set_velocity(gripper_motors[1], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_velocity(gripper_motors[2], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[1], position);
  wb_motor_set_position(gripper_motors[2], position);
}


void moveForwards(double speed) {
  wb_motor_set_velocity(wheel_motors[0], speed);
  wb_motor_set_velocity(wheel_motors[1], speed);
}

void turn(double speed) {
  wb_motor_set_velocity(wheel_motors[0], speed);
  wb_motor_set_velocity(wheel_motors[1], -speed);
}

void stop(double seconds) {
  wb_motor_set_velocity(wheel_motors[0], 0.0);
  wb_motor_set_velocity(wheel_motors[1], 0.0);
  step(seconds);
}

int main()
{
  initialize();
  /*
  lift(0.05);
  moveFingers(0.06);
  moveForwards(1.25);
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
