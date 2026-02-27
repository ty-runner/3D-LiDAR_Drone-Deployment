/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include <time.h>
#include <stdio.h>

#define RAD2DEG(x) ((x)*180./M_PI)

int min_count(int* quadrants);
int min_count(int* quadrants){
    int res;
    if(quadrants[0] < quadrants[1])
        res = 0; //passed in as 0. no need to update
    else
        res = 1;
    if(res < quadrants[3])
        res = res;
    else
        res = 3;
    if(res < quadrants[2])
        res = res;
    else
        res = 2;//we want to prioritize 2 because this is going forward.
    
    return res;
}
/*
static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;
  static int last_direction = -1;
  //printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  //printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         //RAD2DEG(scan->angle_max));
  float threshold = 0.6; //2 ft
  bool prox_flag = false; //flag if there is any danger
  int quadrants[4] = {0,0,0,0};
  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    
    if(scan->ranges[i] < threshold){
	   if(degree > -45 && degree < 45){
            printf("BACK: count: %d\n", quadrants[0]);
            quadrants[0]++;
       } 
	   if(degree > 45 && degree < 135){
            printf("RIGHT: count: %d\n", quadrants[1]);
            quadrants[1]++;
       } 
	   if(degree > 135 || degree < -135){
            printf("FRONT: count: %d\n", quadrants[2]);
            quadrants[2]++;
       } 
	   if(degree < -45 && degree > -135){
            printf("LEFT: count: %d\n", quadrants[3]);
            quadrants[3]++;
       } 
       prox_flag = true;
    }
     
//    printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
  }
  if(prox_flag){
      int min = min_count(quadrants);
      printf("MIN OF SESSION: %d\n", quadrants[min]);
      printf("INDEX: %d\n", min);
      if(min!=last_direction){
          const char* direction_files[4] = {
            "back.wav",
            "right.wav",
            "front.wav",
            "left.wav"
          };
          last_direction = min;
          char command[100];
          //system("amixer sset 'Master' 20%");
          snprintf(command, sizeof(command), "play %s vol 0.2 &", direction_files[min]);
          system(command);
      }
  }
  else
      last_direction = -1;
}*/
static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);  // Start timing

    int count = scan->scan_time / scan->time_increment;
    static int last_direction = -1;

    float threshold = 0.6f; // 2 ft proximity threshold
    bool prox_flag = false;
    int quadrants[4] = {0, 0, 0, 0}; // BACK, RIGHT, FRONT, LEFT

    for (int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);

        if (scan->ranges[i] < threshold) {
            if (degree > -45 && degree < 45) {
                quadrants[0]++; // BACK
            } 
            if (degree > 45 && degree < 135) {
                quadrants[1]++; // RIGHT
            } 
            if (degree > 135 || degree < -135) {
                quadrants[2]++; // FRONT
            } 
            if (degree < -45 && degree > -135) {
                quadrants[3]++; // LEFT
            } 
            prox_flag = true;
        }
    }

    if (prox_flag) {
        int min = min_count(quadrants);
        if (min != last_direction) {
            const char* direction_files[4] = {
                "back.wav",
                "right.wav",
                "front.wav",
                "left.wav"
            };
            last_direction = min;

            char command[100];
            snprintf(command, sizeof(command), "play %s vol 0.2 &", direction_files[min]);
            system(command);
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &end);  // End timing

    double elapsed_ms = (end.tv_sec - start.tv_sec) * 1000.0 +
                        (end.tv_nsec - start.tv_nsec) / 1e6;

    FILE* file = fopen("execution_times.csv", "a");
    if (file) {
        fprintf(file, "%f\n", elapsed_ms);
        fclose(file);
    } else {
        fprintf(stderr, "Could not open execution_times.csv for writing\n");
    }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sllidar_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();


  return 0;
}
