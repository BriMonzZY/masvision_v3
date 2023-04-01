/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/
#include <cmath>
#include <stdio.h>

#include <ros/ros.h>
#include "gimbal_control.h"

namespace roborts_detection {

double armor_distance;
cv::Point3f target_3d_pre;
extern double dt_s;
cv::Point3f postion_last;
double last_yaw;


void GimbalContrl::Init(float x,float y,float z,float pitch,float yaw, float init_v, float init_k) {
  offset_.x = x;
  offset_.y = y;
  offset_.z = z;
  offset_pitch_ = pitch;
  offset_yaw_ = yaw;
  init_v_ = init_v;
  init_k_ = init_k;
}

//air friction is considered
float GimbalContrl::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
  float t, y;
  t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
  y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
  return y;
}

//x:distance , y: height
float GimbalContrl::GetPitch(float x, float y, float v) {
  float y_temp, y_actual, dy;
  float a;
  y_temp = y;
  // by iteration
  for (int i = 0; i < 20; i++) {
    a = (float) atan2(y_temp, x);
    y_actual = BulletModel(x, v, a);
    dy = y - y_actual;
    y_temp = y_temp + dy;
    if (fabsf(dy) < 0.001) {
      break;
    }
    //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
  }
  return a;

}

void GimbalContrl::Transform(cv::Point3f &postion, float &pitch, float &yaw)
// void GimbalContrl::Transform(cv::Point3f &postion, cv::Point3f &postion_pre, float &pitch, float &yaw)
{
  double distance = armor_distance;
  ROS_WARN("distance : %.2f", distance);
  // ROS_WARN("positionx : %.2f positiony : %.2f positionz : %.2f", postion.x, postion.y, postion.z);
  
  // pitch = -GetPitch((postion.z + offset_.z) / 100, -(postion.y + offset_.y) / 100, 15) + (float)(offset_pitch_ * 3.1415926535 / 180);
  // pitch = -GetPitch((postion.z + offset_.z) / 100, -(postion.y + offset_.y) / 100, init_v_) + (float)(offset_pitch_ * 3.1415926535 / 180);

  // 记得加入重力补偿（或者角度补偿）
  double tan_pitch = postion.y / sqrt(postion.x * postion.x + postion.z * postion.z);
	pitch = -atan(tan_pitch);  // pitch = -atan(tan_pitch) * 180 / CV_PI;
  pitch += offset_pitch_ / 180 * CV_PI; // 线性角度偏移

  // pitch = atan2(0.6, distance/1000);// / 3.1415926535 * 180;
  ROS_WARN("pitch : %.4f", pitch);
      
  //yaw positive direction :anticlockwise
  yaw = -(float) (atan2(postion.x + offset_.x, postion.z + offset_.z)) + (float)(offset_yaw_ * 3.1415926535 / 180);

  // 卡尔曼滤波预测
  // float yaw_pre;
  // yaw_pre = -(float) (atan2(target_3d_pre.x + offset_.x, target_3d_pre.z + offset_.z)) + (float)(offset_yaw_ * 3.1415926535 / 180);

  ROS_WARN("yaw : %.4f", yaw);
  // ROS_WARN("yaw pre : %.4f", yaw_pre);
  // ROS_WARN("yaw diff : %.4f", yaw_pre - yaw);
  // yaw = yaw_pre;

  // 速度预测
  // double v_x = (postion.x-postion_last.x)/dt_s;
  // double v_y = (postion.y-postion_last.y)/dt_s;
  // double v_z = (postion.z-postion_last.z)/dt_s;
  // ROS_WARN("v : %.4f %.4f %.4f", v_x, v_y, v_z);
  // yaw = -(float) (atan2(postion.x + v_x*12*dt_s + offset_.x, postion.z  + v_z*dt_s*12 + offset_.z)) + (float)(offset_yaw_ * 3.1415926535 / 180);
  // postion_last = postion;
}

} // roborts_detection



