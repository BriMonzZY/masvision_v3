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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "constraint_set.h"

#include "timer/timer.h"
#include "io/io.h"

namespace roborts_detection {

ConstraintSet::ConstraintSet(std::shared_ptr<CVToolbox> cv_toolbox):
    ArmorDetectionBase(cv_toolbox){
  filter_x_count_ = 0;
  filter_y_count_ = 0;
  filter_z_count_ = 0;
  filter_distance_count_ = 0;
  filter_pitch_count_ = 0;
  filter_yaw_count_ = 0;
  read_index_ = -1;
  detection_time_ = 0;
  thread_running_ = false;


  armorprocessor_ = std::make_unique<ArmorProcessor>();

  LoadParam();
  error_info_ = ErrorInfo(roborts_common::OK);
}



unsigned int enemy_color_from_referee;

/**
 * @brief 获取敌方颜色
 * 
 * @param robot_status 
 */
void GetEnemyColor(const roborts_msgs::RobotStatus::ConstPtr &robot_status)
{
  ROS_WARN("robort id : %d", robot_status->id);
  if(robot_status->id == 3 || robot_status->id == 4 || robot_status->id == 5) {  // 红方步兵
    enemy_color_from_referee = BLUE;
  }
  else if(robot_status->id == 103 || robot_status->id == 104 || robot_status->id == 105) {  // 蓝方步兵
    enemy_color_from_referee = RED;
  }
}

void ConstraintSet::LoadParam()
{
  //read parameters
  ConstraintSetConfig constraint_set_config_;
  std::string file_name = ros::package::getPath("roborts_detection") + \
      "/armor_detection/constraint_set/config/constraint_set.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &constraint_set_config_);
  ROS_ASSERT_MSG(read_state, "Cannot open %s", file_name.c_str());

  enable_debug_ = constraint_set_config_.enable_debug();

  // enemy_color_ = constraint_set_config_.enemy_color();  // 设置敌方颜色

  using_hsv_ = constraint_set_config_.using_hsv();

  //armor info
  float armor_width = constraint_set_config_.armor_size().width();
  float armor_height = constraint_set_config_.armor_size().height();
  SolveArmorCoordinate(armor_width, armor_height);

  //algorithm threshold parameters
  light_max_aspect_ratio_ = constraint_set_config_.threshold().light_max_aspect_ratio();
  light_min_area_ = constraint_set_config_.threshold().light_min_area();
  light_max_angle_ = constraint_set_config_.threshold().light_max_angle();
  light_max_angle_diff_ = constraint_set_config_.threshold().light_max_angle_diff();
  armor_max_angle_ = constraint_set_config_.threshold().armor_max_angle();
  armor_min_area_ = constraint_set_config_.threshold().armor_min_area();
  armor_max_aspect_ratio_ = constraint_set_config_.threshold().armor_max_aspect_ratio();
  armor_max_pixel_val_ = constraint_set_config_.threshold().armor_max_pixel_val();
  armor_max_stddev_ = constraint_set_config_.threshold().armor_max_stddev();
  armor_max_mean_   = constraint_set_config_.threshold().armor_max_mean();

  color_thread_ = constraint_set_config_.threshold().color_thread();
  blue_thread_ = constraint_set_config_.threshold().blue_thread();
  red_thread_ = constraint_set_config_.threshold().red_thread();

  int get_intrinsic_state = -1;
  int get_distortion_state = -1;

  while ((get_intrinsic_state < 0) || (get_distortion_state < 0)) {
    ROS_WARN("Wait for camera driver launch %d", get_intrinsic_state);
    usleep(50000);
    ros::spinOnce();
    get_intrinsic_state = cv_toolbox_->GetCameraMatrix(intrinsic_matrix_);
    get_distortion_state = cv_toolbox_->GetCameraDistortion(distortion_coeffs_);
  }
}

extern cv::Point3f target_3d_pre;
int detection_flag;
double dt_s;
ros::Time last_time_s;


ErrorInfo ConstraintSet::DetectArmor(bool &detected, cv::Point3f &target_3d)
// ErrorInfo ConstraintSet::DetectArmor(bool &detected, cv::Point3f &target_3d, cv::Point3f &target_3d_pre)
{
  std::vector<cv::RotatedRect> lights;
  std::vector<ArmorInfo> armors;

  // enemy_color_ = enemy_color_from_referee;  // 设置敌方颜色
  enemy_color_ = RED;  // 设置敌方颜色

  ros::Time time = ros::Time::now();
  dt_s = (time - last_time_s).toSec();
  ROS_WARN("dt_ : %.4f", dt_s);
  last_time_s = time;

  auto img_begin = std::chrono::high_resolution_clock::now();
  bool sleep_by_diff_flag = true;
  while (true) {
    // Ensure exit this thread while call Ctrl-C
    if (!thread_running_) {
      ErrorInfo error_info(ErrorCode::STOP_DETECTION);
      return error_info;
    }
    read_index_ = cv_toolbox_->NextImage(src_img_);
    if (read_index_ < 0) {
      // Reducing lock and unlock when accessing function 'NextImage'
      if (detection_time_ == 0) {
        usleep(20000);
        continue;
      } else {
        double capture_time = 0;
        cv_toolbox_->GetCaptureTime(capture_time);
        if (capture_time == 0) {
          // Make sure the driver is launched and the image callback is called
          usleep(20000);
          continue;
        } else if (capture_time > detection_time_ && sleep_by_diff_flag) {
          // ROS_WARN("time sleep %lf", (capture_time - detection_time_));
          usleep((unsigned int)(capture_time - detection_time_));
          sleep_by_diff_flag = false;
          continue;
        } else {
          //For real time request when image call back called, the function 'NextImage' should be called.
          usleep(500);
          continue;
        }
      }
    } else {
      break;
    }
  }
  /*ROS_WARN("time get image: %lf", std::chrono::duration<double, std::ratio<1, 1000>>
      (std::chrono::high_resolution_clock::now() - img_begin).count());*/

  auto detection_begin = std::chrono::high_resolution_clock::now();  // C++高精度时钟获取当前时间

  cv::cvtColor(src_img_, gray_img_, CV_BGR2GRAY);  // 转换为灰度图像
  if (enable_debug_) {
    show_lights_before_filter_ = src_img_.clone();
    show_lights_after_filter_ = src_img_.clone();
    show_armors_befor_filter_ = src_img_.clone();
    show_armors_after_filter_ = src_img_.clone();
    cv::waitKey(1);
  }

  DetectLights(src_img_, lights);  // 灯条检测
  FilterLights(lights);  // 滤除不符合特征的灯条
  PossibleArmors(lights, armors);  // 检测可能的装甲板
  FilterArmors(armors); // 滤除不符合特征的装甲板


  roborts_msgs::Armors armors_;
  roborts_msgs::Target target_msg;



  if(!armors.empty()) {
    
    detection_flag = true;

    detected = true;
    ArmorInfo final_armor = SlectFinalArmor(armors);
    cv_toolbox_->DrawRotatedRect(src_img_, armors[0].rect, cv::Scalar(0, 255, 0), 2);
    CalcControlInfo(final_armor, target_3d);  // PnP解算

    // 卡尔曼滤波预测
    // roborts_msgs::Armor armor_msg;
    // armors_.header.stamp = ros::Time::now();
    // armor_msg.number = 3;
    // armor_msg.distance_to_image_center = calculateDistanceToCenter(final_armor.rect.center);
    // armor_msg.position.x = target_3d.x;
    // armor_msg.position.y = target_3d.y;
    // armor_msg.position.z = target_3d.z;
    // armors_.armors.emplace_back(armor_msg);
    cv::circle(src_img_, cv::Point(armors[0].rect.center.x, armors[0].rect.center.y), 10, cv::Scalar(255, 0, 0), 3, 8);
  }
  else {

    detection_flag = false;

    detected = false;
  }

  // 卡尔曼滤波预测
  // target_msg = armorprocessor_->armorsCallback(&armors_);
  // target_3d_pre.x = target_msg.position.x;
  // target_3d_pre.y = target_msg.position.y;
  // target_3d_pre.z = target_msg.position.z;
  // ROS_WARN("%.2f %.2f %.2f", target_msg.position.x, target_msg.position.y, target_msg.position.z);
  // cv::circle(src_img_, cv::Point(armors[0].rect.center.x, armors[0].rect.center.y), 10, cv::Scalar(0, 0, 255), 3, 8);


  if(enable_debug_) {
    cv::imshow("result_img_", src_img_);
  }

  lights.clear();
  armors.clear();
  cv_toolbox_->ReadComplete(read_index_);
  ROS_INFO("read complete");
  detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>
      (std::chrono::high_resolution_clock::now() - detection_begin).count();

  // 显示处理时间
  if(enable_debug_) ROS_WARN("detection time : %.2f", detection_time_);

  return error_info_;
}

float ConstraintSet::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = intrinsic_matrix_.at<double>(0, 2);
  float cy = distortion_coeffs_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

/**
 * @brief 进行图像预处理并根据轮廓寻找可能的灯条
 * 
 * @param src 
 * @param lights 
 */
void ConstraintSet::DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights)
{
  // std::cout << "********************************************DetectLights********************************************" << std::endl;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));  // 3*3矩形核
  cv::dilate(src, src, element, cv::Point(-1, -1), 1);  // 膨胀处理
  cv::Mat binary_brightness_img, binary_light_img, binary_color_img;

  if(using_hsv_) {
    binary_color_img = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);
    cv::threshold(gray_img_, binary_brightness_img, color_thread_, 255, CV_THRESH_BINARY);
  }
  else {
    auto light = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);  // 高亮指定颜色
    cv::threshold(gray_img_, binary_brightness_img, color_thread_, 255, CV_THRESH_BINARY);  // 二值化灰度图像
    float thresh;
    if (enemy_color_ == BLUE)
      thresh = blue_thread_;
    else
      thresh = red_thread_;
    cv::threshold(light, binary_color_img, thresh, 255, CV_THRESH_BINARY);  //二值化高亮颜色的图像
    if(enable_debug_) cv::imshow("light", light);
  }
  //binary_light_img = binary_color_img & binary_brightness_img;
  if (enable_debug_) {
    cv::imshow("binary_brightness_img", binary_brightness_img);
    //cv::imshow("binary_light_img", binary_light_img);
    cv::imshow("binary_color_img", binary_color_img);
  }

  auto contours_light = cv_toolbox_->FindContours(binary_color_img);  //轮廓检测
  auto contours_brightness = cv_toolbox_->FindContours(binary_brightness_img);

  lights.reserve(contours_light.size());  // reserve分配内存
  lights_info_.reserve(contours_light.size());
  // TODO: To be optimized
  //std::vector<int> is_processes(contours_light.size());
  for (unsigned int i = 0; i < contours_brightness.size(); ++i) {
    for (unsigned int j = 0; j < contours_light.size(); ++j) {

        if (cv::pointPolygonTest(contours_light[j], contours_brightness[i][0], false) >= 0.0) {  // 检测点是否在多边形内部  参数1：轮廓列表  参数2：点坐标 参数3：false：输出为正表示在轮廓内，0为轮廓上，负为轮廓外
          cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[i]);  // 求点集的最小面积矩形
          cv::Point2f vertices_point[4];
          single_light.points(vertices_point);  // 求矩形的4个顶点
          LightInfo light_info(vertices_point);  // 实例化灯条类

          if (enable_debug_) cv_toolbox_->DrawRotatedRect(show_lights_before_filter_, single_light, cv::Scalar(0, 255, 0), 2, light_info.angle_);

          single_light.angle = light_info.angle_;
          lights.push_back(single_light);  // 将灯条信息压入vector中
          break;
        }
    }
  }

  if (enable_debug_) cv::imshow("show_lights_before_filter", show_lights_before_filter_);

  auto c = cv::waitKey(1);
  if (c == 'a') {
    cv::waitKey(0);
  }
}

/**
 * @brief 滤除不符合长宽和角度特征的灯条
 * 
 * @param lights 
 */
void ConstraintSet::FilterLights(std::vector<cv::RotatedRect> &lights)
{
  //std::cout << "********************************************FilterLights********************************************" << std::endl;
  std::vector<cv::RotatedRect> rects;
  rects.reserve(lights.size());

  for (const auto &light : lights) {
    float angle;
    auto light_aspect_ratio = std::max(light.size.width, light.size.height) / std::min(light.size.width, light.size.height);
    //https://stackoverflow.com/questions/15956124/minarearect-angles-unsure-about-the-angle-returned/21427814#21427814
    if(light.size.width < light.size.height) {
      angle = light.angle; // -light.angle
    }
    else {
      angle = light.angle; // light.angle + 90
    }
    //std::cout << "light angle: " << angle << std::endl;
    //std::cout << "light_aspect_ratio: " << light_aspect_ratio << std::endl;
    //std::cout << "light_area: " << light.size.area() << std::endl;
    if (light_aspect_ratio < light_max_aspect_ratio_ && light.size.area() >= light_min_area_) { //angle < light_max_angle_ &&
      rects.push_back(light);
      if (enable_debug_) cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, light, cv::Scalar(0, 255, 0), 2, angle);
    }
  }
  if (enable_debug_) cv::imshow("lights_after_filter", show_lights_after_filter_);

  lights = rects;
}

/**
 * @brief 找到可能的所有装甲板
 * 
 * @param lights 
 * @param armors 
 */
void ConstraintSet::PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors)
{
  //std::cout << "********************************************PossibleArmors********************************************" << std::endl;
  for (unsigned int i = 0; i < lights.size(); i++) {
    for (unsigned int j = i + 1; j < lights.size(); j++) {
      cv::RotatedRect light1 = lights[i];
      cv::RotatedRect light2 = lights[j];
      auto edge1 = std::minmax(light1.size.width, light1.size.height);  // 获取最小值和最大值
      auto edge2 = std::minmax(light2.size.width, light2.size.height);
      auto lights_dis = std::sqrt((light1.center.x - light2.center.x) * (light1.center.x - light2.center.x) +
          (light1.center.y - light2.center.y) * (light1.center.y - light2.center.y));
      auto center_angle = std::atan(std::abs(light1.center.y - light2.center.y) / std::abs(light1.center.x - light2.center.x)) * 180 / CV_PI;
      center_angle = center_angle > 90 ? 180 - center_angle : center_angle;
      //std::cout << "center_angle: " << center_angle << std::endl;

      // 计算装甲板的参数
      cv::RotatedRect rect;
      rect.angle = static_cast<float>(center_angle);  // 进行强制类型转换
      rect.center.x = (light1.center.x + light2.center.x) / 2;
      rect.center.y = (light1.center.y + light2.center.y) / 2;
      float armor_width = std::abs(static_cast<float>(lights_dis) - std::max(edge1.first, edge2.first));
      float armor_height = std::max<float>(edge1.second, edge2.second);

      rect.size.width = std::max<float>(armor_width, armor_height);
      rect.size.height = std::min<float>(armor_width, armor_height);

      float light1_angle = light1.angle; //light1.size.width < light1.size.height ? -light1.angle : light1.angle + 90
      float light2_angle = light2.angle; //light2.size.width < light2.size.height ? -light2.angle : light2.angle + 90
      //std::cout << "light1_angle: " << light1_angle << std::endl;
      //std::cout << "light2_angle: " << light2_angle << std::endl;

      if (enable_debug_) {
        // std::cout << "*******************************" << std::endl;
        // std::cout << "light_angle_diff_: " << std::abs(light1_angle - light2_angle) << std::endl;
        // std::cout << "radio: " << std::max<float>(edge1.second, edge2.second)/std::min<float>(edge1.second, edge2.second) << std::endl;
        // std::cout << "armor_angle_: " << std::abs(center_angle) << std::endl;
        // std::cout << "armor_aspect_ratio_: " << rect.size.width / (float) (rect.size.height) << std::endl;
        // std::cout << "armor_area_: " << std::abs(rect.size.area()) << std::endl;
        // std::cout << "armor_pixel_val_: " << (float)(gray_img_.at<uchar>(static_cast<int>(rect.center.y), static_cast<int>(rect.center.x))) << std::endl;
        // std::cout << "pixel_y" << static_cast<int>(rect.center.y) << std::endl;
        // std::cout << "pixel_x" << static_cast<int>(rect.center.x) << std::endl;
      }
      
      auto angle_diff = std::abs(light1_angle - light2_angle);
      // Avoid incorrect calculation at 180 and 0.
      if (angle_diff > 175) {
        angle_diff = 180 -angle_diff;
      }

      // 判断装甲板是否符合条件
      if (angle_diff < light_max_angle_diff_ &&
          std::max<float>(edge1.second, edge2.second)/std::min<float>(edge1.second, edge2.second) < 2.0 &&
          rect.size.width / (rect.size.height) < armor_max_aspect_ratio_ &&
          std::abs(rect.size.area()) > armor_min_area_ &&
          gray_img_.at<uchar>(static_cast<int>(rect.center.y), static_cast<int>(rect.center.x))
              < armor_max_pixel_val_) { //std::abs(center_angle) < armor_max_angle_ &&

        if (light1.center.x < light2.center.x) {
          std::vector<cv::Point2f> armor_points;
          CalcArmorInfo(armor_points, light1, light2);
          armors.emplace_back(ArmorInfo(rect, armor_points));
          if (enable_debug_)
            cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(0, 255, 0), 2);
          armor_points.clear();
        } else {
          std::vector<cv::Point2f> armor_points;
          CalcArmorInfo(armor_points, light2, light1);
          armors.emplace_back(ArmorInfo(rect, armor_points));
          if (enable_debug_)
            cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(0, 255, 0), 2);
          armor_points.clear();
        }
      }
    }
  }
  if (enable_debug_) cv::imshow("armors_before_filter", show_armors_befor_filter_);
}

/**
 * @brief 滤除不符合特征的装甲板
 * 
 * @param armors 
 */
void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors)
{
  //std::cout << "********************************************FilterArmors********************************************" << std::endl;
  // 通过均值和标准差滤除装甲板
  cv::Mat mask = cv::Mat::zeros(gray_img_.size(), CV_8UC1);
  for (auto armor_iter = armors.begin(); armor_iter != armors.end();) {
    cv::Point pts[4];
    for (unsigned int i = 0; i < 4; i++) {
      pts[i].x = (int) armor_iter->vertex[i].x;
      pts[i].y = (int) armor_iter->vertex[i].y;
    }
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255), 8, 0);  // 填充多边形 参数2：多边形顶点 参数3：颜色

    cv::Mat mat_mean;
    cv::Mat mat_stddev;
    cv::meanStdDev(gray_img_, mat_mean, mat_stddev, mask);  // 计算矩阵的均值和标准差

    auto stddev = mat_stddev.at<double>(0, 0);
    auto mean = mat_mean.at<double>(0, 0);
    //std::cout << "stddev: " << stddev << std::endl;
    //std::cout << "mean: " << mean << std::endl;

    if (stddev > armor_max_stddev_ || mean > armor_max_mean_) {
      armor_iter = armors.erase(armor_iter);
    } else {
      armor_iter++;
    }
  }

  // nms 非极大值抑制 将重叠在一起的多余的结果去除
  std::vector<bool> is_armor(armors.size(), true);
  for (int i = 0; i < armors.size() && is_armor[i] == true; i++) {
    for (int j = i + 1; j < armors.size() && is_armor[j]; j++) {
      float dx = armors[i].rect.center.x - armors[j].rect.center.x;
      float dy = armors[i].rect.center.y - armors[j].rect.center.y;
      float dis = std::sqrt(dx * dx + dy * dy);
      if (dis < armors[i].rect.size.width + armors[j].rect.size.width) {
        if (armors[i].rect.angle > armors[j].rect.angle) {
          is_armor[i] = false;
          //std::cout << "i: " << i << std::endl;
        } else {
          is_armor[j] = false;
          //std::cout << "j: " << j << std::endl;
        }
      }
    }
  }

  // 更新装甲板信息
  //std::cout << armors.size() << std::endl;
  for (unsigned int i = 0; i < armors.size(); i++) {
    if (!is_armor[i]) {
      armors.erase(armors.begin() + i);
      is_armor.erase(is_armor.begin() + i);
      //std::cout << "index: " << i << std::endl;
    } else if (enable_debug_) {
      cv_toolbox_->DrawRotatedRect(show_armors_after_filter_, armors[i].rect, cv::Scalar(0, 255, 0), 2);
    }
  }
  if (enable_debug_) cv::imshow("armors_after_filter", show_armors_after_filter_);
}


/**
 * @brief 寻找面积最大的装甲板
 * 
 * @param armors 
 * @return ArmorInfo 
 */
ArmorInfo ConstraintSet::SlectFinalArmor(std::vector<ArmorInfo> &armors)
{
  std::sort(armors.begin(),
            armors.end(),
            [](const ArmorInfo &p1, const ArmorInfo &p2) { return p1.rect.size.area() > p2.rect.size.area(); });

  return armors[0];
}

extern double armor_distance;


/**
 * @brief PnP解算
 * 
 * @param armor 
 * @param target_3d 
 */
void ConstraintSet::CalcControlInfo(const ArmorInfo & armor, cv::Point3f &target_3d)
{
  cv::Mat rvec;
  cv::Mat tvec;
  cv::solvePnP(armor_points_,
               armor.vertex,
               intrinsic_matrix_,
               distortion_coeffs_,
               rvec,
               tvec);
  target_3d = cv::Point3f(tvec);


  // caculate distance
  int GUN_CAM_DISTANCE_Y = 0;
  tvec.at<double>(1, 0) -= GUN_CAM_DISTANCE_Y;
  double x_pos = tvec.at<double>(0, 0);
  double y_pos = tvec.at<double>(1, 0);
  double z_pos = tvec.at<double>(2, 0);
  armor_distance = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);
  armor_distance = armor_distance/1.4;  // compensate
  // double distance = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);
  // ROS_WARN("distance : %.2f", armor_distance);

}

/**
 * @brief 计算装甲板信息
 * 
 * @param armor_points 
 * @param left_light 
 * @param right_light 
 */
void ConstraintSet::CalcArmorInfo(std::vector<cv::Point2f> &armor_points,
                                 cv::RotatedRect left_light,
                                 cv::RotatedRect right_light) {
  cv::Point2f left_points[4], right_points[4];
  left_light.points(left_points);
  right_light.points(right_points);

  cv::Point2f right_lu, right_ld, lift_ru, lift_rd;
  std::sort(left_points, left_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  std::sort(right_points, right_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  if (right_points[0].y < right_points[1].y) {
    right_lu = right_points[0];
    right_ld = right_points[1];
  } else {
    right_lu = right_points[1];
    right_ld = right_points[0];
  }

  if (left_points[2].y < left_points[3].y) {
    lift_ru = left_points[2];
    lift_rd = left_points[3];
  } else {
    lift_ru = left_points[3];
    lift_rd = left_points[2];
  }
  armor_points.push_back(lift_ru);
  armor_points.push_back(right_lu);
  armor_points.push_back(right_ld);
  armor_points.push_back(lift_rd);

}
/**
 * @brief 根据装甲板的宽和高设置装甲板的三维坐标
 * 
 * @param width 宽
 * @param height 高
 */
void ConstraintSet::SolveArmorCoordinate(const float width,
                                         const float height) {
  armor_points_.emplace_back(cv::Point3f(-width/2, height/2,  0.0));
  armor_points_.emplace_back(cv::Point3f(width/2,  height/2,  0.0));
  armor_points_.emplace_back(cv::Point3f(width/2,  -height/2, 0.0));
  armor_points_.emplace_back(cv::Point3f(-width/2, -height/2, 0.0));
}

/**
 * @brief 滤波 滤除毛刺
 * 
 * @param new_num 
 * @param old_num 
 * @param filter_count 
 * @param max_diff 
 */
void ConstraintSet::SignalFilter(double &new_num, double &old_num, unsigned int &filter_count, double max_diff) {
  if(fabs(new_num - old_num) > max_diff && filter_count < 2) {
    filter_count++;
    new_num += max_diff;
  } else {
    filter_count = 0;
    old_num = new_num;
  }
}

void ConstraintSet::SetThreadState(bool thread_state) {
  thread_running_ = thread_state;
}






/************************************************************************** KalmanFilter *****************************************************************************************/

KalmanFilter::KalmanFilter(const KalmanFilterMatrices & matrices)
: F(matrices.F),
  H(matrices.H),
  Q(matrices.Q),
  R(matrices.R),
  P_post(matrices.P),
  n(matrices.F.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pre(n),
  x_post(n)
{
}

void KalmanFilter::init(const Eigen::VectorXd & x0) { x_post = x0; }

Eigen::MatrixXd KalmanFilter::predict(const Eigen::MatrixXd & F)
{
  ROS_WARN("in predict");
  this->F = F;

  std::cout << this->F << std::endl;
  std::cout << x_post << std::endl;

  x_pre = F * x_post;
  P_pre = F * P_post * F.transpose() + Q; // 转置

  // handle the case when there will be no measurement before the next predict
  x_post = x_pre;
  P_post = P_pre;

  return x_pre;
}


Eigen::MatrixXd KalmanFilter::update(const Eigen::VectorXd & z)
{
  ROS_WARN("in update");
  K = P_pre * H.transpose() * (H * P_pre * H.transpose() + R).inverse();
  x_post = x_pre + K * (z - H * x_pre);
  P_post = (I - K * H) * P_pre;

  return x_post;
}






Tracker::Tracker(
  const KalmanFilterMatrices & kf_matrices, double max_match_distance, int tracking_threshold,
  int lost_threshold)
: tracker_state(LOST),
  tracking_id(0),
  kf_matrices_(kf_matrices),
  tracking_velocity_(Eigen::Vector3d::Zero()),
  max_match_distance_(max_match_distance),
  tracking_threshold_(tracking_threshold),
  lost_threshold_(lost_threshold)
{
}

// void Tracker::init(const Armors::SharedPtr & armors_msg)
void Tracker::init(const Armors * armors_msg)
{
  if (armors_msg->armors.empty()) {
    return;
  }

  // 选择离得最近的装甲板 Simply choose the armor that is closest to image center
  double min_distance = DBL_MAX;
  auto chosen_armor = armors_msg->armors[0];
  // for (const auto & armor : armors_msg->armors) {
  //   if (armor.distance_to_image_center < min_distance) {
  //     min_distance = armor.distance_to_image_center;
  //     chosen_armor = armor;
  //   }
  // }

  // 初始化卡尔曼滤波器 KF init
  kf_ = std::make_unique<KalmanFilter>(kf_matrices_);  // 实例化卡尔曼滤波器
  Eigen::VectorXd init_state(6);
  const auto position = chosen_armor.position;
  init_state << position.x, position.y, position.z, 0, 0, 0;
  kf_->init(init_state);

  tracking_id = chosen_armor.number;
  tracker_state = DETECTING;
}

/**
 * @brief 
 * 
 * @param armors_msg 
 * @param dt 
 */
// void Tracker::update(const Armors::SharedPtr & armors_msg, const double & dt)
void Tracker::update(Armors * armors_msg, const double & dt)
{
  armors_msg_last = armors_msg;

  // 卡尔曼预测阶段 KF predict
  kf_matrices_.F(0, 3) = kf_matrices_.F(1, 4) = kf_matrices_.F(2, 5) = dt;
  Eigen::VectorXd kf_prediction = kf_->predict(kf_matrices_.F);

  bool matched = false;
  // 如果没有找到匹配的装甲，则使用KF预测作为默认目标状态 Use KF prediction as default target state if no matched armor is found
  target_state = kf_prediction;

  if (!armors_msg->armors.empty()) {
    Armor matched_armor;
    double min_position_diff = DBL_MAX;

    for (const auto & armor : armors_msg->armors) {
      Eigen::Vector3d position_vec(armor.position.x, armor.position.y, armor.position.z);
      Eigen::Vector3d predicted_position = kf_prediction.head(3);
      // 当前装甲位置与跟踪装甲预测位置的差异 Difference of the current armor position and tracked armor's predicted position
      double position_diff = (predicted_position - position_vec).norm();  // 范数 （求距离）
      // 寻找预测与跟踪距离最近的装甲板
      if (position_diff < min_position_diff) {
        min_position_diff = position_diff;
        matched_armor = armor;
      }
    }

    ROS_WARN("min_position_diff : %.2f  max_match_distance_ : %.2f", min_position_diff, max_match_distance_);
    // if (min_position_diff/1000.0 < max_match_distance_) {
    if (min_position_diff < max_match_distance_) {
      // Matching armor found
      matched = true;
      Eigen::Vector3d position_vec(
        matched_armor.position.x, matched_armor.position.y, matched_armor.position.z);
      target_state = kf_->update(position_vec); // 状态更新
    } else {
      // 检查当前帧中是否有相同id的装甲板 Check if there is same id armor in current frame
      for (const auto & armor : armors_msg->armors) {
        ROS_WARN("initial vel");
        // if (armor.number == tracking_id) {
          matched = true;
          // 初始化卡尔曼滤波器 Reset KF
          kf_ = std::make_unique<KalmanFilter>(kf_matrices_);
          Eigen::VectorXd init_state(6);

          velocity_[0] = (armor.position.x - armors_msg_last->armors[0].position.x)/1000/dt;
          velocity_[1] = (armor.position.y - armors_msg_last->armors[0].position.x)/1000/dt;
          velocity_[2] = (armor.position.z - armors_msg_last->armors[0].position.x)/1000/dt;

          // Set init state with current armor position and tracking velocity before
          // init_state << armor.position.x, armor.position.y, armor.position.z, tracking_velocity_;
          init_state << armor.position.x, armor.position.y, armor.position.z, velocity_;
          kf_->init(init_state);
          target_state = init_state;
          break;
        // }
      }
    }
  }

  // 保存跟踪目标速度 Save tracking target velocity
  tracking_velocity_ = target_state.tail(3);

  // Tracking state machine
  if (tracker_state == DETECTING) {
    // DETECTING
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_threshold_) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }

  } else if (tracker_state == TRACKING) {
    // TRACKING
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }

  } else if (tracker_state == TEMP_LOST) {
    // LOST
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_threshold_) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }
}












ArmorProcessor::ArmorProcessor()
: last_time_(0), dt_(0.0)
{

  // Kalman Filter initial matrix
  // A - state transition matrix
  // clang-format off
  Eigen::Matrix<double, 6, 6> f;
  f <<  1,  0,  0, dt_, 0,  0,
        0,  1,  0,  0, dt_, 0,
        0,  0,  1,  0,  0, dt_,
        0,  0,  0,  1,  0,  0,
        0,  0,  0,  0,  1,  0,
        0,  0,  0,  0,  0,  1;
  // f <<  2,  0,  0, dt_, 0,  0,
  //       0,  2,  0,  0, dt_, 0,
  //       0,  0,  2,  0,  0, dt_,
  //       0,  0,  0,  2,  0,  0,
  //       0,  0,  0,  0,  2,  0,
  //       0,  0,  0,  0,  0,  2;
  // clang-format on

  // H - measurement matrix  
  Eigen::Matrix<double, 3, 6> h;
  h.setIdentity();  // 将矩阵中所有行列索引不相等的元素设置为0，将所有行列索引相等的元素设置为指定的值

  // Q - process noise covariance matrix
  Eigen::DiagonalMatrix<double, 6> q;
  q.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;
  // q.diagonal() << 0.05, 0.05, 0.05, 0.5, 0.5, 0.5;

  // R - measurement noise covariance matrix
  Eigen::DiagonalMatrix<double, 3> r;
  r.diagonal() << 0.05, 0.05, 0.05;

  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 6> p;
  p.setIdentity();

  kf_matrices_ = KalmanFilterMatrices{f, h, q, r, p};

  // Tracker
  double max_match_distance = 20;
  int tracking_threshold = 5;
  int lost_threshold = 5;
  tracker_ = std::make_unique<Tracker>(kf_matrices_, max_match_distance, tracking_threshold, lost_threshold);  // 实例化tracker
}

int once_flag = 0;

/**
 * @brief 
 * 
 * @param armors_msg 
 */
// void ArmorProcessor::armorsCallback(const Armors * armors_msg)
roborts_msgs::Target ArmorProcessor::armorsCallback(Armors * armors_msg)
{
  // Tranform armor position from image frame to world coordinate
  // for (auto & armor : armors_msg->armors) {
  //   geometry_msgs::PointStamped ps;  // 初始化带时间戳的点
  //   ps.header = armors_msg->header;
  //   ps.point = armor.position;
  //   try {
  //     armor.position = tf2_buffer_->transform(ps, target_frame_).point;
  //   } catch (const tf2::ExtrapolationException & ex) {
  //     // RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
  //     return;
  //   }
  // }

  roborts_msgs::Target target_msg;
  ros::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  // target_msg.header.frame_id = target_frame_;

  // if (tracker_->tracker_state == Tracker::LOST) {
  //   tracker_->init(armors_msg);
  //   target_msg.tracking = false;
  // }
  // else {
  //   // 计算dt Set dt
  //   dt_ = (time - last_time_).toSec();
  //   // Update state
  //   tracker_->update(armors_msg, dt_);

  //   if (tracker_->tracker_state == Tracker::DETECTING) {
  //     target_msg.tracking = false;
  //   } else if (
  //     tracker_->tracker_state == Tracker::TRACKING ||
  //     tracker_->tracker_state == Tracker::TEMP_LOST) {
  //     target_msg.tracking = true;
  //     target_msg.id = tracker_->tracking_id;
  //   }
  // }

  if(detection_flag) {

    if(!once_flag) {
      tracker_->init(armors_msg);
      ROS_WARN("tracker initiaed");
      once_flag = 1;
    }


    // 计算dt Set dt
    dt_ = (time - last_time_).toSec();
    ROS_WARN("dt_ : %.4f", dt_);
    // Update state
    tracker_->update(armors_msg, dt_);
    ROS_WARN("tracker updated");
    target_msg.id = tracker_->tracking_id;
    target_msg.tracking = true;
  }
  else {
    once_flag = 0;
  }

  if (target_msg.tracking) {
    target_msg.position.x = tracker_->target_state(0);
    target_msg.position.y = tracker_->target_state(1);
    target_msg.position.z = tracker_->target_state(2);
    target_msg.velocity.x = tracker_->target_state(3);
    target_msg.velocity.y = tracker_->target_state(4);
    target_msg.velocity.z = tracker_->target_state(5);
  }

  ROS_WARN("target_msg.tracking : %d", target_msg.tracking);
  ROS_WARN("tracker_state : %d", tracker_->tracker_state);


  last_time_ = time;

  return target_msg;
}















ConstraintSet::~ConstraintSet() {

}
} //namespace roborts_detection
