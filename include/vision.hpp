#ifndef VISION_HEADER
#define VISION_HEADER

#include <iostream>
#include <string>
#include <cstring>
#include <ros/ros.h>
#include <tuple>
#include "copter.hpp"

#ifdef ROS_NOETIC
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#else
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

namespace EMIRO
{
  class Vision
  {
  private:
    int camera_idx;
    std::string camera_string;
    int width, height;
    EMIRO::Copter copter;

  public:
    Vision();
    void init(EMIRO::Copter *copter, int camera_idx = 0, int width = 800, int height = 600);
    WayPoint Position(WayPoint currentPos, Color_Range range);
    std::tuple<bool, WayPoint> Position2(WayPoint currentPos, Color_Range range, int erode_int, int dilate_int, int tolerance_range = 80, int timeout = 0, int max_radius = 90, int min_radius = 10);
    void scan_payload(Color_Range &range, int timeout);
    ~Vision();
  };

  Vision::Vision()
  {
  }

  void Vision::init(EMIRO::Copter *copter, int camera_idx, int width, int height)
  {
    this->copter = *copter;
    this->camera_idx = camera_idx;
    this->width = width;
    this->height = height;

    char buffer_int[5];
    const char temp1[26] = "v4l2src device=/dev/video";
    const char temp2[23] = " ! video/x-raw, width=";
    const char temp3[10] = ", height=";
    const char temp4[51] = " ! videoconvert ! video/x-raw,format=BGR ! appsink";
    char temp_char[256] = "";
    strcat(temp_char, temp1);
    std::snprintf(buffer_int, sizeof(buffer_int), "%d", camera_idx);
    strcat(temp_char, buffer_int);
    strcat(temp_char, temp2);
    std::snprintf(buffer_int, sizeof(buffer_int), "%d", width);
    strcat(temp_char, buffer_int);
    strcat(temp_char, temp3);
    std::snprintf(buffer_int, sizeof(buffer_int), "%d", height);
    strcat(temp_char, buffer_int);
    strcat(temp_char, temp4);
    camera_string = temp_char;
    std::cout << "Camera set :"
              << "\n\tIndex\t: " << this->camera_idx << "\n\tWidth\t: " << this->width << "\n\tHeight\t: " << this->height << "\n\tString\t: " << camera_string << '\n';
  }

  void Vision::Position(int timeout)
  {
    cv::Mat img;
    cv::VideoCapture cap(camera_string);

    LinearSpeed linear_temp = {0.0f, 0.0f};

    // Get camera dimension
    cap >> img;
    int cameraWidth = img.size().width;
    int cameraHeight = img.size().height;
    std::cout << "Camera Properties\n"
              << "\tWidth\t: " << cameraWidth << "\tHeight\t: " << cameraHeight << '\n';

    std::vector<cv::Vec3f> circles;

    // Dilate
    cv::Mat elementDilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));

    cv::Mat img_hsv, img_hsv_result, img_hsv_dilate;

    int key = 0;
    int counting = 0;
    cv::Point center;
    std::string cmd;
    int posx, posy, circleRadius = 0;

    ros::Rate r(10);
    int go_counter = 15;
    bool go_flag = false;
    while (cap.isOpened() && ros::ok() && timeout > 0)
    {
      timeout--;
      cap >> img;
      if ((go_counter <= 0 && go_flag) || !go_flag)
      {
        go_counter = 15;
        go_flag = false;
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
        cv::inRange(img_hsv,
                    cv::Scalar(range.Low.Hue, range.Low.Saturation, range.Low.Value),
                    cv::Scalar(range.High.Hue, range.High.Saturation, range.High.Value),
                    img_hsv_result);
        cv::dilate(img_hsv_result, img_hsv_dilate, elementDilate);
        cv::GaussianBlur(img_hsv_dilate, img_hsv_dilate, cv::Size(31, 31), 0, 0);
        circles.clear();
        cv::HoughCircles(img_hsv_dilate, circles, cv::HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);

        // Draw Circles
        if (circles.size() != 0)
        {
          if (circles.size() > 1)
          {
            // Get Largest Circle
            int radius = 0;
            for (size_t i = 0; i < circles.size(); i++)
              if (cvRound(circles[i][2]) > radius)
              {
                center = cv::Point(cvRound(circles[i][0]), cvRound(circles[i][1]));
                radius = cvRound(circles[i][2]);
                circleRadius = radius;
              }
          }
          else
          {
            center = cv::Point(cvRound(circles[0][0]), cvRound(circles[0][1]));
            int radius = cvRound(circles[0][2]);
            circleRadius = radius;
          }

          posx = circles[0][0] - cameraWidth / 2;
          posy = circles[0][1] - cameraHeight / 2;
          cmd = "";
          bool center_x = false, center_y = false;
          if (circleRadius > 0.7 * cameraHeight / 2 || circleRadius > 0.7 * cameraWidth / 2)
            cmd = "Object is too close";
          else
          {
            // Camera heading is reverse
            if (posx < -40)
            {
              cmd = "Right";
              temp_pos.x += 0.1f;
            }
            else if (posx > 40)
            {
              cmd = "Left";
              temp_pos.x -= 0.1f;
            }
            else
            {
              cmd = "Center x";
              center_x = true;
            }

            if (posy < -40)
            {
              cmd = cmd + ", Back";
              temp_pos.y -= 0.1f;
            }
            else if (posy > 40)
            {
              cmd = cmd + ", Front";
              temp_pos.y += 0.1f;
            }
            else
            {
              cmd = cmd + ", Center y";
              center_y = true;
            }
          }
          if (center_x && center_y)
          {
            ROS_INFO("Vision Position in center target");
            break;
          }
          std::cout << cmd << std::endl;
          this->copter.Go(temp_pos);
          go_flag = true;
        }
      }
      if (go_flag)
        go_counter--;
      time_out_vision--;
      if (time_out_vision <= 0)
      {
        ROS_INFO("Vision break by timeout");
        break;
      }
      cv::waitKey(1);
      ros::spinOnce();
      r.sleep();
    }
    cap.release();
    return temp_pos;
  }

  // is detected, currentPos
  std::tuple<bool, WayPoint> Vision::Position2(WayPoint currentPos, Color_Range range, int erode_int, int dilate_int, int tolerance_range, int timeout, int max_radius, int min_radius)
  {
    bool isTimeoutSet = timeout > 0 ? true : false;
    if (isTimeoutSet)
      ROS_INFO("Vision timeout set : %d", timeout);
    cv::Mat img;
    cv::VideoCapture cap;
    WayPoint temp_pos = currentPos;
    std::vector<cv::Vec3f> circles;
    cv::Mat img_hsv, img_temp;

    int key = 0;
    int counting = 0;
    cv::Point center;
    std::string cmd;
    int posx, posy, circleRadius = 0;

    ros::Rate r(1);
    int go_counter = 15;
    bool go_flag = false, is_detected = false;
    this->copter.Go(temp_pos);

    bool erode_enable = erode_int > 0 ? true : false;
    bool dilate_enable = dilate_int > 0 ? true : false;

    cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erode_int + 1, 2 * erode_int + 1));
    cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilate_int + 1, 2 * dilate_int + 1));

    if (erode_enable)
      ROS_INFO("Erode ENABLE %d= %d", erode_int, 2 * erode_int + 1);
    else
      ROS_INFO("Erode DISABLE");

    if (dilate_enable)
      ROS_INFO("Dilate ENABLE %d= %d", dilate_int, 2 * dilate_int + 1);
    else
      ROS_INFO("Dilate DISABLE");

    ROS_INFO("Scanning circle ...");

    while (ros::ok())
    {
      if (this->copter.is_reached(temp_pos, 0.1f))
      {
        // ros::Duration(1).sleep();
        cap.open(camera_string);
        cap >> img;
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
        cv::inRange(img_hsv,
                    cv::Scalar(range.Low.Hue, range.Low.Saturation, range.Low.Value),
                    cv::Scalar(range.High.Hue, range.High.Saturation, range.High.Value),
                    img_temp);

        if (erode_enable)
          cv::erode(img_temp, img_temp, element_erode);

        if (dilate_enable)
          cv::dilate(img_temp, img_temp, element_dilate);

        cv::GaussianBlur(img_temp, img_temp, cv::Size(31, 31), 0, 0);
        cv::bitwise_not(img_temp, img_temp);

        circles.clear();
        cv::HoughCircles(img_temp, circles, cv::HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);

        // Draw Circles
        if (circles.size() != 0)
        {
          is_detected = true;
          if (circles.size() > 1)
          {
            // Get Largest Circle
            int radius = 0;
            for (size_t i = 0; i < circles.size(); i++)
              if (cvRound(circles[i][2]) > radius)
              {
                center = cv::Point(cvRound(circles[i][0]), cvRound(circles[i][1]));
                radius = cvRound(circles[i][2]);
                circleRadius = radius;
              }
          }
          else
          {
            center = cv::Point(cvRound(circles[0][0]), cvRound(circles[0][1]));
            int radius = cvRound(circles[0][2]);
            circleRadius = radius;
          }

          posx = circles[0][0] - width / 2;
          posy = circles[0][1] - height / 2;
          cmd = "";
          bool center_x = false, center_y = false;
          if (circleRadius > min_radius && circleRadius <= max_radius)
          {
            // Camera heading is reverse
            if (posx < -tolerance_range)
            {
              cmd = "Right";
              temp_pos.x += 0.1f;
            }
            else if (posx > tolerance_range)
            {
              cmd = "Left";
              temp_pos.x -= 0.1f;
            }
            else
            {
              cmd = "Center x";
              center_x = true;
            }

            if (posy < -tolerance_range)
            {
              cmd = cmd + ", Back";
              temp_pos.y -= 0.1f;
            }
            else if (posy > tolerance_range)
            {
              cmd = cmd + ", Front";
              temp_pos.y += 0.1f;
            }
            else
            {
              cmd = cmd + ", Center y";
              center_y = true;
            }
          }
          else if (circleRadius > max_radius)
          {
            cmd = "Object is too close";
          }
          else
          {
            cmd = "Object is too small";
          }

          if (center_x && center_y)
          {
            ROS_INFO("Vision Position in center target");
            cap.release();
            break;
          }
          ROS_INFO("%s\tposx : %d, posy : %d", cmd.c_str(), posx, posy);
          this->copter.Go(temp_pos);
        }
        cap.release();
        if (isTimeoutSet)
        {
          timeout--;
          if (timeout <= 0)
          {
            ROS_INFO("Break by Timeout");
            break;
          }
          ROS_INFO("Timeout Counter %d", timeout);
        }
      }
      ros::spinOnce();
      r.sleep();
    }
    ROS_INFO("Vision positioning result :\nx\t: %f\ny\t: %f\nz\t: %f", temp_pos.x, temp_pos.y, temp_pos.z);
    return std::make_tuple(is_detected, temp_pos);
  }

  inline void scan_payload(Color_Range &range, int timeout)
  {
    cv::Mat img;
    cv::VideoCapture cap;
    std::vector<cv::Vec3f> circles;
    cv::Mat img_hsv, img_temp;

    int key = 0;
    int counting = 0;
    cv::Point center;
    std::string cmd;
    int posx, posy, circleRadius = 0;

    ros::Rate r(1);

    cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erode_int + 1, 2 * erode_int + 1));
    cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilate_int + 1, 2 * dilate_int + 1));

    while (ros::ok() && timeout > 0)
    {
      timeout--;
      cap.open(camera_string);
      cap >> img;
      cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
      cv::inRange(img_hsv,
                  cv::Scalar(range.Low.Hue, range.Low.Saturation, range.Low.Value),
                  cv::Scalar(range.High.Hue, range.High.Saturation, range.High.Value),
                  img_temp);

      if (erode_enable)
        cv::erode(img_temp, img_temp, element_erode);

      if (dilate_enable)
        cv::dilate(img_temp, img_temp, element_dilate);

      cv::GaussianBlur(img_temp, img_temp, cv::Size(31, 31), 0, 0);
      cv::bitwise_not(img_temp, img_temp);

      circles.clear();
      cv::HoughCircles(img_temp, circles, cv::HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);

      // Draw Circles
      if (circles.size() != 0)
      {
        is_detected = true;
        if (circles.size() > 1)
        {
          // Get Largest Circle
          int radius = 0;
          for (size_t i = 0; i < circles.size(); i++)
            if (cvRound(circles[i][2]) > radius)
            {
              center = cv::Point(cvRound(circles[i][0]), cvRound(circles[i][1]));
              radius = cvRound(circles[i][2]);
              circleRadius = radius;
            }
        }
        else
        {
          center = cv::Point(cvRound(circles[0][0]), cvRound(circles[0][1]));
          int radius = cvRound(circles[0][2]);
          circleRadius = radius;
        }

        posx = circles[0][0] - width / 2;
        posy = circles[0][1] - height / 2;
        cmd = "";
        bool center_x = false, center_y = false;
        if (circleRadius > min_radius && circleRadius <= max_radius)
        {
          // Camera heading is reverse
          if (posx < -tolerance_range)
          {
            cmd = "Right";
            temp_pos.x += 0.1f;
          }
          else if (posx > tolerance_range)
          {
            cmd = "Left";
            temp_pos.x -= 0.1f;
          }
          else
          {
            cmd = "Center x";
            center_x = true;
          }

          if (posy < -tolerance_range)
          {
            cmd = cmd + ", Back";
            temp_pos.y -= 0.1f;
          }
          else if (posy > tolerance_range)
          {
            cmd = cmd + ", Front";
            temp_pos.y += 0.1f;
          }
          else
          {
            cmd = cmd + ", Center y";
            center_y = true;
          }
        }
        else if (circleRadius > max_radius)
        {
          cmd = "Object is too close";
        }
        else
        {
          cmd = "Object is too small";
        }

        if (center_x && center_y)
        {
          ROS_INFO("Vision Position in center target");
          cap.release();
          break;
        }
        ROS_INFO("%s\tposx : %d, posy : %d", cmd.c_str(), posx, posy);
        this->copter.Go(temp_pos);
      }
      cap.release();
      if (isTimeoutSet)
      {
        timeout--;
        if (timeout <= 0)
        {
          ROS_INFO("Break by Timeout");
          break;
        }
        ROS_INFO("Timeout Counter %d", timeout);
      }
    }

    ros::spinOnce();
    r.sleep();
  }
}

Vision::~Vision()
{
}

#endif // VISION_HEADER
}
