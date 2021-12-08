/**
 * @file image_processor.cpp
 * @author Shon Cortes (scortes3@umd.edu), Sameer Pusegaonkar (sameer@umd.edu), Pooja Kabra (pkabra@terpmail.umd.edu)
 * @brief Class to parse an image and define goal locations in a map.
 * @version 0.1
 * @date 2021-11-29
 * 
 * @copyright Copyright (c) 2021
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the 
 *      documentation and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this 
 *      software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include <ros/console.h>
#include "../include/image_processor.h"

ImageProcessor::ImageProcessor(cv::Mat img) {
  this->frame_ = img;
  this->height_ = img.size().height;
  this->width_ = img.size().width;
  this->kernal_size_ = 499;
  ROS_INFO_STREAM("Image loaded!");
  ROS_INFO_STREAM(this->height_ << " pixels :Image height");
  ROS_INFO_STREAM(this->width_ << " pixels :Image width");
  ROS_INFO_STREAM(this->kernal_size_ << "x" << this->kernal_size_ << " : Kernal size");
  ROS_DEBUG_STREAM(this->height_ << " pixels :Image height");
  ROS_DEBUG_STREAM(this->width_ << " pixels :Image width");
}

void ImageProcessor::GetGoalPoints(cv::Mat binary_image) {
  goal_points_.clear();
  ROS_DEBUG_STREAM("Received an image!");
  ROS_DEBUG_STREAM(kernal_size_ << ": Current Kernel size");
  // Pass Kernal over image
  for (int i = 0; i < height_ - kernal_size_ ; i = i + kernal_size_) {
    for (int j = 0; j < width_ -kernal_size_; j = j + kernal_size_) {
      cv::Mat kern_window = binary_image(cv::Range(i, i+ kernal_size_),
                                          cv::Range(j, j + kernal_size_));
      // ROS_INFO_STREAM("Created a kernal!");
      if (cv::countNonZero(kern_window) > 0) {
        /* If edge is within kernal, add location of center of kernal
        *  to vector of goal locations
        */
        double x_center = j + (kernal_size_  / 2);
        double y_center =  i + (kernal_size_  / 2);
        std::vector<double> center{x_center, y_center};
        goal_points_.push_back(center);
      }
    }
  }

  num_goal_locations_ = goal_points_.size();
  ROS_DEBUG_STREAM(num_goal_locations_ << ": Number of goal points so far");
  ROS_DEBUG_STREAM("Completed GetGoalPoints!");
}

void ImageProcessor::RemoveExcessGoalPoints(int num_agents) {
  std::vector<std::vector<double>> new_points;
  for ( auto i = 0; i < this->goal_points_.size(); i++ ) {
    if ( i < num_agents ) {
        new_points.push_back(this->goal_points_[i]);
    }
  }
  goal_points_ = new_points;
}

cv::Mat ImageProcessor::GetEdges() {
  cv::Mat contours;
  cv::Mat gray_image;

  cv::resize(this->frame_, this->frame_, cv::Size(500, 500), cv::INTER_LINEAR);
  this->height_ = 500;
  this->width_ = 500;

  cvtColor(this->frame_, gray_image, cv::COLOR_BGR2GRAY);

  cv::Canny(this->frame_, contours, 10, 350);

  cv::namedWindow("Image");
  cv::imshow("Image", this->frame_);

  cv::namedWindow("Gray");
  cv::imshow("Gray", gray_image);

  cv::namedWindow("Canny");
  cv::imshow("Canny", contours);
  this->frame_ = contours;
  cv::waitKey(0);

  for ( auto i  = 0; i < this->height_; i++ ) {
    for ( auto j = 0; j < this->width_; j++ ) {
      ROS_DEBUG_STREAM((int)contours.at<uchar>(i, j)<< ":Color, " << i << ":i, " << j << ":j");
    }
  }
  return contours;
}

std::vector<std::vector<double>> ImageProcessor::RefineGoalPoints(
                                        int num_agents, cv::Mat binary_image) {
    GetGoalPoints(binary_image);
    int step_size = 2;  // kernal step size

    if (num_goal_locations_ == num_agents) {
        // If num_goal_locations equals num_agents return vector of goal points
        return goal_points_;
    } else if (num_goal_locations_< num_agents) {
        // Reduce kernal size to increase the number of goal locations
        kernal_size_ -= step_size;
        GetGoalPoints(binary_image);

        if (num_goal_locations_ > num_agents) {
            /* Potential infinite loop due to kernal step size,
             * remove excess goal points
             */
            RemoveExcessGoalPoints(num_agents);
            return goal_points_;
        } else {
            RefineGoalPoints(num_agents, binary_image);
            }
    } else {
        // Increase kernal size to reduce the number of goal locations
        kernal_size_ += step_size;
        RefineGoalPoints(num_agents, binary_image);
        }
}

std::vector<std::vector<double>> ImageProcessor::TransformToMapCoordinates() {
  std::vector<std::vector<double>> transformed_points_;
  for ( auto points : this->goal_points_ ) {
    std::vector<double>new_point;
    auto x = points[0];
    auto y = points[1];
    double new_x = x/20;
    double new_y = -y/20;
    ROS_INFO_STREAM(x << ": ImageX, " << y << ": ImageY");
    ROS_INFO_STREAM(new_x << ": MapX, " << new_y << ": MapY");
    new_point.push_back(new_x);
    new_point.push_back(new_y);
    transformed_points_.push_back(new_point);
  }
  return transformed_points_;
}

int ImageProcessor::GetHeight() {
    return height_;
}

int ImageProcessor::GetWidth() {
    return width_;
}

cv::Mat ImageProcessor::GetFrame() {
    return frame_;
}

int ImageProcessor::GetGoalLocationCount() {
    return num_goal_locations_;
}

std::vector<std::vector<double>> ImageProcessor::GetGoalPoints() {
    return goal_points_;
}

int ImageProcessor::GetKernalSize() {
    return kernal_size_;
}
