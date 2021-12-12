/**
 * @file image_processor.h
 * @author Shon Cortes (scortes3@umd.edu), Sameer Pusegaonkar (sameer@umd.edu), Pooja Kabra (pkabra@terpmail.umd.edu)
 * @brief Class to parse an image and define goal locations in a map.
 * @version 0.1
 * @date 2021-11-29
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

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>


/**
 * @brief Class to parse an image and define goal locations in a map.
 * 
 */
class ImageProcessor {
 private:
  int height_;  // in pixels
  int width_;   // in pixels
  cv::Mat frame_;
  int num_goal_locations_;
  std::vector<std::vector<double>> goal_points_;
  int kernal_size_;

  /**
   * @brief Define the Goal Points in the image frame
   * @param binary_image Binary image with the outline of the shape the goal points will define
   */
  void GetGoalPoints(cv::Mat binary_image);

  /**
   * @brief Remove any excess goal points
   * @param num_agents number of robots being used in swarm
   */
  void RemoveExcessGoalPoints(int num_agents);

 public:
  /**
   * @brief Construct a new Image Processor object
   * @param frame Image to be processes
   */
  explicit ImageProcessor(cv::Mat frame);
  /**
   * @brief Perform Canny edge detection to define a binary image with the outline of the desired shape
   * @return cv::Mat Binary image 
   */
  cv::Mat GetEdges();

  /**
   * @brief Iterate the kernal size as necessary until the goal points equals the number of robots used in swarm
   * @param num_agents Number of robots in the swarm
   * @param binary_image Binary image with the outline of the desired shape
   * @return std::vector<std::vector<double>> List of goal locations in the image frame
   */
  std::vector<std::vector<double>> RefineGoalPoints(int num_agents,
                                              cv::Mat binary_image);

  /**
   * @brief With a kernal size of 1x1 pixel, collect goal locations. 
   * Sort the goal locations and remove excess until num_goals equals the number of robots used in swarm.
   * @param num_agents Number of robots in the swarm
   * @param binary_image Binary image with the outline of the desired shape
   * @return std::vector<std::vector<double>> List of goal locations in the image frame
   */
  std::vector<std::vector<double>> ImprovedRefineGoalPoints(
                                      int num_agents, cv::Mat binary_image);
  /**
   * @brief Transforms the Goal Locations into the map frame
   * @return std::vector<std::vector<double>> List of goal locations in the map frame
   */
  std::vector<std::vector<double>> TransformToMapCoordinates();

  /**
   * @brief Get the frame height
   * @return int Frame height
   */
  int GetHeight();

  /**
   * @brief Get the image width
   * @return int Image width
   */
  int GetWidth();

  /**
   * @brief Get the frame matrix
   * @return cv::Mat Frame
   */
  cv::Mat GetFrame();

  /**
   * @brief Get the Goal Location Count 
   * @return int Goal location count
   */
  int GetGoalLocationCount();

  /**
   * @brief Get the list of goal locations
   * @return std::vector<std::vector<double>> List of goal locations
   */
  std::vector<std::vector<double>> GetGoalPoints();

  /**
   * @brief Get the kernal size
   * @return int Kernal size
   */
  int GetKernalSize();

  /**
   * @brief Destroy the Image Processor object
   */
  ~ImageProcessor() {}
};
