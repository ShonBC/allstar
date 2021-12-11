/**
 * @file image_processor_test.cpp
 * @author Shon Cortes (scortes3@umd.edu), Sameer Pusegaonkar (sameer@umd.edu), Pooja Kabra (pkabra@terpmail.umd.edu)
 * @brief 
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/image_processor.h"


cv::Mat image = cv::imread("/home/poo/Downloads/heart.png");
auto img = new ImageProcessor(image);
cv::Mat countours = img->GetEdges();

TEST(test_imageProcessor_7, check_get_kernal_size)
{
  int expected_kernal_size = 20;
  img->SetKernalSize(expected_kernal_size);
  int obtained_kernal_size= img->GetKernalSize();
  EXPECT_EQ(obtained_kernal_size, expected_kernal_size);
}

TEST(test_imageProcessor_1, check_get_goal_points)
{
    img->SetKernalSize(50);
    std::vector<std::vector<double>> expected_goal_points{
    {75,25}, {125,25}, {175,25}, {225,25}, {275,25},
    {325,25}, {375,25}, {425,25}, {25,75}, {75,75},
    {225,75}, {275,75}, {425,75}, {475,75}, {25,125}, {225,125},
    {275,125}, {475,125}, {25,175}, {475,175}, {25,225}, {475,225},
    {25,275}, {75,275}, {425,275}, {475,275}, {75,325}, {125,325},
    {375,325}, {425,325}, {125,375}, {175,375}, {325,375}, {375,375}, 
    {175,425}, {225,425}, {275,425}, {325,425}, {225,475}, {275,475}};

    img->GetGoalPoints(countours);
    std::vector<std::vector<double>> obtained_goal_points = img->GetGoalPoints();
    
    auto result = std::equal(expected_goal_points.begin(), expected_goal_points.end(), obtained_goal_points.begin());

    EXPECT_EQ(result, true);
}

TEST(test_imageProcessor_2, check_remove_excess_goal_points)
{
  int expected_num_goal_points = 3;

  img->RemoveExcessGoalPoints(expected_num_goal_points);
  auto obtained_num_goal_points= img->GetGoalPoints().size();

  EXPECT_EQ(obtained_num_goal_points, expected_num_goal_points);
}

TEST(test_imageProcessor_3, check_transform_to_map_coordinates)
{
  std::vector<std::vector<double>> goal_points{
    {10,20}, {30,30}, {225,200}};
  std::vector<std::vector<double>> expected_transformed_goal_points{
    {0.5,-1}, {1.5,-1.5}, {11.25,-10}};

  auto obtained_transformed_goal_points = img->TransformToMapCoordinates(goal_points);

  EXPECT_EQ(obtained_transformed_goal_points, expected_transformed_goal_points);

}

TEST(test_imageProcessor_4, check_get_height)
{
  int obtained_height= img->GetHeight();
  EXPECT_EQ(obtained_height, 500);
}

TEST(test_imageProcessor_5, check_get_width)
{
  int obtained_width= img->GetWidth();
  EXPECT_EQ(obtained_width, 500);
}

TEST(test_imageProcessor_6, check_get_goal_location_count)
{
  int obtained_goal_location_count= img->GetGoalLocationCount();
  EXPECT_EQ(obtained_goal_location_count, 3);
}



TEST(test_imageProcessor_8, check_get_height)
{
  int obtained_height= img->GetHeight();
  EXPECT_EQ(obtained_height, 500);
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

