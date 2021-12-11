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


cv::Mat image = cv::imread("/home/poo/Downloads/test.jpg");
auto img = new ImageProcessor(image);
cv::Mat countours = img->GetEdges();
int num_agents = 5;

/**
 * @brief Test for GetHeight()
 **/
TEST(test_imageProcessor_1, check_get_height)
{
  int obtained_height= img->GetHeight();
  ASSERT_EQ(obtained_height, 500);
}

/**
 * @brief Test for GetWidth()
 **/
TEST(test_imageProcessor_2, check_get_width)
{
  int obtained_width= img->GetWidth();
  ASSERT_EQ(obtained_width, 500);
}

/**
 * @brief Test for GetKernalSize()
 **/
TEST(test_imageProcessor_3, check_get_kernal_size)
{
  int obtained_kernal_size= img->GetKernalSize(); 
  ASSERT_EQ(obtained_kernal_size, 499);
}

/**
 * @brief Test for RefineGoalPoints()
 **/
TEST(test_imageProcessor_4, check_refine_goal_points)
{
  std::vector<std::vector<double>> expected_refined_goal_points{
    {82,82}, {247,82}, {412,82}, {82,247}, {247,247}};

  img->RefineGoalPoints(num_agents, countours);
  std::vector<std::vector<double>> obtained_refined_goal_points = img->GetGoalPoints();
  
  ASSERT_EQ(obtained_refined_goal_points, expected_refined_goal_points);
}


/**
 * @brief Test for GetGoalLocationCount()
 **/
TEST(test_imageProcessor_5, check_get_goal_location_count)
{
  int obtained_goal_location_count= img->GetGoalLocationCount();

  ASSERT_EQ(obtained_goal_location_count, num_agents);
}

/**
 * @brief Test for TransformToMapCoordinates()
 **/
TEST(test_imageProcessor_6, check_transform_to_map_coordinates)
{
  std::vector<std::vector<double>> expected_transformed_goal_points{
    {-8.4,8.4}, {-0.15,8.4}, {8.1,8.4}, {-8.4,0.15}, {-0.15,0.15}};

  auto obtained_transformed_goal_points = img->TransformToMapCoordinates();

  ASSERT_EQ(obtained_transformed_goal_points, expected_transformed_goal_points);

}

/**
 * @brief Test for ImprovedRefineGoalPoints()
 **/
TEST(test_imageProcessor_6, check_improve_refined_goal_points)
{

}