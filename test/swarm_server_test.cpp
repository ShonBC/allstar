/**
 * @file swarm_server_test.cpp
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
#include "../include/swarm_server.h"


std::vector<std::vector<double>> store_vec;  // stores goal points captured on "/goals"

void chatterCallback(const tuw_multi_robot_msgs::RobotGoalsArray::ConstPtr& ptr) {
    /* iterate over robots member*/
    for (auto robot : ptr->robots) {
        /* destinations is of type array, even though we have just one destination for each robot */
        for (auto destination : robot.destinations) {
            std::vector<double> temp;

            temp.push_back(destination.position.x);
            temp.push_back(destination.position.y);
            /* push back each goal {xi, yi} */
            store_vec.push_back(temp);
        }
    }
}

/**
 * @brief Test for AssignGoalPoints()
 **/
TEST(test_swarmServer_1, check_assign_goal_points) {
    ros::NodeHandle n;
    std::vector<std::vector<double>> expected_goal_points{{10, 10}, {20, 25}, {35, 30}};
    SwarmServer swarm;

    /* create subscriber that listens to "/goals" */
    ros::Subscriber sub = n.subscribe("goals", 1000, chatterCallback);

    /* publish goal points */
    swarm.AssignGoals(expected_goal_points);

    ros::spinOnce();
    /* compare expected and obtained vector of goal points */
    auto result = std::equal(expected_goal_points.begin(), expected_goal_points.end(), store_vec.begin());
    ASSERT_EQ(result, true);
}
