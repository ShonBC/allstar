/**
 * @file swarm_server.cpp
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

#include "../include/swarm_server.h"

void SwarmServer::AssignGoals(std::vector<std::vector<double>> goal_points) {
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<tuw_multi_robot_msgs::RobotGoalsArray>("goals", 1000, true);

    ROS_INFO_STREAM("Assigning goals to robots!");
    sort(goal_points.begin(), goal_points.end());
    for ( auto points : goal_points ) {
    auto x = points[0];
    auto y = points[1];
    ROS_DEBUG_STREAM(x << ": MapX, " << y << ": MapY");
  }
    for (int i = 0; i < goal_points.size(); i++) {
      tuw_multi_robot_msgs::RobotGoalsArray robot_goals_array;
      robot_goals_array.header.frame_id = "map";
      robot_goals_array.header.stamp = ros::Time::now();
      geometry_msgs::Pose pose;
      std::string name;
      tuw_multi_robot_msgs::RobotGoals robot;
      name = "robot_" + std::to_string(i);
      robot.robot_name = name;
      pose.position.x = goal_points[i][0];
      pose.position.y = goal_points[i][1];
      robot.destinations.push_back(pose);
      robot_goals_array.robots.push_back(robot);
      chatter_pub.publish(robot_goals_array);
      ros::spinOnce();
      ros::Duration(1.000001).sleep();
    }
  ROS_INFO_STREAM("Finished assigning goal points!");
}
