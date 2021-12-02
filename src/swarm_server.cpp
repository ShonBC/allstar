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


void SwarmServer::InitiateMap() {
    // To-Do
    ros::NodeHandle n;
    ros::Publisher chatter_pub =
        n.advertise<nav_msgs::OccupancyGrid>("map", 1000);
    // ros::Subscriber chatter_sub = n.subscribe("map", 1000, callback);
    ros::Rate loop_rate(10);
    // ros::spin();
    std::vector<int8_t> vec(250000, 0);
    while (ros::ok()) {
        nav_msgs::OccupancyGrid msg;
        msg.info.height = 500;
        msg.info.width = 500;
        msg.info.origin.orientation.w = 1;
        msg.info.origin.orientation.x = 0;
        msg.info.origin.orientation.y = 0;
        msg.info.origin.orientation.z = 0;
        msg.info.origin.position.x = 0;
        msg.info.origin.position.y = 0;
        msg.info.origin.position.z = 0;
        msg.data = vec;
        chatter_pub.publish(msg);

        ros::spinOnce();
        // std::cout << "working" << std::endl;

        loop_rate.sleep();
    }
}

void SwarmServer::AssignInitPos() {
    // To-do
}

void SwarmServer::AssignGoals(std::vector<std::vector<double>> goal_points) {
    // To-Do
}
