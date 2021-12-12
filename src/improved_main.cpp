/**
 * @file improved_main.cpp
 * @author Shon Cortes (scortes3@umd.edu), Sameer Pusegaonkar (sameer@umd.edu), Pooja Kabra (pkabra@terpmail.umd.edu)
 * @brief Improved node that processes an input image and assign goal points to all robots in swarm based off of user input. This implementation uses the ImprovedRefineGoalPoints method of the ImageProcessor Class.
 * @version 0.1
 * @date 2021-12-11
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
#include "../include/image_processor.h"

int main(int argc, char** argv) {
    if (argc <= 1) {
    ROS_WARN_STREAM
    ("Enter the # of Robots (robots_) and absolute Image File Path.");
    return 1;
    }

  ros::init(argc, argv, "main");
  cv::Mat image = cv::imread(argv[2]);
  auto img = new ImageProcessor(image);

  auto bin = img->GetEdges();
  img->ImprovedRefineGoalPoints(std::atoi(argv[1]), bin);
  auto points = img->TransformToMapCoordinates();

  for ( auto i  = 0; i < points.size(); i++ ) {
    ROS_DEBUG_STREAM("goal points: " << points[i][0] << " " << points[i][1]);
  }

  SwarmServer swarm;
  swarm.num_agents = std::atoi(argv[1]);
  ROS_INFO_STREAM("Kernel size: " << img->GetKernalSize());
  swarm.AssignGoals(points);
}
