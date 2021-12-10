#include "../include/swarm_server.h"
#include "../include/image_processor.h"

// int main(int argc, char **argv) {
//   ros::init(argc, argv, "walker");

//   std::vector<std::vector<double>> input = {
//     {1, 1},
//     {3, 3}
//     // {5, 5}
//     // {34, 34}
//   };
//   SwarmServer swarm;
//   // swarm.InitiateMap();
//   swarm.AssignGoals(input);
// }
int main(int argc, char** argv) {
  ros::init(argc, argv, "main");
  cv::Mat image = cv::imread("/home/sameer/catkin_ws/src/allstar/src/test.jpg");
  auto img = new ImageProcessor(image);
  auto countours = img->GetEdges();
  img->RefineGoalPoints(3, countours);
  // auto points = img->GetGoalPoints();
  auto points = img->TransformToMapCoordinates();
  ROS_INFO_STREAM("Got " << points.size() << " goal points!");
  SwarmServer swarm;
  swarm.AssignGoals(points);
  ROS_INFO_STREAM("Finished assigning points!");
}
