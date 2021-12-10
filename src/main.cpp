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
    if (argc <= 1) {
    ROS_WARN_STREAM
    ("Enter the # of Robots (robots_) and absolute Image File Path.");
    return 1;
    }
  ROS_INFO_STREAM(argc << "argc");
  ros::init(argc, argv, "main");
  cv::Mat image = cv::imread(argv[2]);
  auto img = new ImageProcessor(image);
  auto countours = img->GetEdges();
  img->RefineGoalPoints(std::atoi(argv[1]), countours);
  // auto points = img->GetGoalPoints();
  auto points = img->TransformToMapCoordinates();
  ROS_INFO_STREAM("Got " << points.size() << " goal points!");
  SwarmServer swarm;
  swarm.num_agents = std::atoi(argv[1]);
  ROS_INFO_STREAM("Kernel size: " << img->GetKernalSize());
  swarm.AssignGoals(points);
  ROS_INFO_STREAM("Finished assigning points!");
}
