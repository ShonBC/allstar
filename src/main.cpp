
#include "../include/swarm_server.h"
#include "../include/image_processor.h"

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "walker");

//     std::vector<std::vector<double>> input = {
//         {1,1},
//         {3,4},
//         {5,5},
//         {4,8}
//     };
//     SwarmServer swarm;
//     // swarm.InitiateMap();
//     swarm.AssignGoals(input);

int main() {
  cv::Mat image = cv::imread("/home/sameer/catkin_ws/src/allstar/src/test.jpg");
  cv::imshow("window", image);
  cv::waitKey(0);
  auto img = new ImageProcessor(image);
  img->GetEdges();
}
