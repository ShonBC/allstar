#include "../include/image_processor.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

int main() {
  cv::Mat image = cv::imread("/home/sameer/catkin_ws/src/allstar/src/test.jpg");
  cv::imshow("window", image);
  cv::waitKey(0);
  auto img = new ImageProcessor(image);
  img->GetEdges();
}
