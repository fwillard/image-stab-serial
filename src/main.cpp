#include <opencv2/opencv.hpp>

#include <iostream>

int main(int argc, char **argv) {
  cv::Mat img = cv::imread("./../test_img.jpg");

  if (img.empty()) {
    std::cout << "Could not open or find the image" << std::endl;
    std::cin.get(); // wait for any key press
    return -1;
  }

  cv::String windowName = "Open CV Test"; // Name of the window

  cv::namedWindow(windowName); // Create a window

  cv::imshow(windowName, img); // Show our image inside the created window.

  cv::waitKey(0); // Wait for any keystroke in the window

  cv::destroyWindow(windowName); // destroy the created window
  return 0;
}