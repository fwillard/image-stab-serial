//
//  optical_flow.hpp
//  ImageStab
//
//  Created by Finn Willard on 4/22/21.
//

#ifndef optical_flow_hpp
#define optical_flow_hpp

#include <iostream>
#include <opencv2/core.hpp>
#include <cmath>

#define WIN_SIZE cv::Size(21,21)
#define MAX_LEVEL 3
#define THRESHOLD 1e-4

const cv::Mat gaussian_kernel = (cv::Mat_<uchar>(5,5) << 1,  4,  7,  4, 1,
                                                                        4, 16, 26, 16, 4,
                                                                        7, 26, 41, 26, 7,
                                                                        4, 16, 26, 16, 4,
                                                                        1,  4,  7,  4, 1);

void lucas_kanade(cv::Mat, cv::Mat, std::vector<cv::Point2f>, std::vector<cv::Point2f> &);
std::tuple<cv::Mat, cv::Mat, cv::Mat> construct_gaussian_pyramid(int levels = MAX_LEVEL);
cv::Mat gaussian_blur(cv::Mat, cv::Mat);
//cv::Mat gaussian_kernel(int, int, double);
#endif /* optical_flow_hpp */
