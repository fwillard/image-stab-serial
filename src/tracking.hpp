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

const cv::Mat gaussian_kernel = (cv::Mat_<float>(5,5) <<1,  4,  7,  4, 1,
                                                        4, 16, 26, 16, 4,
                                                        7, 26, 41, 26, 7,
                                                        4, 16, 26, 16, 4,
                                                        1,  4,  7,  4, 1);

const cv::Mat sobel_x_kernel = (cv::Mat_<float>(3,3) << 1, 0, -1,
                                                        2, 0, -2,
                                                        1, 0, -1);

const cv::Mat sobel_y_kernel = (cv::Mat_<float>(3,3) <<  1,  2,  1,
                                                         0,  0,  0,
                                                        -1, -2, -1);

void lucas_kanade(cv::Mat, cv::Mat, std::vector<cv::Point2f>, std::vector<cv::Point2f> &);
void pyr_LK(cv::Mat, cv::Mat, std::vector<cv::Point2f>, std::vector<cv::Point2f> &);
float bilinear(cv::Mat I, float x, float y);
float magnitude(cv::Mat in);
cv::Mat calculate_It(cv::Mat I, cv::Mat J, float gx, float gy, float vx, float vy);
std::vector<cv::Mat> construct_gaussian_pyramid(cv::Mat, int levels = 3);
cv::Mat convolve(cv::Mat, cv::Mat);
cv::Mat sobel_filter(cv::Mat, int);
short sobel_filter(cv::Mat, int, int, int);
cv::Mat gaussian_blur(cv::Mat);
cv::Mat sub_sample(cv::Mat);
cv::Mat invert(cv::Mat);
//cv::Mat gaussian_kernel(int, int, double);
#endif /* optical_flow_hpp */
