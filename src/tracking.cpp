//
//  optical_flow.cpp
//  ImageStab
//
//  Created by Finn Willard on 4/22/21.
//

#include "tracking.hpp"
void lucas_kanade(cv::Mat f0, cv::Mat f1, std::vector<cv::Point2f> p0, std::vector<cv::Point2f> &p1){
    
    //ensure that the images are grayscale
    int channels = f0.channels();
    assert(channels == 1);
    channels = f1.channels();
    assert(channels == 1);
    
    
    //ensure the images are the same size
    assert(f0.rows == f1.rows);
    assert(f0.cols == f1.cols);
    
    int num_rows = f0.rows;
    int num_cols = f0.cols;
    
    cv::Mat Ix = cv::Mat::zeros(num_rows, num_cols, f0.type());
    cv::Mat Iy = cv::Mat::zeros(num_rows, num_cols, f0.type());
    
    for(auto point : p0){
        
    }
    
}

std::vector<cv::Mat> construct_gaussian_pyramid(cv::Mat in, int levels){
    std::vector<cv::Mat> out;
    cv::Mat temp, temp_old;
    temp = in;
    out.push_back(temp);
    for (int i = 0; i < levels - 1; i++) {
        temp = gaussian_blur(temp);
        temp = sub_sample(temp);
        out.push_back(temp);
    }

    
    return out;
}

cv::Mat sub_sample(cv::Mat in){
    int num_rows = in.rows;
    int num_cols = in.cols;
    cv::Mat out = cv::Mat::zeros(num_rows/2, num_cols/2, in.type());
    
    for(int i = 0; i < num_rows; i += 2){
        for(int j = 0; j < num_cols; j += 2){
            out.at<uchar>(i/2, j/2) = in.at<uchar>(i, j);
        }
    }
    
    return out;
}

cv::Mat gaussian_blur(cv::Mat in){
    
    int num_rows = in.rows;
    int num_cols = in.cols;
    int filter_rows = gaussian_kernel.rows;
    int filter_cols = gaussian_kernel.cols;
    
    cv::Mat out = cv::Mat::zeros(num_rows, num_cols, in.type());
    
    for(int i = 0; i < num_rows - 2; i++){
        for(int j = 0; j < num_cols - 2; j++){
            double sum = 0.0;
            for(int k = 0; k < filter_rows; k++){
                for(int l = 0; l < filter_cols; l++){
                    int row_idx = i + k - 2;
                    int col_idx = j + l - 2;
                    if(row_idx >= 0 && row_idx < num_rows){
                        if(col_idx >= 0 && col_idx < num_cols)
                        sum += gaussian_kernel.at<uchar>(k, l) * in.at<uchar>(row_idx, col_idx);
                    }
                }
            }
            out.at<uchar>(i,j) = sum/273;
        }
    }

    return out;
}
