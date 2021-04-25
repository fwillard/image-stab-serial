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
    
    int row_offset = floor(WIN_SIZE.height/2);
    int col_offset = floor(WIN_SIZE.width/2);
    
    f0.convertTo(f0, CV_16S);
    f1.convertTo(f0, CV_16S);
    
    
    
    for(auto point : p0){
        cv::Mat A = cv::Mat::zeros(2, 2, CV_32F);
        cv::Mat B = cv::Mat::zeros(2, 1, CV_32F);
        for(int k = 0; k < WIN_SIZE.height; k++){
            for(int l = 0; l < WIN_SIZE.width; l++){
                
                int row_idx = (int)point.x + k - row_offset;
                int col_idx = (int)point.y + l - col_offset;
                
                if(row_idx >= 0 && row_idx < num_rows){
                    if(col_idx >= 0 && col_idx < num_cols){
                        short dx0 = sobel_filter(f0, 0, row_idx, col_idx);
                        short dx1 = sobel_filter(f1, 0, row_idx, col_idx);
                        
                        short dy0 = sobel_filter(f0, 1, row_idx, col_idx);
                        short dy1 = sobel_filter(f1, 1, row_idx, col_idx);
                        
                        float ix = (float)(dx0 + dx1)/2;
                        float iy = (float)(dy0 + dy1)/2;
                        float it = (float) (f1.at<short>(row_idx, col_idx) - f0.at<short>(row_idx, col_idx));
                        
                        A.at<float>(0,0) += ix * ix;
                        A.at<float>(0,1) += ix * iy;
                        A.at<float>(1,0) += iy * ix;
                        A.at<float>(1,1) += iy * iy;
                        
                        B.at<float>(0,0) -= ix * it;
                        B.at<float>(1,0) -= iy * it;
                    }
                }
            }
        }
    }
}

void invert(cv::Mat &in){
    assert(in.rows == 2);
    assert(in.cols == 2);
    
    float a = in.at<float>(0,0);
    float b = in.at<float>(0,1);
    float c = in.at<float>(1,0);
    float d = in.at<float>(1,1);
    
    float det = 1/(a*d - b*c);
    
    in.at<float>(0,0) = d * det;
    in.at<float>(0,1) = -b * det;
    in.at<float>(1,0) = -c * det;
    in.at<float>(1,1) = a * det;
}

std::vector<cv::Mat> construct_gaussian_pyramid(cv::Mat in, int levels){
    std::vector<cv::Mat> out;
    cv::Mat temp;
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
short sobel_filter(cv::Mat in, int direction, int x, int y){
    int num_rows = in.rows;
    int num_cols = in.cols;
    
    cv::Mat kernel;
    switch (direction) {
        case 0:
            kernel = sobel_x_kernel;
            break;
        case 1:
            kernel = sobel_y_kernel;
            break;
        default:
            std::cerr << "Invalid direction for sobel filter: 0 for X, 1 for Y" << std::endl;
            return;
    }
    int filter_rows = kernel.rows;
    int filter_cols = kernel.cols;
    
    in.convertTo(in, CV_16S);
    
    cv::Mat out = cv::Mat::zeros(num_rows, num_cols, in.type());

    short sum = 0;
    
    for(int k = 0; k < filter_rows; k++){
        for(int l = 0; l < filter_cols; l++){
            int row_idx = x + k - 1;
            int col_idx = y + l - 1;
            if(row_idx >= 0 && row_idx < num_rows){
                if(col_idx >= 0 && col_idx < num_cols){
                    sum += kernel.at<short>(k, l) * in.at<short>(row_idx, col_idx);
                }
            }
        }
    }
    
    return sum;
}
cv::Mat sobel_filter(cv::Mat in, int direction){
    int num_rows = in.rows;
    int num_cols = in.cols;
    cv::Mat kernel;
    switch (direction) {
        case 0:
            kernel = sobel_x_kernel;
            break;
        case 1:
            kernel = sobel_y_kernel;
            break;
        default:
            std::cerr << "Invalid direction for sobel filter: 0 for X, 1 for Y" << std::endl;
            return;
    }
    int filter_rows = kernel.rows;
    int filter_cols = kernel.cols;
    
    in.convertTo(in, CV_16S);
    
    cv::Mat out = cv::Mat::zeros(num_rows, num_cols, in.type());
    
    for(int i = 0; i < num_rows; i++){
        for(int j = 0; j < num_cols; j++){
            for(int k = 0; k < filter_rows; k++){
                for(int l = 0; l < filter_cols; l++){
                    int row_idx = i + k - 1;
                    int col_idx = j + l - 1;
                    if(row_idx >= 0 && row_idx < num_rows){
                        if(col_idx >= 0 && col_idx < num_cols){
                            out.at<short>(i,j) += kernel.at<short>(k, l) * in.at<short>(row_idx, col_idx);
                        }
                    }
                }
            }
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
    
    for(int i = 0; i < num_rows; i++){
        for(int j = 0; j < num_cols; j++){
            double sum = 0.0;
            for(int k = 0; k < filter_rows; k++){
                for(int l = 0; l < filter_cols; l++){
                    int row_idx = i + k - 1;
                    int col_idx = j + l - 1;
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
