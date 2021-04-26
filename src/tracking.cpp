//
//  optical_flow.cpp
//  ImageStab
//
//  Created by Finn Willard on 4/22/21.
//
#include <opencv2/highgui.hpp>
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
        invert(A);
        float Vx = A.at<float>(0,0) * B.at<float>(0,0) + A.at<float>(0,1) * B.at<float>(1,0);
        float Vy = A.at<float>(1,0) * B.at<float>(0,1) + A.at<float>(1,1) * B.at<float>(1,0);
        
        p1.push_back(cv::Point2f(point.x + Vx, point.y + Vy));
    }
}

void pyr_LK(cv::Mat f0, cv::Mat f1, std::vector<cv::Point2f> p0, std::vector<cv::Point2f> &p1){
    
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
    
    f0.convertTo(f0, CV_32FC1, 1/255.0);
    f1.convertTo(f1, CV_32FC1, 1/255.0);
    
    int num_levels = 3;
    
    auto pyramid_0 = construct_gaussian_pyramid(f0, num_levels);
    auto pyramid_1 = construct_gaussian_pyramid(f1, num_levels);
    
    for(auto a: pyramid_0){
        assert(a.type() == CV_32FC1);
    }
    
    for(auto a: pyramid_1){
        assert(a.type() == CV_32FC1);
    }
    
    int w = 2;
    int max_iterations = 20;
    float threshold = 0.03;
    bool first = true;
    for(auto point: p0){
        cv::Mat g = cv::Mat::zeros(2, 1, CV_32FC1);
        cv::Mat d = cv::Mat::zeros(2, 1, CV_32FC1);
        
        for(int l = num_levels - 1; l >= 0; l--){
//            std::cout << "Level: " << l << std::endl;
            cv::Point2f u;
            u.x = point.x / pow(2, l);
            u.y = point.y / pow(2, l);
            
            auto Ix = sobel_filter(pyramid_0[l], 0);
            auto Iy = sobel_filter(pyramid_0[l], 1);
//            std::cout << Ix << std::endl;
//            std::cout << Iy << std::endl;
            cv::Mat A = cv::Mat::zeros(2, 2, CV_32FC1);
            
            for(float x = u.x - w; x < u.x + w; x++){
                for(float y = u.y - w; y < u.y + w; y++){
                    A.at<float>(0,0) += bilinear(Ix, x, y) * bilinear(Ix, x, y);
                    A.at<float>(0,1) += bilinear(Ix, x, y) * bilinear(Iy, x, y);
                    A.at<float>(1,0) += bilinear(Ix, x, y) * bilinear(Iy, x, y);
                    A.at<float>(1,1) += bilinear(Iy, x, y) * bilinear(Iy, x, y);
                }
            }
//            std::cout << A << std::endl;
            cv::Mat v = cv::Mat::zeros(2, 1, CV_32FC1);
            cv::Mat n = cv::Mat::zeros(2, 1, CV_32FC1);
            
            for(int k = 0; k < max_iterations || magnitude(n) < threshold; k++){
                cv::Mat It = cv::Mat::zeros(pyramid_0[l].rows, pyramid_0[l].cols, CV_32FC1);
                It = calculate_It(pyramid_0[l], pyramid_1[l], g.at<float>(0,0), g.at<float>(1,0), v.at<float>(0,0), v.at<float>(1,0));
//                if(l ==0) {
//                    std::cout << It << std::endl;
//                    cv::imshow("", It);
//                    cv::waitKey(0);
//                }
                
                cv::Mat B = cv::Mat::zeros(2, 1, CV_32FC1);
                
                for(float x = u.x - w; x < u.x + w; x++){
                    for(float y = u.y - w; y < u.y + w; y++){
                        B.at<float>(0,0) += bilinear(Ix, x, y) * bilinear(It, x, y);
                        B.at<float>(1,0) += bilinear(Iy, x, y) * bilinear(It, x, y);
                    }
                }
                cv::Mat A_inv = invert(A);
//                std::cout << A_inv << std::endl;
//                std::cout << B << std::endl;
                n.at<float>(0,0) = A_inv.at<float>(0,0) * B.at<float>(0,0) + A_inv.at<float>(0,1) * B.at<float>(1,0);
                n.at<float>(1,0) = A_inv.at<float>(1,0) * B.at<float>(0,1) + A_inv.at<float>(1,1) * B.at<float>(1,0);
//                std::cout << "n_" << k << ": \n" << n << std:: endl;
                v.at<float>(0,0) += n.at<float>(0,0);
                v.at<float>(1,0) += n.at<float>(1,0);
//                std::cout << "v_" << k << ": \n" << v << std:: endl;
            }
            
            d = v;
            
            g.at<float>(0,0) = 2*(g.at<float>(0,0) + d.at<float>(0,0));
            g.at<float>(1,0) = 2*(g.at<float>(1,0) + d.at<float>(1,0));
        }
        
        cv::Point2f p;
        p.x = point.x + d.at<float>(0,0);
        p.y = point.y + d.at<float>(1,0);
        p1.push_back(p);
    }
    
}

float magnitude(cv::Mat in){
    return sqrt(pow(in.at<float>(0,0), 2) + pow(in.at<float>(1,0), 2));
}

cv::Mat calculate_It(cv::Mat I, cv::Mat J, float gx, float gy, float vx, float vy){
    int num_rows = I.rows;
    int num_cols = I.cols;
    
    cv::Mat out = cv::Mat::zeros(num_rows, num_cols, I.type());
    
    for(int i = 0; i < num_rows; i++){
        for(int j = 0; j < num_cols; j++){
            out.at<float>(i, j) = bilinear(I, i, j) - bilinear(J, i + gx + vx, j + gy + vy);
        }
    }
    return out;
}
float bilinear(cv::Mat I, float x, float y){
    float ax, ay;
    float x0, y0;
    ax = std::modf(x, &x0);
    ay = std::modf(y, &y0);
    
    float I1, I2, I3, I4;
    I1 = I.at<float>(x0, y0);
    I2 = I.at<float>(x0+1, y0);
    I3 = I.at<float>(x0, y0+1);
    I4 = I.at<float>(x0+1, y0+1);
    
    float res = (1-ax)*(1-ay)*I1 + ax*(1-ay)*I2 + (1-ax)*ay*I3 + ax*ay*I4;
    return res;
}
cv::Mat invert(cv::Mat in){
    assert(in.rows == 2);
    assert(in.cols == 2);
    
    cv::Mat out = cv::Mat::zeros(in.rows, in.cols, in.type());
    
    float a = in.at<float>(0,0);
    float b = in.at<float>(0,1);
    float c = in.at<float>(1,0);
    float d = in.at<float>(1,1);
    
    float det = 1/(a*d - b*c);
    
    out.at<float>(0,0) = d * det;
    out.at<float>(0,1) = -b * det;
    out.at<float>(1,0) = -c * det;
    out.at<float>(1,1) = a * det;
    
    return out;
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
            out.at<float>(i/2, j/2) = in.at<float>(i, j);
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
    
    cv::Mat out = cv::Mat::zeros(num_rows, num_cols, in.type());
    
    for(int i = 0; i < num_rows; i++){
        for(int j = 0; j < num_cols; j++){
            for(int k = 0; k < filter_rows; k++){
                for(int l = 0; l < filter_cols; l++){
                    int row_idx = i + k - 1;
                    int col_idx = j + l - 1;
                    if(row_idx >= 0 && row_idx < num_rows){
                        if(col_idx >= 0 && col_idx < num_cols){
                            out.at<float>(i,j) += kernel.at<float>(k, l) * in.at<float>(row_idx, col_idx);
                        }
                    }
                }
            }
        }
    }
    return out;
}

cv::Mat gen_gaus_kernel(float sigma){
    cv::Mat out = cv::Mat::zeros(5, 5, CV_32FC1);
    int W = 5;
    double mean = W/2;
    double sum = 0.0; // For accumulating the kernel values
    for (int x = 0; x < W; ++x)
        for (int y = 0; y < W; ++y) {
            out.at<float>(x,y) = exp( -0.5 * (pow((x-mean)/sigma, 2.0) + pow((y-mean)/sigma,2.0)) )
                             / (2 * M_PI * sigma * sigma);

            // Accumulate the kernel values
            sum += out.at<float>(x,y);
        }

    // Normalize the kernel
    for (int x = 0; x < W; ++x)
        for (int y = 0; y < W; ++y)
            out.at<float>(x,y) /= sum;
    
    return out;
}

cv::Mat gaussian_blur(cv::Mat in){
    
    int num_rows = in.rows;
    int num_cols = in.cols;
    
    auto kernel = gen_gaus_kernel(1);
    
    int filter_rows = kernel.rows;
    int filter_cols = kernel.cols;
    
    cv::Mat out = cv::Mat::zeros(num_rows, num_cols, in.type());
    
    for(int i = 0; i < num_rows; i++){
        for(int j = 0; j < num_cols; j++){
            float sum = 0.0;
            for(int k = 0; k < filter_rows; k++){
                for(int l = 0; l < filter_cols; l++){
                    int row_idx = i + k - 1;
                    int col_idx = j + l - 1;
                    if(row_idx >= 0 && row_idx < num_rows){
                        if(col_idx >= 0 && col_idx < num_cols)
                        sum += kernel.at<float>(k, l) * in.at<float>(row_idx, col_idx);
                    }
                }
            }
            out.at<float>(i,j) = sum;
        }
    }

    return out;
}
