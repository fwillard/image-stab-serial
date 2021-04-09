#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>

#include "util.hpp"

#define TRACK_POINTS 100
#define QUALITY_LEVEL 0.3
#define MIN_DISTANCE 7


struct TransformParam{
    TransformParam(){}
    
    // given the following affine2d transform matrix form
    // +-------+-------+--------+
    // |   a   |   b   |   dx   |
    // |   c   |   d   |   dy   |
    // +-------+-------+--------+
    // this constructor decomposes the matrix into the components
    // dx, dy, s_x, s_y, and dtheta
    TransformParam(cv::Mat T){
        //ensure matrix is correct size
        assert(T.rows == 2);
        assert(T.cols == 3);
        
        
        dx = T.at<double>(0,2);
        dy = T.at<double>(1,2);
        
        double a = T.at<double>(0,0);
        double b = T.at<double>(0,1);
        double c = T.at<double>(1,0);
        double d = T.at<double>(1,1);
        
        s_x = sgn(a) * sqrt(pow(a, 2) + pow(b, 2));
        s_y = sgn(d) * sqrt(pow(c, 2) + pow(d, 2));
        
        dtheta = atan2(-b, a);
    }
    
    double dx;
    double dy;
    double dtheta;
    double s_x;
    double s_y;
    
    // returns the following transform matrix
    // +----------------+-----------------+----+
    // | s_x*cos(theta) | -s_x*sin(theta) | dx |
    // | s_y*sin(theta) |  s_y*cos(theta) | dy |
    // |    0           |     0           | 1  |
    // +----------------+-----------------+----+
    cv::Mat getTransformMatrix(){
        cv::Mat T = cv::Mat::zeros(3, 3, CV_64F);
        T.at<double>(0,0) = s_x*cos(dtheta);
        T.at<double>(0,1) = -s_x*sin(dtheta);
        T.at<double>(0,2) = dx;
        
        T.at<double>(1,0) = s_y*sin(dtheta);
        T.at<double>(1,1) = s_y*cos(dtheta);
        T.at<double>(1,2) = dy;
        
        T.at<double>(2,0) = 0.0;
        T.at<double>(2,1) = 0.0;
        T.at<double>(2,2) = 1.0;
        return T;
    }
};

int main(int argc, char **argv) {
    // parser keys
    std::string keys =  "{help h usage ? |      | print this message           }"
                        "{@input         |      | input video to be stabilized }"
                        "{@output        |      | output video file            }"
                        "{jitter j       |      | apply jitter to video        }";
    
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Serial Image Stabilization v1.0");
    
    // if parser contains help flag, display usage then exit
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    
    // get the input video file name from the parser
    std::string input_file = cv::samples::findFile(parser.get<std::string>("@input"));
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }
    
    // get the output video file name from the parser
    std::string output_file = cv::samples::findFile(parser.get<std::string>("@output"));
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }
    
    //open input file
    cv::VideoCapture capture(input_file);
    if (!capture.isOpened()) {
        // error in opening the video
        std::cerr << "Unable to open input file!" << std::endl;
        return 0;
    }
    
    //get input video properties
    int frame_count = int(capture.get(cv::CAP_PROP_FRAME_COUNT));
    int width = int(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = int(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = capture.get(cv::CAP_PROP_FPS);
    int fourcc = int(capture.get(cv::CAP_PROP_FOURCC));
    
    //set up output writer
    cv::VideoWriter out(output_file, fourcc, fps, cv::Size(2 * width, height));
    
    cv::Mat old_frame, old_gray;
    cv::Mat frame, frame_gray;
    
    // Read first frame and convert to grayscale
    capture >> old_frame;
    cvtColor(old_frame, old_gray, cv::COLOR_BGR2GRAY);
    
    
    
    //vector to hold the transformations
    std::vector<TransformParam> transforms;
    
    cv::Mat last_T;
    
    for(int i = 1; i < frame_count - 1; i++){
        std::vector<cv::Point2f> old_points, new_points;
        
        //get tracking points from old frame;
        goodFeaturesToTrack(old_gray, old_points, TRACK_POINTS, QUALITY_LEVEL, MIN_DISTANCE);
        
        //read next frame, break if failure
        if(!capture.read(frame)){
            break;
        }
        
        //convert to grayscale
        cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
        
        //calculate optical flow
        std::vector<unsigned char> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(old_gray, frame_gray, old_points, new_points, status, err);
        
        //eliminate invalid points
        auto old_it = old_points.begin();
        auto new_it = new_points.begin();
        for(auto j = 0; j < status.size(); j++){
            if(status[j] == 1){
                old_it++;
                new_it++;
            }
            else{
                old_it = old_points.erase(old_it);
                new_it = new_points.erase(new_it);
            }
        }
        
        //estimate affine transformation
        cv::Mat T = cv::estimateAffine2D(old_points, new_points);
        
        transforms.push_back(TransformParam(T));
        
        std::cout << "Frame: " << i << "/" << frame_count << " -  Tracked points : " << old_points.size() << std::endl;
    }

    return 0;
}
