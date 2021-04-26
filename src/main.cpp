#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

#include "util.hpp"
#include "tracking.hpp"
#include <chrono>

using namespace cv;
using namespace std;
int main(int argc, char **argv)
{
    const string about =
        "This sample demonstrates Lucas-Kanade Optical Flow calculation.\n"
        "The example file can be downloaded from:\n"
        "  https://www.bogotobogo.com/python/OpenCV_Python/images/mean_shift_tracking/slow_traffic_small.mp4";
    const string keys =
        "{ h help |      | print this help message }"
        "{ @image | vtest.avi | path to image file }";
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    string filename = samples::findFile(parser.get<string>("@image"));
    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }
    VideoCapture capture(filename);
    if (!capture.isOpened()){
        //error in opening the video input
        cerr << "Unable to open file!" << endl;
        return 0;
    }
    // Create some random colors
    vector<Scalar> colors;
    RNG rng;
    for(int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(Scalar(r,g,b));
    }
    Mat old_frame, old_gray;
    vector<Point2f> p0, p1;
    // Take first frame and find corners in it
    capture >> old_frame;
    cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    goodFeaturesToTrack(old_gray, p0, 1, 0.3, 7, Mat(), 7, false, 0.04);
    // Create a mask image for drawing purposes
    Mat mask = Mat::zeros(old_frame.size(), old_frame.type());
    while(true){
        Mat frame, frame_gray;
        capture >> frame;
        if (frame.empty())
            break;
        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
        // calculate optical flow
        vector<uchar> status;
        vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
//        calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(15,15), 2, criteria);
        
        auto start = std::chrono::high_resolution_clock::now();
        pyr_LK(old_gray, frame_gray, p0, p1);
        auto stop = std::chrono::high_resolution_clock::now();
    
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout <<"Exec time (ms): " << diff.count() << std::endl;
        vector<Point2f> good_new;
        for(uint i = 0; i < p0.size(); i++)
        {
            // Select good points
            if(status[i] == 1) {
                good_new.push_back(p1[i]);
                // draw the tracks
                line(mask,p1[i], p0[i], colors[i], 2);
                circle(frame, p1[i], 5, colors[i], -1);
            }
        }
        Mat img;
        add(frame, mask, img);
        imshow("Frame", img);
        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;
        // Now update the previous frame and previous points
        old_gray = frame_gray.clone();
        p0 = good_new;
    }
}

//#include <iostream>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/video.hpp>
//#include <opencv2/videoio.hpp>
//#include <opencv2/calib3d.hpp>
//#include <chrono>
//
//#include "util.hpp"
//#include "tracking.hpp"
//
//#define TRACK_POINTS 100
//#define QUALITY_LEVEL 0.3
//#define MIN_DISTANCE 7
//typedef enum AlgoType{
//    PYRLK      = 0,
//    SIFT       = 1,
//    PYRLK_CUDA = 2,
//    SIFT_CUDA  = 3
//} AlgoType;
//
//struct TransformParam{
//    TransformParam(){}
//
//    // given the following affine2d transform matrix form
//    // +-------+-------+--------+
//    // |   a   |   b   |   dx   |
//    // |   c   |   d   |   dy   |
//    // +-------+-------+--------+
//    // this constructor decomposes the matrix into the components
//    // dx, dy, s_x, s_y, and dtheta
//    TransformParam(cv::Mat T){
//        //ensure matrix is correct size
//        assert(T.rows == 2);
//        assert(T.cols == 3);
//
//
//        dx = T.at<double>(0,2);
//        dy = T.at<double>(1,2);
//
//        double a = T.at<double>(0,0);
//        double b = T.at<double>(0,1);
//        double c = T.at<double>(1,0);
//        double d = T.at<double>(1,1);
//
//        s_x = sgn(a) * sqrt(pow(a, 2) + pow(b, 2));
//        s_y = sgn(d) * sqrt(pow(c, 2) + pow(d, 2));
//
//        dtheta = atan2(-b, a);
//    }
//
//    double dx;
//    double dy;
//    double dtheta;
//    double s_x;
//    double s_y;
//
//    // returns the following transform matrix
//    // +----------------+-----------------+----+
//    // | s_x*cos(theta) | -s_x*sin(theta) | dx |
//    // | s_y*sin(theta) |  s_y*cos(theta) | dy |
//    // |    0           |     0           | 1  |
//    // +----------------+-----------------+----+
//    cv::Mat getTransformMatrix(){
//        cv::Mat T = cv::Mat::zeros(3, 3, CV_64F);
//        T.at<double>(0,0) = s_x*cos(dtheta);
//        T.at<double>(0,1) = -s_x*sin(dtheta);
//        T.at<double>(0,2) = dx;
//
//        T.at<double>(1,0) = s_y*sin(dtheta);
//        T.at<double>(1,1) = s_y*cos(dtheta);
//        T.at<double>(1,2) = dy;
//
//        T.at<double>(2,0) = 0.0;
//        T.at<double>(2,1) = 0.0;
//        T.at<double>(2,2) = 1.0;
//        return T;
//    }
//};
//
//static void optical_flow(cv::VideoCapture &capture, int frame_count) {
//    cv::Mat old_frame, old_gray;
//    cv::Mat frame, frame_gray;
//
//    // Read first frame and convert to grayscale
//    capture >> old_frame;
//    cvtColor(old_frame, old_gray, cv::COLOR_BGR2GRAY);
//
////    auto blurred = construct_gaussian_pyramid(old_gray);
////
////    for(auto img: blurred){
////        cv::imshow("", img);
////        cv::waitKey(0);
////    }
////
//    std::vector<TransformParam> transforms;
//
//    cv::Mat last_T;
//
//    for(int i = 1; i < frame_count - 1; i++){
//        std::vector<cv::Point2f> old_points, new_points;
//
//        //get tracking points from old frame;
//        goodFeaturesToTrack(old_gray, old_points, TRACK_POINTS, QUALITY_LEVEL, MIN_DISTANCE);
//
//        //read next frame, break if failure
//        if(!capture.read(frame)){
//            break;
//        }
//
//        //convert to grayscale
//        cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
//
//        //calculate optical flow
//        std::vector<unsigned char> status;
//        std::vector<float> err;
////        cv::calcOpticalFlowPyrLK(old_gray, frame_gray, old_points, new_points, status, err);
//        pyr_LK(old_gray, frame_gray, old_points, new_points);
//
//
//        //eliminate invalid points
//        auto old_it = old_points.begin();
//        auto new_it = new_points.begin();
//        for(auto j = 0; j < status.size(); j++){
//            if(status[j] == 1){
//                old_it++;
//                new_it++;
//            }
//            else{
//                old_it = old_points.erase(old_it);
//                new_it = new_points.erase(new_it);
//            }
//        }
//
//        //estimate affine transformation
//        cv::Mat T = cv::estimateAffine2D(old_points, new_points);
//
//        transforms.push_back(TransformParam(T));
//
//    }
//}
//
//static void sift(cv::VideoCapture &capture, int frame_count) {
//    cv::Mat old_frame, old_gray;
//    cv::Mat frame, frame_gray;
//
//    // Read first frame and convert to grayscale
//    capture >> old_frame;
//    cvtColor(old_frame, old_gray, cv::COLOR_BGR2GRAY);
//
//    std::vector<TransformParam> transforms;
//
//    cv::Mat last_T;
//    //create sift keypoint detector
//    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
//    //create brute force matcher
//    cv::BFMatcher matcher = cv::BFMatcher();
//
//    for(int i = 1; i < frame_count - 1; i++){
//        std::vector<cv::KeyPoint> old_points, new_points;
//        cv::Mat old_descriptors, new_descriptors;
//
//        //read next frame, break if failure
//        if(!capture.read(frame)){
//            break;
//        }
//
//        //convert to grayscale
//        cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
//
//        //detect keypoints
//        sift->detectAndCompute(old_gray, cv::Mat(), old_points, old_descriptors);
//        sift->detectAndCompute(frame_gray, cv::Mat(), new_points, new_descriptors);
//
//
//        //match points
//        std::vector<cv::DMatch> matches;
//        matcher.match(old_descriptors, new_descriptors, matches);
//
//        //sort by distance
//        std::sort(matches.begin(), matches.end());
//
//        std::vector<cv::Point2f> p0, p1;
//
//        //get the best N points for affine estimation
//        for(int j = 0; j < TRACK_POINTS; j++){
//            p0.push_back(old_points[matches[j].queryIdx].pt);
//            p1.push_back(new_points[matches[j].trainIdx].pt);
//        }
//
//        //estimate affine transformation
//        cv::Mat T = cv::estimateAffine2D(p0, p1);
//
//        transforms.push_back(TransformParam(T));
//
//    }
//}
//
//int main(int argc, char **argv) {
//    // parser keys
//    std::string keys =  "{help h usage ? |      | print this message                    }"
//                        "{@input         |      | input video to be stabilized          }"
//                        "{@output        |      | output video file                     }"
//                        "{jitter j       |      | apply jitter to video                 }"
//                        "{algorithm a    |  0   | the algorithm to use for stabilization\n"
//                                         "          0 for Serial Lucas-Kannade Optical Flow (default)\n"
//                                         "          1 for Serial SIFT\n"
//                                         "          2 for CUDA Lucas-Kannade Optical Flow\n"
//                                         "          3 for CUDA SIFT}";
//
//    cv::CommandLineParser parser(argc, argv, keys);
//    parser.about("Serial Image Stabilization v1.0");
//
//    // if parser contains help flag, display usage then exit
//    if (parser.has("help")) {
//        parser.printMessage();
//        return -1;
//    }
//
//    // get the input video file name from the parser
//    std::string input_file = cv::samples::findFile(parser.get<std::string>("@input"));
//    if (!parser.check()) {
//        parser.printErrors();
//        return -1;
//    }
//
//    // get the output video file name from the parser
//    std::string output_file = cv::samples::findFile(parser.get<std::string>("@output"));
//    if (!parser.check()) {
//        parser.printErrors();
//        return -1;
//    }
//
//    //get the stabilization algorithm
//    const int cl_arg = parser.get<int>("algorithm");
//    if (!parser.check()) {
//        parser.printErrors();
//        return -1;
//    }
//    AlgoType algo;
//    switch (cl_arg) {
//        case 0:
//            algo = PYRLK;
//            break;
//        case 1:
//            algo = SIFT;
//            break;
//        case 2:
//            algo = PYRLK_CUDA;
//            break;
//        case 3:
//            algo = SIFT_CUDA;
//            break;
//        default:
//            std::cerr << "Invalid value for algorithm flag" << std::endl;
//            return -1;
//    }
//
//    //open input file
//    cv::VideoCapture capture(input_file);
//    if (!capture.isOpened()) {
//        // error in opening the video
//        std::cerr << "Unable to open input file!" << std::endl;
//        return -1;
//    }
//
//    //get input video properties
//    int frame_count = int(capture.get(cv::CAP_PROP_FRAME_COUNT));
//    int width = int(capture.get(cv::CAP_PROP_FRAME_WIDTH));
//    int height = int(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
//    double fps = capture.get(cv::CAP_PROP_FPS);
//    int fourcc = int(capture.get(cv::CAP_PROP_FOURCC));
//
//    //set up output writer
//    cv::VideoWriter out(output_file, fourcc, fps, cv::Size(2 * width, height));
//
//    auto start = std::chrono::high_resolution_clock::now();
//    optical_flow(capture, frame_count);
//    auto stop = std::chrono::high_resolution_clock::now();
//
//    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
//    std::cout <<"Exec time (ms): " << diff.count() << std::endl;
//
//    return 0;
//}
