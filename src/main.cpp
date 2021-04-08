#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>

#define TRACK_POINTS 100
#define QUALITY_LEVEL 0.3
#define MIN_DISTANCE 7

struct TransformParam{
    TransformParam(){}
    TransformParam(double _dx, double _dy, double _dtheta){
        dx = _dx;
        dy = _dy;
        dtheta = _dtheta;
    }
    
    double dx;
    double dy;
    double dtheta;
    
    // generates the following transform matrix
    // +-------------+------------+----+
    // | cos(theta) | -sin(theta) | dx |
    // | sin(theta) |  cos(theta) | dy |
    // +--------------------------+----+
    // see section 3 of deng et al. for more info
    void getTransformMatrix(cv::Mat &T){
        T.at<double>(0,0) = *cos(dtheta);
        T.at<double>(0,1) = s*sin(dtheta);
        T.at<double>(0,2) = dx;
        
        T.at<double>(1,0) = sin(dtheta);
        T.at<double>(1,1) = cos(dtheta);
        T.at<double>(1,2) = dy;
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
    
    for(int i = 1; i < frame_count - 1; i++){
        std::vector<cv::Point> old_points, new_points;
        
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
        
        
        
    }

    return 0;
}