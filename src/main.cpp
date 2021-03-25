#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>

int main(int argc, char **argv) {
    // parser keys
    std::string keys =  "{help h usage ? |      | print this message     }"
                        "{@video         |      | video to be stabilized }";
    
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Serial Image Stabilization v1.0");
    
    // if parser contains help flag, display usage then exit
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    
    // get the video file name from the parser
    std::string filename = cv::samples::findFile(parser.get<std::string>("@video"));
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }
    
    cv::VideoCapture capture(filename);
    if (!capture.isOpened()) {
        // error in opening the video input
        std::cerr << "Unable to open file!" << std::endl;
        return 0;
    }
    
    return 0;
}
