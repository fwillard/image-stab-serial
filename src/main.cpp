#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>

static void help(){
    std::cout << "usage: ImageStab video_file" << std::endl;
}
int main(int argc, char **argv) {
    // parser keys
    std::string keys =  "{help h usage ? |      | print this message     }"
                        "{@video         |      | video to be stabilized }";
    
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Serial Image Stabilization v1.0");
    
    // if parser contains help flag, display usage then exit
    if (parser.has("help")) {
        help();
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
        // error in opening the video
        std::cerr << "Unable to open file!" << std::endl;
        return 0;
    }

    
    std::vector<cv::Scalar> colors;
    cv::RNG rng;
    for (auto i = 0; i < 100; i++) {
        auto r = rng.uniform(0, 256);
        auto g = rng.uniform(0, 256);
        auto b = rng.uniform(0, 256);
        
        colors.push_back(cv::Scalar(r,g,b));
    }
    
    cv::Mat old_frame, old_gray;
    std::vector<cv::Point2f> p0, p1;

    // Read first frame and find corners in it
    capture >> old_frame;
    cvtColor(old_frame, old_gray, cv::COLOR_BGR2GRAY);
    goodFeaturesToTrack(old_gray, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);

    // Create a mask image for drawing purposes
    cv::Mat mask = cv::Mat::zeros(old_frame.size(), old_frame.type());
    
    bool save = false;
    int counter = 0;
    while(true){
        //read new frame
        cv::Mat frame, frame_gray;
        capture >> frame;
        
        if(frame.empty()){
            break;;
        }
        
        cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
        
        std::vector<uchar> status;
        std::vector<float> err;
        //define our termination criteria for the LK algorithm
        //here we are using iteration count + epsilon value
        //count = 10, eps = 0.03
        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.03);
        
        //calculate optical flow
        calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, cv::Size(15, 15), 2, criteria);
        
        std::vector<cv::Point2f> good_new;
        // Visualization part
        for(uint i = 0; i < p0.size(); i++)
        {
            // Select good points
            if(status[i] == 1) {
                good_new.push_back(p1[i]);
                // Draw the tracks
                line(mask,p1[i], p0[i], colors[i], 2);
                circle(frame, p1[i], 5, colors[i], -1);
            }
        }

        // Display the demo
        cv::Mat img;
        add(frame, mask, img);
        if (save) {
            std::string save_path = "./optical_flow_frames/frame_" + std::to_string(counter) + ".jpg";
            imwrite(save_path, img);
        }
        imshow("flow", img);
        int keyboard = cv::waitKey(25);
        if (keyboard == 'q' || keyboard == 27)
            break;

        // Update the previous frame and previous points
        old_gray = frame_gray.clone();
        p0 = good_new;
        counter++;
    }
    return 0;
}
