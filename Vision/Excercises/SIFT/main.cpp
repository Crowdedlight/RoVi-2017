#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <functional>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


using namespace std;
using namespace cv;

// Calls the std::function<void()> passed as 'userdata'
void on_trackbar(int, void* userdata)
{
    (*static_cast<std::function<void()>*>(userdata))();
}

void showImage(string windowName, cv::Mat &image)
{
    cv::namedWindow(windowName, CV_WINDOW_NORMAL);
    cv::imshow(windowName, image);
}

int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(argc, argv,
                                 "{help   |             | print this message}"
                                 "{@image | ../book_cover.jpg  | image path}"
                                 "{@image2| ../book_scene.jpg  | image2 path"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Load image
    std::string filename = parser.get<std::string>("@image");
    std::string filename2 = parser.get<std::string>("@image2");

    cv::Mat src = cv::imread(filename, cv::IMREAD_COLOR);
    cv::Mat src2 = cv::imread(filename2, cv::IMREAD_COLOR);

    if (src.empty() || src2.empty()) {
        std::cout << "Input image not found at '" << filename << "'\n";
        return 1;
    }


    //detect keypoints with SIFT
    Ptr<


    //end wait
    while (cv::waitKey() != 27)
        ;

    return 0;
}