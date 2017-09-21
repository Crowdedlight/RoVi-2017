/*
 * RoVi1
 * Example application to load and display an image
 */

// v1.0-4-gdfe246a

#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
    // Parse command line arguments
    cv::CommandLineParser parser(argc, argv,
        "{help   |            | print this message}"
        "{@image | ./book.jpg | image path}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Load image
    std::string filename = parser.get<std::string>("@image");
    cv::Mat img = cv::imread(filename);

    if (img.empty()) {
        std::cout << "Input image not found at '" << filename << "'\n";
        return 1;
    }

    // make header with reactangle part
    cv::Mat rect (img, cv::Rect(350, 100, 90, 120));
    rect.setTo(cv::Scalar(0,0,0));

    cv::imshow("Image", img);
    cv::waitKey();

    return 0;
}
