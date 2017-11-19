#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <functional>
#include <iostream>

using namespace std;

// Calls the std::function<void()> passed as 'userdata'
void on_trackbar(int, void* userdata)
{
    (*static_cast<std::function<void()>*>(userdata))();
}

void applyErosion(cv::Mat &img, cv::Mat &outimg, int kernelSize)
{
    //if kernelsize = 0, do not apply
    if (kernelSize <= 0)
        return;

    //make structering element
    cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
    //apply erosion
    cv::erode(img, outimg, element_erode);
}

void applyDilation(cv::Mat &img, cv::Mat &outimg, int kernelSize)
{
    //if kernelsize = 0, do not apply
    if (kernelSize <= 0)
        return;

    //make structering element
    cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
    //apply dilation
    cv::dilate(img, outimg, element_dilate);
}

void applyOpening(cv::Mat &img, cv::Mat &outImg, int kernelSize)
{
    //first erosion then dilate
    applyErosion(img, outImg, kernelSize);
    applyDilation(img, outImg, kernelSize);
}

void applyClosing(cv::Mat &img, cv::Mat &outImg, int kernelSize)
{
    //first dilate the erode
    applyDilation(img, outImg, kernelSize);
    applyErosion(img, outImg, kernelSize);
}

cv::Mat HSVsegmentation(cv::Mat &image)
{
    int H_min = 0, H_max = 180, V = 255, S = 255, kernelSizeOpen = 0, kernelSizeClose = 0;
    const string windowName = "Color Segmentation";
    cv::Mat outMask;

    function<void()> f = [&]() {
        cv::Mat imageHSV, imgThreshold1, imgThreshold2, mask;
        cv::cvtColor(image, imageHSV, cv::COLOR_BGR2HSV);

        //check for wrap around
        if (H_min > H_max) {
            //get thresholds
            cv::inRange(imageHSV,
                        cv::Scalar(H_min, 0, 0),
                        cv::Scalar(180, S, V),
                        imgThreshold1);

            cv::inRange(imageHSV,
                        cv::Scalar(0, 0, 0),
                        cv::Scalar(H_max, S, V),
                        imgThreshold2);

            //combine thresholds
            mask = max(imgThreshold1, imgThreshold2);
        } else
        {
            //get thresholds
            cv::inRange(imageHSV,
                        cv::Scalar(H_min, 0, 0),
                        cv::Scalar(H_max, S, V),
                        mask);
        }

        //apply opening
        applyOpening(mask, mask, kernelSizeOpen);

        //apply closing
        applyClosing(mask, mask, kernelSizeClose);

        //apply mask to original image
        cv::Mat endImg;
        cv::bitwise_and(image, image, endImg, mask);

        cv::imshow(windowName, mask);
        cv::imshow("Original Image", endImg);

        //update out
        outMask = mask;
    };
    cv::namedWindow("Original Image");
    cv::namedWindow(windowName);
    cv::createTrackbar("Hue_min", windowName, &H_min, 180, on_trackbar, &f);
    cv::createTrackbar("Hue_max", windowName, &H_max, 180, on_trackbar, &f);
    cv::createTrackbar("Saturation", windowName, &S, 255, on_trackbar, &f);
    cv::createTrackbar("Value", windowName, &V, 255, on_trackbar, &f);
    cv::createTrackbar("Opening", windowName, &kernelSizeOpen, 50, on_trackbar, &f);
    cv::createTrackbar("Closing", windowName, &kernelSizeClose, 50, on_trackbar, &f);
    f();

    while (cv::waitKey() != 27)
        ;

    cv::destroyWindow(windowName);
    cv::destroyWindow("Original Image");

    return outMask;
}

int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(argc, argv,
                                 "{help   |             | print this message}"
                                 "{@image | ../marker_color/marker_color_01.png  | image path}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Load image
    std::string filename = parser.get<std::string>("@image");
    cv::Mat src = cv::imread(filename, cv::IMREAD_COLOR);

    if (src.empty()) {
        std::cout << "Input image not found at '" << filename << "'\n";
        return 1;
    }

    //color segmentation - Find the best values to track the red and green circle
    cv::Mat segmented = HSVsegmentation(src);

    return 0;
}