#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <functional>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

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

void applyMorph(cv::Mat &img, cv::Mat &imgOut, cv::MorphTypes type, const cv::Size &kernelSize, cv::MorphShapes morphShapes = cv::MORPH_ELLIPSE)
{
    //structuring element
    cv::Mat kernel = cv::getStructuringElement(morphShapes, kernelSize);
    //apply morph
    cv::Mat morphedImg;
    cv::morphologyEx(img, morphedImg, type, kernel);

    //return result
    imgOut = morphedImg;
}

void applyHSVSegmentation(cv::Mat &image, cv::Mat &imgOut, cv::Mat &maskOut,
                             cv::Scalar thresholdMAX, cv::Scalar thresholdMin,
                             cv::MorphTypes morphType = cv::MORPH_OPEN,
                             cv::Size kernelSize = cv::Size(0,0))
{
    //convert image to hsv
    cv::Mat imageHSV, imgThreshold1, imgThreshold2, mask;
    cv::cvtColor(image, imageHSV, cv::COLOR_BGR2HSV);

    //check if thresholdMAX's HUE is less then thresholdMIN's HUE and apply wraparound logic
    if (thresholdMin[0] > thresholdMAX[0])
    {
        //wrap around
        cv::inRange(imageHSV,
                    cv::Scalar(thresholdMin[0], thresholdMin[1], thresholdMin[2]),
                    cv::Scalar(180, thresholdMin[1], thresholdMin[2]),
                    imgThreshold1);

        cv::inRange(imageHSV,
                    cv::Scalar(0, 0, 0),
                    cv::Scalar(thresholdMAX[0], thresholdMAX[1], thresholdMAX[2]),
                    imgThreshold2);

        //combine thresholds
        mask = max(imgThreshold1, imgThreshold2);
    }
    else
    {
        //single inrange
        cv::inRange(imageHSV,
                    cv::Scalar(thresholdMin[0], thresholdMin[1], thresholdMin[2]),
                    cv::Scalar(thresholdMAX[0], thresholdMAX[1], thresholdMAX[2]),
                    mask);
    }

    //if morph type != nullptr and kernelsize != nullptr do the morph operation
    if (kernelSize != cv::Size(0,0))
    {
        applyMorph(mask, mask, morphType, kernelSize);
    }

    cv::Mat endImg;
    cv::bitwise_and(image, image, endImg, mask);

    imgOut = endImg;
    maskOut = mask;
}

cv::Mat findHSVSegmentation(cv::Mat &image)
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
        if (kernelSizeOpen > 0)
            applyMorph(mask, mask, cv::MORPH_OPEN, cv::Size(kernelSizeOpen, kernelSizeOpen));

        //apply closing
        if (kernelSizeClose > 0)
            applyMorph(mask, mask, cv::MORPH_CLOSE, cv::Size(kernelSizeClose, kernelSizeClose));

        //apply mask to original image
        cv::Mat endImg;
        cv::bitwise_and(image, image, endImg, mask);

        cv::imshow(windowName, mask);
        cv::imshow("Original Image", endImg);

        //update out
        outMask = mask;
    };
    cv::namedWindow("Original Image", CV_GUI_NORMAL);
    cv::namedWindow(windowName, CV_GUI_NORMAL);
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

void findBlobDetecting(cv::Mat &image)
{
    cv::Mat in_grey;
    cv::cvtColor(image, in_grey, CV_BGR2GRAY);

    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

    function<void()> f = [&]() {


    // Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 255;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 600;
    params.maxArea = 100000;

    //filter by color
    params.filterByColor = false;
    params.blobColor = 20;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.9;

    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.87;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.9;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // Detect blobs.
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(in_grey, keypoints);

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    cv::Mat img_with_keypoints;
    drawKeypoints(in_grey, keypoints, img_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // Show blobs
    showImage("keypoints", img_with_keypoints );
    };
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
    //findHSVSegmentation(src);

    //cv::Mat segmented, mask;
    //get resulting mask
    //applyHSVSegmentation(src, segmented, mask, cv::Scalar(109,255,255), cv::Scalar(110,255,255), cv::MORPH_CLOSE, cv::Size(10,10));

    //showImage("segmented", segmented);
    showImage("input image", src);




    //end wait
    while (cv::waitKey() != 27)
        ;

    return 0;
}