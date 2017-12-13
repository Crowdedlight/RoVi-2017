#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <functional>
#include <opencv2/calib3d.hpp>
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
    int minThres = 0, maxThres = 255, minArea = 0, maxArea = 50000, blobColor = 0, minCircular = 0, minInertia = 0;
    const string windowName = "Blob Detector";

    cv::Mat in_grey;
    cv::cvtColor(image, in_grey, CV_BGR2GRAY);

    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

    function<void()> f = [&]() {
        // Change thresholds
        params.minThreshold = minThres;
        params.maxThreshold = maxThres;

        // Filter by Area.
        params.filterByArea = true;
        params.minArea = minArea;
        params.maxArea = maxArea;

        //filter by color
        params.filterByColor = true;
        params.blobColor = blobColor;

        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = float(minCircular/100);

        // Filter by Convexity
        params.filterByConvexity = false;
        params.minConvexity = 0.87;

        // Filter by Inertia
        params.filterByInertia = true;
        params.minInertiaRatio = float(minInertia/100);

        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

        // Detect blobs.
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(in_grey, keypoints);

        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        cv::Mat img_with_keypoints;
        drawKeypoints(in_grey, keypoints, img_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        // Show blobs
        showImage(windowName, img_with_keypoints );
    };
    cv::namedWindow(windowName, CV_GUI_NORMAL);

    cv::createTrackbar("Min Threshold", windowName, &minThres, 255, on_trackbar, &f);
    cv::createTrackbar("Max Threshold", windowName, &maxThres, 255, on_trackbar, &f);
    cv::createTrackbar("Min Area", windowName, &minArea, 50000, on_trackbar, &f);
    cv::createTrackbar("Max Area", windowName, &maxArea, 50000, on_trackbar, &f);
    cv::createTrackbar("Blob Color", windowName, &blobColor, 255, on_trackbar, &f);
    cv::createTrackbar("Min Circularity", windowName, &minCircular, 100, on_trackbar, &f);
    cv::createTrackbar("Min Inertia", windowName, &minInertia, 100, on_trackbar, &f);
    f();

    while (cv::waitKey() != 27)
        ;

    cv::destroyWindow(windowName);
}

void applyBlobDetecting(cv::Mat &image, vector<cv::KeyPoint>& keypoints, cv::Mat &imageKeypoints,
                        int minThres = 0, int maxThres = 255,
                        int minArea = 0, int maxArea = 100000,
                        int blobColor = 0, float minCircular = 0, float minInertia = 0)
{
    cv::Mat in_grey = image;
    //cv::cvtColor(image, in_grey, CV_BGR2GRAY);

    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = minThres;
    params.maxThreshold = maxThres;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = minArea;
    params.maxArea = maxArea;

    //filter by color
    params.filterByColor = true;
    params.blobColor = blobColor;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = minCircular;
    params.maxCircularity = 1;

    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.87;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = minInertia;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // Detect blobs.
    detector->detect(in_grey, keypoints);

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    drawKeypoints(in_grey, keypoints, imageKeypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
}

int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(argc, argv,
                                 "{help   |             | print this message}"
                                 "{@image | ../marker_color_hard/marker_color_hard_%02d.png  | image path}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    cv::VideoCapture sequence(parser.get<std::string>("@image"));

    //todo debug
//    cv::Mat img = cv::imread("../marker_color_hard/marker_color_hard_01.png", cv::IMREAD_COLOR);
//    findBlobDetecting(img);


    //Is sequence open?
    if (!sequence.isOpened())
    {
        cerr << "Failed to open Image Sequence!\n" << endl;
        return 1;
    }

    //loop sequence
    cv::Mat current_image;
    int counter = 0;
    while(true)
    {
        sequence >> current_image;
        //end of sequence?
        if (current_image.empty()) {
            std::cout << "End of Sequence." << endl;
            return 0;
        }

        //do threatment
        //color segmentation - Find the best values to track the red and green circle
        //findHSVSegmentation(src);

        //time how long processing takes
        long beforeCount = cv::getTickCount();

        cv::Mat segmented, mask;
        //get resulting mask
        applyHSVSegmentation(current_image, segmented, mask, cv::Scalar(109,255,255), cv::Scalar(110,255,255), cv::MORPH_CLOSE, cv::Size(10,10));

        vector<cv::KeyPoint> keypoints;
        cv::Mat imageKeypoints; //9630 - 12658
        applyBlobDetecting(mask, keypoints, imageKeypoints, 0, 81, 2500, 15000, 0, 0.8  , 0);

        //after processing, before showing
        long afterCount = cv::getTickCount();
        double time = (afterCount - beforeCount)/ cv::getTickFrequency();

        showImage("keypoints", imageKeypoints);

        //get points of the three circles
        cout << ++counter << ". Processing Time: " << time <<  " sec" << endl;
        for(const auto& point : keypoints)
        {
            cout << "point:" << point.pt << ", size:" << point.size << endl;
            cv::drawMarker(current_image, point.pt, cv::Scalar(0,255,0));
        }

        showImage("Tracking Points", current_image);

        //end wait 27 == esc, 32 == spacebar
        while (cv::waitKey() != 32)
            ;
    }
}