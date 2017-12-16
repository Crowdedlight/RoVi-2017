//
// Created by frederik on 12/16/17.
//

#include "Vision.h"

Vision::Vision() {}

vector<cv::Point2f> Vision::getPointsFromMaker1(cv::Mat &image) {

    cv::Mat segmented, mask;
    //get resulting mask
    applyHSVSegmentation(image, segmented, mask, cv::Scalar(109,255,255), cv::Scalar(110,255,255), cv::MORPH_CLOSE, cv::Size(10,10));

    vector<cv::KeyPoint> keypoints;
    cv::Mat imageKeypoints; //9630 - 12658
    applyBlobDetecting(mask, keypoints, imageKeypoints, 0, 81, 2500, 15000, 0, 0.8  , 0);

    //make keypoints to a vector for mid points
    vector<cv::Point2f> trackedPoints;

    for(const auto& p : keypoints)
        trackedPoints.push_back(p.pt);

    return trackedPoints;
}

vector<cv::Point2f> Vision::getPointsFromMaker1(cv::Mat &image, cv::Mat &imgOut) {
    cv::Mat segmented, mask;
    //get resulting mask
    applyHSVSegmentation(image, segmented, mask, cv::Scalar(109,255,255), cv::Scalar(110,255,255), cv::MORPH_CLOSE, cv::Size(10,10));

    vector<cv::KeyPoint> keypoints;
    cv::Mat imageKeypoints; //9630 - 12658
    applyBlobDetecting(mask, keypoints, imageKeypoints, 0, 81, 2500, 15000, 0, 0.8  , 0);

    //make keypoints to a vector for mid points
    vector<cv::Point2f> trackedPoints;

    for(const auto& p : keypoints)
        trackedPoints.push_back(p.pt);

    //draw keypoints circles
    drawKeypoints(image, keypoints, imageKeypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    //draw center points
    for(const auto& p : trackedPoints)
        cv::drawMarker(imageKeypoints, p, cv::Scalar(0,255,0));

    //save in outimage
    imgOut = imageKeypoints;

    return trackedPoints;
}

void Vision::applyHSVSegmentation(cv::Mat &image, cv::Mat &imgOut, cv::Mat &maskOut, cv::Scalar thresholdMAX,
                                  cv::Scalar thresholdMin, cv::MorphTypes morphType, cv::Size kernelSize) {
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

void Vision::applyMorph(cv::Mat &img, cv::Mat &imgOut, cv::MorphTypes type, const cv::Size &kernelSize,
                        cv::MorphShapes morphShapes) {
    //structuring element
    cv::Mat kernel = cv::getStructuringElement(morphShapes, kernelSize);
    //apply morph
    cv::Mat morphedImg;
    cv::morphologyEx(img, morphedImg, type, kernel);

    //return result
    imgOut = morphedImg;
}

void Vision::applyBlobDetecting(cv::Mat &image, vector<cv::KeyPoint> &keypoints, cv::Mat &imageKeypoints, int minThres,
                                int maxThres, int minArea, int maxArea, int blobColor, float minCircular,
                                float minInertia) {
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
//    drawKeypoints(in_grey, keypoints, imageKeypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
}
