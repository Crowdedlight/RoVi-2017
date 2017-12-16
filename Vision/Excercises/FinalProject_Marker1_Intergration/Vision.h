//
// Created by frederik on 12/16/17.
//

#ifndef FINALPROJECT_MARKER1_VISION_H
#define FINALPROJECT_MARKER1_VISION_H

#include <functional>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

class Vision {

public:
    Vision();

    //Only get points out with no visual drawing. Is faster without drawing
    vector<cv::Point2f> getPointsFromMaker1(cv::Mat& image);
    //Also draws on the image given what keypoints it detects and what points it track
    vector<cv::Point2f> getPointsFromMaker1(cv::Mat& image, cv::Mat &imgOut);


private:
    void applyHSVSegmentation(cv::Mat &image, cv::Mat &imgOut, cv::Mat &maskOut,
                              cv::Scalar thresholdMAX, cv::Scalar thresholdMin,
                              cv::MorphTypes morphType = cv::MORPH_OPEN,
                              cv::Size kernelSize = cv::Size(0,0));

    void applyMorph(cv::Mat &img, cv::Mat &imgOut, cv::MorphTypes type, const cv::Size &kernelSize, cv::MorphShapes morphShapes = cv::MORPH_ELLIPSE);
    void applyBlobDetecting(cv::Mat &image, vector<cv::KeyPoint>& keypoints, cv::Mat &imageKeypoints,
                            int minThres = 0, int maxThres = 255,
                            int minArea = 0, int maxArea = 100000,
                            int blobColor = 0, float minCircular = 0, float minInertia = 0);
};


#endif //FINALPROJECT_MARKER1_VISION_H
