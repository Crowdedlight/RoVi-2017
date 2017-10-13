/*
 * RoVi1
 * Exercise 3: The Frequency domain and filtering
 */

// e17-1-gbe848aa

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/shape.hpp>

#include <iostream>

using namespace cv;
using namespace std;

//settings for trackbar
int threshold_value = 97;
int threshold_type = 1;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

Mat blurred, thresImg;
string window_name_threshold = "Threshold";

string trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
string trackbar_value = "Value";

//contours found
vector<vector<Point> > contours;

//Coins sum
int countSum = 0;

void showContours ()
{
    vector<Vec4i> hierarchy;
    /// Find contours
    findContours(thresImg, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Draw contours
    Mat drawing = Mat::zeros( thresImg.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        if (hierarchy[i][3] != -1)
            continue;

        Scalar color = Scalar( 255,0,0 );
        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );

        //moments for contour
//        Moments mo = moments(contours[i]);
//        double cx = mo.m10/mo.m00;
//        double cy = mo.m01/mo.m00;

        //add center text
        Point2f textCenter;
        float tempRadius;
        minEnclosingCircle(contours[i], textCenter, tempRadius);

        // draw the minEnclosingCircle
        circle(drawing, textCenter, tempRadius, Scalar(0,0,255), 2);

        //print radius on circle
        cout << "num: " << i << ", " << tempRadius << endl;

        //change value based on size. Issue with two 10's that is marked as 15, as their radius is 93 & 96
        string coinText = "";
        if (tempRadius >= 124.6) {
            coinText = "50";
            countSum += 50;
        }
        else if (tempRadius>=121) {
            coinText = "5";
            countSum += 5;
        }
        else if (tempRadius>=107) {
            coinText = "20";
            countSum += 20;
        }
        else if (tempRadius>=91) {
            coinText = "15";
            countSum += 15;
        }
        else if (tempRadius>=80) {
            coinText = "10";
            countSum += 10;
        }
        else
            coinText = "??";

        //add index to text
        coinText += "(" + to_string(i) + ")";

        int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontScale = 2;
        int thickness = 3;

        int baseline=0;
        Size textSize = getTextSize(coinText, fontFace, fontScale, thickness, &baseline);
        baseline += thickness;

        // center the text
        Point textOrg((textCenter.x - (textSize.width/2)),
                      (textCenter.y + (textSize.height/2)));

        // then put the text in
        putText(drawing, coinText, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
    }

    /// Show in a window
    namedWindow( "Contours", CV_WINDOW_NORMAL );
    imshow( "Contours", drawing );
}

void on_trackbar(int, void*)
{
    threshold(blurred, thresImg, threshold_value, max_BINARY_value, threshold_type);
    imshow(window_name_threshold, thresImg);
    showContours();
}

int main(int argc, char* argv[])
{
    // Load image as grayscale
    std::string filename = "../coins.jpg";
    cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);

    if (img.empty()) {
        std::cout << "Input image not found at '" << filename << "'\n";
        return 1;
    }

    //blur it
    Mat filter;
    blur(img, filter, Size(20,20));
    bilateralFilter(filter, blurred, 5, 60,60);


    // Create Trackbar to choose type and parameter of Threshold
    namedWindow(window_name_threshold, WINDOW_NORMAL);
    createTrackbar( trackbar_type,
                    window_name_threshold, &threshold_type,
                    max_type, on_trackbar );

    createTrackbar( trackbar_value,
                    window_name_threshold, &threshold_value,
                    max_value, on_trackbar );

    on_trackbar(0,0);
    waitKey();

    //count the coins
    cout << "Number of contours: " << contours.size() << endl;
    cout << "Sum of Coins: " << countSum << endl;

    cv::waitKey();
    return 0;
}