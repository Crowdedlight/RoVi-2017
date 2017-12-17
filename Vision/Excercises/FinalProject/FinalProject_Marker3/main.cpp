#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/videoio.hpp"

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

vector<Point2f> applyShift(Mat &image_object, Mat &image_scene, Mat &detecedObject)
{

    //-- Step 1: Verify if marker and scene images are being read
    if( !image_object.data || !image_scene.data )
    {
        cout<< " --(!) Error reading images " << endl;
        throw;
    }

    //-- Step 2: Detect the keypoints using SURF Detector
    int minHessian = 400;

    Ptr<SURF>detector = SURF::create( minHessian );

    vector<KeyPoint> keypoints_object, keypoints_scene;

    Mat descriptors_object, descriptors_scene;

    detector->detectAndCompute( image_object,Mat(), keypoints_object, descriptors_object );
    detector->detectAndCompute( image_scene,Mat(),  keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    vector< DMatch > matches;
    matcher.match(descriptors_object, descriptors_scene, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Step 4: Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ )
    { if( matches[i].distance < 3*min_dist )
        { good_matches.push_back( matches[i]); }
    }

    Mat img_matches;
    drawMatches( image_object, keypoints_object, image_scene, keypoints_scene,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Step 5: Localize the object
    vector<Point2f> obj;
    vector<Point2f> scene;

    for (auto &good_matche : good_matches) {
        //-- Step 6: Get the keypoints from the good matches
        obj.push_back( keypoints_object[good_matche.queryIdx ].pt );
        scene.push_back( keypoints_scene[good_matche.trainIdx ].pt );
    }
    Mat H = findHomography( obj, scene, RANSAC);

    //-- Step 7: Get the corners from the image_1 ( the object to be "detected" )
    vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( image_object.cols, 0 );
    obj_corners[2] = cvPoint( image_object.cols, image_object.rows );
    obj_corners[3] = cvPoint( 0, image_object.rows );
    vector<Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);

    //-- Step 8: Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f( image_object.cols, 0), scene_corners[1] + Point2f( image_object.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f( image_object.cols, 0), scene_corners[2] + Point2f( image_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f( image_object.cols, 0), scene_corners[3] + Point2f( image_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f( image_object.cols, 0), scene_corners[0] + Point2f( image_object.cols, 0), Scalar( 0, 255, 0), 4 );

    detecedObject = img_matches;

    return scene_corners;
}

/** @function main */
int main( int argc, char* argv[] )
{

    cv::CommandLineParser parser(argc, argv,
                                 "{help     |                                               | print this message}"
                                 "{@image   | ../marker_corny_hard/marker_corny_hard_%02d.png| image path}"
                                 "{@marker  | ../marker3.png                                | marker image path}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    string imgPath = parser.get<std::string>("@image");
    string markerPath = parser.get<std::string>("@marker");

    Mat img_object = imread(markerPath);

    VideoCapture sequence(imgPath);

    if (!sequence.isOpened()){
        cerr << "Failed to open Image Sequence!\n" << endl;
        return 1;
    }

    ofstream outfile;
    outfile.open("../Outfiles/time_processing.txt");
    outfile.clear();

    int counter = 0;

    Mat img_scene;
    for(;;){

        sequence>>img_scene;
        if(img_scene.empty()){
            cout<<"End of Sequence"<<endl;
            break;
        }

        //get amount of time to process each image
        long beforeCount = getTickCount();

        Mat detectedObject = img_scene.clone();
        vector<Point2f> scene_corners = applyShift(img_object, img_scene, detectedObject);

        long afterCount = getTickCount();
        double time = (afterCount - beforeCount) / getTickFrequency();

        outfile << time << endl;

        cout << "Image " << ++counter << "  Time taken: " << time << "sec" << endl;
        for(int i=0;i<4;++i){
            cout<<"scene_corners: p"<<i<<"("<<scene_corners[i].x<<","<<scene_corners[i].y<<")"<<endl;//"\t scene_corners: p"<<i<<"("<<scene_corners[i].x<<","<<scene_corners[i].y<<")"<<endl;
        }

        //indicate corners found on input image
        circle(img_scene, scene_corners[0], 3, Scalar(0,0,255), -1,8,0);
        circle(img_scene, scene_corners[1], 3, Scalar(0,0,255), -1,8,0);
        circle(img_scene, scene_corners[2], 3, Scalar(0,0,255), -1,8,0);
        circle(img_scene, scene_corners[3], 3, Scalar(0,0,255), -1,8,0);


        //-- Step 9: Show detected matches
        imshow("Marker 3 & Marker detection", img_scene );
        imshow("Object detected in image", detectedObject);
        waitKey(0);
    }
    outfile.close();
    return 0;
}
