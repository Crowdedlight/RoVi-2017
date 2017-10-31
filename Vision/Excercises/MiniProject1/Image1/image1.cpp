#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

using namespace cv;
using namespace std;

void showImage(string name, Mat img, bool convert = false)
{
    namedWindow(name, WINDOW_NORMAL);
    imshow(name,img);

    //convert so image can be saved
    if(convert)
        img.convertTo(img,CV_8UC3,255.0);

    imwrite("../../outimg/image1/" + name + ".png", img);
}

void dftshift(cv::Mat& mag);
Mat histogram_image(const Mat& hist); //from Kims solution to assignment 2
void ContraharmonicFilter(Mat &img, Mat &imgFiltered, int windowSize, double Q);
Mat getWindowFiltering(Mat &img, int x_center, int y_center, int windowSize);

int main(int argc, char* argv[]) {

    //region Image1
    CommandLineParser parser(argc, argv,
            "{help   |            | print this message}"
            "{@image | ../../imgs/Image1.png | image path}"
            "{filter |            | toggle to high-pass filter the input image}"
        );


    // Load image1 as grayscale
    string filename = parser.get<string>("@image");//"./Image1.png";
     Mat img = imread(filename, IMREAD_GRAYSCALE);

    if (img.empty()) {
        cout << "Input image not found at '" << filename << "'\n";
        return 1;
    }

    //crop image to largest uniform surface
    Mat cropped (img, Rect(Point(830,1460), Point(1468, 1747)));

    //histogram of the original image
    Mat hist0;
     calcHist(vector<Mat>{img},
                 {0}, // channels
                 noArray(), // mask
                 hist0, // output histogram
                 {256}, // sizes (number of bins)
                 {0, 256} // ranges (bin lower/upper boundaries)
    );

    //histogram of cropped
    Mat hist;
    calcHist(vector<Mat>{cropped},
                 {0}, // channels
                 noArray(), // mask
                 hist, // output histogram
                 {256}, // sizes (number of bins)
                 {0, 256} // ranges (bin lower/upper boundaries)
    );

    // Expand the image to an optimal size.
    Mat padded;
    int opt_rows = getOptimalDFTSize(img.rows);
    int opt_cols = getOptimalDFTSize(img.cols);
    copyMakeBorder(img, padded, 1, opt_rows - img.rows, 1, opt_cols - img.cols, BORDER_CONSTANT, Scalar::all(0));

    // Make place for both the real and complex values by merging planes into a
    // cv::Mat with 2 channels.
    Mat planes[] = {Mat_<float>(padded), Mat_<float>::zeros(padded.size())};
    Mat complex;
    merge(planes, 2, complex);

    // Compute DFT
    dft(complex, complex);

    //split
    split(complex, planes);

    //get magnitude and phase
    Mat mag;
    magnitude(planes[0], planes[1], mag);

    //shift dft
    dftshift(mag);
    // Switch to logarithmic scale to be able to display on screen
    mag += Scalar::all(1);
    log(mag, mag);

    //normalize images
    normalize(mag, mag, 0, 1, NORM_MINMAX);

    //Visualize input, cropped, histogram and spectrum
    showImage("Input_image", img);
    showImage("Cropped_image", cropped);
    showImage("Full Histogram",histogram_image(hist0));
    showImage("Histogram_cropped", histogram_image(hist));
    showImage("Magnitude_cropped", mag, true);

    Mat filtered(img.size(), img.type());

    ContraharmonicFilter(img, filtered, 5, 1.8);

    //new histogram to see filter effect
    Mat cropped2 (filtered, Rect(Point(830,1460), Point(1468, 1747)));
    Mat filteredHist;
    calcHist(vector<Mat>{cropped2},
             {0}, // channels
             noArray(), // mask
             filteredHist, // output histogram
             {256}, // sizes (number of bins)
             {0, 256} // ranges (bin lower/upper boundaries)
    );
    showImage("Filtered_histogram", histogram_image(filteredHist));

    //show filtered image
    showImage("Filtered image", filtered);

    //endregion

    waitKey();
    return 0;
}

// Draws the 1D histogram 'hist' and returns the image. From Kims solution
Mat histogram_image(const Mat& hist)
{
    int nbins = hist.rows;

    double max = 0;
    double min = 0;
    minMaxLoc(hist, &min, &max);

    Mat img = Mat::zeros(nbins, nbins, CV_8U);

    for (int i = 0; i < nbins; i++) {
        double h = nbins * (hist.at<float>(i) / max); // Normalize
        line(img, cv::Point(i, nbins), Point(i, nbins - h), Scalar::all(255));
    }

    return img;
}

// Rearranges the quadrants of a Fourier image so that the origin is at the
// center of the image.
void dftshift(Mat& mag)
{
    int cx = mag.cols / 2;
    int cy = mag.rows / 2;

    Mat tmp;
    Mat q0(mag, Rect(0, 0, cx, cy));
    Mat q1(mag, Rect(cx, 0, cx, cy));
    Mat q2(mag, Rect(0, cy, cx, cy));
    Mat q3(mag, Rect(cx, cy, cx, cy));

    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);
}

Mat getWindowFiltering(Mat &img, int x_center, int y_center, int windowSize)
{
    int x1 = x_center-windowSize;
    int y1 = y_center-windowSize;
    int x2 = x_center+windowSize;
    int y2 = y_center+windowSize;

    //check for bounds. x1,y1 can only be less than 0
    if (x1 < 0)
        x1 = 0;

    if (y1 < 0)
        y1 = 0;

    //x2,y2 can only be more than width/height
    if (x2 > img.size().width)
        x2 = img.size().width;

    if (y2 > img.size().height)
        y2 = img.size().height;

    //get window
    Mat roi(img, Rect(Point(x1,y1), Point(x2,y2)));
    return roi;
}

void ContraharmonicFilter(Mat &img, Mat &imgFiltered, int windowSize, double Q)
{
    for (int x = 0; x < img.size().width; x++)
    {
        for (int y = 0; y < img.size().height; y++)
        {
            double upperSum = 0;
            double lowerSum = 0;

            //get window
            Mat window = getWindowFiltering(img, x,y,windowSize);

            //go though entire window
            for (int s = 0; s < window.size().width; s++)
            {
                for (int t = 0; t < window.size().height; t++)
                {
                    //remember .at() is .at(rows,cols)
                    int pvalue = window.at<uchar>(s,t);
                    upperSum += pow(pvalue, Q+1);
                    lowerSum += pow(pvalue,Q);
                }
            }
            //apply new value to current pixel
            imgFiltered.at<uchar>(y,x) = upperSum/lowerSum;
        }
    }
}
