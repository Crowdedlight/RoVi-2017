#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace cv;
using namespace std;

void showImage(string name, Mat img, bool convert = false)
{
    namedWindow(name, WINDOW_NORMAL);
    imshow(name, img);

    //convert so image can be saved
    if(convert)
        img.convertTo(img,CV_8UC3,255.0);

    imwrite("../../outimg/image3/" + name + ".png", img);
}

void dftshift(cv::Mat& mag);
Mat getMagnitudeSpectrum(Mat& img, bool shift = true);
Mat applyDft(Mat img);
Mat getHistogram(Mat &img);
void applyContraharmonicFilter(Mat &img, Mat &imgFiltered, int windowSize, double Q);
void applyLocalNoiseAdaptiveFilter(Mat &img, Mat &imgFiltered, int windowSize, double globalVar);
Mat getWindowFiltering(Mat &img, int x_center, int y_center, int windowSize);

int main() {

    //region Image3

    // Load image3 as grayscale
    string filename = "../../imgs/Image3.png";
    Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);

    if (img.empty()) {
        std::cout << "Input image not found at '" << filename << "'\n";
        return 1;
    }

    //crop image to largest uniform surface
    Mat cropped (img, Rect(Point(830,1460), Point(1468, 1747)));

    //histogram of cropped
    Mat hist = getHistogram(cropped);

    //Frequency magnitude of image
    Mat complex = applyDft(img);
    Mat mag = getMagnitudeSpectrum(complex);

    //Visualize input, cropped, histogram and spectrum
    showImage("Input_image", img);
    showImage("Cropped_image", cropped);
    showImage("Historigram_cropped", hist);
    showImage("Magnitude_cropped", mag, true);

    //looks like some uniform noise. Using Adaptive local noise reduction and then contraharmonicfilter
    Mat filtered(img.size(), img.type());

    //adaptive local noise reduction
    //need global variance for a area with constant intensity
    Mat globalMat, globalStd;
    //the area with constant intensity. (Same area as cropped histogram)
    Rect r(Point(830,1460), Point(1468, 1747));
    meanStdDev(img(r), globalMat, globalStd);
    double globalVariance = pow(globalStd.at<uchar>(0,0),2);

    //apply filter
    applyLocalNoiseAdaptiveFilter(img, filtered, 5, globalVariance);

    //apply contraHarmonic filter
    Mat filtered2(img.size(), img.type());
    applyContraharmonicFilter(filtered, filtered2, 5, -3.5);

    //new histogram to see filter effect
    Mat cropped2 (filtered2, Rect(Point(830,1460), Point(1468, 1747)));
    Mat filteredHist = getHistogram(cropped2);
    showImage("Filtered_histogram_cropped", filteredHist);

    //show filtered image
    showImage("Filtered_image", filtered);
    showImage("Filtered_image2", filtered2);

    //endregion

    waitKey();
    return 0;
}

///Splits complex image and returns magnitude spectrum
Mat getMagnitudeSpectrum(Mat& img, bool shift)
{
    Mat planes[2];

    //split
    split(img, planes);
    magnitude(planes[0], planes[1], planes[0]);

    //get magnitude
    Mat mag = (planes[0]).clone();
    mag += Scalar::all(1);
    log(mag, mag);

    //shift so low-frequency is in middle
    if (shift)
        dftshift(mag);

    //normalize
    normalize(mag, mag, 0, 1, CV_MINMAX);

    return mag;
}

Mat applyDft(Mat img)
{
    //expand input image to optimal size
    Mat padded;
    int m = getOptimalDFTSize( img.rows );
    int n = getOptimalDFTSize( img.cols ); // on the border add zero values
    copyMakeBorder(img, padded, 0, m - img.rows, 0, n - img.cols, BORDER_CONSTANT, Scalar::all(0));

    //create 2-channel planes to store dtf
    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
    Mat complex;

    //merge into planes
    merge(planes, 2, complex);

    //do dft
    dft(complex, complex);

    return complex;
}

Mat getHistogram(Mat &img)
{
    int histSize = 256;
    float range[] = {0,256};
    const float* histRange= {range};
    bool uniform = true;
    bool accumulate = false;
    Mat hist;

    calcHist( &img, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );

    // Draw the histograms
    int nbins = hist.rows;

    double max = 0;
    double min = 0;
    cv::minMaxLoc(hist, &min, &max);

    cv::Mat imgOut = cv::Mat::zeros(nbins, nbins, CV_8U);

    for (int i = 0; i < nbins; i++) {
        double h = nbins * (hist.at<float>(i) / max); // Normalize
        cv::line(imgOut, cv::Point(i, nbins), cv::Point(i, nbins - h), cv::Scalar::all(255));
    }
    return imgOut;
}


// Rearranges the quadrants of a Fourier image so that the origin is at the
// center of the image.
void dftshift(cv::Mat& mag)
{
    int cx = mag.cols / 2;
    int cy = mag.rows / 2;

    cv::Mat tmp;
    cv::Mat q0(mag, cv::Rect(0, 0, cx, cy));
    cv::Mat q1(mag, cv::Rect(cx, 0, cx, cy));
    cv::Mat q2(mag, cv::Rect(0, cy, cx, cy));
    cv::Mat q3(mag, cv::Rect(cx, cy, cx, cy));

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

void applyContraharmonicFilter(Mat &img, Mat &imgFiltered, int windowSize, double Q)
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

void applyLocalNoiseAdaptiveFilter(Mat &img, Mat &imgFiltered, int windowSize, double globalVar)
{
    for (int x = 0; x < img.size().width; x++)
    {
        for (int y = 0; y < img.size().height; y++)
        {

            //get window
            Mat window = getWindowFiltering(img, x, y, windowSize);

            //Local
            Mat localMat;
            Mat localStd;

            meanStdDev(window, localMat, localStd);
            double localVariance = pow(localStd.at<uchar>(0,0),2);
            double localMean = localMat.at<double>(0,0);
            double factor = globalVar/localVariance;

            //clamp if above 1
            if (factor > 1)
                factor = 1;

            //apply new value to current pixel
            imgFiltered.at<uchar>(y,x) = (double) img.at<uchar>(y,x) - factor * ((double)img.at<uchar>(y,x) - localMean);
        }
    }
}