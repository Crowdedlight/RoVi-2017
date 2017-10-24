#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace cv;
using namespace std;

void showImage(string name, Mat img)
{
    namedWindow(name, WINDOW_NORMAL);
    imshow(name, img);
    imwrite("../../outimg/image4_1/" + name + ".png", img);
}

void dftshift(cv::Mat& mag);
Mat butterworth(float d0, int n, Size size, bool highpass);
Mat getMagnitudeSpectrum(Mat& img, bool shift = true);
Mat applyDft(Mat img);
void butterworthFrequencyFilter(Mat &filter, int D, int n, bool highpass, bool band, int from = 0, int to = 0);
Mat applyFilterComplexImage(Mat &img, double radius, int order, int from, int to, Mat &filterOut);
Mat getHistogram(Mat &img);

int main() {

    // Load image4 as grayscale
    string filename = "../../imgs/Image4_1.png";
    Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);

    if (img.empty()) {
        std::cout << "Input image not found at '" << filename << "'\n";
        return 1;
    }

    //show input
    showImage("input", img);

    //crop image to largest uniform surface
    Mat cropped (img, Rect(Point(830,1460), Point(1468, 1747)));

    //histogram of cropped
    Mat hist;

    //get histogram
    hist = getHistogram(cropped);
    showImage("Histogram", hist);

    //Frequency analyse
    Mat complex = applyDft(img);

    //Get magnitude to detect fails
    Mat mag = getMagnitudeSpectrum(complex);
    showImage("mag", mag);

    //looks like... TODO

    //Apply filter in frequency domain
    int radius = 200;
    int order = 6;
    Mat filterOut;
    Mat imgOut = applyFilterComplexImage(complex, radius, order, 130, 160, filterOut);

    Mat filterMag = getMagnitudeSpectrum(filterOut, false);
    showImage("FilterOut", filterMag);

    //do dft on image with filter and show new mag
    Mat filteredComplex = applyDft(imgOut);
    Mat filteredMag = getMagnitudeSpectrum(filteredComplex);
    showImage("filteredImg", imgOut);
    showImage("filteredMag", filteredMag);

    //Make another notch filter
    Mat filterOut2;
    Mat imgOut2 = applyFilterComplexImage(filteredComplex, radius, order, 432, 451, filterOut2);

    Mat filterMag2 = getMagnitudeSpectrum(filterOut2, false);
    showImage("FilterOut2", filterMag2);

    //do dft on image with filter and show new mag
    Mat filteredComplex2 = applyDft(imgOut2);
    Mat filteredMag2 = getMagnitudeSpectrum(filteredComplex2);
    showImage("filteredImg2", imgOut2);
    showImage("filteredMag2", filteredMag2);

    //endregion

    waitKey();
    return 0;
}

Mat butterworth(float d0, int n, Size size, bool highpass)
{
    cv::Mat_<cv::Vec2f> bwf(size.height, size.width, CV_32F);
    cv::Point2f c = cv::Point2f(size) / 2;

    for (int i = 0; i < size.height; ++i) {
        for (int j = 0; j < size.width; ++j) {
            // Distance from point (i,j) to the origin of the Fourier transform
            float d = std::sqrt((i - c.y) * (i - c.y) + (j - c.x) * (j - c.x));

            if(highpass)
            { //HIGHPASS
                // Real part
                if (std::abs(d) < 1.e-9f) // Avoid division by zero
                    bwf(i, j)[0] = 0;
                else {
                    bwf(i, j)[0] = 1 / (1 + std::pow(d0 / d, 2 * n));
                }
            } else //LOWPASS
            {
                bwf(i,j)[0] = 1 / (1 + std::pow(d / d0, 2 * n));
            }

            // Imaginary part
            bwf(i, j)[1] = 0;
        }
    }
    return bwf;
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
    Mat padded;                            //expand input image to optimal size
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

void butterworthFrequencyFilter(Mat &filter, int d, int n, bool highpass, bool band, int from, int to)
{
    Mat tmp;
    if(band && highpass)
    {
        tmp = butterworth(to,n,filter.size(),true);
        Mat reject = butterworth(from,n,filter.size(),false);
        tmp = tmp+reject;
    }
    else if (band && !highpass)
    {
        tmp = butterworth(to,n,filter.size(),false);
        Mat reject = butterworth(from,n,filter.size(),false);
        tmp = tmp-reject;
    }
    else
    {
        tmp = butterworth(d,n,filter.size(),highpass);
    }
    //give out filter
    filter = tmp;
}

Mat applyFilterComplexImage(Mat &img, double radius, int order, int from, int to, Mat &filterOut)
{
    //vars
    Mat imgOut;
    Mat planes[2];
    Mat filter;

    //clone img
    filter = img.clone();
    //make filter
    butterworthFrequencyFilter(filter, radius, order,true,true,from,to);

    //set filterout - So filter can be shown
    filterOut = filter;

    //shift and mul spectrums
    dftshift(img);
    mulSpectrums(img, filter, img, 0);
    //shift back
    dftshift(img);
    //inverse dft
    idft(img, img);
    //split and normalize image
    split(img, planes);
    normalize(planes[0], imgOut, 0, 1, CV_MINMAX);

    return imgOut;
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

