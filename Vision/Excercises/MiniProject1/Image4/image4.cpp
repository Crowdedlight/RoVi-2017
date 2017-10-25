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

    imwrite("../../outimg/image4_2/" + name + ".png", img);
}

void dftshift(cv::Mat& mag);
Mat butterworth(float d0, int n, Size size, bool highpass, bool band = false, float bandWidth = 0);
Mat getMagnitudeSpectrum(Mat& img, bool shift = true);
Mat applyDft(Mat img);
Mat applyFilterComplexImage(Mat &img, Mat &filter);
Mat getHistogram(Mat &img);

int main() {

    // Load image4 as grayscale
    string filename = "../../imgs/Image4_2.png";
    Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);

    if (img.empty()) {
        std::cout << "Input image not found at '" << filename << "'\n";
        return 1;
    }

    //show input
    showImage("input", img);

    //crop image to largest uniform surface
    Mat cropped (img, Rect(Point(830,1460), Point(1468, 1747)));

    //get histogram
    Mat hist = getHistogram(cropped);
    showImage("Histogram", hist);

    //Frequency analyse
    Mat complex = applyDft(img);

    //Get magnitude to detect fails
    Mat mag = getMagnitudeSpectrum(complex);
    showImage("mag", mag, true);

    //looks like periodic noise. Maybe sinusoidal

    //Apply filter in frequency domain
    int radius = 410;
    int order = 4;
    int bandWidth = 35;

    //make bandreject filter
    Mat filter = butterworth(radius, order, complex.size(), false, true, bandWidth);

    Mat imgOut = applyFilterComplexImage(complex, filter);

    // Crop image (remove padded borders)
    imgOut = Mat(imgOut, cv::Rect(cv::Point(0, 0), img.size()));

    Mat filterMag = getMagnitudeSpectrum(filter, false);
    showImage("FilterOut", filterMag, true);

    //do dft on image with filter and show new mag
    Mat filteredComplex = applyDft(imgOut);
    Mat filteredMag = getMagnitudeSpectrum(filteredComplex);
    showImage("filteredImg", imgOut, true);
    showImage("filteredMag", filteredMag, true);

    waitKey();
    return 0;
}

Mat butterworth(float d0, int n, Size size, bool highpass, bool band, float bandWidth)
{
    cv::Mat_<cv::Vec2f> bwf(size.height, size.width, CV_32F);
    cv::Point2f c = cv::Point2f(size) / 2;

    for (int i = 0; i < size.height; ++i) {
        for (int j = 0; j < size.width; ++j) {
            // Distance from point (i,j) to the origin of the Fourier transform
            float d = std::sqrt((i - c.y) * (i - c.y) + (j - c.x) * (j - c.x));

            //Bandreject
            if (band)
            {
                // Real part
                if (std::abs(d) < 1.e-9f) // Avoid division by zero
                    bwf(i, j)[0] = 0;
                else {
                    bwf(i, j)[0] = 1 / (1 + pow((d0*bandWidth) / (pow(d,2)-pow(d0,2)), 2 * n));
                }
            }
            else //lowpass
            {
                bwf(i,j)[0] = 1 / (1 + std::pow(d / d0, 2 * n));
            }

            // Imaginary part
            bwf(i, j)[1] = 0;
        }

        //make highpass if needed
        if (highpass)
        {
            Mat filterOut = Scalar::all(255) - bwf;
            return filterOut;
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

Mat applyFilterComplexImage(Mat &img, Mat &filter)
{
    //vars
    Mat imgOut;
    Mat planes[2];

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

