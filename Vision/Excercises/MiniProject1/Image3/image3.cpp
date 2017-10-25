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
    imwrite("../../outimg/image3/" + name + ".png", img);
}

void dftshift(cv::Mat& mag);
Mat getMagnitudeSpectrum(Mat& img, bool shift = true);
Mat applyDft(Mat img);
Mat getHistogram(Mat &img);

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
    showImage("Magnitude_cropped", mag);

    //looks like some random noise. So like gaussian noise. Using bilateralFilter
    Mat filtered;
    bilateralFilter(img, filtered, 9, 40,40);

    //new histogram to see filter effect
    Mat cropped2 (filtered, Rect(Point(830,1460), Point(1468, 1747)));
    Mat filteredHist = getHistogram(cropped2);
    showImage("Filtered histogram", filteredHist);

    //show filtered image
    //normalize(filtered, filtered, 0, 1, NORM_MINMAX);
    showImage("Filtered image", filtered);

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