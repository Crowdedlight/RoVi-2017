#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace cv;
using namespace std;

void dftshift(cv::Mat& mag);
Mat bhpf(float d0, int n, Size size); //from Kims solution to assignment 4.
cv::Mat histogram_image(const cv::Mat& hist); //from Kims solution to assignment 2

int main() {

    //region Image3

    // Load image3 as grayscale
    string filename = "../imgs/Image3.png";
    Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);

    if (img.empty()) {
        std::cout << "Input image not found at '" << filename << "'\n";
        return 1;
    }

    //crop image to largest uniform surface
    Mat cropped (img, Rect(Point(830,1460), Point(1468, 1747)));

    //histogram of cropped
    Mat hist;
    calcHist(std::vector<cv::Mat>{cropped},
                 {0}, // channels
                 noArray(), // mask
                 hist, // output histogram
                 {256}, // sizes (number of bins)
                 {0, 256} // ranges (bin lower/upper boundaries)
    );

    // Expand the image to an optimal size.
    cv::Mat padded;
    int opt_rows = cv::getOptimalDFTSize(cropped.rows);
    int opt_cols = cv::getOptimalDFTSize(cropped.cols);
    cv::copyMakeBorder(cropped, padded, 1, opt_rows - cropped.rows, 1, opt_cols - cropped.cols, BORDER_CONSTANT, Scalar::all(0));

    // Make place for both the real and complex values by merging planes into a
    // cv::Mat with 2 channels.
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat_<float>::zeros(padded.size())};
    cv::Mat complex;
    merge(planes, 2, complex);

    // Compute DFT
    dft(complex, complex);

    split(complex, planes);

    cv::Mat mag, phase;
    cartToPolar(planes[0], planes[1], mag, phase);

    //shift dft
    dftshift(mag);
    // Switch to logarithmic scale to be able to display on screen
    mag += cv::Scalar::all(1);
    cv::log(mag, mag);

    //normalize images
    cv::normalize(mag, mag, 0, 1, NORM_MINMAX);

    //Visualize input, cropped, histogram and spectrum
    //imshow("Input image", img);
    imshow("Cropped image", cropped);
    imshow("Historigram cropped", histogram_image(hist));
    imshow("Magnitude cropped", mag);

    //looks like uniform noise that isn't completely ideal. From magnitude it looks like we could enhance image
    //with


    //endregion

    //region Image4_1

    //endregion

    waitKey();
    return 0;
}

//Kims implementation of butterworth highpass filter.
Mat bhpf(float d0, int n, Size size)
{

}

// Draws the 1D histogram 'hist' and returns the image. From Kims solution
cv::Mat histogram_image(const cv::Mat& hist)
{
    int nbins = hist.rows;

    double max = 0;
    double min = 0;
    cv::minMaxLoc(hist, &min, &max);

    cv::Mat img = cv::Mat::zeros(nbins, nbins, CV_8U);

    for (int i = 0; i < nbins; i++) {
        double h = nbins * (hist.at<float>(i) / max); // Normalize
        cv::line(img, cv::Point(i, nbins), cv::Point(i, nbins - h), cv::Scalar::all(255));
    }

    return img;
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