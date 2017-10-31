#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

void showImage(std::string name, cv::Mat img);

cv::Mat histogram_image(const cv::Mat& hist);
cv::Mat histogram(const cv::Mat& img);

int main(int argc, char* argv[])
{
    // Parse command line arguments
    cv::CommandLineParser parser(argc, argv,
        "{help   |               | print this message}"
        "{@image | ../../imgs/Image2.png | image path}"
    );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Load image
    std::string filename = parser.get<std::string>("@image");
    cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);

    if (img.empty()) {
        std::cout << "Input image not found at '" << filename << "'\n";
        return 1;
    }

    // Show the image
    showImage("Original", img);
    showImage("Histogram of Original image", histogram_image(histogram(img)));


    cv::Mat img_2 = img.clone();
    cv::rectangle(img_2, cv::Point(830,1460), cv::Point(1468,1747), 0, cv::FILLED);
    showImage("Original with cropped shown", img_2);

    // Make a cropped to investigate the noise:
    cv::Mat cropped(img, cv::Rect(cv::Point(830,1460), cv::Point(1468,1747)));
    showImage("Cropped", cropped);
    showImage("Histogram of Cropped image", histogram_image(histogram(cropped)));

    /** Lookes like gausian noise i the middle, and (almost) pebber noise.
        There is alot of pixles with value 1 (not 0, as for pebber).
        */

    // Perform a median blur to remove pebber noise.
    cv::Mat img_mean;
    cv::medianBlur(img, img_mean, 9);
    showImage("Mean filtering", img_mean);
    cv::Mat cropped_mean;
    cv::medianBlur(cropped, cropped_mean, 9);
    showImage("Histogram of mean filter on cropped", histogram_image(histogram(cropped_mean)));

    // Perform bilateral filter to minimize gausian noise and preserve edges
    cv::Mat img_bilateral;
    int diameter = 9;
    int sigma = 150;
    cv::bilateralFilter(img, img_bilateral, diameter, sigma, sigma, cv::BORDER_REPLICATE);
    showImage("Bilateral filter", img_bilateral);
    cv::Mat cropped_bilateral;
    cv::bilateralFilter(cropped, cropped_bilateral, diameter, sigma, sigma, cv::BORDER_REPLICATE);
    showImage("Histogram of bilateral filter on cropped", histogram_image(histogram(cropped_bilateral)));

    // Combine filters
    cv::Mat img_final;
    cv::bilateralFilter(img_mean, img_final, diameter, sigma, sigma, cv::BORDER_REPLICATE);
    showImage("Filters compined", img_final);
    cv::Mat cropped_final;
    cv::bilateralFilter(cropped_mean, cropped_final, diameter, sigma, sigma, cv::BORDER_REPLICATE);
    showImage("Histogram of filters combined on cropped", histogram_image(histogram(cropped_final)));

    cv::waitKey();
    return 0;
}

void showImage(std::string name, cv::Mat img){
    cv::namedWindow(name, cv::WINDOW_GUI_EXPANDED);
    cv::imshow(name, img);
    imwrite("../../outimg/image2/" + name + ".png", img);
}

cv::Mat histogram_image(const cv::Mat& hist){
    int nBins = hist.rows;

    double max = 0, min = 0;
    cv::minMaxLoc(hist, &min, &max);

    cv::Mat img = cv::Mat::zeros(nBins, nBins, CV_8U);

    for (int i = 0 ; i < nBins ; i++) {
        double h = nBins * (hist.at<float>(i) / max);
        cv::line(img, cv::Point(i,nBins), cv::Point(i, nBins - h), cv::Scalar::all(255));
    }

    return img;
}

cv::Mat histogram(const cv::Mat& img){
    assert(img.type() == CV_8U);

    std::vector<cv::Mat> src;
    src.push_back(img);

    std::vector<int> channels;
    channels.push_back(0);

    std::vector<int> size;
    size.push_back(256);

    std::vector<float> range;
    range.push_back(0);
    range.push_back(255);

    cv::Mat hist;
    cv::calcHist(src, channels, cv::noArray(), hist, size, range);

    return hist;
}
