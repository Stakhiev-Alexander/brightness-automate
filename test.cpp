#include <opencv2/opencv.hpp>


test(cv::Mat image) {
    // float gammas[] {0.1, 0.5, 0.8, 1.0, 1.2, 1.5, 1.9};
    // cv::Mat luts[7];

    // for (int n = 0; n < 7; n++) {
    //     cv::Mat lut(1, 256, cv::CV_8U);
    //     luts[n] = lut;
    //     uchar* p = luts[n].ptr();
    //     for (int i = 0; i < 256; ++i)
    //         p[i] = saturate_cast<uchar>(pow(i / 255.0, gammas[n]) * 255.0);
    // }

    // cv::Mat res = img.clone();
    // LUT(image, lookUpTable, res);

    // Blur the image for better edge detection
    // Mat img_blur;
    // GaussianBlur(img_gray, img_blur, Size(3,3), 0);

    Mat sobelxy;
    Sobel(image, sobelxy, CV_64F, 1, 1, 5);
    return sobelxy;
}
