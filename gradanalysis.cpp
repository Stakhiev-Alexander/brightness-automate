#include "gradanalysis.hpp"

#define SHOW_SOBEL false

GradAnalysis::GradAnalysis(NET_CAMERA_CAPABILITES currentCameraParam, double currExposure) : Et(currExposure),
                                                                                             currentCameraParam_(
                                                                                                 currentCameraParam) {
  currentMinMaxParams_.gain.minValue = 1;
  currentMinMaxParams_.gain.maxValue = 470;
  currentMinMaxParams_.shutter.minValue = 1;
  currentMinMaxParams_.shutter.maxValue = 14;
  currentMinMaxParams_.gamma.minValue = 3;
  currentMinMaxParams_.gamma.maxValue = 15;

  for (int n = 0; n < 7; n++) {
    cv::Mat lut(1, 256, CV_8U);
    luts[n] = lut;
    uchar *p = luts[n].ptr();
    for (int i = 0; i < 256; ++i)
      p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gammas[n]) * 255.0);
  }
}


double GradAnalysis::getGradientInfoAmount(cv::Mat &image) {
  const double lambda = powf(10.0, 3.0);
  const int ksize = 3;
  cv::Mat afterBlur;
  medianBlur(image, afterBlur, ksize);
  const double sigma = cv::mean(cv::abs(afterBlur - image)).val[0] / 255.0;
  std::cout << "sigma = " << sigma << std::endl;
  const double N = log(lambda * (1 - sigma) + 1);

//  double ret = std::reduce(image.begin<uchar>(), image.end<uchar>(), 0.0,
//                           [sigma, N, lambda](double current, const uchar &p) -> double {
//                             double gradientMagnitude = (double) p / 255.0;
//                             return current + ((gradientMagnitude < sigma) ? 0 : (1 / N * log(lambda *
//                                                                                              (gradientMagnitude -
//                                                                                               sigma) + 1)));
//                           });

//  double ret = 0;
//  for (int i = 0; i < image.rows; i++) {
//    for (int j = 0; j < image.cols; j++) {
//      double gradientMagnitude = (double) image.at<uchar>(i, j) / 255.0;
//      ret += (gradientMagnitude < sigma) ? 0 : (1 / N * log(lambda * (gradientMagnitude - sigma) + 1));
//    }
//  }
//  return ret;

  cv::Mat cp;
  image.convertTo(cp, CV_32F);
  cp.forEach<float>([lambda, sigma, N](
                        float &pixel,
                        const int *position) -> void {
                      pixel /= 255.0;
                      pixel = (pixel < sigma) ? 0 : (1 / N * log(lambda * (pixel - sigma) + 1));
                    }
  );
//  std::cout << cv::sum(cp)[0] << std::endl;
  return cv::sum(cp)[0];
}

double GradAnalysis::getGammaH(std::vector<double> &y) {
  // default cubic spline (C^2) with natural boundary conditions (f''=0)
  tk::spline s(gammas, y);
  double maxY = 0;
  double maxX = 0;
  const double STEP = 0.01;
  for (double i = 0.1; i < 1.91; i += STEP) {
    double value = s(i);    // interpolated value at 1.3
    if (maxY < value) {
      maxX = i;
      maxY = value;
    }
  }

  return maxX;
}

double GradAnalysis::getNewExposure(cv::Mat &image) {
  std::vector<double> gradientInfoAmountVec(7);
  for (int i = 0; i < 7; i++) {
    cv::Mat image_cp = image.clone();
//    cv::resize(image_cp, image_cp, cv::Size(), 0.25, 0.25);
    LUT(image_cp, luts[i], image_cp);
    // Blur the image for better edge detection
//    GaussianBlur(image_cp, image_cp, cv::Size(5, 5), 0);
    Sobel(image_cp, image_cp, CV_8U, 1, 1, 5);
    gradientInfoAmountVec[i] = getGradientInfoAmount(image_cp);
  }

#if SHOW_SOBEL
  long i = std::max_element(gradientInfoAmountVec.begin(),
                            gradientInfoAmountVec.end()) - gradientInfoAmountVec.begin();
  std::cout << i << std::endl;
  LUT(image, luts[i], image);
  GaussianBlur(image, image, cv::Size(5, 5), 0);
  Sobel(image, image, CV_8U, 1, 1, 5);
#endif

  double gamma_h = getGammaH(gradientInfoAmountVec);

  const double Kp = 1.0;
  double Et1 = (1 + (gamma_h >= 1 ? 0.5 : 1) * Kp * (1 - gamma_h));


  return Et1;
}

Analysis::CAM_PARAMS
GradAnalysis::getNewParams(cv::Mat image, esenetcam_unsigned_long_t currShutter, esenetcam_unsigned_long_t currGain,
                           esenetcam_unsigned_long_t currGamma) {
  std::cout << std::endl;
  std::cout << "gain = " << currGain << std::endl;
  std::cout << "shutter = " << currShutter << std::endl;
  std::cout << "gamma = " << currGamma << std::endl;

  Analysis::CAM_PARAMS camParams = {currShutter, currGain, currGamma};
  double Et1 = getNewExposure(image);
  const double EXPOSURE_THRESHOLD = 0.01;
  if (abs(Et1 - 1) < EXPOSURE_THRESHOLD) {
    return camParams;
  }
  Et = Et1;
  changeShutterOrGain(Et, camParams);
  std::cout << Et << std::endl;
  return camParams;
}

Analysis::CAM_PARAMS GradAnalysis::changeShutterOrGain(float metric, Analysis::CAM_PARAMS &camParams) {
  // if (metric < 1.0) {
  //   if (camParams.gain - 1 > currentMinMaxParams_.gain.minValue) {
  //     camParams.gain -= 1;
  //   } else if (camParams.shutter - 20 > currentMinMaxParams_.shutter.minValue) {
  //     camParams.shutter -= 20;
  //   }
  // } else {
  //   if (camParams.shutter + 20 < currentMinMaxParams_.shutter.maxValue) {
  //     camParams.shutter += 20;
  //   } else if (camParams.gain + 1 < currentMinMaxParams_.gain.maxValue) {
  //     camParams.gain += 1;
  //   }
  // }
  // return camParams;
  /*TODO:
   * 1)edge case gain|shutter == 1
   * 2)if >max then =max
  */
  if (camParams.shutter * metric > currentMinMaxParams_.shutter.minValue &&
      camParams.shutter * metric < currentMinMaxParams_.shutter.maxValue) {
    camParams.shutter *= metric;
  } else if (camParams.gain * metric > currentMinMaxParams_.gain.minValue &&
             camParams.gain * metric < currentMinMaxParams_.gain.maxValue) {
    camParams.gain = (esenetcam_unsigned_long_t)roundf(camParams.gain * metric);
  }
  return camParams;
}
