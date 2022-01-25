#ifndef GRADANALYSIS_HPP
#define GRADANALYSIS_HPP

#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "esenetcam.h"

#include "spline.hpp"
#include "analysis.hpp"

class GradAnalysis {
public:
  GradAnalysis(NET_CAMERA_CAPABILITES currentCameraParam, double currExposure);

  Analysis::CAM_PARAMS
  getNewParams(cv::Mat image, esenetcam_unsigned_long_t currShutter, esenetcam_unsigned_long_t currGain,
               esenetcam_unsigned_long_t currGamma);

private:
  NET_CAMERA_CAPABILITES currentCameraParam_;
  Analysis::MIN_MAX_PARAMS currentMinMaxParams_;
  const std::vector<double> gammas = {0.1, 0.5, 0.8, 1.0, 1.2, 1.5, 1.9};
  cv::Mat luts[7];
  double Et;

  double getGammaH(std::vector<double> &y);

  static double getGradientInfoAmount(cv::Mat &image);

  double getNewExposure(cv::Mat &image);

  Analysis::CAM_PARAMS changeShutterOrGain(float metric, Analysis::CAM_PARAMS &camParams);
};

#endif //GRADANALYSIS_HPP
