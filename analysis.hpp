#ifndef ANALYSIS_HPP
#define ANALYSIS_HPP

#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "esenetcam.h"

class Analysis
{
public:
  Analysis(NET_CAMERA_CAPABILITES currentCameraParam)
      : currentCameraParam_(currentCameraParam){};
  ~Analysis() = default;

  struct CAM_PARAMS
  {
    esenetcam_unsigned_long_t shutter;
    esenetcam_unsigned_long_t gain;
  };

  cv::Mat getHistImage(const cv::Mat &histogram);
  cv::Mat getHist(const cv::Mat &image);
  void showHist(const cv::Mat &hist);

  CAM_PARAMS getNewParams(cv::Mat image, esenetcam_unsigned_long_t currShutter, esenetcam_unsigned_long_t currGain);

private:
  NET_CAMERA_CAPABILITES currentCameraParam_;
  float firstMoment(const cv::Mat &hist);
  float thirdMoment(const cv::Mat &hist);
  esenetcam_unsigned_long_t f_shutter(double metric);
  esenetcam_unsigned_long_t f_gain(double metric);
};

#endif //ANALYSIS_HPP
