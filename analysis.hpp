#ifndef ANALYSIS_HPP
#define ANALYSIS_HPP

#include <iostream>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <opencv2/opencv.hpp>

#include "esenetcam.h"

class Analysis
{
public:
  struct MIN_MAX
  {
    esenetcam_unsigned_long_t minValue = 0;
    esenetcam_unsigned_long_t maxValue;
  };

  struct MIN_MAX_PARAMS
  {
    MIN_MAX shutter;
    MIN_MAX gain;
    MIN_MAX gamma;
  };

public:
  Analysis(NET_CAMERA_CAPABILITES currentCameraParam)
      : currentCameraParam_(currentCameraParam)
  {
    currentMinMaxParams_.gain.minValue = 0;
    currentMinMaxParams_.gain.maxValue = 100;
    currentMinMaxParams_.shutter.minValue = 0;
    currentMinMaxParams_.shutter.maxValue = 1000;
    currentMinMaxParams_.gamma.minValue = 3;
    currentMinMaxParams_.gamma.maxValue = 15;
  };
  ~Analysis() = default;

  struct CAM_PARAMS
  {
    esenetcam_unsigned_long_t shutter;
    esenetcam_unsigned_long_t gain;
    esenetcam_unsigned_long_t gamma;
  };

  cv::Mat getHistImage(const cv::Mat &histogram);
  cv::Mat getHist(const cv::Mat &image);
  void showHist(const cv::Mat &hist);

  CAM_PARAMS getNewParams(cv::Mat image, esenetcam_unsigned_long_t currShutter, esenetcam_unsigned_long_t currGain, esenetcam_unsigned_long_t currGamma);

private:
  NET_CAMERA_CAPABILITES currentCameraParam_;
  MIN_MAX_PARAMS currentMinMaxParams_;
  float firstMoment(const cv::Mat &hist);
  float thirdMoment(const cv::Mat &hist);
  int unusedSidesDiff(const cv::Mat &hist);
  float edgeEqualization(const cv::Mat &hist);
  esenetcam_unsigned_long_t f_shutter_ee(float metric);
  esenetcam_unsigned_long_t f_gain_ee(float metric);
  esenetcam_unsigned_long_t f_shutter(double metric);
  esenetcam_unsigned_long_t f_gain(double metric);
  CAM_PARAMS changeShutterOrGain(float metric, CAM_PARAMS &camParams);
};

#endif //ANALYSIS_HPP
