#include "analysis.hpp"
#include <iostream>
#include <cmath>

cv::Mat Analysis::getHistImage(const cv::Mat &histogram)
{
  assert(!histogram.empty());

  const int bin_w = 4;
  int n_bins = histogram.rows;
  cv::Mat histogram_view(400, n_bins * bin_w, CV_8UC1);
  histogram_view.setTo(0);
  cv::Mat hist_normed;
  cv::normalize(histogram, hist_normed, 0, histogram_view.rows, cv::NORM_MINMAX, -1, cv::Mat());

  for (int i = 1; i < n_bins; i++)
  {
    cv::line(histogram_view,
             cv::Point(bin_w * (i - 1), histogram_view.rows - cvRound(hist_normed.at<float>(i - 1))),
             cv::Point(bin_w * (i), histogram_view.rows - cvRound(hist_normed.at<float>(i))),
             cv::Scalar(255), 2, 8, 0);
    if (i % 64 == 0)
    {
      cv::line(histogram_view,
               cv::Point(bin_w * i, 0),
               cv::Point(bin_w * i, histogram_view.rows - 1),
               cv::Scalar(255), 1, 8, 0);
    }
  }

  return histogram_view;
}

cv::Mat Analysis::getHist(const cv::Mat &image)
{
  cv::Mat hist;
  int histSize = 256;
  float range[] = {0, 256};
  const float *histRange = {range};
  bool uniform = true;
  bool accumulate = false;
  cv::calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
  return hist;
}

void Analysis::showHist(const cv::Mat &hist)
{
  cv::Mat hist_img = getHistImage(hist);
  cv::imshow("Hist", hist_img);
  cv::waitKey(1);
}

float Analysis::firstMoment(const cv::Mat &hist)
{
  double desired_balance = 0.5;
  double sum = 0.0;
  int total_pix = cv::sum(hist)[0];
  hist /= total_pix;
  for (int i = 0; i < hist.rows; i++)
  {
    sum += (i - 128) / 128.0 * hist.at<float>(i);
  }
  return sum;
};

float Analysis::thirdMoment(const cv::Mat &hist)
{
  double desired_balance = 0.5;
  double sum = 0.0;
  int total_pix = cv::sum(hist)[0];
  hist /= total_pix;
  for (int i = 0; i < hist.rows; i++)
  {
    sum += std::pow((i - 128) / 128.0, 3) * hist.at<float>(i);
  }
  return sum;

  // sum /= hist.rows;
  // float light_required = desired_balance / balance;
  // light_required = std::min(light_required, 5.0f);
  // return light_required;
};

esenetcam_unsigned_long_t Analysis::f_shutter(double metric)
{
  if (abs(metric) < 0.01)
  {
    return abs(metric * 4000);
  }
  return (esenetcam_unsigned_long_t)abs(metric * 400);
}

esenetcam_unsigned_long_t Analysis::f_gain(double metric)
{
  if (abs(metric) < 0.01)
  {
    return abs(metric * 500);
  }
  return (esenetcam_unsigned_long_t)abs(metric * 50);
}

Analysis::CAM_PARAMS Analysis::getNewParams(cv::Mat image, esenetcam_unsigned_long_t currShutter, esenetcam_unsigned_long_t currGain)
{
  cv::Mat hist = getHist(image);
  showHist(hist);
  double metric = thirdMoment(hist);
  std::cout << "metric = " << metric << std::endl;
  std::cout << "f_gain = " << f_gain(metric) << std::endl;
  std::cout << "f_shutter = " << f_shutter(metric) << std::endl;
  std::cout << "currShutter = " << currShutter << std::endl;
  std::cout << "currGain = " << currGain << std::endl;

  CAM_PARAMS camParams = {currShutter, currGain};

  if (metric > 0.0)
  {
    if (currGain - f_gain(metric) > currentCameraParam_.Gain.MinValue)
    {
      camParams.gain = currGain - f_gain(metric);
    }
    else if (currShutter - f_shutter(metric) > currentCameraParam_.Shutter.MinValue)
    {
      camParams.shutter = currShutter - f_shutter(metric);
    }
  }
  else
  {
    if (currShutter + f_shutter(metric) < currentCameraParam_.Shutter.MaxValue)
    {
      camParams.shutter = currShutter + f_shutter(metric);
    }
    else if (currGain + f_gain(metric) < currentCameraParam_.Gain.MaxValue)
    {
      camParams.gain = currGain + f_gain(metric);
    }
  }
  return camParams;
}
