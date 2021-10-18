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
  double sum = 0.0;
  int totalPix = cv::sum(hist)[0];
  hist /= totalPix;
  for (int i = 0; i < hist.rows; i++)
  {
    sum += (i - 128) / 128.0 * hist.at<float>(i);
  }
  return sum;
}

float Analysis::thirdMoment(const cv::Mat &hist)
{
  double sum = 0.0;
  int totalPix = cv::sum(hist)[0];
  hist /= totalPix;
  for (int i = 0; i < hist.rows; i++)
  {
    sum += std::pow((i - 128) / 128.0, 3) * hist.at<float>(i);
  }
  return sum;
}

float Analysis::edgeEqualization(const cv::Mat &hist)
{
  float totalPix = cv::sum(hist)[0];
  float edgePixDiff = (hist.at<float>(255) - hist.at<float>(0)) / totalPix;

  // if one edge of the histogram is bigger than other within a threshold
  const float EDGE_PIX_DIFF_THRESHOLD = 0.001;
  if (abs(edgePixDiff) > EDGE_PIX_DIFF_THRESHOLD)
  {
    return edgePixDiff;
  }

  // count number of unused bins in the histogram from the left side
  const int UNUSED_BIN_THRESHOLD = 1000;
  int zerosLeft = 0;
  for (int i = 0; i < hist.rows; i++)
  {
    if (hist.at<float>(i) < UNUSED_BIN_THRESHOLD)
    {
      zerosLeft++;
    }
    else
      break;
  }

  // count number of unused bins in the histogram from the right side
  int zerosRight = 0;
  for (int i = hist.rows; i > 0; i--)
  {
    if (hist.at<float>(i) < UNUSED_BIN_THRESHOLD)
    {
      zerosRight++;
    }
    else
      break;
  }

  // if one end is empty, move the histogram there
  const int UNUSED_NUMBER_OF_BINS_THRESHOLD = 3;
  if (abs(zerosLeft - zerosRight) > UNUSED_NUMBER_OF_BINS_THRESHOLD)
  {
    return (zerosLeft - zerosRight);
  }
  else
  {
    return 0.0;
  }
}

esenetcam_unsigned_long_t Analysis::f_shutter_ee(float metric)
{
  if (abs(metric) > 0.0001)
  {
    return 20;
  }
  return 0;
}

esenetcam_unsigned_long_t Analysis::f_gain_ee(float metric)
{
  if (abs(metric) > 0.0001)
  {
    return 1;
  }
  return 0;
}

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
  float metric = edgeEqualization(hist);
  std::cout << "metric = " << metric << std::endl;
  // std::cout << "f_gain = " << f_gain_ee(metric) << std::endl;
  // std::cout << "f_shutter = " << f_shutter_ee(metric) << std::endl;

  CAM_PARAMS camParams = {currShutter, currGain};

  if (metric > 0.0)
  {
    if (currGain - f_gain_ee(metric) > currentCameraParam_.Gain.MinValue)
    {
      camParams.gain = currGain - f_gain_ee(metric);
    }
    else if (currShutter - f_shutter_ee(metric) > currentCameraParam_.Shutter.MinValue)
    {
      camParams.shutter = currShutter - f_shutter_ee(metric);
    }
  }
  else
  {
    if (currShutter + f_shutter_ee(metric) < currentCameraParam_.Shutter.MaxValue)
    {
      camParams.shutter = currShutter + f_shutter_ee(metric);
    }
    else if (currGain + f_gain_ee(metric) < currentCameraParam_.Gain.MaxValue)
    {
      camParams.gain = currGain + f_gain_ee(metric);
    }
  }
  return camParams;
}
