#include "analysis.hpp"

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

int Analysis::unusedSidesDiff(const cv::Mat &hist)
{
  // count number of unused bins in the histogram from the left side
  const int UNUSED_BIN_THRESHOLD = 500;
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

  if ((zerosLeft > zerosRight && zerosLeft < 15) || (zerosLeft < zerosRight && zerosRight < 15))
  {
    return 0;
  }

  return zerosLeft - zerosRight;
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

Analysis::CAM_PARAMS Analysis::getNewParams(cv::Mat image, esenetcam_unsigned_long_t currShutter, esenetcam_unsigned_long_t currGain, esenetcam_unsigned_long_t currGamma)
{
  cv::Mat hist = getHist(image);
  showHist(hist);

  std::cout << "gain = " << currGain << std::endl;
  std::cout << "shutter = " << currShutter << std::endl;
  std::cout << "gamma = " << currGamma << std::endl;
  std::cout << std::endl;

  CAM_PARAMS camParams = {currShutter, currGain, currGamma};
  float totalPix = cv::sum(hist)[0];
  float edgePixDiff = (hist.at<float>(255) - hist.at<float>(0)) / totalPix;

  // if one edge of the histogram is bigger than other within a threshold
  const float EDGE_PIX_DIFF_THRESHOLD = 0.0025;
  if (abs(edgePixDiff) > EDGE_PIX_DIFF_THRESHOLD)
  {
    std::cout << "in  EDGE_PIX_DIFF_THRESHOLD" << std::endl;
    std::cout << "GainShutter" << std::endl;

    return changeShutterOrGain(edgePixDiff, camParams);
  }

  int unusedSidesDiffRet = unusedSidesDiff(hist);
  // if one end is empty, move the histogram there
  const int UNUSED_NUMBER_OF_BINS_THRESHOLD = 25;
  if (abs(unusedSidesDiffRet) > UNUSED_NUMBER_OF_BINS_THRESHOLD)
  {
    std::cout << "in  UNUSED_NUMBER_OF_BINS_THRESHOLD" << std::endl;
    return changeShutterOrGain(unusedSidesDiffRet, camParams);
  }

  // float thridMomentRet = thirdMoment(hist);

  // const float THIRD_MOMENT_THRESHOLD = 0.025;
  // if (abs(thridMomentRet) > THIRD_MOMENT_THRESHOLD)
  // {
  //   std::cout << "in  THIRD_MOMENT_THRESHOLD" << std::endl;
  //   std::cout << "GainShutter" << std::endl;
  //   return changeShutterOrGain(thridMomentRet, camParams);
  // }

  // // gamma
  // double min, max;
  // int minLoc, maxLoc;
  // cv::minMaxIdx(hist, &min, &max, &minLoc, &maxLoc);
  // const int PEAK_CENTERED_THRESHOLD = 3;
  // if (abs(maxLoc - 128) > PEAK_CENTERED_THRESHOLD)
  // {
  //   int diff = maxLoc - 128;
  //   camParams.gamma = currGamma + (diff > 0 ? 1 : -1);
  // }
  // return camParams;

  float thridMomentRet = thirdMoment(hist);

  const float THIRD_MOMENT_THRESHOLD = 0.01;
  if (abs(thridMomentRet) > 0.01)
  {
    std::cout << "in  THIRD_MOMENT_THRESHOLD" << std::endl;
    std::cout << "Gamma" << std::endl;
    if (camParams.gamma - 1 > currentMinMaxParams_.gamma.minValue && thridMomentRet < 0)
    {
      camParams.gamma -= 1;
    }
    else if (camParams.gamma + 1 < currentMinMaxParams_.gamma.maxValue && thridMomentRet > 0)
    {
      camParams.gamma += 1;
    }
  }

  return camParams;
}

Analysis::CAM_PARAMS Analysis::changeShutterOrGain(float metric, CAM_PARAMS &camParams)
{
  // TODO: there are cases on edges then params dont change then they should
  // example: gain && shutter && gamma == 0, so we have to make gamma higher but we end up in this method

  if (metric > 0.0)
  {
    if (camParams.gain - f_gain_ee(metric) > currentMinMaxParams_.gain.minValue)
    {
      camParams.gain -= f_gain_ee(metric);
    }
    else if (camParams.shutter - f_shutter_ee(metric) > currentMinMaxParams_.shutter.minValue)
    {
      camParams.shutter -= f_shutter_ee(metric);
    }
  }
  else
  {
    if (camParams.shutter + f_shutter_ee(metric) < currentMinMaxParams_.shutter.maxValue)
    {
      camParams.shutter += f_shutter_ee(metric);
    }
    else if (camParams.gain + f_gain_ee(metric) < currentMinMaxParams_.gain.maxValue)
    {
      camParams.gain += f_gain_ee(metric);
    }
  }
  return camParams;
}
