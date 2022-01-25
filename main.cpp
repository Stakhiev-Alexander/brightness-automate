#include <chrono>
#include <iostream>

#include "cameradatamanager.hpp"


int main()
{
  CameraDataManager &camDM = CameraDataManager::GetInstance();

  cv::namedWindow("Main", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Hist", cv::WINDOW_AUTOSIZE);

  camDM.loadCamerasList();
  camDM.setCurrentCameraName("192.168.1.34");

  std::cout << camDM.getCurrentCameraName() << std::endl;

  camDM.startVideoStream();
  camDM.setCameraFeature(SHUTTER, 14);
  camDM.setCameraFeature(GAIN, 250);
  camDM.setCameraFeature(GAMMA, 10);

  std::this_thread::sleep_for(std::chrono::seconds(200000));
  camDM.stopVideoStream();
}
