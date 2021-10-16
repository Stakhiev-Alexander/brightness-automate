#ifndef CAMERA_DATA_MANAGER_HPP
#define CAMERA_DATA_MANAGER_HPP

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <algorithm>
#include <variant>
#include <deque>
#include <iostream>

#include "analysis.hpp"
#include "esenetcam.h"

class CameraDataManager
{
public:
  using service_data_t =
      std::variant<STREAM_SERV_DATA_TYPE_1, STREAM_SERV_DATA_TYPE_2,
                   STREAM_SERV_DATA_ITK4_0, STREAM_SERV_DATA_BRZ_5012HM_GE_V1>;

private:
  CameraDataManager() = default;
  ~CameraDataManager();

  std::string currentCameraName_;
  std::vector<std::string> camNames_;

  NET_CAMERA_CAPABILITES currentCameraParam_ = {};
  DIRECT_BUFFER_ARRAY camBuffers_ = {};
  std::vector<std::unique_ptr<unsigned char[]>> realBuffers_;
  unsigned char *pCurrentBuffer_ = nullptr;

  service_data_t serviceData_; // type of current image header

  bool streamWorked_ = false;
  std::mutex frameProcessingMutex_;

  std::unique_ptr<Analysis> analysis_;

public:
  static CameraDataManager &GetInstance();

  void loadCamerasList();
  const std::string &getCurrentCameraName() const;
  void setCurrentCameraName(std::string newCameraName);
  void updateCameraCapabilities();

  esenetcam_unsigned_long_t
  getCameraFeature(esenetcam_unsigned_long_t featureId);
  void setCameraFeature(esenetcam_unsigned_long_t featureId,
                        esenetcam_unsigned_long_t featureVal);

  bool isStreamWorked();
  bool startVideoStream();
  bool stopVideoStream();
  void stopVideoStreamOnError();
  void checkFrameProcessingIsFinished();
  void processNewFrame();
};

#define CamDM CameraDataManager::GetInstance()

#endif //CAMERA_DATA_MANAGER_HPP
