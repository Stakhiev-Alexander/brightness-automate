#include "cameradatamanager.hpp"
#include "gradanalysis.hpp"

namespace {
  void cameraCallback(const char *szCameraName, int pEventID, int pEventDataSize,
                      void *pEventData, void *pUserData) {
    CameraDataManager &camDM = CameraDataManager::GetInstance();
    try {
      if (pEventID == NEW_FRAME) {
        camDM.checkFrameProcessingIsFinished();
        std::thread processingThread(&CameraDataManager::processNewFrame, &camDM);
        processingThread.detach();
      } else {
        std::cout << "not a new frame; code : " << pEventID << std::endl;
        camDM.checkFrameProcessingIsFinished();
        std::thread processingThread(&CameraDataManager::stopVideoStreamOnError,
                                     &camDM);
        processingThread.detach();
      }
    }
    catch (...) {
    }
  }
}

CameraDataManager::~CameraDataManager() { stopVideoStream(); }

CameraDataManager &CameraDataManager::GetInstance() {
  static CameraDataManager camDM;
  return camDM;
};

void CameraDataManager::loadCamerasList() {
  DEVICE_DATA cameraData;
  Net_GetCameraList(&cameraData);
  camNames_.clear();

  for (std::size_t i = 0; i < cameraData.numDevices; ++i) {
    camNames_.emplace_back(cameraData.deviceList[i].DeviceName);
  }
  if (camNames_.empty()) {
    currentCameraName_.clear();
  } else if (currentCameraName_.empty() ||
             std::find(camNames_.begin(), camNames_.end(),
                       currentCameraName_) == camNames_.end()) {
    currentCameraName_ = camNames_.front();
  }

  updateCameraCapabilities();
}

const std::string &CameraDataManager::getCurrentCameraName() const {
  return currentCameraName_;
}

const NET_CAMERA_CAPABILITES &CameraDataManager::getCurrentCameraParams() const {
  return currentCameraCapabilites_;
}

void CameraDataManager::setCurrentCameraName(std::string newCameraName) {
  if (currentCameraName_ == newCameraName) {
    return;
  }
  bool streamWasRun = isStreamWorked();
  stopVideoStream();
  if (std::find(camNames_.begin(), camNames_.end(), newCameraName) == camNames_.end()) {
    return;
  }
  currentCameraName_ = newCameraName;
  updateCameraCapabilities();
  if (streamWasRun) {
    startVideoStream();
  }
}

void CameraDataManager::updateCameraCapabilities() {
  if (currentCameraName_.empty()) {
    return;
  }

  currentCameraCapabilites_ = NET_CAMERA_CAPABILITES{};
  PNET_CAMERA_CAPABILITES pCamCap = nullptr;
  if (Net_GetCameraCapabilites(currentCameraName_.c_str(), &pCamCap) != 0) {
    if (pCamCap) {
      currentCameraCapabilites_ = *pCamCap;
    }
  }

  analysis_.reset(new Analysis(currentCameraCapabilites_));
  gradAnalysis_.reset(
      new GradAnalysis(currentCameraCapabilites_, 1.0));
}

esenetcam_unsigned_long_t
CameraDataManager::getCameraFeature(esenetcam_unsigned_long_t featureId) {
  if (currentCameraName_.empty()) {
    std::cout << "Failed to get feature. No camera set as active!" << std::endl;
    return 0;
  }
  if (!isStreamWorked()) {
    std::cout << "Failed to get feature. The stream has not been started!" << std::endl;
    return 0;
  }
  esenetcam_unsigned_long_t featureVal;
  Net_GetCameraFeature(currentCameraName_.c_str(), featureId, &featureVal);
  return featureVal;
}

void CameraDataManager::setCameraFeature(esenetcam_unsigned_long_t featureId,
                                         esenetcam_unsigned_long_t featureVal) {
  if (currentCameraName_.empty()) {
    std::cout << "Failed to set feature. No camera set as active!" << std::endl;
    return;
  }
  if (!isStreamWorked()) {
    std::cout << "Failed to set feature. The stream has not been started!" << std::endl;
    return;
  }

  bool retStatus = Net_SetCameraFeature(currentCameraName_.c_str(), featureId, featureVal);
  if (retStatus) {
    switch (featureId) {
      case GAIN:
        currentCameraParams_.gain = featureVal;
        break;
      case SHUTTER:
        currentCameraParams_.shutter = featureVal;
        break;
      case GAMMA:
        currentCameraParams_.gamma = featureVal;
        break;
    }
  }
}

bool CameraDataManager::isStreamWorked() { return streamWorked_; }

bool CameraDataManager::startVideoStream() {
  if (currentCameraName_.empty()) {
    loadCamerasList();
    if (currentCameraName_.empty()) {
      return false;
    }
  }

  if (isStreamWorked()) {
    return true;
  }

  realBuffers_.clear();
  camBuffers_.BufferCnt = 3;
  camBuffers_.BufferSize =
      Net_GetRequiredBufferSize(currentCameraName_.c_str());
  for (std::size_t i = 0; i < camBuffers_.BufferCnt; ++i) {
    realBuffers_.emplace_back(new unsigned char[camBuffers_.BufferSize]);
    camBuffers_.pBuffer[i] = realBuffers_.back().get();
  }
  camBuffers_.pCurrentBuffer = camBuffers_.pBuffer[0];

  VIDEO_STREAM_PARAM_EX streamParam = {};
  streamParam.ppReturnedParams = nullptr;
  streamParam.pBuffBitmap = nullptr;
  streamParam.pDirectBuffer = &camBuffers_;
  streamParam.pEvent = nullptr;



  streamWorked_ = Net_VideoOnCallbackStart(
      currentCameraName_.c_str(), &streamParam, cameraCallback, nullptr);

  currentCameraParams_.gain = getCameraFeature(GAIN);
  currentCameraParams_.shutter =  getCameraFeature(SHUTTER);
  currentCameraParams_.gamma =  getCameraFeature(GAMMA);
  std::cout << "Shutter step: " << currentCameraCapabilites_.ShutterStepTime << " ns" << std::endl;
  std::cout << "Shutter zero time: " << currentCameraCapabilites_.ShutterZeroTime << " ns" << std::endl;

  return streamWorked_;
}

bool CameraDataManager::stopVideoStream() {
  if (!isStreamWorked()) {
    return true;
  }
  streamWorked_ = !Net_StopVideoStream(currentCameraName_.c_str());
  if (!streamWorked_) {
    checkFrameProcessingIsFinished();
    camBuffers_ = DIRECT_BUFFER_ARRAY{};
    realBuffers_.clear();
  }
  return !streamWorked_;
}

void CameraDataManager::stopVideoStreamOnError() {
  if (!isStreamWorked()) {
    return;
  }
  checkFrameProcessingIsFinished();
  camBuffers_ = DIRECT_BUFFER_ARRAY{};
  realBuffers_.clear();
  streamWorked_ = false;
}

void CameraDataManager::checkFrameProcessingIsFinished() {
  std::lock_guard oLock(frameProcessingMutex_);
  pCurrentBuffer_ = camBuffers_.pCurrentBuffer;
}

void CameraDataManager::processNewFrame() {
  std::lock_guard oLock(frameProcessingMutex_);

  try {
    if (camBuffers_.pCurrentBuffer) {
      RET_SERV_DATA_HEADER &rRetHeader =
          *reinterpret_cast<RET_SERV_DATA_HEADER *>(pCurrentBuffer_);
      unsigned char *pDirectImageData = nullptr;
      unsigned short width = 0;
      unsigned short height = 0;
      unsigned short currGain = 0;
      esenetcam_unsigned_long_t currShutter = 0;
      unsigned char currGamma = 0;

      if (rRetHeader.ulNextDataSize == 0) {
        return;
      }
      switch (rRetHeader.ulServDataType) {
        case 3: {
          RET_SERV_DATA_ITK4_0 &rRetSrv3 = *reinterpret_cast<RET_SERV_DATA_ITK4_0 *>(pCurrentBuffer_);
          serviceData_ = rRetSrv3.StreamServData;
          STREAM_SERV_DATA_ITK4_0 &sServDataItk4 = std::get<2>(serviceData_);

          esenetcam_unsigned_long_t ulServTableSize = sizeof(RET_SERV_DATA_ITK4_0);

          pDirectImageData = (pCurrentBuffer_ + ulServTableSize);

          width = sServDataItk4.usWidth;
          height = sServDataItk4.usHeight;
          currGain = sServDataItk4.usCurrGain;
          currShutter = sServDataItk4.ulCurrShutter;
          currGamma = sServDataItk4.chCurrGamma;
        }
          break;
        case 2: {
          RET_SERV_DATA_TYPE_2 &rRetSrv2 = *reinterpret_cast<RET_SERV_DATA_TYPE_2 *>(pCurrentBuffer_);
          serviceData_ = rRetSrv2.StreamServData;
          STREAM_SERV_DATA_TYPE_2 &sServDataType2 = std::get<1>(serviceData_);

          esenetcam_unsigned_long_t ulServTableSize = sizeof(RET_SERV_DATA_TYPE_2);
          pDirectImageData = (pCurrentBuffer_ + ulServTableSize);

          width = sServDataType2.ulWidth;
          height = sServDataType2.ulHeight;
          currGain = sServDataType2.ulCurrGain;
          currShutter = sServDataType2.ulCurrShutter;
          currGamma = sServDataType2.chCurrGamma;
        }
          break;
      }

      // check if new params were applied to a frame
      if ((currentCameraParams_.gain != currGain ||
           currentCameraParams_.shutter != currShutter ||
           currentCameraParams_.gamma != currGamma)
          && (notAppliedCounter_ < 10)) {
        notAppliedCounter_++;
        return;
      }
      notAppliedCounter_ = 0;

      cv::Mat image(height, width, CV_8UC1, pDirectImageData);

      // Analyse here!
//      currentCameraParams_ = analysis_->getNewParams(image, currShutter, currGain, currGamma);
      currentCameraParams_ = gradAnalysis_->getNewParams(image, currShutter, currGain, currGamma);
      setCameraFeature(GAIN, currentCameraParams_.gain);
      setCameraFeature(SHUTTER, currentCameraParams_.shutter);
      setCameraFeature(GAMMA, currentCameraParams_.gamma);

      cv::resize(image, image, cv::Size(), 0.25, 0.25);
      cv::imshow("Main", image);
      cv::waitKey(1);
    }
  }
  catch (...) {
  }
}
