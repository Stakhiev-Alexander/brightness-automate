#include "cameradatamanager.hpp"

namespace
{
  void cameraCallback(const char *szCameraName, int pEventID, int pEventDataSize,
                      void *pEventData, void *pUserData)
  {
    CameraDataManager &camDM = CameraDataManager::GetInstance();
    try
    {
      if (pEventID == NEW_FRAME)
      {
        camDM.checkFrameProcessingIsFinished();
        std::thread processingThread(&CameraDataManager::processNewFrame, &camDM);
        processingThread.detach();
      }
      else
      {
        std::cout << "not a new frame; code : " << pEventID << std::endl;
        camDM.checkFrameProcessingIsFinished();
        std::thread processingThread(&CameraDataManager::stopVideoStreamOnError,
                                     &camDM);
        processingThread.detach();
      }
    }
    catch (...)
    {
    }
  }
}

CameraDataManager::~CameraDataManager() { stopVideoStream(); }

CameraDataManager &CameraDataManager::GetInstance()
{
  static CameraDataManager camDM;
  return camDM;
};

void CameraDataManager::loadCamerasList()
{
  DEVICE_DATA cameraData;
  Net_GetCameraList(&cameraData);
  camNames_.clear();

  for (std::size_t i = 0; i < cameraData.numDevices; ++i)
  {
    camNames_.emplace_back(cameraData.deviceList[i].DeviceName);
  }
  if (camNames_.empty())
  {
    currentCameraName_.clear();
  }
  else if (currentCameraName_.empty() ||
           std::find(camNames_.begin(), camNames_.end(),
                     currentCameraName_) == camNames_.end())
  {
    currentCameraName_ = camNames_.front();
  }

  updateCameraCapabilities();
}

const std::string &CameraDataManager::getCurrentCameraName() const
{
  return currentCameraName_;
}

void CameraDataManager::setCurrentCameraName(std::string newCameraName)
{
  currentCameraName_ = newCameraName;
}

void CameraDataManager::updateCameraCapabilities()
{
  if (currentCameraName_.empty())
  {
    return;
  }

  currentCameraParam_ = NET_CAMERA_CAPABILITES{};
  PNET_CAMERA_CAPABILITES pCamCap = nullptr;
  if (Net_GetCameraCapabilites(currentCameraName_.c_str(), &pCamCap) != 0)
  {
    if (pCamCap)
    {
      currentCameraParam_ = *pCamCap;
    }
  }

  analysis_.reset(new Analysis(currentCameraParam_));
}

esenetcam_unsigned_long_t
CameraDataManager::getCameraFeature(esenetcam_unsigned_long_t featureId)
{
  if (currentCameraName_.empty())
  {
    return 0;
  }
  esenetcam_unsigned_long_t featureVal;
  Net_GetCameraFeature(currentCameraName_.c_str(), featureId, &featureVal);
  return featureVal;
}

void CameraDataManager::setCameraFeature(esenetcam_unsigned_long_t featureId,
                                         esenetcam_unsigned_long_t featureVal)
{
  if (currentCameraName_.empty())
  {
    return;
  }
  Net_SetCameraFeature(currentCameraName_.c_str(), featureId, featureVal);
}

bool CameraDataManager::isStreamWorked() { return streamWorked_; }

bool CameraDataManager::startVideoStream()
{
  if (currentCameraName_.empty())
  {
    loadCamerasList();
    if (currentCameraName_.empty())
    {
      return false;
    }
  }

  if (isStreamWorked())
  {
    return true;
  }

  realBuffers_.clear();
  camBuffers_.BufferCnt = 3;
  camBuffers_.BufferSize =
      Net_GetRequiredBufferSize(currentCameraName_.c_str());
  for (std::size_t i = 0; i < camBuffers_.BufferCnt; ++i)
  {
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

  return streamWorked_;
}

bool CameraDataManager::stopVideoStream()
{
  if (!isStreamWorked())
  {
    return true;
  }
  streamWorked_ = !Net_StopVideoStream(currentCameraName_.c_str());
  if (!streamWorked_)
  {
    checkFrameProcessingIsFinished();
    camBuffers_ = DIRECT_BUFFER_ARRAY{};
    realBuffers_.clear();
  }
  return !streamWorked_;
}

void CameraDataManager::stopVideoStreamOnError()
{
  if (!isStreamWorked())
  {
    return;
  }
  checkFrameProcessingIsFinished();
  camBuffers_ = DIRECT_BUFFER_ARRAY{};
  realBuffers_.clear();
  streamWorked_ = false;
}

void CameraDataManager::checkFrameProcessingIsFinished()
{
  std::lock_guard oLock(frameProcessingMutex_);
  pCurrentBuffer_ = camBuffers_.pCurrentBuffer;
}

void CameraDataManager::processNewFrame()
{
  std::lock_guard oLock(frameProcessingMutex_);

  try
  {
    if (camBuffers_.pCurrentBuffer)
    {
      RET_SERV_DATA_HEADER &rRetHeader =
          *reinterpret_cast<RET_SERV_DATA_HEADER *>(pCurrentBuffer_);
      unsigned char *pDirectImageData = nullptr;
      unsigned short width = 0;
      unsigned short height = 0;
      unsigned short curGain = 0;
      esenetcam_unsigned_long_t curShutter = 0;

      if ((rRetHeader.ulNextDataSize == 0))
      {
        return;
      }
      switch (rRetHeader.ulServDataType)
      {
      case 3:
      {
        RET_SERV_DATA_ITK4_0 &rRetSrv3 = *reinterpret_cast<RET_SERV_DATA_ITK4_0 *>(pCurrentBuffer_);
        serviceData_ = rRetSrv3.StreamServData;
        STREAM_SERV_DATA_ITK4_0 &sServDataItk4 = std::get<2>(serviceData_);

        esenetcam_unsigned_long_t ulServTableSize =
            sizeof(RET_SERV_DATA_ITK4_0);

        pDirectImageData =
            (pCurrentBuffer_ + ulServTableSize);

        width = sServDataItk4.usWidth;
        height = sServDataItk4.usHeight;
        curGain = sServDataItk4.usCurrGain;
        curShutter = sServDataItk4.ulCurrShutter;
      }
      break;
      case 2:
      {
        RET_SERV_DATA_TYPE_2 &rRetSrv2 = *reinterpret_cast<RET_SERV_DATA_TYPE_2 *>(pCurrentBuffer_);
        serviceData_ = rRetSrv2.StreamServData;
        STREAM_SERV_DATA_TYPE_2 &sServDataType2 = std::get<1>(serviceData_);

        esenetcam_unsigned_long_t ulServTableSize =
            sizeof(RET_SERV_DATA_TYPE_2);
        pDirectImageData =
            (pCurrentBuffer_ + ulServTableSize);

        width = sServDataType2.ulWidth;
        height = sServDataType2.ulHeight;
        curGain = sServDataType2.ulCurrGain;
        curShutter = sServDataType2.ulCurrShutter;
      }
      break;
      }

      cv::Mat image(height, width, CV_8UC1, pDirectImageData);
      cv::resize(image, image, cv::Size(), 0.25, 0.25);
      cv::imshow("Main", image);
      char c = (char)cv::waitKey(1);

      // Analyse here!
      Analysis::CAM_PARAMS newParams = analysis_->getNewParams(image, curShutter, curGain);

      setCameraFeature(GAIN, newParams.gain);
      setCameraFeature(SHUTTER, newParams.shutter);
    }
  }
  catch (...)
  {
  }
}
