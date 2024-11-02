#include "hnurm_camera/Camera.h"
#include "hnurm_camera/MvErrorDefine.h"
#include <chrono>
#include <cstdio>
#include <iostream>

#include <string>

#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

namespace hnurm
{
HKcam::HKcam(const std::shared_ptr<rclcpp::Node> &node) : _node(node), _logger(node->get_logger())
{
    nRet   = MV_OK;
    handle = nullptr;
    pData  = nullptr;

    init_params();
    std::string camera_id = _node->declare_parameter("camera_id", "");
    RCLCPP_INFO(_logger, "Camera id set to: %s", camera_id.c_str());

    if(OpenCam(camera_id))  // here open and config camera
    {
        RCLCPP_INFO(_logger, "Camera %s opened successfully.", camera_id.c_str());
    }
    else
    {
        RCLCPP_ERROR(_logger, "Camera %s failed to open!", camera_id.c_str());
    }
}

HKcam::~HKcam()
{
    CloseCam();
}

void HKcam::init_params()
{
    _nImageOrientation = static_cast<int>(_node->declare_parameter("image_orientation", 0));

    _nWidth              = static_cast<int>(_node->declare_parameter("nWidth", 1'280));
    _nHeight             = static_cast<int>(_node->declare_parameter("nHeight", 720));
    _nOffsetX            = static_cast<int>(_node->declare_parameter("nOffsetX", 0));
    _nOffsetY            = static_cast<int>(_node->declare_parameter("nOffsetY", 0));
    _bReverseX           = _node->declare_parameter("bReverseX", false);
    _bReverseY           = _node->declare_parameter("bReverseY", false);
    _nPixelFormat        = static_cast<int>(_node->declare_parameter("nPixelFormat", 0x02180015));
    _nAcqFrameRate       = static_cast<int>(_node->declare_parameter("nAcqFrameRate", 30));
    _fFPS                = static_cast<float>(_node->declare_parameter("fFPS", 404.));
    _bEnableAcqFrameRate = _node->declare_parameter("bEnableAcqFrameRate", false);
    _fExposureTime       = static_cast<float>(_node->declare_parameter("fExposureTime", 5000.));
    _fGain               = static_cast<float>(_node->declare_parameter("fGain", 8.));
    _nBlackLevelValue    = static_cast<int>(_node->declare_parameter("nBlackLevelValue", 30));
    _bEnableBlackLevel   = _node->declare_parameter("bEnableBlackLevel", false);
    _nBayerCvtQuality    = static_cast<int>(_node->declare_parameter("nBayerCvtQuality", 3));

    MV_CC_SetBayerCvtQuality(handle, 3);
}

void HKcam::SetParam()
{
    MV_CC_SetEnumValue(handle, "ExposureMode", 1);

    nRet = MV_CC_SetIntValue(handle, "Width", _nWidth);
    if(MV_OK == nRet)
        printf("set Width = %d OK!\n", _nWidth);
    else
        printf("set Width failed! nRet [%x]\n", nRet);

    // 设置高度
    nRet = MV_CC_SetIntValue(handle, "Height", _nHeight);
    if(MV_OK == nRet)
        printf("set height = %d OK!\n", _nHeight);
    else
        printf("set height failed! nRet [%x]\n", nRet);

    // 设置水平偏移
    nRet = MV_CC_SetIntValue(handle, "OffsetX", _nOffsetX);
    if(MV_OK == nRet)
        printf("set OffsetX = %d OK!\n", _nOffsetX);
    else
        printf("set OffsetX failed! nRet [%x]\n", nRet);

    // 设置垂直偏移
    nRet = MV_CC_SetIntValue(handle, "OffsetY", _nOffsetY);
    if(MV_OK == nRet)
        printf("set OffsetY = %d OK!\n", _nOffsetY);
    else
        printf("set OffsetY failed! nRet [%x]\n", nRet);

    // 设置水平镜像
    nRet = MV_CC_SetBoolValue(handle, "ReverseX", _bReverseX);
    if(MV_OK == nRet)
        printf("set ReverseX = %d OK!\n", _bReverseX);
    else
        printf("set ReverseX Failed! nRet = [%x]\n", nRet);

    // 设置垂直镜像
    nRet = MV_CC_SetBoolValue(handle, "ReverseY", _bReverseY);
    if(MV_OK == nRet)
        printf("Set ReverseY = %d OK!\n", _bReverseY);
    else
        printf("Set ReverseY Failed! nRet = [%x]\n", nRet);

    // 设置像素格式
    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", _nPixelFormat);
    if(MV_OK == nRet)
        printf("set PixelFormat = %x OK!\n", _nPixelFormat);
    else
        printf("set PixelFormat failed! nRet [%x]\n", nRet);

    // 设置采集触发帧率
    nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", _nAcqFrameRate);
    if(MV_OK == nRet)
        printf("set AcquisitionBurstFrameCount = %d OK!\n", _nAcqFrameRate);
    else
        printf("set AcquisitionBurstFrameCount failed! nRet [%x]\n", nRet);

    // 设置采集帧率
    nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", _fFPS);
    if(MV_OK == nRet)
        printf("set AcquisitionFrameRate = %f OK!\n", _fFPS);
    else
        printf("set AcquisitionFrameRate failed! nRet [%x]\n", nRet);

    // 设置使能采集帧率控制
    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", _bEnableAcqFrameRate);
    if(MV_OK == nRet)
        printf("Set AcquisitionFrameRateEnable = %d OK!\n", _bEnableAcqFrameRate);
    else
        printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n", nRet);

    // 设置曝光时间
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", _fExposureTime);
    if(MV_OK == nRet)
        printf("set ExposureTime = %f OK!\n", _fExposureTime);
    else
        printf("set ExposureTime failed! nRet [%x]\n", nRet);

    // 设置增益
    nRet = MV_CC_SetFloatValue(handle, "Gain", _fGain);
    if(MV_OK == nRet)
        printf("set Gain = %f OK!\n", _fGain);
    else
        printf("set Gain failed! nRet [%x]\n", nRet);

    // 设置黑电平
    nRet = MV_CC_SetIntValue(handle, "BlackLevel", _nBlackLevelValue);
    if(MV_OK == nRet)
        printf("set BlackLevel = %d OK!\n", _nBlackLevelValue);
    else
        printf("set BlackLevel failed! nRet [%x]\n", nRet);

    // 设置黑电平使能
    nRet = MV_CC_SetBoolValue(handle, "BlackLevelEnable", _bEnableBlackLevel);
    if(MV_OK == nRet)
        printf("Set BlackLevelEnable = %d OK!\n", _bEnableBlackLevel);
    else
        printf("Set BlackLevelEnable Failed! nRet = [%x]\n", nRet);
}

bool HKcam::OpenCam(const string &cameraID)
{
    //        nRet = MV_OK;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if(!stDeviceList.nDeviceNum)
    {
        printf("Find No Devices! nRet = [%x]\n", nRet);
        return false;
    }

    // select the first camera connected
    unsigned int nIndex = 0;

    while(true)
    {
        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if(MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            return false;
        }

        // 获取设备id
        // get device id
        stringstream ss;
        ss << stDeviceList.pDeviceInfo[nIndex]->SpecialInfo.stUsb3VInfo.chDeviceGUID;
        ss >> _id;
        cout << "camera id " << _id << endl;

        // 若指定了相机id，则判断是否为指定相机
        // if camera id is specified, check if it is the specified camera
        if(!cameraID.empty())
        {
            if(cameraID != _id)  // 若不是指定相机，则关闭句柄并继续枚举
            {
                printf("camera id %s not matched to desired %s\n", _id.c_str(), cameraID.c_str());
                MV_CC_CloseDevice(handle);
                MV_CC_DestroyHandle(handle);
                nIndex++;
                if(nIndex >= stDeviceList.nDeviceNum)  // 若已枚举完所有相机，则返回
                {
                    printf("Find No Devices!\n");
                    return false;
                }
                continue;
            }
            else
            {
                printf("ready to open camera %s\n", _id.c_str());
            }
        }
        else
        {
            printf("camera id not set, ready to open camera %s\n", _id.c_str());
        }

        // close first
        nRet = MV_CC_CloseDevice(handle);
        if(MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
        }

        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle);
        if(MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            return false;
        }

        // 设置触发模式为off
        // set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if(MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
            return false;
        }

        // set param
        // 设置参数
        SetParam();

        // Get payload size
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if(MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            return false;
        }

        // s tart grab stream
        nRet = MV_CC_StartGrabbing(handle);
        if(MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            return false;
        }

        // check
        stImageInfo = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        if(NULL == pData)
        {
            std::cout << "can't get size of a frame!" << std::endl;
            return false;
        }

        return true;
    }
}

// todo: this function takes too long to execute than expected when camera is not connected
bool HKcam::CloseCam()
{
    try
    {
        // 停止取流
        // end grab image
        nRet = MV_CC_StopGrabbing(handle);
        if(MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        // 关闭设备
        // close device
        nRet = MV_CC_CloseDevice(handle);
        if(MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            return false;
        }
        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if(MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            return false;
        }
        return true;
    }
    catch(...)
    {
        printf("something went wrong...");
        return false;
    }
}

bool HKcam::GetFrame(std::vector<uint8_t> &img)
{
    nRet = MV_OK;
    // todo get time stamp for imu alignment
    nRet = MV_CC_GetOneFrameTimeout(handle, pData, stParam.nCurValue, &stImageInfo, 100);
    // img.time_stamp = std::chrono::steady_clock::now();

    if(nRet != MV_OK)
    {
        return false;
    }
    img.resize(stImageInfo.nHeight * stImageInfo.nWidth * 3);
    // convert bayer
    prepare_convert(img.data());

    // do conversion
    nRet = MV_CC_ConvertPixelTypeEx(handle, &stConvertParam);

    return true;
}

void HKcam::prepare_convert(uint8_t *dstData)
{
    stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
    stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    stConvertParam.pSrcData       = pData;
    stConvertParam.nSrcDataLen    = stImageInfo.nFrameLen;
    stConvertParam.nWidth         = stImageInfo.nWidth;
    stConvertParam.nHeight        = stImageInfo.nHeight;
    stConvertParam.nDstBufferSize = stImageInfo.nWidth * stImageInfo.nHeight * 3;
    stConvertParam.pDstBuffer     = dstData;
}

}  // namespace hnurm
