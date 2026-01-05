# -- coding: utf-8 --

import sys
import numpy as np
from ctypes import *
import cv2

import os
sys.path.append(os.path.join(os.path.dirname(__file__), "./MvImport"))
from MvCameraControl_class import *


# 适用CS060, 修改像素格式以适配其他型号
class HKCam:
    def __init__(self, cameraId) -> None:
        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
        # ch:枚举设备 | en:Enum device
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            print("enum devices fail! ret[0x%x]" % ret)
            sys.exit()

        if deviceList.nDeviceNum == 0:
            print("find no device!")

        self.cam = MvCamera()
        stDeviceList = cast(
            deviceList.pDeviceInfo[int(cameraId)], POINTER(MV_CC_DEVICE_INFO)
        ).contents

        # ch:创建句柄 | en:Create handle
        ret = self.cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            print("create handle fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:打开设备 | en:Open device
        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            print("open device fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:设置触发模式为off | en:Set trigger mode as off
        ret = self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if ret != 0:
            print("set trigger mode fail! ret[0x%x]" % ret)
            sys.exit()

        # 设置相机高度
        ret = self.cam.MV_CC_SetIntValue("Height", 2048)
        if ret != 0:
            print("set height fail! ret[0x%x]" % ret)
            sys.exit()

        # 设置相机宽度
        ret = self.cam.MV_CC_SetIntValue("Width", 3072)
        if ret != 0:
            print("set width fail! ret[0x%x]" % ret)
            sys.exit()

        # 设置像素格式
        ret = self.cam.MV_CC_SetEnumValue("PixelFormat", 0x01100011)
        if ret != 0:
            print("set pixel format fail! ret[0x%x]" % ret)
            sys.exit()

        # 设置曝光时间
        ret = self.cam.MV_CC_SetFloatValue("ExposureTime", 8000)
        if ret != 0:
            print("set exposure time fail! ret[0x%x]" % ret)
            sys.exit()

        # 设置增益
        ret = self.cam.MV_CC_SetFloatValue("Gain", 20)
        if ret != 0:
            print("set gain fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:获取数据包大小 | en:Get payload size
        stParam = MVCC_INTVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))

        ret = self.cam.MV_CC_GetIntValue("PayloadSize", stParam)
        if ret != 0:
            print("get payload size fail! ret[0x%x]" % ret)
            sys.exit()
        self.nPayloadSize = stParam.nCurValue

        # ch:开始取流 | en:Start grab image
        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            print("start grabbing fail! ret[0x%x]" % ret)
            sys.exit()

        self.stOutFrame = MV_FRAME_OUT()
        memset(byref(self.stOutFrame), 0, sizeof(self.stOutFrame))

    def __del__(self):
        # ch:停止取流 | en:Stop grab image
        ret = self.cam.MV_CC_StopGrabbing()
        if ret != 0:
            print("stop grabbing fail! ret[0x%x]" % ret)

        # ch:关闭设备 | Close device
        ret = self.cam.MV_CC_CloseDevice()
        if ret != 0:
            print("close deivce fail! ret[0x%x]" % ret)

        # ch:销毁句柄 | Destroy handle
        ret = self.cam.MV_CC_DestroyHandle()
        if ret != 0:
            print("destroy handle fail! ret[0x%x]" % ret)

        pass

    def getFrame(self):
        stOutFrame = MV_FRAME_OUT()
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))
        ret = self.cam.MV_CC_GetImageBuffer(stOutFrame, 100)
        if ret != 0:
            print("get one frame fail! ret[0x%x]" % ret)
            return None
        # print(
        #     "get one frame: Width[%d], Height[%d], nFrameNum[%d]"
        #     % (
        #         stOutFrame.stFrameInfo.nWidth,
        #         stOutFrame.stFrameInfo.nHeight,
        #         stOutFrame.stFrameInfo.nFrameNum,
        #     )
        # )

        # 转换为bgr格式
        if PixelType_Gvsp_BayerRG12 == stOutFrame.stFrameInfo.enPixelType:
            nRGBSize = (
                stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3
            )
            stConvertParam = MV_CC_PIXEL_CONVERT_PARAM_EX()
            memset(byref(stConvertParam), 0, sizeof(stConvertParam))
            stConvertParam.nWidth = stOutFrame.stFrameInfo.nWidth
            stConvertParam.nHeight = stOutFrame.stFrameInfo.nHeight
            stConvertParam.pSrcData = stOutFrame.pBufAddr
            stConvertParam.nSrcDataLen = stOutFrame.stFrameInfo.nFrameLen
            stConvertParam.enSrcPixelType = stOutFrame.stFrameInfo.enPixelType
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed
            stConvertParam.pDstBuffer = (c_ubyte * nRGBSize)()
            stConvertParam.nDstBufferSize = nRGBSize

            ret = self.cam.MV_CC_ConvertPixelTypeEx(stConvertParam)

        else:
            print("Not Support")
            sys.exit()

        self.cam.MV_CC_FreeImageBuffer(stOutFrame)
        img_buff = (c_ubyte * stConvertParam.nDstLen)()
        memmove(byref(img_buff), stConvertParam.pDstBuffer, stConvertParam.nDstLen)
        frame_data = np.ctypeslib.as_array(img_buff)
        frame_data = frame_data.reshape(
            stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, 3
        )

        # 将图像旋转90度
        # frame_data = cv2.rotate(frame_data, cv2.ROTATE_90_CLOCKWISE)
        # print(frame_data.shape)
        return frame_data


if __name__ == "__main__":
    SDKVersion = MvCamera.MV_CC_GetSDKVersion()
    print("SDKVersion[0x%x]" % SDKVersion)
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("frame", 640, 480)
    cam1 = HKCam(0)
    while True:
        frame = cam1.getFrame()
        
        if frame is not None:
            cv2.imshow("frame", frame)
            if cv2.waitKey(13) & 0xFF == ord("q"):
                break
    # cam2 = HKCam(1)
    # cam2.getFrame()
