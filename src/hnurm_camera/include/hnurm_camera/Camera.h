#ifndef HKCAM
#define HKCAM

#include "hnurm_camera/MvCameraControl.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

namespace hnurm
{
class HKcam
{

public:
    explicit HKcam(const std::shared_ptr<rclcpp::Node> &node);

    ~HKcam();

    void init_params();

    bool OpenCam(const std::string &cameraId = "");

    bool CloseCam();

    bool GetFrame(std::vector<uint8_t> &img);

    void SetParam();

    void prepare_convert(uint8_t *dstData);

private:
    // state num
    int nRet;

    // handle for manipulating the Camera
    void *handle;

    // camera param
    MVCC_INTVALUE stParam {};

    // frame ptr
    unsigned char *pData;

    // format of frame ,read from camera
    MV_FRAME_OUT_INFO_EX         stImageInfo {};
    MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam {};

    rclcpp::Node::SharedPtr _node;
    rclcpp::Logger          _logger;

    std::string _id = "none";

public:
    int   _nImageOrientation   = 0;
    int   _nWidth              = 1'280;
    int   _nHeight             = 720;
    int   _nOffsetX            = 0;
    int   _nOffsetY            = 152;
    bool  _bReverseX           = false;
    bool  _bReverseY           = false;
    int   _nPixelFormat        = 0x02180015;
    int   _nAcqFrameRate       = 30;
    float _fFPS                = 404.0;
    bool  _bEnableAcqFrameRate = false;
    float _fExposureTime       = 2'000;
    float _fGain               = 8;
    int   _nBlackLevelValue    = 30;
    bool  _bEnableBlackLevel   = false;
    int   _nBayerCvtQuality    = 3;

};  // HKcam

}  // hnurm
#endif
