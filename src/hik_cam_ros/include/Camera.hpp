#ifndef CAMERA_H
#define CAMERA_H

#include <stdio.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include "ros/ros.h"
#include <map>


using namespace std;

namespace camera
{
    cv::Mat frame;
    bool frame_empty = false;
    pthread_mutex_t mutex;
	#define MAX_IMAGE_DATA_SIZE (4 * 3648 * 5472)

    enum CameraProperties
    {
        CAM_PROP_FRAMERATEEnable,
        CAM_PROP_FRAMERATE,
        CAM_PROP_BURSTFRAMECOUNT,
        CAM_PROP_HEIGHT,
        CAM_PROP_WIDTH,
        CAM_PROP_TRIGGER_MODE,
        CAM_PROP_TRIGGER_SOURCE,
        CAM_PROP_ACQUISITIONMODE
    };

    class Camera
    {
    public:
        Camera(ros::NodeHandle &node);
        ~Camera();
        static void *workthread(void *pUser);
        void ReadImg(cv::Mat &img);
        bool set(camera::CameraProperties type, float value);

    private:
        void *handle;
        pthread_t threadID;
        int nRet;
        int height;
        int width;
        int BrustFrameCount;
        bool FrameRateEnable;
        int FrameRate;
        int TriggerMode;
        int TriggerSource;
        int AcquisitionMode;
    };
    Camera::Camera(ros::NodeHandle &node)
    {
        handle = NULL;
        node.param("ori_img_width", width, 0);
        node.param("ori_img_height", height, 0);
        node.param("FrameRateEnable", FrameRateEnable, true);
        node.param("FrameRate", FrameRate, 80);
        node.param("BrustFrameCount", BrustFrameCount, 1);
        node.param("TriggerMode", TriggerMode, 1);
        node.param("TriggerSource", TriggerSource,7);
        // node.param("AcquisitionMode", AcquisitionMode, 2);

        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            cout << "MV_CC_EnumDevices fail" << endl;
        }
        map<string,int> names_map;


        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            string camera_name = reinterpret_cast<const char *>(stDeviceList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chUserDefinedName);
	        // for (int j = 0; j < stDeviceList.nDeviceNum-i-1; j++)
	        // {
	        // //USB相机，SpecialInfo.stGigEInfo就需要变成SpecialInfo.stUSBInfo，换成其他设备信息来排序
		    //     if(stDeviceList.pDeviceInfo[j]->SpecialInfo.stGigEInfo.chUserDefinedName < stDeviceList.pDeviceInfo[j+1]->SpecialInfo.stGigEInfo.chUserDefinedName)
		    //     {
			//         MV_CC_DEVICE_INFO* temp = stDeviceList.pDeviceInfo[j];
			//         stDeviceList.pDeviceInfo[j] = stDeviceList.pDeviceInfo[j+1];
			//         stDeviceList.pDeviceInfo[j+1]=temp;
		    //     }
	        // }
            names_map[camera_name] = i; 
            
        }
        
        cout << "input idx default: 1" << endl;
        string idx = "";
        cin >> idx;
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[names_map.find(idx)->second]);
        if (MV_OK != nRet)
        {
            cout << "create handel failed" << endl;
        }
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            cout << "open device error" << endl;
        }

        this->set(CAM_PROP_WIDTH, width);
        this->set(CAM_PROP_HEIGHT, height);
        // this->set(CAM_PROP_ACQUISITIONMODE, AcquisitionMode);
        this->set(CAM_PROP_FRAMERATEEnable, FrameRateEnable);
        if (FrameRateEnable)
            this->set(CAM_PROP_FRAMERATE, FrameRate);
        this->set(CAM_PROP_BURSTFRAMECOUNT, BrustFrameCount);
        this->set(CAM_PROP_TRIGGER_MODE, TriggerMode);
        this->set(CAM_PROP_TRIGGER_SOURCE, TriggerSource);
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
        if (MV_OK == nRet)
        {
            cout << "success set trigger" << endl;
        }
        cout << "start grapping" << endl;
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            cout << "grabbing error" << endl;
        }
        nRet = pthread_mutex_init(&mutex, NULL);
        if (nRet != 0)
        {
            cout << "thread create error" << endl;
            exit(-1);
        }
        nRet = pthread_create(&threadID, NULL, workthread, handle);
    }
    Camera::~Camera()
    {
        int nRet;
        pthread_join(threadID, NULL);
        nRet = MV_CC_StopGrabbing(handle);
        nRet = MV_CC_CloseDevice(handle);
        nRet = MV_CC_DestroyHandle(handle);
        pthread_mutex_destroy(&mutex);
    }

    bool Camera::set(camera::CameraProperties type, float value)
    {
        switch (type)
        {
        case CAM_PROP_WIDTH:
            nRet = MV_CC_SetIntValue(handle, "Width", value);
            if (nRet != MV_OK)
                cout << "width error" << endl;
            break;
        case CAM_PROP_HEIGHT:
            nRet = MV_CC_SetIntValue(handle, "Height", value);
            if (nRet != MV_OK)
                cout << "height error" << endl;
            break;
        case CAM_PROP_FRAMERATEEnable:
            nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", value);
            if (nRet != MV_OK)
                cout << "framerateenable  error" << endl;
            break;
        case CAM_PROP_FRAMERATE:
            nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", value);
            if (nRet != MV_OK)
                cout << "framerate error" << endl;
            break;
        case CAM_PROP_BURSTFRAMECOUNT:
            nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", value);
            if (nRet != MV_OK)
                cout << "BurstFrameCount error" << endl;
            break;
        case CAM_PROP_TRIGGER_MODE:
            nRet = MV_CC_SetEnumValue(handle, "TriggerMode", value);
            if (nRet != MV_OK)
                cout << "TriggerMode error" << endl;
            break;
        case CAM_PROP_TRIGGER_SOURCE:
            nRet = MV_CC_SetEnumValue(handle, "TriggerSource", value);
            if (nRet != MV_OK)
                cout << "TriggerSource error" << endl;
            break;
        case CAM_PROP_ACQUISITIONMODE:
            nRet = MV_CC_SetEnumValue(handle, "AcquisitionMode", 2);
            if (MV_OK != nRet)
            {
                cout << "acquisition error";
            }

        default:
            return 0;
        }
        return nRet;
    }

    void *Camera::workthread(void *pUser)
    {
        int nRet = 1;
        int empty_frame = 0;
        unsigned char *pData = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
        unsigned char *pDataForRGB = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);
        MVCC_INTVALUE stParam;
        double startTime;
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        if (NULL == pData)
        {
            cout << "pData is None" << endl;
            return NULL;
        }

        while (ros::ok())
        {
            startTime = static_cast<double>(cv::getTickCount());
            nRet = MV_CC_GetOneFrameTimeout(pUser, pData, MAX_IMAGE_DATA_SIZE, &stImageInfo, 1000);
            if (MV_OK != nRet)
            {
                cout << "failed to get frame" <<endl;
            }
            stConvertParam.nWidth = stImageInfo.nWidth;
            stConvertParam.nHeight = stImageInfo.nHeight;
            stConvertParam.pSrcData = pData;
            stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            stConvertParam.pDstBuffer = pDataForRGB;
            stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;
            nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);
            pthread_mutex_lock(&mutex);
            camera::frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForRGB).clone();
            pthread_mutex_unlock(&mutex);
            cout << "success get frame" << endl;
            double time = ((double)cv::getTickCount() - startTime) / cv::getTickFrequency();
            cout << "FPS:" << 1 / time << "\r";
        }
        free(pDataForRGB);
        free(pData);
        return 0;
    }
    void Camera::ReadImg(cv::Mat &img)
    {
        pthread_mutex_lock(&mutex);
        if (camera::frame.empty())
        {
            cout << "get frame empty" << endl;
        }
        img = camera::frame;
        pthread_mutex_unlock(&mutex);
    }

}
#endif
