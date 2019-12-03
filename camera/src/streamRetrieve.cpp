#include <stdlib.h>
#include "StreamRetrieve.h"
#include "camParaConfig.h"
#include "shm.hpp"

int receive[3]={2, 0, 1};
//int Mode_last = 0;
//void mode_subCB(const std::vector<cv::Point2f> &data);
int last_mode = 0;
//流检索
StreamRetrieve::StreamRetrieve(ICameraPtr &cameraSptr,daHuaPara_str &daHuaPara,IStreamSourcePtr& streamSptr,ImgCallback imgCB,preProcessFunction preCB):
        _cameraSptr(cameraSptr),_daHuaPara(daHuaPara),m_streamSptr(streamSptr), imgCallback(imgCB), preProcess(preCB),Width(daHuaPara.imgWidth), Height(daHuaPara.imgHeight)
{
    pro_thread  = boost::thread(boost::bind(&StreamRetrieve::Proc, this));    //绑定函数，开辟新的线程
//    pro_thread2  = boost::thread(boost::bind(&StreamRetrieve::Proc2, this));    //绑定函数，开辟新的线程
}

void StreamRetrieve::join()
{
    pro_thread.join();               //ｐｒｏｃ线程
}

void StreamRetrieve::Proc()
{
    std::cout<<"camera runing..."<<std::endl;
    int frame_error_cnt=0;
//    static shm_publisher_create<int> mode_pub("car_mode",3);  //创建串口发布器共享内存
//    mode_pub.broadcast(receive);   //接受串口发布数据
//    static shm_subscriber<int> mode_sub("car_mode");   //订阅器共享内存
    while (1)
    {
//        mode_sub.get(receive);                      //订阅数据
//
//        if(receive[0]!=last_mode)
//        {
//            cout<<"mode change"<<endl;
//            if(receive[0]==2){
//                ArmorModelParaConfig_runtime(_cameraSptr, _daHuaPara);  //默认进入此模式
//            } else if(receive[0]==1){
//                RuneModelParaConfig_runtime(_cameraSptr, _daHuaPara);
//            }
//        }
//        last_mode = receive[0];

        CFrame frame;
        if (!m_streamSptr)
        {
            printf("m_streamPtr is NULL.\n");
            return;
        }
        bool isSuccess = m_streamSptr->getFrame(frame, 350);
        if (!isSuccess)
        {
            frame_error_cnt++;
            if(frame_error_cnt>51)
            {
                printf("\nFrame fail, maybe the USB connecttion fail\n");
                return;
            }else{
                printf("getFrame fail.\n");
                continue;
            }
        } else
            frame_error_cnt=0;

        bool isValid = frame.valid();
        if (!isValid)
        {
            frame_error_cnt++;
            if(frame_error_cnt>2000){
                printf("\nframe invalid, and return...\n");
                return;
            }else{
                printf("frame is invalid!\n");
                continue;
            }
        } else
            frame_error_cnt=0;

        //读取彩色图
        uint8_t *pRGBbuffer = NULL;
        int nRgbBufferSize = 0;
        nRgbBufferSize = frame.getImageHeight() * frame.getImageWidth() * 3;
        pRGBbuffer = (uint8_t *)malloc(nRgbBufferSize);

        if (pRGBbuffer == NULL)
        {
            printf("RGBbuffer malloc failed.\n");
            free(pRGBbuffer);
            continue;
        }

        IMGCNV_SOpenParam openParam;
        openParam.width = frame.getImageWidth();
        openParam.height = frame.getImageHeight();
        openParam.paddingX = frame.getImagePadddingX();
        openParam.paddingY = frame.getImagePadddingY();
        openParam.dataSize = frame.getImageSize();
        openParam.pixelForamt = gvspPixelBayRG8;

        unsigned char * pRgbFrameBuf=new(std::nothrow)  unsigned char[Width*Height*1];

        memcpy(pRgbFrameBuf, frame.getImage(), frame.getImageSize());

        IMGCNV_EErr status = IMGCNV_ConvertToBGR24(pRgbFrameBuf, &openParam, pRGBbuffer, &nRgbBufferSize);
        if (IMGCNV_SUCCESS != status)
        {
            printf("IMGCNV_ConvertToBGR24 failed.\n");

            delete pRgbFrameBuf;
            return;
        }
        delete pRgbFrameBuf;

        cv::Mat pre_out;
        cv::Mat _mat(frame.getImageHeight(),frame.getImageWidth(),CV_8UC3,pRGBbuffer,frame.getImageWidth()*3);

        preProcess(_mat,pre_out);
        imgCallback(pre_out);

        free(pRGBbuffer);
    }
}
