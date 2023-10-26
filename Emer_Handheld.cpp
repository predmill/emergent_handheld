#include <iostream>
#include <sstream>
#include "stdafx.h"
#include <stdio.h>
#include <string.h>
#include <boost/filesystem.hpp>   
#ifdef _MSC_VER
  #include <windows.h>
#else
  #include <unistd.h>
#endif
#include <EmergentCameraAPIs.h>
#include <emergentframe.h>
#include <EvtParamAttribute.h>
#include <gigevisiondeviceinfo.h>
#include <EmergentFrameSave.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace std;
using namespace Emergent;
using namespace boost::filesystem; 
using namespace cv;

#define SUCCESS 0
//#define XML_FILE   "C:\\xml\\Emergent_HS-20000-C_1_0.xml"
#define MAX_FRAMES 7200000

#define POLARITY_NEG 1
#define POLARITY_POS 0

#define TRUE  1
#define FALSE 0

#define EXPOSURE_US 10000 //us

#define MAX_CAMERAS 2

#define CAPTURE_TIMEOUT 3000 //3ms

string desiredSNl = "";
string desiredSNr = "";

void configure_defaults(CEmergentCamera* camera);
char* next_token;

int main(int argc, char* argv[])
{

  
  if ( !exists( argv[1] ) )
  	create_directory(argv[1]);
  
  desiredSNl = argv[2];
  desiredSNr = argv[3];
    	
  CEmergentCamera camera, camera_l, camera_r;
  int ReturnVal = SUCCESS;
  int ReturnVal_l = SUCCESS;
  int ReturnVal_r = SUCCESS;
  
  CEmergentFrame evtFrame, evtFrame_l, evtFrame_r;
  unsigned int height_max, width_max, height_max_l, width_max_l, height_max_r, width_max_r, param_val_max, param_val_min;
  bool gpo_polarity = TRUE;
  char gpo_str[20];
  char filename[100];
  struct GigEVisionDeviceInfo deviceInfo[MAX_CAMERAS];
  unsigned int count, camera_index;
  EVT_ERROR err = EVT_SUCCESS;

  //Find all cameras in system.
  unsigned int listcam_buf_size = MAX_CAMERAS;
  int cameras_found = 0;
  
  EVT_ListDevices(deviceInfo, &listcam_buf_size, &count);
  if(count==0)
  {
  	printf("Enumerate Cameras: \tNo cameras found. Exiting program.\n");
	return 0;
  }

  //Find EVT camera.
  for(camera_index=0; camera_index<MAX_CAMERAS;camera_index++)
  {
	char* EVT_models[] = { "HS", "HT", "HR", "HB", "LR", "LB", "HZ" };
	int EVT_models_count = sizeof(EVT_models) / sizeof(EVT_models[0]);
	bool is_EVT_camera = false;
    
    	for(int i = 0; i < EVT_models_count; i++)
    	{
		if(strncmp(deviceInfo[camera_index].modelName, EVT_models[i], 2) == 0)
		{
			is_EVT_camera = true;
			break; //it is an EVT camera
		}	
	}
	if (is_EVT_camera)
		cameras_found++;
  }  
      
  if(cameras_found < 2)
  {
	printf("No or less than two EVT cameras found. Exiting program\n");
	return 0;
  }

  cout<<"camera found: "<<cameras_found<<endl;

  int desired_lcam_index = -1;
  int desired_rcam_index = -1;


  for (int i = 0; i < cameras_found; i++)
  {
	if (strcmp(deviceInfo[i].serialNumber, desiredSNl.c_str()) == 0)
	{
            desired_lcam_index = i;
            cout<<"Left Camera index:   "<<desired_lcam_index<<"  Serial Number:   "<< deviceInfo[i].serialNumber<<endl;
        }
        if (strcmp(deviceInfo[i].serialNumber, desiredSNr.c_str()) == 0)
	{
            desired_rcam_index = i;
            cout<<"right Camera index:  "<<desired_rcam_index<<"  Serial Number:   "<< deviceInfo[i].serialNumber<<endl;
        }
  }
  
    
  if ((desired_lcam_index == -1) | (desired_rcam_index == -1)) 
  {
	printf("Could not find desired serial number, defaulting to first camera found");
	return 0;
  }

 
  //Open the camera. Example usage. Camera found needs to match XML.
    
#ifdef XML_FILE
  ReturnVal_l = EVT_CameraOpen(&camera_l, &deviceInfo[desired_lcam_index], XML_FILE);
  ReturnVal_r = EVT_CameraOpen(&camera_r, &deviceInfo[desired_rcam_index], XML_FILE);
#else
  ReturnVal_l = EVT_CameraOpen(&camera_l, &deviceInfo[desired_lcam_index]); 
  ReturnVal_r = EVT_CameraOpen(&camera_r, &deviceInfo[desired_rcam_index]);     
#endif

  if((ReturnVal_l == SUCCESS) & (ReturnVal_r == SUCCESS))
  {
    printf("Open both Cameras: \t\tCameras Opened\n\n");
  }
  else
  {
    if(ReturnVal_l != SUCCESS)
    	printf("Open left Camera fail: \t\t%d\n", ReturnVal_l);
    if(ReturnVal_r != SUCCESS)
    	printf("Open right Camera fail: \t\t%d\n", ReturnVal_r);
    	
    printf("Exiting Program\n");
    return 1;
  }

  //To avoid conflict with settings in other examples.
  EVT_CameraSetBoolParam(&camera_l, "HBDisable", false);
  EVT_CameraSetBoolParam(&camera_r, "HBDisable", false);
  configure_defaults(&camera_l);
  configure_defaults(&camera_r);

  //Get resolution.
  EVT_CameraGetUInt32ParamMax(&camera_l, "Height", &height_max_l);
  EVT_CameraGetUInt32ParamMax(&camera_l, "Width" , &width_max_l);
  printf("Left Camera Resolution: \t\t%d x %d\n", width_max_l, height_max_l);
  
  EVT_CameraGetUInt32ParamMax(&camera_r, "Height", &height_max_r);
  EVT_CameraGetUInt32ParamMax(&camera_r, "Width" , &width_max_r);
  printf("Right Camera Resolution: \t\t%d x %d\n", width_max_r, height_max_r);
  
  //Setup continuous software trigger for MAX_FRAMES frame.
  
  EVT_CameraSetEnumParam(&camera_l, "AcquisitionMode", "Continue");
  EVT_CameraSetUInt32Param(&camera_l, "AcquisitionFrameCount", 1);
  EVT_CameraSetEnumParam(&camera_l, "TriggerSelector", "AcquisitionStart");
  EVT_CameraSetEnumParam(&camera_l, "TriggerMode", "On");
  EVT_CameraSetEnumParam(&camera_l, "TriggerSource", "Software");
  EVT_CameraSetEnumParam(&camera_l, "BufferMode", "Off");
  EVT_CameraSetUInt32Param(&camera_l, "BufferNum", 0);
  
  EVT_CameraSetEnumParam(&camera_r, "AcquisitionMode", "Continue");
  EVT_CameraSetUInt32Param(&camera_r, "AcquisitionFrameCount", 1);
  EVT_CameraSetEnumParam(&camera_r, "TriggerSelector", "AcquisitionStart");
  EVT_CameraSetEnumParam(&camera_r, "TriggerMode", "On");
  EVT_CameraSetEnumParam(&camera_r, "TriggerSource", "Software");
  EVT_CameraSetEnumParam(&camera_r, "BufferMode", "Off");
  EVT_CameraSetUInt32Param(&camera_r, "BufferNum", 0);

  //Prepare host side for streaming.
  ReturnVal = EVT_CameraOpenStream(&camera_l);
  if(ReturnVal != SUCCESS)
  {
    printf("Left Camera OpenStream: Error\t\t%d\n", ReturnVal);
    return ReturnVal;
  }
  
  ReturnVal = EVT_CameraOpenStream(&camera_r);
  if(ReturnVal != SUCCESS)
  {
    printf("Right Camera OpenStream: Error\n");
    return ReturnVal;
  }

  // Tell camera to start streaming
  ReturnVal_l = EVT_CameraExecuteCommand(&camera_l, "AcquisitionStart");
  if (ReturnVal_l != SUCCESS)
  {
        printf("Left Camera EVT_CameraExecuteCommand: Error\n");
        return ReturnVal_l;
  }
  
  ReturnVal_r = EVT_CameraExecuteCommand(&camera_r, "AcquisitionStart");
  if (ReturnVal_r != SUCCESS)
  {
        printf("Right Camera EVT_CameraExecuteCommand: Error\n");
        return ReturnVal_r;
  }
  
  //Allocate buffer and queue up frame before entering grab loop.

  //Three params used for memory allocation. Worst case covers all models so no recompilation required.
  evtFrame_l.size_x = width_max_l;
  evtFrame_l.size_y = height_max_l;
  evtFrame_l.pixel_type = GVSP_PIX_MONO8;  //Covers color model using BayerGB8 also.
  EVT_AllocateFrameBuffer(&camera_l, &evtFrame_l, EVT_FRAME_BUFFER_ZERO_COPY);
  EVT_CameraQueueFrame(&camera_l, &evtFrame_l);
  
  evtFrame_r.size_x = width_max_r;
  evtFrame_r.size_y = height_max_r;
  evtFrame_r.pixel_type = GVSP_PIX_MONO8;  //Covers color model using BayerGB8 also.
  EVT_AllocateFrameBuffer(&camera_r, &evtFrame_r, EVT_FRAME_BUFFER_ZERO_COPY);
  EVT_CameraQueueFrame(&camera_r, &evtFrame_r);

  EVT_CameraSetUInt32Param(&camera_l, "Exposure" , EXPOSURE_US);
  EVT_AllocateFrameBuffer(&camera_l, &evtFrame_l, EVT_FRAME_BUFFER_ZERO_COPY);
  
  EVT_CameraSetUInt32Param(&camera_r, "Exposure" , EXPOSURE_US);
  EVT_AllocateFrameBuffer(&camera_r, &evtFrame_r, EVT_FRAME_BUFFER_ZERO_COPY);
  
  std::vector<cv::Mat> images_l, images_r, images_lr;
    
  for(int i=0; i < MAX_FRAMES; i++)
  {
	auto errCode = EVT_CameraQueueFrame(&camera_l, &evtFrame_l);
	if (errCode > 0)
	{
		std::cout << "The camera has errored out! Error code:" << errCode << std::endl;;
	}
	EVT_CameraQueueFrame(&camera_r, &evtFrame_r);

	EVT_CameraExecuteCommand(&camera_l, "TriggerSoftware");
	EVT_CameraExecuteCommand(&camera_r, "TriggerSoftware");		
	
	EVT_CameraGetFrame(&camera_l,&evtFrame_l, CAPTURE_TIMEOUT);
 	EVT_CameraGetFrame(&camera_r,&evtFrame_r, CAPTURE_TIMEOUT); 
 	 
 	
 	cv::Mat out_image_l, out_image_r, out_image_lr;
           
	int width_l = evtFrame_l.size_x;
	int height_l = evtFrame_l.size_y;
	
	int width_r = evtFrame_r.size_x;
	int height_r = evtFrame_r.size_y;
      
      	const unsigned int image_size_l = height_l*width_l;  
	const unsigned int image_size_r = height_r*width_r;  
	
	out_image_l.create(height_l, width_l, CV_8UC1);
	memcpy(out_image_l.data, evtFrame_l.imagePtr, image_size_l);
	
	out_image_r.create(height_r, width_r, CV_8UC1);
	memcpy(out_image_r.data, evtFrame_r.imagePtr, image_size_r);
	
		
	hconcat(out_image_l, out_image_r, out_image_lr);   
	resize(out_image_lr, out_image_lr, Size(out_image_lr.cols/2, out_image_lr.rows/2)); 
	namedWindow( "Display frame",WINDOW_AUTOSIZE);
	imshow("Display frame", out_image_lr);
	waitKey(10); 
		            	
	//images_l.push_back(out_image_l);
	//images_r.push_back(out_image_r);
	        
	//cout<<images_l.size()<<endl;       
	//cout<<images_r.size()<<endl; 
	
	cout<<"count :  " <<i<<endl;
 	
  }
  cv::destroyAllWindows();
/*  
  for(int j = 0; j < images_l.size(); j++)
  {
      cv::imwrite("L Camera" + std::to_string(j) + ".png", images_l[j]);
  }
  for(int k = 0; k < images_r.size(); k++)
  {
      cv::imwrite("R Camera" + std::to_string(k) + ".png", images_r[k]);
  }
*/  
  return(0);
} 

//A function to set all appropriate defaults so 
//running other examples does not require reconfiguration.
void configure_defaults(CEmergentCamera* camera)
{
  unsigned int width_max, height_max, param_val_max;
  const unsigned long enumBufferSize = 1000;
  unsigned long enumBufferSizeReturn = 0;
  char enumBuffer[enumBufferSize];

  //Order is important as param max/mins get updated.
  EVT_CameraGetEnumParamRange(camera, "PixelFormat", enumBuffer, enumBufferSize, &enumBufferSizeReturn);
  char* enumMember = strtok_s(enumBuffer, ",", &next_token);
  EVT_CameraSetEnumParam(camera,      "PixelFormat", enumMember);

  EVT_CameraSetUInt32Param(camera,    "FrameRate", 30);

  EVT_CameraSetUInt32Param(camera,    "OffsetX", 0);
  EVT_CameraSetUInt32Param(camera,    "OffsetY", 0);

  EVT_CameraGetUInt32ParamMax(camera, "Width", &width_max);
  EVT_CameraSetUInt32Param(camera,    "Width", width_max);

  EVT_CameraGetUInt32ParamMax(camera, "Height", &height_max);
  EVT_CameraSetUInt32Param(camera,    "Height", height_max);

  EVT_CameraSetEnumParam(camera,      "AcquisitionMode",        "Continuous");
  EVT_CameraSetUInt32Param(camera,    "AcquisitionFrameCount",  1);
  EVT_CameraSetEnumParam(camera,      "TriggerSelector",        "AcquisitionStart");
  EVT_CameraSetEnumParam(camera,      "TriggerMode",            "on");
  EVT_CameraSetEnumParam(camera,      "TriggerSource",          "Software");
  EVT_CameraSetEnumParam(camera,      "BufferMode",             "Off");
  EVT_CameraSetUInt32Param(camera,    "BufferNum",              0);

  EVT_CameraGetUInt32ParamMax(camera, "GevSCPSPacketSize", &param_val_max);
  EVT_CameraSetUInt32Param(camera,    "GevSCPSPacketSize", param_val_max);

  EVT_CameraSetUInt32Param(camera,    "Gain", 256);
  EVT_CameraSetUInt32Param(camera,    "Offset", 0);

  EVT_CameraSetBoolParam(camera,      "LUTEnable", FALSE);
  EVT_CameraSetBoolParam(camera,      "AutoGain", FALSE);

  EVT_CameraSetBoolParam(camera,     "UartEnable", FALSE);
}
