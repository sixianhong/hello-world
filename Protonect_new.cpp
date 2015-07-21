/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <iostream>
#include <signal.h>
#include <opencv2/opencv.hpp>

#include <XKin/libbody.h>
#include <XKin/libhand.h>
#include <XKin/libgesture.h>
#include <XKin/libposture.h>

#include <UI/libui.h>
#include "portal/Framework2/App.h"
#include "portal/Framework2/ParticleSystem.h"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

//#include <libfreenect2/libfreenect_cv.h>
#include <cmath>
#include <time.h>
using namespace std;
bool protonect_shutdown = false;

int posture;

libfreenect2::mutex brush_mutex;

extern App *app;
extern float frameTime;

COLOR_SCHEME colorType = COLOR_SCHEME_FIRE;

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

/////////////////////////////////
/*            test             */
#define WT CV_GUI_NORMAL | CV_WINDOW_AUTOSIZE

enum {
        W=1920,
        H=1080,
        T=10,
        N=5
};

char *infile = NULL;

int        load_posture_models      (char*, CvPostModel**);
void       usage                    (void);
void       parse_args               (int,char**);
/////////////////////////////////////

IplImage *freenect_sync_get_depth_cv(int index, unsigned char* depthData)
{
  static IplImage *image = 0;
  static unsigned char *data = 0;
  if (!image) image = cvCreateImageHeader(cvSize(512,424), IPL_DEPTH_32F, 1);
  unsigned int timestamp;
  // if (freenect_sync_get_depth((void**)&data, &timestamp, index, FREENECT_DEPTH_11BIT))
  //  return NULL;
  data = depthData;
  cvSetData(image, data, 512*4);

  /****************Get a portion of the image**************/
  static IplImage *subimg = 0;
  cvSetImageROI(image, cvRect(75, 32, 362, 350));
  cvCopy(image, subimg);
  cvResetImageROI(image);
  return subimg;
}

IplImage *freenect_sync_get_rgb_cv(int index, unsigned char* rgbData)
{
  static IplImage *image = 0;
  static unsigned char *data = 0;
  if (!image) image = cvCreateImageHeader(cvSize(1920,1080), IPL_DEPTH_8U, 3);
  
  unsigned int timestamp;
  // if (freenect_sync_get_depth((void**)&data, &timestamp, index, FREENECT_DEPTH_11BIT))
  //  return NULL;
  data = rgbData;
  cvSetData(image, data, 1920*3);
  
  return image;
}

int main(int argc, char *argv[])
{

  libfreenect2::thread UI_thread(run_main, NULL);

  std::string program_path(argv[0]);
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";

  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }

  std::string serial = freenect2.getDefaultDeviceSerialNumber();

   
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
      std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
   

  if(pipeline)
  {
    dev = freenect2.openDevice(serial, pipeline);
  }
  else
  {
    dev = freenect2.openDevice(serial);
  }

  if(dev == 0)
  {
    std::cout << "failure opening device!" << std::endl;
    return -1;
  }

  signal(SIGINT,sigint_handler);
  protonect_shutdown = false;

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  unsigned char* registered = NULL;

    ///////////////////////////////////
  /*            test               */
  IplImage *rgb_test, *depth_test, *number;
  CvPostModel *models;
  int num, count=0;
  char w1[] = "rgb";
  char w2[] = "number";
  char buff[5];
  CvFont font[N];
  CvScalar color[N];

  parse_args(argc,argv);
  num = load_posture_models(infile, &models);
  rgb_test = cvCreateImage(cvSize(W,H), 8, 3);
  // number = cvCreateImage(cvSize(256,256), 8, 3);
  color[0] = CV_RGB(0,0,255);
  color[1] = CV_RGB(0,255,0);
  color[2] = CV_RGB(255,0,0);
  color[3] = CV_RGB(255,0,255);
  color[4] = CV_RGB(0,255,255);

  cvNamedWindow(w1, WT);
  // cvNamedWindow(w2, WT);
  //////////////////////////////////
  int curX = 250, curZ = 150;
  int oldX = 250, oldY = 200, oldZ = 50;
  int deltaZ = 1, deltaX = 1, thresholdX = 90, thresholdY = 70, thresholdZ = 5, thresholdZ1 = 1;
  bool flag = false;
  int centX = 250, centY = 200, centZ = 38;
  time_t startTime = 0, curTime = 0;
  double colorTime = 0;

  while(!protonect_shutdown)
  {    
    IplImage *tmp;
    listener.waitForNewFrame(frames);
    // libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    // libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    // std::cout<<"?1"<<std::endl;

    //cv::imshow("rgb", cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data));
    //cv::imshow("ir", cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f);
    // cv::Mat depthMatrix = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f;
    IplImage *ptr = freenect_sync_get_depth_cv(0,depth->data);
    // tmp = freenect_sync_get_rgb_cv(0, rgb->data);
    // cvCvtColor(tmp, rgb_test, CV_BGR2RGB);

    IplImage* body = body_detection(ptr);
    if (body == NULL) {
      std::cout<<"error body detection"<<std::endl;
      listener.release(frames);
      continue;
    }
    IplImage *a, *b;
    CvMat *fd;
    CvSeq *cnt;
    CvPoint cent;
    ptseq seq;
    int z, k, c; 
    int idx=0, zoom=0, update=1;

    IplImage* hand = hand_detection(body, &z);

    //if (!get_hand_contour_advanced(hand, rgb_test, z, &cnt, &cent))
    if (!get_hand_contour_basic(hand, &cnt, &cent))
    {
      std::cout<<"continue hand contour" << std::endl;
      listener.release(frames);
      continue;
    }

    if ((posture = advanced_posture_classification(cnt, models, num)) == -1){
      std::cout<<"continue"<<std::endl;
      listener.release(frames);
      continue;
    }

    cvShowImage("hand", hand);
    
    int newX = cent.x;
    int newY = cent.y;
    int newZ = z;

    if (posture == 1) {
      startTime = 0;
      colorTime = 0;
      curTime = 0;

    /**************     control fire    ******************/
    // increse fps for Protonect to solve the problem that cannot draw the curve
    // we think the speed of Protonect is mainly restricted by openCV calculation in body_detection
    // solution: implement openCV using openCL
      vec3 temp((newX-oldX)/4, (newY-oldY)/3, newZ-oldZ);
      app->fire_pos = temp;
      oldX = newX;
      oldY = newY;
      oldZ = newZ;
    }
    else if (posture == 0) {
      startTime = 0;
      colorTime = 0;
      curTime = 0;

      /*************    control camera    ************/
      // works fine
      // can go up/down, left/right, forward/backward
      // act as pressing keyboard
      float sqrLen;
      vec3 dir(0,0,0);

      mat4 modelView = app->get();

      vec3 dx(modelView.elem[0][0], modelView.elem[0][1], modelView.elem[0][2]);
      vec3 dy(modelView.elem[1][0], modelView.elem[1][1], modelView.elem[1][2]);
      vec3 dz(modelView.elem[2][0], modelView.elem[2][1], modelView.elem[2][2]);

      // the forward/backward direction cannot move at the same time with 
      // up/down, and left/right direction
      bool flag = false;
      if (centX - newX > thresholdX) {dir -= dx; flag = true;}
      if (newX - centX > thresholdX) {dir += dx; flag = true;}

      if (centY - newY > thresholdY) {dir += dy; flag = true;}
      if (newY - centY > thresholdY) {dir -= dy; flag = true;} 

      if (!flag) {
        if (centZ - newZ > thresholdZ) dir += dz;
        if (newZ - centZ > thresholdZ1) dir -= dz;
      }

      if ((sqrLen = dot(dir, dir)) != 0){
        dir *= 1.0f / sqrtf(sqrLen);
      }

      app->direction = frameTime * 256 * dir;
      
    }
    else if (posture == 2) {
      /*************    change color    ************/
      // works fine
      // can change the color of the fire if the posture 
      // stands for more than a second
      if (startTime == 0) {
        time(&startTime);
      }
      else {
        time(&curTime);
        colorTime = difftime(curTime, startTime);
        if (colorTime >= 1) {
          colorType = (COLOR_SCHEME)((colorType + 1) % 4);
                  std::cout<<"color: "<<colorType<<std::endl;
          startTime = 0;
          colorTime = 0;
          curTime = 0;
        }
      }
    }

    if ((k = cvWaitKey(T)) == 1048689)
      break;


    int key = cv::waitKey(1);
    protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

    listener.release(frames);
    //libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
  }

  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
  dev->stop();
  dev->close();

  delete[] registered;
  delete registration;

  return 0;
}

/////////////////////////////////
/*        test          */
int load_posture_models (char *infile, CvPostModel **p)
{
  CvFileStorage *fs;
  CvFileNode *node;
  CvMat *tmp;
  char name[] = "posture-00";
  int i, total;

  fs = cvOpenFileStorage(infile, NULL, CV_STORAGE_READ, NULL);
  assert(fs);
  total = cvReadIntByName(fs, NULL, "total", 0);
  *p = (CvPostModel*)malloc(sizeof(CvPostModel)*total);

  for (i=0; i<total; i++) {
    node = cvGetFileNodeByName(fs, NULL, name);
    tmp = (CvMat*)cvReadByName(fs, node, "mean", NULL);
    (*p)[i].mean = (CvMat*)cvClone(tmp);
    tmp = (CvMat*)cvReadByName(fs, node, "cov", NULL);
    (*p)[i].cov = (CvMat*)cvClone(tmp);

    if (++name[9] == ':') {
      name[9] = '0';
      name[8]++;
    }
  }

  cvReleaseFileStorage(&fs);

  return total;
}

void parse_args (int argc, char **argv)
{
  int c;

  opterr=0;
  while ((c = getopt(argc, argv, "i:h")) != -1) {
    switch (c) {
    case 'i':
      infile = optarg;
      cout<<infile<<endl;
      break;
    case 'h':
    default:
      usage();
      exit(-1);
    }
  }
  if (infile == NULL) {
    usage();
    exit(-1);
  }
}

void usage (void)
{
  printf("usage: testposture -i [file] [-h]\n");
  printf("  -i  posture models file\n");
  printf("  -h  show this message\n");
}
