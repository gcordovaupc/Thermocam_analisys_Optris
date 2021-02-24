#include <stdio.h>
#include <string.h>
#include <iostream>
#include <pthread.h>
#include <sys/time.h>
#include <signal.h>

// Optris device interface
#include "IRDevice.h"

// Optris imager interface
#include "IRImager.h"

// Optris logging interface
#include "IRLogger.h"

// Optris image converter
#include "ImageBuilder.h"

// Visualization
#include "Obvious2D.h"

using namespace std;
using namespace evo;

IRDeviceParams _params1;
IRDeviceParams _params2;

EnumOptrisColoringPalette _palette = eIron;

struct thread_context
{
  pthread_mutex_t mutex;
  pthread_cond_t  available;
  IRImager*       imager;
  IRDevice*       device;
  unsigned short* thermal;
};

bool _shutdown = false;

void onThermalFrame(unsigned short* thermal, unsigned int w, unsigned int h, IRFrameMetadata meta, void* arg)
{
  thread_context* context = (thread_context*)arg;
  pthread_mutex_lock( &(context->mutex) );
  if(context->thermal==NULL) context->thermal = new unsigned short[w*h];
  if(context->imager->isFlagOpen())
  {
    memcpy(context->thermal, thermal, w*h*sizeof(*thermal));
    pthread_cond_signal( &(context->available) );
  }
  pthread_mutex_unlock( &(context->mutex) );
}

void* camWorker(void* arg)
{
  thread_context* context = (thread_context*)arg;
  IRImager* imager = context->imager;
  IRDevice* device = context->device;
  unsigned char* bufferRaw = new unsigned char[device->getRawBufferSize()];

  // -----------------------------------------------------------
  // Enter endless loop in order to pass raw data to Optris image processing library.
  // Processed data are provided by the frame callback function.
  // -----------------------------------------------------------
  while(!_shutdown)
  {
    if(device->getFrame(bufferRaw)==IRIMAGER_SUCCESS)
    {
      imager->process(bufferRaw, context);
    }
  }
  // -----------------------------------------------------------

  delete [] bufferRaw;

  pthread_exit(NULL);
  return NULL;
}

void* camWorker2(void* arg)
{
  thread_context* context = (thread_context*)arg;
  IRImager* imager = context->imager;
  IRDevice* device = context->device;
  unsigned char* bufferRaw = new unsigned char[device->getRawBufferSize()];

  // -----------------------------------------------------------
  // Enter endless loop in order to pass raw data to Optris image processing library.
  // Processed data are provided by the frame callback function.
  // -----------------------------------------------------------
  while(!_shutdown)
  {
    if(device->getFrame(bufferRaw)==IRIMAGER_SUCCESS)
    {
      imager->process(bufferRaw, context);
    }
  }
  // -----------------------------------------------------------

  delete [] bufferRaw;

  pthread_exit(NULL);
  return NULL;
}

int main (int argc, char* argv[])
{
  if(argc!=3)
  {
    cout << "usage: " << argv[0] << " <1st xml configuration file> <2nd xml configuration file>" << endl;
    return -1;
  }

  // -----------------------------------------------------------
  // Initialize Optris imager driver and image processing chain
  // -----------------------------------------------------------
  IRLogger::setVerbosity(IRLOG_ERROR, IRLOG_OFF);

  if(!IRDeviceParamsReader::readXML(argv[1], _params1))
    return -1;
  if(!IRDeviceParamsReader::readXML(argv[2], _params2))
      return -1;

  IRDevice* dev1 = NULL;

  dev1 = IRDevice::IRCreateDevice(_params1);

  if(!dev1)
  {
    cout << "Error: Instantiating 1st device with serial " << _params1.serial << endl;
    return -1;
  }

  IRImager imager1;
  imager1.init(&_params1, dev1->getFrequency(), dev1->getWidth(), dev1->getHeight(), dev1->controlledViaHID());

  unsigned int w1 = imager1.getWidth();
  unsigned int h1 = imager1.getHeight();
  if(w1==0 || h1==0)
  {
    cout << "Error: Image streams of 1st camera not available or wrongly configured. Check connection of camera and config file." << endl;
    return -1;
  }
  imager1.setThermalFrameCallback(onThermalFrame);
  cout << "1st connected camera, serial: " << dev1->getSerial() << ", HW(Rev.): " << imager1.getHWRevision() << ", FW(Rev.): " << imager1.getFWRevision() << endl;
  cout << "Thermal channel: " << imager1.getWidth() << "x" << imager1.getHeight() << "@" << dev1->getFrequency() << "Hz" << endl;


  IRDevice* dev2 = NULL;

  dev2 = IRDevice::IRCreateDevice(_params2);

  if(!dev2)
  {
    cout << "Error: Instantiating 2nd device with serial " << _params2.serial << endl;
    if(dev1)
      delete dev1;
    return -1;
  }

  IRImager imager2;
  imager2.init(&_params2, dev2->getFrequency(), dev2->getWidth(), dev2->getHeight(), dev2->controlledViaHID());

  unsigned int w2 = imager2.getWidth();
  unsigned int h2 = imager2.getHeight();
  if(w2==0 || h2==0)
  {
    cout << "Error: Image streams of 2nd camera not available or wrongly configured. Check connection of camera and config file." << endl;
    return -1;
  }
  imager2.setThermalFrameCallback(onThermalFrame);
  cout << "2nd connected camera, serial: " << dev2->getSerial() << ", HW(Rev.): " << imager2.getHWRevision() << ", FW(Rev.): " << imager2.getFWRevision() << endl;
  cout << "Thermal channel: " << imager2.getWidth() << "x" << imager2.getHeight() << "@" << dev2->getFrequency() << "Hz" << endl;
  // -----------------------------------------------------------

  if(dev1->startStreaming()!=0)
  {
    cout << "Error occurred in starting stream from 1st camera ... aborting. You may need to reconnect the camera." << endl;
    exit(-1);
  }

  if(dev2->startStreaming()!=0)
  {
    cout << "Error occurred in starting stream from 2nd camera ... aborting. You may need to reconnect the camera." << endl;
    exit(-1);
  }

  pthread_t th1;
  thread_context context1 = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, &imager1, dev1, NULL};
  pthread_create( &th1, NULL, camWorker, &context1);

  pthread_t th2;
  thread_context context2 = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, &imager2, dev2, NULL};
  pthread_create( &th2, NULL, camWorker2, &context2);


  // -----------------------------------------------------------
  // Set up two OpenGL viewer for the separate data streams
  // -----------------------------------------------------------
  Obvious2D viewer(w1, h1, "Optris Twin Imager Example");
  Obvious2D viewer2(w2, h2, "Optris Twin Imager Example");

  ImageBuilder iBuilder1(true, imager1.getTemprangeDecimal());
  iBuilder1.setPaletteScalingMethod(eMinMax);
  iBuilder1.setManualTemperatureRange(15.0f, 40.0f);

  ImageBuilder iBuilder2(true, imager2.getTemprangeDecimal());
  iBuilder2.setPaletteScalingMethod(eMinMax);
  iBuilder2.setManualTemperatureRange(15.0f, 40.0f);

  unsigned char* bufferThermal  = NULL;
  unsigned char* bufferThermal2  = NULL;
  // -----------------------------------------------------------


  // -----------------------------------------------------------
  // Display loop
  // -----------------------------------------------------------
  while(viewer.isAlive() && viewer2.isAlive() && !_shutdown)
  {

    pthread_mutex_lock( &(context1.mutex) );
    pthread_cond_wait( &(context1.available), &(context1.mutex) );

    iBuilder1.setData(w1, h1, context1.thermal);
    if(bufferThermal==NULL)
      bufferThermal = new unsigned char[iBuilder1.getStride() * h1 * 3];

    iBuilder1.convertTemperatureToPaletteImage(bufferThermal);
    pthread_mutex_unlock( &(context1.mutex) );

    viewer.draw(bufferThermal, iBuilder1.getStride(), h1, 3);

    pthread_mutex_lock( &(context2.mutex) );
    pthread_cond_wait( &(context2.available), &(context2.mutex) );

    iBuilder2.setData(w2, h2, context2.thermal);
    if(bufferThermal2==NULL)
      bufferThermal2 = new unsigned char[iBuilder2.getStride() * h2 * 3];

    iBuilder2.convertTemperatureToPaletteImage(bufferThermal2);
    pthread_mutex_unlock( &(context2.mutex) );

    viewer2.draw(bufferThermal2, iBuilder2.getStride(), h2, 3);
  }
  // -----------------------------------------------------------

  _shutdown = true;
  if(bufferThermal) delete [] bufferThermal;
  if(bufferThermal2) delete [] bufferThermal2;

  pthread_mutex_lock( &(context1.mutex) );
  pthread_cond_signal( &(context1.available) );
  pthread_mutex_unlock( &(context1.mutex) );
  pthread_join(th1, NULL);

  pthread_mutex_lock( &(context2.mutex) );
  pthread_cond_signal( &(context2.available) );
  pthread_mutex_unlock( &(context2.mutex) );
  pthread_join(th2, NULL);

  dev1->stopStreaming();
  dev2->stopStreaming();

  delete dev1;
  delete dev2;

  cout << "Exiting application" << endl;

  raise(SIGTERM);

  return 1;
}
