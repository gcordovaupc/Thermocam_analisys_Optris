#include <stdio.h>
#include <string.h>
#include <iostream>
#include <pthread.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <queue>
#include "IRArray.h"

// IR device interfaces
#include "IRDevice.h"

// IR imager interfaces
#include "IRImager.h"

// Helper class for checking calibration files
#include "IRCalibrationManager.h"

// Logging interface
#include "IRLogger.h"

// Image converter
#include "ImageBuilder.h"

// Visualization
#include "Obvious2D.h"

// Time measurement
#include "Timer.h"

#include "Image.h"

using namespace std;
using namespace evo;

IRDeviceParams _params;
IRImager*      _imager = NULL;
ImageBuilder   _iBuilder;

double _elapsed            = 0.0;
int    _cntElapsed         = 0;
int    _cntFrames          = 0;

bool   _showVisibleChannel = false;
bool   _streaming          = true;
bool   _biSpectral         = false;
bool   _showHelp           = true;
bool   _showFPS            = true;
//bool   _showMean           = true; //CARG Add   
bool   _useMultiThreading  = false;
bool   _shutdown           = false;
bool   _takeSnapshot       = false;
EnumOptrisColoringPalette _palette = eIron;

pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;

// Threaded working function to display images
void* displayWorker(void* arg);

Image<unsigned char>*  _imgYUYV    = NULL;
Image<unsigned short>* _imgThermal = NULL; //<----

queue<Image<unsigned short>*> _qThermal;
queue<Image<unsigned char>*>  _qYUYV;

enum IRImagerState
{
    IRIMAGER_STATE_UNINITIALIZED = 0,
    IRIMAGER_STATE_ROAMING       = 1,
    IRIMAGER_STATE_ATTACHED      = 2,
    IRIMAGER_STATE_ACQUIRE       = 3
};

// Function called within process call of IRImager instance with thermal image as parameter.
// Keep this function free of heavy processing load. Otherwise the frame rate will drop down significantly for the control loop.
void onThermalFrame(unsigned short* thermal, unsigned int w, unsigned int h, IRFrameMetadata meta, void* arg)
{
  if(_showVisibleChannel || !_imager->isFlagOpen()) return;
  _imgThermal = new Image<unsigned short>(w, h, thermal);

  /*cout << meta.counterHW << endl;
  cout << "Hardware timestamp: " << (float)(meta.counterHW) * _imager->getHWInterval() << " s" << endl;
  cout << meta.tempBox << " " << meta.tempChip << " " << meta.tempFlag << endl;*/
}

// Function called within process call of IRImager instance with visible image as parameter.
// Keep this function free of heavy processing load. Otherwise the frame rate will drop down significantly for the control loop.
void onVisibleFrame(unsigned char* yuyv, unsigned int w, unsigned int h, IRFrameMetadata meta, void* arg)
{
  if(!_showVisibleChannel) return;
  _imgYUYV = new Image<unsigned char>(w, h, yuyv);
}

// Function called within process call of IRImager instance, every time the state of the shutter flag changes.
// The flag state changes either automatically or by calling the forceFlagEvent method of IRImager.
void onFlageStateChange(EnumFlagState fs, void* arg)
{
  cout << "Flag state: " << fs << endl;
}

// Function called either by PIF event or via software trigger (raiseSnapshotEvent)
void onThermalFrameEvent(unsigned short* thermal, unsigned short* energy, unsigned int w, unsigned int h, IRFrameMetadata meta, const IRArray<IREventData>& events, void* arg)
{
  if(_showVisibleChannel || !_imager->isFlagOpen()) return;
  _imgThermal = new Image<unsigned short>(w, h, thermal);
}

// Function called either by PIF event or via software trigger (raiseSnapshotEvent)
void onVisibleFrameEvent(unsigned char* yuyv, unsigned int w, unsigned int h, IRFrameMetadata meta, const IRArray<IREventData>& events, void* arg)
{
  if(!_showVisibleChannel) return;
  _imgYUYV = new Image<unsigned char>(w, h, yuyv);
}

// Function called within getFrame call of IRDeviceUVC instance.
// Keep this function free of heavy processing load. Otherwise the frame rate will drop down significantly for the control loop.
void onRawFrame(unsigned char* data, int len, IRDevice* dev)
{
  _cntFrames++;

  evo::Timer t;

  _imager->process(data, (void*)&_mutex);

  _elapsed += t.reset();
  _cntElapsed++;
}

void onProcessExit(void* arg)
{
  //----- Handle data after processing -----
  pthread_mutex_lock(&_mutex);
  if(_imgThermal)
  {
    _qThermal.push(_imgThermal);
  }

  if(_imgYUYV)
  {
    _qYUYV.push(_imgYUYV);
  }
  pthread_mutex_unlock(&_mutex);
  //----------------------------------------

  //---- clear data after processing ----- CARG: I think here I need to comment this
  _imgThermal = NULL;
  _imgYUYV    = NULL;
  //----------------------------------------
}

void drawMeasurementInfo(Obvious2D* viewer, unsigned char* dst, unsigned int w, unsigned int h, unsigned int x, unsigned int y, float value, unsigned char rgba[4], unsigned char rgbaBG[4], short decimalPlaces)
{
  unsigned int width = viewer->getWidth();
  unsigned int height = viewer->getHeight();

  if(viewer->getFullscreen())
  {
    width = viewer->getScreenWidth();
    height = viewer->getScreenHeight();
    float fw = (float)width;
    float fh = (float)height;
    // check aspect ratio, there might be a dual monitor configuration
    if(fw/fh > (16.f/9.f + 1e-3))
      width /= 2;
  }
  else
  {
    width = viewer->getWidth();
    height = viewer->getHeight();
  }

  char text[20];
  char format[6] = "%2.1f";

  if(decimalPlaces > 1 && decimalPlaces < 10)
  {
    format[3] = ((char)decimalPlaces) + 48; //to ascii
  }

  sprintf(text, format, value);

  float radius = 20.f;
  float offset = radius/2.f;
  viewer->addCrosshair(width * x/w, height - height * y/h, text, rgba, rgbaBG, radius, offset);
}

void cbShowHelp()
{
  _showHelp = !_showHelp;
}

void cbShowFPS()
{
  _showFPS = !_showFPS;
}

//void cbShowMean()
//{
//  _showMean = !_showMean; //CARG <----- 
//}

void cbPalette()
{
  unsigned int val = (unsigned int)_palette;
  if((val++)>eAlarmRed) val = 1;
  _palette = (EnumOptrisColoringPalette) val;
}

void cbSnapshot()
{
  _takeSnapshot = true;
}

void cbMultiThreading()
{
  _useMultiThreading = !_useMultiThreading;
  _imager->setUseMultiThreading(_useMultiThreading);
  cout << "Activation state of multi-threading: " << _useMultiThreading << endl;
}

void cbChannel()
{
  _showVisibleChannel = !_showVisibleChannel;
}

void cbManualFlag()
{
  _imager->forceFlagEvent();
}

void cbRaiseEvent()
{
  _imager->raiseSnapshotEvent();
}

void cbStreaming()
{
  _streaming = !_streaming;
  if(_streaming)
  {
    _imager->setThermalFrameCallback(onThermalFrame);
    _imager->setVisibleFrameCallback(onVisibleFrame);
  }
  else
  {
    _imager->setThermalFrameCallback(NULL);
    _imager->setVisibleFrameCallback(NULL);
  }
}

void* displayWorker(void* arg)
{
  pthread_mutex_t* mutex = (pthread_mutex_t*)arg;
  int wInit = _imager->getWidth();
  int hInit = _imager->getHeight();
  int wDisplay = wInit;
  int hDisplay = hInit;
  if(wDisplay<640 && hDisplay<480)
  {
    wDisplay = 640;
    hDisplay = 480;
  }
  Obvious2D viewer(wDisplay, hDisplay, "Optris Imager Example");
  unsigned char green[4] = {0,   255,   0, 255};
  unsigned char black[4] = {0,   0,     0, 255};
  viewer.registerKeyboardCallback('h',   cbShowHelp,       "Show help",                      green, black);
  viewer.registerKeyboardCallback('d',   cbShowFPS,        "Display FPS",                    green, black);
//  viewer.registerKeyboardCallback('a',   cbShowMean,       "Mean temp",                      green, black); //CARG Add
  viewer.registerKeyboardCallback('p',   cbPalette,        "Switch palette",                 green, black);
  viewer.registerKeyboardCallback('m',   cbManualFlag,     "Manual flag event",              green, black);
  viewer.registerKeyboardCallback('s',   cbSnapshot,       "Serialize snapshot",             green, black);
  viewer.registerKeyboardCallback('t',   cbMultiThreading, "Toggle multi-threading",         green, black);
  viewer.registerKeyboardCallback('e',   cbRaiseEvent,     "Raise software event",           green, black);
  viewer.registerKeyboardCallback('x',   cbStreaming,      "Toggle streaming/event mode",    green, black);
  if(_biSpectral)
    viewer.registerKeyboardCallback('c', cbChannel,        "Toggle thermal/visible channel", green, black);

  // Set scaling method for false color conversion
  //_iBuilder.setPaletteScalingMethod(eMinMax);

  // Other possible scaling methods are ...
  //_iBuilder.setPaletteScalingMethod(eSigma1);
  //_iBuilder.setPaletteScalingMethod(eSigma3);
  _iBuilder.setPaletteScalingMethod(eManual);
  _iBuilder.setManualTemperatureRange(22.0f, 37.0f);

  _palette = _iBuilder.getPalette();

  unsigned char* bufferThermal = NULL;
  unsigned char* bufferVisible = NULL;
  unsigned char* bufferEmpty   = new unsigned char[wInit * hInit * 3];
  unsigned char* buffer        = bufferEmpty; // Image to be displayed, as long as no data has been acquired

  int wDraw = wInit;
  int hDraw = hInit;
  bool drawCrosshair = false;

  while(viewer.isAlive() && !_shutdown)
  {
    _iBuilder.setPalette(_palette);

    if(_showVisibleChannel)
    {
      pthread_mutex_lock(mutex);
      while(!_qYUYV.empty())
      {
        Image<unsigned char>* img = _qYUYV.front();
        if(_qYUYV.size()==1)
        {
          wDraw = img->_width;
          hDraw = img->_height;
          if(bufferVisible==NULL)
            bufferVisible = new unsigned char[wDraw * hDraw * 3];
          buffer = bufferVisible;
          _iBuilder.yuv422torgb24(img->_data, buffer, wDraw, hDraw);
        }
        delete img;
        _qYUYV.pop();
      }
      pthread_mutex_unlock(mutex);

      drawCrosshair = false;

      if(_takeSnapshot)
      {
        string file("/tmp/snapshotRGB.ppm");
        _iBuilder.serializePPM(file.c_str(), buffer, wDraw, hDraw);
        cout << "Serialized file to " << file << endl;
        _takeSnapshot = false;
      }
    }
    else if (!_qThermal.empty())
    {
      pthread_mutex_lock(mutex);
      while(!_qThermal.empty())
      {
        Image<unsigned short>* img = _qThermal.front();
        if(_qThermal.size()==1)
        {
          _iBuilder.setData(img->_width, img->_height, img->_data);
          wDraw = _iBuilder.getStride();
          hDraw = img->_height;
//         std::cout << img->_data << "\n data" << std::endl;//CARG: I add this line in 31 Aug 2020
          if(bufferThermal==NULL)
            bufferThermal = new unsigned char[wDraw * hDraw * 3];
          buffer = bufferThermal;
          _iBuilder.convertTemperatureToPaletteImage(buffer);
        }
        delete img;
        _qThermal.pop();
      }
      pthread_mutex_unlock(mutex);

      drawCrosshair = true;

      if(_takeSnapshot)
      {
        unsigned char* ppm;
        unsigned int size;
        string filename("/tmp/snapshot.ppm");
        _iBuilder.convert2PPM(ppm, &size, buffer, _iBuilder.getStride(), hDraw);
        ofstream file(filename.c_str(), std::ios::binary);
        file.write((const char*) ppm, size);
        delete [] ppm;
        cout << "Serialized file to " << filename << endl;

        unsigned char* bar;
        unsigned int wBar = 20;
        unsigned int hBar = 382;
        string fileBar("/tmp/bar.ppm");
        _iBuilder.getPaletteBar(wBar, hBar, bar);
        _iBuilder.serializePPM(fileBar.c_str(), bar, wBar, hBar);
        delete [] bar;
        cout << "Serialized palette bar to " << fileBar << endl;
        _takeSnapshot = false;
      }
    }

    if(drawCrosshair)
    {
      int radius = 3;
      if(wDraw<9 || hDraw<9) radius = 2;
      ExtremalRegion minRegion;
      ExtremalRegion maxRegion;
      _iBuilder.getMinMaxRegion(radius, &minRegion, &maxRegion);
      unsigned char rgba[4]        = {  0,   0, 255, 255};
      unsigned char white[4]       = {255, 255, 255, 128};
      unsigned char whiteBright[4] = {255, 255, 255, 255};
      unsigned char gray[4]        = { 32,  32,  32, 128};
      unsigned char black[4]       = {  0,   0,   0, 255};

      short decimalPlaces = _imager->getTemprangeDecimal();

      drawMeasurementInfo(&viewer, buffer, wDraw, hDraw, (minRegion.u1+minRegion.u2)/2, (minRegion.v1+minRegion.v2)/2, minRegion.t, rgba, white, decimalPlaces);
      rgba[0] = 255;
      rgba[2] = 0;
      drawMeasurementInfo(&viewer, buffer, wDraw, hDraw, (maxRegion.u1+maxRegion.u2)/2, (maxRegion.v1+maxRegion.v2)/2, maxRegion.t, rgba, white, decimalPlaces);
      float mean = _iBuilder.getMeanTemperature(wDraw/2-radius, hDraw/2-radius, wDraw/2+radius, hDraw/2+radius);
      rgba[1] = 255;
      rgba[2] = 255;
      drawMeasurementInfo(&viewer, buffer, wDraw, hDraw, wDraw/2-1, hDraw/2-1, mean, rgba, gray, decimalPlaces);
    }

    viewer.setShowHelp(_showHelp);
    viewer.setShowFPS(_showFPS);
//    viewer.setShowMean(_showMean); //CARG Add
  }

  delete [] bufferEmpty;
  if(bufferThermal) delete [] bufferThermal;
  if(bufferVisible) delete [] bufferVisible;
  _shutdown = true;

  cout << "Shutdown display worker" << endl;
  pthread_exit(NULL);

  return NULL;
}


int main (int argc, char* argv[])
{
  if(argc<2)
  {
    cout << "usage: " << argv[0] << " <xml configuration file>" << endl;
    IRLogger::setVerbosity(IRLOG_DEBUG, IRLOG_DEBUG);
    return -1;
  }

  if(argc > 2)
  {
    cout << "argc > 2 --> printing debug information to stdout and log file" << endl;
    IRLogger::setVerbosity(IRLOG_DEBUG, IRLOG_DEBUG);
  }
  else
  {
    IRLogger::setVerbosity(IRLOG_ERROR, IRLOG_OFF);
  }
  
  // Read parameters from xml file
  if(!IRDeviceParamsReader::readXML(argv[1], _params))
  {
    cout << "<xml configuration file> cannot be read" << endl;
    return -1;
  }

  // Find valid device
  IRDevice* dev;

  dev = IRDevice::IRCreateDevice(_params);


	evo::IRCalibrationManager caliManager(_params.caliPath, _params.formatsPath);
  // Output available optics
  const evo::IRArray<evo::IROptics> *optics = caliManager.getAvailableOptics(dev->getSerial());
  std::cout << "Available optics for camera with serial " << dev->getSerial() << std::endl;
  for (unsigned int i = 0; i < optics->size(); i++)
  {
    evo::IROptics op = (*optics)[i];
    std::cout << "FOV: " << op.fov << " deg, Text: " << op.text << std::endl;
    for (unsigned int j = 0; j < op.tempRanges.size(); j++)
    {
      std::cout << " tMin: " << op.tempRanges[j].tMin << " C, tMax: " << op.tempRanges[j].tMax << " C" << std::endl;
    }
  }
  std::cout << std::endl;

  if(!dev)
  {
    cout << "Error: Device with serial# " << _params.serial << " could not be found" << endl;
    return -1;
  }

  // Initialize Optris image processing chain
  _imager = new IRImager();
  if(_imager->init(&_params, dev->getFrequency(), dev->getWidth(), dev->getHeight(), dev->controlledViaHID(), dev->getHwRev(), dev->getFwRev()))
  {
    // --------------- Print out configuration ---------------
    if(_imager->getWidth()==0 || _imager->getHeight()==0)
    {
      cout << "Error: Image streams not available or wrongly configured. Check connection of camera and config file." << endl;
      return -1;
    }

    cout << "Connected camera, serial: " << dev->getSerial() << ", HW(Rev.): " << _imager->getHWRevision() << ", FW(Rev.): " << _imager->getFWRevision() << endl;

    cout << "Thermal channel: " << _imager->getWidth() << "x" << _imager->getHeight() << "@" << _params.framerate << "Hz" << endl;

    _biSpectral = _imager->hasBispectralTechnology();
    if(_biSpectral)
      cout << "Visible channel: " << _imager->getVisibleWidth() << "x" << _imager->getVisibleHeight() << "@" << _params.framerate << "Hz" << endl;
    // -------------------------------------------------------

    // Set radiation parameters of thermal image
    _imager->setRadiationParameters(1.0, 1.0);

    // --- Set callback methods and start video streaming ----
    dev->setRawFrameCallback(onRawFrame);
    _imager->setThermalFrameCallback(onThermalFrame);
    _imager->setVisibleFrameCallback(onVisibleFrame);
    _imager->setFlagStateCallback(onFlageStateChange);
    _imager->setThermalFrameEventCallback(onThermalFrameEvent);
    _imager->setVisibleFrameEventCallback(onVisibleFrameEvent);
    _imager->setProcessExitCallback(onProcessExit);

    _iBuilder = ImageBuilder(true, _imager->getTemprangeDecimal());
    if(dev->startStreaming()!=0)
    {
      cout << "Error occurred in starting stream ... aborting. You may need to reconnect the camera." << endl;
      exit(-1);
    }
    // -------------------------------------------------------

    pthread_t th;
    pthread_create( &th, NULL, displayWorker, &_mutex);

    FramerateCounter fpsStream;

    // The following state machine enables the handling of unplug/replug events.
    // The user can unplug the USB cable during runtime. As soon as the camera is replugged, the state machine will continue to acquire data.
    IRImagerState irState = IRIMAGER_STATE_ACQUIRE;

    // Enter endless loop in order to pass raw data to Optris image processing library.
    // Processed data are supported by the frame callback function.
    int cntDropped = 0;
    unsigned char* bufferRaw = new unsigned char[dev->getRawBufferSize()];
    while(!_shutdown)
    {
      IRDeviceError retval;

      switch(irState)
      {
      case IRIMAGER_STATE_ACQUIRE:
        double timestamp;

        // getFrame calls back onRawFrame
        retval = dev->getFrame(bufferRaw, &timestamp);

        if(retval==IRIMAGER_SUCCESS)
        {
          //cout << "v4l2 timestamp: " << timestamp << endl;

          if(fpsStream.trigger())
          {
            cout << "Framerate: " << fpsStream.getFps() << " fps, Elapsed (process call): " << _elapsed/(double)_cntElapsed << " ms, Frames: " << _cntFrames << ", Dropped: " << cntDropped << endl;
            _elapsed    = 0.0;
            _cntElapsed = 0;
          }
          usleep(1000);
        }
        else if(retval==IRIMAGER_NOSYNC)
        {
          cntDropped++;
        }
        else if(retval==IRIMAGER_DISCONNECTED)
        {
          delete dev;
          dev = NULL;
          irState = IRIMAGER_STATE_ROAMING;
        }
        else
        {
          cout << "WARNING: Imager returned error code " << retval << endl;
        }
        break;
      case IRIMAGER_STATE_ROAMING:
        cout << "Imager was disconnected ... trying to recover connection state" << endl;
        unsigned long serial = 0;
        IRCalibrationManager::findSerial(serial);
        if(serial==0)
          usleep(500000);
        else
        {
          if(_params.serial != serial)
          {
            cout << "Imager hardware was changed ... skipping" << endl;
            usleep(500000);
            break;
          }

          if(dev==NULL)
          {
            dev = IRDevice::IRCreateDevice(_params);
          }

          if(dev!=NULL)
          {
            _imager->reconnect(&_params, dev->getFrequency(), dev->getWidth(), dev->getHeight(), dev->controlledViaHID(), dev->getHwRev(), dev->getFwRev());
            if(dev->isOpen())
            {
              dev->setRawFrameCallback(onRawFrame);
              _imager->setThermalFrameCallback(onThermalFrame);
              _imager->setVisibleFrameCallback(onVisibleFrame);
              _imager->setFlagStateCallback(onFlageStateChange);
              _imager->setThermalFrameEventCallback(onThermalFrameEvent);
              _imager->setVisibleFrameEventCallback(onVisibleFrameEvent);
              dev->startStreaming();
              irState = IRIMAGER_STATE_ACQUIRE;
            }
          }
        }
        break;
      }
    }

    pthread_join(th, NULL);

    delete [] bufferRaw;

    dev->stopStreaming();
  }
  delete _imager;
  delete dev;

  cout << "Exiting application" << endl;

  //raise(SIGTERM);

  return 1;
}
