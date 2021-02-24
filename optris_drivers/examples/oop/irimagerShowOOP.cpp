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

// Optris frame rate counter
#include "FramerateCounter.h"

// Optris logging interface
#include "IRLogger.h"

// Class wrapping callback routines
#include "IRImagerHandler.h"

// Graphical User Interface
#include "IRImagerGUI.h"

using namespace std;
using namespace evo;

int main (int argc, char* argv[])
{
  if(argc!=2)
  {
    cout << "usage: " << argv[0] << " <xml configuration file>" << endl;
    return -1;
  }

  IRLogger::setVerbosity(IRLOG_ERROR, IRLOG_OFF);
  IRDeviceParams params;
  IRDeviceParamsReader::readXML(argv[1], params);
  IRDevice* dev = NULL;

  dev = IRDevice::IRCreateDevice(params);

  // Initialize Optris image processing chain
  IRImager imager;
  if(imager.init(&params, dev->getFrequency(), dev->getWidth(), dev->getHeight(), dev->controlledViaHID()))
  {
    unsigned int w = imager.getWidth();
    unsigned int h = imager.getHeight();
    if(w==0 || h==0)
    {
      cout << "Error: Image streams not available or wrongly configured. Check connection of camera and config file." << endl;
      return -1;
    }

    cout << "Connected camera, serial: " << dev->getSerial()
         << ", HW(Rev.): "               << imager.getHWRevision()
         << ", FW(Rev.): "               << imager.getFWRevision() << endl;
    cout << "Thermal channel: " << w << "x" << h << "@" << params.framerate << "Hz" << endl;
    if(imager.hasBispectralTechnology())
      cout << "Visible channel: " << imager.getVisibleWidth() << "x" << imager.getVisibleHeight() << "@" << params.framerate << "Hz" << endl;

    IRImagerHandler handler(dev, &imager);

    if(dev->startStreaming()!=0)
    {
      cout << "Error occurred in starting stream ... aborting. You may need to reconnect the camera." << endl;
      exit(-1);
    }

    IRImagerGUI gui(imager.hasBispectralTechnology(), imager.getTemprangeDecimal());

    FramerateCounter fpsStream;

    // Enter endless loop in order to pass raw data to Optris image processing library.
    // Processed data are supported by the frame callback function.
    while(gui.isAlive())
    {
      if(handler.checkForNewFrame())
      {
        if(gui.wantsThermalChannel())
          gui.setThermalImage(handler.getThermalImage(), handler.getThermalWidth(), handler.getThermalHeight());
        else
          gui.setVisibleImage(handler.getVisibleImage(), handler.getVisibleWidth(), handler.getVisibleHeight());
      }
      if(gui.popManualFlagEvent())
        imager.forceFlagEvent();
      if(gui.popSerializeEvent())
      {
        unsigned char* ppm;
        unsigned int size;
        gui.getSnapshot(ppm, size);
        ofstream file("/tmp/snapshot.ppm", std::ios::binary);
        file.write((const char*) ppm, size);
        delete [] ppm;
      }
      if(fpsStream.trigger())
        cout << "Framerate: " << fpsStream.getFps() << " fps" << endl;
    }

    dev->stopStreaming();
  }

  cout << "Exiting application" << endl;
  
  delete dev;

  return 0;
}
