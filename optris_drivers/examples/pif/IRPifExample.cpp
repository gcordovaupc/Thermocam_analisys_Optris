#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <signal.h>

// Optris device interfaces
#include "IRDevice.h"

// Optris imager interfaces
#include "IRImager.h"

// Optris logging interface
#include "IRLogger.h"

// Optris raw image file writer
#include "IRFileWriter.h"

// evo Frame Rate Counter
#include "FramerateCounter.h"

using namespace std;
using namespace evo;

bool _keepCapturing = true;

void sigHandler(int dummy = 0)
{
  _keepCapturing = false;
}

void displayEvents(IRArray<IREventData> events) {
	for (unsigned int i = 0; i < events.size(); i++)
  {
		IREventData& event = events[i];
		cout << "\tChannel: " << +event.channel << " Type: ";
		switch (event.inputType)
		{
      case  IREventInputType::DigitalInput:
        cout << "DigitalInput";
        break;
      case  IREventInputType::AnalogInput:
        cout << "AnalogInput";
        break;
      case  IREventInputType::Software:
        cout << "Software";
        break;
      default:
        cout << "Unknown";
        break;
    }
    cout << endl;
  }
}


void onThermalFrame(unsigned short *thermal, unsigned int w, unsigned int h, IRFrameMetadata meta, void *arg)
{
  //Access to thermal frame corresponding pif input values over meta data:


  cout << "Frameid: " << meta.counter << endl;

  //Print out digital input values
  cout << "DIs: ";
  for (unsigned int i = 0; i < meta.pifDIs.size(); i++)
  {
    cout << meta.pifDIs[i] << " ";
  }

  //Print out analog input values
  cout << endl << "AIs: ";
  for (unsigned int i = 0; i < meta.pifAIs.size(); i++)
  {
    cout << meta.pifAIs[i] << " ";
  }
  
  cout << endl;
}

/**
 * @brief will be called on pif snapshot event
 * @param events List of triggered events.
 */
void onThermalFrameEvent(unsigned short *thermal, unsigned short *energy, unsigned int w, unsigned int h, IRFrameMetadata meta, const IRArray<IREventData>& events, void *arg)
{
  cout << "onThermalFrameEvent" << endl;
  displayEvents(events);
}

/**
 * @brief will be called on pif snapshot event.
 * @param events List of triggered events.
 */

void onVisibleFrameEvent(unsigned char *data, unsigned int w, unsigned int h, IRFrameMetadata meta, const IRArray<IREventData>& events, void *arg)
{
  cout << "onVisibleFrameEvent" << endl;
  displayEvents(events);
}

int main(int argc, char *argv[])
{
  if (argc != 2)
  {
    cout << "usage: " << argv[0] << " <xml configuration file>" << endl;
    return -1;
  }

  signal(SIGINT, sigHandler);
  IRLogger::setVerbosity(IRLOG_ERROR, IRLOG_OFF);
  IRDeviceParams params;
  IRDeviceParamsReader::readXML(argv[1], params);
  IRDevice *dev = IRDevice::IRCreateDevice(params);

  if (dev)
  {
    /**
     * Initialize Optris image processing chain
     */
    IRImager imager;
    if (imager.init(&params, dev->getFrequency(), dev->getWidth(), dev->getHeight(), dev->controlledViaHID()))
    {
      if (imager.getWidth() != 0 && imager.getHeight() != 0)
      {
        cout << "Thermal channel: " << imager.getWidth() << "x" << imager.getHeight() << "@" << imager.getMaxFramerate() << "Hz" << endl;

        if (!dev)
        {
          cout << "NO DEV" << endl;
          exit(1);
        }

        // Start UVC streaming
        if (dev->isOpen())
        {
          // Enter loop in order to pass raw data to Optris image processing library.
          // Processed data are supported by the frame callback function.
          double timestamp;
          unsigned char *bufferRaw = new unsigned char[dev->getRawBufferSize()];

          char nmea[GPSBUFFERSIZE];
          memset(nmea, 0, GPSBUFFERSIZE * sizeof(*nmea));

          imager.forceFlagEvent(1000.0);
          imager.setThermalFrameCallback(onThermalFrame);

          imager.setThermalFrameEventCallback(onThermalFrameEvent);
          imager.setVisibleFrameEventCallback(onVisibleFrameEvent);

          // ##################################
          // BEGIN PIF Configuration
          // ##################################

          IRPifType::Value pifType = imager.getPifType();
          IRPifConfig pifConfig;
          
          pifConfig = imager.setPifType(IRPifType::Intern,1);
          pifType = imager.getPifType();

          unsigned int fwRevision = imager.getFWRevision();
          // For Xi80 only!
          // Choose between intern or stackable pif, here stackable is selected:
          if(fwRevision >= 3000 && fwRevision < 3200 && pifType != IRPifType::Stackable){
            pifConfig = imager.setPifType(IRPifType::Stackable,1);
          }
          else{
            //get initialized default pif config
            pifConfig = imager.getPifConfig();
          }

          // Activate snapshot event on di[0] = true
          if (pifConfig.ChannelsDI.size() > 0)
          {
            pifConfig.ChannelsDI[0].IsLowActive = false;
            pifConfig.ChannelsDI[0].Mode = IRChannelInputMode::Snapshot;
          }

          // Activate snapshot event on ai[0] > 5V
          if (pifConfig.ChannelsAI.size() > 0)
          {
            pifConfig.ChannelsAI[0].IsLowActive = false;
            pifConfig.ChannelsAI[0].Mode = IRChannelInputMode::Snapshot;
            pifConfig.ChannelsAI[0].Threshold = 5;
          }

          // Set ao[0] to output flag state
          if (pifConfig.ChannelsAO.size() > 0)
          {
            pifConfig.ChannelsAO[0].AnalogMode = IRChannelAnalogOutputMode::Range_0V_10V;
            
            pifConfig.FlagOpenOutput.Channel = pifConfig.ChannelsAO[0];

            // Set flag in voltages or ampere depending on pifConfig.ChannelsAO[0].AnalogMode
            pifConfig.FlagOpenOutput.AnalogValueClosed = 10;
            pifConfig.FlagOpenOutput.AnalogValueMoving = 5;
            pifConfig.FlagOpenOutput.AnalogValueOpen = 0;
          }

          // Set ao[1] to output frame sync
          if (pifConfig.ChannelsAO.size() > 1)
          {
            pifConfig.FrameSyncOutput.Channel = pifConfig.ChannelsAO[1];
            // Set frame sync voltages
            pifConfig.FrameSyncOutput.AnalogValue = 3;

            pifConfig.ChannelsAO[1].AnalogMode = IRChannelAnalogOutputMode::Range_0V_10V;
          }

          // Set ao[2] to output manual values in range of 0V..10V
          if (pifConfig.ChannelsAO.size() > 2)
          {
            pifConfig.ChannelsAO[2].Mode = IRChannelOutputMode::Manual;
            pifConfig.ChannelsAO[2].AnalogMode = IRChannelAnalogOutputMode::Range_0V_10V;
          }

          // Set ao[3] to output manual values in range of 0mA..20mA
          if (pifConfig.ChannelsAO.size() > 3)
          {
            pifConfig.ChannelsAO[3].Mode = IRChannelOutputMode::Manual;
            pifConfig.ChannelsAO[3].AnalogMode = IRChannelAnalogOutputMode::Range_0mA_20mA;
          }

          //Alternative DO-Outputs can be set for flag and framesync output

          // Set do[0] to output flag state. If flag is active output is false, otherwise true
          // if (pifConfig.ChannelsDO.size() > 0)
          //   pifConfig.FlagOpenOutput.Channel = pifConfig.ChannelsDO[0];

          // Set do[1] to output frame sync
          // if (pifConfig.ChannelsDO.size() > 1)
          //   pifConfig.FrameSyncOutput.Channel = pifConfig.ChannelsDO[1];


          //Set pif config for apply new configuration
          imager.setPifConfig(pifConfig);
          // ##################################
          // END PIF Configuration
          // ##################################


          //set do[0] to true
          if(pifConfig.ChannelsDO.size() > 0)
            imager.setPifDO(0, true);

          //set ao[2] to 6V
          if (pifConfig.ChannelsAO.size() > 2)
            imager.setPifAO(2, 6); //set 6V

          //set ao[3] to 10mA
          if (pifConfig.ChannelsAO.size() > 3)
            imager.setPifAO(3, 0.010f); //set to 10mA


          dev->startStreaming();

          while (_keepCapturing)
          {
            dev->getFrame(bufferRaw);
            imager.process(bufferRaw, NULL);

            //get the last metadata with pif input values
            const IRFrameMetadata irFrameMetadata = imager.getLastMetadata();

            //Print out digital input values
            cout << "DIs: ";
            for (unsigned int i = 0; i < irFrameMetadata.pifDIs.size(); i++)
            {
              cout << irFrameMetadata.pifDIs[i] << " ";
            }

            //Print out analog input values
            cout << endl << "AIs: ";
            for (unsigned int i = 0; i < irFrameMetadata.pifAIs.size(); i++)
            {
              cout << irFrameMetadata.pifAIs[i] << " ";
            }
            
            cout << endl;
          }
        }
        else
        {
          cout << "Error occurred in starting stream ... aborting. You may need to reconnect the camera." << endl;
        }
      }
    }
    else
    {
      cout << "Error: Image streams not available or wrongly configured. Check connection of camera and config file." << endl;
    }
    delete dev;
  }
  return 0;
}
