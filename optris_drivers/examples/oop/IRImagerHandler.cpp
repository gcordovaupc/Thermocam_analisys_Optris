#include "IRImagerHandler.h"
#include <string.h>
#include <iostream>

using namespace std;

IRImagerHandler::IRImagerHandler(IRDevice* device, IRImager* imager)
{
  _device = device;
  _imager = imager;
  _device->setClient(this);
  _imager->setClient(this);
  _bufferRaw = new unsigned char[device->getRawBufferSize()];
  _thermal = NULL;
  _wThermal = 0;
  _hThermal = 0;

  _yuyv = NULL;
  _wVisible = 0;
  _hVisible = 0;
}

IRImagerHandler::~IRImagerHandler()
{
  if(_thermal) delete [] _thermal;
  if(_yuyv) delete [] _yuyv;
  delete [] _bufferRaw;
}

bool IRImagerHandler::checkForNewFrame()
{
  // getFrame calls method onRawFrame
  _device->getFrame(_bufferRaw);
  bool newFrame = _newFrame;
  _newFrame = false;
  return newFrame;
}

void IRImagerHandler::onRawFrame(unsigned char* data, int size)
{
  // process calls onThermalFrame, onFlagStateChange and onVisibleFrame (bi-spectral)
  _imager->process(_bufferRaw, NULL);
}

void IRImagerHandler::onProcessExit(void* arg)
{

}

void IRImagerHandler::onThermalFrame(unsigned short* thermal, unsigned int w, unsigned int h, IRFrameMetadata meta, void* arg)
{
  _wThermal = w;
  _hThermal = h;
  if(_thermal==NULL) _thermal = new unsigned short[w*h];
  memcpy(_thermal, thermal, w*h*sizeof(*thermal));
  _newFrame = true;
}

void IRImagerHandler::onVisibleFrame(unsigned char* yuyv, unsigned int w, unsigned int h, IRFrameMetadata meta, void* arg)
{
  _wVisible = w;
  _hVisible = h;
  if(_yuyv==NULL) _yuyv = new unsigned char[w*h*2];
  memcpy(_yuyv, yuyv, 2*w*h*sizeof(*yuyv));
  _newFrame = true;
}

void IRImagerHandler::onFlagStateChange(EnumFlagState flagstate, void* arg)
{
  cout << "Flag state: " << flagstate << endl;
}

unsigned short* IRImagerHandler::getThermalImage()
{
  return _thermal;
}

unsigned int IRImagerHandler::getThermalWidth()
{
  return _wThermal;
}

unsigned int IRImagerHandler::getThermalHeight()
{
  return _hThermal;
}

unsigned char* IRImagerHandler::getVisibleImage()
{
  return _yuyv;
}

unsigned int IRImagerHandler::getVisibleWidth()
{
  return _wVisible;
}

unsigned int IRImagerHandler::getVisibleHeight()
{
  return _hVisible;
}
