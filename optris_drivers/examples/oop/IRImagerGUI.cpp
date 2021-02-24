#include "IRImagerGUI.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

IRImagerGUI* _this;

IRImagerGUI::IRImagerGUI(bool biSpectral, short decimalPlaces)
{
  _viewer = NULL;
  _palette = eIron;
  _showVisibleChannel = false;
  _manualFlagEvent    = false;
  _serializeEvent     = false;
  _biSpectral         = biSpectral;

  _thermal = NULL;
  _imgThermal = NULL;
  _wThermal = 0;
  _hThermal = 0;

  _yuyv = NULL;
  _imgVisible = NULL;
  _wVisible = 0;
  _hVisible = 0;

  _this = this;

  _threadDisplay = std::thread(&IRImagerGUI::displayWorker, this);
}

IRImagerGUI::~IRImagerGUI()
{
  if(_threadDisplay.joinable())
  {
    _threadDisplay.detach();
  }
  if(_viewer)     delete _viewer;

  if(_thermal)    delete [] _thermal;
  if(_imgThermal) delete [] _imgThermal;

  if(_yuyv)       delete [] _yuyv;
  if(_imgVisible) delete [] _imgVisible;

}

bool IRImagerGUI::wantsThermalChannel()
{
  return !_showVisibleChannel;
}

void IRImagerGUI::setThermalImage(unsigned short* thermal, unsigned int w, unsigned int h)
{
  if(w==0 || h==0) return;

  if(!_thermal)
    _thermal = new unsigned short[w * h];

  _iBuilder.setData(w, h, _thermal);

  if(!_imgThermal)
    _imgThermal = new unsigned char[_iBuilder.getStride() * h * 3];

  _mutexDisplay.lock();
  memcpy(_thermal, thermal, w*h*sizeof(*thermal));
  _wThermal = w;
  _hThermal = h;
  _mutexDisplay.unlock();

  _conditionDisplay.notify_all();
}

void IRImagerGUI::setVisibleImage(unsigned char* yuyv, unsigned int w, unsigned int h)
{
  if(w==0 || h==0) return;

  if(!_yuyv)
  {
    _yuyv = new unsigned char[2*w*h];
    _imgVisible = new unsigned char[w*h*3];
  }

  _mutexDisplay.lock();
  memcpy(_yuyv, yuyv, 2*w*h*sizeof(*yuyv));
  _wVisible = w;
  _hVisible = h;
  _mutexDisplay.unlock();

  _conditionDisplay.notify_all();
}

void IRImagerGUI::getSnapshot(unsigned char* &ppm, unsigned int &size)
{
  _iBuilder.convert2PPM(ppm, &size, _imgThermal, _iBuilder.getStride(), _hThermal);
}

bool IRImagerGUI::popManualFlagEvent()
{
  _mutexCommand.lock();
  bool manualFlagEvent = _manualFlagEvent;
  _manualFlagEvent = false;
  _mutexCommand.unlock();
  return manualFlagEvent;
}

bool IRImagerGUI::popSerializeEvent()
{
  _mutexCommand.lock();
  bool serializeEvent = _serializeEvent;
  _serializeEvent = false;
  _mutexCommand.unlock();
  return serializeEvent;
}

bool IRImagerGUI::isAlive()
{
  return (_viewer==NULL) || _viewer->isAlive();
}

void IRImagerGUI::keyboardCallback(char key)
{
  switch(key)
  {
    case 'p':
    {
      unsigned int val = (unsigned int)_palette;
      if((val++)>eAlarmRed) val = 1;
      _palette = (EnumOptrisColoringPalette) val;
      break;
    }
    case 's':
      _mutexCommand.lock();
      _serializeEvent = true;
      _mutexCommand.unlock();
      break;
    case 'c':
      if(_biSpectral)
        _showVisibleChannel = !_showVisibleChannel;
      break;
    case 'm':
      _mutexCommand.lock();
      _manualFlagEvent = true;
      _mutexCommand.unlock();
      break;
  }
}

void* IRImagerGUI::displayWorker()
{
  while(_wThermal==0 && _hThermal==0)
  {
    std::unique_lock<std::mutex> lock(_mutexDisplay);
    _conditionDisplay.wait(lock);
    lock.unlock();
  }

  unsigned char green[4]            = {0,   255,   0, 255};
  unsigned char black[4]            = {0, 0, 0, 255};

  int w = _wThermal;
  int h = _hThermal;
  if(w<320 && h<240)
  {
    w *= 2;
    h *= 2;
  }
  _viewer = new Obvious2D(w, h, "Optris Imager Example");
  _viewer->registerKeyboardClient('p', this, std::string("Switch palette"), green, black);
  _viewer->registerKeyboardClient('m', this, "Manual flag event", green, black);
  _viewer->registerKeyboardClient('s', this, "Serialize snapshot", green, black);
  if(_biSpectral) _viewer->registerKeyboardClient('c', this, "Toggle thermal/visible channel", green, black);
  _viewer->setShowHelp(true);

  _iBuilder.setPaletteScalingMethod(eMinMax);
  _iBuilder.setManualTemperatureRange(15.0f, 40.0f);
  _palette = _iBuilder.getPalette();

  while(_viewer->isAlive())
  {
    std::unique_lock<std::mutex> lock(_mutexDisplay);
    _conditionDisplay.wait(lock);
    lock.unlock();
    _iBuilder.setPalette(_palette);
    if(_showVisibleChannel)
    {
      _mutexDisplay.lock();
      _iBuilder.yuv422torgb24(_yuyv, _imgVisible, _wVisible, _hVisible);
      _mutexDisplay.unlock();
      _viewer->draw(_imgVisible, _wVisible, _hVisible, 3);
    }
    else
    {
      _iBuilder.convertTemperatureToPaletteImage(_imgThermal);
      int radius = 3;
      ExtremalRegion minRegion;
      ExtremalRegion maxRegion;
      _iBuilder.getMinMaxRegion(radius, &minRegion, &maxRegion);
      unsigned char rgba[4] = {0, 0, 255, 255};
      unsigned char white[4]   = {255, 255, 255, 128};
      unsigned char black[4]   = {32, 32, 32, 128};
      drawMeasurementInfo(_imgThermal, _wThermal, _hThermal, (minRegion.u1+minRegion.u2)/2, (minRegion.v1+minRegion.v2)/2, minRegion.t, rgba, white);
      rgba[0] = 255;
      rgba[2] = 0;
      drawMeasurementInfo(_imgThermal, _wThermal, _hThermal, (maxRegion.u1+maxRegion.u2)/2, (maxRegion.v1+maxRegion.v2)/2, maxRegion.t, rgba, white);
      float mean = _iBuilder.getMeanTemperature(_wThermal/2-radius, _hThermal/2-radius, _wThermal/2+radius, _hThermal/2+radius);
      rgba[1] = 255;
      rgba[2] = 255;
      drawMeasurementInfo(_imgThermal, _wThermal, _hThermal, _wThermal/2-1, _hThermal/2-1, mean, rgba, black);
      _mutexDisplay.lock();
      _viewer->draw((unsigned char*)_imgThermal, _iBuilder.getStride(), _hThermal, 3);
      _mutexDisplay.unlock();
    }
  }

  std::cout << "Shutdown display worker" << std::endl;

  return NULL;
}

void IRImagerGUI::drawMeasurementInfo(unsigned char* dst, unsigned int w, unsigned int h, unsigned int x, unsigned int y, float value, unsigned char rgba[4], unsigned char rgbaBG[4])
{
  unsigned int width = _viewer->getWidth();
  unsigned int height = _viewer->getHeight();

  if(_viewer->getFullscreen())
  {
    width = _viewer->getScreenWidth();
    height = _viewer->getScreenHeight();
    float fw = (float)width;
    float fh = (float)height;
    // check aspect ratio, there might be a dual monitor configuration
    if(fw/fh > (16.f/9.f + 1e-3))
      width /= 2;
  }
  else
  {
    width = _viewer->getWidth();
    height = _viewer->getHeight();
  }

  char text[20];
  sprintf(text, "%2.1f", value);

  float radius = 20.f;
  float offset = radius/2.f;
  _viewer->addCrosshair(width * x/w, height - height * y/h, text, rgba, rgbaBG, radius, offset);
}
