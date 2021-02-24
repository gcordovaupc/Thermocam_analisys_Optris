#ifndef OOP_IRIMAGERHANDLER_H_
#define OOP_IRIMAGERHANDLER_H_

/**
 * Optris PI imager interface
 */
#include "IRImager.h"

/**
 * Optris UVC device interface
 */
#include "IRDevice.h"

/**
 * Optris image converter
 */
#include "ImageBuilder.h"

/**
 * Optris frame rate calculation helper
 */
#include "FramerateCounter.h"

/**
 * Visualization
 */
#include "Obvious2D.h"

using namespace evo;

/**
 * @class IRImagerHandler
 * @brief Represents an object-oriented example of how to manage thermal and visible images retrieved from PI imagers
 * @author Stefan May (Evocortex GmbH)
 */
class IRImagerHandler : public IRImagerClient
{

public:

  /**
   * Constructor
   * @param device pointer to already instantiated raw video device
   * @param imager pointer to already configured imager instance
   */
  IRImagerHandler(IRDevice* device, IRImager* imager);

  /**
   * Destructor
   */
  virtual ~IRImagerHandler();

  /**
   * Check if new frame is available
   */
  bool checkForNewFrame();

  /**
   * Implementation of pure virtual method from IRImagerClient
   */
  virtual void onRawFrame(unsigned char* data, int size);

  /**
   * Implementation of pure virtual method from IRImagerClient
   */
  virtual void onThermalFrame(unsigned short* data, unsigned int w, unsigned int h, IRFrameMetadata meta, void* arg);

  /**
   * Overwritten method from IRImagerClient
   */
  virtual void onVisibleFrame(unsigned char* data, unsigned int w, unsigned int h, IRFrameMetadata meta, void* arg);

  /**
   * Implementation of pure virtual method from IRImagerClient
   */
  virtual void onFlagStateChange(EnumFlagState flagstate, void* arg);

  /**
   * Implementation of pure virtual method from IRImagerClient
   */
  virtual void onProcessExit(void* arg);

  /**
   * Get pointer to last retrieved thermal image
   * @return thermal image
   */
  unsigned short* getThermalImage();

  /**
   * Get width of last retrieved thermal image
   * @return image width
   */
  unsigned int getThermalWidth();

  /**
   * Get height of last retrieved thermal image
   * @return image height
   */
  unsigned int getThermalHeight();

  /**
   * Get pointer to last retrieved visible image (only available for PI200 and PI230)
   * @return visible image
   */
  unsigned char* getVisibleImage();

  /**
   * Get width of last retrieved visible image
   * @return image width
   */
  unsigned int getVisibleWidth();

  /**
   * Get height of last retrieved visible image
   * @return image height
   */
  unsigned int getVisibleHeight();

private:

  IRImager* _imager;
  IRDevice* _device;
  unsigned char* _bufferRaw;

  unsigned int _wThermal;
  unsigned int _hThermal;
  unsigned int _wVisible;
  unsigned int _hVisible;
  unsigned short* _thermal;
  unsigned char* _yuyv;

  bool _newFrame;

};

#endif // OOP_IRIMAGERHANDLER_H_
