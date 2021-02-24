#ifndef OOP_IRIMAGERGUI_H_
#define OOP_IRIMAGERGUI_H_

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

#include "IRImager.h"

/**
 * C++11 threading support
 */
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace evo;

/**
 * @class IRImagerGUI
 * @brief Represents an object-oriented example of how to display thermal and visible images
 * @author Stefan May (Evocortex GmbH)
 */
class IRImagerGUI : public Obvious2DClient
{

public:

  /**
   * Constructor
   */
  IRImagerGUI(bool biSpectral, short decimalPlaces = 1);

  /**
   * Destructor
   */
  virtual ~IRImagerGUI();

  /**
   * Channel flag
   * true = GUI wants to display thermal images
   * false = GUI wants to display yuyv images
   */
  bool wantsThermalChannel();

  /**
   * Copy thermal image to internal buffer for next display cycle
   * @param thermal thermal image
   * @param w width of image
   * @param h height of image
   */
  void setThermalImage(unsigned short* thermal, unsigned int w, unsigned int h);

  /**
   * Copy visible image to internal buffer for next display cycle
   * @param yuyv visible image in YUYV format
   * @param w width of image
   * @param h height of image
   */
  void setVisibleImage(unsigned char* yuyv, unsigned int w, unsigned int h);

  /**
   * Make snapshot of recently set thermal image, i.e., convert thermal image to false color representation
   * @param ppm image in PPM format, buffer is internally allocated
   * @param size size of allocated image buffer
   */
  void getSnapshot(unsigned char* &ppm, unsigned int &size);

  /**
   * Pop (i.e. remove, if set) manual flag event, which was possibly set by user input via display
   * @return manual flag event status
   */
  bool popManualFlagEvent();

  /**
   * Pop (i.e. remove, if set) serialize event, which was possibly set by user input via display
   * @return serialize event status
   */
  bool popSerializeEvent();

  /**
   *  Get flag whether display is alive
   *  @return alive flag
   */
  bool isAlive();

  /**
   * Execute keyboard callback methods
   * @param key key press event
   */
  void keyboardCallback(char key);

private:

  /**
   * Display loop (runs in own thread)
   */
  void* displayWorker();

  /**
   * Draw crosshairs with measurement info
   * @param dst destination image
   * @param w width of destination image
   * @param h height of destination image
   * @param x x-coordinate of measurement position
   * @param y y-coordinate of measurement position
   * @param float measurement value
   * @param rgba foreground color
   * @param rgbaBG background color
   */
  void drawMeasurementInfo(unsigned char* dst, unsigned int w, unsigned int h, unsigned int x, unsigned int y, float value, unsigned char rgba[4], unsigned char rgbaBG[4]);

  /**
   * Draw help, i.e., keys registered for key press events
   */
  void drawHelp();

  Obvious2D* _viewer;

  bool _biSpectral;

  bool _showVisibleChannel;

  EnumOptrisColoringPalette _palette;

  unsigned short* _thermal;

  unsigned char* _imgThermal;

  unsigned int _wThermal;

  unsigned int _hThermal;

  unsigned char* _yuyv;

  unsigned char* _imgVisible;

  unsigned int _wVisible;

  unsigned int _hVisible;

  std::thread _threadDisplay;
  std::mutex _mutexDisplay;
  std::mutex _mutexCommand;
  std::condition_variable _conditionDisplay;

  ImageBuilder _iBuilder;

  bool _manualFlagEvent;

  bool _serializeEvent;

};

#endif /* OOP_IRIMAGERGUI_H_ */
