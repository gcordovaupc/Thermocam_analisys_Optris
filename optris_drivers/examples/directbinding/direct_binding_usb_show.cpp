#include <iostream>
#include <memory>

//--Code for displaying image -----------------
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include "libirimager/direct_binding.h"
//---------------------------------------------

int main(int argc, char *argv[])
{

  if(argc!=2)
  {
    std::cout << "usage: " << argv[0] << " <xml configuration file>" << std::endl;
    return -1;
  }

  if(::evo_irimager_usb_init(argv[1], 0, 0) != 0) return -1;
  int err;
  int p_w;
  int p_h;
  if((err = ::evo_irimager_get_palette_image_size(&p_w, &p_h)) != 0)
  {
    std::cerr << "error on evo_irimager_get_palette_image_size: " << err << std::endl;
    exit(-1);
  }

  int t_w;
  int t_h;

  //!width of thermal and palette image can be different duo to stride of 4 alignment
  if((err = ::evo_irimager_get_thermal_image_size(&t_w, &t_h)) != 0)
  {
    std::cerr << "error on evo_irimager_get_palette_image_size: " << err << std::endl;
    exit(-1);
  }

  std::vector<unsigned char> palette_image(p_w * p_h * 3);
  std::vector<unsigned short> thermal_data(t_w * t_h);

  do
  {
    if((err = ::evo_irimager_get_thermal_palette_image(t_w, t_h, &thermal_data[0], p_w, p_h, &palette_image[0]))==0)
    {
      unsigned long int mean = 0;
      //--Code for calculation mean temperature of image -----------------
      for (int y = 0; y < t_h; y++)
      {
        for (int x = 0; x < t_w; x++)
        {
          mean += thermal_data[y*t_w + x];
          //temperatura = thermal_data[];
        }
      }
      std::cout << (mean / (t_h * t_w)) / 10.0 - 100 << std::endl;
      //std::cout << temperatura << std::endl;
      //---------------------------------------------

      //--Code for displaying image -----------------
      cv::Mat cv_img(cv::Size(p_w, p_h), CV_8UC3, &palette_image[0], cv::Mat::AUTO_STEP);
      cv::cvtColor(cv_img, cv_img, CV_BGR2RGB); //Check how to add the CV_WINDOW_AUTOSIZE and re-build the executable node to make it work 
      cv::imshow("palette image daemon modifies", cv_img);
      //---------------------------------------------
    }
    else
    {
      std::cerr << "failed evo_irimager_get_thermal_palette_image: " << err << std::endl;
    }
    
  } while(cv::waitKey(1) != 'q');//cvGetWindowHandle("palette image daemon"));

  ::evo_irimager_terminate();

  return 0;
}
