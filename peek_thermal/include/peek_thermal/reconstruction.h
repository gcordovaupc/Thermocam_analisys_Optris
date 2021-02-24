#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2020, Heiko Engemann
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin Hallenbeck nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"

#include <iostream>
#include <string>


class Reconstruction
{
public:
  Reconstruction(cv::Mat cameraMatrix, cv::Mat distCoeffs);
  ~Reconstruction();

  //return the marker points
  struct retTransform_{ cv::Mat rvec; cv::Mat tvec;}; // rotation & tanslation

  struct retTransform_ reconPoints(const std::vector<cv::Point2f> &imgPoints, const std::vector<cv::Point3f> &objectPoints);

 private:

  //Rotations Matrix to Euler
  cv::Mat rotMatToEuler(const cv::Mat &rotMat);
  //Check if Rotation Matrix is valid
  bool checkValidRotMat(const cv::Mat &rotMat);

  cv::Mat cameraMatrix_;
  cv::Mat distCoeffs_;

};



#endif // RECONSTRUCTION_H
