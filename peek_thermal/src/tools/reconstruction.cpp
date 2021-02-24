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



#include <peek_thermal/Reconstruction.h>


Reconstruction::Reconstruction(cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    //Camera Parameters
    cameraMatrix_ = cameraMatrix;
    distCoeffs_ = distCoeffs;
}


Reconstruction::~Reconstruction()
{
  std::cout << "Node Shutting Down Reconstruction now!" << std::endl;
}


Reconstruction::retTransform_ Reconstruction::reconPoints(const std::vector<cv::Point2f> &imgPoints, const std::vector<cv::Point3f> &objectPoints){

    retTransform_ transform;
    //transform.rvec = cv::Mat(3,1,cv::DataType<double>::type);
    //transform.tvec = cv::Mat(3,1,cv::DataType<double>::type);

    cv::Mat rotationMatrix;

    if(imgPoints.size()==objectPoints.size()){
        //get transform
        try{
            cv::solvePnP(objectPoints, imgPoints, cameraMatrix_, distCoeffs_, transform.rvec, transform.tvec);
        }
        catch (cv::Exception& e) {
            std::cerr << "Error by doing solvePnP" << "\". Reason: " << e.msg << std::endl;
        }
        //getting rotation matrix
        try{
            cv::Rodrigues(transform.rvec, rotationMatrix);
        }
        catch (cv::Exception& e) {
            std::cerr << "Error doing Rodrigues Normalization" << "\". Reason: " << e.msg << std::endl;
        }

        //get euler angels from rotation matrix
        transform.rvec = rotMatToEuler(rotationMatrix);
        //convert Mat type of translation (needed for later stuff)
        //TO DO: make clear why!
        transform.tvec.convertTo(transform.tvec,21);
    }
    return transform;
}

//calculate Euler Angles form Rotation Matrix
cv::Mat Reconstruction::rotMatToEuler(const cv::Mat &rotMat)
{
    //check if roation matrix is valid
    assert(checkValidRotMat(rotMat));

    cv::Mat eulerMat;

    //calculate eulerMat anlges
    float sy = sqrt(rotMat.at<double>(0,0) * rotMat.at<double>(0,0) +  rotMat.at<double>(1,0) * rotMat.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float roll, pitch, yaw;
    if (!singular)
    {
        roll = atan2(rotMat.at<double>(2,1) , rotMat.at<double>(2,2));
        pitch = atan2(-rotMat.at<double>(2,0), sy);
        yaw = atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
    }
    else
    {
        roll = atan2(-rotMat.at<double>(1,2), rotMat.at<double>(1,1));
        pitch = atan2(-rotMat.at<double>(2,0), sy);
        yaw = 0;
    }

    eulerMat.push_back(cv::Point3f(roll, pitch, yaw));

    return eulerMat;
}


//check if Rotation Matrix is valid
bool Reconstruction::checkValidRotMat(const cv::Mat &rotMat)
{
    cv::Mat Rt;
    cv::transpose(rotMat, Rt);
    cv::Mat shouldBeIdentity = Rt * rotMat;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  cv::norm(I, shouldBeIdentity) < 1e-6;
}
