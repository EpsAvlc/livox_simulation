/* This file is part of the software provided by arti-robots.
 Copyright (c) 2019, michael
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted  provided that the
 following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
    disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
    products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PROJECT_RAY_DATA_H
#define PROJECT_RAY_DATA_H

#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/physics/physics.hh>

namespace livox
{
  class RayData
  {
  public:
    RayData();
    ~RayData();

    int getSizeLeftUpperQuadrant();
    int getSizeLeftLowerQuadrant();
    int getSizeRightUpperQuadrant();
    int getSizeRightLowerQuadrant();

    void addLeftUpperQuadrant(gazebo::physics::RayShapePtr ray);
    void addLeftLowerQuadrant(gazebo::physics::RayShapePtr ray);
    void addRightUpperQuadrant(gazebo::physics::RayShapePtr ray);
    void addRightLowerQuadrant(gazebo::physics::RayShapePtr ray);

    gazebo::physics::RayShapePtr getLeftUpperQuadrant(unsigned int );
    gazebo::physics::RayShapePtr getLeftLowerQuadrant(unsigned int );
    gazebo::physics::RayShapePtr getRightUpperQuadrant(unsigned int );
    gazebo::physics::RayShapePtr getRightLowerQuadrant(unsigned int );

    std::vector<float> left_upper_vertical_ray_angles_;
    std::vector<float> left_upper_horizontal_ray_angles_;

    std::vector<float> left_lower_vertical_ray_angles_;
    std::vector<float> left_lower_horizontal_ray_angles_;

    std::vector<float> right_upper_vertical_ray_angles_;
    std::vector<float> right_upper_horizontal_ray_angles_;

    std::vector<float> right_lower_vertical_ray_angles_;
    std::vector<float> right_lower_horizontal_ray_angles_;

  //private:
    std::vector<gazebo::physics::RayShapePtr> left_upper_quadrant_rays_;
    std::vector<gazebo::physics::RayShapePtr> left_lower_quadrant_rays_;
    std::vector<gazebo::physics::RayShapePtr> right_upper_quadrant_rays_;
    std::vector<gazebo::physics::RayShapePtr> right_lower_quadrant_rays_;


  };
}

#endif //PROJECT_RAY_DATA_H
