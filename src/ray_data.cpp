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

#include "livox_simulation/ray_data.h"

namespace livox
{
  RayData::RayData()
  {
  }

  RayData::~RayData()
  {
  }

  int RayData::getSizeLeftUpperQuadrant()
  {
    return this->left_upper_quadrant_rays_.size();
  }

  int RayData::getSizeLeftLowerQuadrant()
  {
    return this->left_lower_quadrant_rays_.size();
  }

  int RayData::getSizeRightUpperQuadrant()
  {
    return this->right_upper_quadrant_rays_.size();
  }

  int RayData::getSizeRightLowerQuadrant()
  {
    return this->right_lower_quadrant_rays_.size();
  }

  void RayData::addLeftUpperQuadrant(gazebo::physics::RayShapePtr ray)
  {
    this->left_upper_quadrant_rays_.push_back(ray);
  }
  void RayData::addLeftLowerQuadrant(gazebo::physics::RayShapePtr ray)
  {
    this->left_lower_quadrant_rays_.push_back(ray);
  }
  void RayData::addRightUpperQuadrant(gazebo::physics::RayShapePtr ray)
  {
    this->right_upper_quadrant_rays_.push_back(ray);
  }
  void RayData::addRightLowerQuadrant(gazebo::physics::RayShapePtr ray)
  {
    this->right_lower_quadrant_rays_.push_back(ray);
  }

  gazebo::physics::RayShapePtr RayData::getLeftUpperQuadrant(unsigned int i)
  {
    return this->left_upper_quadrant_rays_[i];
  }
  gazebo::physics::RayShapePtr RayData::getLeftLowerQuadrant(unsigned int i)
  {
    return this->left_lower_quadrant_rays_[i];
  }
  gazebo::physics::RayShapePtr RayData::getRightUpperQuadrant(unsigned int i)
  {
    return this->right_upper_quadrant_rays_[i];
  }
  gazebo::physics::RayShapePtr RayData::getRightLowerQuadrant(unsigned int i)
  {
    return this->right_lower_quadrant_rays_[i];
  }

}