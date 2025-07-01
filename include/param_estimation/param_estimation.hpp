/*!*******************************************************************************************
 *  \file       param_estimation.hpp
 *  \brief      Param estimation definition
 *  \authors    Carmen De Rojas Pita-Romero
 *
 *  \copyright  Copyright (c) 2025 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

 #include <vector>
 #include <cmath>
 #include <iostream>

#ifndef __PARAM_ESTIMATION_HPP__
#define __PARAM_ESTIMATION_HPP__
class ParamEstimation
{
public:
  ParamEstimation(double initial_mass);
  ~ParamEstimation() {}

  // MEMBER ATRIBUTES

private:
  double estimated_mass_;
  double last_estimated_mass_;
  double threshold_;

// PUBLIC FUNCTIONS

public:
  /**
  * @brief
  * Computes the mass based on the thrust and acceleration
  * @param thrust Thrust value (z axis)
  * @param a_z Acceleration in z axis
  */
  void computeMass(float & thrust, double & a_z);
  void set_threshold(double threshold);
  double getEstimatedMass();
// PRIVATE FUNCTIONS

private:
  /**
   * @brief
   * Computes the mass error based on the real and estimated mass
   * @param real_mass Real mass of the drone
   * @param estimated_mass Estimated mass of the drone
   */
  bool computeMassError(double & estimated_mass, double & last_estimated_mass);

};

#endif // __PARAM_ESTIMATION_HPP__
