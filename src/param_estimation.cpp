/*!*******************************************************************************************
 *  \file       param_estimation.cpp
 *  \brief      Param estimation
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

 #include "param_estimation.hpp"

ParamEstimation::ParamEstimation(double initial_mass)
{
  last_estimated_mass_ = initial_mass;
}


void ParamEstimation::computeMass(float & thrust, double & a_z)
{
  double mass;
  if (std::abs(a_z) > 1e-6 && thrust > 0.0) {
    mass = thrust / a_z;
    if (computeMassError(mass, last_estimated_mass_) ) {
      estimated_mass_ = mass;
      last_estimated_mass_ = estimated_mass_;
    } else {
      estimated_mass_ = last_estimated_mass_;
    }
  }
  estimated_mass_ = last_estimated_mass_;
}

bool ParamEstimation::computeMassError(double & compute_mass, double & last_estimated_mass)
{
  return std::abs(compute_mass - last_estimated_mass) > threshold_;
}

void ParamEstimation::set_threshold(double threshold)
{
  threshold_ = threshold;
}

double ParamEstimation::getEstimatedMass()
{
  return estimated_mass_;
}
