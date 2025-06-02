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

ParamEstimation::ParamEstimation(double real_mass)
: real_mass_(real_mass) {}
ParamEstimation::~ParamEstimation()
{
}
void ParamEstimation::computeMass(double & thrust, double & a_z)
{
  thrust_ = thrust;
  acceleration_ = a_z;

  if (std::abs(acceleration_) > 1e-6) {
    estimated_mass_ = thrust_ / acceleration_;
  } else {
    estimated_mass_ = 0.0;
  }

  mass_history_.push_back(estimated_mass_);
  thrust_history_.push_back(thrust_);
  acceleration_history_.push_back(acceleration_);
}

void ParamEstimation::computeMassError(double & real_mass, double & estimated_mass)
{
  real_mass_ = real_mass;
  estimated_mass_ = estimated_mass;

  mass_error_ = std::abs(estimated_mass_ - real_mass_);
  mass_error_history_.push_back(mass_error_);
}

void ParamEstimation::computeCorrectionFactor(double & real_mass, double & estimated_mass)
{
  real_mass_ = real_mass;
  estimated_mass_ = estimated_mass;

  if (std::abs(estimated_mass_) > 1e-6) {
    correction_factor_ = real_mass_ / estimated_mass_;
  } else {
    correction_factor_ = 0.0;
  }

  correction_factor_history_.push_back(correction_factor_);
}

void ParamEstimation::ComputeRMSE()
{
  if (mass_error_history_.empty()) {return;}

  double sum_sq_error = 0.0;
  for (const auto & err : mass_error_history_) {
    sum_sq_error += err * err;
  }
  double rmse = std::sqrt(sum_sq_error / mass_error_history_.size());
}

void ParamEstimation::computeAll(double & thrust, double & aceleration)
{
  computeMass(thrust, aceleration);
  computeMassError(this->real_mass_, this->estimated_mass_);
  computeCorrectionFactor(this->real_mass_, this->estimated_mass_);
  ComputeRMSE();
}


double ParamEstimation::getEstimatedMass()
{
  return estimated_mass_;
}

double ParamEstimation::getRealMass()
{
  return real_mass_;
}

double ParamEstimation::getMassError()
{
  return mass_error_;
}

double ParamEstimation::getCorrectionFactor()
{
  return correction_factor_;
}

double ParamEstimation::getThrust()
{
  return thrust_;
}

double ParamEstimation::getAcceleration()
{
  return acceleration_;
}

const std::vector<double> & ParamEstimation::getMassHistory()
{
  return mass_history_;
}

const std::vector<double> & ParamEstimation::getThrustHistory()
{
  return thrust_history_;
}

const std::vector<double> & ParamEstimation::getThrustTimeHistory()
{
  return thrust_time_history_;
}

const std::vector<double> & ParamEstimation::getAccelerationHistory()
{
  return acceleration_history_;
}

const std::vector<double> & ParamEstimation::getAccelerationTimeHistory()
{
  return acceleration_time_history_;
}

const std::vector<double> & ParamEstimation::getMassErrorHistory()
{
  return mass_error_history_;
}

const std::vector<double> & ParamEstimation::getCorrectionFactorHistory()
{
  return correction_factor_history_;
}
