/*!*******************************************************************************************
 *  \file       csv_logger.hpp
 *  \brief      Tool for save data in a csv file
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
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>


#include "param_estimation/param_estimation.hpp"


class CsvSaver
{
public:
  explicit CsvSaver(const std::string & file_name)
  : file_name_(file_name)
  {
    std::cout << "Saving to file: " << file_name << std::endl;
    file_ = std::ofstream(file_name, std::ofstream::out | std::ofstream::trunc);
    file_ << "time" << "," << "thrust" << "," << "acceleration" << ","
          << "real_mass" << "," << "estimated_mass" << "," << "mass_error" << ","
          << "correction_factor" << std::endl;
  }

  ~CsvSaver() {file_.close();}

  void add_double(const double data)
  {
    if (std::isnan(data)) {
      throw std::invalid_argument("Data is NaN");
    }
    file_ << data << ",";
  }

  void add_string(const std::string & data, const bool add_final_comma = true)
  {
    file_ << data;
    if (add_final_comma) {
      file_ << ",";
    }
  }

  void add_vector_row(
    const std::vector<std::vector<double>> & data,
    const bool add_final_comma = true)
  {
    for (size_t i = 0; i < data.size(); ++i) {
      const auto & row = data[i];
      for (size_t j = 0; j < row.size(); ++j) {
        const double & value = row[j];

        if (std::isnan(value)) {
          throw std::invalid_argument("Data contains NaN");
        }

        file_ << value;
        if (j < row.size() - 1) {
          file_ << ",";
        }
      }

      if (add_final_comma) {
        file_ << ",";
      }
      file_ << std::endl;
    }
  }
  void save(const double time, ParamEstimation & param_estimation)
  {
    add_double(time);
    // Thrust (as double)
    double thrust = param_estimation.getThrust();
    add_double(thrust);
    // Acceleration (as double)
    double acceleration = param_estimation.getAcceleration();
    add_double(acceleration);
    // Real mass (as double)
    double real_mass = param_estimation.getRealMass();
    add_double(real_mass);
    // Estimated mass (as double)
    double estimated_mass = param_estimation.getEstimatedMass();
    add_double(estimated_mass);
    // Mass error (as double)
    double mass_error = param_estimation.getMassError();
    add_double(mass_error);
    // Correction factor (as double)
    double correction_factor = param_estimation.getCorrectionFactor();
    add_double(correction_factor);


    // End line
    file_ << std::endl;
  }

  void close() {file_.close();}

private:
  std::string file_name_;
  std::ofstream file_;
};
