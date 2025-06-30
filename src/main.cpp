/*!*******************************************************************************************
 *  \file       main.cpp
 *  \brief      Executable to test the module
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
#include <iostream>
#include "param_estimation/param_estimation.hpp"
#include "utils/csv_saver.hpp"


int main()
{
  ParamEstimation param_estimation(1.52);


  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream timestamp_stream;
  timestamp_stream << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
  std::string timestamp = timestamp_stream.str();
  std::string file_name = "mass_estimation_results_" + timestamp + ".csv";
  CsvSaver logger(file_name);
  logger.save(now_c, param_estimation);


  // (Opcional) ejecutar alguna funcionalidad
  std::cout << "Ejecutando estimador y logger..." << std::endl;
  float thrust = 10.0; // Fuerza de empuje en Newtons
  float acceleration = 2.0;   // Aceleración en m/s^2
  param_estimation.computeAll(thrust, acceleration);
  return 0;

}
