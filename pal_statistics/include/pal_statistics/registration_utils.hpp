// Copyright 2020 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef PAL_STATISTICS__REGISTRATION_UTILS_HPP_
#define PAL_STATISTICS__REGISTRATION_UTILS_HPP_

#include <string>

#include "pal_statistics/pal_statistics.hpp"

namespace pal_statistics
{
/**
 * @brief Default implementation that accepts anything variable that can be casted to a
 * double
 */
template<typename T>
inline IdType customRegister(
  StatisticsRegistry & registry, const std::string & name, const T * variable,
  RegistrationsRAII * bookkeeping = NULL, bool enabled = true)
{
  std::function<double()> funct = [variable] {return static_cast<double>(*variable);};
  return registry.registerFunction(name, funct, bookkeeping, enabled);
}

/**
 * @brief specialization for where the variable is double, to avoid going through a
 * function
 */
template<>
inline IdType customRegister(
  StatisticsRegistry & registry, const std::string & name, const double * variable,
  RegistrationsRAII * bookkeeping, bool enabled)
{
  return registry.registerVariable(name, variable, bookkeeping, enabled);
}


/**
 * @brief Default implementation that accepts any function whose return value can be
 * casted to a double
 */
template<typename T>
inline IdType customRegister(
  StatisticsRegistry & registry, const std::string & name,
  const std::function<T()> & funct,
  RegistrationsRAII * bookkeeping = NULL, bool enabled = true)
{
  std::function<double()> double_funct = [funct] {return static_cast<double>(funct());};
  return registry.registerFunction(name, double_funct, bookkeeping, enabled);
}


/**
 * @brief specialization for where the variable is double, to avoid going through a
 * function
 */
template<>
inline IdType customRegister(
  StatisticsRegistry & registry, const std::string & name,
  const std::function<double()> & funct,
  RegistrationsRAII * bookkeeping, bool enabled)
{
  return registry.registerFunction(name, funct, bookkeeping, enabled);
}

/**
 * These functions can be extended to enable the registration of your own custom types.
 * Just include the files that extend these functions before the REGISTER_VARIABLE_MACROS
 */

}  // namespace pal_statistics

#endif  // PAL_STATISTICS__REGISTRATION_UTILS_HPP_
