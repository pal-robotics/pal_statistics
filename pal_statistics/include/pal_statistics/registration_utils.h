/**
 *
 * MIT License
 * 
 * Copyright (c) 2019 PAL Robotics S.L.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
**/

#ifndef REGISTRATION_UTILS_H
#define REGISTRATION_UTILS_H

#include <pal_statistics/pal_statistics.h>

namespace pal_statistics
{
/**
 * @brief Default implementation that accepts anything variable that can be casted to a
 * double
 */
template <typename T>
inline IdType customRegister(StatisticsRegistry &registry, const std::string &name, const T * variable,
                        RegistrationsRAII *bookkeeping = NULL, bool enabled = true)
{
  boost::function<double()> funct = [variable] { return static_cast<double>(*variable); };
  return registry.registerFunction(name, funct, bookkeeping, enabled);
}

/**
 * @brief specialization for where the variable is double, to avoid going through a
 * function
 */
template <>
inline IdType customRegister(StatisticsRegistry &registry, const std::string &name, const double * variable,
                        RegistrationsRAII *bookkeeping, bool enabled)
{
  return registry.registerVariable(name, variable, bookkeeping, enabled);
}



/**
 * @brief Default implementation that accepts any function whose return value can be
 * casted to a double
 */
template <typename T>
inline IdType customRegister(StatisticsRegistry &registry, const std::string &name,
                        const boost::function<T()> &funct,
                        RegistrationsRAII *bookkeeping = NULL, bool enabled = true)
{
  boost::function<double()> double_funct = [funct] { return static_cast<double>(funct()); };
  return registry.registerFunction(name, double_funct, bookkeeping, enabled);
}


/**
 * @brief specialization for where the variable is double, to avoid going through a
 * function
 */
template <>
inline IdType customRegister(StatisticsRegistry &registry, const std::string &name,
                        const boost::function<double()> &funct,
                        RegistrationsRAII *bookkeeping, bool enabled)
{
  return registry.registerFunction(name, funct, bookkeeping, enabled);
}

/**
 * These functions can be extended to enable the registration of your own custom types.
 * Just include the files that extend these functions before the REGISTER_VARIABLE_MACROS
 */

}  // namespace pal_statistics

#endif  // REGISTRATION_UTILS_H
