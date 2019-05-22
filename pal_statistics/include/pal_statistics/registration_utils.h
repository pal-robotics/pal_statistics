/**
 * Copyright (C) 2019 PAL Robotics S.L.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
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
