// Copyright 2024 Moritz Schmidt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SOAR_ROS__MSG__DETAIL_HPP_
#define SOAR_ROS__MSG__DETAIL_HPP_

#include <sml_Client.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

namespace soar_ros::msg
{

    /// @brief Primary template declaration for Soar → ROS conversion.
    ///
    /// Explicit specializations are provided in std_msgs_converters.hpp and
    /// geometry_msgs_converters.hpp.  Always include via soar_ros/msg/converters.hpp
    /// so that all specializations are visible together.
    template <typename T>
    T fromSoar(sml::Identifier *id);

    namespace detail
    {

        /// @brief Read a float-typed child WME attribute as double.
        /// Returns 0.0 when the attribute is absent.
        inline double getFloat(sml::Identifier *id, const char *attr)
        {
            const char *v = id->GetParameterValue(attr);
            return v ? std::stod(v) : 0.0;
        }

        /// @brief Read an int-typed child WME attribute as int64_t.
        /// Returns 0 when the attribute is absent.
        inline int64_t getInt(sml::Identifier *id, const char *attr)
        {
            const char *v = id->GetParameterValue(attr);
            return v ? std::stoll(v) : int64_t{0};
        }

        /// @brief Read a string-typed child WME attribute.
        /// Returns an empty string when the attribute is absent.
        inline std::string getString(sml::Identifier *id, const char *attr)
        {
            const char *v = id->GetParameterValue(attr);
            return v ? std::string(v) : std::string{};
        }

        /// @brief Return the child Identifier for the given attribute, or nullptr.
        inline sml::Identifier *getChild(sml::Identifier *id, const char *attr)
        {
            auto *wme = id->FindByAttribute(attr, 0);
            return wme ? wme->ConvertToIdentifier() : nullptr;
        }

        /// @brief Write a numeric vector as indexed WMEs plus a count WME.
        ///
        /// Produces: @p prefix _count (int), @p prefix _0, @p prefix _1, … \n
        /// Float types use FloatWME; integral types use IntWME.
        template <typename T>
        inline void writeNumericArray(
            sml::Identifier *id, const char *prefix, const std::vector<T> &v)
        {
            std::string pfx{prefix};
            id->CreateIntWME((pfx + "_count").c_str(), static_cast<int64_t>(v.size()));
            for (std::size_t i = 0; i < v.size(); ++i)
            {
                auto key = (pfx + "_" + std::to_string(i)).c_str();
                if constexpr (std::is_floating_point_v<T>)
                    id->CreateFloatWME(key, static_cast<double>(v[i]));
                else
                    id->CreateIntWME(key, static_cast<int64_t>(v[i]));
            }
        }

        /// @brief Read a numeric vector from indexed WMEs written by writeNumericArray.
        template <typename T>
        inline std::vector<T> readNumericArray(sml::Identifier *id, const char *prefix)
        {
            std::string pfx{prefix};
            int64_t count = getInt(id, (pfx + "_count").c_str());
            std::vector<T> v;
            v.reserve(static_cast<std::size_t>(count));
            for (int64_t i = 0; i < count; ++i)
            {
                auto key = (pfx + "_" + std::to_string(i)).c_str();
                if constexpr (std::is_floating_point_v<T>)
                    v.push_back(static_cast<T>(getFloat(id, key)));
                else
                    v.push_back(static_cast<T>(getInt(id, key)));
            }
            return v;
        }

        /// @brief Write a string vector as indexed StringWMEs plus a count IntWME.
        inline void writeStringArray(
            sml::Identifier *id, const char *prefix, const std::vector<std::string> &v)
        {
            std::string pfx{prefix};
            id->CreateIntWME((pfx + "_count").c_str(), static_cast<int64_t>(v.size()));
            for (std::size_t i = 0; i < v.size(); ++i)
                id->CreateStringWME(
                    (pfx + "_" + std::to_string(i)).c_str(), v[i].c_str());
        }

        /// @brief Read a string vector from indexed WMEs written by writeStringArray.
        inline std::vector<std::string> readStringArray(
            sml::Identifier *id, const char *prefix)
        {
            std::string pfx{prefix};
            int64_t count = getInt(id, (pfx + "_count").c_str());
            std::vector<std::string> v;
            v.reserve(static_cast<std::size_t>(count));
            for (int64_t i = 0; i < count; ++i)
                v.push_back(getString(id, (pfx + "_" + std::to_string(i)).c_str()));
            return v;
        }

    } // namespace detail
} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__DETAIL_HPP_
