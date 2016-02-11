#pragma once

#include <limits>
#include <type_traits>
#include <cmath>

#include <math.h>
#include <stdint.h>


/*
 * A varient of asin() that checks the input ranges and ensures a
 * valid angle as output. If nan is given as input then zero is returned.
 */
template <class T>
auto safe_asin(const T &v) -> decltype(std::asin(v)) {
    static_assert(std::is_floating_point<T>::value, "ERROR - safe_asin(): template parameter not of type float\n");

    if (std::isnan(v)) {
        return 0.0f;
    }
    if (v >= 1.0f) {
        return M_PI_2;
    }
    if (v <= -1.0f) {
        return -M_PI_2;
    }
    return std::asin(v);
}

/* 
 * A varient of sqrt() that checks the input ranges and ensures a 
 * valid value as output. If a negative number is given then 0 is returned. 
 * The reasoning is that a negative number for sqrt() in our 
 * code is usually caused by small numerical rounding errors, so the 
 * real input should have been zero
 */
template <class T>
auto safe_sqrt(const T &v) -> decltype(std::sqrt(v)) {
    auto ret = std::sqrt(v);
    if (std::isnan(ret)) {
        return 0;
    }
    return ret;
}

/* 
 * @brief: Constrains an euler angle to be within the range: -180 to 180 degrees
 */
template <class T>
auto wrap_180(const T &angle, float unit_mod = 1) -> decltype(std::fmod(angle + 180.f*unit_mod, 360.f*unit_mod)) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_180(): template parameter not of type float or int\n");
    
    const auto ang_180 = 180.f*unit_mod;
    const auto ang_360 = 360.f*unit_mod;
    auto res = std::fmod(angle + ang_180, ang_360);
    if (res < 0) {
        res += ang_360;
    }
    res -= ang_180;
    return res;
}

/* 
 * @brief: Constrains an euler angle to be within the range: 0 to 360 degrees
 * The second parameter changes the units. Standard: 1 == degrees, 10 == dezi, 100 == centi ..
 */
template <class T>
auto wrap_360(const T &angle, float unit_mod = 1) -> decltype(std::fmod(angle, 360.f*unit_mod)) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_360(): template parameter not of type float or int\n");
    
    const auto ang_360 = 360.f*unit_mod;
    auto res = std::fmod(angle, ang_360);
    if (res < 0) {
        res += ang_360;
    }
    return res;
}

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
template <class T>
auto wrap_PI(const T &radian) -> decltype(std::fmod(radian + M_PI, M_2_PI)) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_PI(): template parameter not of type float or int\n");
    auto res = std::fmod(radian + M_PI, M_2_PI);
    if (res < 0.f) {
        res += M_2_PI;
    }
    res -= M_PI;
    return res;
}

/*
 * wrap an angle in radians to 0..2PI
 */
template <class T>
auto wrap_2PI(const T &radian) -> decltype(std::fmod(radian, M_2_PI)) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_2PI(): template parameter not of type float or int\n");
    auto res = std::fmod(radian, M_2_PI);
    if (res < 0.f) {
        res += M_2_PI;
    }
    return res;
} 

template<class T>
auto sq(const T &val) -> decltype(std::pow(val, 2)) {
    return std::pow(val, 2);
}
template<class T, class... Params>
auto sq(const T &first, const Params&... parameters) -> decltype(std::pow(first, 2)) {
    return sq(first) + sq(parameters...);
}
template<class T, class... Params>
auto norm(const T &first, const Params&... parameters) -> decltype(std::sqrt(sq(first, parameters...))) {
    return std::sqrt(sq(first, parameters...));
}

/*
 * @brief: Constrains a value to be within the range: low and high
 */
template <class T>
T constrain_value(const T &amt, const T &low, const T &high) {  
    if (std::isnan(low) || std::isnan(high)) {
        return amt;
    }
    return amt < low ? low : (amt > high ? high : amt);
}

/* 
 * @brief: Converts an euler angle with units 'degree' to an angle with the unit 'radian'
 */
template <class T>
auto radians(const T &deg) -> decltype(deg * DEG_TO_RAD) {
    return deg * DEG_TO_RAD;
}

/* 
 * @brief: Converts an euler angle with units 'radian' to an angle with the unit 'degree'
 */
template <class T>
auto degrees(const T &rad) -> decltype(rad * RAD_TO_DEG) {
    return rad * RAD_TO_DEG;
}