/*
 * This is an extension file for the Eigen library
 * @authors: Daniel Frenzel <dgdanielf@gmail.com>
 */
#include <AP_Eigen/AP_Algebra.h>

/*
 * @brief If <T> is a floating point type, 
 * this functions rounds to the closest integer and returns it as <T>.
 */
template <class IntType, class FloatType>
EIGEN_STRONG_INLINE static Matrix<IntType, _Rows, _Cols, _Options, _MaxRows, _MaxCols> get_nearest_integral(const Matrix<FloatType, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &other) {
    static_assert(std::is_floating_point<IntType>::value || std::is_floating_point<FloatType>::value, 
        "ERROR - Matrix::get_nearest_integral(): template parameters do not make sense\n");
    
    Matrix<IntType, _Rows, _Cols, _Options, _MaxRows, _MaxCols> tmp = Matrix<IntType, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Zero(_Rows, _Cols);
    //#pragma omp simd collapse(2)
    for (int y = 0; y < _Rows; y++) {
        for (int x = 0; x < _Cols; x++) {
            tmp(y,x) = safeFloatToInt<IntType>(round(other(y,x)) );
        }
    }
    return tmp;
}

/*
 * @brief This functions rounds down and returns it as IntType.
 */
template <class IntType, class FloatType>
EIGEN_STRONG_INLINE static Matrix<IntType, _Rows, _Cols, _Options, _MaxRows, _MaxCols> get_integral(const Matrix<FloatType, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &other) {
    static_assert(std::is_floating_point<IntType>::value || std::is_floating_point<FloatType>::value, 
        "ERROR - Matrix::get_integral(): template parameters do not make sense\n");
    
    Matrix<IntType, _Rows, _Cols, _Options, _MaxRows, _MaxCols> tmp = Matrix<IntType, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Zero(_Rows, _Cols);
    //#pragma omp simd collapse(2)
    for (int y = 0; y < _Rows; y++) {
        for (int x = 0; x < _Cols; x++) {
            tmp(y,x) = safeFloatToInt<IntType>(other(y,x) );
        }
    }
    return tmp;
}

/*
 * @brief constrains the values of the matrix to fit within a given range: low and high
 */
EIGEN_STRONG_INLINE void constrain(const _Scalar &low, const _Scalar &high) {
    //#pragma omp simd collapse(2)
    for (int y = 0; y < _Rows; y++) {
        for (int x = 0; x < _Cols; x++) {
            (*this)(y,x) = constrain_value<_Scalar>((*this)(y,x), low, high);
        }
    }
}

/*
 * @brief constrains the values of the matrix to fit within a given range: low and high
 */
EIGEN_STRONG_INLINE Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> constrained(const _Scalar &low, const _Scalar &high) const {
    Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> tmp = Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Zero(_Rows, _Cols);
    //#pragma omp simd collapse(2)
    for (int y = 0; y < _Rows; y++) {
        for (int x = 0; x < _Cols; x++) {
            tmp(y,x) = constrain_value<_Scalar>((*this)(y,x), low, high);
        }
    }
    return tmp;
} 

/*
 * @brief This functions constrains all euler angles in the matrix to be within the range: -180 to 180 degrees.
 */
EIGEN_STRONG_INLINE void constrain_180_deg() {
    Base::_check_template_params();
    static_assert(_Rows > 0 && _Cols > 0, "constrain_euler_180() only for vector like types.");
    
    //#pragma omp simd collapse(2)
    for (int y = 0; y < _Rows; y++) {
        for (int x = 0; x < _Cols; x++) {
            (*this)(y,x) = constrain_euler_180<_Scalar>((*this)(y,0));
        }
    } 
}

/*
 * @brief This functions constrains all euler angles in the matrix to be within the range: -180 to 180 degrees.
 */
EIGEN_STRONG_INLINE Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> constrained_180_deg() const {
    Base::_check_template_params();
    static_assert(_Rows > 0 && _Cols > 0, "constrained_euler_180() only for vector like types.");
    
    Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> tmp = Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Zero(_Rows, _Cols);
    //#pragma omp simd collapse(2)
    for (int x = 0; x < _Cols; x++) {
        for (int y = 0; y < _Rows; y++) {
            tmp(y,x) = constrain_euler_180<_Scalar>((*this)(y,0));
        }
    } 
    return tmp;
}

/*
 * @brief This functions constrains all euler angles in the matrix to be within the range: 0 to 360 degrees.
 */
EIGEN_STRONG_INLINE void constrain_360_deg() {
    Base::_check_template_params();
    static_assert(_Rows > 0 && _Cols > 0, "constrain_euler_360() only for vector like types.");
    
    //#pragma omp simd collapse(2)
    for (int y = 0; y < _Rows; y++) {
        for (int x = 0; x < _Cols; x++) {
            (*this)(y,x) = constrain_euler_360<_Scalar>((*this)(y,0));
        }
    } 
}

/*
 * @brief This functions constrains all euler angles in the matrix to be within the range: 0 to 360 degrees.
 */
EIGEN_STRONG_INLINE Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> constrained_360_deg() const {
    Base::_check_template_params();
    static_assert(_Rows > 0 && _Cols > 0, "constrained_euler_360() only for vector like types.");
    
    Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> tmp = Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Zero(_Rows, _Cols);
    //#pragma omp simd collapse(2)
    for (int y = 0; y < _Rows; y++) {
        for (int x = 0; x < _Cols; x++) {
            tmp(y,x) = constrain_euler_360<_Scalar>((*this)(y,0));
        }
    } 
    return tmp;
}

#include <AP_Math/AP_Math.h>
EIGEN_STRONG_INLINE Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &from_matrix(const Matrix3<_Scalar> &other) {
    Base::_check_template_params();
  
    Eigen::Matrix3<_Scalar, 3, 3, _Options, 3, 3> tmp;
  
    tmp(0,0) = other.a.x;
    tmp(0,1) = other.a.y;
    tmp(0,2) = other.a.z;
    
    tmp(1,0) = other.b.x;
    tmp(1,1) = other.b.y;
    tmp(1,2) = other.b.z;
    
    tmp(2,0) = other.c.x;
    tmp(2,1) = other.c.y;
    tmp(2,2) = other.c.z;

    return tmp;
}