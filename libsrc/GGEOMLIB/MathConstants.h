
#ifndef H_MATH_CONSTANT_VALUES
#define H_MATH_CONSTANT_VALUES

#pragma once

namespace ggeomlib
{
	constexpr double COINC_THRESH	 = 1.0e-10;
	constexpr double COINC_THRESH_E2 = 1.0e-20;
	constexpr double COINC_THRESH_E4 = 1.0e-40;
	constexpr double LOOSE_THRESH	 = 1.0e-4;
	constexpr double LOOSE_THRESH_E2 = 1.0e-8;
	constexpr double LOOSE_THRESH_E4 = 1.0e-16;
	constexpr double DOUBLE_PRECISION_EPSILON	 = 1.0e-6;
	constexpr double DOUBLE_PRECISION_EPSILON_E2 = 1.0e-12;

	constexpr double PI_GRECO_HALF = 1.5707963267948966192313216916398;
	constexpr double PI_GRECO = 3.1415926535897932384626433832795;
	constexpr double TWO_PI_GRECO = 6.283185307179586476925286766559;

	constexpr double DEG_TO_RAD_FACTOR = (PI_GRECO / 180);
	constexpr double RAD_TO_DEG_FACTOR = (180 / PI_GRECO);

	constexpr double VECTOR_PARALLELISM_TOLERANCE = 1.0e-10;
	constexpr double VECTOR_ORTHOGONALITY_TOLERANCE = 1.0e-10;

	constexpr double LARGE_DOUBLE_CONST = (1.0e+12);
}

#endif
