
#ifndef H_MATH_ALGS
#define H_MATH_ALGS

#include <cmath>
#include <vector>
#include <cassert>

#include "CPoint3.h"
#include "CPoint2.h"
#include "MathConstants.h"

namespace ggeomlib
{
	namespace math_algs
	{
		/**
		* @brief Returns the minimum among three double values.
		*/
		inline double FindMinOf3Numbers(double f_a,
									    double f_b,
									    double f_c)
		{
			return std::min(f_a, std::min(f_b, f_c));
		}

		/**
		* @brief Returns the maximum among three double values.
		*/
		inline double FindMaxOf3Numbers(double f_a,
									    double f_b,
									    double f_c)
		{
			return std::max(f_a, std::max(f_b, f_c));
		}

		/**
		* @brief Solves the quadratic equation a*x^2 + b*x + c = 0 for real roots.
		*
		* Uses the numerically stable form of the quadratic formula, which avoids
		* the cancellation of the textbook formula (-b +/- sqrt(delta)) / (2a) when
		* b^2 is much larger than 4ac:
		*
		*     delta = b^2 - 4*a*c
		*     q     = -1/2 * (b + sign(b) * sqrt(delta))
		*     x     = q / a   and   x = c / q
		*
		* @param[in]  f_a  Coefficient a (must be non-zero).
		* @param[in]  f_b  Coefficient b.
		* @param[in]  f_c  Coefficient c.
		* @param[out] f_x0 Larger root.
		* @param[out] f_x1 Smaller root (equal to f_x0 for a double root).
		*
		* @return true if the equation has real roots (f_x0 and f_x1 are then
		*         valid); false if delta < 0 (complex roots, outputs unchanged).
		*
		* @warning f_a must be non-zero: the linear case (a = 0) is not handled and
		*          leads to a division by zero.
		*
		* @note The roots are returned in descending order: f_x0 >= f_x1.
		*
		* @note No tolerance is applied to delta: when the exact delta is zero
		*       (double root), round-off can make the computed value slightly
		*       negative, and false is then returned.
		*
		* @see W. H. Press et al., "Numerical Recipes", 3rd ed., Cambridge
		*      University Press, 2007, section 5.6.
		*/
		inline bool SolveQuadraticEquation(const double f_a,
										   const double f_b,
										   const double f_c,
										   double& f_x0,
										   double& f_x1)
		{
			double l_discriminant = f_b * f_b - 4 * f_a * f_c;
			if (l_discriminant < 0)
			{
				return false;
			}
			else if (l_discriminant == 0)
			{
				f_x0 = f_x1 = -0.5 * f_b / f_a;
			}
			else
			{
				double l_q = 0.0;
				if (f_b > 0)
				{
					l_q = -0.5 * (f_b + sqrt(l_discriminant));
				}
				else
				{
					l_q = -0.5 * (f_b - sqrt(l_discriminant));
				}
				f_x0 = l_q / f_a;
				f_x1 = f_c / l_q;

				if (f_x0 < f_x1)
				{
					double l_f_x0_tmp = f_x0;
					f_x0 = f_x1;
					f_x1 = l_f_x0_tmp;
				}
			}

			return true;
		}

		/**
		* @brief Computes the square of a number.
		*/
		inline double Square(const double f_number_to_square)
		{
			return f_number_to_square * f_number_to_square;
		}

		/**
		* @brief Computes the cubic root of a number, handles negative input correctly.
		*/
		inline double CubicRoot(const double f_number_to_cubic_root)
		{
			if (f_number_to_cubic_root < 0.0)
			{
				return -std::pow(-f_number_to_cubic_root, 1.0 / 3.0);
			}

			return std::pow(f_number_to_cubic_root, 1.0 / 3.0);
		}

		/**
		* @brief Wraps a value within a modular range.
		*
		* @param f_value		Input value.
		* @param f_min_val		Minimum bound.
		* @param f_max_val		Maximum bound.
		* @param f_increment	Wrap step.
		* @return				Wrapped value.
		*/
		inline double Modulo(const double f_value,
							 const double f_min_val,
							 const double f_max_val,
							 const double f_increment)
		{
			double l_res_val = f_value;

			while (l_res_val > f_max_val)
			{
				l_res_val -= f_increment;
			}
			while (l_res_val < f_min_val)
			{
				l_res_val += f_increment;
			}

			return l_res_val;
		}

		/**
		* @brief Checks if all components of a vector are finite.
		*/
		inline bool IsVectorFinite(const CPoint3& f_vec)
		{
			return std::isfinite(f_vec.m_x) || std::isfinite(f_vec.m_y) || std::isfinite(f_vec.m_z);
		}

		/**
		* @brief Returns the element-wise minimum between two 3D vectors.
		*/
		inline CPoint3 MinOf2Vectors(const CPoint3& f_vec_1,
									 const CPoint3& f_vec_2)
		{
			return CPoint3(std::min(f_vec_1.m_x, f_vec_2.m_x),
						   std::min(f_vec_1.m_y, f_vec_2.m_y),
						   std::min(f_vec_1.m_z, f_vec_2.m_z));
		}
		
		/**
		* @brief Returns the element-wise maximum between two 3D vectors.
		*/
		inline CPoint3 MaxOf2Vectors(const CPoint3& f_vec_1,
									 const CPoint3& f_vec_2)
		{
			return CPoint3(std::max(f_vec_1.m_x, f_vec_2.m_x),
						   std::max(f_vec_1.m_y, f_vec_2.m_y),
						   std::max(f_vec_1.m_z, f_vec_2.m_z));
		}  

		/**
		* @brief Returns the closest divisor of f_a to f_b.
		*/
		inline int FindClosestDivisor(const int f_a, const int f_b)
		{
			std::vector<int> l_divisors;

			for (int i = 1; i <= std::sqrt(f_a); i++)
			{
				if (f_a % i == 0)
				{
					l_divisors.push_back(i);
					if (i != f_a / i)
					{
						l_divisors.push_back(f_a / i);
					}
				}
			}

			int l_closest_div = l_divisors[0];
			int l_min_diff = std::abs(l_divisors[0] - f_b);

			for (int d : l_divisors)
			{
				int l_curr_diff = std::abs(d - f_b);
				if (l_curr_diff < l_min_diff)
				{
					l_min_diff = l_curr_diff;
					l_closest_div = d;
				}
			}

			return l_closest_div;
		}

		/**
		* @brief Determines if a number is a prime.
		*/
		inline bool IsPrimeNumber(const int f_number)
		{
			if (f_number <= 1)
				return false;
			if (f_number == 2)
				return true;
			if (f_number % 2 == 0)
				return false;

			int l_num_sqrt = (int)(std::sqrt(f_number));
			for (int i = 3; i <= l_num_sqrt; i += 2)
			{
				if (f_number % i == 0)
					return false;
			}
			return true;
		}

		/**
		* @brief Finds the next non-prime integer greater than the input.
		*/
		inline int FindNextNonPrimeNumber(int f_number)
		{
			int l_next = f_number + 1;
			while (IsPrimeNumber(l_next))
			{
				l_next++;
			}
			return l_next;
		}

		/**
		* @brief Returns the integer sign of a double.
		*
		* @return +1 if positive, -1 if negative, 0 if zero.
		*/
		inline int IntegerSign(const double f_arg)
		{
			if (f_arg > 0) return(1);
			if (f_arg < 0) return(-1);
			return(0);
		}

		/**
		* @brief Evaluates a 2D circle equation (x - cx)^2 + (y - cy)^2 - r^2.
		*
		* @return Positive if point is outside, zero if on the circle, negative if inside.
		*/
		inline double EvaluateCircleFunction(const CPoint2& f_circle_center,
										     const double f_radius,
										     const double f_px,
										     const double f_py)
		{
			return pow(f_px - f_circle_center.m_x, 2) + pow(f_py - f_circle_center.m_y, 2) - pow(f_radius, 2);
		}

		/**
		* @brief Constructs two unit vectors perpendicular to a given unit vector, forming a right-handed 
		*		 orthonormal basis.
		*
		* Given a normalized input vector f_axis, this function computes two additional unit vectors,
		* f_perp_1 and f_perp_2, such that:
		* - f_perp_1 is perpendicular to f_axis
		* - f_perp_2 is perpendicular to both f_axis and f_perp_1
		* - The set {f_axis, f_perp_1, f_perp_2} forms a right-handed orthonormal basis.
		*
		* The method chooses an appropriate vector to construct f_perp_1 using a stable heuristic,
		* depending on the dominant component of f_axis. This avoids degeneracy when the input is close
		* to the coordinate axes.
		*
		* @param[in]  f_axis	A unit vector defining the main axis (assumed normalized).
		* @param[out] f_perp_1	A unit vector perpendicular to f_axis
		* @param[out] f_perp_2	A unit vector perpendicular to both f_axis and f_perp_1
		*/
		inline void BuildOrthonormalBasisFromAxis(const CPoint3& f_axis,
												  CPoint3& f_perp_1,
												  CPoint3& f_perp_2)
		{
			if (std::abs(f_axis.m_x) > std::abs(f_axis.m_z))
			{
				f_perp_1 = CPoint3(-f_axis.m_y, f_axis.m_x, 0.0).UnitizeInPlace();
			}
			else
			{
				f_perp_1 = CPoint3(0.0f, -f_axis.m_z, f_axis.m_y).UnitizeInPlace();
			}
			f_perp_2 = Cross(f_axis, f_perp_1);
		}

		/**
		* @brief Performs spherical linear interpolation (SLERP) between two unit vectors.
		*
		* This function interpolates smoothly between two unit direction vectors f_vec_0 and f_vec_1
		* using spherical linear interpolation, maintaining constant angular velocity.
		*
		* Unlike linear interpolation (LERP), which can cause shrinking, distortion, and non-uniform
		* rotation, SLERP ensures a smooth, uniform rotation across the interpolation interval, making
		* it ideal for applications such as smoothly rotating normals, orientations, or directional vectors.
		*
		* If the angle between the vectors is extremely small (below a defined epsilon),
		* the function returns the starting vector directly to avoid numerical instability.
		*
		* @param[in] f_vec_0 The starting unit vector.
		* @param[in] f_vec_1 The ending unit vector.
		* @param[in] f_t Interpolation factor in [0, 1], where 0 returns f_vec_0 and 1 returns f_vec_1
		*
		* @return A unit vector representing the spherical interpolation between f_vec_0 and f_vec_1
		*/
		inline CPoint3 Slerp(const CPoint3& f_vec_0, 
							 const CPoint3& f_vec_1, 
							 const double f_t)
		{
			double l_dot = std::max(-1.0, std::min(1.0, Dot(f_vec_0, f_vec_1)));
			if (l_dot < -0.999999)
			{
				CPoint3 l_ortho = CPoint3(1, 0, 0);
				if (std::abs(f_vec_0.m_x) > 0.9)
				{
					l_ortho = CPoint3(0, 1, 0);
				}
				l_ortho = (l_ortho - f_vec_0 * Dot(f_vec_0, l_ortho)).UnitizeInPlace();

				return (f_vec_0 * cos(PI_GRECO * f_t) + l_ortho * sin(PI_GRECO * f_t)).UnitizeInPlace();
			}
			double l_angle = std::acos(l_dot);

			if (l_angle < DOUBLE_PRECISION_EPSILON)
			{
				return f_vec_0;
			}

			double l_sin_angle = std::sin(l_angle);
			double l_a = std::sin((1 - f_t) * l_angle) / l_sin_angle;
			double l_b = std::sin(f_t * l_angle) / l_sin_angle;

			return (f_vec_0 * l_a + f_vec_1 * l_b).UnitizeInPlace();
		}

		/**
		* @brief Performs linear interpolation (LERP) between two vectors.
		*
		* Computes a linear interpolation between vectors f_vec_0 and f_vec_1 based on the interpolation
		* param f_t. This function does not assume unit vectors and does not maintain constant angular velocity.
		*
		* Unlike spherical interpolation (SLERP), LERP can cause distortion when used with unit vectors,
		* especially for directional data, but it's faster and sufficient for many simple interpolations.
		*
		* @param[in] f_vec_0 The starting vector.
		* @param[in] f_vec_1 The ending vector.
		* @param[in] f_t Interpolation factor in [0, 1], where 0 returns f_vec_0 and 1 returns f_vec_1
		*
		* @return A linearly interpolated vector between f_vec_0 and f_vec_1
		*/
		inline CPoint3 Lerp(const CPoint3& f_vec_0, 
							const CPoint3& f_vec_1, 
							const double f_t)
		{
			return f_vec_0 * (1.0 - f_t) + f_vec_1 * f_t;
		}

		/**
		* @brief Estimates the point that minimizes the quadratic interpolating three
		*        points.
		*
		* Computes the x-coordinate of the vertex of the second-order (Lagrange)
		* polynomial interpolating (p1, f1), (p2, f2) and (p3, f3):
		*
		*     x = [ f1 (p3^2 - p2^2) + f2 (p1^2 - p3^2) + f3 (p2^2 - p1^2) ]
		*         / ( 2 [ f1 (p3 - p2) + f2 (p1 - p3) + f3 (p2 - p1) ] )
		*
		* Under the preconditions below (a bracketed minimum), the parabola opens
		* upward and its vertex lies in [p1, p3]. This is a typical step of line
		* search methods. The result is clamped to [p1, p3] to guard against
		* round-off or violated preconditions.
		*
		* @param f_p1 x-coordinate of the first point.
		* @param f_p2 x-coordinate of the second point.
		* @param f_p3 x-coordinate of the third point.
		* @param f_f1 Function value at f_p1.
		* @param f_f2 Function value at f_p2.
		* @param f_f3 Function value at f_p3.
		*
		* @pre f_p1 < f_p2 < f_p3
		* @pre f_f1 >= f_f2 and f_f2 <= f_f3 (the minimum is bracketed)
		*
		* @return The x-coordinate of the parabola's vertex, clamped to
		*         [f_p1, f_p3]; f_p2 if the denominator is zero (the three points
		*         are collinear, so the interpolant is a line with no minimum).
		*
		* @see A. Ruszczy?ski, "Nonlinear Optimization", Princeton University
		*      Press, 2006, section 5.2.
		*
		* @note Adapted from lagrange_poly_min_extrap() in the dlib C++ library by
		*       Davis E. King (https://github.com/davisking/dlib), Boost Software
		*       License 1.0, Copyright (C) 2008 Davis E. King.
		*		(https://github.com/davisking/dlib/blob/master/dlib/optimization/optimization_line_search.h)
		*/
        inline double LagrangePolyMinExtrap(double f_p1,
										    double f_p2,
										    double f_p3,
										    double f_f1,
										    double f_f2,
										    double f_f3)
		{
			double l_temp1 = f_f1 * (f_p3 * f_p3 - f_p2 * f_p2) + f_f2 * (f_p1 * f_p1 - f_p3 * f_p3) + f_f3 * (f_p2 * f_p2 - f_p1 * f_p1);
			double l_temp2 = 2 * (f_f1 * (f_p3 - f_p2) + f_f2 * (f_p1 - f_p3) + f_f3 * (f_p2 - f_p1));

			if (l_temp2 == 0)
			{
				return f_p2;
			}

			const double l_result = l_temp1 / l_temp2;

			if (f_p1 <= l_result && l_result <= f_p3)
			{
				return l_result;
			}
			else
			{
				return std::min(std::max(f_p1, l_result), f_p3);
			}
		}

		/**
		* @brief Finds a local minimum of a single-variable function, using
		*        bracketing followed by safeguarded quadratic interpolation.
		*
		* The algorithm works in two phases:
		* 1. Bracketing: starting from three points around f_starting_point, the
		*    search interval is expanded (doubling the step each time) or shrunk
		*    until three points p1 < p2 < p3 with f(p1) > f(p2) < f(p3) are found,
		*    which guarantees a local minimum in [p1, p3].
		* 2. Refinement: the bracket is narrowed by evaluating the function at the
		*    vertex of the parabola through the three points
		*    (LagrangePolyMinExtrap()), with safeguards against points too close
		*    to the existing ones and against very unbalanced brackets, until the
		*    bracket is smaller than f_convergence_threshold.
		*
		* The search never leaves [f_interval_low_bound, f_interval_max_bound]; if
		* the minimum lies on a bound, the bracket shrinks against it.
		*
		* @tparam funct Callable type taking a double and returning a double:
		*               function pointer, lambda, functor or
		*               std::function<double(double)>.
		*
		* @param[in]     f_funct_to_min          Function to minimize.
		* @param[in,out] f_starting_point        Initial guess, within the search
		*                                        interval. On success, replaced by
		*                                        the estimated minimum point.
		* @param[out]    f_min_value             On success, the function value at
		*                                        the estimated minimum point.
		* @param[in]     f_interval_low_bound    Lower bound of the search interval
		*                                        (default -1e200, i.e. unbounded).
		* @param[in]     f_interval_max_bound    Upper bound of the search interval
		*                                        (default 1e200, i.e. unbounded).
		* @param[in]     f_convergence_threshold The search stops when the bracket
		*                                        is narrower than this (default
		*                                        1e-3).
		* @param[in]     f_max_iter_num          Maximum number of iterations,
		*                                        roughly the number of function
		*                                        evaluations (default 100).
		* @param[in]     f_initial_search_radius Initial distance from
		*                                        f_starting_point of the first two
		*                                        points (default 1).
		*
		* @return true if the minimum was located within the convergence threshold;
		*         false if the iteration limit was reached first (f_starting_point
		*         and f_min_value are then unchanged). If the two bounds are equal,
		*         the function is just evaluated at f_starting_point and true is
		*         returned.
		*
		* @pre f_interval_low_bound <= f_starting_point <= f_interval_max_bound
		*
		* @note A local minimum is found; it is the global one only if the function
		*       is unimodal in the search interval.
		*
		* @par Examples
		* @code
		* // Lambda
		* auto quadratic = [](double x) { return x * x + 2 * x + 1; }; // (x + 1)^2
		* double x0 = 0.0;
		* double fmin = 0.0;
		* FindMinSingleVariableFunction(quadratic, x0, fmin);
		* // x0 is close to -1, fmin close to 0
		*
		* // Function pointer
		* double MyFunction(double x) { return std::sin(x); }
		* double start = 2.0;
		* double min_val = 0.0;
		* FindMinSingleVariableFunction(MyFunction, start, min_val);
		*
		* // Functor
		* struct MyFunctor
		* {
		*     double operator()(double x) const { return std::exp(x) + x * x; }
		* };
		* double x_start = 0.5;
		* double f_min = 0.0;
		* FindMinSingleVariableFunction(MyFunctor(), x_start, f_min);
		*
		* // std::function
		* std::function<double(double)> f = [](double x) { return (x - 3) * (x - 3) + 2; };
		* double start_point = 0.0;
		* double minimum_value = 0.0;
		* FindMinSingleVariableFunction(f, start_point, minimum_value);
		* @endcode
		*
		* @note Adapted from find_min_single_variable() in the dlib C++ library by
		*       Davis E. King (https://github.com/davisking/dlib,
		*		(https://github.com/davisking/dlib/blob/master/dlib/optimization/optimization_line_search.h)
		*       dlib/optimization/optimization_line_search.h), Boost Software
		*       License 1.0, Copyright (C) 2008 Davis E. King. Changes from the
		*       original: failure reported through the return value instead of an
		*       exception, GGEOMLIB naming conventions.
		*/
		template <typename funct>
		bool FinDMinSingleVariableFunction(const funct& f_funct_to_min,
										   double& f_starting_point,
										   double& f_min_value,
										   const double f_interval_low_bound = -1e200,
										   const double f_interval_max_bound = 1e200,
										   const double f_convergence_threshold = 1e-3,
										   const long f_max_iter_num = 100,
										   const double f_initial_search_radius = 1)
		{
			double l_search_radius = f_initial_search_radius;
			double l_p1 = 0;
			double l_p2 = 0;
			double l_p3 = 0;
			double l_f1 = 0;
			double l_f2 = 0;
			double l_f3 = 0;
			long l_iters_num = 1;

			if (f_interval_low_bound == f_interval_max_bound)
			{
				f_min_value = f_funct_to_min(f_starting_point);
				return true;
			}


			l_p1 = std::max(f_starting_point - l_search_radius, f_interval_low_bound);
			l_p3 = std::min(f_starting_point + l_search_radius, f_interval_max_bound);
			l_f1 = f_funct_to_min(l_p1);
			l_f3 = f_funct_to_min(l_p3);

			if (f_starting_point == l_p1 || f_starting_point == l_p3)
			{
				l_p2 = (l_p1 + l_p3) / 2;
				l_f2 = f_funct_to_min(l_p2);
			}
			else
			{
				l_p2 = f_starting_point;
				l_f2 = f_funct_to_min(f_starting_point);
			}

			l_iters_num += 2;

			while (!(l_f1 > l_f2 && l_f2 < l_f3))
			{
				if (l_iters_num >= f_max_iter_num)
				{
					return false;
				}
				if (l_p3 - l_p1 < f_convergence_threshold)
				{
					if (l_f1 < std::min(l_f2, l_f3))
					{
						f_starting_point = l_p1;
						f_min_value = l_f1;
						return true;
					}

					if (l_f2 < std::min(l_f1, l_f3))
					{
						f_starting_point = l_p2;
						f_min_value = l_f2;
						return true;
					}

					f_starting_point = l_p3;
					f_min_value = l_f3;
					return true;
				}

				if (l_f1 == l_f2 && l_f1 < l_f3 && l_p1 != f_interval_low_bound)
				{
					l_p1 = std::max(l_p1 - l_search_radius, f_interval_low_bound);
					l_f1 = f_funct_to_min(l_p1);
					++l_iters_num;
					l_search_radius *= 2;
					continue;
				}
				if (l_f2 == l_f3 && l_f3 < l_f1 && l_p3 != f_interval_max_bound)
				{
					l_p3 = std::min(l_p3 + l_search_radius, f_interval_max_bound);
					l_f3 = f_funct_to_min(l_p3);
					++l_iters_num;
					l_search_radius *= 2;
					continue;
				}

				if (l_f1 <= l_f3)
				{
					if (l_p1 == f_interval_low_bound || (l_f1 == l_f2 && (f_interval_max_bound - f_interval_low_bound) < l_search_radius))
					{
						l_p3 = l_p2;
						l_f3 = l_f2;

						l_p2 = (l_p1 + l_p2) / 2.0;
						l_f2 = f_funct_to_min(l_p2);
					}
					else
					{
						l_p3 = l_p2;
						l_f3 = l_f2;

						l_p2 = l_p1;
						l_f2 = l_f1;

						l_p1 = std::max(l_p1 - l_search_radius, f_interval_low_bound);
						l_f1 = f_funct_to_min(l_p1);

						l_search_radius *= 2;
					}
				}
				else
				{
					if (l_p3 == f_interval_max_bound || (l_f2 == l_f3 && (f_interval_max_bound - f_interval_low_bound) < l_search_radius))
					{
						l_p1 = l_p2;
						l_f1 = l_f2;

						l_p2 = (l_p3 + l_p2) / 2.0;
						l_f2 = f_funct_to_min(l_p2);
					}
					else
					{
						l_p1 = l_p2;
						l_f1 = l_f2;

						l_p2 = l_p3;
						l_f2 = l_f3;

						l_p3 = std::min(l_p3 + l_search_radius, f_interval_max_bound);
						l_f3 = f_funct_to_min(l_p3);

						l_search_radius *= 2;
					}
				}
				++l_iters_num;
			}

			const double l_tau = 0.1;
			while (l_iters_num < f_max_iter_num && l_p3 - l_p1 > f_convergence_threshold)
			{
				double l_p_min = LagrangePolyMinExtrap(l_p1, l_p2, l_p3, l_f1, l_f2, l_f3);
				if (l_p_min < l_p2)
				{
					const double min_dist = (l_p2 - l_p1) * l_tau;
					if (abs(l_p1 - l_p_min) < min_dist)
					{
						l_p_min = l_p1 + min_dist;
					}
					else if (abs(l_p2 - l_p_min) < min_dist)
					{
						l_p_min = l_p2 - min_dist;
					}
				}
				else
				{
					const double min_dist = (l_p3 - l_p2) * l_tau;
					if (abs(l_p2 - l_p_min) < min_dist)
					{
						l_p_min = l_p2 + min_dist;
					}
					else if (abs(l_p3 - l_p_min) < min_dist)
					{
						l_p_min = l_p3 - min_dist;
					}
				}

				const double bracket_ratio = abs(l_p1 - l_p2) / abs(l_p2 - l_p3);
				if (bracket_ratio >= 10)
				{
					if (l_p_min > l_p2)
						l_p_min = (l_p1 + l_p2) / 2;
				}
				else if (bracket_ratio <= 0.1)
				{
					if (l_p_min < l_p2)
						l_p_min = (l_p2 + l_p3) / 2;
				}
				const double l_f_min = f_funct_to_min(l_p_min);
				if (l_p_min < l_p2)
				{
					if (l_f1 > l_f_min && l_f_min < l_f2)
					{
						l_p3 = l_p2;
						l_f3 = l_f2;
						l_p2 = l_p_min;
						l_f2 = l_f_min;
					}
					else
					{
						l_p1 = l_p_min;
						l_f1 = l_f_min;
					}
				}
				else
				{
					if (l_f2 > l_f_min && l_f_min < l_f3)
					{
						l_p1 = l_p2;
						l_f1 = l_f2;
						l_p2 = l_p_min;
						l_f2 = l_f_min;
					}
					else
					{
						l_p3 = l_p_min;
						l_f3 = l_f_min;
					}
				}
				++l_iters_num;
			}

			if (l_iters_num >= f_max_iter_num)
			{
				return false;
			}

			f_starting_point = l_p2;
			f_min_value = l_f2;
			return true;
		}

		/**
		* @brief Performs linear interpolation of a property at a specified abscissa value.
		*
		* This function takes a sorted vector of (Abscissa, Property) pairs and performs
		* linear interpolation to estimate the property value at a given abscissa value.
		*
		* If the f_abscissa_to_interp_at lies within one of the intervals in the data,
		* the property is interpolated using a linear formula between the two bounding points.
		*
		* @param f_data_to_interpolate A sorted vector of CPoint2, where each element
		*        contains an abscissa (m_x) and its corresponding property (m_y).
		* @param f_abscissa_to_interp_at The data value at which the property value is to be interpolated.
		*
		* @return False in case of error in calculation, true otherwise
		*/
		inline bool InterpolateProperty(const std::vector<CPoint2>& f_data_to_interpolate,
									    double f_abscissa_to_interp_at,
									    double& f_interpolated_prop)
		{
			f_interpolated_prop = 0.0;
			for (size_t i = 0; i < f_data_to_interpolate.size() - 1; ++i)
			{
				double T1 = f_data_to_interpolate[i].m_x;
				double T2 = f_data_to_interpolate[i + 1].m_x;
				if (f_abscissa_to_interp_at >= T1 && f_abscissa_to_interp_at <= T2)
				{
					double P1 = f_data_to_interpolate[i].m_y;
					double P2 = f_data_to_interpolate[i + 1].m_y;
					f_interpolated_prop = P1 + (P2 - P1) * (f_abscissa_to_interp_at - T1) / (T2 - T1);
					return true;
				}
			}
			return false;
		}

		/**
		* @brief Computes the arc cosine (inverse cosine) of a value with bounds checking.
		*
		* If the input is slightly outside the valid range of [-1.0, 1.0], it is clamped to the nearest limit.
		* An assertion is triggered if the value is out-of-bounds by more than f_over_one_allowed_limit.
		*
		* @param f_cos The input value representing a cosine.
		* @param f_over_one_allowed_limit Maximum tolerance for values slightly out of [-1.0, 1.0]. Default is NUMERIC_THRESHOLD.
		* @return A value in the range [0.0, VALGEBRA_PI_GRECO].
		*
		* @note Examples:
		* - If f_cos =  1.00004, returns acos(1.0) == 0.0.
		* - If f_cos = -1.00004, returns acos(-1.0) == VALGEBRA_PI_GRECO.
		*/
		double GuardedACos(const double& f_cos, const double f_over_one_allowed_limit = DOUBLE_PRECISION_EPSILON)
		{
			assert(f_cos >= -1.0 - f_over_one_allowed_limit && f_cos <= 1.0 + f_over_one_allowed_limit);

			return acos(std::clamp(f_cos, -1.0, 1.0));
		}

		/**
		* @brief Computes the arc sine (inverse sine) of a value with bounds checking.
		*
		* If the input is slightly outside the valid range of [-1.0, 1.0], it is clamped to the nearest limit.
		* An assertion is triggered if the value is out-of-bounds by more than f_over_one_allowed_limit.
		*
		* @param f_sin The input value representing a sine.
		* @param f_over_one_allowed_limit Maximum tolerance for values slightly out of [-1.0, 1.0]. Default is NUMERIC_THRESHOLD.
		* @return A value in the range [-ALGEBRA_PI2, ALGEBRA_PI2].
		*
		* @note Examples:
		* - If f_sin =  1.00004, returns asin(1.0) == ALGEBRA_PI2.
		* - If f_sin = -1.00004, returns asin(-1.0) == -ALGEBRA_PI2.
		*/
		double GuardedASin(const double& f_sin, const double f_over_one_allowed_limit = DOUBLE_PRECISION_EPSILON)
		{
			assert(f_sin >= -1.0 - f_over_one_allowed_limit && f_sin <= 1.0 + f_over_one_allowed_limit);

			return asin(std::clamp(f_sin, -1.0, 1.0));
		}
	}
}

#endif