
#ifndef H_CHERMITE
#define H_CHERMITE

#pragma once
#include "CPoint3.h"
#include <vector>

namespace ggeomlib
{
	/**
	* @class HermiteCurve
	* @brief Represents a cubic Hermite curve in 3D space.
	*
	* Defined by:
	*   P0, P1  — start and end positions
	*   M0, M1  — tangent vectors at P0 and P1 (magnitude affects shape, bigger magnitude makes
	*			  the curve "follow" the tangent for a longer length)
	*
	* Evaluated via the four Hermite basis polynomials:
	*   h00(t) =  2t^3 - 3t^2 + 1
	*   h10(t) =   t^3 - 2t^2 + t
	*   h01(t) = -2t^3 + 3t^2
	*   h11(t) =   t^3 -  t^2
	*
	*   P(t) = h00(t)*P0 + h10(t)*M0 + h01(t)*P1 + h11(t)*M1
	*/
	class CHermiteCurve
	{
	public:

		/**
		* @brief Construct a Hermite curve from two points and their tangents.
		* @param f_p0 Start position
		* @param f_p1 End position
		* @param f_m0 Tangent at P0 (direction AND magnitude matter)
		* @param f_m1 Tangent at P1 (direction AND magnitude matter)
		*/
		CHermiteCurve(const CPoint3& f_p0,
					  const CPoint3& f_p1,
					  const CPoint3& f_m0,
					  const CPoint3& f_m1) : 
							m_p0(f_p0),
							m_p1(f_p1),
							m_m0(f_m0),
							m_m1(f_m1)
		{

		}

		/**
		* @brief Evaluate the curve at parameter t in [0, 1].
		* @param f_t  Parameter value (0 = P0, 1 = P1)
		* @return     Point on the curve at t
		*/
		inline CPoint3 Evaluate(const double f_t) const
		{
			const double t = f_t;
			const double t2 = t * t;
			const double t3 = t2 * t;

			const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
			const double h10 = t3 - 2.0 * t2 + t;
			const double h01 = -2.0 * t3 + 3.0 * t2;
			const double h11 = t3 - t2;

			return m_p0 * h00 + m_m0 * h10 + m_p1 * h01 + m_m1 * h11;
		}

		/**
		* @brief Evaluate the first derivative (tangent) at parameter t.
		* @param f_t  Parameter value in [0, 1]
		* @return     Tangent vector at t
		*/
		inline CPoint3 Derivative(const double f_t) const
		{
			const double t = f_t;
			const double t2 = t * t;

			const double dh00 = 6.0 * t2 - 6.0 * t;
			const double dh10 = 3.0 * t2 - 4.0 * t + 1.0;
			const double dh01 = -6.0 * t2 + 6.0 * t;
			const double dh11 = 3.0 * t2 - 2.0 * t;

			return m_p0 * dh00 + m_m0 * dh10 + m_p1 * dh01 + m_m1 * dh11;
		}

		/**
		* @brief Sample the curve into a polyline of evenly-spaced t values.
		* @param f_num_segments  Number of segments (points = segments + 1)
		* @return                Vector of points along the curve
		*/
		inline std::vector<CPoint3> Sample(const int f_num_segments) const
		{
			std::vector<CPoint3> l_points;
			l_points.reserve(f_num_segments + 1);

			const double step = 1.0 / static_cast<double>(f_num_segments);
			for (int i = 0; i <= f_num_segments; ++i)
			{
				l_points.push_back(Evaluate(i * step));
			}

			return l_points;
		}

		/**
		* @brief Returns the start point of the curve.
		*/
		inline const CPoint3& GetP0() const
		{
			return m_p0;
		}

		/**
		* @brief Returns the end point of the curve.
		*/
		inline const CPoint3& GetP1() const
		{
			return m_p1;
		}

		/**
		* @brief Returns the tangent vector at the start point P0.
		* @note  The magnitude of this vector affects the shape of the curve.
		*/
		inline const CPoint3& GetM0() const
		{
			return m_m0;
		}

		/**
		* @brief Returns the tangent vector at the end point P1.
		* @note  The magnitude of this vector affects the shape of the curve.
		*/
		inline const CPoint3& GetM1() const
		{
			return m_m1;
		}

		/**
		* @brief Sets the start point of the curve.
		* @param f_p0 New start point.
		*/
		inline void SetP0(const CPoint3& f_p0)
		{
			m_p0 = f_p0;
		}

		/**
		* @brief Sets the end point of the curve.
		* @param f_p1 New end point.
		*/
		inline void SetP1(const CPoint3& f_p1)
		{
			m_p1 = f_p1;
		}

		/**
		* @brief Sets the tangent vector at the start point P0.
		* @param f_m0 New tangent at P0. Direction and magnitude both affect the curve shape.
		*/
		inline void SetM0(const CPoint3& f_m0)
		{
			m_m0 = f_m0;
		}

		/**
		* @brief Sets the tangent vector at the end point P1.
		* @param f_m1 New tangent at P1. Direction and magnitude both affect the curve shape.
		*/
		inline void SetM1(const CPoint3& f_m1)
		{
			m_m1 = f_m1;
		}

	private:

		CPoint3 m_p0;
		CPoint3 m_p1;
		CPoint3 m_m0;
		CPoint3 m_m1;
	};
}

#endif