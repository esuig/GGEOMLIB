
#ifndef H_CRAY
#define H_CRAY

#pragma once

#include "CPoint3.h"

namespace ggeomlib
{
	/**
	* @class CRay
	* @brief Represents a 3D ray with a start point, direction, and ray moment.
	*
	* This class stores a ray defined by its origin point and direction vector.
	* It also computes the ray moment, which is the cross product of the normalized
	* direction and the start point. Useful for geometric computations like Plücker coordinates.
	*/
	class CRay
	{
	public:

		/**
		* @brief Constructor initializing the ray with start point and direction.
		* @param f_p_start		The start point of the ray.
		* @param f_direction	The direction vector of the ray (not necessarily normalized).
		*/
		CRay(CPoint3 f_p_start, CPoint3 f_direction)
		{
			m_p_start = f_p_start;
			m_dir = f_direction;

			CalculateRayMoment();
		}

		/**
		* @brief Get the start point of the ray.
		* @return The 3D point representing the ray's origin.
		*/
		inline CPoint3 GetPStart() const
		{
			return m_p_start;
		}
		/**
		* @brief Get the direction vector of the ray.
		* @return The 3D vector representing the ray's direction.
		*/
		inline CPoint3 GetDir() const
		{
			return m_dir;
		}

		/**
		* @brief Calculate the ray moment (cross product of normalized direction and start point).
		*
		* This updates the internal ray moment member variable.
		*/
		inline void CalculateRayMoment()
		{
			m_ray_moment = Cross((GetDir().UnitizeInPlace()), GetPStart());
		}

		/**
		* @brief Get the ray moment vector.
		* @return The ray moment vector.
		*/
		inline CPoint3 GetMoment() const
		{
			return m_ray_moment;
		}

	private:
		CPoint3 m_p_start;
		CPoint3 m_dir;
		CPoint3 m_ray_moment;
	};
}

#endif
