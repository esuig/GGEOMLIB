
#ifndef H_CRAY2D
#define H_CRAY2D

#pragma once

#include "CPoint2.h"

namespace ggeomlib
{
	/**
	* @class CRay
	* @brief Represents a 2D ray with a start point, direction, and ray moment.
	*
	* This class stores a ray defined by its origin point and direction vector.
	*/
	class CRay2D
	{
	public:

		/**
		* @brief Constructor initializing the ray with start point and direction.
		* @param f_p_start		The start point of the ray.
		* @param f_direction	The direction vector of the ray (not necessarily normalized).
		*/
		CRay2D(const CPoint2 f_p_start, CPoint2 f_direction)
		{
			m_p_start = f_p_start;
			m_dir = f_direction;
		}

		/**
		* @brief Get the start point of the ray.
		* @return The 3D point representing the ray's origin.
		*/
		inline const CPoint2& GetPStart() const
		{
			return m_p_start;
		}
		/**
		* @brief Get the direction vector of the ray.
		* @return The 3D vector representing the ray's direction.
		*/
		inline const CPoint2& GetDir() const
		{
			return m_dir;
		}

	private:
		/**
		* Ray start point
		*/
		CPoint2 m_p_start;
		/**
		* Ray direction vector
		*/
		CPoint2 m_dir;       
	};
}

#endif
