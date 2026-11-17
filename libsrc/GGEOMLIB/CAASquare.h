
#ifndef H_AASQUARE
#define H_AASQUARE

#pragma once

#include "CPoint2.h"

namespace ggeomlib
{
	/**
	* @brief Represents an axis-aligned square in 2D space
	*/
	class CAASquare
	{
	public:
		/**
		* @brief Default constructor. Initializes the square at the origin with zero side length.
		*/
		CAASquare()
		{
			m_side_length = 0.0;
			m_half_diagonal = 0.0;

			m_low_left_vertex.m_x = 0.0;
			m_low_left_vertex.m_y = 0.0;

			m_up_right_vertex.m_x = 0.0;
			m_up_right_vertex.m_y = 0.0;

			m_center.m_x = 0.0;
			m_center.m_y = 0.0;
		}
		/**
		* @brief Constructs a square using a center point and side length.
		*
		* @param f_center The center point of the square.
		* @param f_side_length The length of one side of the square.
		*/
		CAASquare(const CPoint2& f_center, const double f_side_length)
		{
			m_side_length = f_side_length;
			m_center = f_center;
			m_low_left_vertex.m_x = m_center.m_x - (m_side_length / 2.0);
			m_low_left_vertex.m_y = m_center.m_y - (m_side_length / 2.0);

			m_up_right_vertex.m_x = m_center.m_x + (m_side_length / 2.0);
			m_up_right_vertex.m_y = m_center.m_y + (m_side_length / 2.0);

			CalculateHalfDiagonal();

		}

		/**
		* @brief Constructs a square using the lower-left and upper-right corner points.
		*
		* @param f_low_left_vert The lower-left vertex of the square.
		* @param f_up_right_vert The upper-right vertex of the square.
		*/
		CAASquare(const CPoint2& f_low_left_vert, const CPoint2& f_up_right_vert)
		{
			m_low_left_vertex = f_low_left_vert;
			m_up_right_vertex = f_up_right_vert;

			m_side_length = f_up_right_vert.m_x - f_low_left_vert.m_x;

			CalculateCentroid();
			CalculateHalfDiagonal();
		}

		/**
		* @brief Gets the centroid (center point) of the square.
		*
		* @return The center of the square.
		*/
		inline CPoint2 GetCentroid() const
		{
			return m_center;
		}
		/**
		* @brief Gets half the length of the square's diagonal.
		*
		* @return Half of the diagonal length.
		*/
		inline double GetHalfDiagonal() const
		{
			return m_half_diagonal;
		}

		/**
		* @brief Gets the lower-left vertex of the square.
		*
		* @return The lower-left corner as a CVec2.
		*/
		inline CPoint2 GetLowerLeftVertex() const
		{
			return m_low_left_vertex;
		}
		/**
		* @brief Gets the upper-right vertex of the square.
		*
		* @return The upper-right corner as a CVec2.
		*/
		inline CPoint2 GetUpperRightVertex() const
		{
			return m_up_right_vertex;
		}

		/**
		* @brief Gets the length of one side of the square.
		*
		* @return The side length.
		*/
		inline double GetSideLength() const
		{
			return m_side_length;
		}
		/**
		* @brief Gets half the side length of the square.
		*
		* @return Half of the side length.
		*/
		inline double GetHalfSideLength() const
		{
			return (m_side_length / 2.0);
		}

	private:
		/**
		* @brief Calculates half of the diagonal based on corner points.
		*/
		inline void CalculateHalfDiagonal() 
		{
			m_half_diagonal = Distance(((m_low_left_vertex + m_up_right_vertex) / 2.0), m_up_right_vertex);
		}
		/**
		* @brief Calculates the centroid (center) based on corner points.
		*/
		inline void CalculateCentroid()
		{
			m_center = (m_low_left_vertex + m_up_right_vertex) / 2.0;
		}

		CPoint2 m_low_left_vertex;
		CPoint2 m_up_right_vertex;
		CPoint2 m_center;

		double m_side_length;
		double m_half_diagonal;
	};
}

#endif
