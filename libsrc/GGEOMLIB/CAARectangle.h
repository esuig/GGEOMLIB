
#ifndef H_CAA_RECTANGLE
#define H_CAA_RECTANGLE

#pragma once
#include "CPoint2.h"

namespace ggeomlib
{
	/**
	* @class CAARectangle
	* @brief Represents a 2D axis-aligned rectangle.
	*
	* This class models a rectangle aligned with the Cartesian axes, defined by its
	* lower-left and upper-right vertices. It computes various geometric properties,
	* such as width, height, centroid, and half-diagonal length.
	*/
	class CAARectangle
	{
	public:
		/**
		* @brief Default constructor.
		*
		* Initializes the rectangle with undefined corner values.
		*/
		CAARectangle()
		{
			m_id = -1;
			m_half_diagonal = 0.0;
			m_centroid.m_x = 0.0;
			m_centroid.m_y = 0.0;
		}

		/**
		* @brief Constructs a rectangle using lower-left and upper-right vertices.
		*
		* @param f_lower_left_vert   The lower-left vertex of the rectangle.
		* @param l_upper_right_vert  The upper-right vertex of the rectangle.
		*/
		CAARectangle(const CPoint2& f_lower_left_vert, const CPoint2& l_upper_right_vert)
		{
			m_id = -1;
			m_lower_left_vert = f_lower_left_vert;
			m_upper_right_vert = l_upper_right_vert;

			CalculateHalfDiagonal();
			CalculateCentroid();

		}

		/**
		* @brief Sets the rectangle's unique identifier.
		*
		* @param f_id Integer ID to associate with the rectangle.
		*/
		inline void SetId(int f_id)
		{
			m_id = f_id;
		}
		/**
		* @brief Gets the rectangle's identifier.
		*
		* @return The integer ID of the rectangle.
		*/
		inline int GetId() const
		{
			return m_id;
		}

		/**
		* @brief Returns the lower-left vertex of the rectangle.
		*
		* @return A 2D vector representing the lower-left corner.
		*/
		inline const CPoint2& GetLowerLeftVertex() const
		{
			return m_lower_left_vert;
		}
		/**
		* @brief Returns the upper-right vertex of the rectangle.
		*
		* @return A 2D vector representing the upper-right corner.
		*/
		inline const CPoint2& GetUpperRightVertex() const
		{
			return m_upper_right_vert;
		}
		
		/**
		* @brief Gets the width of the rectangle along the X-axis.
		*
		* @return The width (positive X span).
		*/
		inline double GetWidth() const
		{
			return (m_upper_right_vert.m_x - m_lower_left_vert.m_x);
		}
		
		/**
		* @brief Gets the height of the rectangle along the Y-axis.
		*
		* @return The height (positive Y span).
		*/
		inline double GetHeight() const
		{
			return (m_upper_right_vert.m_y - m_lower_left_vert.m_y);
		}

		/**
		* @brief Returns the centroid (center) of the rectangle.
		*
		* @return A 2D vector representing the centroid.
		*/
		inline const CPoint2& GetCentroid() const
		{
			return m_centroid;
		}

		/**
		* @brief Returns the length of half the diagonal of the rectangle.
		*
		* Useful for spatial queries, bounding volume tests, or clustering.
		*
		* @return The half-length of the diagonal.
		*/
		inline double GetHalfDiagonal() const
		{
			return m_half_diagonal;
		}

		/**
		* @brief Sets the rectangle's vertices and computes geometric properties.
		*
		* @param f_lower_left_vertex  The new lower-left corner.
		* @param f_upper_right_vertex The new upper-right corner.
		*/
		inline void SetVertexesAndCalculateData(const CPoint2& f_lower_left_vertex,
											    const CPoint2& f_upper_right_vertex)
		{
			m_lower_left_vert = f_lower_left_vertex;
			m_upper_right_vert = f_upper_right_vertex;

			CalculateHalfDiagonal();
			CalculateCentroid();
		}

		inline bool ContainsPoint(const CPoint2& f_pt) const
		{
			return f_pt.m_x >= m_lower_left_vert.m_x && f_pt.m_x <= m_upper_right_vert.m_x &&
				   f_pt.m_y >= m_lower_left_vert.m_y && f_pt.m_y <= m_upper_right_vert.m_y;
		}

		inline double GetXMin() const
		{
			return m_lower_left_vert.m_x;
		}
		inline double GetXMax() const
		{
			return m_upper_right_vert.m_x;
		}
		inline double GetYMin() const
		{
			return m_lower_left_vert.m_y;
		}
		inline double GetYMax() const
		{
			return m_upper_right_vert.m_y;
		}

	private:

		/**
		* @brief Calculates the centroid based on the current corners.
		*/
		inline void CalculateCentroid()
		{
			m_centroid = (m_lower_left_vert + m_upper_right_vert) / 2.0;
		}

		/**
		* @brief Calculates the half-diagonal length.
		*
		* Computes the distance from the centroid to one of the corners.
		*/
		void CalculateHalfDiagonal() 
		{
			CPoint2 l_midpoint((m_lower_left_vert.m_x + m_upper_right_vert.m_x) / 2.0, (m_lower_left_vert.m_y + m_upper_right_vert.m_y) / 2.0);
			m_half_diagonal = Distance(l_midpoint, m_lower_left_vert);
		}

		/**
		* Lower - left corner of the rectangle(minimum X and Y).
		*/
		CPoint2 m_lower_left_vert;
		/**
		* Upper - right corner of the rectangle(maximum X and Y).
		*/
		CPoint2 m_upper_right_vert;
		/**
		* Half the diagonal length from the center to one corner.
		*/
		double m_half_diagonal;
		/**
		* Center(centroid) of the rectangle.
		*/
		CPoint2 m_centroid;
		/**
		* Optional identifier associated with the rectangle.
		*/
		int m_id;
	};
}

#endif
