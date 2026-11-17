
#ifndef H_SEGMENT_2D
#define H_SEGMENT_2D

#pragma once

#include "CPoint2.h"
#include "IGeometricalObject2D.h"
#include "CompGeomAlgorithms.h"

#include <algorithm>

namespace ggeomlib
{
	/**
	* @class CSegment2D
	* @brief Represents a 2D line segment defined by a start and end point.
	*/
	class CSegment2D : public IGeometricalObject2D
	{
	public:
		/**
		* @brief Default constructor. Initializes an empty segment.
		*/
		CSegment2D():	
			m_id(-1),
			m_used(false)
		{

		}

		/**
		* @brief Constructs a segment with given start and end points.
		* @param f_start_pt The start point of the segment.
		* @param f_end_pt The end point of the segment.
		*/
		CSegment2D(const CPoint2& f_start_pt,
				   const CPoint2& f_end_pt):
						m_start_pt(f_start_pt),
						m_end_pt(f_end_pt),
						m_id(-1),
						m_used(false)
		{

		}

		/**
		* @brief Retrieves the axis-aligned bounding span of the segment.
		*
		* This method computes the minimum and maximum X and Y coordinates
		* of the segment and returns them via output parameters.
		* The resulting values define the axis-aligned bounding box (AABB)
		* enclosing the segment.
		*
		* @param f_x_min Minimum X coordinate of the segment.
		* @param f_y_min Minimum Y coordinate of the segment.
		* @param f_x_max Maximum X coordinate of the segment.
		* @param f_y_max Maximum Y coordinate of the segment.
		*/
		inline void Bounds(double& f_x_min, 
						   double& f_y_min, 
						   double& f_x_max, 
						   double& f_y_max) const override
		{
			f_x_max = GetXMax();
			f_x_min = GetXMin();
			f_y_max = GetYMax();
			f_y_min = GetYMin();
		}

		inline bool IntersectsWith2DBox(const double f_x_min,
										const double f_y_min,
										const double f_x_max,
										const double f_y_max) const override
		{
			cgeom_algs::Segment2DBoxIntersection(m_start_pt, m_end_pt, f_x_min, f_x_max, f_y_min, f_y_max);
		}

		/**
		* @brief Sets the start point of the segment.
		* @param f_start_pt The new start point.
		*/
		inline void SetStartPt(const CPoint2& f_start_pt)
		{
			m_start_pt = f_start_pt;
		}

		/**
		* @brief Gets the start point of the segment.
		* @return The start point.
		*/
		inline CPoint2 GetStartPt() const
		{
			return m_start_pt;
		}

		/**
		* @brief Sets the end point of the segment.
		* @param f_end_pt The new end point.
		*/
		inline void SetEndPt(const CPoint2& f_end_pt)
		{
			m_end_pt = f_end_pt;
		}

		/**
		* @brief Gets the end point of the segment.
		* @return The end point.
		*/
		inline CPoint2 GetEndPt() const 
		{
			return m_end_pt;
		}

		/**
		* @brief Sets the unique identifier for this segment.
		*
		* @param f_id Integer ID to assign to the segment.
		*/
		inline void SetID(const int f_id)
		{
			m_id = f_id;
		}

		/**
		* @brief Gets the unique identifier of this segment.
		*
		* @return The integer ID assigned to the segment.
		*/
		inline int GetID() const
		{
			return m_id;
		}

		/**
		* @brief Returns whether the segment is marked as used.
		*
		* This flag can be utilized by algorithms (e.g., intersection,
		* chaining, or graph traversal routines) to indicate that the
		* segment has already been processed.
		*
		* @return true if the segment is marked as used, false otherwise.
		*/
		inline bool GetUsed() const
		{
			return m_used;
		}

		/**
		* @brief Marks the segment as used or unused.
		*
		* @param f_used Boolean flag indicating whether the segment
		*               has been processed.
		*/
		inline void SetUsed(const bool f_used)
		{
			m_used = f_used;
		}

		/**
		* @brief Computes the Euclidean length of the segment.
		*
		* The length is calculated as the distance between the start
		* and end points.
		*
		* @return The segment length.
		*/
		inline double Length() const
		{
			return Distance(m_start_pt, m_end_pt);
		}

	private:

		/**
		* @brief Returns the minimum X of the segment
		*/
		inline double GetXMin() const
		{
			return std::min(m_start_pt.m_x, m_end_pt.m_x);
		}
		/**
		* @brief Returns the maximum X of the segment
		*/
		inline double GetXMax() const
		{
			return std::max(m_start_pt.m_x, m_end_pt.m_x);
		}
		/**
		* @brief Returns the minimum Y of the segment
		*/
		inline double GetYMin() const
		{
			return std::min(m_start_pt.m_y, m_end_pt.m_y);
		}
		/**
		* @brief Returns the maximum Y of the segment
		*/
		inline double GetYMax() const
		{
			return std::max(m_start_pt.m_y, m_end_pt.m_y);
		}

		/**
		* The start point of the segment.
		*/
		CPoint2 m_start_pt;
		/**
		* The end point of the segment.
		*/
		CPoint2 m_end_pt;
		/**
		* The ID of the segment
		*/
		int m_id;
		/**
		* Flag to be sued to signal such segment has been processed (can be used for different algorithms)
		*/
		bool m_used;
	};
}

#endif
