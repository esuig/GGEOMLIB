
#ifndef H_COMP_GEOM_ALGS
#define H_COMP_GEOM_ALGS
#pragma once

#include "CAARectangle.h"
#include "MathAlgorithms.h"
#include "MathConstants.h"
#include "CPoint2.h"
#include "CPoint3.h"
#include "CRay2D.h"

#include "IGraphInterface.h"

#include <cassert>
#include <vector>
#include <sstream>

namespace ggeomlib
{
	namespace cgeom_algs
	{
		/**
		* @brief Determines the relative position of a point with respect to a directed 2D line segment.
		*
		* This function tests whether a given test point lies to the left, to the right,
		* or directly on the line defined by a directed segment from `f_line_start_pt` to `f_line_end_pt`.
		*
		* @param[in] f_line_start_pt  The starting point of the line segment (P0).
		* @param[in] f_line_end_pt    The ending point of the line segment (P1).
		* @param[in] f_test_pt        The point to be tested (P2).
		* @param[in] f_threshold	  A threshold to determine whether the point is on top of the segment	
		*
		* @return
		* - 1 if the test point lies to the left of the directed line (P0 -> P1)
		* - 0 if the test point lies exactly on the line
		* - 2 if the test point lies to the right of the directed line
		*/
		inline int IsLeft(const CPoint2& f_line_start_pt,
						  const CPoint2& f_line_end_pt,
						  const CPoint2& f_test_pt,
						  const double f_threshold = DOUBLE_PRECISION_EPSILON)
		{
			double l_ret_val = ((f_line_end_pt.m_x - f_line_start_pt.m_x) * (f_test_pt.m_y - f_line_start_pt.m_y)
				- (f_test_pt.m_x - f_line_start_pt.m_x) * (f_line_end_pt.m_y - f_line_start_pt.m_y));

			if (fabs(l_ret_val) < f_threshold)
			{
				return 0;
			}

			if (l_ret_val > f_threshold)
			{
				return 1;
			}

			return 2;
		}

		/**
		* @brief Computes the minimum (lower-left) and maximum (upper-right) corners of the bounding box
		*        of a triangle projected onto a specified coordinate plane.
		*
		* The projection plane is selected based on the parameter @p f_axis:
		* - 0: Projection onto the YZ plane
		* - 1: Projection onto the XZ plane
		* - 2: Projection onto the XY plane
		*
		* @param f_vert_1 Pointer to the first vertex of the triangle.
		* @param f_vert_2 Pointer to the second vertex of the triangle.
		* @param f_vert_3 Pointer to the third vertex of the triangle.
		* @param f_axis Specifies the coordinate plane onto which the triangle is projected:
		*               0 = YZ plane, 1 = XZ plane, 2 = XY plane.
		* @param[out] f_bbox_min The minimum corner (lower-left) of the bounding box of the projected triangle.
		* @param[out] f_bbox_max The maximum corner (upper-right) of the bounding box of the projected triangle.
		*/
		void GetTriangleProjectionBoundingBox(const CPoint3& f_vert_1,
											  const CPoint3& f_vert_2,
											  const CPoint3& f_vert_3,
											  const unsigned int f_axis,
											  CPoint2& f_bbox_min,
											  CPoint2& f_bbox_max)
		{
			assert(f_axis < 3);

			const int l_axis_1 = (f_axis + 1) % 3;
			const int l_axis_2 = (f_axis + 2) % 3;

			CPoint2 l_proj_vert_1;
			CPoint2 l_proj_vert_2;
			CPoint2 l_proj_vert_3;

			l_proj_vert_1.m_x = f_vert_1[l_axis_1];
			l_proj_vert_1.m_y = f_vert_1[l_axis_2];

			l_proj_vert_2.m_x = f_vert_2[l_axis_1];
			l_proj_vert_2.m_y = f_vert_2[l_axis_2];

			l_proj_vert_3.m_x = f_vert_3[l_axis_1];
			l_proj_vert_3.m_y = f_vert_3[l_axis_2];

			f_bbox_min.m_x = math_algs::FindMinOf3Numbers(l_proj_vert_1.m_x, l_proj_vert_2.m_x, l_proj_vert_3.m_x);
			f_bbox_min.m_y = math_algs::FindMinOf3Numbers(l_proj_vert_1.m_y, l_proj_vert_2.m_y, l_proj_vert_3.m_y);
			f_bbox_max.m_x = math_algs::FindMaxOf3Numbers(l_proj_vert_1.m_x, l_proj_vert_2.m_x, l_proj_vert_3.m_x);
			f_bbox_max.m_y = math_algs::FindMaxOf3Numbers(l_proj_vert_1.m_y, l_proj_vert_2.m_y, l_proj_vert_3.m_y);
		}

		/**
		 * @brief Checks whether a point lies on a line segment between two other points in 2D space.
		 *
		 * Given three points `f_p`, `f_q`, and `f_r`, this function determines whether `f_p` lies
		 * on the line segment from `f_q` to `f_r`.
		 *
		 * @param[in] f_p  The point to be tested.
		 * @param[in] f_q  One endpoint of the segment.
		 * @param[in] f_r  The other endpoint of the segment.
		 *
		 * @return
		 * - true if `f_p` lies on the segment `f_q`–`f_r`.
		 * - false otherwise.
		 */
		inline bool IsPointOnSegment(const CPoint2& f_p,
									 const CPoint2& f_a,
									 const CPoint2& f_b)
		{
			if (Cross(f_p - f_a, f_b - f_a) != 0.0)
			{
				return false;
			}

			return (f_p.m_x >= std::min(f_a.m_x, f_b.m_x) &&
					f_p.m_x <= std::max(f_a.m_x, f_b.m_x) &&
					f_p.m_y >= std::min(f_a.m_y, f_b.m_y) &&
					f_p.m_y <= std::max(f_a.m_y, f_b.m_y));
		}

		enum class SegmentIntersectionResult
		{
			Parallel,
			Inside,
			OutsideFirst,
			OutsideSecond,
			OutsideBoth,
		};

		/** 
		* @brief Computes the intersection point of two 2D line segments.
		*
		* Given two segments defined by their endpoints, this function determines
		* whether they intersect and classifies the intersection relative to each
		* segment's extent.
		*
		* The intersection is computed parametrically. For segment 1 (a1 -> b1),
		* a parameter @p t1 is found such that:
		* @code
		*     intersection = a1 + t1 * (b1 - a1)
		* @endcode
		* The intersection lies within the segment when @p t1 is in [0, 1].
		* The same applies to @p t2 for segment 2.
		*
		* @note If two endpoints are within @p intersection_threshold of each other,
		*       the function returns SegmentIntersectionResult::Inside immediately,
		*       treating the segments as touching at that tip.
		*
		* @warning The cross product magnitude must exceed @p parallelism_threshold
		*          for a valid intersection to be computed. Callers should choose this
		*          value relative to the scale of their input coordinates.
		*
		* @param[out] f_out_intersection_point The computed intersection point.
		*                                      Undefined if the result is
		*                                      SegmentIntersectionResult::Parallel.
		* @param[in] f_a1 Start point of the first segment.
		* @param[in] f_b1 End point of the first segment.
		* @param[in] f_a2 Start point of the second segment.
		* @param[in] f_b2 End point of the second segment.
		* @param[in] f_parallelism_threshold Minimum absolute cross product magnitude
		*                                    below which segments are considered parallel.
		* @param[in] f_intersection_threshold Tolerance applied both to endpoint
		*                                     coincidence checks and to the [0, 1]
		*                                     parameter range test.
		*
		* @return A SegmentIntersectionResult classifying the intersection:
		*         - SegmentIntersectionResult::Parallel       if segments are parallel.
		*         - SegmentIntersectionResult::Inside         if intersection is within both.
		*         - SegmentIntersectionResult::OutsideFirst   if outside segment 1 only.
		*         - SegmentIntersectionResult::OutsideSecond  if outside segment 2 only.
		*         - SegmentIntersectionResult::OutsideBoth    if outside both segments.
		* */
		inline SegmentIntersectionResult SegmentSegmentIntersection(CPoint2& f_out_intersection_point,
																    const CPoint2& f_a1,
																    const CPoint2& f_b1,
																    const CPoint2& f_a2,
																    const CPoint2& f_b2,
																    double f_parallelism_threshold,
																    double f_intersection_threshold)
		{
			const CPoint2 seg1 = f_b1 - f_a1;
			const CPoint2 seg2 = f_b2 - f_a2;

			const double l_cross_product_magnitude = CrossProductMagnitude(seg2, seg1);
			if (std::fabs(l_cross_product_magnitude) < f_parallelism_threshold)
			{
				return SegmentIntersectionResult::Parallel;
			}

			struct EndpointPair { const CPoint2& m_p1; const CPoint2& m_p2; };
			for (auto [p1, p2] : { EndpointPair{f_a1, f_a2}, EndpointPair{f_a1, f_b2},
								   EndpointPair{f_b1, f_a2}, EndpointPair{f_b1, f_b2} })
			{
				if (Distance(p1, p2) < f_intersection_threshold)
				{
					f_out_intersection_point = p1;
					return SegmentIntersectionResult::Inside;
				}
			}

			const CPoint2 l_start_delta = f_a2 - f_a1;

			const double t1 = CrossProductMagnitude(seg2, l_start_delta) / l_cross_product_magnitude;
			const double t2 = CrossProductMagnitude(seg1, l_start_delta) / l_cross_product_magnitude;

			const double lo = -f_intersection_threshold;
			const double hi = 1.0 + f_intersection_threshold;

			const bool outside_seg1 = (t1 < lo) || (t1 > hi);
			const bool outside_seg2 = (t2 < lo) || (t2 > hi);

			f_out_intersection_point = f_a1 + t1 * seg1;

			if (outside_seg1 && outside_seg2) return SegmentIntersectionResult::OutsideBoth;
			if (outside_seg1)                 return SegmentIntersectionResult::OutsideFirst;
			if (outside_seg2)                 return SegmentIntersectionResult::OutsideSecond;

			return SegmentIntersectionResult::Inside;
		}

		/**
		* @brief Computes the shortest distance from a 3D point to a line segment.
		*
		* This function calculates the distance from point `f_p` to the segment defined by endpoints
		* `f_v1` and `f_v2`. If the orthogonal projection of `f_p` onto the infinite line defined by the segment
		* falls outside the segment bounds, or if the segment is degenerate (its endpoints are nearly coincident),
		* the function returns a negative value indicating the error case.
		*
		* @param[in] f_p                      The point from which the distance is measured.
		* @param[in] f_v1                     The first endpoint of the segment.
		* @param[in] f_v2                     The second endpoint of the segment.
		* @param[in] f_zero_length_segm_threshold Threshold for detecting degenerate segments (default is COINC_THRESH).
		*
		* @return
		* - A positive distance if the projection of `f_p` lies within the segment.
		* - `-1.0` if the projection lies outside the segment.
		* - `-2.0` if the segment is degenerate (endpoints are considered coincident).
		*/
		inline double PointSegmentDistance(const CPoint3& f_p,
										   const CPoint3& f_v1,
										   const CPoint3& f_v2,
										   double f_zero_length_segm_threshold = COINC_THRESH)
		{
			const CPoint3 l_seg = f_v2 - f_v1;
			const double l_seg_length = l_seg.Length();

			if (l_seg_length <= f_zero_length_segm_threshold)
				return -2.0;

			const CPoint3 l_v1p = f_p - f_v1;
			const double l_v1p_lensq = l_v1p.LengthSq();

			const double l_proj = Dot(l_seg, l_v1p) / l_seg_length;

			if ((l_proj < 0.0) || (l_proj > l_seg_length))
				return -1.0;

			return std::sqrt(std::fabs(l_v1p_lensq - (l_proj * l_proj)));
		}

		/**
		* @brief Computes the shortest distance from a 2D point to a line segment.
		*
		* This function calculates the distance from point `f_p` to the segment defined by endpoints
		* `f_v1` and `f_v2` in 2D space. If the orthogonal projection of `f_p` onto the infinite line
		* defined by the segment falls outside the segment bounds, or if the segment is degenerate
		* (its endpoints are nearly coincident), the function returns a negative value indicating the error case.
		*
		* @param[in] f_p                      The 2D point from which the distance is measured.
		* @param[in] f_v1                     The first endpoint of the 2D segment.
		* @param[in] f_v2                     The second endpoint of the 2D segment.
		* @param[in] f_zero_length_segm_threshold Threshold for detecting degenerate segments (default is COINC_THRESH).
		*
		* @return
		* - A positive distance if the projection of `f_p` lies within the segment.
		* - `-1.0` if the projection lies outside the segment.
		* - `-2.0` if the segment is degenerate (endpoints are considered coincident).
		*/
		inline double PointSegmentDistance(const CPoint2& f_p,
										   const CPoint2& f_v1,
										   const CPoint2& f_v2,
										   double f_zero_length_segm_threshold = COINC_THRESH)
		{
			const CPoint2 l_seg = f_v2 - f_v1;
			const double l_seg_length = l_seg.Length();

			if (l_seg_length <= f_zero_length_segm_threshold)
			{
				return -2.0;
			}

			const CPoint2 l_v1p = f_p - f_v1;
			const double l_v1p_lensq = l_v1p.LengthSq();

			const double l_proj = Dot(l_seg, l_v1p) / l_seg_length;

			if ((l_proj < 0.0) || (l_proj > l_seg_length))
			{
				return -1.0;
			}

			return std::sqrt(std::fabs(l_v1p_lensq - (l_proj * l_proj)));
		}

		/**
		* @brief Finds the closest point on a 2D line segment to a given point and returns the distance.
		*
		* Given a point `f_p` and a line segment defined by endpoints `f_v1` and `f_v2`, this function
		* computes the closest point on the segment to `f_p` and stores it in `f_proj_pt`. It also returns
		* the distance between `f_p` and this closest point.
		*
		* The following rules are applied:
		* - If the orthogonal projection of `f_p` onto the infinite line defined by the segment falls
		*   within the segment bounds, the projection is the closest point.
		* - If the projection falls outside the segment, the closest point is the nearest endpoint (`f_v1` or `f_v2`).
		* - If the segment is degenerate (i.e. its length is less than `f_degeneration_threshold`),
		*   the closest point is either of the two (coincident) endpoints, and the distance is computed
		*   from `f_p` to that point.
		*
		* @param[in]  f_p                      The 2D point from which the distance is measured.
		* @param[in]  f_v1                     The first endpoint of the segment.
		* @param[in]  f_v2                     The second endpoint of the segment.
		* @param[out] f_proj_pt                The closest point on the segment to `f_p`.
		* @param[in]  f_zero_length_segm_threshold Threshold below which the segment is considered degenerate (default is COINC_THRESH).
		*
		* @return The Euclidean distance between `f_p` and the closest point on the segment.
		*/
		inline double FindClosestPointOnSegment(const CPoint2& f_p,
												const CPoint2& f_v1,
												const CPoint2& f_v2,
												CPoint2& f_proj_pt,
												double f_zero_length_segm_threshold = COINC_THRESH)
		{
			CPoint2 l_seg = f_v2 - f_v1;
			const double l_seg_length = l_seg.Length();

			if (l_seg_length <= f_zero_length_segm_threshold)
			{
				f_proj_pt = f_v1;
				return Distance(f_p, f_v1);
			}

			const CPoint2 l_v1p = f_p - f_v1;
			const double l_v1p_lensq = l_v1p.LengthSq();

			const double l_proj = Dot(l_seg, l_v1p) / l_seg_length;

			if ((l_proj >= 0.0) && (l_proj <= l_seg_length))
			{
				f_proj_pt = f_v1 + (l_seg.UnitizeInPlace() * l_proj);
				return std::sqrt(std::fabs(l_v1p_lensq - (l_proj * l_proj)));
			}

			const double l_dist_v1 = Distance(f_v1, f_p);
			const double l_dist_v2 = Distance(f_v2, f_p);
			if (l_dist_v1 < l_dist_v2)
			{
				f_proj_pt = f_v1;
				return l_dist_v1;
			}
			else
			{
				f_proj_pt = f_v2;
				return l_dist_v2;
			}
		}

		/**
		* @brief Projects a 3D point onto a line segment and checks whether the projection lies within the segment.
		*
		* This function computes the orthogonal projection of point `f_p` onto the segment defined by endpoints
		* `f_v1` and `f_v2`. The projection point is stored in `f_proj_pt`, and the function returns a boolean
		* indicating whether the projection lies within the segment bounds (including the endpoints).
		*
		* Segments whose length is less than `f_degeneration_threshold` are considered degenerate, in which case
		* the function returns false. The projection point and distance are always computed regardless of the return value.
		*
		* @param[in]  f_p                      The 3D point to be projected.
		* @param[in]  f_v1                     The first endpoint of the segment.
		* @param[in]  f_v2                     The second endpoint of the segment.
		* @param[out] f_proj_pt                The projection of `f_p` onto the segment or its extension.
		* @param[in]  f_zero_length_segm_threshold Threshold below which the segment is considered degenerate (default is COINC_THRESH).
		*
		* @return
		* - `true` if the projection of `f_p` lies within or on the segment.
		* - `false` if the projection falls outside the segment or if the segment is degenerate.
		*/
		inline bool PointProjectionOnSegment(const CPoint3& f_p,
											 const CPoint3& f_v1,
											 const CPoint3& f_v2,
											 CPoint3& f_proj_pt,
											 double f_zero_length_segm_threshold = COINC_THRESH)
		{
			CPoint3 l_seg = f_v2 - f_v1;
			const double l_seg_length = l_seg.Length();

			if (l_seg_length <= f_zero_length_segm_threshold)
				return false;

			const CPoint3 l_v1p = f_p - f_v1;
			double l_proj = Dot(l_seg, l_v1p) / l_seg_length;

			if ((l_proj < -COINC_THRESH) || (l_proj > l_seg_length + COINC_THRESH))
			{
				return false;
			}

			l_proj = std::clamp(l_proj, 0.0, l_seg_length);
			f_proj_pt = f_v1 + l_seg.UnitizeInPlace() * l_proj;

			return true;
		}

		/**
		* @brief Projects a 2D point onto a line segment and determines whether the projection lies within the segment.
		*
		* This function calculates the orthogonal projection of point `f_p` onto the segment defined by endpoints
		* `f_v1` and `f_v2`. The projected point is stored in `f_proj_pt`, and the Euclidean distance between `f_p`
		* and the projection is stored in `f_distance`. The function returns `true` if the projection lies strictly
		* within the segment, and `false` otherwise.
		*
		* Degenerate segments (i.e., segments whose length is less than `f_degeneration_threshold`) are rejected.
		* If `f_ignore_extremes` is set to `true`, projections that lie exactly on one of the endpoints are also
		* treated as invalid (i.e., the function will return `false` in such cases).
		*
		* @param[in]  f_p                      The 2D point to project onto the segment.
		* @param[in]  f_v1                     First endpoint of the segment.
		* @param[in]  f_v2                     Second endpoint of the segment.
		* @param[out] f_proj_pt                The computed projection of `f_p` onto the segment or its extension.
		* @param[out] f_distance               The Euclidean distance between `f_p` and the projected point.
		* @param[in]  f_ignore_extremes        If true, projections that fall exactly on `f_v1` or `f_v2` are ignored (function returns false).
		* @param[in]  f_zero_length_segm_threshold Threshold below which the segment is considered degenerate (default is COINC_THRESH).
		*
		* @return
		* - `true` if the projection of `f_p` lies strictly within the segment (excluding endpoints if `f_ignore_extremes` is true).
		* - `false` otherwise (e.g., projection lies outside, on an endpoint, or segment is degenerate).
		*/
		inline bool PointProjectionOnSegment(const CPoint2& f_p,
											 const CPoint2& f_v1,
											 const CPoint2& f_v2,
											 CPoint2& f_proj_pt,
											 double& f_distance,
											 const bool f_ignore_extremes = false,
											 const double f_zero_length_segm_threshold = COINC_THRESH)
		{
			CPoint2 l_seg = f_v2 - f_v1;
			const double l_seg_length = l_seg.Length();

			if (l_seg_length <= f_zero_length_segm_threshold)
			{
				return false;
			}

			const CPoint2 l_v1p = f_p - f_v1;
			const double l_v1p_lensq = l_v1p.LengthSq();

			double l_proj = Dot(l_seg, l_v1p) / l_seg_length;
			if (std::fabs(l_proj) < COINC_THRESH)
			{
				l_proj = 0.0;
			}

			const auto l_write_outputs = [&]()
				{
					f_proj_pt = f_v1 + l_seg.UnitizeInPlace() * l_proj;
					f_distance = std::sqrt(std::fabs(l_v1p_lensq - (l_proj * l_proj)));
				};

			if ((l_proj >= 0.0) && (l_proj <= l_seg_length))
			{
				if (f_ignore_extremes)
				{
					const bool l_at_start = (l_proj < f_zero_length_segm_threshold);
					const bool l_at_end = (l_seg_length - l_proj) < f_zero_length_segm_threshold;
					if (l_at_start || l_at_end)
					{
						return false;
					}
				}
				l_write_outputs();
				return true;
			}

			l_write_outputs();

			return false;
		}

		/**
		* @brief Calculates the length of a circular arc defined by its endpoints and center.
		*
		* Given the start point `f_pstart`, end point `f_pend`, and the center of the circle `f_pcenter`,
		* this function computes the arc length between the two points along the circle. The direction
		* of traversal (clockwise or counter-clockwise) is determined by the `f_ccw` flag.
		*
		* It is assumed that all points lie on the same circle and plane.
		*
		* @param[in] f_pstart  The starting point of the arc.
		* @param[in] f_pend    The ending point of the arc.
		* @param[in] f_pcenter The center of the circle that defines the arc.
		* @param[in] f_ccw     If true, the arc is assumed counter-clockwise; otherwise, clockwise.
		*
		* @return The length of the arc between `f_pstart` and `f_pend` along the defined circle.
		*/
		inline double CalculateArcLength(const CPoint2& f_pstart,
										 const CPoint2& f_pend,
										 const CPoint2& f_pcenter,
										 const bool f_ccw)
		{
			const CPoint2 l_vstart(f_pstart.m_x - f_pcenter.m_x, f_pstart.m_y - f_pcenter.m_y);
			const CPoint2 l_vend(f_pend.m_x - f_pcenter.m_x, f_pend.m_y - f_pcenter.m_y);

			const double l_cross = l_vstart.m_x * l_vend.m_y - l_vstart.m_y * l_vend.m_x;

			const bool l_concave = (l_cross < 0.0 && f_ccw)
									|| (l_cross > 0.0 && !f_ccw);

			const double l_radius_start = l_vstart.Length();
			const double l_radius_end = l_vend.Length();
			assert(std::fabs(l_radius_start - l_radius_end) <= DOUBLE_PRECISION_EPSILON);
			const double l_radius = std::max(l_radius_start, l_radius_end);

			const double l_cos_angle = Dot(l_vstart, l_vend) / (l_radius * l_radius);
			const double l_angle = math_algs::GuardedACos(l_cos_angle);

			const double l_short_arc = l_radius * l_angle;
			return l_concave ? (TWO_PI_GRECO * l_radius - l_short_arc) : l_short_arc;
		}

		/**
		* @brief Determines the orientation of the ordered triplet (f_p, f_q, f_r)
		*        in the XY plane.
		*
		* Computes the sign of the 2D cross product of the vectors (f_q - f_p) and
		* (f_r - f_q), using only the X and Y coordinates of the points (the Z
		* coordinate is ignored). The result tells whether, going from f_p to f_q to
		* f_r, the path turns clockwise, counterclockwise, or goes straight.
		*
		* @param[in] f_p First point of the triplet.
		* @param[in] f_q Second point of the triplet.
		* @param[in] f_r Third point of the triplet.
		*
		* @return 0 if the points are collinear (within DOUBLE_PRECISION_EPSILON),
		*         1 if they are oriented clockwise,
		*         2 if they are oriented counterclockwise.
		*
		* @note The collinearity test uses an absolute tolerance on the cross
		*       product, whose magnitude scales with the square of the coordinates.
		*       For very large or very small coordinates, consider scaling the
		*       tolerance accordingly.
		*/
		inline int TripletOrientation(const CPoint3& f_p,
									  const CPoint3& f_q,
									  const CPoint3& f_r)
		{
			double l_orient_val = (f_q.m_y - f_p.m_y) * (f_r.m_x - f_q.m_x) - (f_q.m_x - f_p.m_x) * (f_r.m_y - f_q.m_y);

			if (fabs(l_orient_val) < DOUBLE_PRECISION_EPSILON)
			{
				return 0;
			}
			if (l_orient_val > 0.0)
			{
				return 1;
			}
			else
			{
				return 2;
			}

		}

		/**
		* @brief Determines the orientation of the ordered triplet (f_p, f_q, f_r).
		*
		* Computes the sign of the 2D cross product of the vectors (f_q - f_p) and
		* (f_r - f_q). The result tells whether, going from f_p to f_q to f_r, the
		* path turns clockwise, counterclockwise, or goes straight.
		*
		* @param[in] f_p First point of the triplet.
		* @param[in] f_q Second point of the triplet.
		* @param[in] f_r Third point of the triplet.
		*
		* @return 0 if the points are collinear (within DOUBLE_PRECISION_EPSILON),
		*         1 if they are oriented clockwise,
		*         2 if they are oriented counterclockwise.
		*
		* @note The collinearity test uses an absolute tolerance on the cross
		*       product, whose magnitude scales with the square of the coordinates.
		*       For very large or very small coordinates, consider scaling the
		*       tolerance accordingly.
		*/
		inline int TripletOrientation(const CPoint2& f_p,
									  const CPoint2& f_q,
									  const CPoint2& f_r)
		{
			return TripletOrientation(CPoint3(f_p.m_x, f_p.m_y, 0.0), CPoint3(f_q.m_x, f_q.m_y, 0.0), CPoint3(f_r.m_x, f_r.m_y, 0.0));
		}

		/**
		* @brief Computes the axis-aligned bounding rectangle of a 2D segment.
		*
		* Given the start and end points of a segment, this function calculates the minimum and maximum
		* extents along the X and Y axes, effectively defining the smallest axis-aligned rectangle
		* that fully contains the segment.
		*
		* @param[in]  f_start_pt The starting point of the segment.
		* @param[in]  f_end_pt   The ending point of the segment.
		* @param[out] f_x_min    The minimum X-coordinate of the bounding rectangle.
		* @param[out] f_x_max    The maximum X-coordinate of the bounding rectangle.
		* @param[out] f_y_min    The minimum Y-coordinate of the bounding rectangle.
		* @param[out] f_y_max    The maximum Y-coordinate of the bounding rectangle.
		*/
		inline void GetSegmentAARect(const CPoint2& f_start_pt,
									 const CPoint2& f_end_pt,
									 double& f_x_min,
									 double& f_x_max,
									 double& f_y_min,
									 double& f_y_max)
		{
			if (f_start_pt.m_x <= f_end_pt.m_x)
			{
				f_x_min = f_start_pt.m_x;
				f_x_max = f_end_pt.m_x;
			}
			else
			{
				f_x_max = f_start_pt.m_x;
				f_x_min = f_end_pt.m_x;
			}

			if (f_start_pt.m_y <= f_end_pt.m_y)
			{
				f_y_min = f_start_pt.m_y;
				f_y_max = f_end_pt.m_y;
			}
			else
			{
				f_y_max = f_start_pt.m_y;
				f_y_min = f_end_pt.m_y;
			}
		}

		/**
		* @brief Checks if two axis-aligned rectangles overlap, optionally applying padding.
		*
		* Determines whether the two rectangles defined by their lower-left and upper-right vertices overlap.
		* An optional padding value can be added to both rectangles, effectively thickening them before
		* testing for overlap. This is useful for interference or proximity tests.
		*
		* Reference: QGV14-p31 (for context on padding usage in interference tests).
		*
		* @param[in] l_low_left_1  Lower-left vertex of the first rectangle.
		* @param[in] l_up_right_1  Upper-right vertex of the first rectangle.
		* @param[in] l_low_left_2  Lower-left vertex of the second rectangle.
		* @param[in] l_up_right_2  Upper-right vertex of the second rectangle.
		* @param[in] f_padding     Optional padding to expand the rectangles before testing overlap (default is 0.0).
		*
		* @return true if the padded rectangles overlap, false otherwise.
		*/
		inline bool AreRectangleOverlapped(const CPoint2& l_low_left_1,
										   const CPoint2& l_up_right_1,
										   const CPoint2& l_low_left_2,
										   const CPoint2& l_up_right_2,
										   const double f_padding = 0.0)
		{
			if (((l_low_left_1.m_x - f_padding) > (l_up_right_2.m_x + f_padding)) ||
				((l_low_left_2.m_x - f_padding) > (l_up_right_1.m_x + f_padding)))
			{
				return false;
			}

			if (((l_up_right_1.m_y + f_padding) < (l_low_left_2.m_y - f_padding)) ||
				((l_up_right_2.m_y + f_padding) < (l_low_left_1.m_y - f_padding)))
			{
				return false;
			}

			return true;

		}

		/**
		* @brief Checks if two axis-aligned rectangles overlap, optionally applying padding.
		*
		* Determines whether the two input rectangles overlap. An optional padding value can be added,
		* effectively thickening both rectangles before testing for overlap. This is useful for interference
		* or proximity checks.
		*
		* Reference: QGV14-p31 (for context on padding usage in interference tests).
		*
		* @param[in] f_rect_1  The first axis-aligned rectangle.
		* @param[in] f_rect_2  The second axis-aligned rectangle.
		* @param[in] f_padding Optional padding to expand the rectangles before testing overlap (default is 0.0).
		*
		* @return true if the padded rectangles overlap, false otherwise.
		*/
		inline bool AreRectangleOverlapped(const CAARectangle& f_rect_1,
										   const CAARectangle& f_rect_2,
										   const double f_padding = 0.0)
		{
			if (((f_rect_1.GetLowerLeftVertex().m_x - f_padding) > (f_rect_2.GetUpperRightVertex().m_x + f_padding)) ||
				((f_rect_2.GetLowerLeftVertex().m_x - f_padding) > (f_rect_1.GetUpperRightVertex().m_x + f_padding)))
			{
				return false;
			}

			if (((f_rect_1.GetUpperRightVertex().m_y + f_padding) < (f_rect_2.GetLowerLeftVertex().m_y - f_padding)) ||
				((f_rect_2.GetUpperRightVertex().m_y + f_padding) < (f_rect_1.GetLowerLeftVertex().m_y - f_padding)))
			{
				return false;
			}

			return true;
		}

		enum ESegmentRelation
		{
			CollinearTouchingExtremes,
			CollinearlOverlapping,
			ParallelDisjoint,
			NonParallelTouchPoint,
			Intersecting,
			NonParallelDisjoint,
		};

		/**
		* @brief Computes the minimum squared distance between two 3D segments and
		*        classifies their relative position.
		*
		* Given two segments defined by points f_P0-f_P1 and f_Q0-f_Q1, computes the
		* squared minimum distance between them, the parameters and the points on
		* each segment where this minimum distance occurs, and the relation between
		* the two segments (intersecting, touching, parallel, collinear, ...).
		*
		* The core closest-point computation is Dan Sunday's dist3D_Segment_to_Segment
		* algorithm (D. Sunday, "Distance between 3D Lines & Segments",
		* geomalgorithms.com, https://geomalgorithms.com/a07-_distance.html), adapted
		* from the modified version in David Eberly's Geometric Tools sample
		* DistanceSegments3 (https://github.com/davideberly/GeometricTools,
		* GTE/Samples/Distance/DistanceSegments3/DistanceSegments3Console.cpp,
		* Boost Software License 1.0), which takes point pairs as input and computes
		* the closest points. The handling of degenerate (point-like) segments and
		* the ESegmentRelation classification are additions specific to this
		* implementation.
		*
		* Segments shorter than COINC_THRESH are treated as points. Two segments are
		* considered intersecting or touching when their distance is below
		* COINC_THRESH.
		*
		* @warning As Eberly documents, Sunday's logic can return incorrect closest
		*          points for nearly parallel segments (D below threshold). For a
		*          robust alternative, see D. Eberly, "Robust Computation of Distance
		*          Between Line Segments",
		*          https://www.geometrictools.com/Documentation/DistanceLine3Line3.pdf,
		*
		* @param[in]  f_P0             Start point of the first segment.
		* @param[in]  f_P1             End point of the first segment.
		* @param[in]  f_Q0             Start point of the second segment.
		* @param[in]  f_Q1             End point of the second segment.
		* @param[out] f_dist_squared   Squared minimum distance between the segments.
		* @param[out] s                Parameter of the closest point on the first
		*                              segment, in [0, 1] (0 = f_P0, 1 = f_P1).
		* @param[out] t                Parameter of the closest point on the second
		*                              segment, in [0, 1] (0 = f_Q0, 1 = f_Q1).
		* @param[out] f_point_on_seg_1 Closest point on the first segment
		*                              (f_P0 + s * (f_P1 - f_P0)).
		* @param[out] f_point_on_seg_2 Closest point on the second segment
		*                              (f_Q0 + t * (f_Q1 - f_Q0)).
		*
		* @return Relation between the two segments:
		*         - ESegmentRelation::Intersecting: the segments cross at an interior
		*           point of both (or a point-like segment lies on the other one).
		*         - ESegmentRelation::NonParallelTouchPoint: non-parallel segments
		*           that touch at an endpoint of at least one of them.
		*         - ESegmentRelation::NonParallelDisjoint: non-parallel segments that
		*           do not touch (also returned for disjoint point-like segments).
		*         - ESegmentRelation::ParallelDisjoint: parallel segments that do not
		*           touch (on distinct lines, or collinear without overlap).
		*         - ESegmentRelation::CollinearTouchingExtremes: collinear segments
		*           sharing only an endpoint.
		*         - ESegmentRelation::CollinearlOverlapping: collinear segments that
		*           overlap along a portion of positive length.
		*/
		inline ESegmentRelation SegmentSegmentDistance3D(const CPoint3& f_P0,
														 const CPoint3& f_P1,
														 const CPoint3& f_Q0,
														 const CPoint3& f_Q1,
														 double& f_dist_squared,
														 double& s,
														 double& t,
													     CPoint3& f_point_on_seg_1,
														 CPoint3& f_point_on_seg_2)
		{
			const double l_eps2 = COINC_THRESH * COINC_THRESH;
			bool l_segs_parallel = false;
			const double l_seg_1_len_sq = (f_P1 - f_P0).LengthSq();
			const double l_seg_2_len_sq = (f_Q1 - f_Q0).LengthSq();
			bool l_seg_1_is_pt = (l_seg_1_len_sq < l_eps2);
			bool l_seg_2_is_pt = (l_seg_2_len_sq < l_eps2);
			if (l_seg_1_is_pt && l_seg_2_is_pt)
			{
				CPoint3 dP = f_P0 - f_Q0;
				f_dist_squared = Dot(dP, dP);
				s = t = 0.0;
				f_point_on_seg_1 = f_P0;
				f_point_on_seg_2 = f_Q0;

				if (f_dist_squared < l_eps2)
					return ESegmentRelation::Intersecting;

				return ESegmentRelation::NonParallelDisjoint;
			}
			if (l_seg_1_is_pt)
			{
				double l_dist = PointSegmentDistance(f_P0, f_Q0, f_Q1);
				f_dist_squared = l_dist * l_dist;
				if (l_dist < COINC_THRESH)
				{
					f_point_on_seg_1 = f_P0;
					f_point_on_seg_2 = f_P0;
					s = 0.0;
					t = Distance(f_P0, f_Q0) / sqrt(l_seg_2_len_sq);
					return ESegmentRelation::Intersecting;
				}
				return ESegmentRelation::NonParallelDisjoint;
			}
			if (l_seg_2_is_pt)
			{
				double l_dist = PointSegmentDistance(f_Q0, f_P0, f_P1);
				f_dist_squared = l_dist * l_dist;
				if (l_dist < COINC_THRESH)
				{
					f_point_on_seg_1 = f_Q0;
					f_point_on_seg_2 = f_Q0;
					s = Distance(f_Q0, f_P0) / sqrt(l_seg_1_len_sq);
					t = 0.0;
					return ESegmentRelation::Intersecting;
				}
				return ESegmentRelation::NonParallelDisjoint;
			}

			CPoint3 u = f_P1 - f_P0;
			CPoint3 v = f_Q1 - f_Q0;
			CPoint3 w = f_P0 - f_Q0;
			double a = Dot(u, u);
			double b = Dot(u, v);
			double c = Dot(v, v);
			double d = Dot(u, w);
			double e = Dot(v, w);
			double D = a * c - b * b;
			double sc, sN, sD = D;
			double tc, tN, tD = D;

			if (D < l_eps2)
			{
				sN = 0.0;
				sD = 1.0;
				tN = e;
				tD = c;

				l_segs_parallel = true;
			}
			else
			{
				sN = (b * e - c * d);
				tN = (a * e - b * d);
				if (sN < 0.0)
				{
					sN = 0.0;
					tN = e;
					tD = c;
				}
				else if (sN > sD)
				{
					sN = sD;
					tN = e + b;
					tD = c;
				}
			}

			if (tN < 0.0)
			{
				tN = 0.0;
				if (-d < 0.0)
				{
					sN = 0.0;
				}
				else if (-d > a)
				{
					sN = sD;
				}
				else
				{
					sN = -d;
					sD = a;
				}
			}
			else if (tN > tD)
			{
				tN = tD;
				if ((-d + b) < 0.0)
				{
					sN = 0;
				}
				else if ((-d + b) > a)
				{
					sN = sD;
				}
				else
				{
					sN = (-d + b);
					sD = a;
				}
			}

			sc = (std::fabs(sN) < COINC_THRESH ? 0.0 : sN / sD);
			tc = (std::fabs(tN) < COINC_THRESH ? 0.0 : tN / tD);

			s = sc;
			t = tc;
			f_point_on_seg_1 = (1.0 - sc) * f_P0 + sc * f_P1;
			f_point_on_seg_2 = (1.0 - tc) * f_Q0 + tc * f_Q1;
			CPoint3 diff = f_point_on_seg_1 - f_point_on_seg_2;
			f_dist_squared = Dot(diff, diff);

			if (l_segs_parallel)
			{
				bool l_collinear = Cross(w, u).LengthSq() < l_eps2;
				if (!l_collinear)
					return ESegmentRelation::ParallelDisjoint;

				double uu = Dot(u, u);
				double t0 = Dot((f_Q0 - f_P0), u) / uu;
				double t1 = Dot((f_Q1 - f_P0), u) / uu;

				double l_min_t = std::min(t0, t1);
				double l_max_t = std::max(t0, t1);

				double l_overlap_min = std::max(0.0, l_min_t);
				double l_overlap_max = std::min(1.0, l_max_t);

				if (l_overlap_max < l_overlap_min)
					return ESegmentRelation::ParallelDisjoint;

				if (std::fabs(l_overlap_max - l_overlap_min) < COINC_THRESH)
					return ESegmentRelation::CollinearTouchingExtremes;

				return ESegmentRelation::CollinearlOverlapping;
			}

			if (f_dist_squared < l_eps2)
			{
				bool l_touch_at_endpoint = (s <= COINC_THRESH ||
					s >= 1.0 - COINC_THRESH ||
					t <= COINC_THRESH ||
					t >= 1.0 - COINC_THRESH);

				if (l_touch_at_endpoint)
					return ESegmentRelation::NonParallelTouchPoint;

				return ESegmentRelation::Intersecting;
			}

			return ESegmentRelation::NonParallelDisjoint;

		}

		/**
		* @brief Computes the shortest distance between two line segments in 3D space.
		*
		* This function calculates the minimum Euclidean distance between two finite 3D line segments:
		* segment 1 defined by points `f_P0` and `f_P1`, and segment 2 defined by points `f_Q0` and `f_Q1`.
		* It handles degenerate and nearly-parallel cases robustly using a numerical threshold.
		*
		* The method finds the closest points on each segment by solving a system of equations that
		* minimizes the squared distance between points on each segment parameterized by scalar values.
		*
		* @param f_P0 The start point of the first segment.
		* @param f_P1 The end point of the first segment.
		* @param f_Q0 The start point of the second segment.
		* @param f_Q1 The end point of the second segment.
		*
		* @return The shortest distance between the two 3D line segments.
		*/
		inline double SegmentSegmentDistance3D(const CPoint3& f_P0,
											   const CPoint3& f_P1,
											   const CPoint3& f_Q0,
											   const CPoint3& f_Q1)
		{
			double l_dist_squared = 0.0;
			double s = 0.0;
			double t = 0.0;
			CPoint3 l_p1, l_p2;

			SegmentSegmentDistance3D(f_P0, f_P1,
								    f_Q0, f_Q1,
								    l_dist_squared, s, t,
								    l_p1, l_p2);
			if (l_dist_squared < COINC_THRESH * COINC_THRESH)
				return 0.0;

			return std::sqrt(l_dist_squared);
		}

		/**
		* @brief Clips a 2D segment against an axis-aligned box (Liang-Barsky).
		*
		* The segment is written in parametric form P(t) = f_seg_start + t * D, with
		* D = f_seg_end - f_seg_start and t in [0, 1]. Each of the four box sides
		* restricts the admissible range of t; the clipped segment is the part of the
		* original segment for t in [t_min, t_max], the intersection of all these
		* ranges.
		*
		* The box boundary is inclusive: a segment that only touches the box (along
		* a side or at a corner) is considered inside, and the clipped segment may
		* then have zero length. A degenerate segment (f_seg_start == f_seg_end) is
		* accepted if the point lies inside the box or on its boundary.
		*
		* @param[in]  f_seg_start     Start point of the segment.
		* @param[in]  f_seg_end       End point of the segment.
		* @param[in]  f_x_min_box     Minimum X of the box.
		* @param[in]  f_x_max_box     Maximum X of the box.
		* @param[in]  f_y_min_box     Minimum Y of the box.
		* @param[in]  f_y_max_box     Maximum Y of the box.
		* @param[out] f_clipped_start Start point of the clipped segment (on the side
		*                             of f_seg_start). Unchanged if false is returned.
		* @param[out] f_clipped_end   End point of the clipped segment (on the side
		*                             of f_seg_end). Unchanged if false is returned.
		*
		* @return true if the segment intersects or touches the box (the clipped
		*         endpoints are then valid), false if it lies completely outside.
		*
		* @note No tolerance is used: points exactly on the boundary are inside,
		*       points outside by any amount are outside.
		*
		* @see Y.-D. Liang, B. A. Barsky, "A New Concept and Method for Line
		*      Clipping", ACM Transactions on Graphics 3(1), pp. 1-18, 1984.
		* @see Liang-Barsky algorithm, e.g. https://en.wikipedia.org/wiki/Liang%E2%80%93Barsky_algorithm
		*/
		inline bool ClipSegment2DToBox(const CPoint2& f_seg_start,
									   const CPoint2& f_seg_end,
									   const double f_x_min_box,
									   const double f_x_max_box,
									   const double f_y_min_box,
									   const double f_y_max_box,
									   CPoint2& f_clipped_start,
									   CPoint2& f_clipped_end)
		{
			const double l_dx = f_seg_end.m_x - f_seg_start.m_x;
			const double l_dy = f_seg_end.m_y - f_seg_start.m_y;

			double l_t_min = 0.0;
			double l_t_max = 1.0;

			const double p[] = { -l_dx, l_dx, -l_dy, l_dy };

			const double q[] = { f_seg_start.m_x - f_x_min_box,
								 f_x_max_box - f_seg_start.m_x,
								 f_seg_start.m_y - f_y_min_box,
								 f_y_max_box - f_seg_start.m_y };

			for (int i = 0; i < 4; ++i)
			{
				if (p[i] == 0.0)
				{
					if (q[i] < 0.0)
					{
						return false;
					}
				}
				else
				{
					const double t = q[i] / p[i];

					if (p[i] < 0.0)
					{
						l_t_min = std::max(l_t_min, t);
					}
					else
					{
						l_t_max = std::min(l_t_max, t);
					}

					if (l_t_min > l_t_max)
					{
						return false;
					}
				}
			}

			f_clipped_start.m_x = f_seg_start.m_x + l_t_min * l_dx;
			f_clipped_start.m_y = f_seg_start.m_y + l_t_min * l_dy;

			f_clipped_end.m_x = f_seg_start.m_x + l_t_max * l_dx;
			f_clipped_end.m_y = f_seg_start.m_y + l_t_max * l_dy;

			return true;
		}

		/**
		* @brief Checks whether a 2D segment intersects an axis-aligned 2D box.
		*
		* The box is treated as a filled region: the function returns true if any
		* point of the segment lies inside the box or on its boundary. This includes
		* segments completely contained in the box, segments crossing it, and
		* segments only touching it along a side or at a corner. A degenerate segment
		* (f_seg_start == f_seg_end) intersects the box if the point lies inside it
		* or on its boundary.
		*
		* Implemented on top of ClipSegment2DToBox() (Liang-Barsky algorithm),
		* discarding the clipped endpoints.
		*
		* @param[in] f_seg_start Start point of the segment.
		* @param[in] f_seg_end   End point of the segment.
		* @param[in] f_x_min_box Minimum X of the box.
		* @param[in] f_x_max_box Maximum X of the box.
		* @param[in] f_y_min_box Minimum Y of the box.
		* @param[in] f_y_max_box Maximum Y of the box.
		*
		* @return true if the segment intersects or touches the box, false if it lies
		*         completely outside.
		*
		* @note No tolerance is used: points exactly on the boundary are inside.
		*
		* @see ClipSegment2DToBox()
		* @see Y.-D. Liang, B. A. Barsky, "A New Concept and Method for Line
		*      Clipping", ACM Transactions on Graphics 3(1), pp. 1-18, 1984.
		*/
		inline bool Segment2DBoxIntersection(const CPoint2& f_seg_start,
											 const CPoint2& f_seg_end,
											 const double f_x_min_box,
											 const double f_x_max_box,
											 const double f_y_min_box,
											 const double f_y_max_box)
		{
			CPoint2 l_clipped_start;
			CPoint2 l_clipped_end;
			return ClipSegment2DToBox(f_seg_start, f_seg_end,
									  f_x_min_box, f_x_max_box,
									  f_y_min_box, f_y_max_box,
									  l_clipped_start,
									  l_clipped_end);
		}

		/**
		* @brief Determines whether a point lies strictly inside a 2D triangle.
		*
		* Computes the 2D cross products of the vectors from @p f_P to each pair of
		* vertices. The point is strictly inside when all three cross products are
		* non-zero and have the same sign. This works for both clockwise and
		* counterclockwise triangles.
		*
		* Points on the boundary (edges or vertices) are considered outside, and no
		* tolerance is used. For an inclusive test with tolerance, see
		* IsPointInsideTriangle2DInclusive().
		*
		* @param[in] f_A First vertex of the triangle.
		* @param[in] f_B Second vertex of the triangle.
		* @param[in] f_C Third vertex of the triangle.
		* @param[in] f_P Point to test.
		*
		* @return true if @p f_P lies strictly inside the triangle, false if it lies
		*         outside, on its boundary, or if the triangle is degenerate.
		*/
		inline bool IsPointInsideTriangle2DStrictly(const CPoint2& f_A,
													const CPoint2& f_B,
													const CPoint2& f_C,
													const CPoint2& f_P)
		{
			CPoint2 l_v0(f_P - f_A);
			CPoint2 l_v1(f_P - f_B);
			CPoint2 l_v2(f_P - f_C);

			double l_cross01 = l_v0.m_x * l_v1.m_y - l_v0.m_y * l_v1.m_x;
			double l_cross12 = l_v1.m_x * l_v2.m_y - l_v1.m_y * l_v2.m_x;
			double l_cross20 = l_v2.m_x * l_v0.m_y - l_v2.m_y * l_v0.m_x;

			double l_pr0 = l_cross01 * l_cross12;
			if (!(l_pr0 > 0.0))
			{
				return false;
			}

			double l_pr1 = l_cross12 * l_cross20;
			if (!(l_pr1 > 0.0))
			{
				return false;
			}

			double l_pr2 = l_cross20 * l_cross01;
			if (!(l_pr2 > 0.0))
			{
				return false;
			}

			return true;

		}

		/**
		* @brief Determines whether a point lies inside a 2D triangle or on its
		*        boundary, within a tolerance.
		*
		* Computes the 2D cross products of the vectors from @p f_P to each pair of
		* vertices. Cross products whose absolute value is at most @p f_eps are
		* treated as zero (point on the corresponding edge line). The point is
		* accepted unless two cross products have strictly opposite signs. This works
		* for both clockwise and counterclockwise triangles.
		*
		* @param[in] f_A   First vertex of the triangle.
		* @param[in] f_B   Second vertex of the triangle.
		* @param[in] f_C   Third vertex of the triangle.
		* @param[in] f_P   Point to test.
		* @param[in] f_eps Non-negative tolerance on the cross products.
		*
		* @return true if @p f_P lies inside the triangle or on its boundary (within
		*         the tolerance), false otherwise.
		*
		* @warning @p f_eps is compared with cross products, which have units of
		*          length squared (twice the area of the triangle formed by @p f_P
		*          and two vertices). It is therefore not a distance tolerance, and
		*          its effect depends on the scale of the coordinates and the size
		*          of the triangle.
		*
		* @warning The triangle must not be degenerate: if its vertices are
		*          collinear, all cross products are zero for every point on that
		*          line, and such points are accepted even when they lie outside the
		*          segment spanned by the vertices.
		*/
		inline bool IsPointInsideTriangle2DInclusive(const CPoint2& f_A,
													 const CPoint2& f_B,
													 const CPoint2& f_C,
													 const CPoint2& f_P,
													 const double f_eps)
		{
			const CPoint2 l_v0(f_P - f_A);
			const CPoint2 l_v1(f_P - f_B);
			const CPoint2 l_v2(f_P - f_C);

			const double l_cross01 = l_v0.m_x * l_v1.m_y - l_v0.m_y * l_v1.m_x;
			const double l_cross12 = l_v1.m_x * l_v2.m_y - l_v1.m_y * l_v2.m_x;
			const double l_cross20 = l_v2.m_x * l_v0.m_y - l_v2.m_y * l_v0.m_x;

			const bool l_has_neg = (l_cross01 < -f_eps) || (l_cross12 < -f_eps) || (l_cross20 < -f_eps);
			const bool l_has_pos = (l_cross01 > f_eps) || (l_cross12 > f_eps) || (l_cross20 > f_eps);
			return !(l_has_neg && l_has_pos);
		}

		/**
		* @brief Determines whether a point lies inside a 3D triangle.
		*
		* This function checks whether the point `f_P` lies inside the triangle defined by
		* the three 3D vertices `f_A`, `f_B`, and `f_C`. It returns `true` if the point is inside
		* or on the edges of the triangle, and `false` otherwise.
		*
		* @param f_A The first vertex of the triangle.
		* @param f_B The second vertex of the triangle.
		* @param f_C The third vertex of the triangle.
		* @param f_P The point to test for inclusion in the triangle.
		*
		* @return `true` if the point lies inside or on the triangle, `false` otherwise.
		*/
		inline bool IsPointInsideTriangle3D(const CPoint3& f_A,
											const CPoint3& f_B,
											const CPoint3& f_C,
											CPoint3& f_P)
		{
			CPoint3 l_v0(f_P - f_A);
			CPoint3 l_v1(f_P - f_B);
			CPoint3 l_v2(f_P - f_C);

			CPoint3 l_cross01 = Cross(l_v0, l_v1);
			CPoint3 l_cross12 = Cross(l_v1, l_v2);
			CPoint3 l_cross20 = Cross(l_v2, l_v0);

			double l_pr0 = Dot(l_cross01, l_cross12);
			if (!(l_pr0 > 0.0))
				return false;

			double l_pr1 = Dot(l_cross12, l_cross20);
			if (!(l_pr1 > 0.0))
				return false;

			double l_pr2 = Dot(l_cross20, l_cross01);
			if (!(l_pr2 > 0.0))
				return false;


			return true;

		}

		/**
		* @brief Calculates the area of a triangle given the lengths of its three sides.
		*
		* This function computes the area of a triangle using the lengths of its sides
		* (`f_a`, `f_b`, and `f_c`) based on Heron's formula. The computed area is stored
		* in the output parameter `f_area`.
		*
		* The function performs a validity check to ensure that the provided side lengths
		* can form a valid triangle. If the triangle is invalid (e.g., the triangle inequality
		* is not satisfied), the function returns `false`.
		*
		* @param f_a	Length of the first side of the triangle.
		* @param f_b	Length of the second side of the triangle.
		* @param f_c	Length of the third side of the triangle.
		* @param f_area Reference to a variable where the computed area will be stored.
		*
		* @return `true` if the area was successfully calculated, `false` if the triangle is invalid.
		*/
		inline int CalculateTriangleAreaFromSidesLength(double f_a,
														double f_b,
														double f_c,
														double& f_area)
		{
			if ((f_a < 0) || (f_b < 0) || (f_c < 0) || (f_a + f_b <= f_c) || (f_a + f_c <= f_b) || (f_b + f_c <= f_a))
			{
				return false;
			}
			double s = (f_a + f_b + f_c) / 2;
			f_area = sqrt(s * (s - f_a) * (s - f_b) * (s - f_c));
			return true;
		}

		/**
		* @brief Calculates the centroid of a triangle given its three vertices.
		*
		* This function computes the centroid (geometric center) of a triangle defined
		* by the 3D vertices `f_v1`, `f_v2`, and `f_v3`. The result is stored in the output
		* parameter `f_centroid`.
		*
		* The centroid is the point where the triangle's medians intersect and is computed
		* as the average of the three vertex coordinates.
		*
		* @param f_v1		The first vertex of the triangle.
		* @param f_v2		The second vertex of the triangle.
		* @param f_v3		The third vertex of the triangle.
		* @param f_centroid Reference to a variable where the computed centroid will be stored.
		
		* @return `true` if the centroid was successfully computed.
		*/
		inline bool CalculateTriangleCentroidFromVertexes(const CPoint3& f_v1,
														  const CPoint3& f_v2,
														  const CPoint3& f_v3,
														  CPoint3& f_centroid)
		{
			f_centroid.m_x = (f_v1.m_x + f_v2.m_x + f_v3.m_x) / 3;
			f_centroid.m_y = (f_v1.m_y + f_v2.m_y + f_v3.m_y) / 3;
			f_centroid.m_z = (f_v1.m_z + f_v2.m_z + f_v3.m_z) / 3;

			return true;
		}

		/**
		* @brief Computes the point of a 3D triangle closest to a given point.
		*
		* Determines which Voronoi feature region of the triangle (one of the three
		* vertex regions, one of the three edge regions, or the face region) contains
		* @p f_p, and returns the orthogonal projection of @p f_p onto that feature.
		* Only dot products are used; no square roots.
		*
		* @param[in] f_p Query point.
		* @param[in] f_a First vertex of the triangle.
		* @param[in] f_b Second vertex of the triangle.
		* @param[in] f_c Third vertex of the triangle.
		*
		* @return The point of triangle (f_a, f_b, f_c) closest to @p f_p (on the
		*         triangle's face, edges or vertices).
		*
		* @warning The triangle must not be degenerate (collinear vertices): in that
		*          case the face-region computation can divide by zero.
		*
		* @note Code from Real-Time Collision Detection by Christer Ericson,
		*       published by Morgan Kaufmann Publishers, (c) 2005 Elsevier Inc
		*       (ClosestPtPointTriangle, section 5.1.5), adapted to GGEOMLIB types and
		*       double precision.
		*/
		inline CPoint3 ClosestPointOnTriangle(const CPoint3& f_p,
											  const CPoint3& f_a,
											  const CPoint3& f_b,
											  const CPoint3& f_c)
		{
			const CPoint3 l_ab = f_b - f_a;
			const CPoint3 l_ac = f_c - f_a;
			const CPoint3 l_ap = f_p - f_a;
			const double l_d1 = Dot(l_ab, l_ap);
			const double l_d2 = Dot(l_ac, l_ap);
			if (l_d1 <= 0.0 && l_d2 <= 0.0)
			{
				return f_a;
			}

			const CPoint3 l_bp = f_p - f_b;
			const double l_d3 = Dot(l_ab, l_bp);
			const double l_d4 = Dot(l_ac, l_bp);
			if (l_d3 >= 0.0 && l_d4 <= l_d3)
			{
				return f_b;
			}

			const double l_vc = l_d1 * l_d4 - l_d3 * l_d2;
			if (l_vc <= 0.0 && l_d1 >= 0.0 && l_d3 <= 0.0)
			{
				const double l_v = l_d1 / (l_d1 - l_d3);
				return f_a + l_v * l_ab;
			}

			const CPoint3 l_cp = f_p - f_c;
			const double l_d5 = Dot(l_ab, l_cp);
			const double l_d6 = Dot(l_ac, l_cp);
			if (l_d6 >= 0.0 && l_d5 <= l_d6)
			{
				return f_c;
			}

			const double l_vb = l_d5 * l_d2 - l_d1 * l_d6;
			if (l_vb <= 0.0 && l_d2 >= 0.0 && l_d6 <= 0.0)
			{
				const double l_w = l_d2 / (l_d2 - l_d6);
				return f_a + l_w * l_ac;
			}

			const double l_va = l_d3 * l_d6 - l_d5 * l_d4;
			if (l_va <= 0.0 && (l_d4 - l_d3) >= 0.0 && (l_d5 - l_d6) >= 0.0)
			{
				const double l_w = (l_d4 - l_d3) / ((l_d4 - l_d3) + (l_d5 - l_d6));
				return f_b + l_w * (f_c - f_b);
			}

			const double l_denom = 1.0 / (l_va + l_vb + l_vc);
			const double l_v = l_vb * l_denom;
			const double l_w = l_vc * l_denom;
			return f_a + l_ab * l_v + l_ac * l_w;
		}

		/**
		* @brief Tests whether a triangle and a sphere intersect in 3D.
		*
		* Computes the point of the triangle closest to the sphere centre with
		* ClosestPointOnTriangle(); the triangle and the sphere intersect if that
		* point lies inside the sphere or on its surface.
		*
		* The triangle and the sphere are treated as solids: the function returns
		* true when they overlap or touch. No tolerance is used.
		*
		* @param[in]  f_a              First vertex of the triangle.
		* @param[in]  f_b              Second vertex of the triangle.
		* @param[in]  f_c              Third vertex of the triangle.
		* @param[in]  f_sphere_centre  Centre of the sphere.
		* @param[in]  f_sphere_radius  Radius of the sphere (non-negative).
		* @param[out] f_closest_point  Optional: if not null, receives the point of
		*                              the triangle closest to the sphere centre
		*                              (written whether or not they intersect).
		*
		* @return true if the triangle and the sphere intersect or touch, false
		*         otherwise.
		*
		* @warning The triangle must not be degenerate (see ClosestPointOnTriangle()).
		*
		* @note Code from Real-Time Collision Detection by Christer Ericson,
		*       published by Morgan Kaufmann Publishers, (c) 2005 Elsevier Inc
		*       (TestSphereTriangle, section 5.2.7), adapted to GGEOMLIB types and
		*       double precision.
		*/
		inline bool SphereTriangleCollision(const CPoint3& f_a,
											const CPoint3& f_b,
											const CPoint3& f_c,
											const CPoint3& f_sphere_centre,
											const double f_sphere_radius,
											CPoint3* f_closest_point = nullptr)
		{
			const CPoint3 l_p = ClosestPointOnTriangle(f_sphere_centre, f_a, f_b, f_c);
			if (f_closest_point)
			{
				*f_closest_point = l_p;
			}

			const CPoint3 l_v = l_p - f_sphere_centre;
			return Dot(l_v, l_v) <= f_sphere_radius * f_sphere_radius;
		}

		/**
		* @brief Determines the spatial relationship between a point and a sphere in 3D space.
		*
		* This function checks whether a given point `f_point` lies inside, outside, or exactly on the
		* surface of a sphere defined by its center `f_center` and radius `f_radius`.
		*
		* The result is returned as an integer code:
		* - `1`: The point is inside the sphere.
		* - `2`: The point is outside the sphere.
		* - `3`: The point lies exactly on the surface of the sphere.
		* - `4`: An undefined state occurred; this should be treated as an error.
		*
		* @param f_point	The point to test.
		* @param f_center	The center of the sphere.
		* @param f_radius	The radius of the sphere.
		*
		* @return An integer code representing the point's position relative to the sphere.
		* @retval 1 Point is inside the sphere.
		* @retval 2 Point is outside the sphere.
		* @retval 3 Point is exactly on the sphere's surface.
		* @retval 4 Undefined result (error condition).
		*/
		inline int IsPointInsideSphere(const CPoint3& f_point,
									   const CPoint3& f_center,
									   double f_radius)
		{
			double l_resultant = 0.0;

			l_resultant = (f_point.m_x - f_center.m_x) * (f_point.m_x - f_center.m_x) +
						  (f_point.m_y - f_center.m_y) * (f_point.m_y - f_center.m_y) +
						  (f_point.m_z - f_center.m_z) * (f_point.m_z - f_center.m_z);

			double l_square_radius = f_radius * f_radius;

			if (l_resultant < l_square_radius)
			{
				return 1;
			}
			if ((l_resultant - l_square_radius) < COINC_THRESH)
			{
				return 3;
			}
			if (l_resultant > l_square_radius)
			{
				return 2;
			}

			return 4;
		}

		/**
		* @brief Computes the intersection points of a circle with the X and Y axes in the XY plane.
		*
		* Given a circle defined by its center `f_center` and radius `f_radius`, this function calculates
		* the points where the circle intersects the Cartesian X and Y axes of a reference frame whose
		* origin is located at the circle's center.
		*
		* The resulting intersection points on the Y axis are stored in `f_y_pt_up` (above origin) and
		* `f_y_pt_down` (below origin), while the intersection points on the X axis are stored in
		* `f_x_pt_left` (left of origin) and `f_x_pt_right` (right of origin).
		*
		* @param f_radius The radius of the circle.
		* @param f_center The center of the circle (origin of the reference system).
		* @param[out] f_y_pt_up		The intersection point on the positive Y axis.
		* @param[out] f_y_pt_down	The intersection point on the negative Y axis.
		* @param[out] f_x_pt_left	The intersection point on the negative X axis.
		* @param[out] f_x_pt_right	The intersection point on the positive X axis.
		*/
		inline void GetXYCrossingPointsOnXYPlane(const double f_radius,
												 const CPoint2& f_center,
												 CPoint2& f_y_pt_up,
												 CPoint2& f_y_pt_down,
												 CPoint2& f_x_pt_left,
												 CPoint2& f_x_pt_right)
		{
			if (f_radius < COINC_THRESH)
			{
				return;
			}

			f_y_pt_up.m_x = 0.0;
			f_y_pt_up.m_y = f_center.m_y + f_radius;
			f_y_pt_down.m_x = 0.0;
			f_y_pt_down.m_y = f_center.m_y - f_radius;

			f_x_pt_left.m_x = f_center.m_x - f_radius;
			f_x_pt_left.m_y = 0.0;
			f_x_pt_right.m_x = f_center.m_x + f_radius;
			f_x_pt_right.m_y = 0.0;
		}

		/**
		* @brief Approximates a circular arc using a polyline with linear segments within a specified chordal error.
		*
		* This function generates a series of points (`f_points`) that approximate a circular arc defined by
		* a center point (`f_center`), radius (`f_radius`), and start and end points (`f_p_start` and `f_p_end`).
		* The approximation is controlled by a maximum allowable chordal error (`f_chord_err`), which dictates
		* the maximum deviation between the true arc and its polyline approximation.
		*
		* If the distance between `f_p_start` and `f_p_end` is less than a defined COINCIDENCE_THRESHOLD,
		* the arc is considered a full circle.
		*
		* The arc is always assumed to be in a counterclockwise (CCW) direction. For details on the conventions
		* used to determine arc convexity or concavity, see QGV-17 p70.
		*
		* @param f_chord_err			The maximum allowable chordal error for the approximation.
		* @param f_radius				The radius of the circular arc, equal to the distance from the center to the start or end point.
		* @param f_center				The center of the circular arc.
		* @param f_p_start				The starting point of the arc.
		* @param f_p_end				The ending point of the arc.
		* @param f_points				A vector to be filled with the interpolated points approximating the arc.
		* @param f_add_closure_point	If true and the arc is a full circle, adds the starting point as the last point to close the profile; otherwise, does not add it.
		*/
		inline void ApproximateCircularArc(const double f_chord_err,
										   const double f_radius,
										   const CPoint2& f_center,
										   const CPoint2& f_p_start,
										   const CPoint2& f_p_end,
										   std::vector<CPoint2>& f_points,
										   const bool f_add_closure_point)
		{
			assert(fabs(f_radius - Distance(f_center, f_p_start)) < LOOSE_THRESH);
			assert(fabs(f_radius - Distance(f_center, f_p_end)) < LOOSE_THRESH);

			CPoint2 l_start_vec = (f_p_start - f_center).UnitizeInPlace();
			CPoint2 l_end_vec = (f_p_end - f_center).UnitizeInPlace();
			double l_start_angle_rad = math_algs::Modulo(atan2(l_start_vec.m_y, l_start_vec.m_x), 0.0, TWO_PI_GRECO, TWO_PI_GRECO);
			double l_end_angle_rad = math_algs::Modulo(atan2(l_end_vec.m_y, l_end_vec.m_x), 0.0, TWO_PI_GRECO, TWO_PI_GRECO);

			double l_chord_length = Distance(f_p_start, f_p_end);
			bool l_full_circle = false;
			if (l_chord_length < COINC_THRESH)
			{
				l_full_circle = true;
			}

			double l_tot_angle_rad = 0.0;
			if (l_full_circle)
			{
				l_tot_angle_rad = TWO_PI_GRECO;
			}
			else
			{
				l_tot_angle_rad = math_algs::Modulo(l_end_angle_rad - l_start_angle_rad, 0.0, TWO_PI_GRECO, TWO_PI_GRECO);
			}
			double l_angle_sec_rad = 2 * math_algs::GuardedACos(1 - (f_chord_err / f_radius));
			int l_num_segms = (int)(l_tot_angle_rad / l_angle_sec_rad + 0.5);
			l_angle_sec_rad = l_tot_angle_rad / l_num_segms;

			f_points.reserve(l_num_segms + 1);
			f_points.emplace_back(f_p_start);
			for (int l_ipoint = 1; l_ipoint < l_num_segms; l_ipoint++)
			{
				double l_curr_angle = math_algs::Modulo(l_ipoint * l_angle_sec_rad, 0.0, TWO_PI_GRECO, TWO_PI_GRECO);
				double l_x_interp = f_center.m_x + f_radius * cos(l_start_angle_rad + l_curr_angle);
				double l_y_interp = f_center.m_y + f_radius * sin(l_start_angle_rad + l_curr_angle);
				f_points.emplace_back(l_x_interp, l_y_interp);
			}
			if ((!f_add_closure_point) && l_full_circle)
			{
				return;
			}
			f_points.emplace_back(f_p_end);
		}

		/**
		* @brief Interpolates a fixed number of points on a circular arc in 2D space.
		*
		* This function generates exactly `f_num_points` points that lie on a circular arc defined by
		* its center (`f_center`), radius (`f_radius`), and start/end points (`f_p_start` and `f_p_end`).
		* Unlike chordal error-based approximation, this function uses a fixed number of points, so the
		* approximation accuracy depends on the chosen number of points.
		*
		* If the distance between `f_p_start` and `f_p_end` is less than a defined COINCIDENCE_THRESHOLD,
		* the arc is treated as a full circle.
		*
		* The arc is assumed to be counterclockwise (CCW). For conventions regarding arc convexity and concavity,
		* see QGV-17 p70.
		*
		* @param f_num_points			The number of points to interpolate on the arc (fixed, no chordal error control).
		* @param f_radius				The radius of the circular arc.
		* @param f_center				The center point of the circular arc.
		* @param f_p_start				The starting point of the arc.
		* @param f_p_end				The ending point of the arc.
		* @param f_points				Vector to store the interpolated points on the arc.
		* @param f_add_closure_point	If true and the arc is a full circle, adds the first point at the end to close the profile.
		*/
		inline void InterpolatePointsOnCircularArc(const int f_num_points,
												   const double f_radius,
												   const CPoint2& f_center,
												   const CPoint2& f_p_start,
												   const CPoint2& f_p_end,
												   std::vector<CPoint2>& f_points,
												   const bool f_add_closure_point)
		{
			assert(fabs(f_radius - Distance(f_center, f_p_start)) < LOOSE_THRESH);
			assert(fabs(f_radius - Distance(f_center, f_p_end)) < LOOSE_THRESH);

			CPoint2 l_start_vec = (f_p_start - f_center).UnitizeInPlace();
			CPoint2 l_end_vec = (f_p_end - f_center).UnitizeInPlace();
			double l_start_angle_rad = math_algs::Modulo(atan2(l_start_vec.m_y, l_start_vec.m_x), 0.0, TWO_PI_GRECO, TWO_PI_GRECO);
			double l_end_angle_rad = math_algs::Modulo(atan2(l_end_vec.m_y, l_end_vec.m_x), 0.0, TWO_PI_GRECO, TWO_PI_GRECO);

			double l_chord_length = Distance(f_p_start, f_p_end);
			bool l_full_circle = false;
			if (l_chord_length < COINC_THRESH)
			{
				l_full_circle = true;
			}

			double l_tot_angle_rad = 0.0;
			if (l_full_circle)
			{
				l_tot_angle_rad = TWO_PI_GRECO;
			}
			else
			{
				l_tot_angle_rad = math_algs::Modulo(l_end_angle_rad - l_start_angle_rad, 0.0, TWO_PI_GRECO, TWO_PI_GRECO);
			}

			f_points.reserve(f_num_points);
			f_points.emplace_back(f_p_start);
			double l_angle_sec_rad = l_tot_angle_rad / f_num_points;
			for (int l_ipoint = 1; l_ipoint < f_num_points; l_ipoint++)
			{
				double l_curr_angle = math_algs::Modulo(l_ipoint * l_angle_sec_rad, 0.0, TWO_PI_GRECO, TWO_PI_GRECO);
				double l_x_interp = f_center.m_x + f_radius * cos(l_start_angle_rad + l_curr_angle);
				double l_y_interp = f_center.m_y + f_radius * sin(l_start_angle_rad + l_curr_angle);
				f_points.emplace_back(l_x_interp, l_y_interp);
			}
			if ((!f_add_closure_point) && l_full_circle)
			{
				return;
			}
			f_points.emplace_back(f_p_end);
		}

		/**
		* @brief Computes the intersection points between a horizontal line and a
		*        circle.
		*
		* Finds the intersection points (if any) between the horizontal line
		* y = f_y0 and the circle with centre f_center = (Xc, Yc) and radius
		* f_radius = R.
		*
		* The circle equation is:
		*
		*     (x - Xc)^2 + (y - Yc)^2 = R^2
		*
		* Substituting y = f_y0 gives a quadratic equation in x:
		*
		*     x^2 - 2*Xc*x + (Xc^2 - R^2 + (f_y0 - Yc)^2) = 0
		*
		* that is, a*x^2 + b*x + c = 0 with:
		*
		*     a = 1
		*     b = -2*Xc
		*     c = Xc^2 - R^2 + (f_y0 - Yc)^2
		*
		* Its real roots are the X-coordinates of the intersection points. If the
		* two roots differ by less than COINC_THRESH, the line is considered tangent
		* to the circle and a single point is returned.
		*
		* @param[in]  f_center        Centre of the circle (Xc, Yc).
		* @param[in]  f_radius        Radius of the circle.
		* @param[in]  f_y0            Y-coordinate of the horizontal line.
		* @param[out] f_intersections Intersection points, ordered by increasing X.
		*                             Cleared on entry, so it is empty if there is
		*                             no intersection.
		*
		* @return Number of intersection points found (0, 1 or 2).
		*/
		inline int ComputeHorizontalLineCircleIntersection(const CPoint2& f_center,
														   const double f_radius,
														   const double f_y0,
														   std::vector<CPoint2>& f_intersections)
		{
			f_intersections.clear();
			const double l_a = 1.0;
			const double l_b = -2 * f_center.m_x;
			const double l_c = (f_center.m_x * f_center.m_x) - (f_radius * f_radius) + (f_y0 - f_center.m_y) * (f_y0 - f_center.m_y);
			double l_x_left = 0.0;
			double l_x_right = 0.0;
			bool l_sol_quad = math_algs::SolveQuadraticEquation(l_a, l_b, l_c, l_x_right, l_x_left);
			if (l_sol_quad)
			{
				if (fabs(l_x_right - l_x_left) < COINC_THRESH)
				{
					f_intersections.emplace_back(l_x_left, f_y0);
					return 1;
				}
				f_intersections.emplace_back(l_x_left, f_y0);
				f_intersections.emplace_back(l_x_right, f_y0);
				return 2;
			}

			return 0;
		}

		/**
		* @brief Computes the intersection points between a horizontal line and a
		*        circular ring (annulus).
		*
		* The ring is the region between two concentric circles with centre
		* f_center = (Xc, Yc), inner radius r = f_inner_radius and outer radius
		* R = f_outer_radius (R > r). The line is y = f_y0.
		*
		* With dy = |f_y0 - Yc| (distance of the line from the centre), there are
		* three cases:
		*
		*     dy >= R - COINC_THRESH            -> no intersection (0 points)
		*     r - COINC_THRESH <= dy < R - COINC_THRESH
		*                                       -> the line crosses only the outer
		*                                          circle: one segment (2 points)
		*     dy < r - COINC_THRESH             -> the line crosses both circles:
		*                                          two segments (4 points)
		*
		* Tangencies are not reported: a line tangent to the outer circle gives
		* 0 points, and a line tangent to the inner circle gives 2 points (one
		* segment, the tangency point being ignored).
		*
		* The points are ordered by increasing X. With 4 points, (p0, p1) and
		* (p2, p3) are the two segments of the line lying inside the ring:
		*
		*     p0 = outer circle, left    p1 = inner circle, left
		*     p2 = inner circle, right   p3 = outer circle, right
		*
		* @param[in]  f_center        Centre of the two circles (Xc, Yc).
		* @param[in]  f_inner_radius  Radius of the inner circle (r).
		* @param[in]  f_outer_radius  Radius of the outer circle (R), with R > r.
		* @param[in]  f_y0            Y-coordinate of the horizontal line.
		* @param[out] f_intersections Intersection points, ordered by increasing X.
		*                             Cleared on entry, so it is empty if there is
		*                             no intersection.
		*
		* @return Number of intersection points found (0, 2 or 4).
		*/
		inline int ComputeHorizontalLineCircleRingInters(const CPoint2& f_center,
														 const double f_inner_radius,
														 const double f_outer_radius,
														 const double f_y0,
														 std::vector<CPoint2>& f_intersections)
		{
			f_intersections.clear();

			const double l_dy = std::fabs(f_y0 - f_center.m_y);
			if (l_dy - f_outer_radius >= -COINC_THRESH)
			{
				return 0;
			}

			if (l_dy - f_inner_radius < -COINC_THRESH)
			{
				std::vector<CPoint2> l_intersections_outer;
				ComputeHorizontalLineCircleIntersection(f_center, f_outer_radius, f_y0, l_intersections_outer);
				std::vector<CPoint2> l_intersections_inner;
				ComputeHorizontalLineCircleIntersection(f_center, f_inner_radius, f_y0, l_intersections_inner);

				f_intersections.reserve(4);
				f_intersections.emplace_back(l_intersections_outer.front());
				f_intersections.emplace_back(l_intersections_inner.front());
				f_intersections.emplace_back(l_intersections_inner.back());
				f_intersections.emplace_back(l_intersections_outer.back());
				return 4;
			}

			ComputeHorizontalLineCircleIntersection(f_center, f_outer_radius, f_y0, f_intersections);
			return 2;
		}

		/**
		* @brief Calculates the perpendicular distance from a point to a line segment in 3D space.
		*
		* This function computes the shortest distance from a point `f_p` to the line defined
		* by the two points `f_a` and `f_b`. The computation uses the cross product to find
		* the area of the parallelogram formed by the vectors `(f_p - f_a)` and `(f_p - f_b)`,
		* and divides by the length of the base `(f_a - f_b)` to obtain the height (distance).
		*
		* @param f_a The first point defining the line.
		* @param f_b The second point defining the line.
		* @param f_p The point from which the distance to the line is measured.
		* @return The perpendicular distance from point `f_p` to the line defined by `f_a` and `f_b`.
		*/
		inline double PointLineDistance(const CPoint3& f_a, const CPoint3& f_b, const CPoint3& f_p)
		{
			return (Cross((f_p - f_a), (f_p - f_b)).Length() / (f_a - f_b).Length());
		}

		/**
		* @brief Computes the squared distance from a point to a line segment in 2D space,
		*        including projections that fall outside the segment.
		*
		* This function calculates the squared distance from a point `f_pt` to the closest point
		* on the line segment defined by `f_pt_a` and `f_pt_b`. If the perpendicular projection
		* of the point falls outside the segment, the function still returns the distance to
		* the nearest segment endpoint.
		*
		* @param f_pt_a The start point of the segment.
		* @param f_pt_b The end point of the segment.
		* @param f_pt The point from which the distance is measured.
		*
		* @return The squared distance from `f_pt` to the segment [`f_pt_a`, `f_pt_b`], allowing
		*         for projections that fall outside the segment bounds.
		*/
		inline double PointSegmentDistanceSQIncludingProjectionOutside(const CPoint2& f_pt_a,
																	   const CPoint2& f_pt_b,
																	   const CPoint2& f_pt)
		{
			CPoint2 l_vec_p_a = f_pt - f_pt_a;
			CPoint2 l_vec_a_b = f_pt_b - f_pt_a;
			double l_param = Dot(l_vec_p_a, l_vec_a_b) / l_vec_a_b.LengthSq();
			CPoint2 l_dist_seg = l_vec_p_a;

			if (l_param >= 1)
			{
				l_dist_seg = l_vec_p_a - l_vec_a_b;
			}
			else if (l_param > 0)
			{
				l_dist_seg = l_vec_p_a - (l_vec_a_b * l_param);
			}

			return l_dist_seg.LengthSq();
		}

		/**
		* @brief Computes the axis-aligned bounding box (AABB) of a cylinder.
		*
		* The cylinder has its base centre at f_cylinder_center (C), unit axis
		* direction f_cylinder_axis (n), length f_cylinder_length (L) and radius
		* f_cylinder_radius (r). Its two end caps are disks of radius r centred at
		*
		*     P0 = C          and          P1 = C + L * n
		*
		* The half-extents of the AABB of a disk with unit normal n and radius r
		* are, for each axis:
		*
		*     e.x = r * sqrt(1 - n.x^2)
		*     e.y = r * sqrt(1 - n.y^2)
		*     e.z = r * sqrt(1 - n.z^2)
		*
		* The cylinder's AABB is the union of the AABBs of the two end caps,
		* [P0 - e, P0 + e] and [P1 - e, P1 + e], enlarged by f_padding_threshold
		* on every side.
		*
		* @param[in]  f_cylinder_axis     Unit vector along the cylinder axis, from
		*                                 the base towards the top.
		* @param[in]  f_cylinder_center   Centre of the base of the cylinder (e.g. the
		*                                 TCP for a tool), not its midpoint.
		* @param[in]  f_cylinder_length   Length of the cylinder along its axis.
		* @param[in]  f_cylinder_radius   Radius of the cylinder.
		* @param[out] f_ll_vert           Minimum corner of the AABB. Unchanged if
		*                                 false is returned.
		* @param[out] f_ur_vert           Maximum corner of the AABB. Unchanged if
		*                                 false is returned.
		* @param[in]  f_padding_threshold Amount added to the box on every side
		*                                 (0 for a tight box).
		*
		* @return true if the box was computed, false if f_cylinder_axis is not a
		*         unit vector (its length differs from 1 by more than COINC_THRESH).
		*
		* @see I. Quilez, "Disk - bounding box",
		*      https://iquilezles.org/articles/diskbbox/
		*/
		inline bool GetCylinderAABB(const CPoint3& f_cylinder_axis,
									const CPoint3& f_cylinder_center,
									const double f_cylinder_length,
								    const double f_cylinder_radius,
									CPoint3& f_ll_vert,
									CPoint3& f_ur_vert,
									const double f_padding_threshold)
		{
			if (fabs(f_cylinder_axis.Length() - 1.0) > COINC_THRESH)
			{
				return false;
			}

			CPoint3 l_max_pt = f_cylinder_center + f_cylinder_axis * f_cylinder_length;
			CPoint3 l_e;
			l_e.m_x = f_cylinder_radius * sqrt(1.0 - f_cylinder_axis.m_x * f_cylinder_axis.m_x);
			l_e.m_y = f_cylinder_radius * sqrt(1.0 - f_cylinder_axis.m_y * f_cylinder_axis.m_y);
			l_e.m_z = f_cylinder_radius * sqrt(1.0 - f_cylinder_axis.m_z * f_cylinder_axis.m_z);

			CPoint3 l_vec_1 = f_cylinder_center - l_e;
			CPoint3 l_vec_2 = l_max_pt - l_e;
			f_ll_vert.m_x = std::min(l_vec_1.m_x, l_vec_2.m_x) - f_padding_threshold;
			f_ll_vert.m_y = std::min(l_vec_1.m_y, l_vec_2.m_y) - f_padding_threshold;
			f_ll_vert.m_z = std::min(l_vec_1.m_z, l_vec_2.m_z) - f_padding_threshold;

			CPoint3 l_vec_3 = f_cylinder_center + l_e;
			CPoint3 l_vec_4 = l_max_pt + l_e;
			f_ur_vert.m_x = std::max(l_vec_3.m_x, l_vec_4.m_x) + f_padding_threshold;
			f_ur_vert.m_y = std::max(l_vec_3.m_y, l_vec_4.m_y) + f_padding_threshold;
			f_ur_vert.m_z = std::max(l_vec_3.m_z, l_vec_4.m_z) + f_padding_threshold;

			return true;
		}

		/**
		* @brief Calculates the area of a profile (polygon) projected onto the XY plane.
		*
		* Uses the shoelace formula (also known as the surveyor's formula):
		*
		*     A = 1/2 * | sum_i ( x_i * y_(i+1) - x_(i+1) * y_i ) |
		*
		* summing over consecutive vertex pairs and wrapping from the last vertex
		* back to the first. Only the X and Y coordinates are used; Z is ignored, so
		* for a profile lying in a plane other than XY the result is the area of its
		* projection, not its true area.
		*
		* @param[in] f_prf_pts Profile vertices, in order. The profile may be stored
		*                      either closed (last point equal to the first) or open
		*                      (the closing edge is implied): a repeated closing
		*                      point adds a zero term to the sum.
		*
		* @return Area of the profile (always non-negative), or 0.0 if fewer than 3
		*         points are given.
		*
		* @note The profile must be simple (non-self-intersecting). For a
		*       self-intersecting profile, regions with opposite orientation cancel
		*       out and the result is not its geometric area.
		*
		* @see https://en.wikipedia.org/wiki/Shoelace_formula
		*/
		inline double CalculateProfileArea(const std::vector<CPoint3> &f_prf_pts)
		{
			double l_prf_area = 0.0;

			size_t l_prf_size = f_prf_pts.size();
			if (l_prf_size < 3)
			{
				return 0.0;
			}

			for (size_t i = 0; i < l_prf_size; ++i)
			{
				const auto& p1 = f_prf_pts[i];
				const auto& p2 = f_prf_pts[(i + 1) % l_prf_size];
				l_prf_area += (p1.m_x * p2.m_y) - (p2.m_x * p1.m_y);
			}

			l_prf_area /= 2.0;
			l_prf_area = std::fabs(l_prf_area);
			return l_prf_area;
		}

		/**
		* @brief Calculates the area of a 2D profile (polygon).
		*
		* Uses the shoelace formula:
		*
		*     A = 1/2 * | sum_i ( x_i * y_(i+1) - x_(i+1) * y_i ) |
		*
		* summing over consecutive vertex pairs and wrapping from the last vertex
		* back to the first.
		*
		* @param[in] f_prf_pts Profile vertices, in order. The profile may be stored
		*                      either closed (last point equal to the first) or open
		*                      (the closing edge is implied).
		*
		* @return Area of the profile (always non-negative), or 0.0 if fewer than 3
		*         points are given.
		*
		* @note The profile must be simple (non-self-intersecting).
		*
		* @see CalculateProfileArea(const std::vector<CPoint3>&)
		* @see https://en.wikipedia.org/wiki/Shoelace_formula
		*/
		inline double CalculateProfileArea(const std::vector<CPoint2>& f_prf_pts)
		{
			double l_prf_area = 0.0;

			size_t l_prf_size = f_prf_pts.size();
			if (l_prf_size < 3)
			{
				return 0.0;
			}

			for (size_t i = 0; i < l_prf_size; ++i)
			{
				const auto& p1 = f_prf_pts[i];
				const auto& p2 = f_prf_pts[(i + 1) % l_prf_size];
				l_prf_area += (p1.m_x * p2.m_y) - (p2.m_x * p1.m_y);
			}

			l_prf_area /= 2.0;
			l_prf_area = std::fabs(l_prf_area);
			return l_prf_area;
		}

		/**
		* @brief Determines whether a closed 2D profile (polygon) is oriented
		*        clockwise.
		*
		* Computes twice the signed area of the profile with the shoelace formula:
		*
		*     S = sum_i ( x_i * y_(i+1) - x_(i+1) * y_i )
		*
		* S > 0 means counterclockwise, S < 0 means clockwise.
		*
		* @param[in] f_prf Closed profile: the last point must be equal to the
		*                  first. The last point is skipped as a duplicate.
		*
		* @return true if the profile is clockwise; false if it is counterclockwise,
		*         degenerate (zero area), or has fewer than 3 points.
		*
		* @warning The profile must be stored closed. For an open profile (last point
		*          different from the first), the last vertex is wrongly discarded
		*          and the result may be incorrect.
		*
		* @note The profile must be simple (non-self-intersecting) for the
		*       orientation to be meaningful.
		*
		* @see https://en.wikipedia.org/wiki/Shoelace_formula
		*/
		inline bool IsPrfCW(const std::vector<CPoint2>& f_prf)
		{
			size_t n = f_prf.size();
			if (n < 3)
			{
				return false;
			}
			n = n - 1;

			double l_tot_sum = 0.0;
			for (size_t i = 0; i < n; ++i)
			{
				size_t j = (i + 1) % n;
				l_tot_sum += f_prf[i].m_x * f_prf[j].m_y - f_prf[j].m_x * f_prf[i].m_y;
			}


			return l_tot_sum < 0.0;
		}

		/**
		* @brief Determines whether a closed profile (polygon), projected onto the XY
		*        plane, is oriented clockwise.
		*
		* Same as IsPrfCW(const std::vector<CPoint2>&), using only the X and Y
		* coordinates of the points (Z is ignored). The orientation is therefore the
		* one seen looking down the Z axis (from +Z towards -Z).
		*
		* @param[in] f_prf Closed profile: the last point must be equal to the
		*                  first. The last point is skipped as a duplicate.
		*
		* @return true if the profile is clockwise; false if it is counterclockwise,
		*         degenerate (zero area), or has fewer than 3 points.
		*
		* @warning The profile must be stored closed. For an open profile (last point
		*          different from the first), the last vertex is wrongly discarded
		*          and the result may be incorrect.
		*
		* @see IsPrfCW(const std::vector<CPoint2>&)
		*/
		inline bool IsPrfCW(const std::vector<CPoint3>& f_prf)
		{
			size_t n = f_prf.size();
			if (n < 3)
			{
				return false;
			}
			n = n - 1;

			double l_tot_sum = 0.0;
			for (size_t i = 0; i < n; ++i)
			{
				size_t j = (i + 1) % n;
				l_tot_sum += f_prf[i].m_x * f_prf[j].m_y - f_prf[j].m_x * f_prf[i].m_y;
			}


			return l_tot_sum < 0.0;
		}

		/**
		* @brief Computes the intersection point between a plane and a 3D segment,
		*        if it exists.
		*
		* The plane is given by the equation
		*
		*     n . X + D = 0        (i.e. A*x + B*y + C*z + D = 0)
		*
		* with n = (A, B, C) = f_plane_normal and D = f_plane_fourth_coeff.
		* The segment is written as
		*
		*     X(t) = P0 + t * u,   0 <= t <= L
		*
		* with P0 = f_seg_start, u the unit direction from f_seg_start to f_seg_end
		* and L the segment length. Substituting into the plane equation gives
		*
		*     t = -(n . P0 + D) / (n . u)
		*
		* and the segment intersects the plane if 0 <= t <= L. Because u is a unit
		* vector, t is the distance of the intersection point from f_seg_start.
		*
		* Special cases:
		* - Degenerate segments (length below COINC_THRESH) are rejected.
		* - If |n . u| < f_parallelism_threshold, the segment is considered
		*   parallel to the plane and false is returned, even if it lies in the
		*   plane (infinitely many intersections).
		* - No tolerance is applied to the bounds 0 <= t <= L: an intersection at
		*   an endpoint may be rejected because of round-off.
		*
		* @param[in]  f_plane_normal          Normal of the plane (n). Its length
		*                                     does not affect the intersection
		*                                     point, but it scales n . u and thus
		*                                     the meaning of the parallelism
		*                                     threshold (see below).
		* @param[in]  f_plane_fourth_coeff    Fourth coefficient (D) of the plane
		*                                     equation.
		* @param[in]  f_seg_start             Start point of the segment (P0).
		* @param[in]  f_seg_end               End point of the segment.
		* @param[out] f_int_pt                Intersection point. Unchanged if false
		*                                     is returned.
		* @param[in]  f_parallelism_threshold Threshold on |n . u|. With a unit
		*                                     normal, n . u is the cosine of the
		*                                     angle between the segment and the
		*                                     normal, i.e. the sine of the angle
		*                                     between the segment and the plane.
		*
		* @return true if the segment intersects the plane within its length,
		*         false otherwise.
		*
		* @see P. J. Schneider, D. H. Eberly, "Geometric Tools for Computer
		*      Graphics", Morgan Kaufmann, 2003, p. 484.
		*/
		inline bool PlaneSegmentIntersection(const CPoint3&f_plane_normal,
										     const double f_plane_fourth_coeff,
										     const CPoint3 f_seg_start,
										     const CPoint3 f_seg_end,
										     CPoint3& f_int_pt,
										     const double f_parallelism_threshold)
		{
			double l_segm_len = Distance(f_seg_end, f_seg_start);
			if (l_segm_len < COINC_THRESH)
			{
				return false;
			}

			CPoint3 l_segm_dir = (f_seg_end - f_seg_start).UnitizeInPlace();

			double l_denominator = Dot(l_segm_dir, f_plane_normal);
			if (fabs(l_denominator) < f_parallelism_threshold)
			{

				return false;
			}

			double l_t = -((f_plane_normal.m_x * f_seg_start.m_x) +
						  (f_plane_normal.m_y * f_seg_start.m_y) +
						  (f_plane_normal.m_z * f_seg_start.m_z) + f_plane_fourth_coeff);

			l_t = l_t / l_denominator;
			if (l_t < 0.0)
			{
				return false;
			}
			if (l_t > l_segm_len)
			{
				return false;
			}

			f_int_pt = f_seg_start + l_t * l_segm_dir;

			return true;
		}

		/**
		* @brief Computes the clockwise polar angle of a 3D point projected onto the XY plane.
		*
		* The angle is measured from the positive X-axis and increases in the clockwise
		* direction. The returned value is normalized to the range [0, 2PI_GRECO).
		*
		* @param f_p Input point (only x and y components are used).
		* @return Clockwise angle in radians in the range [0, 2PI_GRECO).
		*/
		inline double GetClockwiseAngle(const CPoint3& f_p)
		{
			double l_angle = std::atan2(f_p.m_y, f_p.m_x);
			l_angle = -l_angle;
			if (l_angle < 0)
			{
				l_angle += 2.0 * PI_GRECO;
			}
			return l_angle;
		}

		/**
		* @brief Computes the clockwise polar angle of a 2D point
		*
		* The angle is measured from the positive X-axis and increases in the clockwise
		* direction. The returned value is normalized to the range [0, 2PI_GRECO).
		*
		* @param	f_p Input point 
		* 
		* @return	Clockwise angle in radians in the range [0, 2PI_GRECO).
		*/
		inline double GetClockwiseAngle(const CPoint2& f_p)
		{
			double l_angle = std::atan2(f_p.m_y, f_p.m_x);
			l_angle = -l_angle;
			if (l_angle < 0)
			{
				l_angle += 2.0 * PI_GRECO;
			}
			return l_angle;
		}

		/**
		* @brief Sorts points around the origin based on their polar angle.
		*
		* Points are ordered according to their angular position around the origin
		* in the XY plane. The sorting direction can be either clockwise or
		* counterclockwise.
		*
		* @param f_points_to_sort Vector of points to be sorted.
		* @param f_clockwise If true, points are sorted clockwise; if false,
		*                    points are sorted counterclockwise.
		*/
		inline void SortPointsRespectToPole(std::vector<CPoint3>& f_points_to_sort, const bool f_clockwise)
		{
			std::sort(f_points_to_sort.begin(), f_points_to_sort.end(),
				[f_clockwise](const CPoint3& f_a, const CPoint3& f_b)
				{
					double l_angle_a = GetClockwiseAngle(f_a);
					double l_angle_b = GetClockwiseAngle(f_b);

					if (f_clockwise)
					{
						return l_angle_a < l_angle_b;
					}
					else
					{
						return l_angle_a > l_angle_b;
					}
				});
		}

		/**
		* @brief Computes the intersections between a 2D ray and a circle.
		*
		* The ray is written as X(t) = P + t * d, t >= 0, with P the ray origin and
		* d its unit direction; the circle has centre C and radius r. With
		* w = P - C, substituting X(t) into |X - C|^2 = r^2 gives
		*
		*     t^2 + 2 (d . w) t + (w . w - r^2) = 0
		*
		* whose roots are
		*
		*     t = -(d . w) +/- sqrt(delta),   delta = (d . w)^2 - (w . w - r^2)
		*
		* - delta < -COINC_THRESH: the line misses the circle (0 intersections).
		* - |delta| < COINC_THRESH: the line is tangent to the circle (one double
		*   root).
		* - otherwise: the line crosses the circle at two points.
		*
		* Only roots with t >= 0 (in front of the ray origin) are reported, in
		* ascending order of t. If the ray origin is inside the circle, only the
		* exit point is in front of it.
		*
		* @param[in]  f_circle_center Centre of the circle (C).
		* @param[in]  f_circle_radius Radius of the circle (r).
		* @param[in]  f_ray           Ray to test. Its direction must be a unit
		*                             vector: the formula above assumes it, and t
		*                             is then the distance from the ray origin.
		* @param[out] f_int_pt_1      First intersection point (closest to the ray
		*                             origin). Valid if the return value is >= 1.
		* @param[out] f_int_pt_2      Second intersection point. Valid only if the
		*                             return value is 2.
		* @param[out] f_tparam_1      Ray parameter of f_int_pt_1. Valid if the
		*                             return value is >= 1.
		* @param[out] f_tparam_2      Ray parameter of f_int_pt_2. Valid only if the
		*                             return value is 2.
		*
		* @return Number of intersections in front of the ray origin:
		*         - 0: no intersection (line misses the circle, or the circle lies
		*           behind the ray origin);
		*         - 1: the ray is tangent to the circle, or its origin is inside the
		*           circle (only the exit point is reported);
		*         - 2: the ray enters and exits the circle.
		*         When 0 is returned, the output values are unspecified.
		*
		* @note The tangency tolerance COINC_THRESH is applied to delta, which has
		*       units of length squared, so its effect depends on the scale of the
		*       coordinates.
		*
		* @see P. J. Schneider, D. H. Eberly, "Geometric Tools for Computer
		*      Graphics", Morgan Kaufmann, 2003, p. 248.
		*/
		inline int RayCircleIntersection(const CPoint2& f_circle_center,
									     const double f_circle_radius,
									     const CRay2D& f_ray,
									     CPoint2& f_int_pt_1,
									     CPoint2& f_int_pt_2,
									     double& f_tparam_1,
									     double& f_tparam_2)
		{
			CPoint2 l_delta_vec = f_ray.GetPStart() - f_circle_center;
			double l_delta = std::pow(Dot(f_ray.GetDir(), l_delta_vec), 2) - (l_delta_vec.LengthSq() - f_circle_radius * f_circle_radius);
			if (l_delta < -COINC_THRESH)
			{
				return 0;
			}
			if (fabs(l_delta) < COINC_THRESH)
			{
				l_delta = 0.0;
			}
			f_tparam_1 = -Dot(f_ray.GetDir(), l_delta_vec) + sqrt(l_delta);
			f_tparam_2 = -Dot(f_ray.GetDir(), l_delta_vec) - sqrt(l_delta);
			if (l_delta == 0.0)
			{
				if (f_tparam_1 < 0.0)
				{
					return 0;
				}

				f_int_pt_1 = f_ray.GetPStart() + f_tparam_1 * f_ray.GetDir();
				return 1;
			}

			if (f_tparam_1 > f_tparam_2)
			{
				std::swap(f_tparam_1, f_tparam_2);
			}

			f_int_pt_1 = f_ray.GetPStart() + f_tparam_1 * f_ray.GetDir();
			f_int_pt_2 = f_ray.GetPStart() + f_tparam_2 * f_ray.GetDir();

			if ((f_tparam_1) < 0.0 && (f_tparam_2 < 0.0))
			{
				return 0;
			}
			if ((f_tparam_1) < 0.0 && (f_tparam_2 >= 0))
			{
				f_int_pt_1 = f_int_pt_2;
				f_tparam_1 = f_tparam_2;
				return 1;
			}

			return 2;
		}

		/**
		* @brief Computes the intersection between a 2D line segment and a circle.
		*
		* This function tests a finite line segment against a circle and computes
		* up to two intersection points that lie within the segment extents.
		*
		* The function returns:
		* - 0 if the segment does not intersect the circle
		* - 1 if the segment is tangent to the circle or intersects at exactly one point
		* - 2 if the segment enters and exits the circle
		*
		* Intersection points are reported in the order they appear along the segment
		* from start to end.
		*
		* @param f_circle_center Center of the circle.
		* @param f_circle_radius Radius of the circle.
		* @param f_seg_start     Start point of the segment.
		* @param f_seg_end       End point of the segment.
		* @param f_int_pt_1      First intersection point along the segment.
		* @param f_int_pt_2      Second intersection point along the segment.
		*
		* @return Number of intersections (0, 1, or 2).
		*/
		inline int SegmentCircleIntersection(const CPoint2& f_circle_center,
										     const double f_circle_radius,
										     const CPoint2& f_seg_start,
										     const CPoint2& f_seg_end,
										     CPoint2& f_int_pt_1,
										     CPoint2& f_int_pt_2)
		{
			CRay2D l_ray(f_seg_start, (f_seg_end - f_seg_start).UnitizeInPlace());
			const double l_seg_lenght = (f_seg_end - f_seg_start).Length();
			double l_t1 = 0.0;
			double l_t2 = 0.0;
			const int l_ray_circle_int = RayCircleIntersection(f_circle_center, f_circle_radius, l_ray, f_int_pt_1, f_int_pt_2, l_t1, l_t2);
			if (!l_ray_circle_int)
			{
				return 0;
			}

			if (fabs(l_t1 - l_t2) < COINC_THRESH)
			{
				if (l_t1 >= 0.0 && l_t1 <= l_seg_lenght)
				{
					f_int_pt_1 = l_ray.GetPStart() + l_t1 * l_ray.GetDir();
					return 1;
				}
				return 0;
			}

			int l_intersections_count = 0;
			if (l_t1 >= 0.0 && l_t1 <= l_seg_lenght)
			{
				f_int_pt_1 = l_ray.GetPStart() + l_t1 * l_ray.GetDir();
				++l_intersections_count;
			}

			if (l_t2 >= 0.0 && l_t2 <= l_seg_lenght)
			{
				if (l_intersections_count == 0)
				{
					f_int_pt_1 = l_ray.GetPStart() + l_t2 * l_ray.GetDir();
				}
				else
				{
					f_int_pt_2 = l_ray.GetPStart() + l_t2 * l_ray.GetDir();
				}

				++l_intersections_count;
			}

			return l_intersections_count;
		}

		/**
		* @brief Checks whether a 2D line segment lies entirely inside a circle.
		*
		* Determines if both endpoints of the segment are within the circle
		* defined by its center and radius. If both points are striclty inside
		* the circle (i.e. not on the boundary), the segment is considered 
		* strictly inside.
		*
		* @param f_circle_center The center of the circle.
		* @param f_circle_radius The radius of the circle. Must be non-negative.
		* @param f_seg_start The start point of the segment.
		* @param f_seg_end The end point of the segment.
		*
		* @return true if both segment endpoints are inside or on the circle boundary;
		*         false otherwise.
		*
		* @note This function does not check for segment intersections with the circle
		*       boundary—only the endpoints are tested.
		*/
		inline bool IsSegmentStrictlyInsideCircle(const CPoint2& f_circle_center,
												  double f_circle_radius,
												  const CPoint2& f_seg_start,
												  const CPoint2& f_seg_end)
		{
			double l_dist_start = (f_seg_start - f_circle_center).Length();
			double l_dist_end = (f_seg_end - f_circle_center).Length();

			return (l_dist_start < f_circle_radius) && (l_dist_end < f_circle_radius);
		}

		/**
		* @brief Computes the angle between two 3D vectors in radians.
		*
		* Both input vectors are internally normalized before computing the dot
		* product. The resulting angle is always unsigned and lies in the range
		* [0, PI_GRECO].
		*
		* @param f_v1 First 3D vector
		* @param f_v2 Second 3D vector
		* @return Angle between the vectors in radians
		*/
		inline double AngleBetweenVectorsRad3D(const CPoint3& f_v1, const CPoint3& f_v2)
		{
			CPoint3 l_v1 = f_v1;
			CPoint3 l_v2 = f_v2;
			l_v1.UnitizeInPlace();
			l_v2.UnitizeInPlace();

			double l_dot = Dot(l_v1, l_v2);
			if (l_dot > 1.0)
			{
				l_dot = 1.0;
			}
			if (l_dot < -1.0)
			{
				l_dot = -1.0;
			}

			return acos(l_dot);
		}

		/**
		* @brief Computes the angle between two 3D vectors in degrees.
		*
		* This function wraps AngleBetweenVectorsRad3D() and converts the result
		* from radians to degrees.
		*
		* @param f_v1 First 3D vector
		* @param f_v2 Second 3D vector
		* @return Angle between the vectors in degrees
		*/
		inline double AngleBetweenVectorsDeg3D(const CPoint3& f_v1, const CPoint3& f_v2)
		{
			const double l_angle = AngleBetweenVectorsRad3D(f_v1, f_v2);
			return l_angle * RAD_TO_DEG_FACTOR;
		}

		/**
		* @brief Computes the angle between two 2D vectors in radians.
		*
		* The vectors are implicitly promoted to 3D (with a zero Z component)
		* and the angle is computed using the 3D implementation.
		* The resulting angle is unsigned and lies in the range [0, PI_GRECO].
		*
		* @param f_v1 First 2D vector
		* @param f_v2 Second 2D vector
		* @return Angle between the vectors in radians
		*/
		inline double AngleBetweenVectorsRad2D(const CPoint2& f_v1, const CPoint2& f_v2)
		{
			return AngleBetweenVectorsRad3D(CPoint3(f_v1.m_x, f_v1.m_y, 0.0), CPoint3(f_v2.m_x, f_v2.m_y, 0.0));
		}

		/**
		* @brief Computes the angle between two 2D vectors in degrees.
		*
		* This function wraps AngleBetweenVectorsRad2D() and converts the result
		* from radians to degrees.
		*
		* @param f_v1 First 2D vector
		* @param f_v2 Second 2D vector
		* @return Angle between the vectors in degrees
		*/
		inline double AngleBetweenVectorsDeg2D(const CPoint2& f_v1, const CPoint2& f_v2)
		{
			return AngleBetweenVectorsRad2D(f_v1, f_v2) * RAD_TO_DEG_FACTOR;
		}

		/**
		* @brief Moves a 3D vector to lie on the surface of a cone defined by a semi-
		*		 aperture angle (expressed in deg) and an axis vector
		*
		* If the vector is inside the cone, it is projected onto the cone surface 
		* corresponding to the given threshold angle. The vector is normalized before 
		* and after the operation.
		*
		* @param f_vec			The vector to be moved. This vector will be modified.
		* @param f_cone_axis	The axis of the cone
		* @param f_cone_semi_angle_deg  The cone threshold angle in degrees.
		*/
		inline void MoveVectorToCone(CPoint3& f_vec, 				
								     const double f_cone_semi_angle_deg,
								     const CPoint3& f_cone_axis = CPoint3(0.0,0.0,1.0))
		{
			f_vec.UnitizeInPlace();
			CPoint3 l_axis = f_cone_axis;
			l_axis.UnitizeInPlace();

			double l_cos = cos(f_cone_semi_angle_deg * DEG_TO_RAD_FACTOR);
			double sinTheta = sin(f_cone_semi_angle_deg * DEG_TO_RAD_FACTOR);

			double l_dot = Dot(f_vec, l_axis);

			if (l_dot >= l_cos)
			{
				CPoint3 l_v_perp = f_vec - l_axis * l_dot;
				l_v_perp.UnitizeInPlace();

				f_vec = l_axis * l_cos + l_v_perp * sinTheta;
				f_vec.UnitizeInPlace();
			}
		}

		/**
		* @brief Checks if a 3D vector lies within a cone defined by a semi-aperture
		*		 angle (expressed in deg) and an axis vector
		*
		* The cone is defined by its axis and a maximum angle from the axis. The function
		* returns true if the vector is inside the cone (i.e. angle with cone axis is 
		* less than the threshold), false otherwise.
		*
		* @param f_vec					The vector to check.
		* @param f_cone_axis			The axis of the cone we check the vector against
		* @param f_cone_semi_angle_deg	The cone angle in degrees.
		* @return true if the vector is inside the cone, false otherwise.
		*/
		inline bool CheckVectorInCone(const CPoint3& f_vec, 
								      const double f_cone_semi_angle_deg,
								      const CPoint3& f_cone_axis = CPoint3(0.0,0.0,1.0))
		{
			CPoint3 l_axis = f_cone_axis;
			l_axis.UnitizeInPlace();

			double l_cos = cos(f_cone_semi_angle_deg * DEG_TO_RAD_FACTOR);
			double l_dot = Dot(f_vec, l_axis);

			return (l_dot >= l_cos);
		}
		
		/**
		* @brief Detects whether four consecutive points form a geometric inversion.
		*
		* This function analyzes four consecutive 3D points to determine whether
		* the polyline locally inverts direction (i.e., forms a short backtracking
		* segment between two almost aligned segments).
		*
		* The detection is based on:
		* - Directional similarity between the first and third segments.
		* - Opposite direction between the first and second segments.
		* - A maximum allowed inversion width in the XY plane.
		* A width check ensures that large geometric features are not
		* mistakenly removed as small inversions.
		*
		* @param f_p0 First point.
		* @param f_p1 Second point.
		* @param f_p2 Third point.
		* @param f_p3 Fourth point.
		*
		* @return true if the four points define a small inversion;
		*         false otherwise.
		*/
		inline bool IsInversion(const CPoint3& f_p0,
							    const CPoint3& f_p1,
							    const CPoint3& f_p2,
							    const CPoint3& f_p3)
		{
			CPoint3 V1 = (f_p1 - f_p0).UnitizeInPlace();
			CPoint3 V2 = (f_p2 - f_p1).UnitizeInPlace();
			CPoint3 V3 = (f_p3 - f_p2).UnitizeInPlace();

			if (Dot(V1, V3) <= 0.5)
			{
				return false;
			}
			if (Dot(V1, V2) >= -0.5)
			{
				return false;
			}

			CPoint2 l_proj_pt;
			double dist = 0.0;

			cgeom_algs::PointProjectionOnSegment(CPoint2(f_p1.m_x, f_p1.m_y),
												 CPoint2(f_p2.m_x, f_p2.m_y),
												 CPoint2(f_p3.m_x, f_p3.m_y),
												 l_proj_pt,
												 dist);
			return (dist < 0.1);
		}

		/**
		* @brief Determines whether a point is the apex of a thin spike.
		*
		* This function evaluates whether a point forms a spike between its
		* previous and next neighbors. A spike is defined as a sharp and short
		* deviation that can be safely removed without affecting the intended
		* profile geometry.
		*
		* The algorithm:
		* - Computes lengths of adjacent segments.
		* - If lengths are equal, directly checks the distance between
		*   previous and next points.
		* - If lengths differ, computes the perpendicular distance between
		*   the shorter segment endpoint and the longer segment.
		* - Compares the computed distance against f_spike_threshold.
		*
		* @param f_point The candidate spike apex (current point).
		* @param f_next_point The next point in the profile.
		* @param f_prev_point The previous point in the profile.
		* @param f_spike_threshold Maximum allowed deviation to classify the point as a spike.
		*
		* @return true if the point is classified as a spike apex;
		*         false otherwise.
		*
		* @note
		* - The geometric evaluation is performed in the XY plane.
		*/
		inline bool IsPointSpikeApex(const CPoint3& f_point,
								     const CPoint3& f_next_point,
								     const CPoint3& f_prev_point,
								     const double f_spike_threshold)
		{
			bool l_is_spike = false;
			double l_first_length = 0.0;
			double l_second_length = 0.0;

			l_first_length = Distance(f_point, f_prev_point);
			l_second_length = Distance(f_point, f_next_point);

			CPoint2 l_first_pt_first_segm(f_prev_point.m_x, f_prev_point.m_y);
			CPoint2 l_second_pt_first_segm(f_point.m_x, f_point.m_y);
			CPoint2 l_second_pt_second_segm(f_next_point.m_x, f_next_point.m_y);
			CPoint2 l_first_pt_second_segm(f_point.m_x, f_point.m_y);

			if (fabs(l_first_length - l_second_length) < COINC_THRESH)
			{
				double l_distance = Distance(f_prev_point, f_next_point);
				if (l_distance - f_spike_threshold < COINC_THRESH)
				{
					l_is_spike = true;
				}

			}
			else
			{
				if ((l_first_length - l_second_length) < 0.0)
				{
					double l_dist = cgeom_algs::PointSegmentDistance(l_first_pt_first_segm, l_first_pt_second_segm, l_second_pt_second_segm);
					if (l_dist > 0.0)
					{
						if (l_dist - f_spike_threshold < COINC_THRESH)
						{
							l_is_spike = true;
						}
					}
				}
				else if ((l_first_length - l_second_length) > 0.0)
				{
					double l_dist = cgeom_algs::PointSegmentDistance(l_second_pt_second_segm, l_first_pt_first_segm, l_second_pt_first_segm);
					if (l_dist > 0.0)
					{
						if (l_dist - f_spike_threshold < COINC_THRESH)
						{
							l_is_spike = true;
						}
					}
				}
			}
			
			return l_is_spike;

		}
	}
}

#endif