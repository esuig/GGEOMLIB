
#ifndef H_CPOLYLINE
#define H_CPOLYLINE

#pragma once

#include <fstream>
#include <deque>
#include <map>

#include "CompGeomAlgorithms.h"
#include "CAABox.h"
#include "CQuadtree.h"
#include "CPoint3.h"



namespace ggeomlib
{	
	class CPolylinesList;

	/**
	* @class CPolyline
	* @brief Represents a complete toolpath profile consisting of multiple toolpath points.
	*
	* The profile is defined as an ordered sequence of @ref CPoint3 objects and includes
	* additional geometric and topological information such as:
	* - Profile type classification (e.g., working, link, additive)
	* - Computed area, orientation, and bounding box
	* - Support for geometric queries (point containment, closure, etc.)
	*
	*/
	class CPolyline : public std::vector<CPoint3>
	{
	public:

		/**
		* @brief Default constructor for CPolyline.
		*
		* Initializes the profile with default values:
		* - Area = 0.0
		* - Orientation = false (counter-clockwise)
		*/
		CPolyline():
			m_area(0.0),
			m_clockwise(false),
			m_valid(true)
		{

		}

		/**
		* @brief Checks whether the profile is closed.
		*
		* A profile is considered closed if the first and last points are within
		* a small positional tolerance of each other.
		*
		* @return True if the profile is closed, false otherwise.
		*/
		inline bool CheckIfClosed() const
		{
			return ((!empty()) && (Distance(back(), front()) < DOUBLE_PRECISION_EPSILON));
		}

		/**
		* @brief Calculates the total length of the profile.
		*
		* The length is computed as the sum of distances between consecutive points.
		*
		* @return Total profile length.
		*/
		inline double CalculateLength() const
		{
			double l_tot_length = 0.0;
			for (auto l_pt_it = begin() + 1; l_pt_it != end(); l_pt_it++)
			{
				l_tot_length += Distance(*l_pt_it, *(l_pt_it - 1));
			}

			return l_tot_length;
		}

		/**
		* @brief Calculates the signed area and orientation of the profile.
		*
		* The area is computed using a planar projection, typically onto XY.
		* Orientation (clockwise or counter-clockwise) is derived from the sign of the computed area.
		*
		* @note The calculated area is stored internally in @ref m_area.
		*/
		inline void CalculateAreaAndOrientation()
		{
			double area = 0.0;
			const double l_zero_threshold = DOUBLE_PRECISION_EPSILON;

			size_t n = this->size();
			if (n < 3)
			{
				SetArea(0.0);
				SetPrfCW(false);
				return;
			}

			for (size_t i = 0; i < n; ++i)
			{
				const auto& p1 = (*this)[i];
				const auto& p2 = (*this)[(i + 1) % n];
				area += (p1.m_x * p2.m_y) - (p2.m_x * p1.m_y);
			}

			area /= 2.0;
			double abs_area = std::fabs(area);

			if (abs_area > l_zero_threshold)
			{
				SetArea(abs_area);
				if (area < 0.0)
				{
					SetPrfCW(true);
				}
				else
				{
					SetPrfCW(false);
				}
			}
			else
			{
				SetArea(0.0);
				SetPrfCW(false);
			}
		}

		/**
		* @brief Computes the axis-aligned bounding box (AABB) for the profile.
		*
		* Updates the internal @ref m_bounding_box member.
		*/
		void CalculateBoundingBox()
		{
			CPoint3 l_ll_bbox_vert;
			CPoint3 l_ur_bbox_vert;
			CalculateBoundingBoxOnLine(l_ll_bbox_vert, l_ur_bbox_vert);
			m_bounding_box.SetLLVert(l_ll_bbox_vert);
			m_bounding_box.SetURVert(l_ur_bbox_vert);
		}

		/**
		* @brief Computes the axis-aligned bounding box (AABB) for the profile. Computation is done 
		*		 inside the function
		*
		* @param f_ll_vert	The low left vertex of the bounding box of the profile
		* @param f_ur_vert	The up right vertex of the bounding box of the profile
		* 
		*/
		inline void CalculateBoundingBoxOnLine(CPoint3& f_ll_vert, CPoint3&f_ur_vert) const
		{
			if (size() > 0)
			{
				f_ll_vert = front();
				f_ur_vert = front();

				for (auto l_pt_it = begin(); l_pt_it != end(); l_pt_it++)
				{
					if (l_pt_it->m_x > f_ur_vert.m_x)
					{
						f_ur_vert.m_x = l_pt_it->m_x;
					}
					if (l_pt_it->m_x < f_ll_vert.m_x)
					{
						f_ll_vert.m_x = l_pt_it->m_x;
					}

					if (l_pt_it->m_y > f_ur_vert.m_y)
					{
						f_ur_vert.m_y = l_pt_it->m_y;
					}
					if (l_pt_it->m_y < f_ll_vert.m_y)
					{
						f_ll_vert.m_y = l_pt_it->m_y;
					}

					if (l_pt_it->m_z > f_ur_vert.m_z)
					{
						f_ur_vert.m_z = l_pt_it->m_z;
					}
					if (l_pt_it->m_z < f_ll_vert.m_z)
					{
						f_ll_vert.m_z = l_pt_it->m_z;
					}
				}
			}
		}

		/**
		* @brief Gets the computed profile area.
		* @return Absolute value of the area.
		*/
		inline double GetArea() const
		{
			return m_area;
		}
		/**
		* @brief Manually sets the stored profile area.
		* @param f_area Area value to assign.
		*/
		inline void SetArea(const double f_area)
		{
			m_area = f_area;
		}

		/**
		* @brief Checks if the profile orientation is clockwise.
		* @return True if clockwise, false otherwise.
		*/
		inline bool IsPrfCW() const
		{
			return m_clockwise;
		}
		/**
		* @brief Sets the profile orientation flag.
		* @param f_clockwise True for clockwise, false for counter-clockwise.
		*/
		inline void SetPrfCW(const bool f_clockwise)
		{
			m_clockwise = f_clockwise;
		}

		/**
		* @brief Determines whether the XY projection of a point lies inside the
		*        profile, outside it, or on it.
		*
		* Uses the winding number algorithm: for each edge crossing the horizontal
		* line through the point, an upward crossing with the point on its left
		* adds +1 and a downward crossing with the point on its right adds -1. The
		* point is outside only if the resulting winding number is 0; its sign
		* gives the orientation of the profile around the point.
		*
		* A bounding box test is performed first: points outside the profile's
		* bounding box (enlarged by f_threshold) are reported as outside without
		* testing the edges.
		*
		* Only the X and Y coordinates are used; Z is ignored.
		*
		* @param[in] f_query_point Point to test (only X and Y are used).
		* @param[in] f_threshold   Tolerance used for the bounding box test and,
		*                          through cgeom_algs::IsLeft(), to decide whether
		*                          the point lies on an edge.
		*
		* @return
		*   -  1 if the point is inside and the profile is clockwise around it;
		*   -  2 if the point is inside and the profile is counterclockwise
		*        around it;
		*   -  0 if the point lies on the profile (within f_threshold);
		*   - -1 if the point is outside, or the profile has fewer than 3 points.
		*
		* @warning The profile must be stored closed (last point equal to the
		*          first): the edges tested are (p[i], p[i+1]) for i = 0 .. n-2,
		*          so for an open profile the closing edge is missing.
		*
		* @note Adapted from wn_PnPoly() by Dan Sunday, "Inclusion of a Point in a
		*       Polygon", geomalgorithms.com
		*       (https://geomalgorithms.com/a03-_inclusion.html), released with the
		*       following notice:
		*       Copyright 2000 softSurfer, 2012 Dan Sunday.
		*       This code may be freely used and modified for any purpose providing
		*       that this copyright notice is included with it. SoftSurfer makes no
		*       warranty for this code, and cannot be held liable for any real or
		*       imagined damage resulting from its use. Users of this code must
		*       verify correctness for their application.
		*       Changes from the original: bounding box pre-check, on-edge
		*       detection with tolerance, orientation-dependent return values.
		*/
		inline int IsPointInside(const CPoint3& f_query_point, const double f_threshold = DOUBLE_PRECISION_EPSILON) const
		{
			int l_winding_number = 0;
			int n = (int)size();
			if ((n > 2) &&
				((f_query_point.m_x + f_threshold) > m_bounding_box.GetLLVertex().m_x) &&
				((f_query_point.m_x - f_threshold) < m_bounding_box.GetURVertex().m_x) &&
				((f_query_point.m_y + f_threshold) > m_bounding_box.GetLLVertex().m_y) &&
				((f_query_point.m_y - f_threshold) < m_bounding_box.GetURVertex().m_y))
			{
				for (size_t i = 0; i < (n - 1); ++i)
				{
					const auto& l_p0 = (*this)[i];
					const auto& l_p1 = (*this)[(i + 1)];
					double l_pt_is_left = cgeom_algs::IsLeft(l_p0, l_p1, f_query_point, f_threshold);
					if (l_pt_is_left == 0)
					{
						return 0;
					}

					if (l_p0.m_y <= f_query_point.m_y)
					{
						if (l_p1.m_y > f_query_point.m_y)
						{
							if (l_pt_is_left == 1)
							{
								++l_winding_number;
							}
						}
					}
					else
					{
						if (l_p1.m_y <= f_query_point.m_y)
						{
							if (l_pt_is_left == 2)
							{
								--l_winding_number;
							}
						}
					}
				}
			}

			if (l_winding_number < 0)
			{
				return 1;
			}
			if (l_winding_number > 0)
			{
				return 2;
			}

			return -1;
		}

		/**
		* @brief Reverses the order of points in the profile.
		*
		* The profile orientation flag is toggled accordingly.
		*/
		inline void ReverseProfile()
		{
			if (size() > 0)
			{
				std::reverse(begin(), end());
				m_clockwise = !m_clockwise;
			}
		}

		/**
		* @brief Closes the profile by ensuring the last point matches the first.
		*
		* If the profile is already closed within tolerance, it adjusts the last point.
		* Otherwise, it appends the first point to the end.
		*/
		inline void AddSeamPoint()
		{
			if ((back() - front()).Length() > COINC_THRESH)
			{
				push_back(front());
			}
			else
			{
				back().m_x = front().m_x;
				back().m_y = front().m_y;
				back().m_z = front().m_z;
			}
		}

		/**
		* @brief Retrieves the previous and next points in the profile for a given point index.
		*
		* Given a point index within the profile, this function determines the neighboring
		* points (previous and/or next) and assigns them to the provided output references.
		* The behavior depends on whether the profile is open or closed.
		*
		* ### Closed profile behavior
		* - The profile is treated as circular.
		* - The first and last points wrap around.
		* - Requires at least 3 points.
		* - Both previous and next points are always returned.
		*
		* ### Open profile behavior
		* - Boundary points have only one valid neighbor.
		* - For the first point, only the next point is returned.
		* - For the last point, only the previous point is returned.
		*
		* @param f_pt_id   Index of the reference point in the profile.
		* @param f_prev_pt Receives the previous point (if available).
		* @param f_next_pt Receives the next point (if available).
		*
		* @return int Status code:
		*         - 0 : Success, both previous and next points are valid.
		*         - 1 : Open profile, last point (only previous point is valid).
		*         - 2 : Open profile, first point (only next point is valid).
		*         - -1: Error (invalid index or insufficient number of points).
		*
		* @pre The profile must contain at least:
		*      - 2 points for an open profile
		*      - 3 points for a closed profile
		*/
		inline int GetPrevAndNextPt(const size_t f_pt_id, 
								    CPoint3& f_prev_pt, 
								    CPoint3& f_next_pt) const
		{
			if (f_pt_id > (size() - 1))
			{
				return -1;
			}
			if (size() < 2)
			{
				return -1;
			}
			if (CheckIfClosed())
			{
				if (size() < 3)
				{
					return -1;
				}
				if ((f_pt_id == (size() - 1)) ||
					(f_pt_id == 0))
				{
					f_next_pt = at(1);
					f_prev_pt = at(size() - 2);
				}
				else
				{
					f_next_pt = at(f_pt_id + 1);
					f_prev_pt = at(f_pt_id - 1);
				}
			}
			else
			{
				if (f_pt_id == (size() - 1))
				{
					f_prev_pt = at(size() - 2);
					return 1;
				}
				else if (f_pt_id == 0)
				{
					f_next_pt = at(1);
					return 2;
				}
			}

			return 0;
		}

		/**
		* @brief Extracts convex vertices from the profile.
		*
		* Iterates over the polygon vertices (excluding the duplicated last point)
		* and detects convex points using triplet orientation. Each convex vertex
		* and its corresponding index are stored in the output containers.
		*
		* @param[out] f_convex_points Extracted convex vertices.
		* @param[out] f_convex_points_ids Indices of the convex vertices.
		* 
		* @return True if at least one convex point was found, false otherwise.
		* 
		* @note Convexity is only defined for closed profiles.
		*	    External profiles are expected to be CCW, internal profiles CW.
		*/
		inline bool ExtractConvexPoints(std::vector<CPoint3>& f_convex_points,
									    std::vector<int>& f_convex_points_ids) const
		{
			const bool l_is_closed = CheckIfClosed();
			if (!l_is_closed)
			{
				return false;
			}
			int l_prf_size = int(size()) - 1;
			if (l_prf_size < 3)
			{
				return false;
			}
			bool l_convex_point_found = false;
			for (int i = 0; i < l_prf_size; ++i)
			{
				const int l_ia = (i - 1 + l_prf_size) % l_prf_size;
				const int l_ib = i;
				const int l_ic = (i + 1) % l_prf_size;

				const CPoint3& l_a = at(l_ia);
				const CPoint3& l_b = at(l_ib);
				const CPoint3& l_c = at(l_ic);
				const int l_res = cgeom_algs::TripletOrientation(l_a, l_b, l_c);
				if (l_res == 2)
				{
					l_convex_point_found = true;
					f_convex_points.emplace_back(l_b);
					f_convex_points_ids.emplace_back(l_ib);
				}
			}
			return l_convex_point_found;
		}

		/**
		* @brief Interpolates intermediate points along the profile so that the
		*        distance between consecutive points does not exceed a specified maximum.
		*
		* For each consecutive pair of points in the profile, additional evenly spaced
		* points are inserted if their Euclidean distance exceeds @p f_maxdist.
		* The original endpoints are always preserved.
		* The number of inserted segments is computed using a ceiling division to
		* guarantee that the resulting segment length is less than or equal to
		* @p f_maxdist.
		*
		* @param f_maxdist Maximum allowed distance between consecutive profile points.
		*					Must be strictly greater than zero.
		* @param f_max_pts_per_seg If specified, the maximum number of points per segment
		*						   is capped to this value, if left to -1 there is no limit
		*/
		inline void InterpolateProfilePoints(const double f_maxdist, const int f_max_pts_per_seg = -1)
		{
			if (size() < 2)
			{
				return;
			}
			size_t l_rough_numpoints = size_t(CalculateLength() / f_maxdist) + size();
			CPolyline l_interpolated_prf;
			l_interpolated_prf.reserve(l_rough_numpoints);
			l_interpolated_prf.push_back(front());
			for (auto l_pt_it = begin() + 1; l_pt_it != end(); l_pt_it++)
			{
				CPoint3 l_dir = *l_pt_it - *(l_pt_it - 1);
				double l_distance_btw_pts = l_dir.Length();
				if (l_distance_btw_pts <= COINC_THRESH_E2)
				{
					l_interpolated_prf.push_back(*l_pt_it);
					continue;
				}
				if (l_distance_btw_pts > f_maxdist)
				{
					int l_segments = static_cast<int>(std::ceil(l_distance_btw_pts / f_maxdist));
					if ((f_max_pts_per_seg != -1) && (f_max_pts_per_seg > 0))
					{
						l_segments = std::min(f_max_pts_per_seg, l_segments);
					}
					double l_step = l_distance_btw_pts / l_segments;
					l_dir = l_dir / l_distance_btw_pts;
					for (int i = 1; i < l_segments; ++i)
					{
						l_interpolated_prf.push_back(*(l_pt_it - 1) + l_dir * (l_step * i));
					}
				}
				l_interpolated_prf.push_back(*l_pt_it);
			}
			swap(l_interpolated_prf);

			CalculateAreaAndOrientation();
			CalculateBoundingBox();
		}

		/**
		* @brief Returns the lower-left vertex of the bounding box.
		*
		* Retrieves the minimum corner of the profile's axis-aligned bounding box.
		* This corresponds to the vertex containing the minimum X, Y,
		* and Z coordinates stored in the bounding box.
		*
		* @return math::CPoint3 The lower-left (minimum) vertex of the
		*         bounding box.
		* @note The bounding box must have been previously computed or updated
		*       (e.g., via CalculateBoundingBox()) before calling this method.
		*/
		inline CPoint3 GetLLBBoxVert() const
		{
			return m_bounding_box.GetLLVertex();
		}

		/**
		* @brief Returns the upper-right vertex of the bounding box.
		*
		* Retrieves the maximum corner of the profile's axis-aligned bounding box.
		* This typically corresponds to the vertex containing the maximum X, Y,
		* and Z coordinates stored in the bounding box.
		*
		* @return CPoint3 The upper-right (maximum) vertex of the
		*         bounding box.
		* @note The bounding box must have been previously computed or updated
		*       (e.g., via CalculateBoundingBox()) before calling this method.
		*/
		CPoint3 GetURBBoxVert() const
		{
			return m_bounding_box.GetURVertex();
		}

		/**
		* @brief Removes coincident (or near-coincident) consecutive points from the 
		*		 profile.
		*
		* Iterates through the current profile and removes points that are closer than
		* the specified coincidence threshold to the previously accepted point.
		*
		* @param f_coincidence_threshold The maximum allowed distance between two
		*        consecutive points for them to be considered coincident.
		*        Must be non-negative.
		*
		* @return true if any points were removed or modified; false otherwise.
		*/
		inline bool RemoveCoincidentPoints(const double f_coincidence_threshold = LOOSE_THRESH)
		{
			bool l_res = false;
			double l_coincidence_threshold_sq = f_coincidence_threshold * f_coincidence_threshold;

			CPolyline l_modified_profile;
			l_modified_profile.reserve(size());

			for (int l_ipt = 0; l_ipt < size(); l_ipt++)
			{
				if (l_ipt > 0)
				{
					double l_distance = DistanceSq(at(l_ipt), l_modified_profile.back());
					if (l_distance > l_coincidence_threshold_sq)
					{
						l_modified_profile.emplace_back(at(l_ipt));
					}
					else
					{
						if (l_modified_profile.size() > 1)
						{
							l_modified_profile.pop_back();
							l_modified_profile.emplace_back(at(l_ipt));
						}
					}
				}
				else
				{
					l_modified_profile.emplace_back(at(l_ipt));
				}
			}

			if (l_modified_profile.size() != size())
			{
				l_res = true;
			}
			swap(l_modified_profile);

			return l_res;
		}

		/**
		* @brief Shifts the seam (closure point) of a closed toolpath profile.
		*
		* This function changes the starting/ending point of a closed contour by
		* rotating the internal point sequence so that the specified iterator
		* becomes the new first element. The profile remains closed by re-appending
		* the first point at the end after rotation. If the contour is found to be 
		* open, no change is applied
		*
		* @param f_new_closure_point Iterator to the point that should become
		*        the new seam (first/last duplicated point).
		*/
		void ShiftSeamPoint(const CPolyline::iterator& f_new_closure_point)
		{
			if (CheckIfClosed())
			{
				if ((f_new_closure_point != begin()) && (f_new_closure_point != end() - 1))
				{
					erase(end() - 1);
					std::rotate(begin(), f_new_closure_point, end());
					push_back(front());
				}
			}
		}

		/**
		* @brief Performs 2D chordal deviation filtering on the toolpath profile.
		*
		* This method reduces the number of points in the profile by removing
		* intermediate points whose perpendicular distance to the chord
		* (line segment between anchor and current point) is below the specified 
		* chordal threshold.
		*
		* The algorithm:
		* - Preserves the first point.
		* - Iteratively checks intermediate points between an anchor and candidate point.
		* - If any intermediate point exceeds the chordal threshold, the previous point
		*   is preserved and becomes the new anchor.
		* - Ensures correct handling of open and closed profiles.
		* - Maintains contour closure and orientation consistency.
		*
		* Special handling:
		* - For closed contours, the duplicated last point is ignored during filtering.
		* - Seam adjustment is performed if necessary to prevent degeneration.
		* - The contour is re-closed after filtering.
		*
		* @param f_chordal_threshold Maximum allowed chordal deviation. Points whose 
		*							 perpendicular distance to the chord is less than this
		*							 threshold will be removed.
		* @return int Number of eliminated points.
		*
		* @note Profiles with fewer than 5 points are not filtered.
		*/
		inline int ChordalFilter2D(const double f_chordal_threshold)
		{
			const int l_min_points = 5;
			int l_eliminated_points = 0;

			if (size() < l_min_points)
			{
				return 0;
			}

			const bool l_is_closed = CheckIfClosed();

			CPolyline::iterator l_begin = begin();
			CPolyline::iterator l_end = end();
			if (l_is_closed)
			{
				l_end = end() - 1;
			}

			if (std::distance(l_begin, l_end) < 2)
			{
				return 0;
			}

			CPolyline l_filtered_prf;
			l_filtered_prf.reserve(size());

			l_filtered_prf.emplace_back(*l_begin);

			auto l_anchor_it = l_begin;
			CPoint3 l_last_kept = *l_begin;

			for (auto l_curr_it = l_begin + 1; l_curr_it != l_end; ++l_curr_it)
			{
				bool l_keep_previous = false;

				for (auto l_mid_it = l_anchor_it + 1; l_mid_it != l_curr_it; ++l_mid_it)
				{
					double l_dist = cgeom_algs::PointSegmentDistance(*l_mid_it, *l_anchor_it, *l_curr_it);

					if (l_dist >= f_chordal_threshold)
					{
						l_keep_previous = true;
						break;
					}
				}

				if (l_keep_previous)
				{
					auto l_prev = l_curr_it - 1;

					if (l_prev != l_anchor_it)
					{
						l_filtered_prf.emplace_back(*l_prev);
						l_last_kept = *l_prev;
						l_anchor_it = l_prev;
					}
				}
			}

			auto l_last_input = l_end - 1;
			if (!l_is_closed)
			{
				if (l_filtered_prf.empty() || Distance(*l_last_input, l_filtered_prf.back()) > (COINC_THRESH * COINC_THRESH))
				{
					l_filtered_prf.emplace_back(*l_last_input);
				}
				else
				{
					l_filtered_prf.back() = *l_last_input;
				}
			}
			else
			{
				if (l_filtered_prf.size() > 3)
				{
					const CPoint3& l_second_pt = l_filtered_prf[1];
					const CPoint3& l_penultimate_pt = l_filtered_prf[l_filtered_prf.size() - 1];

					double l_dist = cgeom_algs::PointSegmentDistance(l_filtered_prf.front(), l_second_pt, l_penultimate_pt);

					if (l_dist < f_chordal_threshold)
					{
						l_filtered_prf.ShiftSeamPoint(l_filtered_prf.begin() + 1);
						l_filtered_prf.erase(l_filtered_prf.end() - 1);
					}
				}

				if (!l_filtered_prf.empty())
				{
					l_filtered_prf.emplace_back(l_filtered_prf.front());
				}
			}

			l_eliminated_points = static_cast<int>(size() - l_filtered_prf.size());

			swap(l_filtered_prf);

			CalculateAreaAndOrientation();
			CalculateBoundingBox();

			return l_eliminated_points;
		}

		/**
		* @brief Builds an equispaced polyline from the current toolpath profile.
		*
		* Generates a new polyline composed of approximately equally spaced points
		* along the original profile length.
		* All generated points are assigned the Z value of the first point
		* (i.e., the resulting polyline is planar at constant Z).
		*
		* After rebuilding the polyline, the function:
		*  - Replaces the current profile with the new equispaced one
		*  - Recomputes area and orientation
		*  - Recomputes the bounding box
		*
		* @param f_number_points Desired approximate number of points in the
		*        resulting equispaced polyline. Must be greater than zero.
		* @return true Always returns true. If the profile contains fewer than
		*         two points, no modification is performed.
		*
		* @note
		* - Requires at least two points in the profile to operate.
		* - Uses linear interpolation along segments.
		* - Assumes a planar profile (constant Z level).
		* - The actual number of generated points may differ from
		*   f_number_points due to tolerance handling.
		* 
		* @note
		* To get best result interpolate points using @ref InterpolateProfilePoints with a 
		* distace equal to a fraction of f_distance before using this function
		*/
		inline void EquispaceProfilePoints(const int f_number_points)
		{
			if (size() < 2 || f_number_points <= 0)
				return;

			const bool l_is_closed = CheckIfClosed();
			const double l_prf_tot_len = CalculateLength();

			if (l_prf_tot_len <= COINC_THRESH)
				return;

			const double l_step = l_prf_tot_len / static_cast<double>(f_number_points);
			const double l_z_level = front().m_z;

			CPolyline l_result_prf;
			l_result_prf.reserve(f_number_points + 1);

			l_result_prf.emplace_back(front());

			double l_accumulated_len = 0.0;
			double l_next_sample_dist = l_step;

			CPoint3 l_prev_pt = front();

			for (auto l_pt_it = begin() + 1; l_pt_it != end(); ++l_pt_it)
			{
				CPoint3 l_curr_pt = *l_pt_it;
				CPoint3 l_seg = l_curr_pt - l_prev_pt;
				double l_seg_len = l_seg.Length();

				if (l_seg_len <= COINC_THRESH)
				{
					l_prev_pt = l_curr_pt;
					continue;
				}

				l_seg.UnitizeInPlace();
				while (l_accumulated_len + l_seg_len >= l_next_sample_dist)
				{
					double l_remaining_len = l_next_sample_dist - l_accumulated_len;

					CPoint3 l_interp_pt;
					l_interp_pt.m_x = l_prev_pt.m_x + l_seg.m_x * l_remaining_len;
					l_interp_pt.m_y = l_prev_pt.m_y + l_seg.m_y * l_remaining_len;
					l_interp_pt.m_z = l_z_level;

					l_result_prf.emplace_back(l_interp_pt);

					l_prev_pt = l_interp_pt;
					l_seg_len -= l_remaining_len;
					l_accumulated_len = l_next_sample_dist;
					l_next_sample_dist += l_step;
				}

				l_accumulated_len += l_seg_len;
				l_prev_pt = l_curr_pt;
			}

			if (!l_is_closed)
			{
				const CPoint3& lastInput = back();
				if (Distance(l_result_prf.back(), lastInput) > COINC_THRESH)
				{
					l_result_prf.emplace_back(lastInput);
				}
			}
			else
			{
				l_result_prf.AddSeamPoint();
			}

			swap(l_result_prf);

			if (l_is_closed)
			{
				CalculateAreaAndOrientation();
				if (GetArea() < COINC_THRESH)
				{
					clear();
					return;
				}
			}

			CalculateBoundingBox();
		}

		/**
		* @brief Builds an equispaced polyline using a target spacing distance.
		*
		* @param f_distance Desired approximate distance between consecutive
		*        points in the resulting polyline. Must be strictly positive.
		*
		* @note
		* - The final spacing may slightly differ from f_distance due to rounding.
		* - Internally calls BuildEquispacedPolyline(int).
		* - Ensures at least two points are generated.
		* - Recomputes area, orientation, and bounding box.
		* 
		* @note
		* To get best result interpolate points using @ref InterpolateProfilePoints with a 
		* distace equal to a fraction of f_distance before using this function
		*/
		inline void EquispaceProfilePoints(const double f_distance)
		{
			double l_prf_length = CalculateLength();
			int l_number_points = (int)((l_prf_length / f_distance) + 0.5);
			l_number_points = std::max(l_number_points, 2);

			return EquispaceProfilePoints(l_number_points);
		}

		/**
		* @brief Smooths the current curve using the Chaikin corner-cutting algorithm.
		*
		* Applies one or more iterations of the Chaikin subdivision scheme to
		* generate a smoother version of the current polyline. For open curves the 
		* first and last original points are preserved. 
		* For closed curves:
		*  - The curve is treated cyclically.
		*  - The first point is re-appended at the end after subdivision
		*    to maintain closure.
		*
		* The algorithm is applied recursively for the specified number of
		* iterations. Each iteration increases the number of points and
		* produces a smoother curve.
		*
		* @param f_cutting_ratio Proportion used to cut each edge.
		*						 Must be in the range (0.0, 0.5).
		*						 Typical value: 0.25.
		* @param f_iterations Number of smoothing iterations to perform.
		*					  If 0, the function returns immediately.
		*					  Usually 2–3 iterations provide good smoothing.
		* @return true if smoothing succeeds.
		*         false if a subdivision step fails (e.g., SingleChaikinCut fails).
		*
		* @note
		* - The function modifies the current profile in place.
		* - After smoothing, area/orientation and bounding box are recomputed.
		* - Point count grows approximately by a factor of 2 per iteration.
		* - Excessive iterations may significantly increase memory usage.
		*/
		bool ChaikinSmoothing(const double f_cutting_ratio, int f_iterations)
		{
			if (f_iterations <= 0 || size() <= 2)
			{
				CalculateAreaAndOrientation();
				CalculateBoundingBox();
				return true;
			}

			const bool l_is_closed = CheckIfClosed();

			CPolyline l_smoothed_curve;
			int l_num_points = int(size());
			int l_num_corners = l_num_points - 1;

			for (int l_icorner = 0; l_icorner < l_num_corners; l_icorner++)
			{
				CPoint3& l_a = at(l_icorner);
				CPoint3& l_b = at((l_icorner + 1) % l_num_points);

				CPoint3 l_first_cut;
				CPoint3 l_second_cut;

				if (!SingleChaikinCut(l_a, l_b, f_cutting_ratio,
					l_first_cut, l_second_cut))
				{
					return false;
				}

				if (!l_is_closed && l_icorner == 0)
				{
					l_smoothed_curve.push_back(l_a);
					l_smoothed_curve.push_back(l_second_cut);
				}
				else if (!l_is_closed && l_icorner == l_num_corners - 1)
				{
					l_smoothed_curve.push_back(l_first_cut);
					l_smoothed_curve.push_back(l_b);
				}
				else
				{
					l_smoothed_curve.push_back(l_first_cut);
					l_smoothed_curve.push_back(l_second_cut);
				}
			}

			swap(l_smoothed_curve);

			if (l_is_closed)
			{
				push_back(front());
			}

			return ChaikinSmoothing(f_cutting_ratio, f_iterations - 1);
		}

		/**
		* @brief Smooths the profile in-place using Laplacian smoothing.
		*
		* Each interior point is iteratively blended toward the average of its two neighbors.
		* Behavior depends on whether the profile is open or closed (as determined by CheckIfClosed()):
		*
		* - **Open profile:**  The first and last points are pinned and never modified.
		* - **Closed profile:** All points are smoothed with wraparound neighbors, and coincidence
		*                       of the first and last points is re-enforced after each iteration.
		*
		* The update formula applied at each interior point per iteration:
		* @code
		*   p[i] = (1 - alpha) * p[i] + alpha * 0.5 * (p[i-1] + p[i+1])
		* @endcode
		*
		* @param f_iterations Number of smoothing passes. Higher values produce smoother results.
		* @param f_alpha      Blending factor in [0, 1]. 0 = no change, 1 = full Laplacian step.
		*
		* @note Has no effect if the profile contains fewer than 3 points.
		* @note For closed profiles, index n-2 is used as the wraparound predecessor of index 0,
		*       since the point at n-1 is coincident with the point at 0.
		*/
		inline void LaplacianSmoothFixedEnds(const int f_iterations = 10, const double f_alpha = 0.5)
		{
			const size_t n = size();
			if (n < 3)
			{
				return;
			}

			const bool l_is_closed = CheckIfClosed();
			CPolyline l_tmp_prf;
			l_tmp_prf.resize(n);
			for (int it = 0; it < f_iterations; ++it)
			{
				if (l_is_closed)
				{
					for (size_t i = 0; i < n; ++i)
					{
						const size_t l_prev = (i == 0) ? n - 2 : i - 1;
						const size_t l_next = (i == n - 1) ? 1 : i + 1;
						const CPoint3 l_neighbor_avg = 0.5 * (at(l_prev) + at(l_next));
						l_tmp_prf[i] = (1.0 - f_alpha) * at(i) + f_alpha * l_neighbor_avg;
					}
					l_tmp_prf.back() = l_tmp_prf.front();
				}
				else
				{
					l_tmp_prf.front() = front();
					l_tmp_prf.back() = back();
					for (size_t i = 1; i < n - 1; ++i)
					{
						const CPoint3 l_neighbor_avg = 0.5 * (at(i - 1) + at(i + 1));
						l_tmp_prf[i] = (1.0 - f_alpha) * at(i) + f_alpha * l_neighbor_avg;
					}
				}
				swap(l_tmp_prf);
			}
		}

		/**
		* @brief Computes the unit tangent vector of the polyline at a given point index.
		*
		* The tangent is computed using finite differences, consistent with CalculateTangents():
		* - Open polyline: forward difference at the first point, backward difference
		*   at the last point, central difference elsewhere.
		* - Closed polyline: central difference with wrap-around. If the last point
		*   duplicates the first, it is ignored and gets the same tangent as the first point.
		*
		* The result is normalized to unit length. If the computed tangent has zero length
		* (e.g., coincident points), the zero vector is returned.
		*
		* @param f_pt_index Index of the point at which to compute the tangent.
		* @return Unit tangent vector at a_index, or the zero vector if the polyline has
		*         fewer than two points or a_index is out of range.
		*/
		inline CPoint3 CalculateTangentAt(const size_t f_pt_index) const
		{
			const size_t l_num_pts = size();
			if (l_num_pts < 2 || f_pt_index >= l_num_pts)
			{
				return CPoint3();
			}

			const bool l_is_closed = CheckIfClosed();
			const size_t l_effective_count = (l_is_closed && Distance(at(0), at(l_num_pts - 1)) == 0.0) ? l_num_pts - 1 : l_num_pts;

			const size_t l_idx = (f_pt_index >= l_effective_count) ? 0 : f_pt_index;

			CPoint3 l_tangent;
			if (l_is_closed)
			{
				const size_t l_prev = (l_idx == 0) ? l_effective_count - 1 : l_idx - 1;
				const size_t l_next = (l_idx + 1) % l_effective_count;
				l_tangent = at(l_next) - at(l_prev);
			}
			else
			{
				if (l_idx == 0)
				{
					l_tangent = at(1) - at(0);
				}
				else if (l_idx == l_effective_count - 1)
				{
					l_tangent = at(l_idx) - at(l_idx - 1);
				}
				else
				{
					l_tangent = at(l_idx + 1) - at(l_idx - 1);
				}
			}

			if (l_tangent.Length() != 0.0)
			{
				l_tangent.UnitizeInPlace();
			}
			return l_tangent;
		}

		/**
		* @brief Check if the profile is contained in the profile f_prf (it is assumed that the 2 profiles
		*		  do not intersect, meaning that those must not have a segment-to-segment intersection, but 
		*		  it is allowed that the two profiles touch each other in one point) 
		*
		* @param f_threshold				Threshold to determine whether a point lies exactly on a profile 
		*									or not
		* @param f_ignore_border_points	if true in the case of points that are less than the threshold 
		*									f_threshold far from the other prf (i.e. that are exactly on the profile),
		*									it continues to scroll the points of this until it finds one at a 
		*								    distance greater than f_threshold and defines the inclusion status based
		*									on this point. In case f_ignore_border_points = true, if all the points of
		*									the profile are not valid for inclusion determination (i.e. are exactly on 
		*									the profile f_prf), then the function returns -1 to signal inclusion search 
		*									failure. 
		* @return	0 in case the profile is not included, 1 in case it is included, -1 in case inclusion could not be
		*			determined (error)
		* 
		* @note Call CalculateBoundingBox first
		*/
		inline int IsInsideOtherPrf(const CPolyline& f_prf,
									double f_threshold = COINC_THRESH,
									const bool f_ignore_border_points = true) const
		{
			if ((size() == 0) || (f_prf.size() == 0))
			{
				return 0;
			}
			if ((m_bounding_box.GetURVertex().m_x < f_prf.GetLLBBoxVert().m_x) ||
				(f_prf.GetURBBoxVert().m_x < m_bounding_box.GetLLVertex().m_x) ||
				(m_bounding_box.GetURVertex().m_y < f_prf.GetLLBBoxVert().m_y) ||
				(f_prf.GetURBBoxVert().m_y < m_bounding_box.GetLLVertex().m_y))
			{
				return 0;
			}

			if (f_ignore_border_points)
			{
				for (auto l_pt_it = begin(); l_pt_it != end(); l_pt_it++)
				{
					int l_result = f_prf.IsPointInside(*l_pt_it, f_threshold);
					if (l_result != 0)
					{
						if (l_result > 0)
						{
							return 1;
						}
						else
						{
							return 0;
						}
					}

					if (l_pt_it != begin())
					{
						CPoint3 l_testpoint;
						l_testpoint = (*l_pt_it + *(l_pt_it - 1)) / 2.0;
						l_result = f_prf.IsPointInside(l_testpoint, f_threshold);
						if (l_result != 0)
						{
							if (l_result > 0)
							{
								return 1;
							}
							else
							{
								return 0;
							}
						}
					}
				}
			}
			else
			{
				if (f_prf.IsPointInside(front(), f_threshold) > 0)
				{
					return 1;
				}
				else
				{
					return 0;
				}
			}

			return -1;
		}
		/**
		* @brief Checks whether this toolpath profile intersects with another profile.
		*
		* This function determines if the current profile and the provided profile f_prf
		* intersect or touch each other. The intersection test considers bounding boxes,
		* segment-to-segment proximity using a quadtree for acceleration, and geometric
		* relations between segments.
		*
		* The method also optionally ignores intersections that occur only at single points
		* (segment endpoints) if 'f_ignore_single_points' is set to true.
		*
		* @param f_prf					The other toolpath profile to test against.
		* @param f_ignore_single_points If true, touching at a single point is not considered
		*                               an intersection.
		* @return true	If the profiles intersect or overlap, false If the profiles do not intersect.
		*/
		inline bool IsIntersectingWithPrf(const CPolyline& f_prf,
										  const bool f_ignore_single_points) const
		{
			if (!(((m_bounding_box.GetURVertex().m_x + LOOSE_THRESH) > f_prf.GetLLBBoxVert().m_x) &&
				((f_prf.GetURBBoxVert().m_x + LOOSE_THRESH) > m_bounding_box.GetLLVertex().m_x) &&
				((m_bounding_box.GetURVertex().m_y + LOOSE_THRESH) > f_prf.GetLLBBoxVert().m_y) &&
				((f_prf.GetURBBoxVert().m_y + LOOSE_THRESH) > m_bounding_box.GetLLVertex().m_y)))
			{

				return false;
			}

			std::vector<CSegment2D> l_profile_elements_list;
			l_profile_elements_list.reserve(size());

			int l_segment_id = 0;
			for (auto l_pt_it = begin() + 1; l_pt_it != end(); l_pt_it++)
			{
				CPoint2 l_start = *(l_pt_it - 1);
				CPoint2 l_end = *l_pt_it;

				l_profile_elements_list.emplace_back(CSegment2D(l_start, l_end));
				l_profile_elements_list.back().SetID(l_segment_id);

				l_segment_id++;
			}

			std::vector<IGeometricalObject2D*> l_profile_elements_list_ptrs;
			l_profile_elements_list_ptrs.reserve(l_profile_elements_list.size());
			for (auto l_elem_it = l_profile_elements_list.begin(); l_elem_it != l_profile_elements_list.end(); l_elem_it++)
			{
				l_profile_elements_list_ptrs.push_back(&(*l_elem_it));
			}

			const int l_max_depth = 9;
			const int l_max_elems_per_node = 10;
			CQuadtree l_quadtree(CAARectangle(m_bounding_box.GetLLVertex(), m_bounding_box.GetURVertex()),
								 l_max_depth,
								 l_max_elems_per_node);
			l_quadtree.Build(l_profile_elements_list_ptrs);
			std::vector<IGeometricalObject2D*> l_query_pool;
			bool l_probable_collision = false;
			std::unordered_set<int> l_colliding_segments;
			int l_prf_2_segm_id = 0;
			for (auto l_pt_sec_it = f_prf.begin() + 1; l_pt_sec_it != f_prf.end(); l_pt_sec_it++)
			{
				CPoint2 l_seg_start = *(l_pt_sec_it - 1);
				CPoint2 l_seg_end = *l_pt_sec_it;

				l_query_pool.clear();
				l_quadtree.QueryRegion(CAARectangle(l_seg_start, l_seg_end), l_query_pool);

				for (const auto l_object : l_query_pool)
				{
					auto* l_seg = dynamic_cast<CSegment2D*>(l_object);
					if (!l_seg)
					{
						continue;
					}
					CPoint3 l_pt_1;
					CPoint3 l_pt_2;
					double l_dist = 0.0;
					double l_param_on_seg_1 = 0.0;
					double l_param_on_seg_2 = 0.0;
					cgeom_algs::ESegmentRelation l_seg_rel = cgeom_algs::SegmentSegmentDistance3D(CPoint3(l_seg_start.m_x, l_seg_start.m_y, 0.0),
																								  CPoint3(l_seg_end.m_x, l_seg_end.m_y, 0.0),
																								  CPoint3(l_seg->GetStartPt().m_x, l_seg->GetStartPt().m_y, 0.0),
																								  CPoint3(l_seg->GetEndPt().m_x, l_seg->GetEndPt().m_y, 0.0),
																								  l_dist,
																								  l_param_on_seg_1,
																								  l_param_on_seg_2,
																								  l_pt_1,
																								  l_pt_2);
					switch (l_seg_rel)
					{
					case (cgeom_algs::ESegmentRelation::Intersecting):
						return true;
					case (cgeom_algs::ESegmentRelation::CollinearlOverlapping):
						return true;
					case (cgeom_algs::ESegmentRelation::CollinearTouchingExtremes):
					{
						if (!f_ignore_single_points)
						{
							return true;
						}
						break;
					}
					case(cgeom_algs::ESegmentRelation::NonParallelTouchPoint):
					{
						if (!f_ignore_single_points)
						{
							return true;
						}

						l_colliding_segments.insert(l_prf_2_segm_id);
						l_probable_collision = true;
						break;
					}
					}
				}

				l_prf_2_segm_id += 1;
			}

			if (l_probable_collision)
			{
				int l_inclusion_test_counter = 0;
				int l_colliding_segments_number = (int)(l_colliding_segments.size());
				for (const auto& l_seg_id : l_colliding_segments)
				{
					CPoint2 l_midpoint = (f_prf[l_seg_id] + f_prf[l_seg_id + 1]) / 2.0;
					int l_ret = IsPointInside(CPoint3(l_midpoint.m_x, l_midpoint.m_y, 0.0));
					if (l_ret > 0)
					{
						l_inclusion_test_counter += 1;
					}
				}

				if ((l_inclusion_test_counter != 0) && (l_inclusion_test_counter != l_colliding_segments_number))
				{
					return true;
				}
			}

			return false;
		}

		/**
		* @brief Returns whether the toolpath profile is valid.
		* 
		* @return True if the profile is valid, false otherwise.
		*/
		inline bool GetValidPrf() const
		{
			return m_valid;
		}

		/**
		* @brief Sets the validity flag of the toolpath profile.
		*
		* @param f_valid_prf True to mark the profile as valid, false otherwise.
		*/
		inline void SetValidPrf(const bool f_valid_prf)
		{
			m_valid = f_valid_prf;
		}

		/**
		* @brief Finds the closest point on the profile to a given point.
		*
		* Iterates over all segments of the polyline and determines the closest point
		* to @p f_point. The closest point may lie either:
		* - on a segment (orthogonal projection within segment bounds), or
		* - on one of the segment endpoints.
		*
		* The computation is performed fully in 3D space.
		*
		* The result is stored in @p f_closest_point, including metadata:
		* - Num2: index of the segment containing the closest point
		* - Num3: index of the vertex if the closest point is an endpoint,
		*         or -1 if it is a projected point
		* - DNum1: Euclidean distance to @p f_point
		*
		* In case of equal distances, the first encountered point is retained.
		*
		* @param[in]  f_point           Query point in 3D space.
		* @param[out] f_closest_point   Closest point on the profile with metadata.
		*
		* @return true  if a closest point was found
		* @return false if the profile contains fewer than two points
		*/
		inline bool FindClosestPoint(const CPoint3& f_point, CPoint3& f_closest_point) const
		{
			if (size() < 2)
			{
				return false;
			}
			double l_min_dist = LARGE_DOUBLE_CONST;
			for (int i = 0; i < (size() - 1); ++i)
			{
				double l_dist = 0.0;
				CPoint3 l_proj_pt;
				bool l_proj_inside = cgeom_algs::PointProjectionOnSegment(f_point, at(i), at(i + 1), l_proj_pt);
				if (l_proj_inside)
				{
					l_dist = Distance(l_proj_pt, f_point);
					if (l_dist < l_min_dist)
					{
						l_min_dist = l_dist;
						f_closest_point = l_proj_pt;
						f_closest_point.SetIntVal2(i);
						f_closest_point.SetIntVal3(-1);
						f_closest_point.SetDoubleVal1(l_dist);
					}
				}
				else
				{
					double l_distA = Distance(f_point, at(i));
					if (l_distA < l_min_dist)
					{
						l_min_dist = l_distA;
						f_closest_point = at(i);
						f_closest_point.SetIntVal2(i);
						f_closest_point.SetIntVal3(i);
						f_closest_point.SetDoubleVal1(l_distA);
					}

					double l_distB = Distance(f_point, at(i + 1));
					if (l_distB < l_min_dist)
					{
						l_min_dist = l_distB;
						f_closest_point = at(i + 1);
						f_closest_point.SetIntVal2(i);
						f_closest_point.SetIntVal3(i + 1);
						f_closest_point.SetDoubleVal1(l_distB);
					}
				}
			}

			return true;
		}

		/**
		* @brief Removes geometric inversions from the toolpath profile.
		*
		* This method processes the profile points in a sliding window of four points
		* to detect local inversions (e.g., zig-zag or self-intersecting segments)
		* using the function `cgeom_algs::IsInversion`. When an inversion is detected,
		* the penultimate point in the window is removed, effectively "smoothing"
		* the profile. The window is then collapsed to continue processing.
		*
		* After processing all points:
		* - If the profile is not closed, the first point is appended to close it.
		* - The processed profile replaces the original.
		* - Area, orientation, and bounding box are recalculated.
		*
		* @return The number of points removed from the profile.
		*/
		inline int EliminateInversions()
		{
			const int l_input_size = static_cast<int>(size());
			CPolyline l_processed_prf;
			l_processed_prf.reserve(size());

			std::deque<CPoint3> l_sliding_window;

			for (const auto& pt : *this)
			{
				l_sliding_window.push_back(pt);

				if (l_sliding_window.size() < 4)
				{
					l_processed_prf.emplace_back(pt);
					continue;
				}

				if (cgeom_algs::IsInversion(l_sliding_window[0], l_sliding_window[1], l_sliding_window[2], l_sliding_window[3]))
				{
					if (!l_processed_prf.empty())
						l_processed_prf.pop_back();

					l_sliding_window[2] = l_sliding_window[3];
					l_sliding_window.pop_back();
				}
				else
				{
					l_processed_prf.emplace_back(pt);
					l_sliding_window.pop_front();
				}
			}

			if (!l_processed_prf.empty() &&
				Distance(l_processed_prf.front(), l_processed_prf.back()) > 0.01)
			{
				l_processed_prf.emplace_back(l_processed_prf.front());
			}

			swap(l_processed_prf);
			CalculateAreaAndOrientation();
			CalculateBoundingBox();

			return (l_input_size - int(size()));
		}

		/**
		* @brief Removes spike/hair artifacts from a closed toolpath profile.
		*
		* This is the top-level orchestrator for the spike-removal pipeline. It runs
		* the full sequence: preprocessing, quadtree construction, spike detection,
		* valid/invalid portion splitting, stitching, filtering, and final profile
		* composition. Output profiles are guaranteed to have a positive area; those
		* that do not are discarded.
		*
		* @param f_output_prfs       List that receives the cleaned output profiles.
		* @param f_spike_distance    Distance threshold used to classify a point as a spike.
		* @param f_prf_modified      Set to  true if the profile was altered,  false otherwise.
		*/
		inline bool RemoveHairs(CPolylinesList& f_output_prfs,
								const double f_spike_distance,
								bool& f_prf_modified
								);

		/**
		* @brief Splits a profile into a given number of sub-portions of equal chordal length.
		*
		* Computes the total chordal length of the profile, then walks along it
		* accumulating length until each sub-portion threshold is reached.
		* Split points are interpolated on the segment where the threshold is crossed.
		*
		* @param f_num Number of sub-portions to split into.
		* @return List of profiles, each representing one sub-portion.
		*/
		inline CPolylinesList SplitByChordalLength(const int f_num) const;

		/**
		* @brief Draws the toolpath profile as a polyline using the provided graphics interface.
		*
		* @param f_color Color identifier used for drawing the polyline.
		* @param f_redraw If true, triggers a redraw operation after rendering.
		* @param f_bake If true, the geometry is baked (persistently stored) in the
		*        graphics system; otherwise it may be treated as transient.
		* @param f_graph_ptr Pointer to the graphics interface responsible for rendering.
		* 
		* @return The id of the represented object
		*
		* @note
		* - The function assumes f_graph_ptr is valid (non-null).
		* - Rendering thickness is currently hardcoded to 1
		* - A temporary vertex buffer is created for drawing.
		* - If f_redraw is true, ReDraw() is explicitly called after DrawPolyLine().
		*/
		inline int Draw(const int f_color, 
					    const bool f_redraw, 
					    const bool f_bake, 
					    Graph::IGraphInterface* f_graph_ptr) const
		{
			std::vector<CPoint3> l_drawable_prf;
			l_drawable_prf.reserve(size());
			for (auto l_tp_pt_it = begin(); l_tp_pt_it != end(); l_tp_pt_it++)
			{
				l_drawable_prf.emplace_back(*l_tp_pt_it);
			}

			const int l_id = f_graph_ptr->DrawPolyLine3(l_drawable_prf.data(), l_drawable_prf.size(), f_color, f_redraw, f_bake, 1);

			if (f_redraw)
			{
				f_graph_ptr->ReDraw(f_bake);
			}

			return l_id;
		}

	private:

		struct CPointSortCriterion
		{
			CPointSortCriterion(const CPoint3& f_pt)
			{
				m_pt = f_pt;
			}

			bool operator()(const CPoint3& lhs, const CPoint3& rhs) const
			{
				return Distance(m_pt, lhs) < Distance(m_pt, rhs);
			}

		private:
			CPoint3 m_pt;
		};

		/**
		* @brief Performs a single Chaikin corner cut between two points.
		*
		* The resulting points lie on segment AB and are used to smooth
		* the polyline during Chaikin subdivision.
		*
		* @param f_a First endpoint of the segment.
		* @param f_b Second endpoint of the segment.
		* @param f_ratio Cutting ratio. Must be in the range [0.0, 0.5].
		*				 Typical value: 0.25.
		* @param f_cut_point_1 First generated cut point (closer to f_a).
		* @param f_cut_point_2 Second generated cut point (closer to f_b).
		*
		* @return true if the cut is successfully computed.
		*         false if f_ratio > 0.5.
		*
		* @note
		* - Only X and Y coordinates are interpolated
		* - Z coordinate is not considered
		 */
		inline bool SingleChaikinCut(const CPoint3& f_a,
									 const CPoint3& f_b,
									 const double f_ratio,
									 CPoint3& f_cut_point_1,
									 CPoint3& f_cut_point_2)
		{
			if (f_ratio <= 0.0 || f_ratio > 0.5)
			{
				return false;
			}

			f_cut_point_1.m_x = LinearInterp(f_a.m_x, f_b.m_x, f_ratio);
			f_cut_point_1.m_y = LinearInterp(f_a.m_y, f_b.m_y, f_ratio);
			f_cut_point_2.m_x = LinearInterp(f_b.m_x, f_a.m_x, f_ratio);
			f_cut_point_2.m_y = LinearInterp(f_b.m_y, f_a.m_y, f_ratio);

			return true;
		}
		/**
		* @brief Performs linear interpolation between two scalar values.
		* 
		* @param f_a First value.
		* @param f_b Second value.
		* @param f_param Interpolation parameter.
		*				 - 0.0 returns f_a
		*				 - 1.0 returns f_b
		*				 - Values in between return a weighted average
		*				 Note: f_param is clamped between [0.0-1.0]
		*
		* @return Interpolated scalar value.
		*/
		inline double LinearInterp(double f_a, double f_b, double f_param)
		{
			double t = std::max(0.0, std::min(1.0, f_param));
			return (f_a * (1.0 - t)) + (f_b * t);
		}

		/**
		* @brief Builds a 2-D quadtree that indexes every segment of this profile.
		*
		* Iterates over consecutive point pairs to construct a @c CSegment2D vector,
		* then inserts pointers to those segments into a @c CQuadtree whose bounding
		* box matches the profile's own bounding box.
		*
		* @param[out] f_prf_segs  Receives the segment objects whose lifetimes must
		*                         outlive the returned quadtree (the tree stores raw pointers).
		* @return A fully built @c CQuadtree over all profile segments.
		*/
		inline CQuadtree BuildProfileQuadtree(std::vector<CSegment2D>& f_prf_segs) const
		{
			const int l_max_depth = 30;
			const int l_max_elems_per_node = 10;

			CQuadtree l_quadtree(CAARectangle(GetLLBBoxVert(), GetURBBoxVert()), l_max_depth, l_max_elems_per_node);

			f_prf_segs.reserve(size() - 1);
			int l_seg_id = 0;
			for (auto it = begin() + 1; it != end(); ++it, ++l_seg_id)
			{
				f_prf_segs.emplace_back(CPoint2((it - 1)->m_x, (it - 1)->m_y), CPoint2(it->m_x, it->m_y));
				f_prf_segs.back().SetID(l_seg_id);
			}

			std::vector<IGeometricalObject2D*> l_seg_ptrs;
			l_seg_ptrs.reserve(f_prf_segs.size());
			for (auto& l_seg : f_prf_segs)
			{
				l_seg_ptrs.emplace_back(&l_seg);
			}

			l_quadtree.Build(l_seg_ptrs);
			return l_quadtree;
		}

		/**
		* @brief Prepares the profile for spike removal by eliminating inversions,
		*        duplicate points, and repositioning the seam at a stable location.
		*
		* Runs three sequential clean-up passes:
		*  1. Inversion removal — calls @c EliminateInversions() to strip any
		*     wild direction-reversal artefacts from the point sequence.
		*  2. Coincident-point removal — calls @c RemoveCoincidentPoints() to
		*     collapse points that are closer together than @p f_spike_distance.
		*  3. Seam relocation  — walks the profile from the third point onward and
		*     moves the seam (via @c ShiftSeamPoint()) to the first point that is NOT
		*     the apex of a spike, ensuring that downstream processing does not start
		*     mid-spike.
		*
		* If the profile becomes empty after coincident-point removal the function
		* returns immediately without attempting seam relocation.
		*
		* @param f_threshold_dist  Distance threshold forwarded to coincident-point
		*                          removal and spike-apex detection.
		*/
		inline void PreprocessProfile(const double f_threshold_dist			
									  )
		{
			int l_removed_pts_wild_inversions = EliminateInversions();
			int l_point_in_prf = (int)size();
			bool l_ret_remove_duplicates = RemoveCoincidentPoints(f_threshold_dist);
			if (size() > 3)
			{
				for (auto l_pt_it = begin() + 2; l_pt_it != end(); l_pt_it++)
				{
					if (!cgeom_algs::IsPointSpikeApex(*(l_pt_it - 1), *l_pt_it, *(l_pt_it - 2), f_threshold_dist))
					{
						ShiftSeamPoint(l_pt_it);
						break;
					}
				}
			}
		}
		/**
		* @brief Detects spike points and records their projections onto the profile.
		*
		* Tags each profile point with a sequential ID (Num1). For every point, the
		* quadtree is queried for nearby segments. If the closest eligible segment lies
		* within @p f_spike_distance, the point is flagged as a spike (Num3 = -1) and
		* the foot-of-perpendicular is stored in the returned map, keyed by segment ID.
		* Multiple projections on the same segment are collected for later sorting.
		*
		* @param f_quadtree        Quadtree built over the profile segments.
		* @param f_spike_distance  Maximum perpendicular distance that qualifies a point as a spike.
		* @param f_zlevel          Z coordinate used when constructing 3-D debug points.
		* @return Map from segment ID to the list of projected toolpath points on that segment.
		*/
		inline std::map<int, CPolyline>FindSpikePoints(CQuadtree& f_quadtree,
													   const double f_spike_distance,
													   const double f_zlevel
													   )
		{
			int l_pt_id = 0;
			for (auto& l_pt : *this)
			{
				l_pt.SetIntVal1(l_pt_id++);
			}

			std::map<int, CPolyline> l_added_points_map;
			int l_id_added_point = (int)size() + 1;
			std::vector<IGeometricalObject2D*> l_query_pool;
			l_pt_id = 0;
			for (auto pt_it = begin(); pt_it != end() - 1; ++pt_it, ++l_pt_id)
			{
				f_quadtree.QueryRegion(MakeQueryBox(pt_it->m_x, pt_it->m_y, f_spike_distance), l_query_pool);

				int l_prev_seg_id = -1;
				if (pt_it == begin())
				{
					l_prev_seg_id = (int)size() - 2;
				}
				else
				{
					l_prev_seg_id = l_pt_id - 1;
				}
				const int l_next_seg_id = l_pt_id;
				CPoint2 l_current_pt(pt_it->m_x, pt_it->m_y);
				double l_best_dist = HUGE_VAL;
				bool l_point_found = false;
				auto l_spike_it = begin();
				CPoint2 l_new_point;
				int l_pt_id_spike = 0;
				int l_interp_seg_id = 0;

				for (const auto& l_object : l_query_pool)
				{
					auto* l_seg = dynamic_cast<CSegment2D*>(l_object);
					if (!l_seg)
					{
						continue;
					}	
					if (l_seg->GetID() == l_prev_seg_id || l_seg->GetID() == l_next_seg_id)
					{
						continue;
					}

					double l_dist = 0.0;
					CPoint2 l_proj;
					bool l_proj_inside = cgeom_algs::PointProjectionOnSegment(l_current_pt, l_seg->GetStartPt(), l_seg->GetEndPt(), l_proj, l_dist);

					if (l_proj_inside && (l_dist <= f_spike_distance) && ((l_dist - l_best_dist) < 1.0e-40))
					{
						l_best_dist = l_dist;
						l_point_found = true;
						l_spike_it = pt_it;
						l_new_point = l_proj;
						l_pt_id_spike = l_pt_id;
						l_interp_seg_id = l_seg->GetID();
					}
				}

				if (!l_point_found)
					continue;
				l_spike_it->SetIntVal3(-1);
				int l_id_new_point = l_id_added_point + 1;
				l_id_added_point = l_id_new_point;
				CPoint3 l_new_tp(CPoint3(l_new_point.m_x, l_new_point.m_y, f_zlevel));
				l_new_tp.SetIntVal2(l_pt_id_spike);
				l_new_tp.SetIntVal1(l_id_new_point);
				pt_it->SetIntVal2(l_id_new_point);

				auto [l_map_it, l_inserted] = l_added_points_map.try_emplace(l_interp_seg_id, CPolyline{});
				l_map_it->second.emplace_back(l_new_tp);
			}

			return l_added_points_map;
		}

		/**
		* @brief Sorts each projection list in the spike map along its host segment.
		*
		* For segments that received more than one projected point, orders those points
		* by proximity to the segment's start vertex using @c CPointSortCriterion.
		*
		* @param f_map  Map from segment ID to projected points; sorted in place.
		* @return @c true if at least one non-empty entry was found; @c false if the map
		*         contains no actionable projections (early-exit signal for the caller).
		*/
		inline bool SortSpikeMap(std::map<int, CPolyline>& f_map)
		{
			bool l_found = false;
			for (auto& [seg_id, pts] : f_map)
			{
				if (pts.empty())
				{
					continue;
				}
				l_found = true;
				if (pts.size() > 1)
				{
					std::sort(pts.begin(), pts.end(), CPointSortCriterion(at(seg_id)));
				}
			}
			return l_found;
		}

		/**
		* @brief Splits the profile into alternating valid and invalid (spike) portions.
		*
		* Walks the profile point-by-point. Points flagged as spikes (Num3 != 0) close
		* the current valid chunk and open an invalid one, and vice versa. Projected
		* points stored in @p f_added_points_map are injected at the appropriate segment
		* boundaries as chunk delimiters. The first and last point are handled so that a
		* profile whose seam falls inside a spike region is still split correctly.
		*
		* @param f_added_points_map  Map of projected interpolation points (from FindSpikePoints).
		* @param f_zlevel            Z coordinate (passed through for debug geometry).
		* @param f_valid_portions    Receives all portions that do NOT contain spike points.
		* @param f_invalid_portions  Receives all portions that are classified as spike regions.
		*/
		inline void SplitIntoPortions(const std::map<int, CPolyline>& f_added_points_map,
								      const double f_zlevel,
								      CPolylinesList& f_valid_portions,
								      CPolylinesList& f_invalid_portions		
									  ) const;

		/**
		* @brief Merges the first and last valid portions when they share a common endpoint.
		*
		* Because the input is a closed profile, the splitting step may cut across the
		* seam and produce a first and last portion that are geometrically contiguous.
		* When the start of the first portion coincides (within @c COINC_THRESH) with
		* the end of the last portion, this function appends the first portion's interior
		* points onto the last portion and removes the first portion from the list.
		*
		* @param f_portions   List of valid portions; modified in place.
		*/
		inline void StitchFirstAndLastPortions(CPolylinesList& f_portions
											   );

		/**
		* @brief Removes degenerate three-point spike portions from the valid list.
		*
		* A portion consisting of exactly three points where the middle point is the
		* apex of a spike (tested via @c IsPointSpikeApex) is considered a residual
		* artefact and is erased. All other portions are kept unchanged.
		*
		* @param f_portions        List of valid portions; degenerate entries are erased.
		* @param f_spike_distance  Distance threshold forwarded to @c IsPointSpikeApex.
		*/
		inline void EliminateSpikePortions(CPolylinesList& f_portions,
										   const double f_spike_distance
										   );

		static CAARectangle MakeQueryBox(double f_cx, double f_cy, double f_box_half_side)
		{
			return CAARectangle(CPoint2(f_cx - f_box_half_side - LOOSE_THRESH, f_cy - f_box_half_side - LOOSE_THRESH),
								CPoint2(f_cx + f_box_half_side + LOOSE_THRESH, f_cy + f_box_half_side + LOOSE_THRESH));
		}


		/**
		* @brief Evaluates invalid portions and promotes survivors into the valid list.
		*
		* An invalid portion is discarded when:
		*   - Its start and end points are closer than @p f_spike_distance (zero-length), or
		*   - The midpoint of the portion has a perpendicular projection onto a nearby profile
		*     segment that lies within @p f_spike_distance (the portion sits inside a spike channel).
		*
		* Portions that survive both tests are appended to @p f_valid_portions so that
		* they participate in the final profile composition.
		*
		* @param f_invalid_portions  Invalid portions; eliminated entries are erased in place.
		* @param f_valid_portions    Valid portions; survivors from the invalid list are appended.
		* @param f_quadtree          Quadtree over the original profile segments.
		* @param f_spike_distance    Distance threshold for both elimination tests.
		* @param f_zlevel            Z coordinate used for debug geometry.
		*/
		inline void FilterInvalidPortions(CPolylinesList& f_invalid_portions,
									      CPolylinesList& f_valid_portions,
									      CQuadtree& f_quadtree,
									      const double f_spike_distance,
									      const double f_zlevel
										  );

		/**
		* @brief Chains valid portions into one or more seamless closed profiles.
		*
		* Iterates over the valid portions and links them end-to-end by matching the
		* endpoint IDs of the current composing chain (Num1 / Num2) against the start
		* IDs of each remaining portion. This continues until no unvisited portions
		* remain, producing one composed profile per connected chain. Each portion is
		* marked invalid (SetValidPrf(false)) once it has been consumed.
		*
		* @param[in,out] f_valid_portions  Valid portions to chain; all entries are consumed.
		* @return A list of composed, seam-ready profiles ready for area computation.
		*/
		inline CPolylinesList ComposeSeamedPortions(CPolylinesList& f_valid_portions);

		double m_area;
		bool m_clockwise;
		CAABox m_bounding_box;
		bool m_valid;
	};
}

#include "CPolylinesList.h"

#endif
