
#ifndef H_CPOLYLINESLIST
#define H_CPOLYLINESLIST
#pragma once

#include <fstream>
#include <map>
#include <deque>

#include "CPolyline.h"

namespace ggeomlib
{
	/**
	* @class CPolylinesList
	* @brief A list of CPolyline objects
	*/
	class CPolylinesList : public std::list<CPolyline>
	{
	public:

		/**
		* @brief Gets the total number of points of all profiles in the list
		* 
		* @return Total number of points of profiles in the list
		*/
		inline size_t GetNumPoints() const
		{
			size_t l_tot_points = 0;
			for (auto l_prf_it = begin(); l_prf_it != end(); l_prf_it++)
			{
				l_tot_points += l_prf_it->size();
			}
			return l_tot_points;
		}

		/**
		* @brief Ensures consistent orientation of planar nested profiles: internal profiles
		*		 (holes) must be clockwise (CW), internal ones must be counter clockwise (CCW)
		*
		* This method analyzes the inclusion hierarchy among all contained profiles
		* and adjusts their orientation (clockwise / counter-clockwise) to ensure
		* geometric consistency.
		*
		* @note This method assumes that profiles do not intersect.
		*       It relies on IsInsideOtherPrf(), which returns 1 if a profile
		*       is strictly included in another, and 0 otherwise.
		*
		* @return true  if at least one profile orientation was modified.
		* @return false if all profiles were already consistently oriented.
		*/
		inline bool RearrangeOrientation()
		{
			bool l_profiles_rearranged = false;

			std::vector<int> l_inclusion_level(size(), 0);

			int l_iprf = 0;
			for (auto l_prf_it = begin(); l_prf_it != end(); l_prf_it++)
			{
				for (auto l_prf_it_in = begin(); l_prf_it_in != end(); l_prf_it_in++)
				{
					if (l_prf_it_in == l_prf_it)
					{
						continue;
					}
					int l_is_included = l_prf_it->IsInsideOtherPrf(*l_prf_it_in);
					if (l_is_included == 1)
					{
						l_inclusion_level[l_iprf]++;
					}
				}

				l_iprf++;
			}

			l_iprf = 0;
			for (auto l_prf_it = begin(); l_prf_it != end(); l_prf_it++)
			{
				bool l_dir_to_be_reversed = false;
				if (l_prf_it->IsPrfCW())
				{
					if (l_inclusion_level[l_iprf] % 2 == 0)
					{
						l_dir_to_be_reversed = true;
					}
				}
				else
				{
					if (l_inclusion_level[l_iprf] % 2 == 1)
					{
						l_dir_to_be_reversed = true;
					}
				}

				if (l_dir_to_be_reversed)
				{
					bool l_clockwise_original = l_prf_it->IsPrfCW();
					l_prf_it->ReverseProfile();
					l_prf_it->SetPrfCW(!l_clockwise_original);
					l_profiles_rearranged = true;
				}

				l_iprf += 1;
			}

			return l_profiles_rearranged;
		}

		/**
		* @brief Determines whether a point lies inside the toolpath profiles.
		*
		* This method evaluates the position of a point relative to all profiles
		* contained in the list. Each profile is queried using its IsPointInside()
		* method. The final result is computed using an odd–even rule:
		*
		* - If the point lies exactly on a profile boundary (within the given threshold),
		*   the function immediately returns 0.
		* - If the point is strictly inside an odd number of profiles, it is considered inside.
		* - If the point is inside an even number of profiles (including zero),
		*   it is considered outside. This also covers the case where the point lies
		*   inside a hole.
		*
		* @param f_p The 3D point to test.
		* @param f_threshold Distance tolerance used to determine whether the point
		*                    lies on a profile boundary.
		*
		* @return int
		*         - 1 if the point is inside the profiles.
		*         - 0 if the point is outside or lies on a profile boundary.
		*
		* @note The inside/outside determination follows the parity (odd–even) rule.
		*/
		inline int IsPointInside(const CPoint3& f_p, double f_threshold = COINC_THRESH) const
		{
			int l_result = 0;
			int l_inside_res = 0;
			for (auto l_prf_it = begin(); l_prf_it != end(); l_prf_it++)
			{
				l_result = l_prf_it->IsPointInside(f_p, f_threshold);
				if (l_result == 0)
				{
					return 0;
				}
				if (l_result == 1)
				{
					l_inside_res++;
				}
				if (l_result == 2)
				{
					l_inside_res--;
				}
			}

			if (l_inside_res % 2)
			{
				return 1;
			}

			return -1;
		}

		/**
		* @brief Computes the axis-aligned bounding box of the list of profiles
		* 
		* This function iterates over all elements in the list and expands
		* a 3D bounding box to include each element's bounding extents.
		*
		* @param f_ll_vert Reference to a vector that will hold the lower-left (minimum) corner 
		*				   of the bounding box.
		* @param f_ur_vert Reference to a vector that will hold the upper-right (maximum) corner
		*				   of the bounding box.
		*/
		inline void ComputeBoundingBoxOnLine(CPoint3& f_ll_vert, CPoint3& f_ur_vert)
		{
			if (!empty())
			{
				CPoint3 l_min;
				CPoint3 l_max;
				front().CalculateBoundingBoxOnLine(f_ll_vert, f_ur_vert);
				auto l_prf_it = begin();
				l_prf_it++;
				for (; l_prf_it != end(); l_prf_it++)
				{
					l_prf_it->CalculateBoundingBoxOnLine(l_min, l_max);
					f_ll_vert = math_algs::MinOf2Vectors(f_ll_vert, l_min);
					f_ur_vert = math_algs::MaxOf2Vectors(f_ur_vert, l_max);
				}
			}
		}
		/**
		* @brief Attempts to rejoin open toolpath profiles into closed loops where possible.
		*
		* This function iteratively merges open profiles in the current list into closed
		* profiles based on proximity of endpoints. Profiles whose endpoints are within
		* f_max_connecting_dist are candidates for merging. Mutual nearest-neighbor
		* merging is performed to ensure consistent joining.
		*
		* Key behaviors:
		* - Profiles with endpoints closer than f_max_connecting_dist can be merged.
		* - If f_not_self is false, a profile can self-close if its first and last
		*   points are close enough.
		* - Profiles that cannot be closed or merged are placed in f_open_profiles.
		* - f_error_on_open_prf controls whether the function returns false if any
		*   profile remains open.
		*
		* @param f_closed_prfs			List to store successfully closed (or merged) profiles.
		* @param f_open_profiles		List to store profiles that remain open after processing.
		* @param f_max_connecting_dist	Maximum allowed distance between endpoints for joining profiles.
		* @param f_not_self				If true, profiles are not allowed to self-close.
		* @param f_error_on_open_prf	If true, the function returns false if any profile cannot be closed.
		*
		* @return true if all profiles were successfully closed or placed in open list without error,
		*		  false if f_error_on_open_prf is true and any profile could not be closed.
		*
		* @note Closed profiles have their area, orientation, and bounding box recalculated after merging.
		*/
		bool Rejoin(CPolylinesList& f_closed_prfs,
					CPolylinesList& f_open_profiles,
					const double f_max_connecting_dist = HUGE_VAL,
					const bool f_not_self = false,
					const bool f_error_on_open_prf = false) const
		{
			using List = CPolylinesList;
			using Iterator = List::iterator;

			CPolylinesList l_working_prfs_list = *this;

			double l_max_dist_q = 0.0;
			if (std::isfinite(f_max_connecting_dist))
			{
				l_max_dist_q = f_max_connecting_dist * f_max_connecting_dist;
			}
			else
			{
				l_max_dist_q = std::numeric_limits<double>::infinity();
			}

			while (!l_working_prfs_list.empty())
			{
				if (l_working_prfs_list.size() == 1)
				{
					Iterator l_prf_it = l_working_prfs_list.begin();

					bool l_closed = false;

					if (!f_not_self)
					{
						const double l_dist_sq = (l_prf_it->back() - l_prf_it->front()).LengthSq();

						if (l_dist_sq < l_max_dist_q)
						{
							if (Distance(l_prf_it->back(), l_prf_it->front()) > LOOSE_THRESH)
							{
								l_prf_it->emplace_back(l_prf_it->front());
							}

							f_closed_prfs.splice(f_closed_prfs.end(), l_working_prfs_list, l_prf_it);
							f_closed_prfs.back().CalculateAreaAndOrientation();
							f_closed_prfs.back().CalculateBoundingBox();
							l_closed = true;
						}
					}

					if (!l_closed)
					{
						f_open_profiles.splice(f_open_profiles.end(), l_working_prfs_list, l_prf_it);
						if (f_error_on_open_prf)
							return false;
					}

					continue;
				}

				struct Nearest
				{
					Iterator m_candidate;
					double m_dist_sq = std::numeric_limits<double>::infinity();
				};

				std::vector<std::pair<Iterator, Nearest>> l_nearest_prfs;
				l_nearest_prfs.reserve(l_working_prfs_list.size());

				for (Iterator l_prf_it_a = l_working_prfs_list.begin(); l_prf_it_a != l_working_prfs_list.end(); ++l_prf_it_a)
				{
					Nearest n;
					n.m_candidate = l_working_prfs_list.end();

					for (Iterator l_pfr_it_b = l_working_prfs_list.begin(); l_pfr_it_b != l_working_prfs_list.end(); ++l_pfr_it_b)
					{
						if (f_not_self && l_prf_it_a == l_pfr_it_b)
							continue;

						const double l_dist_sq = (l_prf_it_a->back() - l_pfr_it_b->front()).LengthSq();

						if (l_dist_sq < n.m_dist_sq && l_dist_sq < l_max_dist_q)
						{
							n.m_dist_sq = l_dist_sq;
							n.m_candidate = l_pfr_it_b;
						}
					}

					l_nearest_prfs.emplace_back(l_prf_it_a, n);
				}

				bool l_merged = false;

				for (auto& entry : l_nearest_prfs)
				{
					Iterator a = entry.first;
					Iterator b = entry.second.m_candidate;

					if (b == l_working_prfs_list.end())
						continue;

					auto l_prf_b_it = std::find_if(l_nearest_prfs.begin(), l_nearest_prfs.end(), [b](const auto& p) { return p.first == b; });

					if (l_prf_b_it == l_nearest_prfs.end())
						continue;

					if (l_prf_b_it->second.m_candidate != a)
						continue;

					if (a == b)
					{
						if (Distance(a->back(), a->front()) > LOOSE_THRESH)
						{
							a->emplace_back(a->front());
						}

						f_closed_prfs.splice(f_closed_prfs.end(), l_working_prfs_list, a);
						f_closed_prfs.back().CalculateAreaAndOrientation();
						f_closed_prfs.back().CalculateBoundingBox();
					}
					else
					{
						if (Distance(a->back(), b->front()) > LOOSE_THRESH)
						{
							a->insert(a->end(), b->begin(), b->end());
						}
						else
						{
							a->insert(a->end(), std::next(b->begin()), b->end());
						}

						l_working_prfs_list.erase(b);
					}

					l_merged = true;
					break;
				}

				if (!l_merged)
				{
					Iterator it = l_working_prfs_list.begin();
					f_open_profiles.splice(f_open_profiles.end(), l_working_prfs_list, it);

					if (f_error_on_open_prf)
						return false;
				}
			}

			return true;
		}

		/**
		* @brief Groups toolpath profiles into hierarchical sets based on inclusion.
		*
		* This function separates the profiles in the list into counter-clockwise (CCW)
		* and clockwise (CW) profiles. It then attempts to assign each CW profile to the
		* smallest-area CCW profile that contains it, forming groups where a CCW profile
		* acts as an outer boundary and CW profiles are holes inside it.
		*
		* The grouping respects the following rules:
		* - Profiles with fewer than 3 points are ignored.
		* - CW profiles are assigned to the smallest-area CCW profile that contains them.
		* - If f_err_on_intersecting_prfs is true, the function returns false if a CW
		*   profile intersects any CCW profile.
		*
		* @param f_groups A vector of grouped profiles. Each element contains a CCW profile
		*				  followed by any included CW profiles (holes)
		* @param f_err_on_intersecting_prfs If true, the function fails on intersecting profiles.
		*
		* @return true if the grouping succeeds, false if an intersection error occurs (and
		* f_err_on_intersecting_prfs is true).
		*
		* @note The function uses 2D geometric tests (planar XY) to determine inclusion and intersections.
		* @note Profiles are assumed to be simple polygons; self-intersecting profiles may produce undefined behavior.
		*/
		inline bool GroupProfiles(std::vector<CPolylinesList>& f_groups,
								  const bool f_err_on_intersecting_prfs)
		{
			f_groups.clear();

			if (empty())
				return true;

			std::vector<CPolylinesList::const_iterator> l_ccw_prfs;
			std::vector<CPolylinesList::const_iterator> l_cw_profiles;

			l_ccw_prfs.reserve(size());
			l_cw_profiles.reserve(size());

			for (auto l_prf_it = begin(); l_prf_it != end(); ++l_prf_it)
			{
				if (l_prf_it->size() < 3)
				{
					continue;
				}

				if (l_prf_it->IsPrfCW())
				{
					l_cw_profiles.push_back(l_prf_it);
				}
				else
				{
					l_ccw_prfs.push_back(l_prf_it);
				}
			}

			std::vector<std::vector<CPolylinesList::const_iterator>> l_inclusion_lists(l_ccw_prfs.size());

			for (const auto& l_cw_prf_it : l_cw_profiles)
			{
				double l_min_area = std::numeric_limits<double>::max();
				int l_best_index = -1;

				for (std::size_t i = 0; i < l_ccw_prfs.size(); ++i)
				{
					const auto& l_ccw_prf_it = l_ccw_prfs[i];

					if (l_cw_prf_it->IsIntersectingWithPrf(*l_ccw_prf_it, true))
					{
						if (f_err_on_intersecting_prfs)
							return false;

						continue;
					}

					if (l_cw_prf_it->IsInsideOtherPrf(*l_ccw_prf_it))
					{
						const double l_ccw_prf_area = l_ccw_prf_it->GetArea();
						if (l_ccw_prf_area < l_min_area)
						{
							l_min_area = l_ccw_prf_area;
							l_best_index = static_cast<int>(i);
						}
					}
				}

				if (l_best_index >= 0)
				{
					l_inclusion_lists[l_best_index].push_back(l_cw_prf_it);
				}
			}

			f_groups.reserve(l_ccw_prfs.size());

			for (std::size_t i = 0; i < l_ccw_prfs.size(); ++i)
			{
				CPolylinesList group;

				group.emplace_back(*l_ccw_prfs[i]);

				for (const auto& cwIt : l_inclusion_lists[i])
				{
					group.emplace_back(*cwIt);
				}

				f_groups.emplace_back(std::move(group));
			}

			return true;
		}

		/**
		* @brief Computes the minimum distance from a 3D point to the segments of toolpath profiles.
		*
		* This function iterates over all profiles in the toolpath list and calculates the
		* shortest distance from the given point to any segment of each profile. The minimum
		* distance can occur either:
		*   - At the orthogonal projection of the point onto a segment (if projection lies within the segment), or
		*   - At a segment endpoint (if the projection is outside the segment).
		*
		* @param f_point	 The 3D point for which the minimum distance to the profiles is computed.
		* @param f_min_dist	 The resulting minimum distance
		*
		* @return true if a valid minimum distance was found; false if the list contains no valid segments.
		*
		* @note Each profile must contain at least 2 points to define a segment; profiles with fewer points are ignored.
		* @note The distance is computed in 2D (XY plane)
		*/
		inline bool GetMinDistanceAtPoint(const CPoint3& f_point, double &f_min_dist) const
		{
			f_min_dist = LARGE_DOUBLE_CONST;
			size_t l_prf_id = 0;
			for (const auto& l_prf : *this)
			{
				if (l_prf.size() < 2)
				{
					continue;
				}

				for (size_t i = 0; i < l_prf.size() - 1; ++i)
				{
					double l_dist = 0.0;
					CPoint2 l_proj_pt;
					bool l_proj_inside = cgeom_algs::PointProjectionOnSegment(f_point, l_prf[i], l_prf[i + 1], l_proj_pt, l_dist);
					if (l_proj_inside)
					{
						if (l_dist < f_min_dist)
						{
							f_min_dist = l_dist;
						}
					}
					else
					{

						double l_distA = Distance(f_point, l_prf[i]);
						if (l_distA < f_min_dist)
						{
							f_min_dist = l_distA;
						}

						double l_distB = Distance(f_point, l_prf[i + 1]);
						if (l_distB < f_min_dist)
						{
							f_min_dist = l_distB;
						}
					}
				}
				l_prf_id += 1;
			}

			if (f_min_dist == LARGE_DOUBLE_CONST)
			{
				return false;
			}

			return true;
		}

		/**
		* @brief Finds all boundary points within a distance threshold of the true minimum distance
		*        from a given point to the toolpath profiles.
		*
		* Operates in two passes:
		*  - Pass 1: determines the true minimum distance @p f_min_dist from @p f_point to any
		*            segment in the profile list (via @ref GetMinDistanceAtPoint).
		*  - Pass 2: collects all candidate points (projected or vertex) whose distance to
		*            @p f_point falls within the range [f_min_dist, f_min_dist + f_threshold].
		*
		* Each collected point is annotated with metadata stored in its Num fields:
		*  - Num1: profile index in the list
		*  - Num2: segment index within the profile
		*  - Num3: vertex index within the profile, or -1 if the point is an orthogonal projection
		*          onto the segment interior
		*
		* @note Duplicate vertex suppression: adjacent segments share endpoints (vertex i+1 of
		*       segment i is the same geometric point as vertex i of segment i+1). A vertex is
		*       only added once per profile, keyed on (profile_id, vertex_id) via the internal
		*       VertexAlreadyCollected predicate. Projected points are inherently unique per
		*       segment and require no deduplication.
		*
		* @param f_point          The query point in 3D space.
		* @param f_min_dist       The true minimum distance from @p f_point to any profile
		*                         segment. Unchanged if no valid profile is found.
		* @param f_threshold      The tolerance above @p f_min_dist within which additional
		*                         candidate points are collected. Must be >= 0.
		* @param f_closest_points A deduplicated vector of 3D points on the profile boundary
		*                         whose distance to @p f_point is <= (f_min_dist + f_threshold).
		*                         Cleared on entry. Each point carries profile/segment/vertex
		*                         metadata in its Num1/Num2/Num3 fields.
		*
		* @return true  if at least one valid profile segment was found and @p f_min_dist was set.
		* @return false if all profiles are degenerate (fewer than 2 points), in which case
		*               @p f_closest_points will be empty.
		*
		* @see GetMinDistanceAtPoint
		* @see cgeom_algs::PointProjectionOnSegment
		*/
		inline bool GetClosestPointsWithinThreshold(const CPoint3& f_point,
													double& f_min_dist,
													const double f_threshold,
													std::vector<CPoint3>& f_closest_points) const
		{
			f_min_dist = LARGE_DOUBLE_CONST;
			f_closest_points.clear();

			if (!GetMinDistanceAtPoint(f_point, f_min_dist))
			{
				return false;
			}

			auto VertexAlreadyCollected = [&](int f_prf_id, int f_pt_id) -> bool
				{
					for (const auto& l_pt : f_closest_points)
					{
						if (l_pt.GetIntVal1() == f_prf_id &&
							l_pt.GetIntVal3() == f_pt_id &&
							l_pt.GetIntVal3() != -1)
						{
							return true;
						}
					}
					return false;
				};

			f_closest_points.clear();
			const double l_upper_bound = f_min_dist + f_threshold;
			int l_prf_id = 0;

			for (const auto& l_prf : *this)
			{
				if (l_prf.size() < 2)
				{
					continue;
				}
				for (int i = 0; i < l_prf.size() - 1; ++i)
				{
					double l_dist = 0.0;
					CPoint2 l_proj_pt;
					bool l_proj_inside = cgeom_algs::PointProjectionOnSegment(f_point, l_prf[i], l_prf[i + 1], l_proj_pt, l_dist);

					if (l_proj_inside)
					{
						if (l_dist <= l_upper_bound)
						{
							f_closest_points.emplace_back(l_proj_pt.m_x, l_proj_pt.m_y, 0.0);
							f_closest_points.back().SetIntVal1(l_prf_id);
							f_closest_points.back().SetIntVal2(i);
							f_closest_points.back().SetIntVal3(-1);
							f_closest_points.back().SetDoubleVal1(l_dist);
						}
					}
					else
					{
						double l_distA = Distance(f_point, l_prf[i]);
						if (l_distA <= l_upper_bound && !VertexAlreadyCollected(l_prf_id, i))
						{
							f_closest_points.emplace_back(l_prf[i]);
							f_closest_points.back().SetIntVal1(l_prf_id);
							f_closest_points.back().SetIntVal2(i);
							f_closest_points.back().SetIntVal3(i);
							f_closest_points.back().SetDoubleVal1(l_distA);
						}

						double l_distB = Distance(f_point, l_prf[i + 1]);
						if (l_distB <= l_upper_bound && !VertexAlreadyCollected(l_prf_id, i + 1))
						{
							f_closest_points.emplace_back(l_prf[i + 1]);
							f_closest_points.back().SetIntVal1(l_prf_id);
							f_closest_points.back().SetIntVal2(i);
							f_closest_points.back().SetIntVal3(i + 1);
							f_closest_points.back().SetDoubleVal1(l_distB);
						}
					}
				}
				l_prf_id += 1;
			}

			return true;
		}

		/**
		* @brief Detects whether an offset operation caused trimming using an area-based heuristic.
		*
		* This function compares the analytically estimated area of an ideal (untrimmed) offset
		* with the actual area obtained from the generated offset profiles. If the difference
		* exceeds a relative tolerance, the offset is assumed to have undergone trimming
		* (e.g., due to self-intersections, collapses, or topology changes).
		*
		* The analytical estimation is based on the well-known offset area relation:
		*   A_offset = A_original_prfs +(or - for inward offset) offset_dist * L_original_prfs + PI * offset_dist^2
		* where:
		*   - A_original_prfs is the original total area
		*   - L_original_prfs is the total perimeter length
		*   - offset_dist is the offset distance
		* The analytical model assumes an ideal, non-trimmed offset. Any deviation from this model is 
		* intentionally used as an indicator of trimming or topology changes.
		*
		* The implementation combines outward and inward contributions to derive the
		* expected net area variation.
		*
		* @param f_offset_dist	Offset distance. Positive values typically indicate outward offsets,
		*						negative values inward offsets.
		* @param f_offset_prfs	Collection of resulting profiles after offset.
		* @param f_tol			Absolute tolerance threshold used to detect trimming based on the
		*						difference between the analytically estimated offset area and the
		*						actual computed offset area.
		*						Trimming is considered to have occurred if:
		*						|A_actual - A_estimated| > f_tol
		*						The tolerance is expressed in the same squared-length units as the area.
		*						Typical values depend on model scale and numerical precision requirements
		*
		* @return true if the deviation between estimated and actual offset area exceeds tolerance,
		*         indicating that trimming likely occurred; false otherwise.
		*
		* @note Profile orientation (CW/CCW) must be consistent before calling this function.
		*       Incorrect orientation may lead to invalid signed area computations. Call 
		*		RearrangeOrientation() in case it is needed to rebuild the profile orientation
		*/
		inline bool OffsetWasTrimmedAreaBased(double f_offset_dist,
										      const CPolylinesList& f_offset_prfs,
										      const double f_tol
											  ) const
		{
			double l_original_prfs_area = 0.0;
			double l_original_prfs_len = 0.0;
			for (auto l_prf_it = begin(); l_prf_it != end(); l_prf_it++)
			{
				if (l_prf_it->IsPrfCW())
				{
					l_original_prfs_area += l_prf_it->GetArea();
				}
				else
				{
					l_original_prfs_area += l_prf_it->GetArea();
				}
				l_original_prfs_len += l_prf_it->CalculateLength();
			}

			double l_estimated_offset_area = l_original_prfs_area + f_offset_dist * l_original_prfs_len + PI_GRECO * f_offset_dist * f_offset_dist;
			l_estimated_offset_area -= l_original_prfs_area + -f_offset_dist * l_original_prfs_len + PI_GRECO * f_offset_dist * f_offset_dist;
			double l_actual_offset_area = 0.0;
			for (const auto& l_prf : f_offset_prfs)
			{
				if (l_prf.IsPrfCW())
				{
					l_actual_offset_area -= l_prf.GetArea();
				}
				else
				{
					l_actual_offset_area += l_prf.GetArea();
				}
			}
			return fabs(l_actual_offset_area - l_estimated_offset_area) > f_tol;
		}
		/**
		* @brief Draws a list of toolpath profiles
		*
		* @param f_color	Color identifier used for rendering.
		* @param f_redraw	If true, the graphics system may refresh after drawing.
		* @param f_bake		If true, the geometry is baked (persistently stored)
		*					in the graphics backend.
		* @param f_graph_ptr Pointer to the graphics interface responsible for rendering.
		*
		* @note
		* - Assumes f_graph_ptr is valid (non-null).
		*/
		inline void Draw(const int f_color, 
					     const bool f_redraw, 
					     const bool f_bake, 
					     Graph::IGraphInterface* f_graph_ptr) const
		{
			for (auto l_prf_it = begin(); l_prf_it != end(); l_prf_it++)
			{
				l_prf_it->Draw(f_color, f_redraw, f_bake, f_graph_ptr);
			}
		}
	private:

		CAABox m_bounding_box;

	};

	inline bool CPolyline::RemoveHairs(CPolylinesList& f_output_prfs,
									   const double f_spike_distance,
									   bool& f_prf_modified
									   )
	{
		if (empty())
		{
			f_prf_modified = false;
			return true;
		}

		const double l_zlevel = front().m_z;

		PreprocessProfile(f_spike_distance
							);
		if (empty())
		{
			f_prf_modified = true;
			return true;
		}

		std::vector<CSegment2D> l_segs;
		CQuadtree l_quadtree = BuildProfileQuadtree(l_segs);

		std::map<int, CPolyline> l_added_points_map = FindSpikePoints(l_quadtree,
																	  f_spike_distance,
																	  l_zlevel
																	  );

		back().SetIntVal3(front().GetIntVal3());

		if (!SortSpikeMap(l_added_points_map))
		{
			f_prf_modified = false;
			for (auto& pt : *this)
			{
				pt.SetIntVal1(0);
			}
			return true;
		}
		f_prf_modified = true;

		CPolylinesList l_valid_portions;
		CPolylinesList l_invalid_portions;
		SplitIntoPortions(l_added_points_map,
					 	  l_zlevel,
						  l_valid_portions,
						  l_invalid_portions
						  );
		
		StitchFirstAndLastPortions(l_valid_portions
								   );

		EliminateSpikePortions(l_valid_portions, 
							   f_spike_distance
							   );

		if (!l_invalid_portions.empty())
		{
			FilterInvalidPortions(l_invalid_portions,
								  l_valid_portions,
								  l_quadtree,
								  f_spike_distance,
								  l_zlevel
								  );
		}


		if (l_valid_portions.size() == 1)
		{
			auto& l_prf = l_valid_portions.front();
			if (Distance(l_prf.back(), l_prf.front()) < 1.0e-3)
			{
				l_prf.pop_back();
			}
			l_prf.AddSeamPoint();
			l_prf.CalculateAreaAndOrientation();
			l_prf.CalculateBoundingBox();
			f_output_prfs.emplace_back(l_prf);
		}
		else if (!l_valid_portions.empty())
		{
			CPolylinesList seamed = ComposeSeamedPortions(l_valid_portions);

			if (seamed.size() > 1)
			{
				CPolylinesList open_profiles;
				seamed.Rejoin(f_output_prfs, open_profiles);
				if (!open_profiles.empty())
					return false;
			}
			else if (seamed.size() == 1)
			{
				seamed.front().AddSeamPoint();
				seamed.front().CalculateAreaAndOrientation();
				seamed.front().CalculateBoundingBox();
				f_output_prfs.emplace_back(seamed.front());

			}
		}

		auto l_prf_it = f_output_prfs.begin();
		while (l_prf_it != f_output_prfs.end())
		{
			if (l_prf_it->GetArea() > LOOSE_THRESH)
			{
				l_prf_it++;
			}
			else
			{
				l_prf_it = f_output_prfs.erase(l_prf_it);
			}
		}

		for (auto& l_prf : f_output_prfs)
		{
			for (auto& l_pt : l_prf)
			{
				l_pt.SetIntVal1(0);
				l_pt.SetIntVal2(0);
			}
		}

		return true;
	}

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
	inline CPolylinesList CPolyline::SplitByChordalLength(const int f_num) const
	{
		CPolylinesList l_result;
		if (f_num <= 0 || size() < 2)
		{
			return l_result;
		}
		double l_total_length = 0.0;
		for (int i = 0; i < static_cast<int>(size()) - 1; i++)
		{
			l_total_length += Distance(at(i), at(i + 1));
		}

		const double l_sub_length = l_total_length / static_cast<double>(f_num);

		CPolyline l_current_portion;
		l_current_portion.emplace_back(front());
		double l_accumulated = 0.0;
		int l_portions_done = 0;

		for (int i = 0; i < static_cast<int>(size()) - 1; i++)
		{
			const double l_seg_len = Distance(at(i), at(i + 1));

			while (l_portions_done < (f_num - 1))
			{
				const double l_threshold = (l_portions_done + 1) * l_sub_length;
				if ((l_accumulated + l_seg_len) < l_threshold)
				{
					break;
				}
				const double l_remaining = l_threshold - l_accumulated;
				const double l_t = (l_seg_len > LOOSE_THRESH) ? (l_remaining / l_seg_len) : 0.0;
				const CPoint3 l_split_pt(at(i).m_x + l_t * (at(i + 1).m_x - at(i).m_x),
					at(i).m_y + l_t * (at(i + 1).m_y - at(i).m_y),
					at(i).m_z + l_t * (at(i + 1).m_z - at(i).m_z));

				l_current_portion.emplace_back(l_split_pt);
				l_result.emplace_back(l_current_portion);

				l_current_portion.clear();
				l_current_portion.emplace_back(l_split_pt);
				l_portions_done += 1;
			}

			l_accumulated += l_seg_len;
			l_current_portion.emplace_back(at(i + 1));
		}

		if (!l_current_portion.empty())
		{
			l_result.emplace_back(l_current_portion);
		}
		return l_result;
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
	* @param f_graph_ptr         Graphics/logging interface for debug visualisation.
	* @param f_debug             When @c true, each chunk is drawn and logged as it is formed.
	*/
	inline void CPolyline::SplitIntoPortions(const std::map<int, CPolyline>& f_added_points_map,
										     const double f_zlevel,
											 CPolylinesList& f_valid_portions,
											 CPolylinesList& f_invalid_portions
											 ) const
	{
		CPolyline l_valid_chunk;
		CPolyline l_invalid_chunk;
		CPoint3 l_last_pt;
		int l_seg_id = 0;

		for (auto pt_it = begin(); pt_it != end(); ++pt_it, ++l_seg_id)
		{
			if (pt_it != begin())
			{
				if (pt_it->GetIntVal3() == 0)
				{
					if (l_valid_chunk.size() > 0)
					{
						if (Distance(*pt_it, l_valid_chunk.back()) > COINC_THRESH)
						{
							l_valid_chunk.emplace_back(*pt_it);
						}
					}
					else
					{
						l_valid_chunk.emplace_back(l_last_pt);
						l_valid_chunk.emplace_back(*pt_it);
					}
					if (l_invalid_chunk.size() > 0)
					{
						l_invalid_chunk.clear();
					}
				}
				else
				{
					if (l_valid_chunk.size() > 0)
					{
						l_valid_chunk.emplace_back(*pt_it);
						f_valid_portions.emplace_back(l_valid_chunk);
						l_valid_chunk.clear();
					}
					if (l_invalid_chunk.size() > 0)
					{
						l_invalid_chunk.emplace_back(*pt_it);
						f_invalid_portions.emplace_back(l_invalid_chunk);
						l_invalid_chunk.clear();
						l_invalid_chunk.emplace_back(*pt_it);
					}
					else
					{
						l_invalid_chunk.emplace_back(*pt_it);
					}
				}
			}
			else
			{
				if (pt_it->GetIntVal3())
				{
					l_valid_chunk.emplace_back(*pt_it);
				}
				else
				{
					l_invalid_chunk.emplace_back(*pt_it);
				}
			}

			auto map_it = f_added_points_map.find(l_seg_id);
			if (map_it != f_added_points_map.end() && !map_it->second.empty())
			{
				if (l_valid_chunk.size() > 0)
				{
					l_valid_chunk.emplace_back(map_it->second.front());
					f_valid_portions.emplace_back(l_valid_chunk);
					l_valid_chunk.clear();
				}

				for (auto& l_added_pt : map_it->second)
				{
					if (l_invalid_chunk.empty())
					{
						l_invalid_chunk.emplace_back(l_added_pt);
					}
					else
					{
						l_invalid_chunk.emplace_back(l_added_pt);
						f_invalid_portions.emplace_back(l_invalid_chunk);
						l_invalid_chunk.clear();
						l_invalid_chunk.emplace_back(l_added_pt);
					}
				}
				l_last_pt = map_it->second.back();
			}
			else
			{
				l_last_pt = *pt_it;
			}

			if ((pt_it == end() - 1) && l_valid_chunk.size() > 0)
			{
				f_valid_portions.emplace_back(l_valid_chunk);
				l_valid_chunk.clear();
			}
		}
	}

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
	* @param f_graph_ptr  Graphics/logging interface for debug messages.
	* @param f_debug      When @c true, adjacency result is logged.
	*/
	inline void CPolyline::StitchFirstAndLastPortions(CPolylinesList& f_portions
													  )
	{
		if (f_portions.size() < 2) return;
		if (Distance(f_portions.front().front(), f_portions.back().back()) >= COINC_THRESH)
		{
			return;
		}

		for (auto pt_it = f_portions.front().begin() + 1; pt_it != f_portions.front().end(); ++pt_it)
		{
			f_portions.back().emplace_back(*pt_it);
		}
		f_portions.pop_front();
	}

	inline void CPolyline::EliminateSpikePortions(CPolylinesList& f_portions,
												  const double f_spike_distance
												  )
	{
		const int l_num_portions_before = (int)f_portions.size();
		auto l_it = f_portions.begin();
		while (l_it != f_portions.end())
		{
			if (l_it->size() == 3 && cgeom_algs::IsPointSpikeApex(l_it->at(1), l_it->front(), l_it->back(), f_spike_distance))
			{
				l_it = f_portions.erase(l_it);
			}
			else
			{
				++l_it;
			}
		}
	}

	inline void CPolyline::FilterInvalidPortions(CPolylinesList& f_invalid_portions,
												 CPolylinesList& f_valid_portions,
												 CQuadtree& f_quadtree,
												 const double f_spike_distance,
												 const double f_zlevel
												 )
	{
		std::vector<IGeometricalObject2D*> l_query_pool;
		auto l_inv_it = f_invalid_portions.begin();

		while (l_inv_it != f_invalid_portions.end())
		{
			bool l_to_be_eliminated = false;

			if (Distance(l_inv_it->front(), l_inv_it->back()) < f_spike_distance)
			{
				l_to_be_eliminated = true;
			}

			if (!l_to_be_eliminated)
			{
				CPoint2 l_mid_pt;
				l_mid_pt.m_x = (l_inv_it->front().m_x + l_inv_it->back().m_x) / 2.0;
				l_mid_pt.m_y = (l_inv_it->front().m_y + l_inv_it->back().m_y) / 2.0;
				f_quadtree.QueryRegion(MakeQueryBox(l_mid_pt.m_x, l_mid_pt.m_y, f_spike_distance), l_query_pool, true);

				for (auto& l_object : l_query_pool)
				{
					auto* l_seg = dynamic_cast<CSegment2D*>(l_object);
					if (!l_seg)
					{
						continue;
					}
					double l_dist = 0.0;
					CPoint2 l_proj_pt;
					bool l_proj_is_inside = cgeom_algs::PointProjectionOnSegment(l_mid_pt, l_seg->GetStartPt(), l_seg->GetEndPt(), l_proj_pt, l_dist);

					if (l_proj_is_inside && (l_dist <= f_spike_distance) && (Distance(l_mid_pt, l_proj_pt) > COINC_THRESH))
					{
						l_to_be_eliminated = true;
						break;
					}
				}
			}

			if (l_to_be_eliminated)
			{
				l_inv_it = f_invalid_portions.erase(l_inv_it);
			}
			else
			{
				f_valid_portions.emplace_back(*l_inv_it);
				++l_inv_it;
			}
		}
	}

	inline CPolylinesList CPolyline::ComposeSeamedPortions(CPolylinesList& f_valid_portions)
	{
		CPolylinesList l_seamed;
		CPolyline l_composing;
		bool l_composing_profiles = true;

		while (l_composing_profiles)
		{
			int l_cur_end_id = 0;
			int l_cur_end_eliminating_id = 0;

			for (auto& l_prf : f_valid_portions)
			{
				if (!l_prf.GetValidPrf())
				{
					continue;
				}
				if (l_composing.empty())
				{
					l_composing.reserve(l_prf.size());
					for (auto& pt : l_prf)
					{
						l_composing.emplace_back(pt);
					}
					l_prf.SetValidPrf(false);
					l_cur_end_id = l_prf.back().GetIntVal1();
					l_cur_end_eliminating_id = l_prf.back().GetIntVal2();
					continue;
				}

				if (l_prf.front().GetIntVal1() == l_cur_end_eliminating_id && l_prf.front().GetIntVal2() == l_cur_end_id)
				{
					l_cur_end_id = l_prf.back().GetIntVal1();
					l_cur_end_eliminating_id = l_prf.back().GetIntVal2();

					if (Distance(l_composing.back(), l_prf.front()) < 1.0e-3)
					{
						l_composing.pop_back();
					}

					l_composing.reserve(l_composing.size() + l_prf.size());
					for (auto& l_point : l_prf)
					{
						l_composing.emplace_back(l_point);
					}
					l_prf.SetValidPrf(false);
				}
			}

			if (!l_composing.empty())
			{
				l_seamed.emplace_back(l_composing);
				l_composing.clear();
			}

			l_composing_profiles = std::any_of(f_valid_portions.begin(), f_valid_portions.end(), [](const CPolyline& p) { return p.GetValidPrf(); });
		}

		return l_seamed;
	}
}

#endif