
#ifndef H_CAA_BOX
#define H_CAA_BOX

#include "CPoint3.h"
#include "CRay.h"
#include "EPlaneRelation.h"

#include "CPlane.h"
#include <vector>

namespace ggeomlib
{
	/**
	* @class CAABox
	* @brief Class representing an axis-aligned bounding box (AABB) with geometric properties.
	*
	* This class stores geometric data for an AABB in 3D space, including its center (centroid),
	* half-edge lengths, half-diagonal length, and corner coordinates. It is used for geometric
	* computations and spatial partitioning (e.g., in octrees), but does not store references
	* to external geometry (e.g., triangles).
	*/
	class CAABox
	{
	public:

		/**
		* @brief Default constructor. Initializes the box with zeroed diagonal length.
		*/
		CAABox() :
			m_half_diagonal_length(0.0)
		{

		}

		/**
		* @brief Constructs an axis-aligned box using the lower-left and upper-right corner vertices.
		*
		* @param f_ll_vert The lower-left (back) corner of the box.
		* @param f_ur_vert The upper-right (front) corner of the box.
		*/
		CAABox(const CPoint3& f_ll_vert,
			   const CPoint3& f_ur_vert)
		{
			m_centroid = CPoint3((f_ll_vert.m_x + f_ur_vert.m_x) / 2.0,
								(f_ll_vert.m_y + f_ur_vert.m_y) / 2.0,
								(f_ll_vert.m_z + f_ur_vert.m_z) / 2.0);
			m_half_diagonal_length = 0.0;
			for (int l_i = 0; l_i < 3; l_i++)
			{
				m_halfedge_length[l_i] = fabs((f_ur_vert.m_pc[l_i] - f_ll_vert.m_pc[l_i]) / 2.0);
				m_half_diagonal_length += m_halfedge_length[l_i] * m_halfedge_length[l_i];
			}
			m_half_diagonal_length = sqrt(m_half_diagonal_length);

			m_ll_vert = f_ll_vert;
			m_ur_vert = f_ur_vert;
		}

		/** @brief Sets the centroid (center) of the box. */
		inline void SetCentroid(const CPoint3& f_centroid)
		{
			m_centroid = f_centroid;
		}

		/** @brief Gets the centroid (center) of the box. */
		inline CPoint3 GetCentroid() const
		{
			return m_centroid;
		}

		/**
		* @brief Gets the half-edge lengths of the box in each of the X, Y, Z directions.
		*
		* @return A vector containing the lengths of the semi-edges.
		*/
		inline CPoint3 GetHalfEdgeLength() const
		{
			return m_halfedge_length;
		}
		/**
		* @brief Sets the half-edge lengths (semi-edges) of the box.
		*
		* @param f_halfedge_length A vector with the semi-edge lengths in X, Y, Z.
		*/
		inline void SetHalfEdgeLength(CPoint3& f_halfedge_length)
		{
			m_halfedge_length = f_halfedge_length;
		}

		/**
		* @brief Returns the lower-left vertex (back corner) of the box.
		*
		* @return The minimum corner of the bounding box.
		*/
		inline CPoint3 GetLLVertex() const
		{
			return m_ll_vert;
		}

		/*
		* @brief Set the lower left vertex of the box
		* 
		* @param f_ll_vert	The low left vertex to set
		*/
		inline void SetLLVert(const CPoint3& f_ll_vert)
		{
			m_ll_vert = f_ll_vert;
		}

		/*
		* @brief Set the upper right vertex of the box
		*
		* @param f_ll_vert	The low left vertex to set
		*/
		inline void SetURVert(const CPoint3& f_ur_vert)
		{
			m_ur_vert = f_ur_vert;
		}
		/**
		* @brief Returns the upper-right vertex (front corner) of the box.
		*
		* @return The maximum corner of the bounding box.
		*/
		inline CPoint3 GetURVertex() const
		{
			return m_ur_vert;
		}

		/**
		* @brief Returns the half-length of the diagonal of the box.
		*
		* This value represents the distance from the centroid to a corner.
		*/
		const double GetHalfDiagonaLength() const
		{
			return m_half_diagonal_length;
		}

		/**
		* @brief Checks whether a given point lies inside the box (non-inclusive of faces).
		*
		* @param f_point The point to check.
		* @return True if the point is strictly inside the box, false otherwise.
		*/
		inline bool IsPointInsideBox(CPoint3 f_point) const
		{
			if ((m_ll_vert.m_x < f_point.m_x) && (m_ur_vert.m_x > f_point.m_x) &&
				(m_ll_vert.m_y < f_point.m_y) && (m_ur_vert.m_y > f_point.m_y) &&
				(m_ll_vert.m_z < f_point.m_z) && (m_ur_vert.m_z > f_point.m_z))
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		/**
		* @brief Computes the volume of the box.
		*
		* @return The volume of the axis-aligned bounding box.
		*/
		inline double GetVolume() const
		{
			return ((m_ur_vert.m_x - m_ll_vert.m_x) * (m_ur_vert.m_y - m_ll_vert.m_y) * (m_ur_vert.m_z - m_ll_vert.m_z));
		}

		/**
		* @brief Computes the intersection point between a ray and this axis-aligned bounding box (AABB).
		*
		* This function tests the input ray against all six faces of the AABB.
		* If an intersection point is found within the bounds of the box (considering a small threshold
		* f_threshold), it is returned through the output parameter and the function returns true.
		*
		* @param[in]  f_ray        The ray to test for intersection with the bounding box.
		* @param[in]  f_threshold  A tolerance value used to slightly expand the box bounds during testing.
		*                          Helps account for numerical precision issues.
		*						   (typically DOUBLE_PRECISION_EPSILON)
		* @param[out] f_point      If an intersection is found, this will be set to the intersection point.
		*
		* @return True if the ray intersects the AABB; otherwise, false.
		*/
		inline bool ComputeIntersWithRay(const CRay& f_ray,
								  const double& f_threshold,
								  CPoint3& f_point) const
		{
			const CPoint3& l_ray_dir = f_ray.GetDir();
			const CPoint3& l_ray_start = f_ray.GetPStart();

			for (int l_iaxis = 0; l_iaxis < 3; l_iaxis++)
			{
				double l_dir = l_ray_dir[l_iaxis];
				if (fabs(l_dir) < COINC_THRESH)
				{
					continue;
				}

				for (int l_iplane = 0; l_iplane < 2; l_iplane++)
				{
					double l_box_plane_coord = (l_iplane == 0) ? m_ll_vert[l_iaxis] : m_ur_vert[l_iaxis];
					double l_t_param = (l_box_plane_coord - l_ray_start[l_iaxis]) / l_dir;

					if (l_t_param <= 0.0)
					{
						continue;
					}

					CPoint3 l_intersection;
					l_intersection = l_ray_start + l_ray_dir * l_t_param;

					bool l_is_inside = true;
					for (int l_iother_axis = 0; l_iother_axis < 3; l_iother_axis++)
					{
						if (l_iother_axis == l_iaxis)
						{
							continue;
						}
						double l_min_bound = m_ll_vert[l_iother_axis] - f_threshold;
						double l_max_bound = m_ur_vert[l_iother_axis] + f_threshold;
						if (l_intersection[l_iother_axis] < l_min_bound || l_intersection[l_iother_axis] > l_max_bound)
						{
							l_is_inside = false;
							break;
						}
					}

					if (l_is_inside)
					{
						l_intersection[l_iaxis] = l_box_plane_coord;
						f_point = l_intersection;
						return true;
					}
				}
			}

			return false;

		}
		/**
		* @brief Computes the intersection points between an axis-aligned bounding box (AABB) and a plane (i.e. the
		*		 intersection points between a plane and the edges of the box)
		*
		* This function tests all 12 edges of the box (4 edges along each axis) for intersection with the given plane.
		* If an intersection occurs on a given edge, the intersection point is added to the output vector.
		* If more than two intersection points are found, the points are sorted in counter-clockwise order (with respect
		* to the plane's normal and first point as origin) to represent the resulting polygon on the plane.
		* Duplicate points (within a certain threshold) are removed.
		*
		* @param[in]  f_plane		The plane to test for intersection against the AABB.
		* @param[out] f_int_points  A vector that will contain the intersection points if any exist.
		*                           Points are sorted if more than two are found.
		*
		* @return `true` if the plane intersects the box and at least one intersection point is found;
		*         `false` otherwise.
		*
		* @note
		* - The box is assumed to be axis-aligned (i.e. AABB)
		* - The ray-plane intersection is bounded within the dimensions of the box.
		* - Duplicate points are filtered using a global threshold COINC_THRESH.
		*
		* @see CPlane::IntersectWithRay
		* @see CVec3
		* @see CRay
		*/
		inline bool PlaneBoxIntersection(const CPlane& f_plane, 
								  std::vector<CPoint3>& f_int_points) const
		{
			CPoint3 l_int_pt;
			double l_t_param = 0.0;
			const double l_x_box_side = m_halfedge_length.m_x * 2.0;
			const double l_y_box_side = m_halfedge_length.m_y * 2.0;
			const double l_z_box_side = m_halfedge_length.m_z * 2.0;

			CPoint3 l_ray_dir = CPoint3(1.0, 0.0, 0.0);
			CPoint3 l_ray_orig = m_ll_vert;
			if (f_plane.IntersectWithRay(CRay(l_ray_orig, l_ray_dir), l_int_pt, l_t_param) && ((l_t_param >= 0) && (l_t_param <= l_x_box_side)))
			{
				f_int_points.emplace_back(l_int_pt);
			}
			l_ray_orig = CPoint3(m_ll_vert.m_x, m_ur_vert.m_y, m_ll_vert.m_z);
			if (f_plane.IntersectWithRay(CRay(l_ray_orig, l_ray_dir), l_int_pt, l_t_param) && ((l_t_param >= 0) && (l_t_param <= l_x_box_side)))
			{
				f_int_points.emplace_back(l_int_pt);
			}
			l_ray_orig = CPoint3(m_ll_vert.m_x, m_ll_vert.m_y, m_ur_vert.m_z);
			if (f_plane.IntersectWithRay(CRay(l_ray_orig, l_ray_dir), l_int_pt, l_t_param) && ((l_t_param >= 0) && (l_t_param <= l_x_box_side)))
			{
				f_int_points.emplace_back(l_int_pt);
			}
			l_ray_orig = CPoint3(m_ll_vert.m_x, m_ur_vert.m_y, m_ur_vert.m_z);
			if (f_plane.IntersectWithRay(CRay(l_ray_orig, l_ray_dir), l_int_pt, l_t_param) && ((l_t_param >= 0) && (l_t_param <= l_x_box_side)))
			{
				f_int_points.emplace_back(l_int_pt);
			}

			l_ray_dir = CPoint3(0.0, 1.0, 0.0);
			l_ray_orig = m_ll_vert;
			if (f_plane.IntersectWithRay(CRay(l_ray_orig, l_ray_dir), l_int_pt, l_t_param) && ((l_t_param >= 0) && (l_t_param <= l_y_box_side)))
			{
				f_int_points.emplace_back(l_int_pt);
			}
			l_ray_orig = CPoint3(m_ur_vert.m_x, m_ll_vert.m_y, m_ll_vert.m_z);
			if (f_plane.IntersectWithRay(CRay(l_ray_orig, l_ray_dir), l_int_pt, l_t_param) && ((l_t_param >= 0) && (l_t_param <= l_y_box_side)))
			{
				f_int_points.emplace_back(l_int_pt);
			}
			l_ray_orig = CPoint3(m_ll_vert.m_x, m_ll_vert.m_y, m_ur_vert.m_z);
			if (f_plane.IntersectWithRay(CRay(l_ray_orig, l_ray_dir), l_int_pt, l_t_param) && ((l_t_param >= 0) && (l_t_param <= l_y_box_side)))
			{
				f_int_points.emplace_back(l_int_pt);
			}
			l_ray_orig = CPoint3(m_ur_vert.m_x, m_ll_vert.m_y, m_ur_vert.m_z);
			if (f_plane.IntersectWithRay(CRay(l_ray_orig, l_ray_dir), l_int_pt, l_t_param) && ((l_t_param >= 0) && (l_t_param <= l_y_box_side)))
			{
				f_int_points.emplace_back(l_int_pt);
			}

			l_ray_dir = CPoint3(0.0, 0.0, 1.0);
			l_ray_orig = m_ll_vert;
			if (f_plane.IntersectWithRay(CRay(l_ray_orig, l_ray_dir), l_int_pt, l_t_param) && ((l_t_param >= 0) && (l_t_param <= l_z_box_side)))
			{
				f_int_points.emplace_back(l_int_pt);
			}
			l_ray_orig = CPoint3(m_ur_vert.m_x, m_ll_vert.m_y, m_ll_vert.m_z);
			if (f_plane.IntersectWithRay(CRay(l_ray_orig, l_ray_dir), l_int_pt, l_t_param) && ((l_t_param >= 0) && (l_t_param <= l_z_box_side)))
			{
				f_int_points.emplace_back(l_int_pt);
			}
			l_ray_orig = CPoint3(m_ll_vert.m_x, m_ur_vert.m_y, m_ll_vert.m_z);
			if (f_plane.IntersectWithRay(CRay(l_ray_orig, l_ray_dir), l_int_pt, l_t_param) && ((l_t_param >= 0) && (l_t_param <= l_z_box_side)))
			{
				f_int_points.emplace_back(l_int_pt);
			}
			l_ray_orig = CPoint3(m_ur_vert.m_x, m_ur_vert.m_y, m_ll_vert.m_z);
			if (f_plane.IntersectWithRay(CRay(l_ray_orig, l_ray_dir), l_int_pt, l_t_param) && ((l_t_param >= 0) && (l_t_param <= l_z_box_side)))
			{
				f_int_points.emplace_back(l_int_pt);
			}

			if (f_int_points.size() == 0)
			{
				return false;
			}

			if (f_int_points.size() > 2)
			{

				const CPoint3 l_plane_normal = f_plane.GetNormal();
				const CPoint3 l_origin = f_int_points.front();

				std::sort(f_int_points.begin(), f_int_points.end(), [&](const CPoint3& lhs, const CPoint3& rhs) -> bool {
					CPoint3 v = Cross((lhs - l_origin), (rhs - l_origin));
					return Dot(v, l_plane_normal) < 0;
					});

				std::vector<CPoint3> l_unique_pts;
				l_unique_pts.emplace_back(f_int_points.front());
				for (auto l_pt_it = f_int_points.begin() + 1; l_pt_it != f_int_points.end(); l_pt_it++)
				{
					if (Distance(l_unique_pts.back(), *l_pt_it) > COINC_THRESH)
					{
						l_unique_pts.emplace_back(*l_pt_it);
					}
				}
				f_int_points.swap(l_unique_pts);
			}

			return true;
		}

		/**
		* @brief Determines the position of the box relative to a plane.
		*
		* This function computes whether the box is entirely in front of, entirely behind,
		* or intersecting a given plane. It does this by calculating the signed distance from
		* the box centroid to the plane and comparing it against the projected support radius
		* of the box along the plane normal.
		*
		* @param f_plane The plane to test against.
		* @return A value of type @ref PlaneRelation indicating the relative position.
		*/
		inline EPlaneRelation PositionRespectToPlane(const CPlane& f_plane) const
		{
			CPoint3 l_distance_vec = m_centroid - f_plane.GetPointOnPlane();

			const CPoint3& l_box_half_extents = GetHalfEdgeLength();
			const CPoint3& normal = f_plane.GetNormal();

			double l_support_radius = l_box_half_extents.m_x * fabs(normal.m_x) +
									  l_box_half_extents.m_y * fabs(normal.m_y) +
									  l_box_half_extents.m_z * fabs(normal.m_z);

			double l_signed_distance = Dot(l_distance_vec, normal);

			if ((l_signed_distance + l_support_radius) < -COINC_THRESH)
			{
				return EPlaneRelation::FullyBehind;
			}

			if ((l_signed_distance - l_support_radius) > COINC_THRESH)
			{
				return EPlaneRelation::FullyInFront;
			}

			return EPlaneRelation::Intersecting;
		}

	private:
		CPoint3 m_centroid;
		CPoint3 m_halfedge_length;
		double m_half_diagonal_length;
		CPoint3 m_ll_vert;
		CPoint3 m_ur_vert;

	};
}

#endif