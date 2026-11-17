
#pragma once

#ifndef H_CTRIANGLE
#define H_CTRIANGLE

#include "CPoint3.h"
#include "CompGeomAlgorithms.h"
#include "CRay.h"
#include "CRayCastResult.h"
#include "MathAlgorithms.h"
#include "IGeometricalObject3D.h"

#include <cmath>

namespace ggeomlib
{
	struct THalfEdge;

	/**
	* @enum ETriangleType
	* @brief Enumeration representing the type of a triangle based on its angles.
	*/
	enum class ETriangleType
	{
		Acute,
		Right,
		Obtuse
	};

	/**
	* @struct STriangleAngleInfo
	* @brief Structure that stores information about a triangle's angle classification.
	*/
	struct STriangleAngleInfo
	{
		/**
		* @brief The overall triangle type (acute, right, or obtuse).
		*/
		ETriangleType m_tri_type = ETriangleType::Acute;
		/**
		* @brief Index of the vertex where the obtuse or right angle is located.
		*
		* Possible values:
		* - 1 -> vertex 1
		* - 2 -> vertex 2
		* - 3 -> vertex 3
		* - 0 -> no obtuse/right angle detected (defaults to acute)
		*/
		int m_obtuse_or_right_angle = 0;
	};

	/**
	* @class CTriangle
	* @brief Represents a triangle in 3D space with operations such as normal calculation, area computation,
	*        bounding box generation, and intersection tests.
	*/
	class CTriangle : public IGeometricalObject3D
	{
	public:

		/**
		* @brief Constructs a triangle with given vertex pointers and optional vertex IDs and triangle number.
		*
		* @param f_vertex1 Pointer to the first vertex.
		* @param f_vertex2 Pointer to the second vertex.
		* @param f_vertex3 Pointer to the third vertex.
		* @param f_vert_1_id ID of the first vertex (optional).
		* @param f_vert_2_id ID of the second vertex (optional).
		* @param f_vert_3_id ID of the third vertex (optional).
		* @param f_triangle_number Index or identifier for this triangle (optional).
		*/
		CTriangle(const CPoint3* f_vertex1,
				  const CPoint3* f_vertex2,
				  const CPoint3* f_vertex3,
				  const int f_vert_1_id = -1,
				  const int f_vert_2_id = -1,
				  const int f_vert_3_id = -1,
				  const int f_triangle_number = -1)	:
			m_vertex1(f_vertex1),
			m_vertex2(f_vertex2),
			m_vertex3(f_vertex3),
			m_vert_1_id(f_vert_1_id),
			m_vert_2_id(f_vert_2_id),
			m_vert_3_id(f_vert_3_id)
		{
			SetTriangleNumber(f_triangle_number);
			CalculateNormal();
			m_d_coefficient = Dot(GetVertex1(), GetNormal());
		}

		/**
		* @brief Returns the axis-aligned bounding box of the triangle.
		*
		* @param[out] f_x_min Minimum X of the triangle.
		* @param[out] f_y_min Minimum Y of the triangle.
		* @param[out] f_z_min Minimum Z of the triangle.
		* @param[out] f_x_max Maximum X of the triangle.
		* @param[out] f_y_max Maximum Y of the triangle.
		* @param[out] f_z_max Maximum Z of the triangle.
		*
		* Implementations must return finite values with min <= max on each
		* axis (degenerate boxes, e.g. for a point, are allowed). The box must
		* contain the whole triangle; it does not need to be tight, but a tight box
		* gives fewer candidate cells.
		*/
		inline void Bounds(double& f_x_min,
						   double& f_y_min,
						   double& f_z_min,
						   double& f_x_max,
						   double& f_y_max,
						   double& f_z_max) const override
		{
			CPoint3 l_ll_vert;
			CPoint3 l_ur_vert;
			GetTriangleBoundingBox(l_ll_vert, l_ur_vert);
			f_x_min = l_ll_vert.m_x;
			f_y_min = l_ll_vert.m_y;
			f_z_min = l_ll_vert.m_z;
			f_x_max = l_ur_vert.m_x;
			f_y_max = l_ur_vert.m_y;
			f_z_max = l_ur_vert.m_z;
		}

		/**
		* @brief Tests whether the triangle intersects an axis-aligned box.
		*
		* The box is closed: a triangle touching its boundary intersects it.
		*
		* @param f_x_min Minimum X of the box.
		* @param f_y_min Minimum Y of the box.
		* @param f_z_min Minimum Z of the box.
		* @param f_x_max Maximum X of the box.
		* @param f_y_max Maximum Y of the box.
		* @param f_z_max Maximum Z of the box (with min <= max on each axis).
		*
		* @return true if the triangle and the box share at least one point.
		*
		* Implementations should be exact. A conservative test (returning true
		* for some boxes the triangle does not touch) is also acceptable: spatial
		* structures then store the triangle in a few extra cells, which costs
		* performance but not correctness. Returning false for a box the triangle
		* does touch is not acceptable: the triangle would be missing from that
		* cell.
		*/
		inline bool IntersectsWith3DBox(const double f_x_min,
										const double f_y_min,
										const double f_z_min,
										const double f_x_max,
										const double f_y_max,
										const double f_z_max) const override
		{
			return IsTriangleCollidingWithAABB(CPoint3(f_x_min, f_y_min, f_z_min), CPoint3(f_x_max, f_y_max, f_z_max));
		}

		/** @brief Sets the triangle number. */
		inline void SetTriangleNumber(int f_triangle_number)
		{
			m_triangle_number = f_triangle_number;
		}
		/** @brief Gets the triangle number. */
		inline int GetTriangleNumber() const
		{
			return m_triangle_number;
		}

		/** @brief Gets the value of the first vertex. */
		inline CPoint3 GetVertex1() const
		{
			return *m_vertex1;
		}
		/** @brief Gets the value of the second vertex. */
		inline CPoint3 GetVertex2() const
		{
			return *m_vertex2;
		}
		/** @brief Gets the value of the third vertex. */
		inline CPoint3 GetVertex3() const
		{
			return *m_vertex3;
		}
		/** @brief Gets the normal vector of the triangle. */
		inline CPoint3 GetNormal() const
		{
			return m_normal;
		}
		/** @brief Computes the triangle normal and stores it in m_normal. */
		inline void CalculateNormal()
		{
			m_normal = Cross((GetVertex2() - GetVertex1()), (GetVertex3() - GetVertex1()));
			m_normal.UnitizeInPlace();
		}

		/** @brief Gets a reference to the first vertex. */
		inline const CPoint3& GetVertex1Ref() const
		{
			return *m_vertex1;
		}
		/** @brief Gets a reference to the second vertex. */
		inline const CPoint3& GetVertex2Ref() const
		{
			return *m_vertex2;
		}
		/** @brief Gets a reference to the third vertex. */
		inline const CPoint3& GetVertex3Ref() const
		{
			return *m_vertex3;
		}

		/** @brief Sets the pointer to the first vertex. */
		inline void SetVertex1(CPoint3* f_vertex)
		{
			m_vertex1 = f_vertex;
		}
		/** @brief Sets the pointer to the second vertex. */
		inline void SetVertex2(CPoint3* f_vertex)
		{
			m_vertex2 = f_vertex;
		}
		/** @brief Sets the pointer to the third vertex. */
		inline void SetVertex3(CPoint3* f_vertex)
		{
			m_vertex3 = f_vertex;
		}
		/** @brief Returns the maximum Z-coordinate among the triangle’s vertices. */
		inline double GetZMax() const
		{
			return std::max(std::max(GetVertex1().m_z, GetVertex2().m_z), GetVertex3().m_z);
		}
		/** @brief Returns the minimum Z-coordinate among the triangle’s vertices. */
		inline double GetZMin() const
		{
			return std::min(std::min(GetVertex1().m_z, GetVertex2().m_z), GetVertex3().m_z);
		}
		/** @brief Returns the `d` coefficient of the triangle's plane equation. */
		inline double Get_d_Coefficient() const
		{
			return m_d_coefficient;
		}
		/** @brief Gets the ID of the first vertex. */
		inline int GetVert1ID() const
		{
			return m_vert_1_id;
		}
		/** @brief Sets the ID of the first vertex. */
		inline void SetVert1ID(const int f_vert_1_id)
		{
			m_vert_1_id = f_vert_1_id;
		}
		/** @brief Gets the ID of the second vertex. */
		inline int GetVert2ID() const
		{
			return m_vert_2_id;
		}
		/** @brief Sets the ID of the second vertex. */
		inline void SetVert2ID(const int f_vert_2_id)
		{
			m_vert_2_id = f_vert_2_id;
		}
		/** @brief Gets the ID of the third vertex. */
		inline int GetVert3ID() const
		{
			return m_vert_3_id;
		}
		/** @brief Sets the ID of the third vertex. */
		inline void SetVert3ID(const int f_vert_3_id)
		{
			m_vert_3_id = f_vert_3_id;
		}
		/**
		* @brief Calculates the area of the triangle in 3D space.
		* @return The triangle area.
		*/
		inline double CalculateArea() const
		{
			return (Cross(((*m_vertex2) - (*m_vertex1)), ((*m_vertex3) - (*m_vertex1))).Length()) * 0.5;
		}
		/**
		* @brief Calculates the perimeter of the triangle
		* @return The triangle perimeter.
		*/
		inline double CalculatePerimeter() const
		{
			const double l_edge_12 = Distance(*m_vertex1, *m_vertex2);
			const double l_edge_23 = Distance(*m_vertex2, *m_vertex3);
			const double l_edge_31 = Distance(*m_vertex3, *m_vertex1);

			return l_edge_12 + l_edge_23 + l_edge_31;
		}

		/**
		* @brief Computes the 2D bounding box of the triangle projection on a plane.
		*
		* @param f_axis		Axis perpendicular to the projection plane (0=YZ, 1=XZ, 2=XY).
		* @param f_bbox_min	Output lower-left bounding box corner.
		* @param f_bbox_max	Output upper-right bounding box corner.
		*/
		inline void GetTriangleProjectionBoundingBox(const unsigned int f_axis,
													  CPoint2& f_bbox_min,
													  CPoint2& f_bbox_max) const
		{
			cgeom_algs::GetTriangleProjectionBoundingBox(*m_vertex1,
														 *m_vertex2,
														 *m_vertex3,
														 f_axis,
														 f_bbox_min,
														 f_bbox_max);
		}

		/**
		* @brief Computes the 3D axis-aligned bounding box of the triangle.
		*
		* @param f_ll_box Output lower-left (minimum) bounding box corner.
		* @param f_ur_box Output upper-right (maximum) bounding box corner.
		*/
		inline void GetTriangleBoundingBox(CPoint3& f_ll_box, CPoint3& f_ur_box) const
		{
			f_ll_box.m_x = std::min(std::min(m_vertex1->m_x, m_vertex2->m_x), m_vertex3->m_x);
			f_ll_box.m_y = std::min(std::min(m_vertex1->m_y, m_vertex2->m_y), m_vertex3->m_y);
			f_ll_box.m_z = std::min(std::min(m_vertex1->m_z, m_vertex2->m_z), m_vertex3->m_z);

			f_ur_box.m_x = std::max(std::max(m_vertex1->m_x, m_vertex2->m_x), m_vertex3->m_x);
			f_ur_box.m_y = std::max(std::max(m_vertex1->m_y, m_vertex2->m_y), m_vertex3->m_y);
			f_ur_box.m_z = std::max(std::max(m_vertex1->m_z, m_vertex2->m_z), m_vertex3->m_z);
		}

		/**
		* @brief Checks whether a 3D point is inside the triangle using barycentric coordinates.
		*
		* @param f_point_to_test The point to test.
		* @param f_threshold	 Threshold to compare with barycenter coordinates to determine 
		*						 whether a point is inside or outside the triangle. The lower
		*						 the threshold, the stricter the acceptance of the point
		* 
		* @return true if the point lies within the triangle, false otherwise.
		*/
		const bool IsPointInTriangle(CPoint3 f_point_to_test,
							   const double f_threshold = COINC_THRESH) const
		{

			CPoint3 v0 = GetVertex2() - GetVertex1();
			CPoint3 v1 = GetVertex3() - GetVertex1();
			CPoint3 v2 = f_point_to_test - GetVertex1();

			double d00 = Dot(v0, v0);
			double d01 = Dot(v0, v1);
			double d11 = Dot(v1, v1);
			double d20 = Dot(v2, v0);
			double d21 = Dot(v2, v1);
			double denom = d00 * d11 - d01 * d01;

			if (denom <= 0)
			{
				return false;
			}

			double v = (d11 * d20 - d01 * d21) / denom;
			double w = (d00 * d21 - d01 * d20) / denom;
			double u = 1.0 - v - w;

			double tol = -f_threshold;
			return ((v >= tol) && (w >= tol) && (u >= tol));
		}

		/**
		* @brief Tests intersection between a ray and this triangle.
		*
		* @param f_ray The ray to test.
		* @param f_raycast_result Output intersection data if the ray hits the triangle.
		* @param f_threshold	 Threshold to compare with barycenter coordinates to determine 
		*						 whether a point is inside or outside the triangle. The lower
		*						 the threshold, the stricter the acceptance of the point
		* 
		* @return true if the ray intersects the triangle, false otherwise.
		*/
		inline bool RayTriangleIntersect(const CRay& f_ray, 
								  CRayCastResult& f_raycast_result,
								  const double f_threshold) const
		{

			bool l_exiting_the_model = false;

			double l_crack_tolerance = 0.0001;

			double l_travelled_distance = 0.0;
			CPoint3 l_intersection_point;
			double l_dir_dot_triangle_normal = Dot(f_ray.GetDir(), GetNormal());
			if (fabs(l_dir_dot_triangle_normal) > VECTOR_ORTHOGONALITY_TOLERANCE)
			{
				double s_num = Get_d_Coefficient() - Dot(f_ray.GetPStart(), GetNormal());
				l_travelled_distance = s_num / l_dir_dot_triangle_normal;
				if (l_travelled_distance < -COINC_THRESH)
				{
					return false;
				}

				l_intersection_point = f_ray.GetPStart() + f_ray.GetDir() * l_travelled_distance;

				if (IsPointInTriangle(l_intersection_point, f_threshold))
				{

					if (l_dir_dot_triangle_normal >= 0.0)
					{
						l_exiting_the_model = true;
					}

					f_raycast_result.SetIntersectionPoint(l_intersection_point);
					f_raycast_result.SetTravelledDistance(l_travelled_distance);
					f_raycast_result.SetExitingModel(l_exiting_the_model);
					f_raycast_result.SetNormalAtIntersection(GetNormal());

					return true;
				}
			}
			else
			{
				CPoint3 l_ray_start_to_tri_vertex(f_ray.GetPStart() - GetVertex1());
				double l_plane_ray_distance = Dot(l_ray_start_to_tri_vertex, GetNormal());

				double l_distance_thresh = DOUBLE_PRECISION_EPSILON * l_crack_tolerance;

				if (fabs(l_plane_ray_distance) >= l_distance_thresh)
				{
					return false;
				}


			}


			return false;
		}

		/**
		* @brief Calculates the axis-aligned bounding box (AABB) for the triangle.
		*
		* @param f_ll_vert Output lower-left vertex of the bounding box.
		* @param f_ur_vert Output upper-right vertex of the bounding box.
		*/
		void CalculateTriangleBoundingBox(CPoint3& f_ll_vert,
			CPoint3& f_ur_vert) const
		{
			f_ll_vert.m_x = math_algs::FindMinOf3Numbers(m_vertex1->m_x, m_vertex2->m_x, m_vertex3->m_x);
			f_ll_vert.m_y = math_algs::FindMinOf3Numbers(m_vertex1->m_y, m_vertex2->m_y, m_vertex3->m_y);
			f_ll_vert.m_z = math_algs::FindMinOf3Numbers(m_vertex1->m_z, m_vertex2->m_z, m_vertex3->m_z);

			f_ur_vert.m_x = math_algs::FindMaxOf3Numbers(m_vertex1->m_x, m_vertex2->m_x, m_vertex3->m_x);
			f_ur_vert.m_y = math_algs::FindMaxOf3Numbers(m_vertex1->m_y, m_vertex2->m_y, m_vertex3->m_y);
			f_ur_vert.m_z = math_algs::FindMaxOf3Numbers(m_vertex1->m_z, m_vertex2->m_z, m_vertex3->m_z);

		}

		/**
		* @brief Checks whether the triangle intersects an axis-aligned bounding box.
		*
		* Uses the separating axis theorem (SAT): a triangle and a box are disjoint
		* if and only if their projections are disjoint on at least one of thirteen
		* candidate axes. The box is translated to the origin, so that it is
		* described by its half-extents h, and each candidate axis is tested by
		* comparing the triangle's projection interval with the box's projection
		* radius. The function returns false as soon as a separating axis is found.
		*
		* Axes are tested in this order (cheapest first):
		*   - the three box face normals (world X, Y, Z): the triangle's
		*     coordinates are compared directly with the half-extents;
		*   - the triangle face normal N = E0 x E1: all three vertices project to
		*     the same value, and the box radius is
		*     h.x*|N.x| + h.y*|N.y| + h.z*|N.z|;
		*   - the nine cross products of the triangle edges E0, E1, E2 with the
		*     world axes: the two endpoints of the edge project to the same value,
		*     so only the opposite vertex needs to be tested.
		*
		* The triangle and the box are treated as solids: the function returns true
		* when they overlap or touch. Degenerate cases (zero-area triangle, edges
		* parallel to a world axis) give zero-length axes, which never separate.
		*
		* @param[in] f_ll_bb_vert Minimum corner of the box.
		* @param[in] f_ur_bb_vert Maximum corner of the box. (The two corners may be
		*                         given in either order: centre and half-extents
		*                         are computed symmetrically.)
		* @param[in] f_threshold  Margin used by the separating axis tests for
		*                         numerical robustness.
		*
		* @return true if the triangle intersects or touches the box, false if a
		*         separating axis exists.
		*
		* @note The candidate axes are not normalized. On the box axes the
		*       projections are lengths, but on the face normal and on the edge
		*       cross products they are scaled by the length of the axis vector, so
		*       f_threshold does not have the same geometric meaning on every axis.
		*
		* @see D. Eberly, "Dynamic Collision Detection using Oriented Bounding
		*      Boxes", Geometric Tools, Section 3 (triangle vs OBB; here with
		*      world-aligned box axes).
		*/
		inline bool IsTriangleCollidingWithAABB(const CPoint3&f_ll_bb_vert,
												const CPoint3&f_ur_bb_vert,
												const double f_threshold = DOUBLE_PRECISION_EPSILON) const
		{
			const CPoint3 l_centre((f_ll_bb_vert.m_x + f_ur_bb_vert.m_x) / 2.0,
				(f_ll_bb_vert.m_y + f_ur_bb_vert.m_y) / 2.0,
				(f_ll_bb_vert.m_z + f_ur_bb_vert.m_z) / 2.0);

			const CPoint3 l_h(fabs((f_ur_bb_vert.m_x - f_ll_bb_vert.m_x) / 2.0),
				fabs((f_ur_bb_vert.m_y - f_ll_bb_vert.m_y) / 2.0),
				fabs((f_ur_bb_vert.m_z - f_ll_bb_vert.m_z) / 2.0));

			const CPoint3 l_a = GetVertex1() - l_centre;
			const CPoint3 l_b = GetVertex2() - l_centre;
			const CPoint3 l_c = GetVertex3() - l_centre;

			const CPoint3 l_e0 = l_b - l_a;
			const CPoint3 l_e1 = l_c - l_a;
			const CPoint3 l_e2 = l_c - l_b;

			for (int l_k = 0; l_k < 3; ++l_k)
			{
				if (IsSeparatingAxis(l_a.m_pc[l_k], l_b.m_pc[l_k], l_c.m_pc[l_k], l_h.m_pc[l_k], f_threshold))
				{
					return false;
				}
			}

			{
				const CPoint3 l_N = Cross(l_e0, l_e1);
				const double l_p0 = Dot(l_N, l_a);
				const double l_radius = l_h.m_x * fabs(l_N.m_x) + l_h.m_y * fabs(l_N.m_y) + l_h.m_z * fabs(l_N.m_z);

				if (IsSeparatingAxis(l_p0, l_p0, l_p0, l_radius, f_threshold))
				{
					return false;
				}
			}


			if (IsEdgeCrossSeparatingAxis(l_e0, l_a, l_c, l_h, 1, 2, f_threshold)) return false;
			if (IsEdgeCrossSeparatingAxis(l_e0, l_a, l_c, l_h, 2, 0, f_threshold)) return false;
			if (IsEdgeCrossSeparatingAxis(l_e0, l_a, l_c, l_h, 0, 1, f_threshold)) return false;

			if (IsEdgeCrossSeparatingAxis(l_e1, l_a, l_b, l_h, 1, 2, f_threshold)) return false;
			if (IsEdgeCrossSeparatingAxis(l_e1, l_a, l_b, l_h, 2, 0, f_threshold)) return false;
			if (IsEdgeCrossSeparatingAxis(l_e1, l_a, l_b, l_h, 0, 1, f_threshold)) return false;

			if (IsEdgeCrossSeparatingAxis(l_e2, l_b, l_a, l_h, 1, 2, f_threshold)) return false;
			if (IsEdgeCrossSeparatingAxis(l_e2, l_b, l_a, l_h, 2, 0, f_threshold)) return false;
			if (IsEdgeCrossSeparatingAxis(l_e2, l_b, l_a, l_h, 0, 1, f_threshold)) return false;

			return true;
		}
		/**
		* @brief Determines if the triangle is acute, right, or obtuse.
		*
		* This function calculates the dot products between vectors formed by
		* the triangle's vertices. If any dot product is negative, the corresponding
		* angle is obtuse (> 90°). If the dot product is near zero (within
		* a specified threshold), the angle is right (90°). Otherwise,
		* the triangle is acute (all angles < 90°).
		*
		* @param f_threshold Tolerance value used to determine if an angle is right-angled
		* @return STriangleAngleInfo Structure containing the triangle type and the vertex index
		*                            of the obtuse or right angle, if applicable.
		*/
		inline STriangleAngleInfo GetTriangleAnglesConfig(const double f_threshold = COINC_THRESH) const
		{
			CPoint3 l_12 = *m_vertex1 - *m_vertex2;
			CPoint3 l_13 = *m_vertex1 - *m_vertex3;
			CPoint3 l_21 = *m_vertex2 - *m_vertex1;
			CPoint3 l_23 = *m_vertex2 - *m_vertex3;
			CPoint3 l_31 = *m_vertex3 - *m_vertex1;
			CPoint3 l_32 = *m_vertex3 - *m_vertex2;

			const double l_dot_a = Dot(l_12, l_13);
			const double l_dot_b = Dot(l_21, l_23);
			const double l_dot_c = Dot(l_31, l_32);
			STriangleAngleInfo l_tri_info_res;

			if (std::abs(l_dot_a) < f_threshold)
			{
				l_tri_info_res.m_tri_type = ETriangleType::Right;
				l_tri_info_res.m_obtuse_or_right_angle = 1;
			}
			else if (l_dot_a < 0)
			{
				l_tri_info_res.m_tri_type = ETriangleType::Obtuse;
				l_tri_info_res.m_obtuse_or_right_angle = 1;
			}

			if (std::abs(l_dot_b) < f_threshold)
			{
				l_tri_info_res.m_tri_type = ETriangleType::Right;
				l_tri_info_res.m_obtuse_or_right_angle = 2;
			}
			else if (l_dot_b < 0)
			{
				l_tri_info_res.m_tri_type = ETriangleType::Obtuse;
				l_tri_info_res.m_obtuse_or_right_angle = 2;
			}

			if (std::abs(l_dot_c) < f_threshold)
			{
				l_tri_info_res.m_tri_type = ETriangleType::Right;
				l_tri_info_res.m_obtuse_or_right_angle = 3;
			}
			else if (l_dot_c < 0)
			{
				l_tri_info_res.m_tri_type = ETriangleType::Obtuse;
				l_tri_info_res.m_obtuse_or_right_angle = 3;
			}

			return l_tri_info_res;
		}
		/**
		* @brief Computes the center of the circumcircle (circumcenter) of the triangle.
		*
		* This method returns the 3D position of the circumcircle center of the triangle defined by
		* the three vertices of the triangle
		*
		* - For a non-degenerate triangle (vertices not collinear), the circumcenter is the
		*   intersection of the perpendicular bisectors of the triangle edges, computed in full 
		*   3D space.
		* - For a degenerate (collinear or nearly collinear) triangle, the circumcenter is defined
		*   as the midpoint of the longest edge formed by the triangle’s vertices.
		*
		* The computation is performed using vector algebra and is independent of the triangle's
		* orientation in 3D space.
		*
		* @return The 3D coordinates of the circumcircle center
		*
		* @note The degeneracy threshold is fixed at 1e-14 based on the squared length of the
		*       triangle's normal vector. It is possible to adjust this value if higher numerical
		*		robustness is required for very small or very large coordinate scales.
		*/
		inline CPoint3 GetCircumCircleCenter() const
		{
			const CPoint3& l_vert_1 = *m_vertex1;
			const CPoint3& l_vert_2 = *m_vertex2;
			const CPoint3& l_vert_3 = *m_vertex3;

			CPoint3 l_12_edge = l_vert_2 - l_vert_1;
			CPoint3 l_13_edge = l_vert_3 - l_vert_1;

			CPoint3 l_norm = Cross(l_12_edge, l_13_edge);
			double l_norm_length_sq = l_norm.LengthSq();

			if (l_norm_length_sq < 1e-14)
			{
				double l_12_len_sq = (l_vert_2 - l_vert_1).LengthSq();
				double l_13_len_sq = (l_vert_3 - l_vert_1).LengthSq();
				double l_23_len_sq = (l_vert_3 - l_vert_2).LengthSq();

				const CPoint3* l_p1 = &l_vert_1;
				const CPoint3* l_p2 = &l_vert_2;

				if (l_13_len_sq >= l_12_len_sq && l_13_len_sq >= l_23_len_sq)
				{
					l_p1 = &l_vert_1;
					l_p2 = &l_vert_3;
				}
				else if (l_23_len_sq >= l_12_len_sq && l_23_len_sq >= l_13_len_sq)
				{
					l_p1 = &l_vert_2;
					l_p2 = &l_vert_3;
				}

				return (*l_p1 + *l_p2) * 0.5;
			}

			double l_12_len_sq = l_12_edge.LengthSq();
			double l_13_len_sq = l_13_edge.LengthSq();

			CPoint3 l_term_1 = Cross(l_13_edge, l_norm) * l_12_len_sq;
			CPoint3 l_term_2 = Cross(l_norm, l_12_edge) * l_13_len_sq;

			CPoint3 l_center = l_vert_1 + (l_term_1 + l_term_2) / (2.0 * l_norm_length_sq);

			return l_center;
		}

		/**
		* @brief Computes the internal angle (in radians) at a specified vertex of the triangle.
		*
		* The triangle is assumed to have exactly three vertices indexed as:
		* - 0 -> Vertex 1
		* - 1 -> Vertex 2
		* - 2 -> Vertex 3
		*
		* The function forms two edge vectors originating from the selected vertex,
		* normalizes them, and computes the angle between them.
		*
		* @param f_vert Index of the vertex (0, 1, or 2).
		* @param f_angle_rad Computed angle at the vertex in radians.
		*
		* @return True if the vertex index is valid and the angle was computed;
		*         false if the vertex index is invalid.
		*/
		inline bool GetAngleAtVertexRad(const unsigned int f_vert, double& f_angle_rad) const
		{
			if (f_vert > 2)
			{
				return false;
			}

			const CPoint3 l_verts[3] = { GetVertex1(), GetVertex2(), GetVertex3() };

			const int l_idx1[3] = { 2, 0, 1 };
			const int l_idx2[3] = { 1, 2, 0 };

			CPoint3 l_side1 = l_verts[l_idx1[f_vert]] - l_verts[f_vert];
			CPoint3 l_side2 = l_verts[l_idx2[f_vert]] - l_verts[f_vert];
			l_side1.UnitizeInPlace();
			l_side2.UnitizeInPlace();
			f_angle_rad = cgeom_algs::AngleBetweenVectorsRad3D(l_side1, l_side2);
			return true;
		}

		/**
		* @brief Computes the internal angle (in degrees) at a specified vertex of the triangle.
		*
		* The triangle is assumed to have exactly three vertices indexed as:
		* - 0 -> Vertex 1
		* - 1 -> Vertex 2
		* - 2 -> Vertex 3
		*
		* The function forms two edge vectors originating from the selected vertex,
		* normalizes them, and computes the angle between them.
		*
		* @param f_vert Index of the vertex (0, 1, or 2).
		* @param f_angle_rad Computed angle at the vertex in degrees.
		*
		* @return True if the vertex index is valid and the angle was computed;
		*         false if the vertex index is invalid.
		*/
		inline bool GetAngleAtVertexDeg(const unsigned int f_vert, double& f_angle_deg) const
		{
			if (!GetAngleAtVertexRad(f_vert, f_angle_deg))
			{
				return false;
			}
			f_angle_deg = f_angle_deg * RAD_TO_DEG_FACTOR;
			return true;
		}

		/**
		* @brief Computes the internal angle (in radians) at a vertex specified by its position.
		*
		* This function determines which vertex of the triangle matches the provided position
		* and computes the angle at that vertex. It internally calls the index-based 
		* @ref GetAngleAtVertexRad
		*
		* @param f_vert_pos   Position of the vertex in 3D space.
		* @param f_angle_rad  Computed angle at the vertex in radians.
		* @param f_vert_id	  The ID of the vertex at which the angle is requested
		*
		* @return True if the vertex position matches one of the triangle's vertices
		*         and the angle was computed; false if no vertex matches.
		*
		* @note If no vertex matches f_vert_pos, f_angle_rad is set to -1.0, while f_vert_id to -1
		*/
		inline bool GetAngleAtVertexRad(const CPoint3& f_vert_pos, double& f_angle_rad, int& f_vert_id) const
		{
			if (fabs(Distance(*m_vertex1, f_vert_pos)) < COINC_THRESH)
			{
				f_vert_id = 0;
				return GetAngleAtVertexRad(0, f_angle_rad);
			}
			else if (fabs(Distance(*m_vertex2, f_vert_pos)) < COINC_THRESH)
			{
				f_vert_id = 1;
				return GetAngleAtVertexRad(1, f_angle_rad);
			}
			else if (fabs(Distance(*m_vertex3, f_vert_pos)) < COINC_THRESH)
			{
				f_vert_id = 2;
				return GetAngleAtVertexRad(2, f_angle_rad);
			}

			f_angle_rad = -1.0;
			return false;
		}

		/**
		* @brief Returns the vertex ID at a given local index within the triangle.
		*
		* Local indices map as follows:
		* - 0 -> vertex 1 (m_vert_1)
		* - 1 -> vertex 2 (m_vert_2)
		* - 2 -> vertex 3 (m_vert_3)
		*
		* This is primarily used during half-edge structure traversal, where the
		* destination vertex of a half-edge must be resolved from the face it
		* belongs to and the local slot of that half-edge within the face.
		*
		* @param f_local_index Local vertex index within the triangle (0, 1, or 2).
		* @return The vertex ID corresponding to the local index, or -1 if
		*         f_local_index is out of range.
		*/
		inline int GetVertexIdByLocalIndex(int f_local_index) const
		{
			switch (f_local_index)
			{
			case 0: return GetVert1ID();
			case 1: return GetVert2ID();
			case 2: return GetVert3ID();
			default: return -1;
			}
		}

		/**
		* @brief Return the (modifiable) pointer to the first half edge of the triangle
		*
		* @return Pointer to the first half edge of the triangle
		*/
		inline THalfEdge*& GetHalfEdgePtr()
		{
			return m_halfedge_ptr;
		}

	private:

		/**
		* @brief SAT helper: projects the triangle onto an axis and tests for separation.
		*
		* Given the three scalar projections of the triangle vertices onto a separating
		* axis candidate, and the corresponding projection radius of the AABB onto that
		* same axis, this function determines whether the two projected intervals are
		* disjoint. A disjoint result means the axis separates the two shapes and no
		* collision is possible.
		*
		* @param f_p0        Projection of triangle vertex 0 onto the axis.
		* @param f_p1        Projection of triangle vertex 1 onto the axis.
		* @param f_p2        Projection of triangle vertex 2 onto the axis.
		* @param f_radius    Projection radius of the AABB onto the axis.
		* @param f_threshold Optional tolerance / skin width added to the box radius.
		* @return true if the axis separates the triangle and the AABB, false otherwise.
		*/
		const bool IsSeparatingAxis(const double f_p0,
								    const double f_p1,
								    const double f_p2,
								    const double f_radius,
								    const double f_threshold) const
		{
			const double l_tri_min = std::min({ f_p0, f_p1, f_p2 });
			const double l_tri_max = std::max({ f_p0, f_p1, f_p2 });
			const double l_safe_radius = f_radius + f_threshold;
			return (l_tri_min > l_safe_radius) || (l_tri_max < -l_safe_radius);
		}

		/**
		* @brief SAT helper: tests one (edge x world-axis) separating axis candidate.
		*
		* For a given triangle edge and a world axis, the separating axis candidate L
		* is their cross product. Because L is perpendicular to the edge, the two
		* endpoints of the edge always project to the same scalar value; only the
		* opposite vertex produces a distinct projection. The AABB radius along L
		* is computed from the two half-extents not aligned with the crossed world axis.
		*
		* The projection formula is derived by expanding (edge x e_{j0}) . v, which
		* reduces to:  L . v = -edge[j2]*v[j1] + edge[j1]*v[j2]
		*
		* @param f_edge         Triangle edge vector (e.g. v1 - v0).
		* @param f_edge_vert    Either endpoint of the edge, translated to box-centre origin.
		* @param f_opp_vert     The triangle vertex not on this edge, translated to box-centre origin.
		* @param f_half_extents AABB half-extents.
		* @param f_j1           First component index not corresponding to the crossed world axis
		*                       (e.g. for edge x X: f_j1 = 1).
		* @param f_j2           Second component index not corresponding to the crossed world axis
		*                       (e.g. for edge x X: f_j2 = 2).
		* @param f_threshold    Optional tolerance / skin width.
		* @return true if this axis separates the triangle and the AABB, false otherwise.
		*/
		inline bool IsEdgeCrossSeparatingAxis(const CPoint3& f_edge,
								 		      const CPoint3& f_edge_vert,
										      const CPoint3& f_opp_vert,
										      const CPoint3& f_half_extents,
										      const int f_j1,
										      const int f_j2,
										      const double f_threshold) const
		{
			const double l_pe = -f_edge.m_pc[f_j2] * f_edge_vert.m_pc[f_j1] + f_edge.m_pc[f_j1] * f_edge_vert.m_pc[f_j2];
			const double l_po = -f_edge.m_pc[f_j2] * f_opp_vert.m_pc[f_j1] + f_edge.m_pc[f_j1] * f_opp_vert.m_pc[f_j2];
			const double l_r = f_half_extents.m_pc[f_j1] * fabs(f_edge.m_pc[f_j2]) + f_half_extents.m_pc[f_j2] * fabs(f_edge.m_pc[f_j1]);

			return IsSeparatingAxis(l_pe, l_pe, l_po, l_r, f_threshold);
		}

		CPoint3 m_normal;
		const CPoint3* m_vertex1;
		const CPoint3* m_vertex2;
		const CPoint3* m_vertex3;
		int m_triangle_number;
		int m_vert_1_id;
		int m_vert_2_id;
		int m_vert_3_id;
		double m_d_coefficient;
		THalfEdge* m_halfedge_ptr;
	};
}
#endif
