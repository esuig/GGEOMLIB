
#ifndef H_CPLANE
#define H_CPLANE

#pragma once

#include "CPoint3.h"
#include "CRay.h"

namespace ggeomlib
{
	/**
	 * @class CPlane
	 * @brief Represents a mathematical plane in 3D space defined by its normal vector and offset.
	 *
	 * The plane is represented in the form:
	 *     **n · x + d = 0**,
	 * where `n` is the normal vector and `d` is the fourth (offset) coefficient.
	 */
	class CPlane
	{
	public:
		/**
		* @brief Constructs a plane from a normal vector and a fourth coefficient.
		*
		* This constructor assumes the plane equation is already in the form n · x + d = 0.
		*
		* @param f_normal   Normal vector to the plane (does not need to be unit length).
		* @param f_d_coeff	The scalar offset `d` in the plane equation.
		*/
		CPlane(const CPoint3& f_normal,
			   double f_d_coeff):
					m_normal(f_normal),
					m_d_coeff(f_d_coeff)
		{
			CalculatePointOnPlane();
		}
		/**
		* @brief Constructs a plane given a normal vector and a point lying on the plane.
		*
		* The fourth coefficient is automatically computed as `d = -n · p`.
		*
		* @param f_normal  Normal vector to the plane (does not need to be unit length).
		* @param f_p       A point lying on the plane.
		*/
		CPlane(const CPoint3& f_normal,
			   const CPoint3& f_p) :
					m_normal(f_normal),
					m_d_coeff(Dot(f_normal, f_p)),
					m_point_on_plane(f_p)
		{

		}

		/**
		* @brief Returns the normal vector of the plane.
		*
		* @return A constant reference to the plane's normal vector.
		*/
		inline const CPoint3& GetNormal() const
		{
			return m_normal;
		}
		/**
		* @brief Returns the fourth coefficient (offset) of the plane.
		*
		* @return The scalar offset `d` in the plane equation.
		*/
		inline double GetDCoeff() const
		{
			return m_d_coeff;
		}

		/**
		* @brief Computes the signed distance from a point to the plane.
		*
		* The result is positive if the point is in the direction of the normal, negative otherwise.
		*
		* @param f_point A 3D point.
		* @return Signed distance from the point to the plane.
		*/
		inline double DistanceFromPoint(const CPoint3& f_point) const
		{
			return Dot(m_normal, f_point) + m_d_coeff;
		}

		/**
		* @brief Projects a point onto the plane. Given a 3D point, this method computes its orthogonal 
		* projection onto the plane
		*
		* @param[in]  f_point    The point in 3D space to project onto the plane.
		* @param[out] f_proj_pt  The resulting projected point on the plane.
		*
		* @return true if the projection was successful, false if the normal vector is zero (invalid plane).
		*/
		inline bool FindProjectionOfPointOnPlane(const CPoint3&f_point, CPoint3&f_proj_pt) const
		{
			CPoint3 l_diff = f_point - m_point_on_plane;
			double n_dot_n = Dot(m_normal, m_normal);

			if (n_dot_n == 0.0)
			{
				return false;
			}

			double l_proj_length = Dot(l_diff, m_normal) / n_dot_n;
			CPoint3 l_proj_vec = l_proj_length * m_normal;
			f_proj_pt = f_point - l_proj_vec;

			return true;
		}

		/**
		* @brief Computes the intersection point between the plane and a ray, if it exists.
		*
		* This method calculates the point of intersection between the current plane and a given ray.
		* If the ray intersects the plane at a point in the direction of the ray (i.e., for t >= 0),
		* the function returns true and sets `f_int_pt` to the intersection point.
		*
		* The ray is defined as:
		*     R(t) = o + t·d
		* where `o` is the ray origin and `d` is its direction.
		*
		* The plane is defined as:
		*     n · x + d = 0
		* where `n` is the plane normal and `d` is the offset (`m_d_coeff`).
		*
		* The intersection is computed by solving for `t`:
		*     t = -(n · o + d) / (n · d)
		*
		* @param[in]  f_ray     The ray to test for intersection with the plane.
		* @param[out] f_int_pt  If an intersection exists, this will be set to the point of intersection.
		* @param[out] f_t_param	If the ray intersects with the plane, then this will contain the ray parameter
		*						at which intersection happens
		*
		* @return				`true` if the ray intersects the plane in the positive direction (t >= 0);
		*						`false` if the ray is parallel to the plane or the intersection lies behind the ray origin.
		*/
		inline bool IntersectWithRay(const CRay& f_ray,
								     CPoint3& f_int_pt,
								     double& f_t_param) const
		{
			f_t_param = 0.0;
			f_int_pt = CPoint3();
			const CPoint3& l_plane_norm = GetNormal();
			const double l_denom = Dot(l_plane_norm, f_ray.GetDir());

			if (std::abs(l_denom) < VECTOR_PARALLELISM_TOLERANCE)
			{
				return false;
			}

			f_t_param = (Dot(l_plane_norm, f_ray.GetPStart()) + GetDCoeff()) / l_denom;

			if (f_t_param < 0)
			{
				return false;
			}

			f_int_pt = f_ray.GetPStart() + f_ray.GetDir() * f_t_param;
			return true;
		}
		/*
		* @brief Return a point on the plane
		* 
		* @return A point on the plane
		*/
		inline const CPoint3& GetPointOnPlane() const
		{
			return m_point_on_plane;
		}

	private:

		/**
		* @brief Calculate a point on the plane given the plane params
		*/
		inline void CalculatePointOnPlane()
		{
			const double A = m_normal.m_x;
			const double B = m_normal.m_y;
			const double C = m_normal.m_z;
			const double D = m_d_coeff;

			if (C != 0.0)
			{
				double z = D / C;
				m_point_on_plane = CPoint3(0.0, 0.0, z);
				return;
			}
			else if (B != 0.0)
			{
				double y = D / B;
				m_point_on_plane = CPoint3(0.0, y, 0.0);
				return;
			}
			else if (A != 0.0)
			{
				double x = D / A;
				m_point_on_plane = CPoint3(x, 0.0, 0.0);
				return;
			}

			m_point_on_plane = CPoint3(0.0, 0.0, 0.0);
		}

		CPoint3 m_normal;
		double m_d_coeff;
		CPoint3 m_point_on_plane;
	};
}

#endif