
#ifndef H_RAY_SPHERE_INTERS
#define H_RAY_SPHERE_INTERS

#pragma once

#include <array>
#include <cmath>

#include "CPoint3.h"
#include "CRay.h"

namespace ggeomlib
{
	/**
	* @class CRaySphereIntersector
	* @brief Computes intersections between a ray and a sphere in 3D space.
	*
	* This class provides a method to test if a ray intersects with a sphere and returns detailed
	* intersection information such as the number of intersection points, intersection parameters
	* along the ray, the actual intersection points, and the normals at those points.
	*/
	class CRaySphereIntersector
	{
	public:

		/**
		* @struct Result
		* @brief Stores the result of a ray-sphere intersection test.
		*
		* Contains flags indicating whether the ray intersects or is tangent to the sphere, the
		* number of intersection points (0, 1, or 2), parametric values along the ray for those
		* intersections, the 3D coordinates of the intersection points, and the surface normals at
		* those points.
		*/
		struct Result
		{
			/**
			* @brief Default constructor initializing all members to default no-intersection states.
			*/
			Result()
				:
				m_intersect(false),
				m_tangent(false),
				m_num_intersections(0),
				m_parameters{ 0.0, 0.0 },
				m_points{ CPoint3(), CPoint3() },
				m_normals{ CPoint3(), CPoint3() }  
			{
			}

			bool m_intersect;
			bool m_tangent;
			size_t m_num_intersections;
			std::array<double, 2> m_parameters;
			std::array<CPoint3, 2> m_points;
			std::array<CPoint3, 2> m_normals;
		};

		/**
		* @brief Performs intersection test between a ray and a sphere.
		*
		* Computes the intersections (if any) between the input ray and a sphere defined by its center
		* and radius. Returns detailed intersection information, including points and normals.
		*
		* @param f_ray The ray to test for intersection.
		* @param f_sphere_center The center of the sphere.
		* @param f_sphere_radius The radius of the sphere.
		* @return Result containing intersection details.
		*/
		inline Result operator()(const CRay& f_ray,
						  const CPoint3& f_sphere_center,
						  const double f_sphere_radius)
		{
			Result l_res;
			PerformSphereRayIntersection(f_ray.GetPStart(), f_ray.GetDir(), f_sphere_center, f_sphere_radius, l_res);
			if (l_res.m_intersect)
			{
				for (size_t i = 0; i < 2; ++i)
				{
					l_res.m_points[i] = f_ray.GetPStart() + l_res.m_parameters[i] * f_ray.GetDir();
				}
			}
			return l_res;
		}

	protected:
		/**
		* @brief Core method that performs the mathematical ray-sphere intersection calculation.
		*
		* The caller must ensure that 'result' is default-initialized before calling.
		* If intersections are found, 'result' is updated with the number of intersections,
		* parameters along the ray, intersection points, and surface normals.
		*
		* @param lineOrigin Origin point of the ray.
		* @param lineDirection Direction vector of the ray (assumed normalized).
		* @param f_sphere_center Center of the sphere.
		* @param f_sphere_radius Radius of the sphere.
		* @param result Output structure to hold intersection results.
		*/
		inline void PerformSphereRayIntersection(const CPoint3& lineOrigin,
										  const CPoint3& lineDirection,
										  const CPoint3& f_sphere_center,
										  const double f_sphere_radius,
										  Result& result)
		{
			const CPoint3 diff = lineOrigin - f_sphere_center;
			double a0 = Dot(diff, diff) - f_sphere_radius * f_sphere_radius;
			double a1 = Dot(lineDirection, diff);

			double const zero = 0.0;
			double discr = a1 * a1 - a0;
			if (discr > zero)
			{
				result.m_intersect = true;
				result.m_num_intersections = 2;
				double root = std::sqrt(discr);
				result.m_parameters[0] = -a1 - root;
				result.m_parameters[1] = -a1 + root;

				for (int i = 0; i < 2; ++i)
				{
					CPoint3 point = lineOrigin + result.m_parameters[i] * lineDirection;
					CPoint3 normal = (point - f_sphere_center).GetUnitVector();
					result.m_normals[i] = normal;
				}
			}
			else if (discr == zero)
			{
				result.m_intersect = true;
				result.m_tangent = true;
				result.m_num_intersections = 1;
				result.m_parameters[0] = -a1;
				result.m_parameters[1] = result.m_parameters[0];

				CPoint3 point = lineOrigin + result.m_parameters[0] * lineDirection;
				CPoint3 normal = (point - f_sphere_center).GetUnitVector();
				result.m_normals[0] = normal;
				result.m_normals[1] = normal;
			}
		}
	};
}

#endif