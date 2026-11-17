
#ifndef H_RAY_CYL_INTERS
#define H_RAY_CYL_INTERS

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "CRay.h"
#include "CCylinder.h"
#include "MathAlgorithms.h"


namespace ggeomlib
{
	/**
	* @class CRayCylinderIntersector
	* @brief Computes intersections between a ray and a cylinder in 3D space.
	*
	* This class provides a method to detect and characterize intersections between a given ray and
	* a cylinder. It returns detailed information such as whether an intersection occurs, whether
	* the ray is tangent to the cylinder, the number of intersection points, intersection parameters
	* along the ray, intersection points in 3D space, and surface normals at those points.
	*/
    class CRayCylinderIntersector
    {
    public:
		/**
		* @struct Result
		* @brief Stores the result of a ray-cylinder intersection test.
		*
		* Contains flags for intersection and tangency, the number of intersection points,
		* parametric distances along the ray, the intersection points in 3D space,
		* and the surface normals at those intersection points.
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
				m_points{ CPoint3(0.0,0.0,0.0), CPoint3(0.0,0.0,0.0) },
				m_normals{ CPoint3(0.0,0.0,0.0), CPoint3(0.0,0.0,0.0) }
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
		* @brief Performs intersection test between a ray and a cylinder.
		*
		* Given a ray and a cylinder, computes intersection results including points and normals.
		* If intersections exist, computes the actual 3D intersection points along the ray.
		*
		* @param f_ray		The ray to test for intersection.
		* @param f_cylinder The cylinder to test for intersection.
		* @return Result	containing intersection details.
		*/
		inline Result operator()(const CRay& f_ray, const CCylinder& f_cylinder)
		{
			Result l_res;
			PerformRayCylinderIntersection(f_ray.GetPStart(), f_ray.GetDir(), f_cylinder, l_res);
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
		* @brief Core method that performs the mathematical intersection calculation.
		*
		* The caller must ensure that 'result' is default-initialized before calling.
		* If intersections are found, 'result' is updated with all relevant intersection data.
		*
		* @param lineOrigin Origin point of the ray.
		* @param lineDirection Direction vector of the ray (assumed normalized).
		* @param f_cylinder The cylinder to intersect with.
		* @param result Output structure to hold intersection results.
		*/
		void PerformRayCylinderIntersection(const CPoint3& f_ray_origin,
										    const CPoint3& f_ray_direction,
											const CCylinder &f_cylinder,
											Result& f_result)
		{
			std::array<CPoint3, 3> basis{};
			basis[0] = f_cylinder.GetCylUnitAxis();
			ComputeOrthogonalComplement(1, basis.data());
			auto const& W = basis[0];
			auto const& U = basis[1];
			auto const& V = basis[2];
			double halfHeight = 0.5 * f_cylinder.GetCylLength();
			double rSqr = f_cylinder.GetCylRadius() * f_cylinder.GetCylRadius();

			const CPoint3 l_cyl_center = f_cylinder.GetCylCenter() + halfHeight * W;
			const CPoint3 diff = f_ray_origin - l_cyl_center;
			const CPoint3 P(Dot(U, diff), Dot(V, diff), Dot(W, diff));

			const double zero = 0.0;
			double dz = Dot(W, f_ray_direction);

			if (std::fabs(dz) == 1.0)
			{
				double radialSqrDist = rSqr - P[0] * P[0] - P[1] * P[1];
				if (radialSqrDist >= zero)
				{
					f_result.m_intersect = true;
					f_result.m_num_intersections = 2;
					if (dz > zero)
					{
						f_result.m_parameters[0] = -P[2] - halfHeight;
						f_result.m_parameters[1] = -P[2] + halfHeight;
						f_result.m_normals[0] = -W;
						f_result.m_normals[1] = W;
					}
					else
					{
						f_result.m_parameters[0] = P[2] - halfHeight;
						f_result.m_parameters[1] = P[2] + halfHeight;
						f_result.m_normals[0] = W;
						f_result.m_normals[1] = -W;
					}
				}
				return;
			}

			CPoint3 D(Dot(U, f_ray_direction), Dot(V, f_ray_direction), dz);
			if (D[2] == zero)
			{
				if (std::fabs(P[2]) <= halfHeight)
				{
					double a0 = P[0] * P[0] + P[1] * P[1] - rSqr;
					double a1 = P[0] * D[0] + P[1] * D[1];
					double a2 = D[0] * D[0] + D[1] * D[1];
					double discr = a1 * a1 - a0 * a2;
					if (discr > zero)
					{
						f_result.m_intersect = true;
						f_result.m_num_intersections = 2;
						double root = std::sqrt(discr);
						f_result.m_parameters[0] = (-a1 - root) / a2;
						f_result.m_parameters[1] = (-a1 + root) / a2;

						for (int i = 0; i < 2; ++i)
						{
							double t = f_result.m_parameters[i];
							CPoint3 point = f_ray_origin + t * f_ray_direction;
							CPoint3 local = point - l_cyl_center;
							double x = Dot(U, local);
							double y = Dot(V, local);
							f_result.m_normals[i] = (x * U + y * V).GetUnitVector();
						}
					}
					else if (discr == zero)
					{
						f_result.m_intersect = true;
						f_result.m_tangent = true;
						f_result.m_num_intersections = 1;
						f_result.m_parameters[0] = -a1 / a2;
						f_result.m_parameters[1] = f_result.m_parameters[0];

						double t = f_result.m_parameters[0];
						CPoint3 point = f_ray_origin + t * f_ray_direction;
						CPoint3 local = point - l_cyl_center;
						double x = Dot(U, local);
						double y = Dot(V, local);
						f_result.m_normals[0] = f_result.m_normals[1] = (x * U + y * V).GetUnitVector();
					}
				}
				return;
			}

			double t0 = (-halfHeight - P[2]) / D[2];
			double xTmp = P[0] + t0 * D[0];
			double yTmp = P[1] + t0 * D[1];
			if (xTmp * xTmp + yTmp * yTmp <= rSqr)
			{
				f_result.m_parameters[f_result.m_num_intersections] = t0;
				f_result.m_normals[f_result.m_num_intersections] = -W;
				++f_result.m_num_intersections;
			}

			double t1 = (+halfHeight - P[2]) / D[2];
			xTmp = P[0] + t1 * D[0];
			yTmp = P[1] + t1 * D[1];
			if (xTmp * xTmp + yTmp * yTmp <= rSqr)
			{
				f_result.m_parameters[f_result.m_num_intersections] = t1;
				f_result.m_normals[f_result.m_num_intersections] = W;
				++f_result.m_num_intersections;
			}

			if (f_result.m_num_intersections < 2)
			{
				double a0 = P[0] * P[0] + P[1] * P[1] - rSqr;
				double a1 = P[0] * D[0] + P[1] * D[1];
				double a2 = D[0] * D[0] + D[1] * D[1];
				double discr = a1 * a1 - a0 * a2;
				if (discr > zero)
				{
					double root = std::sqrt(discr);
					double tValue = (-a1 - root) / a2;
					if ((t0 <= t1 && t0 <= tValue && tValue <= t1) ||
						(t1 <= t0 && t1 <= tValue && tValue <= t0))
					{
						f_result.m_parameters[f_result.m_num_intersections] = tValue;
						CPoint3 point = f_ray_origin + tValue * f_ray_direction;
						CPoint3 local = point - l_cyl_center;
						double x = Dot(U, local);
						double y = Dot(V, local);
						f_result.m_normals[f_result.m_num_intersections] = (x * U + y * V).GetUnitVector();
						++f_result.m_num_intersections;
					}

					if (f_result.m_num_intersections < 2)
					{
						tValue = (-a1 + root) / a2;
						if ((t0 <= t1 && t0 <= tValue && tValue <= t1) ||
							(t1 <= t0 && t1 <= tValue && tValue <= t0))
						{
							f_result.m_parameters[f_result.m_num_intersections] = tValue;
							CPoint3 point = f_ray_origin + tValue * f_ray_direction;
							CPoint3 local = point - l_cyl_center;
							double x = Dot(U, local);
							double y = Dot(V, local);
							f_result.m_normals[f_result.m_num_intersections] = (x * U + y * V).GetUnitVector();
							++f_result.m_num_intersections;
						}
					}
				}
				else if (discr == zero)
				{
					double tValue = -a1 / a2;
					if ((t0 <= t1 && t0 <= tValue && tValue <= t1) ||
						(t1 <= t0 && t1 <= tValue && tValue <= t0))
					{
						f_result.m_parameters[f_result.m_num_intersections] = tValue;
						CPoint3 point = f_ray_origin + tValue * f_ray_direction;
						CPoint3 local = point - l_cyl_center;
						double x = Dot(U, local);
						double y = Dot(V, local);
						f_result.m_normals[f_result.m_num_intersections] = (x * U + y * V).GetUnitVector();
						++f_result.m_num_intersections;
					}
				}
			}

			if (f_result.m_num_intersections == 2)
			{
				f_result.m_intersect = true;
				if (f_result.m_parameters[0] > f_result.m_parameters[1])
				{
					std::swap(f_result.m_parameters[0], f_result.m_parameters[1]);
					std::swap(f_result.m_normals[0], f_result.m_normals[1]);
				}
			}
			else if (f_result.m_num_intersections == 1)
			{
				f_result.m_intersect = true;
				f_result.m_parameters[1] = f_result.m_parameters[0];
				f_result.m_normals[1] = f_result.m_normals[0];
			}
		}

		/**
		* @brief Normalizes a 3D vector in place and returns its original length.
		*
		* If the vector has zero length, it is set to the zero vector and 0 is
		* returned.
		*
		* When @p f_robust is true, the vector is first divided by its largest
		* absolute component before computing the length. This avoids overflow and
		* underflow in the sum of squares for vectors with very large or very small
		* components, at the cost of an extra division.
		*
		* @param[in,out] f_vec_to_normalize Vector to normalize; on return it has
		*                                   unit length, or is zero if the input was
		*                                   zero.
		* @param f_robust                   If true, use the robust (scaled)
		*                                   computation of the length.
		*
		* @return Length of the vector before normalization.
		*
		* @note Adapted from Normalize() in Vector.h of the Geometric Tools Engine by
		*       David Eberly (https://www.geometrictools.com), Boost Software License
		*       1.0.
		*/
		inline double Normalize(CPoint3& f_vec_to_normalize, 
								bool f_robust = false)
		{
			if (f_robust)
			{
				double l_max_abs_comp = std::fabs(f_vec_to_normalize[0]);
				for (int32_t i = 1; i < 3; ++i)
				{
					double l_abs_comp = std::fabs(f_vec_to_normalize[i]);
					if (l_abs_comp > l_max_abs_comp)
					{
						l_max_abs_comp = l_abs_comp;
					}
				}

				double l_length = 0.0;
				if (l_max_abs_comp > 0.0)
				{
					f_vec_to_normalize = f_vec_to_normalize / l_max_abs_comp;
					l_length = std::sqrt(Dot(f_vec_to_normalize, f_vec_to_normalize));
					f_vec_to_normalize = f_vec_to_normalize / l_length;
					l_length *= l_max_abs_comp;
				}
				else
				{
					l_length = 0.0;
					for (int32_t i = 0; i < 3; ++i)
					{
						f_vec_to_normalize[i] = 0.0;
					}
				}
				return l_length;
			}
			else
			{
				double l_length = std::sqrt(Dot(f_vec_to_normalize, f_vec_to_normalize));
				if (l_length > 0.0)
				{
					f_vec_to_normalize = f_vec_to_normalize / l_length;
				}
				else
				{
					for (int32_t i = 0; i < 3; ++i)
					{
						f_vec_to_normalize[i] = 0.0;
					}
				}
				return l_length;
			}
		}

		/**
		* @brief Orthonormalizes 1 to 3 3D vectors in place using Gram-Schmidt.
		*
		* The first vector is normalized; each following vector is made orthogonal
		* to all the previous ones by subtracting its projections onto them, and is
		* then normalized.
		*
		* The return value can be used to detect nearly linearly dependent inputs:
		* a value close to zero means that at least one vector was (nearly) a linear
		* combination of the previous ones, so the result is not reliable.
		*
		* @param f_num_input_vecs        Number of vectors to orthonormalize (1 to 3).
		* @param[in,out] f_vecs_to_normalize Array of at least @p f_num_input_vecs
		*                                vectors, replaced by the orthonormal vectors.
		* @param f_robust                If true, use the robust normalization (see
		*                                Normalize()).
		*
		* @return Minimum length of the vectors before their normalization, or 0 if
		*         @p f_vecs_to_normalize is null or @p f_num_input_vecs is not in
		*         [1, 3] (in which case the vectors are left unchanged).
		*
		* @note Adapted from Orthonormalize() in Vector.h of the Geometric Tools Engine
		*       by David Eberly (https://www.geometrictools.com), Boost Software
		*       License 1.0.
		*/
		inline double Orthonormalize(int32_t f_num_input_vecs,
									 CPoint3* f_vecs_to_normalize,
									 bool f_robust = false)
		{
			if (f_vecs_to_normalize && 1 <= f_num_input_vecs && f_num_input_vecs <= 3)
			{
				double l_min_len = Normalize(f_vecs_to_normalize[0], f_robust);
				for (int32_t i = 1; i < f_num_input_vecs; ++i)
				{
					for (int32_t j = 0; j < i; ++j)
					{
						double dot = Dot(f_vecs_to_normalize[i], f_vecs_to_normalize[j]);
						f_vecs_to_normalize[i] = f_vecs_to_normalize[i] - f_vecs_to_normalize[j] * dot;
					}
					double l_len = Normalize(f_vecs_to_normalize[i], f_robust);
					if (l_len < l_min_len)
					{
						l_min_len = l_len;
					}
				}
				return l_min_len;
			}
			return 0.0;
		}

		/**
		* @brief Completes 1 or 2 3D vectors to a right-handed orthonormal basis.
		*
		* - With 1 input vector, a second vector orthogonal to it is built by
		*   zeroing one component and swapping the other two (choosing the
		*   combination that avoids a near-zero result), and the third is computed
		*   as the cross product of the first two.
		* - With 2 input vectors, the third is computed as their cross product.
		*
		* The three vectors are then orthonormalized with Orthonormalize(), so the
		* input vectors do not need to be normalized (or, with 2 inputs, exactly
		* orthogonal) beforehand.
		*
		* @param f_num_input_vecs    Number of input vectors (1 or 2).
		* @param[in,out] f_input_vecs Array of at least 3 vectors: the first
		*                            @p f_num_input_vecs are the inputs; on return
		*                            the 3 vectors form an orthonormal basis.
		* @param f_robust            If true, use the robust normalization (see
		*                            Normalize()).
		*
		* @return The value returned by Orthonormalize() (minimum length before
		*         normalization), or 0 if @p f_num_input_vecs is neither 1 nor 2 (in
		*         which case the array is left unchanged).
		*
		* @warning @p f_input_vecs must have room for 3 vectors, even when only 1 or
		*          2 are given as input.
		*
		* @note Adapted from ComputeOrthogonalComplement() in Vector3.h of the
		*       Geometric Tools Engine by David Eberly (https://www.geometrictools.com),
		*       Boost Software License 1.0.
		*/
		inline double ComputeOrthogonalComplement(int32_t f_num_input_vecs,
												  CPoint3* f_input_vecs,
												  bool f_robust = false)
		{
			if (f_num_input_vecs == 1)
			{
				if (std::fabs(f_input_vecs[0][0]) > std::fabs(f_input_vecs[0][1]))
				{
					f_input_vecs[1] = { -f_input_vecs[0][2], 0.0, +f_input_vecs[0][0] };
				}
				else
				{
					f_input_vecs[1] = { 0.0, +f_input_vecs[0][2], -f_input_vecs[0][1] };
				}
				f_num_input_vecs = 2;
			}

			if (f_num_input_vecs == 2)
			{
				f_input_vecs[2] = Cross(f_input_vecs[0], f_input_vecs[1]);
				return Orthonormalize(3, f_input_vecs, f_robust);
			}

			return 0.0;
		}
	};
}

#endif