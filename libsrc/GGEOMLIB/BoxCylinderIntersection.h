

#ifndef H_BOX_CYL_INTERS
#define H_BOX_CYL_INTERS

#pragma once

#include "CCylinder.h"
#include "CPoint3.h"
#include "MathAlgorithms.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace ggeomlib
{
	/**
	* @class CCylBoxIntersector
	* @brief Class to determine intersection between a cylinder and an axis-aligned bounding box.
	*/
	class CCylBoxIntersector
	{
	public:

		/**
		* @brief Tests whether a cylinder intersects with an axis-aligned bounding box (AABB).
		*
		* The AABB is defined by its lower-left vertex (`f_ll_box_vert`) and upper-right vertex (`f_ur_box_vert`).
		* The cylinder is defined by its center, unit axis direction, length, and radius.
		*
		* The method works by:
		* - Translating the cylinder to be relative to the box center (assuming the box is centered at the origin).
		* - Checking if the box is outside the infinite slab defined by the cylinder's end disks.
		* - Reflecting the cylinder's axis vector so that all components are non-negative, simplifying intersection tests.
		* - Using different helper query functions (`DoQueryNoZeros`, `DoQueryOneZero`, `DoQueryTwoZeros`) depending on
		*   which components of the cylinder axis direction are zero or non-zero to perform precise intersection tests.
		*
		* @param f_cylinder The cylinder to test for intersection.
		* @param f_ll_box_vert The lower-left corner vertex of the axis-aligned bounding box.
		* @param f_ur_box_vert The upper-right corner vertex of the axis-aligned bounding box.
		* @return true if the cylinder intersects the box, false otherwise.
		*/
		inline bool operator()(const CCylinder &f_cylinder,
							   const CPoint3& f_ll_box_vert,
							   const CPoint3& f_ur_box_vert)
		{
			CPoint3 E = (f_ur_box_vert - f_ll_box_vert) / 2.0;

			CPoint3 l_box_center = f_ll_box_vert + E;
			CCylinder l_translated_cylinder((f_cylinder.GetCylCenter() - l_box_center) +
											((f_cylinder.GetCylLength() / 2.0) * f_cylinder.GetCylUnitAxis()),
											f_cylinder.GetCylUnitAxis(),
											f_cylinder.GetCylLength(),
											f_cylinder.GetCylRadius());

			if (BoxIsOutsideCylinderSlab(E, l_translated_cylinder))
			{
				return false;
			}

			const double l_zero = COINC_THRESH;
			CPoint3 C = l_translated_cylinder.GetCylCenter();
			CPoint3 D = l_translated_cylinder.GetCylUnitAxis();
			const double r = l_translated_cylinder.GetCylRadius();
			const double hDiv2 = (0.5) * l_translated_cylinder.GetCylLength();
			for (int32_t i = 0; i < 3; ++i)
			{
				if (D[i] < l_zero)
				{
					C[i] = -C[i];
					D[i] = -D[i];
				}
			}

			bool l_intersection = false;

			if (D[0] > l_zero)
			{
				if (D[1] > l_zero)
				{
					if (D[2] > l_zero)
					{
						l_intersection = DoQueryNoZeros(C, D, r, hDiv2, E);
					}
					else
					{
						l_intersection = DoQueryOneZero({ 0, 1, 2 }, C, D, r, hDiv2, E);
					}
				}
				else
				{
					if (D[2] > l_zero)
					{
						l_intersection = DoQueryOneZero({ 2, 0, 1 }, C, D, r, hDiv2, E);
					}
					else
					{
						l_intersection = DoQueryTwoZeros({ 0, 1, 2 }, C, r, E);
					}
				}
			}
			else
			{
				if (D[1] > l_zero)
				{
					if (D[2] > l_zero)
					{
						l_intersection = DoQueryOneZero({ 1, 2, 0 }, C, D, r, hDiv2, E);
					}
					else
					{
						l_intersection = DoQueryTwoZeros({ 1, 2, 0 }, C, r, E);
					}
				}
				else
				{
					if (D[2] > l_zero)
					{
						l_intersection = DoQueryTwoZeros({ 2, 0, 1 }, C, r, E);
					}
				}
			}

			return l_intersection;
		}

	private:
		/**
		* @brief Checks if the bounding box is completely outside the slab defined by the cylinder's end disk planes.
		* 		 Test whether the box is outside the slab contained by the planes of the cylinder end disks. This is 
		*		 accomplished by computing the interval of projection of the box onto the cylinder axis.
		*
		* This test projects the box onto the cylinder's axis and compares intervals to determine if the box lies
		* outside the cylindrical slab.
		*
		* @param f_box_extent The extent (half-dimensions) of the box along each axis.
		* @param f_cylinder The cylinder for which the slab is defined.
		* @return true if the box lies completely outside the slab, false otherwise.
		*/
		inline bool BoxIsOutsideCylinderSlab(const CPoint3& f_box_extent,
									  const CCylinder& f_cylinder)
		{
			const CPoint3& C = f_cylinder.GetCylCenter();
			const CPoint3& D = f_cylinder.GetCylUnitAxis();
			CPoint3 absD(std::fabs(D.m_x), std::fabs(D.m_y), std::fabs(D.m_z));
			double hDiv2 = 0.5 * f_cylinder.GetCylLength();

			double intervalCenter = -Dot(D, C);
			double intervalRadius = Dot(f_box_extent, absD);
			if (std::fabs(intervalCenter) > intervalRadius + hDiv2)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		/**
		* @brief Checks whether the cylinder's axis intersects the bounding box in 3D space.
		*
		* @param C The center point of the cylinder axis.
		* @param D The direction vector of the cylinder axis.
		* @param hDiv2 Half the height of the cylinder.
		* @param E The extent (half-size) of the bounding box.
		* @return true if the cylinder's axis intersects the box, false otherwise.
		*/
		inline bool CylinderAxisIntersectsBox3D(const CPoint3& C,
											    const CPoint3& D,
											    const double hDiv2,
											    const CPoint3& E)
		{
			CPoint3 negEmCDivD((-E[0] - C[0]) / D[0], (-E[1] - C[1]) / D[1], (-E[2] - C[2]) / D[2]);
			CPoint3 posEmCDivD((E[0] - C[0]) / D[0], (E[1] - C[1]) / D[1], (E[2] - C[2]) / D[2]);

			double max01 = std::max(negEmCDivD[0], negEmCDivD[1]);
			double max23 = std::max(negEmCDivD[2], -hDiv2);
			double lower = std::max(max01, max23);
			double min01 = std::min(posEmCDivD[0], posEmCDivD[1]);
			double min23 = std::min(posEmCDivD[2], hDiv2);
			double upper = std::min(min01, min23);
			return lower <= upper;
		}
		/**
		* @brief Computes the squared distance from the origin to the projection of a line segment onto a plane.
		*
		* The projection plane is defined by its origin `C` and orthonormal basis vectors `W0` and `W1`.
		*
		* @param P0 The first endpoint of the segment.
		* @param P1 The second endpoint of the segment.
		* @param C The origin of the projection plane.
		* @param W0 The first orthonormal vector spanning the projection plane.
		* @param W1 The second orthonormal vector spanning the projection plane.
		* @return The squared distance from (0,0) to the projected segment.
		*/
		double ComputeSqrDistance(const CPoint3& P0,
								  const CPoint3& P1,
								  const CPoint3& C,
								  const CPoint3& W0,
								  const CPoint3& W1)
		{
			CPoint3 P0mC = P0 - C;
			CPoint3 P1mC = P1 - C;
			CPoint2 Q0(Dot(W0, P0mC), Dot(W1, P0mC));
			CPoint2 Q1(Dot(W0, P1mC), Dot(W1, P1mC));

			const double l_zero = 0.0;
			CPoint2 direction = Q1 - Q0;
			double s = Dot(direction, Q1);
			if (s <= l_zero)
			{
				return Dot(Q1, Q1);
			}
			else
			{
				s = Dot(direction, Q0);
				if (s >= l_zero)
				{
					return Dot(Q0, Q0);
				}
				else
				{
					s /= Dot(direction, direction);
					CPoint2 closest = Q0 - s * direction;
					return Dot(closest, closest);
				}
			}
		}
		/**
		* @brief Handles the intersection query for the case where exactly two components of the intersection test are zero.
		*
		* @param i An array representing the zero components.
		* @param C The center point used in the query.
		* @param r The radius of the cylinder.
		* @param E The box extent.
		* @return true if intersection detected, false otherwise.
		*/
		bool DoQueryTwoZeros(std::array<int32_t, 3> const& i,
							 const CPoint3& C,
							 const double r,
							 const CPoint3& E)
		{
			double absC1 = std::fabs(C[i[1]]), absC2 = std::fabs(C[i[2]]);
			double E1 = E[i[1]], E2 = E[i[2]];

			if (absC1 <= E1 && absC2 <= E2)
			{
				return true;
			}

			const double l_zero = 0.0;
			double sqrDistance = l_zero;
			double delta = absC1 - E1;
			if (delta > l_zero)
			{
				sqrDistance += delta * delta;
			}

			delta = absC2 - E2;
			if (delta > l_zero)
			{
				sqrDistance += delta * delta;
			}

			return sqrDistance <= r * r;
		}

		/**
		* @brief Handles the intersection query for the case where exactly one component of the intersection test is zero.
		*
		* @param i An array representing the zero component.
		* @param C The center point used in the query.
		* @param D The direction vector of the cylinder axis.
		* @param r The radius of the cylinder.
		* @param hDiv2 Half the height of the cylinder.
		* @param E The box extent.
		* @return true if intersection detected, false otherwise.
		*/
		bool DoQueryOneZero(std::array<int32_t, 3> const& i,
							const CPoint3& C,
							const CPoint3& D,
							const double r, 
							const double hDiv2,
							const CPoint3& E)
		{
			const double c0 = C[i[0]];
			const double c1 = C[i[1]];
			const double c2 = C[i[2]];
			const double d0 = D[i[0]];
			const double d1 = D[i[1]];
			const double e0 = E[i[0]];
			const double e1 = E[i[1]];
			const double e2 = E[i[2]];
			double e0pc0 = e0 + c0, e0mc0 = e0 - c0, e1pc1 = e1 + c1, e1mc1 = e1 - c1;

			double absC2 = std::fabs(c2);
			if (absC2 <= e2)
			{
				std::array<double, 2> negEmCDivD{ -e0pc0 / d0, -e1pc1 / d1 };
				std::array<double, 2> posEmCDivD{ e0mc0 / d0, e1mc1 / d1 };
				double lower = std::max(std::max(negEmCDivD[0], negEmCDivD[1]), -hDiv2);
				double upper = std::min(std::min(posEmCDivD[0], posEmCDivD[1]), hDiv2);
				if (lower <= upper)
				{
					return true;
				}
			}

			const double l_zero = 0.0;
			double sMin = l_zero;
			double tHat = d1 * e1mc1 - d0 * e0pc0;
			if (-hDiv2 <= tHat)
			{
				if (tHat <= hDiv2)
				{
					sMin = -(d0 * e1mc1 + d1 * e0pc0);
				}
				else
				{
					sMin = -(e0pc0 + d0 * hDiv2) / d1;
				}
			}
			else
			{
				sMin = -(e1mc1 + d1 * hDiv2) / d0;
			}

			double sMax = l_zero;
			double tBar = d0 * e0mc0 - d1 * e1pc1;
			if (-hDiv2 <= tBar)
			{
				if (tBar <= hDiv2)
				{
					sMax = d0 * e1pc1 + d1 * e0mc0;
				}
				else
				{
					sMax = (e1pc1 + d1 * hDiv2) / d0;
				}
			}
			else
			{
				sMax = (e0mc0 + d0 * hDiv2) / d1;
			}

			double sqrDistance = l_zero;
			if (l_zero < sMin)
			{
				sqrDistance += sMin * sMin;
			}
			else if (sMax < l_zero)
			{
				sqrDistance += sMax * sMax;
			}

			double delta = absC2 - e2;
			if (delta > l_zero)
			{
				sqrDistance += delta * delta;
			}

			return sqrDistance <= r * r;
		}
		/**
		* @brief Handles the intersection query for the case where no components of the intersection test are zero.
		*
		* @param C The center point used in the query.
		* @param D The direction vector of the cylinder axis.
		* @param r The radius of the cylinder.
		* @param hDiv2 Half the height of the cylinder.
		* @param E The box extent.
		* @return true if intersection detected, false otherwise.
		*/
		bool DoQueryNoZeros(const CPoint3& C,
							const CPoint3& D,
							const double r,
							const double hDiv2,
							const CPoint3& E)
		{
			std::array<double, 3> negEmCDivD
			{
				(-E[0] - C[0]) / D[0],
				(-E[1] - C[1]) / D[1],
				(-E[2] - C[2]) / D[2]
			};

			std::array<double, 3> posEmCDivD
			{
				(E[0] - C[0]) / D[0],
				(E[1] - C[1]) / D[1],
				(E[2] - C[2]) / D[2]
			};

			double max01 = std::max(negEmCDivD[0], negEmCDivD[1]);
			double max23 = std::max(negEmCDivD[2], -hDiv2);
			double lower = std::max(max01, max23);
			double min01 = std::min(posEmCDivD[0], posEmCDivD[1]);
			double min23 = std::min(posEmCDivD[2], hDiv2);
			double upper = std::min(min01, min23);
			if (lower <= upper)
			{
				return true;
			}

			double dotDC = Dot(D, C);
			double d0e0 = D[0] * E[0], d1e1 = D[1] * E[1], d2e2 = D[2] * E[2];
			double t1 = +d0e0 - d1e1 - d2e2 - dotDC, s1p = t1 + hDiv2, s1n = t1 - hDiv2;
			double t2 = -d0e0 + d1e1 - d2e2 - dotDC, s2p = t2 + hDiv2, s2n = t2 - hDiv2;
			double t3 = +d0e0 + d1e1 - d2e2 - dotDC, s3p = t3 + hDiv2, s3n = t3 - hDiv2;
			double t4 = -d0e0 - d1e1 + d2e2 - dotDC, s4p = t4 + hDiv2, s4n = t4 - hDiv2;
			double t5 = +d0e0 - d1e1 + d2e2 - dotDC, s5p = t5 + hDiv2, s5n = t5 - hDiv2;
			double t6 = -d0e0 + d1e1 + d2e2 - dotDC, s6p = t6 + hDiv2, s6n = t6 - hDiv2;

			std::array<CPoint3, 3> basis{};
			basis[0] = D;
			ComputeOrthogonalComplement(1, basis.data());
			auto const& W0 = basis[1];
			auto const& W1 = basis[2];

			const double l_zero = 0.0;
			double sqrRadius = r * r;
			double sqrDistance{};
			CPoint3 P0;
			CPoint3 P1;

			lower = (s1p >= l_zero ? -E[2] : -E[2] - s1p / D[2]);
			upper = (s5n <= l_zero ? +E[2] : +E[2] - s5n / D[2]);
			if (lower <= upper)
			{
				P0 = { +E[0], -E[1], lower };
				P1 = { +E[0], -E[1], upper };
				sqrDistance = ComputeSqrDistance(P0, P1, C, W0, W1);
				if (sqrDistance <= sqrRadius)
				{
					return true;
				}
			}

			lower = (s2p >= l_zero ? -E[2] : -E[2] - s2p / D[2]);
			upper = (s6n <= l_zero ? +E[2] : +E[2] - s6n / D[2]);
			if (lower <= upper)
			{
				P0 = { -E[0], +E[1], lower };
				P1 = { -E[0], +E[1], upper };
				sqrDistance = ComputeSqrDistance(P0, P1, C, W0, W1);
				if (sqrDistance <= sqrRadius)
				{
					return true;
				}
			}

			lower = (s1p >= l_zero ? -E[1] : -E[1] - s1p / D[1]);
			upper = (s3n <= l_zero ? +E[1] : +E[1] - s3n / D[1]);
			if (lower <= upper)
			{
				P0 = { +E[0], lower, -E[2] };
				P1 = { +E[0], upper, -E[2] };
				sqrDistance = ComputeSqrDistance(P0, P1, C, W0, W1);
				if (sqrDistance <= sqrRadius)
				{
					return true;
				}
			}

			lower = (s4p >= l_zero ? -E[1] : -E[1] - s4p / D[1]);
			upper = (s6n <= l_zero ? +E[1] : +E[1] - s6n / D[1]);
			if (lower <= upper)
			{
				P0 = { -E[0], lower, +E[2] };
				P1 = { -E[0], upper, +E[2] };
				sqrDistance = ComputeSqrDistance(P0, P1, C, W0, W1);
				if (sqrDistance <= sqrRadius)
				{
					return true;
				}
			}

			lower = (s2p >= l_zero ? -E[0] : -E[0] - s2p / D[0]);
			upper = (s3n <= l_zero ? +E[0] : +E[0] - s3n / D[0]);
			if (lower <= upper)
			{
				P0 = { lower, +E[1], -E[2] };
				P1 = { upper, +E[1], -E[2] };
				sqrDistance = ComputeSqrDistance(P0, P1, C, W0, W1);
				if (sqrDistance <= sqrRadius)
				{
					return true;
				}
			}

			lower = (s4p >= l_zero ? -E[0] : -E[0] - s4p / D[0]);
			upper = (s5n <= l_zero ? +E[0] : +E[0] - s5n / D[0]);
			if (lower <= upper)
			{
				P0 = { lower, -E[1], +E[2] };
				P1 = { upper, -E[1], +E[2] };
				sqrDistance = ComputeSqrDistance(P0, P1, C, W0, W1);
				if (sqrDistance <= sqrRadius)
				{
					return true;
				}
			}

			lower = (s3p >= l_zero ? -E[2] : -E[2] - s3p / D[2]);
			upper = (s5p <= l_zero ? +E[2] : +E[2] - s5p / D[2]);
			if (lower <= upper)
			{
				if (s3p >= l_zero)
				{
					P0 = { +E[0], +E[1] - s3p / D[1], -E[2] };
				}
				else
				{
					P0 = { +E[0], +E[1], -E[2] - s3p / D[2] };
				}

				if (s5p <= l_zero)
				{
					P1 = { +E[0], -E[1] - s5p / D[1], +E[2] };
				}
				else
				{
					P1 = { +E[0], -E[1], +E[2] - s5p / D[2] };
				}

				sqrDistance = ComputeSqrDistance(P0, P1, C, W0, W1);
				if (sqrDistance <= sqrRadius)
				{
					return true;
				}
			}

			lower = (s2n >= l_zero ? -E[2] : -E[2] - s2n / D[2]);
			upper = (s4n <= l_zero ? +E[2] : +E[2] - s4n / D[2]);
			if (lower <= upper)
			{
				if (s2n >= l_zero)
				{
					P0 = { -E[0], +E[1] - s2n / D[1], -E[2] };
				}
				else
				{
					P0 = { -E[0], +E[1], -E[2] - s2n / D[2] };
				}

				if (s4n <= l_zero)
				{
					P1 = { -E[0], -E[1] - s4n / D[1], +E[2] };
				}
				else
				{
					P1 = { -E[0], -E[1], +E[2] - s4n / D[2] };
				}

				sqrDistance = ComputeSqrDistance(P0, P1, C, W0, W1);
				if (sqrDistance <= sqrRadius)
				{
					return true;
				}
			}

			lower = (s6p >= l_zero ? -E[0] : -E[0] - s6p / D[0]);
			upper = (s3p <= l_zero ? +E[0] : +E[0] - s3p / D[0]);
			if (lower <= upper)
			{
				if (s6p >= l_zero)
				{
					P0 = { -E[0], +E[1], +E[2] - s6p / D[2] };
				}
				else
				{
					P0 = { -E[0] - s6p / D[0], +E[1], +E[2] };
				}

				if (s3p <= l_zero)
				{
					P1 = { +E[0], +E[1], -E[2] - s3p / D[2] };
				}
				else
				{
					P1 = { +E[0] - s3p / D[0], -E[1], -E[2] };
				}

				sqrDistance = ComputeSqrDistance(P0, P1, C, W0, W1);
				if (sqrDistance <= sqrRadius)
				{
					return true;
				}
			}

			lower = (s4n >= l_zero ? -E[0] : -E[0] - s4n / D[0]);
			upper = (s1n <= l_zero ? +E[0] : +E[0] - s1n / D[0]);
			if (lower <= upper)
			{
				if (s4n >= l_zero)
				{
					P0 = { -E[0], -E[1], +E[2] - s4n / D[2] };
				}
				else
				{
					P0 = { -E[0] - s4n / D[0], -E[1], +E[2] };
				}

				if (s1n <= l_zero)
				{
					P1 = { +E[0], -E[1], -E[2] - s1n / D[2] };
				}
				else
				{
					P1 = { +E[0] - s1n / D[0], -E[1], -E[2] };
				}

				sqrDistance = ComputeSqrDistance(P0, P1, C, W0, W1);
				if (sqrDistance <= sqrRadius)
				{
					return true;
				}
			}

			lower = (s5p >= l_zero ? -E[1] : -E[1] - s5p / D[1]);
			upper = (s6p <= l_zero ? +E[1] : +E[1] - s6p / D[1]);
			if (lower <= upper)
			{
				if (s5p >= l_zero)
				{
					P0 = { +E[0] - s5p / D[0], -E[1], +E[2] };
				}
				else
				{
					P0 = { +E[0], -E[1] - s5p / D[1], +E[2] };
				}

				if (s6p <= l_zero)
				{
					P1 = { -E[0] - s6p / D[0], +E[1], +E[2] };
				}
				else
				{
					P1 = { -E[0], E[1] - s6p / D[1], +E[2] };
				}

				sqrDistance = ComputeSqrDistance(P0, P1, C, W0, W1);
				if (sqrDistance <= sqrRadius)
				{
					return true;
				}
			}

			lower = (s1n >= l_zero ? -E[1] : -E[1] - s1n / D[1]);
			upper = (s2n <= l_zero ? +E[1] : +E[1] - s2n / D[1]);
			if (lower <= upper)
			{
				if (s1n >= l_zero)
				{
					P0 = { +E[0] - s1n / D[0], -E[1], -E[2] };
				}
				else
				{
					P0 = { +E[0], -E[1] - s1n / D[1], -E[2] };
				}

				if (s2n <= l_zero)
				{
					P1 = { -E[0] - s2n / D[0], +E[1], -E[2] };
				}
				else
				{
					P1 = { -E[0], E[1] - s2n / D[1], -E[2] };
				}

				sqrDistance = ComputeSqrDistance(P0, P1, C, W0, W1);
				if (sqrDistance <= sqrRadius)
				{
					return true;
				}
			}

			return false;
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
		inline double Normalize(CPoint3& f_vec_to_normalize, bool f_robust = false)
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
};

#endif
