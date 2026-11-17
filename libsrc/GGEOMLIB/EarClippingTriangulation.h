
#ifndef H_EAR_TRIANGU
#define H_EAR_TRIANGU

#include <vector>

#include "CPoint2.h"
#include "CompGeomAlgorithms.h"

namespace ggeomlib
{
	/**
	* @brief A 2D triangle given by its three vertices.
	*
	* Triangles produced by TriangulateByEarClipping() are counterclockwise.
	*/
	struct TTriangle
	{
		CPoint2 m_vert_1;
		CPoint2 m_vert_2;
		CPoint2 m_vert_3;
	};

	/**
	* @brief Computes the 2D cross product of the vectors from a to b and from a
	*        to c.
	*
	*     (b - a) x (c - a) = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
	*
	* It equals twice the signed area of triangle (a, b, c): positive if the
	* triangle is counterclockwise, negative if clockwise, zero if the points
	* are collinear.
	*
	* @param a Corner point.
	* @param b First point.
	* @param c Second point.
	*
	* @return Twice the signed area of triangle (a, b, c).
	*/
	inline double CornerVectorsCross(const CPoint2& a, const CPoint2& b, const CPoint2& c)
	{
		return Cross((b - a), (c - a));
	}

	/**
	* @brief Checks whether a vertex of a counterclockwise polygon is strictly
	*        convex.
	*
	* The vertex is convex if the turn from the previous vertex, through it, to
	* the next vertex is a left turn, i.e. if the cross product computed by
	* CornerVectorsCross(prev, vertex, next) is greater than f_eps. Collinear
	* vertices are therefore not convex.
	*
	* @param f_poly Polygon vertices, counterclockwise, without a repeated
	*               closing point (the polygon is implicitly closed).
	* @param i      Index of the vertex to test, in [0, f_poly.size()).
	* @param f_eps  Non-negative tolerance on the cross product (area units).
	*
	* @return true if the vertex is strictly convex, false if it is reflex or
	*         collinear with its neighbours.
	*/
	inline bool IsVertexConvex(const std::vector<CPoint2>& f_poly, const size_t i, const double f_eps)
	{
		const size_t l_prev_vert_idx = (i + f_poly.size() - 1) % f_poly.size();
		const size_t l_next_vert_idx = (i + 1) % f_poly.size();
		const double c = CornerVectorsCross(f_poly[l_prev_vert_idx], f_poly[i], f_poly[l_next_vert_idx]);
		return c > f_eps;
	}

	/**
	* @brief Checks whether a vertex of a counterclockwise polygon is an ear.
	*
	* A vertex is an ear if it is strictly convex and the triangle formed with
	* its two neighbours contains no other vertex of the polygon (inside or on
	* its boundary, within f_eps). Such a triangle can be cut off the polygon
	* without crossing any edge.
	*
	* @param f_poly Polygon vertices, counterclockwise, without a repeated
	*               closing point.
	* @param i      Index of the vertex to test, in [0, f_poly.size()).
	* @param f_eps  Non-negative tolerance used for the convexity test and the
	*               point-in-triangle test (area units).
	*
	* @return true if the vertex is an ear, false otherwise.
	*
	* @note The containment test is inclusive: a vertex lying on the boundary
	*       of the candidate triangle, or coinciding with one of its vertices
	*       (duplicate points), prevents the ear.
	*/
	inline bool IsVertexEar(const std::vector<CPoint2>& f_poly, size_t i, const double f_eps)
	{
		size_t l_prev_vert_idx = (i + f_poly.size() - 1) % f_poly.size();
		size_t l_next_vert_idx = (i + 1) % f_poly.size();

		const CPoint2& l_prev_vert = f_poly[l_prev_vert_idx];
		const CPoint2& l_candidate_vert = f_poly[i];
		const CPoint2& l_next_vert = f_poly[l_next_vert_idx];

		if (!IsVertexConvex(f_poly, i, f_eps))
		{
			return false;
		}
		for (size_t j = 0; j < f_poly.size(); ++j)
		{
			if (j == l_prev_vert_idx || j == i || j == l_next_vert_idx)
			{
				continue;
			}
			if (cgeom_algs::IsPointInsideTriangle2DInclusive(l_prev_vert,
															 l_candidate_vert,
															 l_next_vert,
															 f_poly[j],
															 f_eps))
			{
				return false;
			}
		}

		return true;

	}

	/**
	* @brief Triangulates a simple polygon by ear clipping.
	*
	* Repeatedly finds an ear (see IsVertexEar()), outputs the triangle formed
	* by the ear and its two neighbours, and removes the ear vertex from the
	* polygon, until fewer than 3 vertices are left. For a simple polygon with n
	* vertices, this yields n - 2 triangles.
	*
	* The input may be given in either orientation (it is reversed internally if
	* clockwise) and with or without a repeated closing point. The tolerance for
	* the geometric tests is computed once from the size of the polygon's
	* bounding box (1e-12 * size^2, since the tests use cross products, which
	* are in area units).
	*
	* @param f_input_poly Vertices of a simple (non-self-intersecting) polygon,
	*                     without holes.
	*
	* @return Counterclockwise triangles covering the polygon. Empty if the
	*         polygon has fewer than 3 distinct vertices.
	*
	* @warning If no ear can be found before the polygon is exhausted, the loop
	*          stops and the triangles found so far are returned. This happens
	*          when only collinear vertices are left (zero area, harmless), but
	*          also for self-intersecting polygons or polygons with duplicate or
	*          touching vertices: the result then covers only part of the
	*          polygon. Compare the number of triangles with n - 2 to detect it.
	*
	* @note Complexity is O(n^3) in the worst case (O(n) candidate vertices,
	*       each tested against O(n) vertices, for each of the n - 2 ears). This
	*       is adequate for small and medium polygons.
	*
	* @see D. Eberly, "Triangulation by Ear Clipping", Geometric Tools.
	*/
	inline std::vector<TTriangle> TriangulateByEarClipping(const std::vector<CPoint2>& f_input_poly)
	{
		std::vector<CPoint2> l_poly = f_input_poly;
		if (l_poly.size() > 1 && Distance(l_poly.front(), l_poly.back()) < COINC_THRESH)
		{
			l_poly.pop_back();
		}

		std::vector<TTriangle> tris;
		if (l_poly.size() < 3)
		{
			return tris;
		}

		double l_min_x = l_poly[0].m_x, l_max_x = l_poly[0].m_x;
		double l_min_y = l_poly[0].m_y, l_max_y = l_poly[0].m_y;
		for (const auto& p : l_poly)
		{
			l_min_x = std::min(l_min_x, p.m_x); l_max_x = std::max(l_max_x, p.m_x);
			l_min_y = std::min(l_min_y, p.m_y); l_max_y = std::max(l_max_y, p.m_y);
		}
		const double l_scale = std::max(l_max_x - l_min_x, l_max_y - l_min_y);
		const double l_eps = 1e-12 * l_scale * l_scale;

		tris.reserve(l_poly.size() - 2);
		while (l_poly.size() >= 3)
		{
			bool l_removed = false;
			for (size_t i = 0; i < l_poly.size(); ++i)
			{
				if (IsVertexEar(l_poly, i, l_eps))
				{
					const size_t prev = (i + l_poly.size() - 1) % l_poly.size();
					const size_t next = (i + 1) % l_poly.size();

					tris.push_back({ l_poly[prev], l_poly[i], l_poly[next] });
					l_poly.erase(l_poly.begin() + static_cast<std::ptrdiff_t>(i));
					l_removed = true;
					break;
				}
			}

			if (!l_removed)
			{
				break;
			}
		}

		return tris;
	}
}

#endif