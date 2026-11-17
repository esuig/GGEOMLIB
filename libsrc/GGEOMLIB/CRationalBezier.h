
#ifndef H_RATIONAL_BEZIER
#define H_RATIONAL_BEZIER
#pragma once

#include "CPoint3.h"
#include "CPoint4.h"
#include "MathAlgorithms.h"

#include <cmath>
#include <vector>
#include <algorithm>

namespace ggeomlib
{
	/**
	* @brief Rational Bézier curve of arbitrary degree, stored with homogeneous
	*        control points.
	*
	* Each control point is stored in homogeneous form (w*x, w*y, w*z, w), where
	* w is its weight. With this representation, a rational Bézier curve is
	* evaluated and subdivided by applying the ordinary (polynomial) de
	* Casteljau algorithm to the homogeneous points, and then dividing by w
	* (dehomogenizing).
	*
	* The curve is parameterized on [0, 1]; its degree is the number of control
	* points minus one.
	*
	* @warning EvaluateHom(), Split() and FromTo() assume that the curve has at
	*          least one control point.
	*
	* @see G. Farin, "Curves and Surfaces for CAGD: A Practical Guide",
	*      5th ed., Morgan Kaufmann, 2002 (rational Bézier curves).
	* @see L. Piegl, W. Tiller, "The NURBS Book", 2nd ed., Springer, 1997.
	*/
	class CRationalBezier
	{
	public:
		/**
		* @brief Control points in homogeneous form (w*x, w*y, w*z, w), in order
		*        from the start to the end of the curve.
		*/
		std::vector<CPoint4> m_control_points;

		/**
		* @brief Constructs a rational Bézier curve from its homogeneous control
		*        points.
		*
		* @param f_cp Control points in homogeneous form (w*x, w*y, w*z, w). Taken
		*             by value and moved into the curve.
		*/
		CRationalBezier(std::vector<CPoint4> f_cp) :
			m_control_points(std::move(f_cp))
		{

		}

		/**
		* @brief Returns the control polygon in Euclidean coordinates.
		*
		* Each homogeneous control point (w*x, w*y, w*z, w) is converted to
		* (x, y, z) by dividing by its weight.
		*
		* @return The dehomogenized control points, in order.
		*
		* @warning Control points with zero weight (points at infinity) cannot be
		*          dehomogenized.
		*/
		inline std::vector<CPoint3> GetControlPolygon()
		{
			std::vector<CPoint3> l_control_points_euc;
			l_control_points_euc.reserve(m_control_points.size());
			for (const auto& l_pt : m_control_points)
			{
				l_control_points_euc.emplace_back(l_pt.DeHom());
			}
			return l_control_points_euc;
		}

		/**
		* @brief Evaluates the curve at a parameter value, in homogeneous form.
		*
		* Applies the de Casteljau algorithm to the homogeneous control points:
		* the control polygon is repeatedly replaced by the linear interpolations
		* of consecutive points at f_t, until a single point is left.
		*
		* @param f_t Parameter value, normally in [0, 1] (values outside the range
		*            extrapolate the curve).
		*
		* @return The curve point at f_t in homogeneous form (w*x, w*y, w*z, w).
		*         Call DeHom() on it to obtain the Euclidean point.
		*/
		inline CPoint4 EvaluateHom(double f_t) const
		{
			std::vector<CPoint4> l_pts = m_control_points;
			const size_t l_n = l_pts.size();
			for (size_t r = 1; r < l_n; ++r)
			{
				for (size_t i = 0; i < l_n - r; ++i)
				{
					l_pts[i] = Lerp(l_pts[i], l_pts[i + 1], f_t);
				}
			}

			return l_pts[0];
		}

		/**
		* @brief Splits the curve at a parameter value into two curves of the same
		*        degree.
		*
		* Uses the de Casteljau algorithm: the left curve's control points are the
		* first points of each de Casteljau level, the right curve's control points
		* are the last ones. Together, the two curves represent exactly the same
		* geometry as the original curve.
		*
		* @param[in]  f_t     Split parameter, in [0, 1].
		* @param[out] f_left  Curve corresponding to the parameter range [0, f_t] of
		*                     the original curve (reparameterized on [0, 1]).
		* @param[out] f_right Curve corresponding to the parameter range [f_t, 1] of
		*                     the original curve (reparameterized on [0, 1]).
		*/
		inline void Split(double f_t, CRationalBezier& f_left, CRationalBezier& f_right) const
		{
			std::vector<CPoint4> l_pts = m_control_points;
			const size_t l_n = l_pts.size();
			f_left.m_control_points.resize(l_n);
			f_right.m_control_points.resize(l_n);

			f_left.m_control_points[0] = l_pts[0];
			f_right.m_control_points[l_n - 1] = l_pts[l_n - 1];
			for (size_t r = 1; r < l_n; ++r)
			{
				for (size_t i = 0; i < l_n - r; ++i)
				{
					l_pts[i] = Lerp(l_pts[i], l_pts[i + 1], f_t);
				}
				f_left.m_control_points[r] = l_pts[0];
				f_right.m_control_points[l_n - 1 - r] = l_pts[l_n - 1 - r];
			}
		}

		
		/**
		* @brief Returns the piece of the curve between two parameter values.
		*
		* The curve is first split at f_t1, keeping the left part; this part is
		* then split at f_t0 / f_t1 (the position of f_t0 in the rescaled
		* parameterization of the left part), keeping the right part.
		*
		* @param f_t0 Start parameter of the piece, in [0, 1].
		* @param f_t1 End parameter of the piece, in [0, 1], with f_t0 <= f_t1.
		*
		* @return A curve of the same degree representing the parameter range
		*         [f_t0, f_t1] of this curve, reparameterized on [0, 1].
		*
		* @note Structure adapted from fromTo() in flo-bezier3 by Floris Steenkamp
		*       (https://github.com/FlorisSteenkamp/FloBezier), MIT License,
		*       Copyright (c) 2026 Floris Steenkamp.
		*/
		inline CRationalBezier FromTo(double f_t0, double f_t1) const
		{
			CRationalBezier l_head = *this;
			CRationalBezier l_dummy({});
			if (f_t1 < 1.0)
			{
				CRationalBezier l_left({});
				Split(f_t1, l_left, l_dummy);
				l_head = l_left;
			}
			if (f_t0 > 0.0 && f_t1 > 0.0)
			{
				CRationalBezier l_right({});
				l_head.Split(f_t0 / f_t1, l_dummy, l_right);
				return l_right;
			}
			return l_head;
		}

		/**
		* @brief Computes the parameter values at which the curve must be split to
		*        approximate it with a polyline within a given tolerance.
		*
		* Starting from the whole interval [0, 1], the current piece of the curve is
		* tested with IsFlatEnough(); if it is not flat enough, its parameter
		* interval is halved and the left half is tested next. Intervals are
		* processed from left to right, so the returned parameters are sorted.
		*
		* Joining the curve points at the returned parameters gives a polyline whose
		* distance from the curve is at most f_chord_tol_mm, and whose segments
		* correspond to curve pieces not longer than f_max_seg_len_mm (if given).
		* The criteria are conservative (based on the control polygon), so the
		* curve may be split somewhat more than strictly needed.
		*
		* @param f_chord_tol_mm   Maximum allowed distance between the curve and
		*                         the polyline, in the units of the control points.
		* @param f_max_seg_len_mm Maximum allowed length of each curve piece, in the
		*                         units of the control points; 0 = no limit.
		* @param f_min_t_span     Minimum parameter interval: intervals not longer
		*                         than this are accepted even if not flat enough.
		*                         It guarantees termination, e.g. for curves with
		*                         non-positive weights.
		*
		* @return Sorted parameter values, starting with 0.0 and ending with 1.0.
		*         Consecutive values delimit the curve pieces.
		*
		* @note Subdivision structure adapted from splitByCurvatureAndLength() in
		*       flo-bezier3 by Floris Steenkamp
		*       (https://github.com/FlorisSteenkamp/FloBezier), MIT License,
		*       Copyright (c) 2026 Floris Steenkamp; the flatness criterion is
		*       replaced by IsFlatEnough().
		*/
		inline std::vector<double> SplitCurveByTolerance(const double f_chord_tol_mm,
														 const double f_max_seg_len_mm = 0.0,
														 double f_min_t_span = 1.0e-12) const
		{
			std::vector<double> l_start_params;
			std::vector<double> l_end_params;

			l_start_params.push_back(0.0);
			l_end_params.push_back(1.0);

			while (true)
			{
				const double l_t_start = l_start_params.back();
				const double l_t_end = l_end_params.back();

				const CRationalBezier l_seg = this->FromTo(l_t_start, l_t_end);

				if (IsFlatEnough(l_seg.m_control_points, f_chord_tol_mm, f_max_seg_len_mm) ||
					(l_t_end - l_t_start <= f_min_t_span))
				{
					l_start_params.push_back(l_t_end);
					l_end_params.pop_back();

					if (l_t_end == 1.0)
					{
						return l_start_params;
					}
					continue;
				}

				l_end_params.push_back((l_t_start + l_t_end) * 0.5);
			}
		}
	private:

		/**
		* @brief Checks whether a curve piece can be replaced by the segment joining
		*        its end points.
		*
		* For a rational Bézier curve with positive weights, the curve lies inside
		* the convex hull of its dehomogenized control points. Since the distance
		* from a segment is a convex function, if all the control points are within
		* f_chord_tol_mm of the segment joining the first and last control points
		* (which are the curve's end points), the whole convex hull, and thus the
		* whole curve, is too.
		*
		* Optionally, the piece is also rejected if its control polygon is longer
		* than f_max_seg_len_mm. The control polygon length is an upper bound of the
		* curve's arc length, so this test is conservative.
		*
		* @param f_cp             Homogeneous control points of the curve piece.
		* @param f_chord_tol_mm   Maximum allowed distance between the curve and the
		*                         chord, in the units of the control points.
		* @param f_max_seg_len_mm Maximum allowed length of the piece, in the units
		*                         of the control points; 0 = no limit.
		*
		* @return true if the piece is flat (and short) enough; false otherwise,
		*         and also if any weight is not positive (the convex hull bound does
		*         not hold, so the piece must be split further).
		*/
		inline bool IsFlatEnough(const std::vector<CPoint4>& f_cp,
								 const double f_chord_tol_mm,
								 const double f_max_seg_len_mm) const
		{
			if (f_cp.size() < 2)
			{
				return true;
			}

			for (const CPoint4& l_h : f_cp)
			{
				if (l_h.m_w <= 0.0)
				{
					return false;
				}
			}

			const CPoint3 l_a = f_cp.front().DeHom();
			const CPoint3 l_b = f_cp.back().DeHom();
			for (size_t i = 1; i + 1 < f_cp.size(); ++i)
			{
				if (cgeom_algs::PointSegmentDistance(f_cp[i].DeHom(), l_a, l_b) > f_chord_tol_mm)
				{
					return false;
				}
			}

			if (f_max_seg_len_mm > 0.0)
			{
				double l_len = 0.0;
				CPoint3 l_prev = l_a;
				for (size_t i = 1; i < f_cp.size(); ++i)
				{
					const CPoint3 l_cur = f_cp[i].DeHom();
					l_len += Distance(l_prev, l_cur);
					l_prev = l_cur;
				}
				if (l_len > f_max_seg_len_mm)
				{
					return false;
				}
			}

			return true;
		}
	};
}

#endif H_RATIONAL_BEZIER
