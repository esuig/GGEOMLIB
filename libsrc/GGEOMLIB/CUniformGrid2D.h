
#ifndef H_UNIFORM_GRID
#define H_UNIFORM_GRID

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "IGeometricalObject2D.h"
#include "CAARectangle.h"
#include "CompGeomAlgorithms.h"

namespace ggeomlib
{
	/**
	* @brief Uniform grid that buckets 2D geometrical objects by the cells they
	*        intersect, for fast spatial queries.
	*
	* The rectangular domain is divided into Columns() x Rows() cells of size
	* CellWidth() x CellHeight() (cells in the last column and row may be
	* smaller, clipped to the domain). Each cell stores shared pointers to the
	* objects intersecting it; an object spanning several cells is stored in
	* each of them.
	*
	* Cell (i, j) covers [xmin + i*hx, xmin + (i+1)*hx] x
	* [ymin + j*hy, ymin + (j+1)*hy], where i is the column (along X) and j the
	* row (along Y).
	*
	* Objects must implement IGeometricalObject2D, providing their bounding box
	* (Bounds()) and an exact intersection test with an axis-aligned box
	* (IntersectsWith2DBox()). The bounding box selects the candidate cells; the
	* exact test decides whether the object is actually stored in each of them.
	* 
	* 
	*/
	class CUniformGrid2D
	{
	public:
		/** @brief Shared pointer to a stored object. */
		using ObjectPtr = std::shared_ptr<const IGeometricalObject2D>;
		/** @brief Content of a cell: the objects intersecting it. */
		using Cell = std::vector<ObjectPtr>;

		/**
		* @brief Constructs an empty grid over a rectangular domain.
		*
		* The number of columns and rows is ceil(domain width / f_hx) and
		* ceil(domain height / f_hy).
		*
		* @param f_domain Rectangular domain covered by the grid.
		* @param f_hx     Cell width (along X), > 0.
		* @param f_hy     Cell height (along Y), > 0.
		*
		* @throw std::invalid_argument If any value is not finite, the domain has
		*        zero or negative width or height, or f_hx or f_hy is not positive.
		*
		* @warning Very small cell sizes compared to the domain create a very large
		*          number of cells (Columns() * Rows() vectors are allocated).
		*/
		CUniformGrid2D(CAARectangle f_domain,
					   double f_hx,
					   double f_hy) :
							m_domain(f_domain),
							m_hx(f_hx),
							m_hy(f_hy)
		{
			if (!std::isfinite(f_domain.GetXMin()) ||
				!std::isfinite(f_domain.GetYMin()) ||
				!std::isfinite(f_domain.GetXMax()) ||
				!std::isfinite(f_domain.GetYMax()) ||
				!std::isfinite(f_hx) || !std::isfinite(f_hy) ||
				!(f_domain.GetXMax() > f_domain.GetXMin()) ||
				!(f_domain.GetYMax() > f_domain.GetYMin()) ||
				!(f_hx > 0.0) || !(f_hy > 0.0))
			{
				throw std::invalid_argument("Invalid grid parameters");
			}

			m_nx = static_cast<size_t>(std::ceil((m_domain.GetXMax() - m_domain.GetXMin()) / m_hx));
			m_ny = static_cast<size_t>(std::ceil((m_domain.GetYMax() - m_domain.GetYMin()) / m_hy));
			m_cells.resize(m_nx * m_ny);
		}

		/**
		* @brief Inserts an object into all the grid cells it intersects.
		*
		* The object's bounding box, clipped to the domain, selects the candidate
		* cells; the object is then stored in each candidate cell for which
		* IntersectsWith2DBox() returns true.
		*
		* @param f_object Object to insert (shared ownership is kept by the grid).
		*
		* @return true if the object's bounding box overlaps the domain (the object
		*         may still end up in no cell if its exact shape misses all the
		*         candidate cells); false if it lies completely outside the domain,
		*         in which case it is not stored.
		*
		* @throw std::invalid_argument If f_object is null, or its bounds are not
		*        finite or not ordered (min > max).
		*
		* @note Inserting the same object twice stores it twice in each cell.
		*/
		inline bool Insert(ObjectPtr f_object)
		{
			if (!f_object)
				throw std::invalid_argument("Null geometry");

			double l_obj_x_min = 0.0;
			double l_obj_x_max = 0.0;
			double l_obj_y_min = 0.0;
			double l_obj_y_max = 0.0;
			f_object->Bounds(l_obj_x_min, l_obj_y_min, l_obj_x_max, l_obj_y_max);

			if (!std::isfinite(l_obj_x_min) ||
				!std::isfinite(l_obj_y_min) ||
				!std::isfinite(l_obj_x_max) ||
				!std::isfinite(l_obj_y_max) ||
				l_obj_x_min > l_obj_x_max ||
				l_obj_y_min > l_obj_y_max)
			{
				throw std::invalid_argument("Invalid object bounds");
			}

			CAARectangle l_obj_rect(CPoint2(l_obj_x_min, l_obj_y_min), CPoint2(l_obj_x_max, l_obj_y_max));

			if (!cgeom_algs::AreRectangleOverlapped(l_obj_rect, m_domain))
				return false;

			l_obj_x_min = std::max(l_obj_x_min, m_domain.GetXMin());
			l_obj_y_min = std::max(l_obj_y_min, m_domain.GetYMin());
			l_obj_x_max = std::min(l_obj_x_max, m_domain.GetXMax());
			l_obj_y_max = std::min(l_obj_y_max, m_domain.GetYMax());

			auto l_i0 = ToIndexX(l_obj_x_min);
			auto l_j0 = ToIndexY(l_obj_y_min);
			auto l_i1 = ToIndexX(l_obj_x_max);
			auto l_j1 = ToIndexY(l_obj_y_max);

			for (std::size_t j = l_j0; j <= l_j1; ++j)
			{
				for (std::size_t i = l_i0; i <= l_i1; ++i)
				{
					double l_cell_x_min = 0.0;
					double l_cell_y_min = 0.0;
					double l_cell_x_max = 0.0;
					double l_cell_y_max = 0.0;
					CellBounds(i, j, l_cell_x_min, l_cell_y_min, l_cell_x_max, l_cell_y_max);
					if (f_object->IntersectsWith2DBox(l_cell_x_min, l_cell_y_min, l_cell_x_max, l_cell_y_max))
					{
						m_cells[LinearIndex(i, j)].push_back(f_object);
					}
				}
			}

			return true;
		}

		/**
		* @brief Returns the objects stored in the cell containing a point.
		*
		* @param f_x X-coordinate of the query point.
		* @param f_y Y-coordinate of the query point.
		*
		* @return Pointer to the content of the cell containing (f_x, f_y), or
		*         nullptr if the point is not finite or lies outside the domain.
		*         The domain is half-open for queries: [xmin, xmax) x [ymin, ymax).
		*         The pointer remains valid until the grid is destroyed.
		*
		* @note The returned objects are candidates: they intersect the cell, not
		*       necessarily the query point.
		*/
		inline const Cell* Query(double f_x, double f_y) const
		{
			if (!std::isfinite(f_x) || !std::isfinite(f_y) ||
				f_x < m_domain.GetXMin() || f_x >= m_domain.GetXMax() ||
				f_y < m_domain.GetYMin() || f_y >= m_domain.GetYMax())
				return nullptr;

			const auto l_i = ToIndexX(f_x);
			const auto l_j = ToIndexY(f_y);

			return &m_cells[LinearIndex(l_i, l_j)];
		}

		/**
		* @brief Returns the content of a cell by its indices.
		*
		* @param f_i Column index, in [0, Columns()).
		* @param f_j Row index, in [0, Rows()).
		*
		* @return The objects stored in cell (f_i, f_j).
		*
		* @throw std::out_of_range If an index is out of range.
		*/
		inline const Cell& At(const size_t f_i, const size_t f_j) const
		{
			if (f_i >= m_nx || f_j >= m_ny)
				throw std::out_of_range("Invalid cell index");

			return m_cells[LinearIndex(f_i, f_j)];
		}

		/** @brief Returns the number of cells along X. */
		inline std::size_t Columns() const
		{
			return m_nx;
		}
		/** @brief Returns the number of cells along Y. */
		inline std::size_t Rows() const
		{
			return m_ny;
		}
		/** @brief Returns the domain covered by the grid. */
		inline const CAARectangle& Domain() const
		{
			return m_domain;
		}
		/** @brief Returns the nominal cell width (along X). */
		inline double CellWidth() const
		{
			return m_hx;
		}
		/** @brief Returns the nominal cell height (along Y). */
		inline double CellHeight() const
		{
			return m_hy;
		}

		/**
		* @brief Returns the column index of the cells containing an X-coordinate.
		*
		* Values at or beyond the domain's maximum X are clamped to the last column.
		*
		* @param f_x X-coordinate, with f_x >= domain minimum X.
		*
		* @return Column index, in [0, Columns()).
		*
		* @warning f_x must not be smaller than the domain's minimum X (nor NaN):
		*          the negative cell index cannot be converted to std::size_t
		*          (undefined behavior).
		*/
		inline std::size_t ToIndexX(double f_x) const
		{
			return std::min(static_cast<std::size_t>(std::floor((f_x - m_domain.GetXMin()) / m_hx)), m_nx - 1);
		}
		/**
		* @brief Returns the row index of the cells containing a Y-coordinate.
		*
		* Same as ToIndexX(), along Y.
		*
		* @param f_y Y-coordinate, with f_y >= domain minimum Y.
		*
		* @return Row index, in [0, Rows()).
		*
		* @warning f_y must not be smaller than the domain's minimum Y (nor NaN).
		*/
		inline std::size_t ToIndexY(double f_y) const
		{
			return std::min(static_cast<std::size_t>(std::floor((f_y - m_domain.GetYMin()) / m_hy)), m_ny - 1);
		}

	private:
		/** @brief Rectangular domain covered by the grid.*/
		CAARectangle m_domain;
		/** @brief Nominal cell width (along X) */
		double m_hx;
		/** @brief Nominal cell height (along Y) */
		double m_hy;
		/** @brief Number of cells along X (columns) */
		size_t m_nx;
		/** @brief Number of cells along Y (rows) */
		size_t m_ny;
		/** @brief Cells, stored row by row: index j * m_nx + i */
		std::vector<Cell> m_cells;

		/**
		* @brief Converts cell indices (column f_i, row f_j) to the index in
		*        m_cells.
		*/
		inline std::size_t LinearIndex(const size_t f_i, const size_t f_j) const
		{
			return f_j * m_nx + f_i;
		}

		/**
		* @brief Computes the bounds of cell (f_i, f_j), clipped to the domain.
		*
		* @param[in]  f_i          Column index.
		* @param[in]  f_j          Row index.
		* @param[out] f_cell_x_min Minimum X of the cell.
		* @param[out] f_cell_y_min Minimum Y of the cell.
		* @param[out] f_cell_x_max Maximum X of the cell.
		* @param[out] f_cell_y_max Maximum Y of the cell.
		*/
		inline void CellBounds(const size_t f_i, const size_t f_j,
							   double& f_cell_x_min, double& f_cell_y_min,
							   double& f_cell_x_max, double& f_cell_y_max) const
		{
			f_cell_x_min = m_domain.GetXMin() + f_i * m_hx;
			f_cell_y_min = m_domain.GetYMin() + f_j * m_hy;
			f_cell_x_max = std::min(f_cell_x_min + m_hx, m_domain.GetXMax());
			f_cell_y_max = std::min(f_cell_y_min + m_hy, m_domain.GetYMax());
		}
	};
}

#endif
