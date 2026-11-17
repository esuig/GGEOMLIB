
#ifndef H_UNIFORM_GRID
#define H_UNIFORM_GRID

#include "CUniformGrid2D.h"

namespace ggeomlib
{
	/**
	* @brief Uniform grid that rejects objects coinciding with an object already
	*        stored.
	*
	* Extends CUniformGrid2D with InsertIfUnduplicated(), which inserts an
	* object only if no coincident object is already present in the grid.
	*
	* Currently only points (CPoint2) can be coincident: two points coincide if
	* their distance is below COINC_THRESH. Objects of any other type are
	* never considered coincident, so they are always inserted.
	*
	* @note The plain Insert() inherited from CUniformGrid2D is still available
	*       and does not check for duplicates.
	*/
	class CUniqueUniformGrid2D : public CUniformGrid2D
	{
	public:

		using CUniformGrid2D::CUniformGrid2D;

		/**
		* @brief Inserts an object only if no coincident object is already stored.
		*
		* The cells overlapping the object's bounding box, enlarged by
		* COINC_THRESH, are searched for a coincident object (see
		* Coincident()). If none is found, the object is inserted with
		* CUniformGrid2D::Insert().
		*
		* @param f_object Object to insert (shared ownership is kept by the grid).
		*
		* @return true if the object was inserted; false if a coincident object is
		*         already stored, or if the object lies completely outside the
		*         domain (in both cases the object is not stored).
		*
		* @throw std::invalid_argument If f_object is null, or its bounds are not
		*        finite or not ordered (min > max).
		*/
		inline bool InsertIfUnduplicated(ObjectPtr f_object)
		{
			if (!f_object)
			{
				throw std::invalid_argument("Null geometry");
			}
			double l_xmin = 0.0;
			double l_ymin = 0.0;
			double l_xmax = 0.0;
			double l_ymax = 0.0;;
			f_object->Bounds(l_xmin, l_ymin, l_xmax, l_ymax);

			const auto& l_domain = Domain();

			if (l_xmax < l_domain.GetXMin() ||
				l_xmin > l_domain.GetXMax() ||
				l_ymax < l_domain.GetYMin() ||
				l_ymin > l_domain.GetYMax())
			{
				return false;
			}

			l_xmin = std::max(l_xmin, l_domain.GetXMin());
			l_ymin = std::max(l_ymin, l_domain.GetYMin());
			l_xmax = std::min(l_xmax, l_domain.GetXMax());
			l_ymax = std::min(l_ymax, l_domain.GetYMax());

			const auto l_i0 = ToIndexX(l_xmin);
			const auto l_j0 = ToIndexY(l_ymin);
			const auto l_i1 = ToIndexX(l_xmax);
			const auto l_j1 = ToIndexY(l_ymax);

			for (std::size_t j = l_j0; j <= l_j1; ++j)
			{
				for (std::size_t i = l_i0; i <= l_i1; ++i)
				{
					for (const auto& existing : At(i, j))
					{
						if (Coincident(*f_object, *existing))
							return false;
					}
				}
			}
			const bool l_insert_ret = Insert(std::move(f_object));
			return l_insert_ret;
		}

	private:
		/**
		* @brief Checks whether two objects coincide.
		*
		* Two points (CPoint2) coincide if their distance is below
		* COINC_THRESH. For any other combination of types, the objects are
		* considered distinct.
		*
		* @param a First object.
		* @param b Second object.
		*
		* @return true if a and b are points closer than COINC_THRESH,
		*         false otherwise.
		*/
		inline bool Coincident(const IGeometricalObject2D& f_a,
							   const IGeometricalObject2D& f_b)
		{
			if (const CPoint2* l_ca = dynamic_cast<const CPoint2*>(&f_a))
			{
				if (const CPoint2* l_cb = dynamic_cast<const CPoint2*>(&f_b))
				{
					return Distance(*l_ca, *l_cb) < COINC_THRESH;
				}
			}
			return false;
		}
	};
}

#endif