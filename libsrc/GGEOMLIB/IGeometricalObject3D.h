
#ifndef H_IGEOM_OBJ3D
#define H_IGEOM_OBJ3D

#pragma once

namespace ggeomlib
{
	/**
	* @brief Interface for 3D geometrical objects that can be stored in spatial
	*        structures (e.g. uniform grids or octrees).
	*
	* Implementations provide their axis-aligned bounding box and an
	* intersection test with an axis-aligned box. Spatial structures use the
	* bounding box to find candidate cells quickly, and the box test to decide
	* whether the object actually intersects each candidate cell.
	*
	* @see IGeometricalObject2D for the 2D counterpart.
	*/
	class IGeometricalObject3D
	{
	public:
		/** @brief Virtual destructor, for deletion through a base pointer. */
		virtual ~IGeometricalObject3D() = default;
		
		/**
		* @brief Returns the axis-aligned bounding box of the object.
		*
		* @param[out] f_x_min Minimum X of the object.
		* @param[out] f_y_min Minimum Y of the object.
		* @param[out] f_z_min Minimum Z of the object.
		* @param[out] f_x_max Maximum X of the object.
		* @param[out] f_y_max Maximum Y of the object.
		* @param[out] f_z_max Maximum Z of the object.
		*
		* Implementations must return finite values with min <= max on each
		* axis (degenerate boxes, e.g. for a point, are allowed). The box must
		* contain the whole object; it does not need to be tight, but a tight box
		* gives fewer candidate cells.
		*/
		virtual void Bounds(double& f_x_min, 
							double& f_y_min,
							double& f_z_min,
							double& f_x_max,
							double& f_y_max,
							double& f_z_max) const = 0;
		/**
		* @brief Tests whether the object intersects an axis-aligned box.
		*
		* The box is closed: an object touching its boundary intersects it.
		*
		* @param f_x_min Minimum X of the box.
		* @param f_y_min Minimum Y of the box.
		* @param f_z_min Minimum Z of the box.
		* @param f_x_max Maximum X of the box.
		* @param f_y_max Maximum Y of the box.
		* @param f_z_max Maximum Z of the box (with min <= max on each axis).
		*
		* @return true if the object and the box share at least one point.
		*
		* Implementations should be exact. A conservative test (returning true
		* for some boxes the object does not touch) is also acceptable: spatial
		* structures then store the object in a few extra cells, which costs
		* performance but not correctness. Returning false for a box the object
		* does touch is not acceptable: the object would be missing from that
		* cell.
		*/
		virtual bool IntersectsWith3DBox(const double f_x_min, 
										 const double f_y_min, 
										 const double f_z_min,
										 const double f_x_max, 
										 const double f_y_max,
										 const double f_z_max) const = 0;
	};
}

#endif
