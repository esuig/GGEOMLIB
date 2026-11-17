
#ifndef H_CCYLINDER
#define H_CCYLINDER

#include "CPoint3.h"

namespace ggeomlib
{
	/**
	* @class CCylinder
	* @brief Represents a 3D finite cylinder defined by a central point, a unit axis direction,
	*        a length, and a radius.
	*
	* This class stores the geometric representation of a finite cylinder. The axis vector is assumed
	* to be normalized. The cylinder is symmetric around the axis and centered at a given point.
	*/
	class CCylinder
	{
	public:
		/**
		* @brief Constructs a cylinder using center, axis direction, length, and radius.
		*
		* @param f_cylinder_center     Center point of the cylinder (the base point).
		* @param f_cylinder_unit_axis  Unit vector representing the direction of the cylinder's axis.
		* @param f_cylinder_length     Length of the cylinder along its axis.
		* @param f_cylinder_radius     Radius of the cylinder.
		*/
		CCylinder(const CPoint3& f_cylinder_center, 
				  const CPoint3& f_cylinder_unit_axis,
				  const double f_cylinder_length, 
				  const double f_cylinder_radius) :
						m_center(f_cylinder_center),
						m_unit_axis(f_cylinder_unit_axis),
						m_radius(f_cylinder_radius),
						m_length(f_cylinder_length)
		{

		}

		/**
		* @brief Gets the center of the cylinder (the base point).
		*
		* @return A constant reference to the center point.
		*/
		inline const CPoint3& GetCylCenter() const
		{
			return m_center;
		}

		/**
		* @brief Gets the unit vector representing the cylinder's axis.
		*
		* @return A constant reference to the unit axis vector.
		*/
		inline const CPoint3& GetCylUnitAxis() const
		{
			return m_unit_axis;
		}
		/**
		* @brief Gets the radius of the cylinder.
		*
		* @return Radius value (non-negative).
		*/
		inline double GetCylRadius() const
		{
			return m_radius;
		}
		/**
		* @brief Gets the length of the cylinder along its axis.
		*
		* @return Length value (positive).
		*/
		inline double GetCylLength() const
		{
			return m_length;
		}

	private:

		CPoint3 m_center;
		CPoint3 m_unit_axis;
		double m_radius;
		double m_length;
	};
}

#endif