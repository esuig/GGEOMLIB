
#ifndef H_CPOINT2
#define H_CPOINT2

#pragma once

#include <cmath>

#include "IGeometricalObject2D.h"
#include "CPoint3.h"

namespace ggeomlib
{
	/**
	* @class CPoint2
	* @brief 2D point or vector with double precision coordinates
	*
	* Besides its coordinates, each point carries three auxiliary integer values and
	* three auxiliary double values that can be used to attach user data (e.g. ids,
	* flags, weights). These values are copied by the copy constructor and the copy
	* assignment operator, but are not propagated by arithmetic operators.
	*/
	class CPoint2 : public IGeometricalObject2D
	{
	public:

		/**
		* @brief Union to allow access to the vector's data via array or named members.
		* https://www.gamedev.net/forums/topic/671481-most-efficient-way-of-designing-a-vector-class-in-3d/
		*/
		union
		{
			/**
			* Array access to coordinates(x, y)
			*/
			double m_pc[2];
			struct
			{
				/**
				* X Coordinate
				*/
				double m_x;
				/**
				* Y Coordinate
				*/
				double m_y;
			};
		};


		/**
		* @brief Default constructor, initializes both coordinates and all auxiliary values to 
		* zero
		*/
		CPoint2() :
			m_x(0.0),
			m_y(0.0),
			m_val_int_1(0),
			m_val_int_2(0),
			m_val_int_3(0),
			m_val_double_1(0.0),
			m_val_double_2(0.0),
			m_val_double_3(0.0)
		{

		}

		/**
		* @brief Constructs a point from its coordinates
		*
		* All auxiliary values are initialized to zero.
		*
		* @param f_x X coordinate
		* @param f_y Y coordinate
		*/
		CPoint2(const double& f_x,
				const double& f_y) :
					m_x(f_x),
					m_y(f_y),
					m_val_int_1(0),
					m_val_int_2(0),
					m_val_int_3(0),
					m_val_double_1(0.0),
					m_val_double_2(0.0),
					m_val_double_3(0.0)
		{

		}

		/**
		* @brief Copy constructor
		*
		* Copies coordinates and all auxiliary values.
		*
		* @param f_p Point to copy
		*/
		CPoint2(const CPoint2& f_p) :
					m_x(f_p.m_x),
					m_y(f_p.m_y),
					m_val_int_1(f_p.m_val_int_1),
					m_val_int_2(f_p.m_val_int_2),
					m_val_int_3(f_p.m_val_int_3),
					m_val_double_1(f_p.m_val_double_1),
					m_val_double_2(f_p.m_val_double_2),
					m_val_double_3(f_p.m_val_double_3)
		{

		}

		/**
		* @brief Converting constructor from a 3D point
		*
		* Copies the x and y coordinates and all auxiliary values; the z coordinate is discarded.
		*
		* @param f_p CPoint3 to initialize this CPoint2 with (z coordinate is ignored)
		*/
		CPoint2(const CPoint3& f_p) :
					m_x(f_p.m_x),
					m_y(f_p.m_y),
					m_val_int_1(f_p.GetIntVal1()),
					m_val_int_2(f_p.GetIntVal2()),
					m_val_int_3(f_p.GetIntVal3()),
					m_val_double_1(f_p.GetDoubleVal1()),
					m_val_double_2(f_p.GetDoubleVal2()),
					m_val_double_3(f_p.GetDoubleVal3())
		{

		}

		/**
		* @brief Computes the axis-aligned bounding box of the point
		*
		* For a point the bounding box is degenerate: minimum and maximum both equal the point's
		* coordinates.
		*
		* @param[out] f_x_min Minimum X coordinate
		* @param[out] f_y_min Minimum Y coordinate
		* @param[out] f_x_max Maximum X coordinate
		* @param[out] f_y_max Maximum Y coordinate
		*/
		inline void Bounds(double& f_x_min,
						   double& f_y_min,
						   double& f_x_max,
						   double& f_y_max) const override
		{
			f_x_min = m_x;
			f_y_min = m_y;
			f_x_max = m_x;
			f_y_max = m_y;
		}

		/**
		* @brief Tests whether the point lies inside an axis-aligned 2D box
		*
		* Box boundaries are inclusive.
		*
		* @param f_x_min Minimum X coordinate of the box
		* @param f_y_min Minimum Y coordinate of the box
		* @param f_x_max Maximum X coordinate of the box
		* @param f_y_max Maximum Y coordinate of the box
		* @return true if the point is inside or on the boundary of the box, false otherwise
		*/
		inline bool IntersectsWith2DBox(const double f_x_min,
										const double f_y_min,
										const double f_x_max,
										const double f_y_max) const override
		{
			return m_x >= f_x_min && m_x <= f_x_max && m_y >= f_y_min && m_y <= f_y_max;
		}

		/**
		* @brief Copy assignment operator
		*
		* Copies coordinates and all auxiliary values.
		*
		* @param f_p Point to copy
		* @return Reference to this point
		*/
		CPoint2& operator=(const CPoint2& f_p)
		{
			m_pc[0] = f_p.m_pc[0];
			m_pc[1] = f_p.m_pc[1];
			m_val_double_1 = f_p.m_val_double_1;
			m_val_double_2 = f_p.m_val_double_2;
			m_val_double_3 = f_p.m_val_double_3;
			m_val_int_1 = f_p.m_val_int_1;
			m_val_int_2 = f_p.m_val_int_2;
			m_val_int_3 = f_p.m_val_int_3;
			return *this;
		}

		/**
		* @brief Sets the first auxiliary integer value
		* @param f_val New value
		*/
		inline void SetIntVal1(const int f_val)
		{
			m_val_int_1 = f_val;
		}
		/**
		* @brief Gets the first auxiliary integer value
		* @return First auxiliary integer value
		*/
		inline int GetIntVal1() const
		{
			return m_val_int_1;
		}
		/**
		* @brief Sets the second auxiliary integer value
		* @param f_val New value
		*/
		inline void SetIntVal2(const int f_val)
		{
			m_val_int_2 = f_val;
		}
		/**
		* @brief Gets the second auxiliary integer value
		* @return Second auxiliary integer value
		*/
		inline int GetIntVal2() const
		{
			return m_val_int_2;
		}
		/**
		* @brief Sets the third auxiliary integer value
		* @param f_val New value
		*/
		inline void SetIntVal3(const int f_val)
		{
			m_val_int_3 = f_val;
		}
		/**
		* @brief Gets the third auxiliary integer value
		* @return Third auxiliary integer value
		*/
		inline int GetIntVal3() const
		{
			return m_val_int_3;
		}
		/**
		* @brief Sets the first auxiliary double value
		* @param f_val New value
		*/
		inline void SetDoubleVal1(const double f_val)
		{
			m_val_double_1 = f_val;
		}
		/**
		* @brief Gets the first auxiliary double value
		* @return First auxiliary double value
		*/
		inline double GetDoubleVal1() const
		{
			return m_val_double_1;
		}
		/**
		* @brief Sets the second auxiliary double value
		* @param f_val New value
		*/
		inline void SetDoubleVal2(const double f_val)
		{
			m_val_double_2 = f_val;
		}
		/**
		* @brief Gets the second auxiliary double value
		* @return Second auxiliary double value
		*/
		inline double GetDoubleVal2() const
		{
			return m_val_double_2;
		}
		/**
		* @brief Sets the third auxiliary double value
		* @param f_val New value
		*/
		inline void SetDoubleVal3(const double f_val)
		{
			m_val_double_3 = f_val;
		}
		/**
		* @brief Gets the third auxiliary double value
		* @return Third auxiliary double value
		*/
		inline double GetDoubleVal3() const
		{
			return m_val_double_3;
		}

		/**
		* @brief Computes the squared length (x^2 + y^2)
		* @return Squared length
		*/
		inline double LengthSq() const
		{
			return m_x * m_x + m_y * m_y;
		}

		/**
		* @brief Computes the Euclidean length
		* @return Length
		*/
		inline double Length() const
		{
			return std::sqrt(LengthSq());
		}

		/**
		* @brief Normalizes this vector to unit length
		*
		* If the length is zero, the vector is left unchanged
		*
		* @return Reference to this vector
		*/
		inline CPoint2& UnitizeInPlace()
		{
			const double l_length = Length();
			if (l_length > 0.0)
			{
				m_x /= l_length;
				m_y /= l_length;
			}

			return *this;
		}

		/**
		* @brief Returns a copy of this vector normalized to unit length
		*
		* If the length is zero, an unchanged copy is returned
		*
		* @return Normalized copy
		*/
		inline CPoint2 GetUnitVector() const
		{
			const double l_length = Length();
			if (l_length > 0.0)
			{
				return CPoint2(m_x / l_length, m_y / l_length);
			}

			return *this;
		}

		/**
		* @brief Returns the vector rotated 90 degrees counter-clockwise
		* @return Perpendicular vector (-y, x), same length as this vector
		*/
		inline CPoint2 GetPerpendicular() const
		{
			return CPoint2(-m_y, m_x);
		}


		/**
		* @brief Accesses a coordinate by index
		* @param f_index Coordinate index (0 = x, 1 = y, 2 = z)
		* @return Reference to the selected coordinate
		*/
		inline double& operator[](size_t f_index)
		{
			return m_pc[f_index];
		}

		/**
		* @brief Accesses a coordinate by index (const version)
		* @param f_index Coordinate index (0 = x, 1 = y, 2 = z)
		* @return Const reference to the selected coordinate
		*/
		inline const double& operator[](size_t f_index) const
		{
			return m_pc[f_index];
		}

		/**
		* @brief Component-wise addition
		* @param f_p Point to add
		* @return Sum of the two points
		*/
		inline CPoint2 operator+(const CPoint2& f_p) const
		{
			return CPoint2(m_x + f_p.m_x, m_y + f_p.m_y);
		}
		/**
		* @brief Component-wise subtraction
		* @param f_p Point to subtract
		* @return Difference of the two points
		*/
		inline CPoint2 operator-(const CPoint2& f_p) const
		{
			return CPoint2(m_x - f_p.m_x, m_y - f_p.m_y);
		}
		/**
		* @brief Invert the vector
		* @return The negated vector
		*/
		inline CPoint2 operator-() const
		{
			return CPoint2(-m_x, -m_y);
		}
		/**
		* @brief Multiplies both components by a scalar
		* @param f_scalar Scale factor
		* @return Scaled vector
		*/
		inline CPoint2 operator*(double f_scalar) const
		{
			return CPoint2(m_x * f_scalar, m_y * f_scalar);
		}
		/**
		* @brief Divides both components by a scalar
		* @param f_scalar Divisor
		* @pre f_scalar != 0
		* @return Scaled vector
		*/
		inline CPoint2 operator/(double f_scalar) const
		{
			return CPoint2(m_x / f_scalar, m_y / f_scalar);
		}
		/**
		* @brief Multiplies both components of a vector by a scalar (scalar on the left)
		* @param f_scalar Scale factor
		* @param f_p Vector to scale
		* @return Scaled vector
		*/
		inline friend CPoint2 operator*(double f_scalar, const CPoint2& f_p)
		{
			return CPoint2(f_p.m_x * f_scalar, f_p.m_y * f_scalar);
		}

	private:

		/**
		* First auxiliary integer value
		*/
		int m_val_int_1;
		/**
		* Second auxiliary integer value
		*/
		int m_val_int_2;
		/**
		* Third auxiliary integer value
		*/
		int m_val_int_3;
		/**
		* First auxiliary double value
		*/
		double m_val_double_1;
		/**
		* Second auxiliary double value
		*/
		double m_val_double_2;
		/**
		* Third auxiliary double value
		*/
		double m_val_double_3;
	};

	/**
	* @brief Computes the dot product of two vectors
	* @param f_a First vector
	* @param f_b Second vector
	* @return a.x * b.x + a.y * b.y
	*/
	inline static double Dot(const CPoint2& f_a, const CPoint2& f_b)
	{
		return f_a.m_x * f_b.m_x + f_a.m_y * f_b.m_y;
	}

	/**
	* @brief Computes the 2D cross product of two vectors
	*
	* Equals the z-component of the 3D cross product (a.x, a.y, 0) x (b.x, b.y, 0)
	*
	* @param f_a First vector
	* @param f_b Second vector
	* @return Positive if @p f_b is counter-clockwise from @p f_a, negative if clockwise, 0 if collinear
	*/
	inline static double Cross(const CPoint2& f_a, const CPoint2& f_b)
	{
		return f_a.m_x * f_b.m_y - f_a.m_y * f_b.m_x;
	}

	/**
	* @brief Computes the squared distance between two points
	* @param f_a First point
	* @param f_b Second point
	* @return Squared distance
	*/
	inline static double DistanceSq(const CPoint2& f_a, const CPoint2& f_b)
	{
		const double l_dx = f_b.m_x - f_a.m_x;
		const double l_dy = f_b.m_y - f_a.m_y;

		return l_dx * l_dx + l_dy * l_dy;
	}

	/**
	* @brief Computes the distance between two points
	* @param f_a First point
	* @param f_b Second point
	* @return Euclidean distance
	*/
	inline static double Distance(const CPoint2& f_a, const CPoint2& f_b)
	{
		return std::sqrt(DistanceSq(f_a, f_b));
	}

	/**
	* @brief Computes the signed 2D cross product of two vectors
	*
	* Despite its name, the result is signed (it is not the absolute value):
	* it equals v1.x * v2.y - v2.x * v1.y, i.e. the same value returned by Cross().
	*
	* @param f_v1 First vector
	* @param f_v2 Second vector
	* @return Positive if @p f_v2 is counter-clockwise from @p f_v1, negative if clockwise, 0 if collinear
	* @see Cross
	*/
	inline double CrossProductMagnitude(const CPoint2& f_v1, const CPoint2& f_v2)
	{
		return (f_v1.m_pc[0] * f_v2.m_pc[1] - f_v2.m_pc[0] * f_v1.m_pc[1]);
	}
}
#endif