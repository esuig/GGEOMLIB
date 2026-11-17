
#ifndef H_CPOINT3
#define H_CPOINT3

#pragma once

#include <algorithm>

#include "IGeometricalObject3D.h"

namespace ggeomlib
{
	/**
	* @class CPoint3
	* @brief 3D point or vector with double precision coordinates
	*
	* Besides its coordinates, each point carries three auxiliary integer values and
	* three auxiliary double values that can be used to attach user data (e.g. ids,
	* flags, weights).
	*
	* @warning The copy constructor does NOT copy the auxiliary values (they are reset
	* to zero), while the implicitly generated copy assignment operator DOES copy them.
	* Arithmetic operators and GetUnitVector() always return points with zeroed
	* auxiliary values.
	*/
	class CPoint3 : public IGeometricalObject3D
	{
	public:

		/**
		* @brief Union to allow access to the vector's data via array or named members.
		* https://www.gamedev.net/forums/topic/671481-most-efficient-way-of-designing-a-vector-class-in-3d/
		*/
		union
		{
			/**
			* Array access to coordinates(x, y, z)
			*/
			double m_pc[3];
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
				/**
				* Z Coordinate
				*/
				double m_z;
			};
		};


		/**
		* @brief Default constructor, initializes all coordinates and all auxiliary values to zero
		*/
		CPoint3():
			m_x(0.0),
			m_y(0.0),
			m_z(0.0),
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
		* @param f_z Z coordinate
		*/
		CPoint3(const double& f_x,
				const double& f_y,
				const double& f_z):
					m_val_int_1(0),
					m_val_int_2(0),
					m_val_int_3(0),
					m_val_double_1(0.0),
					m_val_double_2(0.0),
					m_val_double_3(0.0)
		{
			m_pc[0] = f_x;
			m_pc[1] = f_y;
			m_pc[2] = f_z;
		}

		/**
		* @brief Copy constructor
		*
		* Copies the coordinates only: the auxiliary values of the new point are
		* initialized to zero and are NOT copied from @p f_p.
		*
		* @param f_p Point to copy
		*/
		CPoint3(const CPoint3& f_p):
			m_val_int_1(0),
			m_val_int_2(0),
			m_val_int_3(0),
			m_val_double_1(0.0),
			m_val_double_2(0.0),
			m_val_double_3(0.0)
		{
			m_pc[0] = f_p.m_pc[0];
			m_pc[1] = f_p.m_pc[1];
			m_pc[2] = f_p.m_pc[2];
		}

		/**
		* @brief Computes the axis-aligned bounding box of the point
		*
		* For a point the bounding box is degenerate: minimum and maximum both equal the point's coordinates.
		*
		* @param[out] f_x_min Minimum X coordinate
		* @param[out] f_y_min Minimum Y coordinate
		* @param[out] f_z_min Minimum Z coordinate
		* @param[out] f_x_max Maximum X coordinate
		* @param[out] f_y_max Maximum Y coordinate
		* @param[out] f_z_max Maximum Z coordinate
		*/
		inline void Bounds(double& f_x_min,
						   double& f_y_min,
						   double& f_z_min,
						   double& f_x_max,
						   double& f_y_max,
						   double& f_z_max) const override
		{
			f_x_min = m_x;
			f_y_min = m_y;
			f_z_min = m_z;
			f_x_max = m_x;
			f_y_max = m_y;
			f_z_max = m_z;
		}

		/**
		* @brief Tests whether the point lies inside an axis-aligned 3D box
		*
		* Box boundaries are inclusive.
		*
		* @param f_x_min Minimum X coordinate of the box
		* @param f_y_min Minimum Y coordinate of the box
		* @param f_z_min Minimum Z coordinate of the box
		* @param f_x_max Maximum X coordinate of the box
		* @param f_y_max Maximum Y coordinate of the box
		* @param f_z_max Maximum Z coordinate of the box
		* @return true if the point is inside or on the boundary of the box, false otherwise
		*/
		inline bool IntersectsWith3DBox(const double f_x_min,
										const double f_y_min,
										const double f_z_min,
										const double f_x_max,
										const double f_y_max,
										const double f_z_max) const override
		{
			return m_x >= f_x_min && m_x <= f_x_max &&
				   m_y >= f_y_min && m_y <= f_y_max &&
				   m_z >= f_z_min && m_z <= f_z_max;
		}

		/**
		* @brief Computes the squared length (x^2 + y^2 + z^2)
		* @return Squared length
		*/
		inline double LengthSq() const
		{
			return m_x * m_x + m_y * m_y + m_z * m_z;
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
		inline CPoint3& UnitizeInPlace()
		{
			const double l_length = Length();
			if (l_length > 0.0)
			{
				m_x /= l_length;
				m_y /= l_length;
				m_z /= l_length;
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
		inline CPoint3 GetUnitVector() const
		{
			const double l_length = Length();
			if (l_length > 0.0)
			{
				return CPoint3(m_x / l_length, m_y / l_length, m_z / l_length);
			}

			return *this;
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
		* @brief Sets the first auxiliary integer value
		* @param f_int_val New value
		*/
		inline void SetIntVal1(const int f_int_val)
		{
			m_val_int_1 = f_int_val;
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
		* @brief Sets the second auxiliary integer value
		* @param f_int_val New value
		*/
		inline void SetIntVal2(const int f_int_val)
		{
			m_val_int_2 = f_int_val;
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
		* @brief Sets the third auxiliary integer value
		* @param f_int_val New value
		*/
		inline void SetIntVal3(const int f_int_val)
		{
			m_val_int_3 = f_int_val;
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
		* @brief Sets the first auxiliary double value
		* @param f_double_val New value
		*/
		inline void SetDoubleVal1(const double f_double_val)
		{
			m_val_double_1 = f_double_val;
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
		* @brief Sets the second auxiliary double value
		* @param f_double_val New value
		*/
		inline void SetDoubleVal2(const double f_double_val)
		{
			m_val_double_2 = f_double_val;
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
		* @brief Sets the third auxiliary double value
		* @param f_double_val New value
		*/
		inline void SetDoubleVal3(const double f_double_val)
		{
			m_val_double_3 = f_double_val;
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
		inline CPoint3 operator+(const CPoint3& f_p) const
		{
			return CPoint3(m_x + f_p.m_x, m_y + f_p.m_y, m_z + f_p.m_z);
		}
		/**
		* @brief Component-wise subtraction
		* @param f_p Point to subtract
		* @return Difference of the two points
		*/
		inline CPoint3 operator-(const CPoint3& f_p) const
		{
			return CPoint3(m_x - f_p.m_x, m_y - f_p.m_y, m_z - f_p.m_z);
		}
		/**
		* @brief Multiplies all three components by a scalar
		* @param f_scalar Scale factor
		* @return Scaled vector
		*/
		inline CPoint3 operator*(double f_scalar) const
		{
			return CPoint3(m_x * f_scalar, m_y * f_scalar, m_z * f_scalar);
		}
		/**
		* @brief Divides all three components by a scalar
		* @param f_scalar Divisor
		* @pre f_scalar != 0
		* @return Scaled vector
		*/
		inline CPoint3 operator/(double f_scalar) const
		{
			return CPoint3(m_x / f_scalar, m_y / f_scalar, m_z / f_scalar);
		}
		/**
		* @brief Multiplies all three components of a vector by a scalar (scalar on the left)
		* @param f_scalar Scale factor
		* @param f_p Vector to scale
		* @return Scaled vector
		*/
		inline friend CPoint3 operator*(double f_scalar, const CPoint3& f_p)
		{
			return CPoint3(f_p.m_x * f_scalar, f_p.m_y * f_scalar, f_p.m_z * f_scalar);
		}
		/**
		* @brief Invert the vector
		* @return The negated vector
		*/
		inline CPoint3 operator-() const
		{
			return CPoint3(-m_x, -m_y, -m_z);
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
	* @return a.x * b.x + a.y * b.y + a.z * b.z
	*/
	inline static double Dot(const CPoint3& f_a, const CPoint3& f_b)
	{
		return f_a.m_x * f_b.m_x + f_a.m_y * f_b.m_y + f_a.m_z * f_b.m_z;
	}

	/**
	* @brief Computes the cross product of two vectors
	*
	* The result is perpendicular to both inputs, follows the right-hand rule,
	* and its length equals the area of the parallelogram spanned by @p f_a and @p f_b
	*
	* @param f_a First vector
	* @param f_b Second vector
	* @return a x b
	*/
	inline static CPoint3 Cross(const CPoint3& f_a, const CPoint3& f_b)
	{
		return CPoint3(f_a.m_y * f_b.m_z - f_a.m_z * f_b.m_y,
			f_a.m_z * f_b.m_x - f_a.m_x * f_b.m_z,
			f_a.m_x * f_b.m_y - f_a.m_y * f_b.m_x);
	}

	/**
	* @brief Computes the squared distance between two points
	* @param f_a First point
	* @param f_b Second point
	* @return Squared distance
	*/
	inline static double DistanceSq(const CPoint3& f_a, const CPoint3& f_b)
	{
		const double l_dx = f_b.m_x - f_a.m_x;
		const double l_dy = f_b.m_y - f_a.m_y;
		const double l_dz = f_b.m_z - f_a.m_z;

		return l_dx * l_dx + l_dy * l_dy + l_dz * l_dz;
	}

	/**
	* @brief Computes the distance between two points
	* @param f_a First point
	* @param f_b Second point
	* @return Euclidean distance
	*/
	inline static double Distance(const CPoint3& f_a, const CPoint3& f_b)
	{
		return sqrt(DistanceSq(f_a, f_b));
	}
}
#endif
