
#ifndef H_CPOINT4
#define H_CPOINT4

#pragma once

#include <cmath>

namespace ggeomlib
{
	/**
	* @class CPoint4
	* @brief Rational (homogeneous) point in 4D space.
	*
	* Stores a point in homogeneous coordinates (x, y, z, w), where (x, y, z) are
	* the weighted Cartesian coordinates and w is the weight. The Cartesian point
	* is recovered by dividing by w (see DeHom()). Typically used for the control
	* points of rational curves and surfaces (e.g. NURBS).
	*/
	class CPoint4
	{
	public:

		/**
		* @brief Union to allow access to the point's data via array or named members.
		* https://www.gamedev.net/forums/topic/671481-most-efficient-way-of-designing-a-vector-class-in-3d/
		*/
		union
		{
			/**
			* Array access to coordinates(x, y, z)
			*/
			double m_pc[4];
			struct
			{
				/**
				* X Coordinate (weighted)
				*/
				double m_x;
				/**
				* Y Coordinate (weighted)
				*/
				double m_y;
				/**
				* Z Coordinate (weighted)
				*/
				double m_z;
				/**
				* Weight
				*/
				double m_w;
			};
		};


		/**
		* @brief Default constructor. Initializes all coordinates, including the weight, to zero.
		*/
		CPoint4()
		{
			m_x = 0.0;
			m_y = 0.0;
			m_z = 0.0;
			m_w = 0.0;
		}

		/**
		* @brief Constructs a point from its homogeneous coordinates.
		* @param f_x Weighted X coordinate
		* @param f_y Weighted Y coordinate
		* @param f_z Weighted Z coordinate
		* @param f_w Weight
		*/
		CPoint4(const double& f_x,
				const double& f_y,
				const double& f_z,
				const double& f_w)
		{
			m_pc[0] = f_x;
			m_pc[1] = f_y;
			m_pc[2] = f_z;
			m_pc[3] = f_w;
		}

		/**
		* @brief Constructs a homogeneous point from a Cartesian point and a weight
		*
		* The resulting coordinates are (w*x, w*y, w*z, w), so that DeHom() returns @p f_p
		*
		* @param f_p Cartesian point
		* @param f_w Weight
		*/
		CPoint4(const CPoint3& f_p, const double& f_w)
		{
			m_pc[0] = f_p.m_x * f_w;
			m_pc[1] = f_p.m_y * f_w;
			m_pc[2] = f_p.m_z * f_w;
			m_pc[3] = f_w;
		}

		/**
		* @brief Copy constructor
		* @param f_p Point to copy
		*/
		CPoint4(const CPoint4& f_p)
		{
			m_pc[0] = f_p.m_pc[0];
			m_pc[1] = f_p.m_pc[1];
			m_pc[2] = f_p.m_pc[2];
			m_pc[3] = f_p.m_pc[3];
		}

		/**
		* @brief Copy assignment operator
		* @param f_p Point to copy
		* @return Reference to this point
		*/
		inline CPoint4& operator=(const CPoint4& f_p)
		{
			m_pc[0] = f_p.m_pc[0];
			m_pc[1] = f_p.m_pc[1];
			m_pc[2] = f_p.m_pc[2];
			m_pc[3] = f_p.m_pc[3];

			return *this;
		}

		/**
		* @brief Computes the squared 4D length (x^2 + y^2 + z^2 + w^2)
		* @return Squared length.
		*/
		inline double LengthSq() const
		{
			return m_x * m_x + m_y * m_y + m_z * m_z + m_w * m_w;
		}

		/**
		* @brief Computes the 4D Euclidean length, including the weight
		* @return Length.
		*/
		inline double Length() const
		{
			return std::sqrt(LengthSq());
		}

		/**
		* @brief Normalizes this point to unit 4D length
		*
		* If the length is zero, the point is left unchanged
		*
		* @return Reference to this point
		*/
		inline CPoint4& UnitizeInPlace()
		{
			const double l_length = Length();
			if (l_length > 0.0)
			{
				m_x /= l_length;
				m_y /= l_length;
				m_z /= l_length;
				m_w /= l_length;
			}

			return *this;
		}

		/**
		* @brief Returns a copy of this point normalized to unit 4D length
		*
		* If the length is zero, an unchanged copy is returned
		*
		* @return Normalized copy
		*/
		inline CPoint4 GetUnitVector() const
		{
			const double l_length = Length();
			if (l_length > 0.0)
			{
				return CPoint4(m_x / l_length, m_y / l_length, m_z / l_length, m_w / l_length);
			}

			return *this;
		}

		/**
		* @brief Projects the point to Cartesian space by dividing by the weight
		* @pre m_w != 0
		* @return Cartesian point (x/w, y/w, z/w)
		*/
		inline CPoint3 DeHom() const
		{
			const double l_iw = 1.0 / m_w;
			return CPoint3(m_x * l_iw, m_y * l_iw, m_z * l_iw);
		}

		/**
		* @brief Drops the weight without dividing by it
		* @return Point (x, y, z) with the weighted coordinates as they are
		*/
		inline CPoint3 ToCPoint3() const
		{
			return CPoint3(m_x, m_y, m_z);
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
		* @brief Component-wise addition, including the weight
		* @param f_p Point to add
		* @return Sum of the two points
		*/
		inline CPoint4 operator+(const CPoint4& f_p) const
		{
			return CPoint4(m_x + f_p.m_x, m_y + f_p.m_y, m_z + f_p.m_z, m_w + f_p.m_w);
		}

		/**
		* @brief Component-wise subtraction, including the weight
		* @param f_p Point to subtract
		* @return Difference of the two points
		*/
		inline CPoint4 operator-(const CPoint4& f_p) const
		{
			return CPoint4(m_x - f_p.m_x, m_y - f_p.m_y, m_z - f_p.m_z, m_w - f_p.m_w);
		}
		/**
		* @brief Multiplies all four components by a scalar
		* @param f_scalar Scale factor
		* @return Scaled point
		*/
		inline CPoint4 operator*(double f_scalar) const
		{
			return CPoint4(m_x * f_scalar, m_y * f_scalar, m_z * f_scalar, m_w * f_scalar);
		}
		/**
		* @brief Divides all four components by a scalar
		* @param f_scalar Divisor
		* @pre f_scalar != 0
		* @return Scaled point
		*/
		inline CPoint4 operator/(double f_scalar) const
		{
			return CPoint4(m_x / f_scalar, m_y / f_scalar, m_z / f_scalar, m_w / f_scalar);
		}

		/**
		* @brief Multiplies all four components of a point by a scalar (scalar on the left)
		* @param f_scalar Scale factor
		* @param f_p Point to scale
		* @return Scaled point
		*/
		inline friend CPoint4 operator*(double f_scalar, const CPoint4& f_p)
		{
			return CPoint4(f_p.m_x * f_scalar, f_p.m_y * f_scalar, f_p.m_z * f_scalar, f_p.m_w * f_scalar);
		}

	private:

	};

	/**
	* @brief Computes the 4D dot product of two homogeneous points
	* @param f_a First point
	* @param f_b Second point
	* @return x_a*x_b + y_a*y_b + z_a*z_b + w_a*w_b
	*/
	inline static double Dot(const CPoint4& f_a, const CPoint4& f_b)
	{
		return f_a.m_x * f_b.m_x + f_a.m_y * f_b.m_y + f_a.m_z * f_b.m_z + f_a.m_w * f_b.m_w;
	}

	/**
	* @brief Computes the squared 4D distance between two homogeneous points
	*
	* @note This is the distance in homogeneous space, not between the Cartesian
	*       points. For that, use DistanceSq(f_a.DeHom(), f_b.DeHom())
	*
	* @param f_a First point
	* @param f_b Second point
	* @return Squared 4D distance
	*/
	inline static double DistanceSq(const CPoint4& f_a, const CPoint4& f_b)
	{
		const double l_dx = f_b.m_x - f_a.m_x;
		const double l_dy = f_b.m_y - f_a.m_y;
		const double l_dz = f_b.m_z - f_a.m_z;
		const double l_dw = f_b.m_w - f_a.m_w;

		return l_dx * l_dx + l_dy * l_dy + l_dz * l_dz + l_dw * l_dw;
	}

	/**
	* @brief Computes the 4D distance between two homogeneous points
	*
	* @note This is the distance in homogeneous space, not between the Cartesian
	*       points. For that, use Distance(f_a.DeHom(), f_b.DeHom())
	*
	* @param f_a First point
	* @param f_b Second point
	* @return 4D distance
	*/
	inline static double Distance(const CPoint4& f_a, const CPoint4& f_b)
	{
		return std::sqrt(DistanceSq(f_a, f_b));
	}

	/**
	* @brief Linearly interpolates between two homogeneous points, including the weight
	*
	* Interpolating in homogeneous space and then calling DeHom() gives the
	* rational (projectively correct) interpolation of the Cartesian points
	*
	* @param f_a Start point, returned when @p f_t = 0
	* @param f_b End point, returned when @p f_t = 1
	* @param f_t Interpolation parameter
	* @return f_a * (1 - f_t) + f_b * f_t
	*/
	inline static CPoint4 Lerp(const CPoint4& f_a, const CPoint4& f_b, double f_t)
	{
		return f_a * (1.0 - f_t) + f_b * f_t;
	}
}
#endif