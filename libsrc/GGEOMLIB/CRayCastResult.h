
#ifndef H_CRAYCAST_RESULT
#define H_CRAYCAST_RESULT

#pragma once

#include "CPoint3.h"

namespace ggeomlib
{
	/**
	* @class CRayCastResult
	* @brief Represents the result of a ray casting operation.
	*
	* Stores information about the intersection point of a ray with a surface,
	* including the intersection location, surface normal at the intersection,
	* travelled distance along the ray, and whether the ray is exiting the model.
	*/
	class CRayCastResult
	{
	public:

		/**
		* @brief Constructs a ray cast result with given intersection data.
		* @param f_intersection_point The point where the ray intersects the surface.
		* @param f_exiting_the_model True if the ray is exiting the model at this intersection.
		* @param f_travelled_distance Distance travelled along the ray to the intersection point.
		*/
		CRayCastResult(CPoint3 f_intersection_point,
					   bool f_exiting_the_model,
					   double f_travelled_distance) :
			m_intersection_point(f_intersection_point),
			m_exiting_the_model(f_exiting_the_model),
			m_travelled_distance(f_travelled_distance)
		{

		}

		/**
		* @brief Default constructor initializing an empty ray cast result.
		*/
		CRayCastResult() :
			m_exiting_the_model(false),
			m_travelled_distance(0.0)
		{

		}

		/**
		* @brief Set the intersection point.
		* @param f_intersection_point The 3D intersection point.
		*/
		inline void SetIntersectionPoint(const CPoint3& f_intersection_point)
		{
			m_intersection_point = f_intersection_point;
		}
		/**
		* @brief Set the surface normal at the intersection point.
		* @param f_normal The normal vector at the intersection.
		*/
		inline void SetNormalAtIntersection(const CPoint3& f_normal)
		{
			m_normal = f_normal;
		}
		/**
		* @brief Set the travelled distance from the ray origin to the intersection.
		* @param f_travelled_distance The distance value.
		*/
		inline void SetTravelledDistance(const double f_travelled_distance)
		{
			m_travelled_distance = f_travelled_distance;
		}
		/**
		* @brief Set whether the intersection is exiting the model.
		* @param f_exiting_model True if exiting the model; false otherwise.
		*/
		inline void SetExitingModel(const bool f_exiting_model)
		{
			m_exiting_the_model = f_exiting_model;
		}
		/**
		* @brief Get the intersection point.
		* @return The 3D intersection point.
		*/
		inline const CPoint3& GetIntersectionPoint() const
		{
			return m_intersection_point;
		}
		/**
		* @brief Get the surface normal at the intersection point.
		* @return The normal vector.
		*/
		inline const CPoint3& GetNormalAtIntersection() const
		{
			return m_normal;
		}
		/**
		* @brief Get the travelled distance along the ray to the intersection.
		* @return The distance value.
		*/
		inline double GetTravelledDistance() const
		{
			return m_travelled_distance;
		}
		/**
		* @brief Check if the intersection represents exiting the model.
		* @return True if exiting the model, false if entering.
		*/
		inline bool GetExitingModel() const
		{
			return m_exiting_the_model;
		}

	private:

		CPoint3 m_intersection_point;
		double m_travelled_distance;
		bool m_exiting_the_model;
		CPoint3 m_normal;
	};
}

#endif
