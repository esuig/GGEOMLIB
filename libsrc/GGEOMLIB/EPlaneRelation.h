
#ifndef H_PLANE_RELATION
#define H_PLANE_RELATION

namespace ggeomlib
{
	/**
	* @enum PlaneRelation
	* @brief Represents the spatial relationship between a geometrical item and a plane.
	*/
	enum EPlaneRelation
	{
		/** @brief Entire item is on the opposite side of the plane normal */ 
		FullyBehind,
		/** @brief Item intersects or lie on the plane */
		Intersecting,
		/** @brief Item box is in front of or on the same side as the plane normal */
		FullyInFront
	};
}

#endif