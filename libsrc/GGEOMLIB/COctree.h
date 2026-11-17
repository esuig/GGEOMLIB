
#ifndef H_OCTREE
#define H_OCTREE

#pragma once

#include <memory>
#include "COctreeNode.h"
#include "CAABox.h"
#include "CTriangle.h"

namespace ggeomlib
{
	/**
	* @class COctree
	* @brief Represents an octree spatial data structure used for organizing 3D triangles.
	*
	* The COctree class provides functionality to construct and build a spatial partitioning
	* structure (octree) for efficient triangle queries, typically used in graphics, collision
	* detection, and ray tracing applications.
	*/
	class COctree
	{
	public:
		/**
		* @brief Instantiate an octree with specified spatial bounds and configuration parameters.
		*
		* @param f_bounds The axis-aligned bounding box that defines the spatial extent of the octree.
		* @param f_max_depth The maximum allowed depth of the octree.
		* @param f_max_triangles_per_node The maximum number of triangles per node before subdivision.
		*/
		COctree(const CAABox& f_bounds,
				const int f_max_depth = 9,
				const int f_max_triangles_per_node = 32) :
				m_max_depth(f_max_depth),
				m_max_triangles_per_node(f_max_triangles_per_node)
		{
			m_root_node = std::make_unique<COctreeNode>(f_bounds, 0);
		}

		/**
		* @brief Builds the octree structure using the provided list of triangles.
		*
		* This method assigns the input triangles to the root node and recursively subdivides
		* the octree as needed based on the configuration parameters.
		*
		* @param f_triangles A vector of pointers to constant CTriangle objects to be inserted into the octree.
		*/
		inline void Build(const std::vector<IGeometricalObject3D*>& f_objects)
		{
			m_root_node->AssignObjects(f_objects);
			m_root_node->Build(m_max_depth, m_max_triangles_per_node);
		}

		/**
		* @brief Performs a raycast query on the octree to find triangles intersecting with the given ray.
		*
		* This function starts the raycast from the root node. It collects all triangles
		* located in octree nodes intersected by the ray into the provided vector.
		*
		* @param f_ray The ray used for intersection testing.
		* @param f_triangles_in_intersected_octants Output vector that will be populated with pointers to triangles
		*        found in intersected octree nodes.
		*/
		void Raycast(const CRay& f_ray,
					 std::vector<IGeometricalObject3D*>& f_triangles_in_intersected_octants) const
		{
			if (m_root_node)
			{
				m_root_node->Raycast(f_ray, f_triangles_in_intersected_octants);
			}
		}


	private:
		std::unique_ptr<COctreeNode> m_root_node;
		const int m_max_depth;
		const int m_max_triangles_per_node;
	};
}

#endif
