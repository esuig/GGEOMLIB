
#ifndef H_OCTREE_NODE
#define H_OCTREE_NODE

#pragma once

#include <memory>
#include <vector>
#include <unordered_set>

#include "CTriangle.h"
#include "CAABox.h"
#include "IGraphInterface.h"

namespace ggeomlib
{
	/**
	* @class COctreeNode
	* @brief Represents a node in an octree structure for spatial partitioning of 3D triangles.
	*
	* A COctreeNode holds a bounding box and a list of triangle references. It can subdivide
	* itself into 8 child nodes for more granular spatial representation, based on depth and
	* triangle count thresholds.
	*/
	class COctreeNode
	{
	public:
		/**
		* @brief Constructs an octree node with a bounding box and an optional depth level.
		*
		* @param f_bounding_box The axis-aligned bounding box (AABB) defining this node's spatial region.
		* @param f_depth The depth of this node in the octree hierarchy. Defaults to 0 for root.
		*/
		COctreeNode(const CAABox& f_bounding_box, int f_depth = 0) :
						m_bounds(f_bounding_box),
						m_depth(f_depth)
		{

		}

		/**
		* @brief Recursively builds the subtree starting from this node.
		*
		* Subdivides the node if it contains more triangles than the allowed threshold and
		* the maximum depth has not been reached.
		*
		* @param f_max_depth Maximum allowed depth for subdivision.
		* @param f_max_triangles_per_node Maximum triangles allowed in a node before subdivision occurs.
		*/
		inline void Build(const int f_max_depth, const int f_max_triangles_per_node)
		{
			if (m_objects.size() <= f_max_triangles_per_node || m_depth >= f_max_depth)
			{
				return;
			}

			SubdivideNode();

			for (const auto& l_obj_ptr : m_objects)
			{
				for (auto& child : m_children)
				{
					if (l_obj_ptr->IntersectsWith3DBox(child->GetAABB().GetLLVertex().m_x, child->GetAABB().GetLLVertex().m_y, child->GetAABB().GetLLVertex().m_z,
													   child->GetAABB().GetURVertex().m_x, child->GetAABB().GetURVertex().m_y, child->GetAABB().GetURVertex().m_z))
					{
						child->m_objects.push_back(l_obj_ptr);
					}
				}
			}

			m_objects.clear();

			for (auto& child : m_children)
			{
				child->Build(f_max_depth, f_max_triangles_per_node);
			}
		}

		/**
		* @brief Assigns a collection of objects to this node.
		*
		* Typically called at the root node before initiating the build process
		*
		* @param f_objects Vector of object pointers to be assigned to this node
		*/
		inline void AssignObjects(const std::vector<IGeometricalObject3D*>& f_objects)
		{
			m_objects = f_objects;
		}

		/**
		* @brief Adds a single object to this node.
		*
		* @param f_tri Pointer to the object to be added.
		*/
		inline void AddObject(IGeometricalObject3D* f_object)
		{
			m_objects.emplace_back(f_object);
		}

		/**
		* @brief Returns the axis-aligned bounding box (AABB) of this node.
		*
		* @return Reference to the node's bounding box.
		*/
		inline const CAABox& GetAABB() const
		{
			return m_bounds;
		}

		/**
		* @brief Checks whether the current octree node is a leaf.
		*
		* A node is considered a leaf if it contains any triangles.
		*
		* @return true if the node is a leaf (i.e., contains one or more triangles), false if it is a branch node.
		*/
		inline bool IsLeaf() const
		{
			if (m_objects.size() > 0)
			{
				return true;
			}
			return false;
		}

		/**
		* @brief Performs a raycast query on this octree node to find intersecting triangles.
		*
		* The function first checks if the ray intersects the node's bounding box.
		* If no intersection occurs, it returns immediately.
		* If the node is a leaf (i.e., has no children), it adds all triangles stored in this node to the output vector.
		* Otherwise, it recursively performs the raycast on each non-null child node.
		*
		* @param f_ray The ray used for intersection testing.
		* @param out_triangles Output vector that will be populated with pointers to triangles
		*        found in this node or its descendants that intersect the ray.
		*/
		inline void Raycast(const CRay& f_ray, std::vector<IGeometricalObject3D*>& f_out_objects) const
		{
			std::unordered_set<IGeometricalObject3D*> l_visited_triangles;
			RaycastInternal(f_ray, f_out_objects, l_visited_triangles);
		}

	private:
		/**
		* @brief Subdivides the node into 8 child octants.
		*
		* Each child node receives a portion of this node's AABB based on spatial partitioning.
		*/
		inline void SubdivideNode()
		{
			CPoint3 size((m_bounds.GetURVertex().m_x - m_bounds.GetLLVertex().m_x) / 2.0f,
						 (m_bounds.GetURVertex().m_y - m_bounds.GetLLVertex().m_y) / 2.0f,
						 (m_bounds.GetURVertex().m_z - m_bounds.GetLLVertex().m_z) / 2.0f);

			for (int i = 0; i < 8; ++i)
			{
				double l_offset_x = 0.0f;
				double l_offset_y = 0.0f;
				double l_offset_z = 0.0f;

				if (i & 1)
				{
					l_offset_x = size.m_x;
				}
				if (i & 2)
				{
					l_offset_y = size.m_y;
				}
				if (i & 4)
				{
					l_offset_z = size.m_z;
				}

				CPoint3 l_offset(l_offset_x, l_offset_y, l_offset_z);

				CPoint3 l_new_min(m_bounds.GetLLVertex().m_x + l_offset.m_x,
								  m_bounds.GetLLVertex().m_y + l_offset.m_y,
								  m_bounds.GetLLVertex().m_z + l_offset.m_z);
				CPoint3 l_new_max(l_new_min.m_x + size.m_x,
								  l_new_min.m_y + size.m_y,
								  l_new_min.m_z + size.m_z);
				m_children[i] = std::make_unique<COctreeNode>(CAABox(l_new_min, l_new_max), m_depth + 1);
			}
		}

		/**
		* @brief Internal helper for performing a raycast query on this octree node with duplicate triangle filtering.
		*
		* This method checks if the ray intersects the node's bounding box. If not, it returns immediately.
		* If the node is a leaf, it adds all triangles that have not yet been visited to the output vector,
		* marking them as visited to avoid duplicates.
		* If the node is not a leaf, it recursively calls this method on each non-null child node.
		*
		* @param f_ray The ray used for intersection testing.
		* @param out_triangles Output vector to be populated with pointers to triangles intersected by the ray.
		* @param f_visited_tris Set of pointers to triangles already added, used to avoid duplicates.
		*/
		inline void RaycastInternal(const CRay& f_ray,
									std::vector<IGeometricalObject3D*>& out_triangles,
									std::unordered_set<IGeometricalObject3D*>& f_visited_tris) const
		{
			CPoint3 l_pt_int;
			if (!m_bounds.ComputeIntersWithRay(f_ray, DOUBLE_PRECISION_EPSILON, l_pt_int))
				return;

			bool is_leaf = true;
			for (const auto& child : m_children)
			{
				if (child)
				{
					is_leaf = false;
					break;
				}
			}

			if (is_leaf)
			{
				for (IGeometricalObject3D* l_obj_ptr : m_objects)
				{
					if (f_visited_tris.find(l_obj_ptr) == f_visited_tris.end())
					{
						f_visited_tris.insert(l_obj_ptr);
						out_triangles.push_back(l_obj_ptr);
					}
				}
				return;
			}

			for (const auto& child : m_children)
			{
				if (child)
				{
					child->RaycastInternal(f_ray, out_triangles, f_visited_tris);
				}
			}
		}

		CAABox m_bounds;
		std::vector<IGeometricalObject3D*> m_objects;
		std::unique_ptr<COctreeNode> m_children[8];
		int m_depth;
	};
}

#endif
