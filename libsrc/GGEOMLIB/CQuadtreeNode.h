
#ifndef H_QUADTREE_NODE
#define H_QUADTREE_NODE

#pragma once

#include <vector>
#include <unordered_set>
#include <memory>

#include "CAARectangle.h"
#include "CSegment2D.h"

namespace ggeomlib
{
	/**
	* @class CQuadtreeNode
	* @brief Represents a node in the spatial quadtree.
	*/
	class CQuadtreeNode
	{
	public:
		/**
		* @brief Constructs a quadtree node.
		* @param f_bounds The bounds of the node.
		* @param f_depth Current depth of the node in the tree.
		*/
		CQuadtreeNode(const CAARectangle& f_bounds, 
					  int f_depth = 0) :
						m_bounds(f_bounds),
						m_depth(f_depth)
		{
		}

		/**
		* @brief Assigns segments to this node.
		* @param f_segments List of segments to assign.
		*/
		void AssignSegments(const std::vector<IGeometricalObject2D*>& f_segments)
		{
			m_segments = f_segments;
		}
		/**
		* @brief Builds the subtree recursively from this node.
		* @param f_max_depth Maximum allowed tree depth.
		* @param f_max_objects_per_node Max segments per node before subdivision.
		*/
		void Build(int f_max_depth, int f_max_objects_per_node)
		{
			if (m_segments.size() <= f_max_objects_per_node || m_depth >= f_max_depth)
			{
				return;
			}

			Subdivide();

			for (auto seg : m_segments)
			{
				for (auto& child : m_children)
				{
					if(seg->IntersectsWith2DBox(child->m_bounds.GetXMin(), 
											    child->m_bounds.GetYMin(),
											    child->m_bounds.GetXMax(), 
											    child->m_bounds.GetYMax()))
					{
						child->m_segments.push_back(seg);
					}
				}
			}

			m_segments.clear();

			for (auto& child : m_children)
			{
				child->Build(f_max_depth, f_max_objects_per_node);
			}
		}
		/**
		* @brief Queries this node and its children for segments intersecting the query region.
		* @param f_query_rect The region to query.
		* @param f_segments_in_boxes Output list of intersecting segments.
		* @param f_strict_check If True, then only segments that collide with the query box will be returned
		*						If False, then all items inside leaves colliding  with the query box will be returned
		*/
		void QueryRegion(const CAARectangle& f_query_rect,
						std::vector<IGeometricalObject2D*>& f_segments_in_boxes,
						const bool f_strict_check) const
		{
			std::unordered_set<IGeometricalObject2D*> l_visited_segs;
			QueryRegionInternal(f_query_rect, 
								f_segments_in_boxes, 
								l_visited_segs, 
								f_strict_check);
		}

	private:
		/**
		* @brief Subdivides the current node into four children.
		*/
		void Subdivide()
		{
			const CPoint2& min = m_bounds.GetLowerLeftVertex();
			const CPoint2& max = m_bounds.GetUpperRightVertex();
			CPoint2 center = (min + max) * 0.5f;

			m_children[0] = std::make_unique<CQuadtreeNode>(CAARectangle(min, center), m_depth + 1);
			m_children[1] = std::make_unique<CQuadtreeNode>(CAARectangle(CPoint2(center.m_x, min.m_y), CPoint2(max.m_x, center.m_y)), m_depth + 1);
			m_children[2] = std::make_unique<CQuadtreeNode>(CAARectangle(CPoint2(min.m_x, center.m_y), CPoint2(center.m_x, max.m_y)), m_depth + 1);
			m_children[3] = std::make_unique<CQuadtreeNode>(CAARectangle(center, max), m_depth + 1);
		}
		/**
		* @brief Internal recursive region query.
		* @param f_query_rect Query region.
		* @param out_segments Accumulated list of found segments.
		* @param visited Set of already visited segments to avoid duplicates.
		* @param f_strict_check If True, then only segments that collide with the query box will be returned
		*						If False, then all items inside leaves colliding  with the query box will be returned
		*/
		void QueryRegionInternal(const CAARectangle& f_query_rect,
								 std::vector<IGeometricalObject2D*>& f_out_segments,
								 std::unordered_set<IGeometricalObject2D*>& f_visited_segs,
								 const bool f_strict_check) const
		{
			if (!cgeom_algs::AreRectangleOverlapped(m_bounds, f_query_rect))
			{
				return;
			}

			if (IsLeaf())
			{
				for (IGeometricalObject2D* seg : m_segments)
				{
					if(seg->IntersectsWith2DBox(f_query_rect.GetXMin(), 
												f_query_rect.GetYMin(), 
												f_query_rect.GetXMax(), 
												f_query_rect.GetYMax()))
					{
						if (f_visited_segs.insert(seg).second)
						{
							f_out_segments.push_back(seg);
						}
					}
				}
			}
			else
			{
				for (const auto& child : m_children)
				{
					if (child)
					{
						child->QueryRegionInternal(f_query_rect, f_out_segments, f_visited_segs, f_strict_check);
					}
				}
			}
		}
		/**
		* @brief Checks if the node is a leaf (has no children).
		* @return True if this node is a leaf.
		*/
		bool IsLeaf() const
		{
			for (const auto& child : m_children)
			{
				if (child) return false;
			}
			return true;
		}

		/**
		* Spatial bounds of this node.
		*/
		CAARectangle m_bounds;
		/**
		* Depth of the node in the tree.
		*/
		int m_depth;
		/**
		* Segments contained in this node(leaf only).
		*/
		std::vector<IGeometricalObject2D*> m_segments;
		/**
		* Child nodes(SW, SE, NW, NE).
		*/
		std::unique_ptr<CQuadtreeNode> m_children[4];
	};
}

#endif
