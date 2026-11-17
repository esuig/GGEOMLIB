
#ifndef H_QUADTREE
#define H_QUADTREE

#pragma once

#include <memory>
#include <vector>

#include "CAARectangle.h"
#include "CSegment2D.h"
#include "CQuadtreeNode.h"


namespace ggeomlib
{
	/**
	* @class CQuadtree
	* @brief Spatial quadtree for efficient 2D segment management and querying.
	*/
	class CQuadtree
	{
	public:
		/**
		* @brief Constructs a quadtree with given spatial bounds and parameters.
		* @param f_bounds The spatial bounds of the root node.
		* @param f_max_depth Maximum depth allowed for the tree.
		* @param f_max_segments_per_node Maximum segments a node can contain before subdivision.
		*/
		CQuadtree(const CAARectangle& f_bounds,
				  int f_max_depth = 9,
				  int f_max_objects_per_node = 16):
						m_max_depth(f_max_depth),
						m_max_objects_per_node(f_max_objects_per_node)
		{
			m_root_node = std::make_unique<CQuadtreeNode>(f_bounds, 0);
		}

		/**
		* @brief Builds the quadtree from the provided list of segments.
		* @param f_segments A list of pointers to the segments to insert.
		*/
		inline void Build(const std::vector<IGeometricalObject2D*>& f_segments)
		{
			m_root_node->AssignSegments(f_segments);
			m_root_node->Build(m_max_depth, m_max_objects_per_node);
		}
		/**
		* @brief Queries the quadtree for segments intersecting the given rectangle.
		* @param f_query_rect The query region.
		* @param f_out_segments Output list filled with intersecting segments.
		* @param f_strict_check If True, then only segments that collide with the query box will be returned
		*						If False, then all items inside leaves colliding  with the query box will be returned
		*/
		inline void QueryRegion(const CAARectangle& f_query_rect, 
							    std::vector<IGeometricalObject2D*>& f_out_segments,
							    const bool f_strict_check = true) const
		{
			f_out_segments.clear();
			if (m_root_node)
			{
				m_root_node->QueryRegion(f_query_rect, f_out_segments, f_strict_check);
			}
		}

	private:
		/** 
		* Root node of the quadtree.
		*/
		std::unique_ptr<class CQuadtreeNode> m_root_node;
		/**
		* Maximum depth of the quadtree
		*/
		const int m_max_depth;
		/*
		* Maximum items per node before subdivision.
		*/
		const int m_max_objects_per_node;
	};
}

#endif
