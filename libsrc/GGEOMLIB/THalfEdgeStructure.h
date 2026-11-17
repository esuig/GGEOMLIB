
#pragma once

#ifndef H_CHALFEDGE_STRUCTURE
#define H_CHALFEDGE_STRUCTURE
#pragma once

#include <vector>
#include <unordered_map>
#include <utility>
#include <string>

namespace ggeomlib
{
	namespace triangle_soup
	{
		static constexpr int HE_INVALID = -1;

		/**
		* @struct CHalfEdge
		* @brief A single directed half-edge in a half-edge structure.
		*
		* Each undirected mesh edge (v_i, v_j) is split into two directed
		* half-edges: (v_i -> v_j) and (v_j -> v_i). These are called twins.
		* Every triangle owns exactly three half-edges that form a loop via
		* the next/prev links.
		*/
		struct THalfEdge
		{
			int m_index;
			int m_vertex_origin;
			int m_face;
			int m_next;
			int m_prev;
			int m_twin;

			THalfEdge()
				: m_index(HE_INVALID)
				, m_vertex_origin(HE_INVALID)
				, m_face(HE_INVALID)
				, m_next(HE_INVALID)
				, m_prev(HE_INVALID)
				, m_twin(HE_INVALID)
			{
			}
		};

		/**
		* @enum half_edge_build_error
		* @brief Result codes for BuildHalfEdgeStructure().
		*/
		enum half_edge_build_error
		{
			HE_EXIT_NO_ERROR,
			HE_ERROR_EMPTY_MESH,
			HE_ERROR_NON_MANIFOLD_EDGE,
		};

		/**
		* @struct CHalfEdgeStructure
		* @brief Stores the complete half-edge structure built from a triangle soup,
		*        plus the results of manifold and boundary analysis.
		*/
		struct THalfEdgeStructure
		{
			/** @brief All half - edges, indexed by their m_index field*/
			std::vector<THalfEdge> m_half_edges;

			/** @brief For each vertex, one outgoing half - edge index(fan traversal entry point).
			* HE_INVALID if the vertex has no incident half-edge */
			std::vector<int> m_vertex_to_halfedge;

			/** @brief For each face, the index of its first half-edge (he[i], he[i+1], he[i+2]) */
			std::vector<int> m_face_to_halfedge;

			/** @brief Indices of boundary (open) half-edges (m_twin == HE_INVALID) */
			std::vector<int> m_boundary_halfedges;

			/** @brief Each entry is one closed loop of boundary half-edge indices.
			* A single connected loop corresponds to one hole/open border */
			std::vector<std::vector<int>> m_boundary_loops;

			/** @brief True if the mesh is manifold (every edge shared by at most 2 triangles) */
			bool m_is_manifold = true;

			/** @brief True if the mesh is closed (no boundary half-edges found) */
			bool m_is_closed = true;

			/** @brief Number of distinct boundary loops (holes / open borders) */
			int m_boundary_loop_count = 0;
		};
	}
}

#endif
