
#ifndef H_CTRIANGLE_SOUP
#define H_CTRIANGLE_SOUP
#pragma once

#include <vector>

#include "CPoint3.h"
#include "CTriangle.h"
#include "THalfEdgeStructure.h"
#include "COctree.h"

namespace ggeomlib
{
	namespace triangle_soup
	{
		/**
		* @enum triangle_soup_error
		* @brief Represents possible errors or success states during STL file or data loading into a triangle soup.
		*/
		enum triangle_soup_error
		{
			EXIT_NO_ERROR,
			ERROR_INPUT_STL_GEOMETRY_NOT_EXISTING,
			ERROR_COULD_NOT_READ_INPUT_FILE,
			ERROR_INPUT_STL_FILE_EMPTY,
			ERROR_INCOHERENT_INPUT_DATA,

			ERROR_UNDEFINED,
		};

		/**
		* @class CTriangleSoup
		* @brief Handles the storage, loading, querying, and exporting of triangle mesh data from STL files or raw data.
		*/
		class CTriangleSoup
		{
		public:
			/** @brief Default constructor. */
			CTriangleSoup()
			{

			}

			/**
			* @brief Loads a triangle mesh from vertex and connectivity data. To load STL data from 
			* file one can use for example: https://github.com/sreiter/stl_reader
			*
			* @param f_vertexes_coords Flat array of vertex coordinates (x, y, z for each vertex).
			* @param f_triangle_connectivity Triangle indices referring to vertices in f_vertexes_coords.
			* @return triangle_soup_error Code indicating success or type of error.
			*/
			inline triangle_soup_error LoadStlModelFromData(const std::vector<double>& f_vertexes_coords,
															const std::vector<unsigned int>& f_triangle_connectivity)
			{
				int l_num_coords_vert_shared = (int)f_vertexes_coords.size();
				if ((l_num_coords_vert_shared % 3 != 0) || (f_triangle_connectivity.size() % 3 != 0))
				{
					return triangle_soup_error::ERROR_INCOHERENT_INPUT_DATA;
				}
				for (auto l_vert_id_it = f_triangle_connectivity.begin(); l_vert_id_it != f_triangle_connectivity.end(); l_vert_id_it++)
				{
					if ((*l_vert_id_it) > (f_vertexes_coords.size() / 3))
					{
						return triangle_soup_error::ERROR_INCOHERENT_INPUT_DATA;
					}
				}
				int l_counter = 0;
				m_vertexes_coords.reserve(f_vertexes_coords.size());
				for (int l_ivert = 0; l_ivert < l_num_coords_vert_shared; l_ivert++)
				{
					m_vertexes_coords.emplace_back(f_vertexes_coords[l_ivert]);
					l_counter += 1;
					if (l_counter == 3)
					{
						CPoint3 l_shared_vertex(f_vertexes_coords[l_ivert - 2], f_vertexes_coords[l_ivert - 1], f_vertexes_coords[l_ivert]);
						m_vertexes.push_back(l_shared_vertex);
						l_counter = 0;
					}
				}

				int l_ret_triangle_build = AssembleTriangles(f_triangle_connectivity);
				if (l_ret_triangle_build > 0)
				{
					switch (l_ret_triangle_build)
					{
					case 1:
						return triangle_soup_error::ERROR_INPUT_STL_FILE_EMPTY;
					default:
						return triangle_soup_error::ERROR_UNDEFINED;
					}
				}

				CalculateBoundingBox();

				return triangle_soup_error::EXIT_NO_ERROR;
			}

			/**
			* @brief Retrieves the coordinates of a vertex by index.
			*
			* @param f_vertex_index Index of the vertex.
			* @param f_x Output X coordinate.
			* @param f_y Output Y coordinate.
			* @param f_z Output Z coordinate.
			* @return true if vertex was found, false otherwise.
			*/
			inline bool GetVertexByIndex(int f_vertex_index,
										  double& f_x,
										  double& f_y,
										  double& f_z) const
			{
				if ((f_vertex_index < 0) || (f_vertex_index > (m_vertexes.size() - 1)))
				{
					return false;
				}

				f_x = m_vertexes[f_vertex_index].m_x;
				f_y = m_vertexes[f_vertex_index].m_y;
				f_z = m_vertexes[f_vertex_index].m_z;

				return true;
			}

			/**
			* @brief Gets vertex IDs that form a specific triangle.
			*
			* @param f_id_vert_1 Output vertex ID 1.
			* @param f_id_vert_2 Output vertex ID 2.
			* @param f_id_vert_3 Output vertex ID 3.
			* @param f_triangle_id ID of the triangle.
			* @return true if successful, false otherwise.
			*/
			inline bool GetTriangleVertexIdsByTriangleId(int& f_id_vert_1,
													     int& f_id_vert_2,
													     int& f_id_vert_3,
													     const int f_triangle_id) const
			{

				if ((f_triangle_id < 0) || (f_triangle_id > (m_triangles.size() - 1)))
				{
					return false;
				}

				f_id_vert_1 = m_triangles.at(f_triangle_id).GetVert1ID();
				f_id_vert_2 = m_triangles.at(f_triangle_id).GetVert2ID();
				f_id_vert_3 = m_triangles.at(f_triangle_id).GetVert3ID();

				return true;
			}
			/** @brief Returns the number of vertices in the triangle soup. */
			inline size_t GetVertexCount() const
			{
				return m_vertexes.size();
			}
			/** @brief Returns the number of triangles in the triangle soup. */
			inline size_t GetTriangleCount() const
			{
				return m_triangles.size();
			}
			/**
			* @brief Retrieves a triangle object by its index.
			*
			* @param f_triangle_id ID of the triangle.
			* @return Pointer to the triangle, or nullptr if not found.
			*/
			inline const CTriangle* GetTriangleById(const int f_triangle_id) const
			{
				if ((f_triangle_id > m_triangles.size() - 1) || f_triangle_id < 0)
				{
					return nullptr;
				}

				return &m_triangles[f_triangle_id];;
			}

			/**
			* @brief Computes the total surface area of the triangle soup (sum of triangle areas).
			*
			* @return The surface area in square meters.
			*/
			inline double CalculateSurfaceArea() const
			{
				double l_tot_surface_area = 0.0;
				for (auto l_triangle_it = m_triangles.begin(); l_triangle_it != m_triangles.end(); l_triangle_it++)
				{
					l_tot_surface_area += l_triangle_it->CalculateArea();
				}
				return l_tot_surface_area / (1e6);
			}

			/**
			* @brief Computes the signed volume of the triangle soup using the divergence theorem.
			*        The mesh must be closed (watertight) and consistently outward-oriented for
			*        the result to be meaningful. A negative result indicates inward-facing normals.
			*
			* @return The signed volume in cubic meters. Take std::fabs() if only the magnitude is needed.
			*/
			inline double CalculateVolume() const
			{

				double l_signed_volume = 0.0;

				for (const auto& l_tri : m_triangles)
				{
					const CPoint3& v1 = l_tri.GetVertex1Ref();
					const CPoint3& v2 = l_tri.GetVertex2Ref();
					const CPoint3& v3 = l_tri.GetVertex3Ref();

					l_signed_volume += v1.m_x * (v2.m_y * v3.m_z - v2.m_z * v3.m_y)
									   + v1.m_y * (v2.m_z * v3.m_x - v2.m_x * v3.m_z)
									   + v1.m_z * (v2.m_x * v3.m_y - v2.m_y * v3.m_x);
				}

				return (l_signed_volume / 6.0) / 1e9;
			}

			/**
			* @brief Exports the triangle soup to an STL file.
			*
			* @param f_file_path Output file path.
			* @return true if successful, false otherwise.
			*/
			inline bool ExportToSTLFile(const std::string& f_file_path)
			{
				if (m_triangles.size() == 0)
				{
					return false;
				}
				std::ofstream l_out_file(f_file_path, std::ios::binary);
				if (!l_out_file.is_open())
				{
					return false;
				}

				char header[80] = "File Generated by GGEOMLIB";
				l_out_file.write(header, 80);
				uint32_t l_triangle_count = (uint32_t)m_triangles.size();
				l_out_file.write((const char*)(&l_triangle_count), 4);

				for (auto l_tri_it = m_triangles.begin(); l_tri_it != m_triangles.end(); l_tri_it++)
				{
					float l_norm_x = (float)l_tri_it->GetNormal().m_x;
					float l_norm_y = (float)l_tri_it->GetNormal().m_y;
					float l_norm_z = (float)l_tri_it->GetNormal().m_z;
					l_out_file.write((const char*)(&l_norm_x), 4);
					l_out_file.write((const char*)(&l_norm_y), 4);
					l_out_file.write((const char*)(&l_norm_z), 4);

					float l_v1_x = (float)l_tri_it->GetVertex1Ref().m_x;
					float l_v1_y = (float)l_tri_it->GetVertex1Ref().m_y;
					float l_v1_z = (float)l_tri_it->GetVertex1Ref().m_z;
					l_out_file.write((const char*)(&l_v1_x), 4);
					l_out_file.write((const char*)(&l_v1_y), 4);
					l_out_file.write((const char*)(&l_v1_z), 4);

					float l_v2_x = (float)l_tri_it->GetVertex2Ref().m_x;
					float l_v2_y = (float)l_tri_it->GetVertex2Ref().m_y;
					float l_v2_z = (float)l_tri_it->GetVertex2Ref().m_z;
					l_out_file.write((const char*)(&l_v2_x), 4);
					l_out_file.write((const char*)(&l_v2_y), 4);
					l_out_file.write((const char*)(&l_v2_z), 4);

					float l_v3_x = (float)l_tri_it->GetVertex3Ref().m_x;
					float l_v3_y = (float)l_tri_it->GetVertex3Ref().m_y;
					float l_v3_z = (float)l_tri_it->GetVertex3Ref().m_z;
					l_out_file.write((const char*)(&l_v3_x), 4);
					l_out_file.write((const char*)(&l_v3_y), 4);
					l_out_file.write((const char*)(&l_v3_z), 4);

					uint16_t l_attr = 0;
					l_out_file.write((const char*)(&l_attr), 2);
				}

				l_out_file.close();
				return true;
			}

			/** @brief Returns the top-right corner of the 3D bounding box. */
			inline const CPoint3& GetTopRightBBoxVert() const
			{
				return m_top_right_bbox_vert;
			}

			/** @brief Returns the bottom-left corner of the 3D bounding box. */
			inline const CPoint3& GetBottomLeftBBoxVert() const
			{
				return m_bottom_left_bbox_vert;
			}

			/**
			* @brief Returns the raw coordinate list of shared vertices.
			*
			* @return Reference to vertex coordinate array (x, y, z repeated).
			*/
			inline const std::vector<double>& GetVertexesCoords() const
			{
				return m_vertexes_coords;
			}
			/**
			* @brief Returns the triangle connectivity list.
			*
			* @return Reference to array of vertex indices forming triangles.
			*/
			inline const std::vector<unsigned int>& GetTriangleIndexes() const
			{
				return m_triangles_connectivity_indexes;
			}
			/**
			* @brief Returns the list of all triangles.
			*
			* @return Reference to triangle vector.
			*/
			inline const std::vector<CTriangle>& GetTriangles()
			{
				return m_triangles;
			}
			/**
			* @brief Returns the list of all vertexes (vertexes are shared).
			*
			* @return Reference to vertex vector (vertexes are shared).
			*/
			inline const std::vector<CPoint3>& GetVertexes()
			{
				return m_vertexes;
			}

			/**
			* @brief Builds a half-edge structure from the current triangle soup.
			*
			* Populates f_out with twin links, boundary loops, and manifold/closure flags.
			* Call this after a successful LoadStlModelFromFile() or LoadStlModelFromData().
			*
			* @param f_out Output half-edge structure. Cleared and rebuilt on every call.
			* @return half_edge_build_error HE_EXIT_NO_ERROR on success, error code otherwise.
			*/
			inline half_edge_build_error BuildHalfEdgeStructure(THalfEdgeStructure& f_out) const
			{
				if (m_triangles.empty())
				{
					return half_edge_build_error::HE_ERROR_EMPTY_MESH;
				}

				f_out = THalfEdgeStructure{};

				const int l_num_faces = static_cast<int>(m_triangles.size());
				const int l_num_vertices = static_cast<int>(m_vertexes.size());
				const int l_num_he = l_num_faces * 3;

				f_out.m_half_edges.resize(l_num_he);
				f_out.m_face_to_halfedge.resize(l_num_faces, HE_INVALID);
				f_out.m_vertex_to_halfedge.resize(l_num_vertices, HE_INVALID);

				std::unordered_map<long long, int> l_edge_map;
				l_edge_map.reserve(l_num_he);

				auto EdgeKey = [](int v_from, int v_to) -> long long
					{
						return (static_cast<long long>(v_from) << 32) | static_cast<unsigned int>(v_to);
					};

				for (int l_f = 0; l_f < l_num_faces; l_f++)
				{
					const CTriangle& l_tri = m_triangles[l_f];

					const int l_v[3] =
					{
						l_tri.GetVert1ID(),
						l_tri.GetVert2ID(),
						l_tri.GetVert3ID()
					};

					const int l_base = l_f * 3;

					f_out.m_face_to_halfedge[l_f] = l_base;

					for (int l_e = 0; l_e < 3; l_e++)
					{
						const int l_he_idx = l_base + l_e;
						const int l_v_from = l_v[l_e];
						const int l_v_to = l_v[(l_e + 1) % 3];

						THalfEdge& l_he = f_out.m_half_edges[l_he_idx];
						l_he.m_index = l_he_idx;
						l_he.m_vertex_origin = l_v_from;
						l_he.m_face = l_f;
						l_he.m_next = l_base + (l_e + 1) % 3;
						l_he.m_prev = l_base + (l_e + 2) % 3;
						l_he.m_twin = HE_INVALID;

						f_out.m_vertex_to_halfedge[l_v_from] = l_he_idx;

						const long long l_key = EdgeKey(l_v_from, l_v_to);
						if (l_edge_map.count(l_key))
						{
							f_out.m_is_manifold = false;
							return half_edge_build_error::HE_ERROR_NON_MANIFOLD_EDGE;
						}
						l_edge_map[l_key] = l_he_idx;
					}
				}

				for (int l_f = 0; l_f < l_num_faces; l_f++)
				{
					const CTriangle& l_tri = m_triangles[l_f];
					const int l_v[3] =
					{
						l_tri.GetVert1ID(),
						l_tri.GetVert2ID(),
						l_tri.GetVert3ID()
					};

					const int l_base = l_f * 3;

					for (int l_e = 0; l_e < 3; l_e++)
					{
						const int l_he_idx = l_base + l_e;

						if (f_out.m_half_edges[l_he_idx].m_twin != HE_INVALID)
						{
							continue;
						}

						const int l_v_from = l_v[l_e];
						const int l_v_to = l_v[(l_e + 1) % 3];

						const long long l_twin_key = EdgeKey(l_v_to, l_v_from);
						auto l_twin_it = l_edge_map.find(l_twin_key);

						if (l_twin_it != l_edge_map.end())
						{
							const int l_twin_idx = l_twin_it->second;
							f_out.m_half_edges[l_he_idx].m_twin = l_twin_idx;
							f_out.m_half_edges[l_twin_idx].m_twin = l_he_idx;
						}
						else
						{
							f_out.m_boundary_halfedges.push_back(l_he_idx);
						}
					}
				}

				f_out.m_is_closed = f_out.m_boundary_halfedges.empty();
				f_out.m_is_manifold = true;

				if (f_out.m_is_closed)
				{
					f_out.m_boundary_loop_count = 0;
					return half_edge_build_error::HE_EXIT_NO_ERROR;
				}

				std::unordered_map<int, bool> l_visited_boundary;
				for (int l_bhe : f_out.m_boundary_halfedges)
				{
					l_visited_boundary[l_bhe] = false;
				}

				for (int l_bhe_start : f_out.m_boundary_halfedges)
				{
					if (l_visited_boundary[l_bhe_start])
					{
						continue;
					}

					std::vector<int> l_loop;
					int l_current = l_bhe_start;

					do
					{
						l_visited_boundary[l_current] = true;
						l_loop.push_back(l_current);

						const THalfEdge& l_he_cur = f_out.m_half_edges[l_current];
						const int l_v_dest = m_triangles[l_he_cur.m_face].GetVertexIdByLocalIndex((l_he_cur.m_index % 3 + 1) % 3);

						int l_next_bhe = HE_INVALID;
						for (int l_candidate : f_out.m_boundary_halfedges)
						{
							if (f_out.m_half_edges[l_candidate].m_vertex_origin == l_v_dest)
							{
								l_next_bhe = l_candidate;
								break;
							}
						}

						if (l_next_bhe == HE_INVALID)
						{
							break;
						}

						l_current = l_next_bhe;

					} while (l_current != l_bhe_start);

					f_out.m_boundary_loops.push_back(std::move(l_loop));
				}

				f_out.m_boundary_loop_count = static_cast<int>(f_out.m_boundary_loops.size());
				return half_edge_build_error::HE_EXIT_NO_ERROR;

			}

			/**
			* @brief Builds an internal octree over the triangles of this triangle soup.
			*
			* Must be called before RaycastMesh() or IsPointInsideMesh(). The octree is
			* rebuilt from scratch on every call, so calling this repeatedly is valid but
			* expensive. The bounding box of the soup is used as the octree root bounds,
			* expanded by a small epsilon to avoid boundary precision issues.
			*
			* @param f_max_depth         Maximum subdivision depth of the octree. Default: 9.
			* @param f_max_tris_per_node Maximum triangles per leaf node before subdivision. Default: 32.
			* @return true if the octree was built successfully, false if the mesh is empty.
			*/
			inline bool BuildOctree(int f_max_depth = 9, int f_max_tris_per_node = 32)
			{
				if (m_triangles.empty())
				{
					return false;
				}

				constexpr double l_eps = 1.0;

				CPoint3 l_min(m_bottom_left_bbox_vert.m_x - l_eps,
							  m_bottom_left_bbox_vert.m_y - l_eps,
							  m_bottom_left_bbox_vert.m_z - l_eps);
				CPoint3 l_max(m_top_right_bbox_vert.m_x + l_eps,
							  m_top_right_bbox_vert.m_y + l_eps,
							  m_top_right_bbox_vert.m_z + l_eps);

				CAABox l_root_bounds(l_min, l_max);

				std::vector<IGeometricalObject3D*> l_tri_ptrs;
				l_tri_ptrs.reserve(m_triangles.size());
				for (auto& l_tri : m_triangles)
				{
					l_tri_ptrs.push_back(&l_tri);
				}

				m_octree = std::make_unique<COctree>(l_root_bounds, f_max_depth, f_max_tris_per_node);
				m_octree->Build(l_tri_ptrs);

				return true;
			}

			/**
			* @brief Performs a raycast against the triangle soup using the internal octree.
			*
			* Queries the octree for candidate triangles, runs precise ray-triangle
			* intersection via RayTriangleIntersect, removes spatially coincident duplicate
			* hits, and returns the results sorted by distance from the ray origin (nearest
			* first).
			*
			* BuildOctree() must be called before this method. If the octree has not been
			* built, the function returns false immediately.
			*
			* @param f_ray       The ray to cast against the mesh.
			* @param f_out_hits  Output vector of deduplicated hit results, sorted nearest-first.
			* @return true if at least one triangle was hit, false otherwise.
			*/
			inline bool RaycastMesh(const CRay& f_ray,
								    std::vector<CRayCastResult>& f_out_unique) const
			{
				f_out_unique.clear();

				if (!m_octree)
				{
					return false;
				}

				std::vector<IGeometricalObject3D*> l_candidates;
				m_octree->Raycast(f_ray, l_candidates);

				if (l_candidates.empty())
				{
					return false;
				}

				std::vector<CRayCastResult> l_intersections;
				l_intersections.reserve(l_candidates.size());

				for (IGeometricalObject3D* l_object : l_candidates)
				{
					auto* l_tri = dynamic_cast<CTriangle*>(l_object);
					if (l_tri)
					{
						continue;
					}
					CRayCastResult l_res;
					if (l_tri->RayTriangleIntersect(f_ray, l_res, COINC_THRESH))
					{
						if (l_res.GetTravelledDistance() > 0.0)
						{
							l_intersections.emplace_back(l_res);
						}
					}
				}

				if (l_intersections.empty())
				{
					return false;
				}

				std::sort(l_intersections.begin(), l_intersections.end(),
					[](const auto& f_intersection_1, const auto& f_intersection_2)
					{
						return f_intersection_2.GetTravelledDistance() >
							f_intersection_1.GetTravelledDistance();
					});

				double l_last_valid_distance = -HUGE_VAL;

				for (const auto& l_res : l_intersections)
				{
					const double l_dist = l_res.GetTravelledDistance();

					if (f_out_unique.empty() || (l_dist - l_last_valid_distance) > COINC_THRESH)
					{
						f_out_unique.push_back(l_res);
						l_last_valid_distance = l_dist;
					}
					else
					{
						const bool l_prev_exiting = f_out_unique.back().GetExitingModel();
						const bool l_curr_exiting = l_res.GetExitingModel();

						if (l_prev_exiting != l_curr_exiting)
						{
							f_out_unique.pop_back();

							l_last_valid_distance = 0.0;
							if (f_out_unique.empty())
							{
								l_last_valid_distance = -HUGE_VAL;
							}
							else
							{
								l_last_valid_distance = f_out_unique.back().GetTravelledDistance();
							}
						}
					}
				}

				return !f_out_unique.empty();
			}

			/**
			* @brief Tests whether a 3D point lies inside the closed triangle soup.
			*
			* Uses the ray casting method: fires a ray from the query point in a fixed
			* direction (+X axis) and counts intersections with the mesh via the octree.
			* An odd count means the point is inside; an even count means outside.
			*
			* The mesh must be closed (watertight) for the result to be reliable. Use
			* BuildHalfEdgeStructure() to verify closure before calling this method.
			* BuildOctree() must be called before this method.
			*
			* @param f_point  The 3D point to test.
			* @return true if the point is inside the mesh, false if outside or if the
			*         octree has not been built.
			*/
			inline bool IsPointInsideMesh(const CPoint3& f_point) const
			{
				if (!m_octree)
				{
					return false;
				}

				CPoint3 l_direction(1.0, 0.0, 0.0);
				CRay l_ray(f_point, l_direction);

				std::vector<CRayCastResult> l_unique;
				if (!RaycastMesh(l_ray, l_unique))
				{
					return false;
				}

				return (l_unique.size() % 2) == 1;
			}
		private:

			/**
			* @brief Builds the triangles of the soup from the shared vertices and the
			*        connectivity indices.
			*
			* The indices are read in groups of three: indices (3k, 3k+1, 3k+2) refer to
			* the vertices of triangle k in m_vertexes. Each triangle stores pointers to
			* its three vertices, their indices, and its own index k, and is appended to
			* m_triangles.
			*
			* No validation of the mesh is performed: holes, duplicated or degenerate
			* triangles and non-manifold configurations are accepted as they are.
			*
			* @param[in] f_triangles_indexes Connectivity indices into m_vertexes, three
			*                                per triangle. If the number of indices is
			*                                not a multiple of 3, the last one or two
			*                                indices are ignored.
			*
			* @return 0 on success, 1 if CheckIfEmptyStl() reports an empty soup (no
			*         triangles are created in that case).
			*
			* @warning The indices are not range-checked: every index must be smaller
			*          than m_vertexes.size().
			*
			* @warning The triangles store pointers into m_vertexes. Any later operation
			*          that reallocates m_vertexes (e.g. adding vertices) invalidates
			*          them.
			*
			* @warning m_triangles is not cleared and triangle indices always restart
			*          from 0: calling this function more than once appends a second set
			*          of triangles with duplicate indices.
			*/
			inline int AssembleTriangles(const std::vector<unsigned int>& f_triangles_indexes)
			{
				if (!CheckIfEmptyStl())
				{
					return 1;
				}

				int l_i_vertex = 0;
				int l_triangle_index = 0;
				for (auto l_it = f_triangles_indexes.begin(); l_it != f_triangles_indexes.end(); l_it++)
				{
					l_i_vertex++;
					if (l_i_vertex == 3)
					{
						m_triangles.emplace_back(&m_vertexes[*(l_it - 2)],
												 &m_vertexes[*(l_it - 1)],
												 &m_vertexes[*l_it],
												 *(l_it - 2),
												 *(l_it - 1),
												 *l_it,
												 l_triangle_index);

						l_triangle_index++;
						l_i_vertex = 0;
					}
				}

				return 0;
			}

			/**
			* @brief Validates whether the STL mesh is non-empty.
			*
			* @return true if STL is valid, false otherwise.
			*/
			inline bool CheckIfEmptyStl()
			{
				if (m_vertexes.size() < 4)
				{
					return false;
				}

				return true;
			}

			/**
			* @brief Computes the 3D bounding box for the triangle soup.
			*
			* @return true if bounding box was successfully calculated.
			*/
			inline bool CalculateBoundingBox()
			{
				m_bottom_left_bbox_vert.m_x = 0.0;
				m_bottom_left_bbox_vert.m_y = 0.0;
				m_bottom_left_bbox_vert.m_z = 0.0;
				m_top_right_bbox_vert.m_x = 0.0;
				m_top_right_bbox_vert.m_y = 0.0;
				m_top_right_bbox_vert.m_z = 0.0;


				if (!m_vertexes.empty())
				{
					m_bottom_left_bbox_vert = m_vertexes.front();
					m_top_right_bbox_vert = m_bottom_left_bbox_vert;
				}

				for (auto l_i_vertex = m_vertexes.begin(); l_i_vertex != m_vertexes.end(); l_i_vertex++)
				{
					if (l_i_vertex->m_x > m_top_right_bbox_vert.m_x)
					{
						m_top_right_bbox_vert.m_x = l_i_vertex->m_x;
					}
					if (l_i_vertex->m_y > m_top_right_bbox_vert.m_y)
					{
						m_top_right_bbox_vert.m_y = l_i_vertex->m_y;
					}
					if (l_i_vertex->m_z > m_top_right_bbox_vert.m_z)
					{
						m_top_right_bbox_vert.m_z = l_i_vertex->m_z;
					}
					if (l_i_vertex->m_x < m_bottom_left_bbox_vert.m_x)
					{
						m_bottom_left_bbox_vert.m_x = l_i_vertex->m_x;
					}
					if (l_i_vertex->m_y < m_bottom_left_bbox_vert.m_y)
					{
						m_bottom_left_bbox_vert.m_y = l_i_vertex->m_y;
					}
					if (l_i_vertex->m_z < m_bottom_left_bbox_vert.m_z)
					{
						m_bottom_left_bbox_vert.m_z = l_i_vertex->m_z;
					}
				}

				if ((fabs(m_top_right_bbox_vert.m_x) > LARGE_DOUBLE_CONST) || (fabs(m_bottom_left_bbox_vert.m_x) > LARGE_DOUBLE_CONST) ||
					(fabs(m_top_right_bbox_vert.m_y) > LARGE_DOUBLE_CONST) || (fabs(m_bottom_left_bbox_vert.m_y) > LARGE_DOUBLE_CONST) ||
					(fabs(m_top_right_bbox_vert.m_z) > LARGE_DOUBLE_CONST) || (fabs(m_bottom_left_bbox_vert.m_z) > LARGE_DOUBLE_CONST))
				{
					return false;
				}

				return true;
			}

			std::vector<CTriangle> m_triangles; 
			std::vector<CPoint3> m_vertexes;    
			std::vector<double> m_vertexes_coords;  
			std::vector<unsigned int> m_triangles_connectivity_indexes;	
			std::vector<unsigned int> m_solids;                        

			CPoint3 m_bottom_left_bbox_vert;    
			CPoint3 m_top_right_bbox_vert;

			std::unique_ptr<COctree> m_octree;
		};
	}
}
#endif
