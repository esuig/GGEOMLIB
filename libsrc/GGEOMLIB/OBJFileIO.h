
#ifndef H_OBJ_MESH_IO
#define H_OBJ_MESH_IO

#pragma once

#include <string>
#include <vector>
#include <cassert>
#include <fstream>

#include "CPoint3.h"

namespace ggeomlib
{
	/**
	* @brief Saves triangle geometry with per-face colors to an OBJ + MTL file pair.
	*
	* This function exports a list of triangle vertices and their corresponding face
	* colors into a Wavefront OBJ file, along with an associated MTL file containing
	* one colour for each entry in the provided color palette. Each triangle (3 vertices)
	* is assigned a colours based on `f_face_colors`.
	*
	* The OBJ file will contain:
	*   - Vertex positions (v)
	*   - Face definitions (f)
	*   - Material usage commands (usemtl)
	*
	* The MTL file will contain:
	*   - One material per palette entry (newmtl)
	*   - Diffuse (Kd), ambient (Ka), and specular (Ks) color components
	*
	* @param f_obj_path Path to the output OBJ file. The MTL file will be created next to it
	*				    with the same name but a `.mtl` extension.
	* @param f_tri_vertexes A flat array of triangle vertices. Its size must be divisible by 3.
	*						Every consecutive triplet of vertices represents one triangle.
	*
	* @param f_face_colors A list of color indices, one per face (triangle).
	*					   Must have size equal to `f_tri_vertexes.size() / 3`.
	*					   Each value selects an entry from `f_color_palette`.
	*
	* @param f_color_palette A list of RGB colors (ranging from 0.0 to 1.0), each 
							 defining a material to be written into the MTL file.
	*
	* @return `true` on successful write of both OBJ and MTL files,
	*         or `false` if file creation fails or an invalid color index is encountered.
	*
	* @note
	*   - The OBJ and MTL file will reside in the same directory.
	*   - The function checks that triangle and color counts are consistent.
	*   - Vertex indices in the OBJ file are 1-based.
	*/
	inline bool SaveTrianglesToOBJWithColor(const std::string& f_obj_path,
											const std::vector<CPoint3>& f_tri_vertexes,
											const std::vector<int>& f_face_colors,
											const std::vector<CPoint3>& f_color_palette)
	{
		assert((f_tri_vertexes.size() % 3) == 0);
		assert((f_tri_vertexes.size() / 3) == f_face_colors.size());

		std::string l_mtl_path = f_obj_path.substr(0, f_obj_path.find_last_of('.')) + ".mtl";

		std::ofstream l_obj(f_obj_path);
		std::ofstream l_mtl(l_mtl_path);
		if (!l_obj.is_open() || !l_mtl.is_open())
			return false;

		l_obj << "# Exported OBJ with per-face colors\n";
		l_obj << "mtllib " << l_mtl_path.substr(l_mtl_path.find_last_of("/\\") + 1) << "\n\n";

		for (const auto& v : f_tri_vertexes)
			l_obj << "v " << (float)v.m_x << " " << (float)v.m_y << " " << (float)v.m_z << "\n";

		l_obj << "\n";

		for (size_t i = 0; i < f_color_palette.size(); ++i)
		{
			const auto& c = f_color_palette[i];

			l_mtl << "newmtl color_" << i << "\n";
			l_mtl << "Kd " << c.m_x << " " << c.m_y << " " << c.m_z << "\n";
			l_mtl << "Ka " << c.m_x << " " << c.m_y << " " << c.m_z << "\n";
			l_mtl << "Ks 0.2 0.2 0.2\n\n";
		}

		const size_t l_num_faces = f_tri_vertexes.size() / 3;
		for (size_t face = 0; face < l_num_faces; ++face)
		{
			int cid = f_face_colors[face];
			if (cid >= f_color_palette.size())
				return false;

			size_t v1 = face * 3 + 1;
			size_t v2 = face * 3 + 2;
			size_t v3 = face * 3 + 3;

			l_obj << "usemtl color_" << cid << "\n";
			l_obj << "f " << v1 << " " << v2 << " " << v3 << "\n";
		}

		l_obj.close();
		l_mtl.close();
		return true;
	}
}

#endif
