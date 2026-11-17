
#ifndef H_IGRAPH_ITFC
#define H_IGRAPH_ITFC
#pragma once

#include <sstream>
#include "CPoint2.h"
#include "CPoint3.h"

namespace ggeomlib
{
	namespace Graph
	{
#define GOV_COLOR_BLACK 0
#define GOV_COLOR_BLUE 1
#define GOV_COLOR_DARK_GREEN 2
#define GOV_COLOR_SKY_BLUE 3
#define GOV_COLOR_RED 4  
#define GOV_COLOR_WINE 5
#define GOV_COLOR_BROWN 6
#define GOV_COLOR_GREY 8
#define GOV_COLOR_ELECTRIC_BLUE 9
#define GOV_COLOR_GREEN 10
#define GOV_COLOR_LIGHT_BLUE 11
#define GOV_COLOR_GARNET 12
#define GOV_COLOR_VIOLET 13
#define GOV_COLOR_YELLOW 14
#define GOV_COLOR_SAPPHIRE_BLUE 15
#define GOV_COLOR_COBALT_BLUE 16
#define GOV_COLOR_AZURE 17
#define GOV_COLOR_CORNFLOWER_BLUE 18
#define GOV_COLOR_CERULEAN 19
#define GOV_COLOR_ARCTIC_BLUE 20
#define GOV_COLOR_CYAN 21
#define GOV_COLOR_OPAL 22
#define GOV_COLOR_TORQUOISE 23
#define GOV_COLOR_AQUAMARINE 24
#define GOV_COLOR_SPRING_GREEN 25
#define GOV_COLOR_SEA_GREEN 26
#define GOV_COLOR_MALACHITE_GREEN 27
#define GOV_COLOR_EMERALD_GREEN 28
#define GOV_COLOR_SAP_GREEN 29
#define GOV_COLOR_HARLEQUIN 30
#define GOV_COLOR_PISTACCHIO 31
#define GOV_COLOR_CHARTREUSE 32
#define GOV_COLOR_SPRING_BUD 33
#define GOV_COLOR_LIME_GREEN 34
#define GOV_COLOR_APPLE_GREEN 35
#define GOV_COLOR_DARK_YELLOW 36
#define GOV_COLOR_GOLD 37
#define GOV_COLOR_AMBER 38
#define GOV_COLOR_GAMBOGE 39
#define GOV_COLOR_ORANGE 40
#define GOV_COLOR_TANGELO 41
#define GOV_COLOR_VERMILION 42
#define GOV_COLOR_SCARLET 43
#define GOV_COLOR_WHITE 44

		enum LogType
		{
			LogInfo = 1,
			LogWarning = 2,
			LogError = 3,
		};

		enum EMeshFacetType
		{
			TRIANGULAR_FACES,
			QUADRANGULAR_FACES,
		};

		enum EPointShapeType
		{
			CIRCLE_PT,
			SQUARE_PT,
			CROSS_PT,
		};

		/**
		* @brief Rendering helper interface for drawing and managing graphical primitives.
		*
		* This interface provides methods for drawing and managing geometric primitives
		* such as meshes, points, lines, polylines, and analytical shapes. Each draw call
		* typically returns an integer identifier that can be used later to remove or
		* manage the rendered object. Concrete implementations are responsible for rendering
		* using the underlying graphics system.
		*/
		class IGraphInterface
		{
		public:

			/**
			* @brief Draws a triangle mesh.
			*
			* @param f_vertexes_coords		Pointer to array of vertex coordinates (x, y, z).
			* @param f_vertexes_coords_size Number of elements in the vertex array.
			* @param f_tri_indexes			Pointer to triangle index connectivity.
			* @param f_tri_indexes_size		Number of elements in the index array.
			* @param f_draw_wireframe		Enables wireframe rendering.
			* @param f_draw_faces			Enables face rendering.
			* @param f_wireframe_color		Packed integer wireframe color.
			* @param f_facet_color			Packed integer face color.
			* @param f_redraw				Forces immediate redraw if true.
			* @param f_bake					If true, geometry is baked (persistent).
			* @param f_increment_id			If true, generates a new object identifier.
			*
			* @return Identifier of the drawn mesh object.
			*/
			virtual int DrawTriMesh(const double* f_vertexes_coords,
									const size_t f_vertexes_coords_size,
									const unsigned int* f_tri_indexes,
									const size_t f_tri_indexes_size,
									const bool f_draw_wireframe,
									const bool f_draw_faces,
									const int f_wireframe_color,
									const int f_facet_color,
									const bool f_redraw,
									const bool f_bake = true,
									const bool f_increment_id = true) = 0;
			/**
			* @brief Draws a 3D point.
			*
			* @param f_coords			Point coordinates (x, y, z).
			* @param f_color			Packed integer color.
			* @param f_redraw			Forces immediate redraw if true.
			* @param f_bake				If true, point is baked (persistent).
			* @param f_size				Point size in pixels.
			* @param f_point_shape_type Shape used to render the point.
			* @param f_increment_id		If true, generates a new object identifier.
			*
			* @return Identifier of the drawn point.
			*/
			virtual int DrawPoint3(const double f_coords[3],
								   const int f_color,
								   const bool f_redraw,
								   const bool f_bake = true,
								   const int f_size = 2,
								   const EPointShapeType f_point_shape_type = EPointShapeType::CIRCLE_PT,
								   const bool f_increment_id = true) = 0;
			/**
			* @brief Draws a 2D point.
			*
			* @param f_coords			Point coordinates (x, y).
			* @param f_color			Packed integer color.
			* @param f_redraw			Forces immediate redraw if true.
			* @param f_bake				If true, point is baked (persistent).
			* @param f_size				Point size in pixels.
			* @param f_point_shape_type Shape used to render the point.
			* @param f_increment_id		If true, generates a new object identifier.
			*
			* @return Identifier of the drawn point.
			*/
			virtual int DrawPoint2(const double f_coords[2],
								   const int f_color,
								   const bool f_redraw,
								   const bool f_bake = true,
								   const int f_size = 2,
								   const EPointShapeType f_point_shape_type = EPointShapeType::CIRCLE_PT,
								   const bool f_increment_id = true) = 0;
			/**
			* @brief Draws a 3D  line segment.
			*
			* @param f_start_point_coords	Start point coordinates.
			* @param f_end_point_coords		End point coordinates.
			* @param f_color				Packed integer color.
			* @param f_redraw				Forces immediate redraw if true.
			* @param f_bake					If true, line is baked (persistent).
			* @param f_line_width			Line width in pixels.
			* @param f_increment_id			If true, generates a new object identifier.
			*
			* @return Identifier of the drawn line.
			*/
			virtual int DrawLine3(const double f_start_point_coords[3],
								  const double f_end_point_coords[3],
								  const int f_color,
								  const bool f_redraw,
								  const bool f_bake = true,
								  const int f_line_width = 1,
								  const bool f_increment_id = true) = 0;

			/**
			* @brief Draws a 2D  line segment.
			*
			* @param f_start_point_coords	Start point coordinates.
			* @param f_end_point_coords		End point coordinates.
			* @param f_color				Packed integer color.
			* @param f_redraw				Forces immediate redraw if true.
			* @param f_bake					If true, line is baked (persistent).
			* @param f_line_width			Line width in pixels.
			* @param f_increment_id			If true, generates a new object identifier.
			*
			* @return Identifier of the drawn line.
			*/
			virtual int DrawLine2(const double f_start_point_coords[2],
								  const double f_end_point_coords[2],
								  const int f_color,
								  const bool f_redraw,
								  const bool f_bake = true,
								  const int f_line_width = 1,
								  const bool f_increment_id = true) = 0;

			/**
			* @brief Draws a 3D vector as a segment starting at an application point.
			*
			* The segment starts at f_application_point_coords and points in the direction
			* of f_vec, with length f_lenght (the direction of f_vec is used, its own
			* length is ignored) (confirm).
			*
			* @param f_vec                      Vector direction (x, y, z).
			* @param f_application_point_coords Start point of the segment (x, y, z).
			* @param f_lenght                   Length of the drawn segment, in model
			*                                   units.
			* @param f_color                    Color of the segment (see the color
			*                                   constants of the graphics interface).
			* @param f_redraw                   If true, the view is refreshed
			*                                   immediately after drawing.
			* @param f_bake                     If true, the segment is stored
			*                                   permanently in the scene; if false, it
			*                                   is temporary and removed at the next
			*                                   refresh (confirm).
			* @param f_line_width               Line width, in pixels.
			* @param f_increment_id             If true, a new entity ID is assigned to
			*                                   the segment; if false, the current ID is
			*                                   reused, e.g. to group several entities
			*                                   (confirm).
			*
			* @return ID of the drawn entity, or a negative value on error (confirm).
			*/
			virtual int DrawApplicatedVec3(const double f_vec[3],
										   const double f_application_point_coords[3],
										   const double f_lenght,
										   const int f_color,
										   const bool f_redraw,
										   const bool f_bake = true,
										   const int f_line_width = 1,
										   const bool f_increment_id = true) = 0;
			
			/**
			* @brief Draws a 2D vector as a segment starting at an application point.
			*
			* Same as DrawApplicatedVec3(), for 2D vectors and points (drawn in the XY
			* plane).
			*
			* @param f_vec                      Vector direction (x, y).
			* @param f_application_point_coords Start point of the segment (x, y).
			* @param f_lenght                   Length of the drawn segment, in model
			*                                   units.
			* @param f_color                    Color of the segment.
			* @param f_redraw                   If true, the view is refreshed
			*                                   immediately after drawing.
			* @param f_bake                     See DrawApplicatedVec3().
			* @param f_line_width               Line width, in pixels.
			* @param f_increment_id             See DrawApplicatedVec3().
			*
			* @return ID of the drawn entity, or a negative value on error (confirm).
			*/
			virtual int DrawApplicatedVec2(const double f_vec[2],
										   const double f_application_point_coords[2],
										   const double f_lenght,
										   const int f_color,
										   const bool f_redraw,
										   const bool f_bake = true,
										   const int f_line_width = 1,
										   const bool f_increment_id = true) = 0;
			/**
			* @brief Draws a 2D polyline.
			*
			* @param f_poly_points		Pointer to array of 2D polyline points.
			* @param f_poly_points_num	Number of points in the polyline.
			* @param f_color			Packed integer color.
			* @param f_redraw			Forces immediate redraw if true.
			* @param f_bake				If true, polyline is baked (persistent).
			* @param f_line_width		Line width in pixels.
			* @param f_increment_id		If true, generates a new object identifier.
			*
			* @return Identifier of the drawn polyline.
			*/
			virtual int DrawPolyLine2(const CPoint2* f_poly_points,
									  const size_t f_poly_points_num,
									  const int f_color,
									  const bool f_redraw,
									  const bool f_bake = true,
									  const int f_line_width = 1,
									  const bool f_increment_id = true) = 0;
			/**
			* @brief Draws a 3D polyline.
			*
			* @param f_poly_points		Pointer to array of 3D polyline points.
			* @param f_poly_points_num	Number of points in the polyline.
			* @param f_color			Packed integer color.
			* @param f_redraw			Forces immediate redraw if true.
			* @param f_bake				If true, polyline is baked (persistent).
			* @param f_line_width		Line width in pixels.
			* @param f_increment_id		If true, generates a new object identifier.
			*
			* @return Identifier of the drawn polyline.
			*/
			virtual int DrawPolyLine3(const CPoint3* f_poly_points,
									  const size_t f_poly_points_num,
									  const int f_color,
									  const bool f_redraw,
									  const bool f_bake = true,
									  const int f_line_width = 1,
									  const bool f_increment_id = true) = 0;
			/**
			* @brief Draws a cylinder.
			*
			* @param f_center			Center point of the cylinder.
			* @param f_axis				Axis direction of the cylinder.
			* @param f_length			Cylinder length.
			* @param f_radius			Cylinder radius.
			* @param f_color			Packed integer color.
			* @param f_redraw			Forces immediate redraw if true.
			* @param f_bake				If true, cylinder is baked (persistent).
			* @param f_chord_error		Maximum chordal deviation.
			* @param f_draw_bottom_cap	Enables drawing of the bottom cap.
			* @param f_draw_top_cap		Enables drawing of the top cap.
			* @param f_line_width		Line width in pixels.
			* @param f_increment_id		If true, generates a new object identifier.
			*
			* @return Identifier of the drawn cylinder.
			*/
			virtual int DrawCylinder(const double f_center[3],
									 const double f_axis[3],
									 const double f_length,
									 const double f_radius,
									 const int f_color,
									 const bool f_redraw,
									 const bool f_bake = true,
									 const double f_chord_error = 0.01,
									 const bool f_draw_bottom_cap = false,
									 const bool f_draw_top_cap = false,
									 const int f_line_width = 1,
									 const bool f_increment_id = true) = 0;

			/**
			* @brief Draws a truncated cone.
			*
			* @param f_center			Center point of the truncated cone.
			* @param f_axis				Axis direction of the truncated cone.
			* @param f_length			Length of the truncated cone.
			* @param f_lower_radius		Radius at the bottom/base of the cone.
			* @param f_upper_radius		Radius at the top of the cone.
			* @param f_color			Packed integer color.
			* @param f_redraw			Forces immediate redraw if true.
			* @param f_bake				If true, cone is baked (persistent).
			* @param f_chord_error		Maximum chordal deviation.
			* @param f_draw_bottom_cap	Enables drawing of the bottom cap.
			* @param f_draw_top_cap		Enables drawing of the top cap.
			* @param f_line_width		Line width in pixels.
			* @param f_increment_id		If true, generates a new object identifier.
			*
			* @return Identifier of the drawn truncated cone.
			*/
			virtual int DrawTruncatedCone(const double f_center[3],
										  const double f_axis[3],
										  const double f_length,
										  const double f_lower_radius,
										  const double f_upper_radius,
										  const int f_color,
										  const bool f_redraw,
										  const bool f_bake = true,
										  const double f_chord_error = 0.01,
										  const bool f_draw_bottom_cap = false,
										  const bool f_draw_top_cap = false,
										  const int f_line_width = 1,
										  const bool f_increment_id = true) = 0;
			/**
			* @brief Draws an axis-aligned box.
			*
			* @param f_low_left_vert	Lower-left corner of the box.
			* @param f_up_right_vert	Upper-right corner of the box.
			* @param f_color			Packed integer color.
			* @param f_redraw			Forces immediate redraw if true.
			* @param f_bake				If true, box is baked (persistent).
			* @param f_line_width		Line width in pixels.
			* @param f_increment_id		If true, generates a new object identifier.
			*
			* @return Identifier of the drawn box.
			*/
			virtual int DrawBox(const double f_low_left_vert[3],
								const double f_up_right_vert[3],
								const int f_color,
								const bool f_redraw,
								const bool f_bake = true,
								const int f_line_width = 1,
								const bool f_increment_id = true) = 0;
			/**
			* @brief Draws a 2D rectangle.
			*
			* @param f_ll_vert		Lower-left vertex (2D).
			* @param f_ur_vert		Upper-right vertex (2D).
			* @param f_color		Packed integer color.
			* @param f_redraw		Forces immediate redraw if true.
			* @param f_bake			If true, rectangle is baked (persistent).
			* @param f_line_width	Line width in pixels.
			* @param f_increment_id If true, generates a new object identifier.
			*
			* @return Identifier of the drawn rectangle.
			*/
			virtual int DrawRectangle(const double f_ll_vert[2],
									  const double f_ur_vert[2],
									  const int f_color,
									  const bool f_redraw,
									  const bool f_bake = true,
									  const int f_line_width = 1,
									  const int f_increment_id = true) = 0;
			/**
			* @brief Draws a circle in the space
			*
			* @param f_center		Center of the circle.
			* @param f_radius		Circle radius.
			* @param f_color		Packed integer color.
			* @param f_redraw		Forces immediate redraw if true.
			* @param f_bake			If true, circle is baked (persistent).
			* @param f_line_width	Line width in pixels.
			* @param f_chord_error	Maximum chordal deviation.
			* @param f_increment_id If true, generates a new object identifier.
			*
			* @return Identifier of the drawn circle.
			*/
			virtual int DrawCircle3(const double f_center[3],
									const double f_radius,
									const int f_color,
									const bool f_redraw,
									const bool f_bake = true,
									const int f_line_width = 1,
									const double f_chord_error = 0.001,
									const bool f_increment_id = true) = 0;

			/**
			* @brief Draws a 2D circle on the XY plame
			*
			* @param f_center		Center of the circle.
			* @param f_radius		Circle radius.
			* @param f_color		Packed integer color.
			* @param f_redraw		Forces immediate redraw if true.
			* @param f_bake			If true, circle is baked (persistent).
			* @param f_line_width	Line width in pixels.
			* @param f_chord_error	Maximum chordal deviation.
			* @param f_increment_id If true, generates a new object identifier.
			*
			* @return Identifier of the drawn circle.
			*/
			virtual int DrawCircle2(const double f_center[2],
									const double f_radius,
									const int f_color,
									const bool f_redraw,
									const bool f_bake = true,
									const int f_line_width = 1,
									const double f_chord_error = 0.001,
									const bool f_increment_id = true) = 0;
			/**
			* @brief Draws a hemisphere.
			*
			* @param f_center			Center of the hemisphere.
			* @param f_axis				Axis direction of the hemisphere.
			* @param f_radius			Hemisphere radius.
			* @param f_color			Packed integer color.
			* @param f_redraw			Forces immediate redraw if true.
			* @param f_bake				If true, hemisphere is baked (persistent).
			* @param f_line_width		Line width in pixels.
			* @param f_chordal_error	Maximum chordal deviation.
			* @param f_increment_id		If true, generates a new object identifier.
			*
			* @return Identifier of the drawn hemisphere.
			*/
			virtual int DrawEmisphere(const double f_center[3],
									  const double f_axis[3],
									  const double f_radius,
									  const int f_color,
									  const bool f_redraw,
									  const bool f_bake = true,
									  const int f_line_width = 1,
									  const double f_chordal_error = 0.01,
									  const bool f_increment_id = true) = 0;
			/**
			* @brief Draws a plane with optional grid and normal visualization.
			*
			* @param f_plane_normal		Normal vector of the plane.
			* @param f_point_on_plane	A point lying on the plane.
			* @param f_height			Plane height.
			* @param f_width			Plane width.
			* @param f_grid_spacing		Grid spacing.
			* @param f_draw_normal		Enables drawing of the plane normal.
			* @param f_color			Packed integer color.
			* @param f_redraw			Forces immediate redraw if true.
			* @param f_bake				If true, plane is baked (persistent).
			* @param f_line_width		Line width in pixels.
			* @param f_increment_id		If true, generates a new object identifier.
			*
			* @return Identifier of the drawn plane.
			*/
			virtual int DrawPlane(const double f_plane_normal[3],
								  const double f_point_on_plane[3],
								  const double f_height,
								  const double f_width,
								  const double f_grid_spacing,
								  const bool f_draw_normal,
								  const int f_color,
								  const bool f_redraw,
								  const bool f_bake = true,
								  const int f_line_width = 1,
								  const bool f_increment_id = true) = 0;
			/**
			* @brief Removes a previously drawn object by its identifier
			*
			* @param f_object_to_remove_id Identifier of the object to remove.
			*
			* @return True if the object was successfully removed.
			*/
			virtual bool RemoveObjectbyId(const int f_object_to_remove_id) = 0;

			/**
			* @brief Clears drawn objects.
			*
			* @param f_drawreset_baked_geom If true, also clears baked geometry.
			*/
			virtual void DrawReset(const bool f_drawreset_baked_geom) = 0;
			/**
			* @brief Forces a redraw of the graphical area.
			*
			* @param f_redraw_baked_geom If true, redraws baked geometry as well.
			*/
			virtual void ReDraw(const bool f_redraw_baked_geom) = 0;

			/**
			* @brief Outputs a wide-character log message.
			*
			* @param f_message Null-terminated wide-character string.
			* @param f_log_type Log severity/type.
			*/
			virtual void LogOutput(const wchar_t* f_message, const LogType f_log_type) = 0;
			/**
			* @brief Outputs a narrow-character log message.
			*
			* @param f_message Null-terminated string.
			* @param f_log_type Log severity/type.
			*/
			virtual void LogOutput(const char* f_message, const LogType f_log_type) = 0;

			/**
			* @brief Virtual destructor.
			*/
			virtual ~IGraphInterface() = default;

		};
	}
}

#endif
