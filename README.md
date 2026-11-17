# GGeomLib

A curated C++ collection of proven computational geometry algorithms, types and
auxiliary functions: the building blocks that keep recurring in higher-level
geometry code, such as CAD/CAM, toolpath generation, mesh processing and
collision detection. The intention of the library is to be simple but effective. It is not perfect and still lacks many things, but I hope it can be useful.

Some algorithms are original implementations based on books and papers; others
are adapted from well-known existing work. Every adapted part is credited in its file
header and in [CREDITS.md](CREDITS.md).

<!-- TODO: one sentence on the C++ standard required and whether GGeomLib is fully
header-only, e.g. "Header-only, C++17, no dependencies beyond the standard
library." -->

> **Project status:** this library is at version 0.0.1, an early release. It
> is still being tested and will be further developed, so its API may change
> in future versions. Feedback and bug reports are welcome.
>
> This library has been tested only on Windows and compiled with MSVC 2022 so far.

## Some examples of what's inside

### Basic types

- **Points and vectors**: 2D (`CPoint2`), 3D (`CPoint3`) and homogeneous 4D
  (`CPoint4`) points, with the usual vector operations (dot and cross products,
  lengths, normalization, distances).
- **Rays, rectangles and boxes**: 2D rays, axis-aligned rectangles and
  bounding boxes.
- **Profiles and polylines**: closed and open profiles, with area, orientation
  and point containment.
- **Triangles and triangle soups**: triangles with intersection tests, and
  triangle soups built from shared vertices and connectivity (e.g. from STL
  files).
- **Ray cast results**: intersection point, surface normal, travelled distance
  and entering/exiting state.

### 2D algorithms

- Orientation of point triplets, point-in-triangle tests (strict and inclusive)
- Point-in-polygon test (winding number), profile area and orientation
  (shoelace formula)
- Segment clipping against axis-aligned boxes (Liang-Barsky)
- Intersections: horizontal line with circle and with circular ring, ray with
  circle
- Polygon triangulation by ear clipping

### 3D algorithms

- Segment-segment distance with closest points and relation classification
- Plane-segment, ray-sphere and ray-cylinder intersections
- Box-cylinder, sphere-triangle and triangle-box (separating axis) intersection
  tests
- Closest point on a triangle
- Axis-aligned bounding box of a cylinder
- Gram-Schmidt orthonormalization, orthogonal complements, angles between
  vectors

### Curves

- Rational Bézier curves of any degree: evaluation, subdivision, extraction of
  sub-curves, and tolerance-based splitting for polyline approximation
- Hermite curves

### Spatial structures

- Uniform 2D grid for fast spatial queries, with a variant that rejects
  coincident objects
- Interfaces for 2D and 3D geometrical objects (bounding box and box
  intersection), so that your own shapes can be stored in the spatial
  structures
- Quadtree
- Octree

<!-- TODO: add any modules not listed above. -->

## Getting started

<!-- TODO: describe how to add GGeomLib to a project, e.g.:
- copy the `include/` folder into your project and add it to the include path, or
- with CMake FetchContent:

```cmake
include(FetchContent)
FetchContent_Declare(ggeomlib
    GIT_REPOSITORY https://github.com/<user>/<repo>.git
    GIT_TAG        v0.0.1)
FetchContent_MakeAvailable(ggeomlib)
target_link_libraries(my_app PRIVATE ggeomlib)
```
-->

Just download the repo and include the .h file you need.

## Documentation

All public types and functions are documented with Doxygen comments in the
headers. Each comment states the algorithm used, the conventions (orientation,
closed or open profiles, tolerances) and the source the algorithm comes from.

## Credits

GGeomLib includes code adapted from the Geometric Tools Engine (David Eberly),
*Real-Time Collision Detection* (Christer Ericson), geomalgorithms.com
(Dan Sunday), flo-bezier3 (Floris Steenkamp) and dlib (Davis E. King), and
implements algorithms from many books and papers. See [CREDITS.md](CREDITS.md)
for the full list of sources, their licenses and references.

## License

GGeomLib is released under the [Boost Software License 1.0](LICENSE). Adapted
portions retain their original licenses, all of which are permissive and
compatible with it: Boost Software License 1.0, MIT, the softSurfer notice, and
the *Real-Time Collision Detection* book license. See
[CREDITS.md](CREDITS.md).
