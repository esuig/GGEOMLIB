# Credits

GGeomLib is a curated collection of computational geometry algorithms, types and
auxiliary functions. Most of the code is original work by Giuseppe Venturini.
Some parts are adapted from existing work by other authors; this file lists
them, together with their licenses. Each adapted file or function also carries
the original notice in its own header or documentation.

GGeomLib itself is released under the Boost Software License 1.0 (see `LICENSE`).
Adapted portions retain their original licenses, all of which are permissive
and compatible with the Boost Software License.

The file is organized in three parts:

1. [Adapted code](#1-adapted-code): code derived from other sources, which requires keeping the original copyright notice.
2. [Third-party license texts](#2-third-party-license-texts): the full text of the licenses other than the Boost Software License.
3. [References](#3-references): books, papers and articles whose algorithms were implemented independently (no code reused; listed as a courtesy).

---

## 1. Adapted code

| GGeomLib file / function                                                                    | Original source                                                                                                                                                      | Author                          | License                                                                       |
| ---------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------- | ----------------------------------------------------------------------------- |
| `BoxCylinderIntersection.h`                                                              | Geometric Tools Engine, `GTE/Mathematics/IntrCanonicalBox3Cylinder3.h` (File Version 8.0.2025.05.10)                                                                 | David Eberly                    | BSL-1.0                                                                       |
| `RayCylinderIntersection.h`                                                              | Geometric Tools Engine, `GTE/Mathematics/IntrLine3Cylinder3.h` (File Version 8.0.2025.05.10)                                                                         | David Eberly                    | BSL-1.0                                                                       |
| `RaySphereIntersection.h`                                                                | Geometric Tools Engine, `GTE/Mathematics/IntrLine3Sphere3.h`                                                                                                         | David Eberly                    | BSL-1.0                                                                       |
| `BoxCylinderIntersection.h` and `RayCylinderIntersection.h`: `Normalize`, `Orthonormalize`     | Geometric Tools Engine, `GTE/Mathematics/Vector.h`                                                                                                                   | David Eberly                    | BSL-1.0                                                                       |
| `BoxCylinderIntersection.h` and `RayCylinderIntersection.h`: `ComputeOrthogonalComplement` | Geometric Tools Engine, `GTE/Mathematics/Vector3.h`                                                                                                                  | David Eberly                    | BSL-1.0                                                                       |
| `CompGeomAlgorithms.h`: `SegmentSegmentDistance3D`                                       | Geometric Tools sample `GTE/Samples/Distance/DistanceSegments3/DistanceSegments3Console.cpp` (Eberly's modified version of Dan Sunday's `dist3D_Segment_to_Segment`) | David Eberly (after Dan Sunday) | BSL-1.0                                                                       |
| `CompGeomAlgorithms.h`: `ClosestPointOnTriangle`, `SphereTriangleCollision`              | *Real-Time Collision Detection*, `ClosestPtPointTriangle`, `TestSphereTriangle`                                                                                      | Christer Ericson                | RTCD book license (see [2.3](#23-real-time-collision-detection-book-license)) |
| `CPolyline.h`: `CPolyline::IsPointInside`                                                | `wn_PnPoly()`, "Inclusion of a Point in a Polygon", geomalgorithms.com                                                                                               | Dan Sunday                      | softSurfer notice (see [2.2](#22-dan-sunday--softsurfer-notice))              |
| `CRationalBezier.h`: `CRationalBezier::FromTo`, `CRationalBezier::SplitCurveByTolerance` | flo-bezier3, `fromTo()` and `splitByCurvatureAndLength()`                                                                                                            | Floris Steenkamp                | MIT (see [2.1](#21-mit-license-flo-bezier3))                                  |
| `MathAlgorithms.h`: `LagrangePolyMinExtrap`                                                | dlib, `dlib/optimization/optimization_line_search.h`, `lagrange_poly_min_extrap()`                                                                                   | Davis E. King                   | BSL-1.0                                                                       |
| `MathAlgorithms.h`: `FindMinSingleVariableFunction`                                        | dlib, `dlib/optimization/optimization_line_search.h`, `find_min_single_variable()`                                                                                   | Davis E. King                   | BSL-1.0                                                                       |

### Copyright notices of the Boost-licensed sources

The Boost Software License requires the original copyright notices to be
retained; they are reproduced in the headers of the adapted files and listed
here for convenience. The full Boost Software License text is in `LICENSE`.

- **Geometric Tools Engine**
  David Eberly, Geometric Tools, Redmond WA 98052
  Copyright (c) 1998-2025
  Distributed under the Boost Software License, Version 1.0.
  https://github.com/davideberly/GeometricTools

- **dlib**
  Copyright (C) 2008 Davis E. King (davis@dlib.net)
  License: Boost Software License.
  https://github.com/davisking/dlib

---

## 2. Third-party license texts

### 2.1 MIT License (flo-bezier3)

Applies to `CRationalBezier::FromTo` and `CRationalBezier::SplitCurveByTolerance`
in `CRationalBezier.h`.
Original source: https://github.com/FlorisSteenkamp/FloBezier

```
The MIT License (MIT)

Copyright © 2026 Floris Steenkamp

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

### 2.2 Dan Sunday / softSurfer notice

Applies to `CPolyline::IsPointInside`.
Original source: https://geomalgorithms.com/a03-_inclusion.html
(archived copy: https://web.archive.org/web/20130126163405/http://geomalgorithms.com/a03-_inclusion.html)

```
Copyright 2000 softSurfer, 2012 Dan Sunday
This code may be freely used and modified for any purpose
providing that this copyright notice is included with it.
SoftSurfer makes no warranty for this code, and cannot be held
liable for any real or imagined damage resulting from its use.
Users of this code must verify correctness for their application.
```

### 2.3 Real-Time Collision Detection book license

Applies to `ClosestPointOnTriangle` and `SphereTriangleCollision`.
SPDX identifier used in GGeomLib file headers: `LicenseRef-RTCD`.

The code presented in the book *Real-Time Collision Detection* is distributed
under the Software License Agreement printed in the book ("About the CD-ROM",
p. 593). Its grant of license reads:

> We grant you a nonexclusive license to use the software for any purpose,
> commercial or non-commercial, as long as the following credit is included
> identifying the original source of the software: "from Real-Time Collision
> Detection by Christer Ericson, published by Morgan Kaufmann Publishers,
> © 2005 Elsevier Inc".

Required credit:

```
from Real-Time Collision Detection by Christer Ericson, published by Morgan
Kaufmann Publishers, © 2005 Elsevier Inc
```

---

## 3. References

The following works describe algorithms that were implemented independently
in GGeomLib (no code reused). They are listed as a courtesy and as suggested
reading.

### Books

- P. J. Schneider, D. H. Eberly, *Geometric Tools for Computer Graphics*,
  Morgan Kaufmann, 2003. (Plane-segment intersection, p. 484; ray-circle
  intersection, p. 248.)
- C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005.
  (Uniform grids.)
- G. Farin, *Curves and Surfaces for CAGD: A Practical Guide*, 5th ed.,
  Morgan Kaufmann, 2002. (Rational Bézier curves.)
- L. Piegl, W. Tiller, *The NURBS Book*, 2nd ed., Springer, 1997.
  (Rational Bézier curves.)
- A. Ruszczyński, *Nonlinear Optimization*, Princeton University Press, 2006,
  section 5.2. (Quadratic interpolation for line search.)
- W. H. Press et al., *Numerical Recipes*, 3rd ed., Cambridge University Press,
  2007, section 5.6. (Numerically stable quadratic formula.)

### Papers and articles

- Y.-D. Liang, B. A. Barsky, "A New Concept and Method for Line Clipping",
  *ACM Transactions on Graphics* 3(1), pp. 1-18, 1984.
  (`ClipSegment2DToBox`, `Segment2DBoxIntersection`.)
- D. Sunday, "Distance between 3D Lines & Segments", geomalgorithms.com,
  https://geomalgorithms.com/a07-_distance.html. (`SegmentSegmentDistance3D`.)
- D. Eberly, "Robust Computation of Distance Between Line Segments",
  Geometric Tools,
  https://www.geometrictools.com/Documentation/DistanceLine3Line3.pdf.
- D. Eberly, "Intersection of a Box and a Cylinder", Geometric Tools,
  https://www.geometrictools.com/Documentation/IntersectionBoxCylinder.pdf.
- D. Eberly, "Dynamic Collision Detection using Oriented Bounding Boxes",
  Geometric Tools. (`CTriangle::IsTriangleCollidingWithAABB`.)
- T. Akenine-Möller, "Fast 3D Triangle-Box Overlap Testing",
  *Journal of Graphics Tools* 6(1), pp. 29-33, 2001.
- D. Eberly, "Triangulation by Ear Clipping", Geometric Tools.
  (`TriangulateByEarClipping`.)
- G. H. Meisters, "Polygons have ears", *American Mathematical Monthly*
  82(6), pp. 648-651, 1975.
- I. Quilez, "Disk - bounding box", https://iquilezles.org/articles/diskbbox/.
  (`GetCylinderAABB`.)
- Shoelace formula, https://en.wikipedia.org/wiki/Shoelace_formula.
  (`CalculateProfileArea`, `IsPrfCW`.)
