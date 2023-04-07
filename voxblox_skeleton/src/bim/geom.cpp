#include "voxblox_skeleton/bim/geom.h"

#include <glog/logging.h>

namespace geom {

bool Cube::isPointInside(const Point &point) const {
  if (!aabb.isPointInside(point))
    return false;

  for (const auto &tri : triangles()) {
    const auto u = (tri.center() - point).normalized();
    const auto n = tri.normal();

    if (u.dot(n) < 0) {
      return false;
    }
  }

  return true;
}

Point Cube::center() const { return (a + b + c + d + e + f + g + h) / 8; }

void Cube::print() const {
  auto tris = triangles();
  auto cc = center();

  printf("[cube] center [%f, %f, %f] ok %i\n", cc.x(), cc.y(), cc.z(),
         isPointInside(cc));

  for (int i = 0; i < tris.size(); ++i) {
    auto c = tris[i].center();
    auto n = tris[i].normal();
    printf("[%i] center [%f, %f, %f] normal [%f, %f, %f] \n", c.x(), c.y(),
           c.z(), n.x(), n.y(), n.z());
  }
}

Eigen::Vector3f Triangle::normal() const {
  auto cb = b - c;
  auto ca = a - c;
  return (cb.cross(ca)).normalized();
}

Point Triangle::center() const { return (a + b + c) / 3.0f; }

std::vector<Triangle> Cube::triangles() const {
  if (is_plane) {
    return std::vector<Triangle>{
        Triangle{d, g, h},
        Triangle{d, c, g},
    };
  } else {
    return std::vector<Triangle>{Triangle{a, e, f}, Triangle{a, f, b}, //
                                 Triangle{a, d, h}, Triangle{a, h, e},
                                 Triangle{a, c, d}, Triangle{a, b, c},
                                 Triangle{e, g, f}, Triangle{e, h, g},
                                 Triangle{b, g, c}, Triangle{b, f, g},
                                 Triangle{c, g, h}, Triangle{c, h, d}};
  }
} // namespace geom

// This function is heavily based on the example on page 141 of:
// "Real-time collision detection" by Christer Ericson
float TriangleGeometer::getDistanceToPoint(const Point &point) const {
  using Vector = Point;

  // Check if the point is in the region outside A
  const Vector ab = vertices_.b - vertices_.a;
  const Vector ac = vertices_.c - vertices_.a;
  const Vector ap = point - vertices_.a;
  const float d1 = ab.dot(ap);
  const float d2 = ac.dot(ap);
  if (d1 <= 0.0f && d2 <= 0.0f) {
    // The barycentric coordinates are (1,0,0) => the closest point is vertex_a
    return (vertices_.a - point).norm();
  }

  // Check if P in vertex region outside B
  const Vector bp = point - vertices_.b;
  const float d3 = ab.dot(bp);
  const float d4 = ac.dot(bp);
  if (d3 >= 0.0f && d4 <= d3) {
    // The barycentric coordinates are (0,1,0) => the closest point is vertex_b
    return (vertices_.b - point).norm();
  }

  // Check if P in edge region of AB, if so return projection of P onto AB
  const float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    const float v = d1 / (d1 - d3);
    // The barycentric coordinates are (1-v,v,0)
    Point closest_pt = vertices_.a + v * ab;
    return (closest_pt - point).norm();
  }

  // Check if P in vertex region outside C
  Vector cp = point - vertices_.c;
  const float d5 = ab.dot(cp);
  const float d6 = ac.dot(cp);
  if (d6 >= 0.0f && d5 <= d6) {
    // The barycentric coordinates are (0,0,1) => the closest point is vertex_c
    return (vertices_.c - point).norm();
  }

  // Check if P in edge region of AC, if so return projection of P onto AC
  const float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    const float w = d2 / (d2 - d6);
    // The barycentric coordinates are (1-w,0,w)
    const Point closest_pt = vertices_.a + w * ac;
    return (closest_pt - point).norm();
  }

  // Check if P in edge region of BC, if so return projection of P onto BC
  const float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    // The barycentric coordinates are (0,1-w,w)
    const Point closest_pt = vertices_.b + w * (vertices_.c - vertices_.b);
    return (closest_pt - point).norm();
  }

  // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
  const float denom = 1.0f / (va + vb + vc);
  const float v = vb * denom;
  const float w = vc * denom;
  // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
  const Point closest_pt = vertices_.a + ab * v + ac * w;
  return (closest_pt - point).norm();
}

AABB TriangleGeometer::getAABB() const {
  return AABB(vertices_.a, vertices_.b, vertices_.c);
}

bool TriangleGeometer::getRayIntersection(
    const Point2D &ray_yz, Point *barycentric_coordinates) const {
  CHECK_NOTNULL(barycentric_coordinates);

  // Express the vertices A, B and C relative to the point
  const Point2D vertex_a_relative = vertices_.a.tail<2>() - ray_yz;
  const Point2D vertex_b_relative = vertices_.b.tail<2>() - ray_yz;
  const Point2D vertex_c_relative = vertices_.c.tail<2>() - ray_yz;

  // Check the orientation of B relative to C
  // NOTE: As a byproduct of the orientation checks, we also compute the signed
  //       areas. After being normalized, these correspond to the barycentric
  //       coordinates (see the end of this method).
  const int sign_a =
      getRelativeOrientation(vertex_b_relative, vertex_c_relative,
                             &barycentric_coordinates->operator[](0));
  // If the relative orientation is zero, vertices B and C must be equal.
  // This would mean that the triangle has no surface area and the ray therefore
  // cannot intersect it.
  if (sign_a == 0)
    return false;

  // Check the orientation of C relative to A
  const int sign_b =
      getRelativeOrientation(vertex_c_relative, vertex_a_relative,
                             &barycentric_coordinates->operator[](1));
  // If the signs differ, the solution to the intersection equation does not lie
  // inside the triangle (i.e. the ray does not intersect the triangle)
  if (sign_b != sign_a)
    return false;

  // Check the orientation of A relative to B
  const int sign_c =
      getRelativeOrientation(vertex_a_relative, vertex_b_relative,
                             &barycentric_coordinates->operator[](2));
  // If the signs differ, the solution to the intersection equation does not lie
  // inside the triangle (i.e. the ray does not intersect the triangle)
  if (sign_c != sign_a)
    return false;

  // If the point is within the triangle,
  // the barymetric coordinates should never be zero
  const double sum = barycentric_coordinates->sum();
  CHECK_NE(sum, 0);

  // Normalize the barycentric coordinates
  barycentric_coordinates->operator[](0) /= sum;
  barycentric_coordinates->operator[](1) /= sum;
  barycentric_coordinates->operator[](2) /= sum;

  return true;
}

int TriangleGeometer::getRelativeOrientation(const Point2D &vertex_one,
                                             const Point2D &vertex_two,
                                             float *twice_signed_area) const {
  CHECK_NOTNULL(twice_signed_area);

  // Compute the signed area (scaled by factor 2, but we don't care)
  *twice_signed_area =
      vertex_one[1] * vertex_two[0] - vertex_one[0] * vertex_two[1];

  // Return the relative orientation
  if (*twice_signed_area > 0)
    return 1;
  else if (*twice_signed_area < 0)
    return -1;
  else if (vertex_two[1] > vertex_one[1])
    return 1;
  else if (vertex_two[1] < vertex_one[1])
    return -1;
  else if (vertex_one[0] > vertex_two[0])
    return 1;
  else if (vertex_one[0] < vertex_two[0])
    return -1;
  else
    return 0; // only true when both vertices are equal
}

// Ref:
// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution.html
bool TriangleGeometer::getRayIntersection3(const Point &orig,
                                           Eigen::Vector3f dir,
                                           float &t) const {
  const float kEpsilon = 1e-6;

  // compute the plane's normal
  Eigen::Vector3f v0v1 = vertices_.b - vertices_.a;
  Eigen::Vector3f v0v2 = vertices_.c - vertices_.a;
  // no need to normalize
  Eigen::Vector3f N = v0v1.cross(v0v2); // N
  float area2 = N.norm();

  // Step 1: finding P

  // check if the ray and plane are parallel.
  float NdotRayDirection = N.dot(dir);
  if (fabs(NdotRayDirection) < kEpsilon) // almost 0
    return false; // they are parallel, so they don't intersect!

  // compute d parameter using equation 2
  float d = -N.dot(vertices_.a);

  // compute t (equation 3)
  t = -(N.dot(orig) + d) / NdotRayDirection;

  // check if the triangle is behind the ray
  if (t < 0)
    return false; // the triangle is behind

  // compute the intersection point using equation 1
  Eigen::Vector3f P = orig + t * dir;

  // Step 2: inside-outside test
  Eigen::Vector3f C; // vector perpendicular to triangle's plane

  // edge 0
  Eigen::Vector3f edge0 = vertices_.b - vertices_.a;
  Eigen::Vector3f vp0 = P - vertices_.a;
  C = edge0.cross(vp0);
  if (N.dot(C) < 0)
    return false; // P is on the right side

  // edge 1
  Eigen::Vector3f edge1 = vertices_.c - vertices_.b;
  Eigen::Vector3f vp1 = P - vertices_.b;
  C = edge1.cross(vp1);
  if (N.dot(C) < 0)
    return false; // P is on the right side

  // edge 2
  Eigen::Vector3f edge2 = vertices_.a - vertices_.c;
  Eigen::Vector3f vp2 = P - vertices_.c;
  C = edge2.cross(vp2);
  if (N.dot(C) < 0)
    return false; // P is on the right side;

  return true; // this ray hits the triangle
}

std::vector<Intersection>
getIntersections(const std::vector<Triangle> &triangles, const Point &origin,
                 const Point &target) {

  const auto taor = (target - origin);
  const auto l = taor.norm();
  const auto dir = taor / l;

  std::vector<Intersection> intersections;

  for (const auto &tri : triangles) {
    auto geom = TriangleGeometer(tri);
    float t = 0;
    if (geom.getRayIntersection3(origin, dir, t) && t < l) {
      intersections.emplace_back(
          Intersection{.point = dir * t, .normal = tri.normal(), .t = t});
    }
  }

  // sort so that the closest collision is the first one in the array
  std::sort(intersections.begin(), intersections.end(),
            [](const auto &a, const auto &b) { return a.t < b.t; });

  return intersections;
}

} // namespace geom