#pragma once

#include <Eigen/Core>
#include <utility>

namespace geom {

typedef Eigen::Vector3f Point;
typedef Eigen::Vector2f Point2D;

struct AABB {
  Point min = {INFINITY, INFINITY, INFINITY};
  Point max = {-INFINITY, -INFINITY, -INFINITY};

  AABB(){};
  AABB(const Point &a, const Point &b, const Point &c) {
    addPoint(a);
    addPoint(b);
    addPoint(c);
  }

  inline void addPoint(const Point &a) {
    min = min.cwiseMin(a);
    max = max.cwiseMax(a);
  }
};

struct Triangle {
  Point a;
  Point b;
  Point c;

  Triangle(Point a, Point b, Point c) : a(a), b(b), c(c) {}
};

struct Cube {
  bool isPointInside(Point point) const;
  std::vector<Triangle> triangles() const;

  Point a;
  Point b;
  Point c;
  Point d;

  Point e;
  Point f;
  Point g;
  Point h;

  AABB aabb;

  bool is_plane;
};

class TriangleGeometer {
public:
  explicit TriangleGeometer(Triangle vertex_coordinates)
      : vertices_(vertex_coordinates) {}

  AABB getAABB() const;

  float getDistanceToPoint(const Point &point) const;

  bool getRayIntersection(const Point2D &ray_yz,
                          Point *barycentric_coordinates) const;

private:
  const Triangle vertices_;

  int getRelativeOrientation(const Point2D &vertex_one,
                             const Point2D &vertex_two,
                             float *twice_signed_area) const;
};
} // namespace geom
