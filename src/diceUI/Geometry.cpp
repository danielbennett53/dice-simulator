#include "Geometry.h"
#include "ConvexPolytope.h"
#include <iostream>

namespace geometry {

double rayIntersectsTriangle(const Eigen::Vector3d &rayOrigin,
                             const Eigen::Vector3d &rayVector,
                             const std::vector<Eigen::Vector3d> &verts)
{
    // Check that verts is the right size
    if (verts.size() != 3)
        return -1;

    // Algorithm is Moller-Trumbore
    // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
    const double tol = 0.0000001;
    Eigen::Vector3d e1, e2, h, s, q;
    double a, f, u, v;

    e1 = verts[1] - verts[0];
    e2 = verts[2] - verts[0];

    h = rayVector.cross(e2);
    a = e1.dot(h);
    if (std::fabs(a) < tol)
        return -1; // Ray is parallel to triangle

    f = 1.0/a;
    s = rayOrigin - verts[0];
    u = f * s.dot(h);
    if ((u < 0.0) || (u > 1.0))
        return -1;

    q = s.cross(e1);
    v = f * rayVector.dot(q);
    if ((v < 0.0) || (u + v > 1.0))
        return -1;

    // Compute t to find out where the intersection point is on the line
    return f * e2.dot(q);
}


void sortCCW(std::vector<Eigen::Vector2d> &points)
{
    // Reference point (lowest y value)
    Eigen::Vector2d ref = points[0];
    unsigned int idx = 0;
    // Find point with lowest y value
    for (unsigned int i = 1; i < points.size(); ++i) {
        if ((points[i][1] < ref[1]) || ((points[i][1] == ref[1]) && (points[i][0] < ref[0]))) {
            ref = points[i];
            idx = i;
        }
    }
    // Swap the reference into the first position of the vector
    auto temp = points[0];
    points[0] = ref;
    points[idx] = temp;

    // Comparison function for sorting
    auto pointCompare = [&ref](Eigen::Vector2d i, Eigen::Vector2d j) {
        auto ir = i - ref;
        auto jr = j - ref;
        // Return slope of line
        return (ir[0]/ir[1]) < (jr[0]/jr[1]);
    };
    // Sort based on relative angle with the reference point
    std::sort(points.begin()+1, points.end(), pointCompare);
}


std::vector<Eigen::Vector2d> convexHull2d(std::vector<Eigen::Vector2d> &points)
{
    // This is based on the Graham Scan algorithm
    // Sort points into CCW order
    sortCCW(points);

    // Initialize stack with first 2 points
    std::vector<Eigen::Vector2d> hull;
    hull.push_back(points[0]);
    hull.push_back(points[1]);

    // Lambda for calculating direction
    auto dirCalc = [] (const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3) {
        return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0]);
    };

    // Iterate through points to find convex hull
    for (unsigned int i = 2; i < points.size(); ++i) {
        auto &p3 = points[i];
        auto &p2 = hull.back();
        hull.pop_back();
        auto &p1 = hull.back();

        // Compute Z-component of cross product of vectors from last three points
        // 0 - colinear, positive - CCW, negative - CW
        double dir = dirCalc(p1, p2, p3);
        while ((dir > 0) && (hull.size() > 1)) {
            p2 = hull.back();
            hull.pop_back();
            p1 = hull.back();
            dir = dirCalc(p1, p2, p3);
        }

        // Push the points back onto the stack
        hull.push_back(p2);
        hull.push_back(p3);
    }

    return hull;
}

// Finds the area of the convex hull of the points given in points
double polygonArea(std::vector<Eigen::Vector2d>& points)
{
    auto convexHull = convexHull2d(points);
    double sum = 0.0;
    for (unsigned int i = 0; i < convexHull.size() - 1; ++i) {
        sum += convexHull[i][0] * convexHull[i+1][1] - convexHull[i+1][0] * convexHull[i][1];
    }
    sum += convexHull.back()[0] * convexHull[0][1] - convexHull[0][0] * convexHull.back()[1];

    return std::fabs(0.5 * sum);
}


std::vector<Eigen::Vector2d> planeProjection(const std::vector<Eigen::Vector3d> &points,
                                             const Eigen::Vector3d &normal)
{
    // Find rotation from global to local coords
    Eigen::Quaterniond tf = Eigen::Quaterniond::FromTwoVectors(normal, Eigen::Vector3d(0,0,1));
    tf.normalize();
    // Output vector
    std::vector<Eigen::Vector2d> out;

    for (const auto& p : points) {
        Eigen::Vector3d tf_vec = tf * p;
        out.emplace_back(tf_vec[0], tf_vec[1]);
    }

    return out;
}


bool GJKIntersection(const Shape& s1, const Shape& s2)
{
    if (!s1.isectPossible(s2.getCentroid(), s2.getRadius()) &&
        !s2.isectPossible(s1.getCentroid(), s1.getRadius()) ) {
        return false;
    }

    // Initialize search
    Eigen::Vector3d support_dir = s2.getCentroid() - s1.getCentroid();
    Simplex smplx;
    unsigned int face_to_keep = 0;
    // Cap number of iterations
    for (int i = 0; i < 15; ++i) {
        smplx.addVertex(s1.support(support_dir) - s2.support(-support_dir), face_to_keep);
        if (smplx.getVertices().back()->getPos().dot(support_dir) < 0)
            return false;
        if (smplx.nearestSimplex(support_dir, face_to_keep))
            return true;
    }

    return false;
}

} //namespace geometry
