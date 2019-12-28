#include "Geometry.h"
#include <iostream>
#include <stack>


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


// Calculates intersection points of triangle with line, assuming specific order of vectors
// VTX: Triangle vertices
// VV: Simplified projection of triangle vertices onto intersection line
// D: Intersection line vector
static inline void isect2(const std::vector<Eigen::Vector3d> &VTX, const Eigen::Vector3d &VV,
                          const Eigen::Vector3d &D, Eigen::Vector2d &isect,
                          std::vector<Eigen::Vector3d> &isectpoints)
{
    isectpoints.clear();
    double tmp = D[0] / (D[0] - D[1]);
    isect[0] = VV[0] + (VV[1] - VV[0]) * tmp;
    Eigen::Vector3d diff = (VTX[1] - VTX[0]) * tmp;
    isectpoints.push_back(diff + VTX[0]);
    tmp = D[0] / (D[0] - D[2]);
    isect[1] = VV[0] + (VV[2] - VV[0]) * tmp;
    diff = (VTX[2] - VTX[0]) * tmp;
    isectpoints.push_back(diff + VTX[0]);
}


// VTX: Triangle vertices
// VV: Simplified projection of triangle vertices onto intersection line
// D: Intersection line vector
// D0D1, D0D2: product of projection onto triangle plane, used to check which vertices are on which side of other plane
// Returns 1 if triangles are coplanar
static inline int isectLineIntervals(const std::vector<Eigen::Vector3d> &VTX, const Eigen::Vector3d &VV,
                                     const Eigen::Vector3d &D, const double D0D1, const double D0D2,
                                     Eigen::Vector2d &isect, std::vector<Eigen::Vector3d> &isectpoints)
{
    if (D0D1 > 0.0) {
        // here we know that D0D2<=0.0
        // that is D0, D1 are on the same side, D2 on the other or on the plane
        isect2({VTX[2], VTX[0], VTX[1]}, {VV[2], VV[0], VV[1]}, {D[2], D[0], D[1]},
               isect, isectpoints);
    } else if (D0D2 > 0.0) {
        isect2({VTX[1], VTX[0], VTX[2]}, {VV[1], VV[0], VV[2]}, {D[1], D[0], D[2]},
               isect, isectpoints);
    } else if ( (D[1] * D[2] > 0.0) || (D[0] != 0.0) ) {
        isect2({VTX[0], VTX[1], VTX[2]}, {VV[0], VV[1], VV[2]}, {D[0], D[1], D[2]},
               isect, isectpoints);
    } else if (D[1] != 0.0) {
        isect2({VTX[1], VTX[0], VTX[2]}, {VV[1], VV[0], VV[2]}, {D[1], D[0], D[2]},
               isect, isectpoints);
    } else if (D[2] != 0.0) {
        isect2({VTX[2], VTX[0], VTX[1]}, {VV[2], VV[0], VV[1]}, {D[2], D[0], D[1]},
               isect, isectpoints);
    } else {
        // Triangles are coplanar
        return 1;
    }
    return 0;
}

// Handy definition for sorting two numbers so that a <= b
#define SORT(a, b, smallest)   \
    if (a > b) {                \
        double c;               \
        c = a;                  \
        a = b;                  \
        b = c;                  \
        smallest = 1;           \
    } else smallest = 0         \

bool triTriIntersection3d(const std::vector<Eigen::Vector3d> &V,
                          const std::vector<Eigen::Vector3d> &U,
                          std::vector<Eigen::Vector3d> &intersectionPoints,
                          int &coplanar)
{
    // Moller triangle-triangle intersection test, from
    // http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tritri_isectline.txt
    const double e = 0.000001;
    if ((V.size() != 3) || (U.size() != 3)) {
        std::cout << "triTriIntersection: Inputs not triangles" << std::endl;
        return false;
    }

    intersectionPoints.clear();

    // Define T1 plane and check T2 vertices against it
    Eigen::Vector3d N1 = (V[1] - V[0]).cross(V[2] - V[0]);
    double d1 = -N1.dot(V[0]);
    // Signed distance of T2 vertices to T1
    std::vector<double> du;
    for (int i = 0; i < 3; ++i) {
        double d = N1.dot(U[i]) + d1;
        if (abs(d) < e)
            d = 0;
        du.push_back(d);
    }

    double du0du1 = du[0] * du[1];
    double du0du2 = du[0] * du[2];

    // No intersection occurs (no plane intersection)
    if ((du0du1 > 0.0) && (du0du2 > 0.0))
        return 0;

    // Define T2 plane and check T1 vertices against it
    Eigen::Vector3d N2 = (U[1] - U[0]).cross(U[2] - U[0]);
    double d2 = -N2.dot(U[0]);
    // Signed distance of T2 vertices to T1
    std::vector<double> dv;
    for (int i = 0; i < 3; ++i) {
        double d = N2.dot(V[i]) + d2;
        if (fabs(d) < e)
            d = 0;
        dv.push_back(d);
    }

    double dv0dv1 = dv[0] * dv[1];
    double dv0dv2 = dv[0] * dv[2];

    // No intersection occurs (no plane intersection)
    if ((dv0dv1 > 0.0) && (dv0dv2 > 0.0))
        return 0;

    // Find plane intersection line
    Eigen::Vector3d D = N1.cross(N2);

    // Compute value and index of largest component of D
    double max = fabs(D[0]);
    int index = 0;
    double b = fabs(D[1]);
    double c = fabs(D[2]);
    if (b > max) { max = b; index = 1; }
    if (c > max) { max = c; index = 2; }

    // Simplified projection onto intersection line
    Eigen::Vector3d Vp(V[0][index], V[1][index], V[2][index]);
    Eigen::Vector3d Up(U[0][index], U[1][index], U[2][index]);

    // Compute intervals
    Eigen::Vector2d isect1, isect2;
    std::vector<Eigen::Vector3d> isectPointA, isectPointB;

    // Triangle 1
    coplanar = isectLineIntervals(V, Vp, D, dv0dv1, dv0dv2, isect1, isectPointA);

    if (coplanar) {
        // TODO: Handle coplanar case
        return 0;
    }

    // Triangle 2
    isectLineIntervals(U, Up, D, du0du1, du0du2, isect2, isectPointB);

    int smallest1, smallest2;
    SORT(isect1[0], isect1[1], smallest1);
    SORT(isect2[0], isect2[1], smallest2);

    if ( (isect1[1] < isect2[0]) || (isect2[1] < isect1[0]) )
        return 0;

    // Now we know the triangles intersect
    if (isect2[0] < isect1[0]) {
        if (smallest1 == 0)
            intersectionPoints.push_back(isectPointA[0]);
        else
            intersectionPoints.push_back(isectPointA[1]);

        if (isect2[1] < isect1[1]) {
            if (smallest2 == 0)
                intersectionPoints.push_back(isectPointB[1]);
            else
                intersectionPoints.push_back(isectPointB[0]);
        } else {
            if (smallest1 == 0)
                intersectionPoints.push_back(isectPointA[1]);
            else
                intersectionPoints.push_back(isectPointA[0]);
        }
    } else {
        if (smallest2 == 0)
            intersectionPoints.push_back(isectPointB[0]);
        else
            intersectionPoints.push_back(isectPointB[1]);

        if (isect2[1] > isect1[1]) {
            if (smallest1 == 0)
                intersectionPoints.push_back(isectPointA[1]);
            else
                intersectionPoints.push_back(isectPointA[0]);
        } else {
            if (smallest2 == 0)
                intersectionPoints.push_back(isectPointB[1]);
            else
                intersectionPoints.push_back(isectPointB[0]);
        }
    }
    for (const auto& v : intersectionPoints)
    {
        std::cout << v << std::endl;
    }
    return 1;
}
#undef SORT


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
        static const double eps = 1e-6;
        auto ir = i - ref;
        auto jr = j - ref;
        // Handle singularity case
        if ((std::fabs(ir[1]) < eps) || (std::fabs(jr[1]) < eps))
            return (ir[0] < jr[0]);
        // Return slope of line
        return (ir[0]/ir[1]) < (jr[0]/jr[1]);
    };
    // Sort based on relative angle with the reference point
    std::sort(points.begin()+1, points.end(), pointCompare);
}


std::vector<Eigen::Vector2d> convexHull2D(std::vector<Eigen::Vector2d> &points)
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


std::vector<Eigen::Vector3d> planeProjection(const std::vector<Eigen::Vector3d> &points,
                                             const Eigen::Vector3d &normal)
{
    // Find rotation from global to local coords
    Eigen::Quaterniond tf = Eigen::Quaterniond::FromTwoVectors(normal, Eigen::Vector3d(0,0,1));
    tf.normalize();
    // Output vector
    std::vector<Eigen::Vector3d> out;

    for (const auto& p : points) {
        out.push_back(tf * p);
    }

    return out;
}

} //namespace geometry
