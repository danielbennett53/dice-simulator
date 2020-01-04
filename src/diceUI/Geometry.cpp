#include "Geometry.h"
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


std::vector<Eigen::Vector3d> triTriIntersection3d(std::vector<Eigen::Vector3d> &A,
                                                  std::vector<Eigen::Vector3d> &B)
{
    // Algorithm comes from Devillers et al. Faster Triangle-Triangle Intersection Tests
    // Olivier Devillers, Philippe Guigue.  Faster Triangle-Triangle Intersection Tests.  RR-4488, INRIA.2002. ￿inria-00072100￿
    // https://hal.inria.fr/inria-00072100/document

    // Output vector
    std::vector<Eigen::Vector3d> iSectPoints;

    // Check inputs are triangles
    if ( (A.size() != 3) || (B.size() != 3))
        return iSectPoints;

    // Check both triangles to see if they intersect the other plane
    std::vector<double> detsA;
    for (const auto& s : A) {
        Eigen::Matrix3d mat;
        mat << B[0] - s, B[1] - s, B[2] - s;
        detsA.push_back(mat.determinant());
    }
    const double Acheck1 = detsA[0] * detsA[1];
    const double Acheck2 = detsA[0] * detsA[2];
    if ( (Acheck1 >= 0.0) && (Acheck2 >= 0.0) )
        return iSectPoints;

    std::vector<double> detsB;
    for (const auto& s : B) {
        Eigen::Matrix3d mat;
        mat << A[0] - s, A[1] - s, A[2] - s;
        detsB.push_back(mat.determinant());
    }
    const double Bcheck1 = detsB[0] * detsB[1];
    const double Bcheck2 = detsB[0] * detsB[2];
    if ( (Bcheck1 >= 0.0) && (Bcheck2 >= 0.0) )
        return iSectPoints;

    // Reorder triangles. first index is the vertex that is alone on one side of the plane
    int idx = 0;
    if (Acheck1 > 0.0)
        idx = 2;
    else if (Acheck2 > 0.0)
        idx = 1;

    Eigen::Vector3d temp_vtx = A[idx];
    A[idx] = A[0];
    A[0] = temp_vtx;

    idx = 0;
    if (Bcheck1 > 0.0)
        idx = 2;
    else if (Bcheck2 > 0.0)
        idx = 1;

    temp_vtx = B[idx];
    B[idx] = B[0];
    B[0] = temp_vtx;

    // Triangle plane normal vectors and other reused quantities
    const Eigen::Vector3d n1 = (A[1] - A[0]).cross(A[2] - A[0]);
    const Eigen::Vector3d n2 = (B[1] - B[0]).cross(B[2] - B[0]);
    const Eigen::Vector3d L = n1.cross(n2);

    const Eigen::Vector3d p2_p1 = B[0] - A[0];

    // Find intersection between triangle vertices and line of intersection
    // s and t parameterize triangle A, u and v parameterize triangle B
    const double s = p2_p1.dot(n2) / (A[1] - A[0]).dot(n2);
    const double t = p2_p1.dot(n2) / (A[2] - A[0]).dot(n2);
    const double u = -p2_p1.dot(n1) / (B[1] - B[0]).dot(n1);
    const double v = -p2_p1.dot(n1) / (B[2] - B[0]).dot(n1);

    // Calculate intersection points and project them onto line segment
    std::vector<Eigen::Vector3d> points = {
        s * (A[1] - A[0]) + A[0],
        t * (A[2] - A[0]) + A[0],
        u * (B[1] - B[0]) + B[0],
        v * (B[2] - B[0]) + B[0]
    };

    // Find the order of point indices based on the projection onto the intersection line
    std::vector<double> projection;
    for (const auto& p : points) {
        projection.emplace_back(p.dot(L));
    }
    std::vector<int> indices = {0, 1, 2, 3};
    auto pointSort = [&projection] (int i, int j) {
        return projection[i] < projection[j];
    };
    std::sort(indices.begin(), indices.end(), pointSort);

    // No intersection if the first two elements are from one triangle
    if ( ((indices[0] < 2) && (indices[1] < 2)) ||
         ((indices[0] > 1) && (indices[1] > 1)) ) {
        return iSectPoints;
    }
    else {
        iSectPoints.push_back(points[indices[1]]);
        iSectPoints.push_back(points[indices[2]]);
    }

    return iSectPoints;
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


bool meshIntersection(const Mesh& m1, const Eigen::Transform<double, 3, Eigen::Affine>& tf1,
                      const Mesh& m2, const Eigen::Transform<double, 3, Eigen::Affine>& tf2,
                      std::vector<Eigen::Vector3d>& iSectPoints, Eigen::Vector3d& iSectVector)
{
    Eigen::Transform<double, 3, Eigen::Affine> tf2to1 = tf1.inverse() * tf2;
    Eigen::Transform<double, 3, Eigen::Affine> tf1to2 = tf2.inverse() * tf1;

    // Check if the sphere representations of the meshes overlap
    double meshDist = (tf2to1 * m2.centroid_ - m1.centroid_).norm();
    if (meshDist > (m1.radius_ + m2.radius_))
        return false;

    // Find faces that might intersect
    std::vector<Face> m1FaceOpts;
    std::vector<Face> m2FaceOpts;
    Eigen::Vector3d m1Centroidin2 = tf1to2 * m1.centroid_;
    Eigen::Vector3d m2Centroidin1 = tf2to1 * m2.centroid_;
    for (const auto& f : m1.faces_) {
        // Based on the angle of the face with the other mesh, calculate the minimum distance between the centroid
        // of the face and the surface of the mesh to guarantee no intersection
        double face_radius = (f.centroid - m2Centroidin1).normalized().cross(f.normal).norm() * f.radius;
        double dist = (f.centroid - m2Centroidin1).norm();
        if (dist < (face_radius + m2.radius_))
            m1FaceOpts.emplace_back(f);
    }
    for (const auto& f : m2.faces_) {
        // Based on the angle of the face with the other mesh, calculate the minimum distance between the centroid
        // of the face and the surface of the mesh to guarantee no intersection
        double face_radius = (f.centroid - m1Centroidin2).normalized().cross(f.normal).norm() * f.radius;
        double dist = (f.centroid - m1Centroidin2).norm();
        if (dist < (face_radius + m1.radius_))
            m2FaceOpts.emplace_back(f);
    }

    // If there are no potential face intersections
    if (m1FaceOpts.empty() || m2FaceOpts.empty())
        return false;

    // Stores intersection points sorted by face
    std::vector<std::vector<Eigen::Vector3d>> m1FaceIsct(m1FaceOpts.size());
    std::vector<std::vector<Eigen::Vector3d>> m2FaceIsct(m2FaceOpts.size());
    // Stores list of vertices of all face options
    std::vector<int> m1VertexOpts;
    std::vector<int> m2VertexOpts;

    // Check all possible collisions
    int num_isects = 0;
    for (unsigned int i = 0; i < m1FaceOpts.size(); ++i) {
        std::vector<Eigen::Vector3d> m1Tri = {
            tf1to2 * m1.vertices_[m1FaceOpts[i].vertexIdxs[0]].point,
            tf1to2 * m1.vertices_[m1FaceOpts[i].vertexIdxs[1]].point,
            tf1to2 * m1.vertices_[m1FaceOpts[i].vertexIdxs[2]].point };
        m1VertexOpts.insert(m1VertexOpts.end(), m1FaceOpts[i].vertexIdxs.begin(), m1FaceOpts[i].vertexIdxs.end());

        for (unsigned int j = 0; j < m2FaceOpts.size(); ++j) {
            if ( i == 0 ) {
                m2VertexOpts.insert(m2VertexOpts.end(), m2FaceOpts[j].vertexIdxs.begin(), m2FaceOpts[j].vertexIdxs.end());
            }
            std::vector<Eigen::Vector3d> m2Tri = {
                m2.vertices_[m2FaceOpts[j].vertexIdxs[0]].point,
                m2.vertices_[m2FaceOpts[j].vertexIdxs[1]].point,
                m2.vertices_[m2FaceOpts[j].vertexIdxs[2]].point };
            // Find intersection points
            auto isects = triTriIntersection3d(m1Tri, m2Tri);
            num_isects += isects.size();
            m1FaceIsct[i].insert(m1FaceIsct[i].end(), isects.begin(), isects.end());
            m2FaceIsct[j].insert(m2FaceIsct[j].end(), isects.begin(), isects.end());
        }
    }

    if (num_isects == 0)
        return false;

    // Get list of unique vertices
    std::sort(m1VertexOpts.begin(), m1VertexOpts.end());
    std::sort(m2VertexOpts.begin(), m2VertexOpts.end());
    m1VertexOpts.erase(std::unique(m1VertexOpts.begin(), m1VertexOpts.end()), m1VertexOpts.end());
    m2VertexOpts.erase(std::unique(m2VertexOpts.begin(), m2VertexOpts.end()), m2VertexOpts.end());

    // Check if any vertices of either mesh are inside the other mesh
    for (const auto& vtx : m1VertexOpts) {
        bool inside = true;
        for (const auto& face : m2.faces_) {
            if ( (m1.vertices_[vtx].point - m2.vertices_[face.vertexIdxs[0]].point).dot(face.normal) > 0 ) {
                inside = false;
                break;
            }
        }
        if (inside) {
            for (auto faceIdx : m1.vertices_[vtx].connectedFaces) {
                m1FaceIsct[faceIdx].push_back(m1.vertices_[vtx].point);
            }
        }
    }
    for (const auto& vtx : m2VertexOpts) {
        bool inside = true;
        for (const auto& face : m1.faces_) {
            if ( (m2.vertices_[vtx].point - m1.vertices_[face.vertexIdxs[0]].point).dot(face.normal) > 0 ) {
                inside = false;
                break;
            }
        }
        if (inside) {
            for (auto faceIdx : m2.vertices_[vtx].connectedFaces) {
                m2FaceIsct[faceIdx].push_back(m2.vertices_[vtx].point);
            }
        }
    }

    // Find area of intersectioni
    iSectVector.setZero();
    iSectPoints.clear();
    for (unsigned int i = 0; i < m1FaceIsct.size(); ++i) {
        if (m1FaceIsct[i].size() < 3)
            continue;
        auto pts = planeProjection(m1FaceIsct[i], m1.faces_[i].normal);
        iSectVector += m1.faces_[i].normal * polygonArea(pts);
    }
    for (unsigned int i = 0; i < m2FaceIsct.size(); ++i) {
        if (m2FaceIsct[i].size() < 3)
            continue;
        auto pts = planeProjection(m2FaceIsct[i], m2.faces_[i].normal);
        iSectVector += m2.faces_[i].normal * polygonArea(pts);
    }

    if (iSectVector.norm() < 1e-6)
        return false;

    std::cout << "Isect vector: " << std::endl << iSectVector << std::endl << std::endl;
    return true;
}

} //namespace geometry
