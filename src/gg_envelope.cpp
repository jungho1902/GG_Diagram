#include "gg_envelope.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace gg {

namespace {

constexpr double kEpsilon = 1e-9;

inline double cross(const Point2D& o, const Point2D& a, const Point2D& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

inline double cross2D(double ax, double ay, double bx, double by) {
    return ax * by - ay * bx;
}

inline double dot2D(double ax, double ay, double bx, double by) {
    return ax * bx + ay * by;
}

inline double squaredDistance(const Point2D& a, const Point2D& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

inline bool almostEqual(double a, double b, double eps = kEpsilon) {
    return std::abs(a - b) <= eps;
}

inline bool pointsEqual(const Point2D& lhs, const Point2D& rhs) {
    return almostEqual(lhs.x, rhs.x) && almostEqual(lhs.y, rhs.y);
}

std::vector<Point2D> makeSortedUnique(std::vector<Point2D> points) {
    if (points.empty()) {
        return points;
    }

    std::sort(points.begin(), points.end(), [](const Point2D& lhs, const Point2D& rhs) {
        if (almostEqual(lhs.x, rhs.x)) {
            return lhs.y < rhs.y;
        }
        return lhs.x < rhs.x;
    });
    points.erase(std::unique(points.begin(), points.end(), pointsEqual), points.end());
    return points;
}

std::vector<Point2D> computeConvexHull(const std::vector<Point2D>& input) {
    if (input.size() <= 1) {
        return input;
    }

    std::vector<Point2D> points = makeSortedUnique(input);
    if (points.size() <= 2) {
        return points;
    }

    std::vector<Point2D> lower;
    std::vector<Point2D> upper;
    lower.reserve(points.size());
    upper.reserve(points.size());

    for (const auto& p : points) {
        while (lower.size() >= 2 && cross(lower[lower.size() - 2], lower.back(), p) <= 0.0) {
            lower.pop_back();
        }
        lower.push_back(p);
    }

    for (auto it = points.rbegin(); it != points.rend(); ++it) {
        const auto& p = *it;
        while (upper.size() >= 2 && cross(upper[upper.size() - 2], upper.back(), p) <= 0.0) {
            upper.pop_back();
        }
        upper.push_back(p);
    }

    lower.pop_back();
    upper.pop_back();

    std::vector<Point2D> hull = lower;
    hull.insert(hull.end(), upper.begin(), upper.end());
    return hull;
}

struct EdgeKey {
    std::size_t u;
    std::size_t v;

    bool operator==(const EdgeKey& other) const noexcept {
        return u == other.u && v == other.v;
    }
};

struct EdgeKeyHash {
    std::size_t operator()(const EdgeKey& key) const noexcept {
        return (key.u * 73856093u) ^ (key.v * 19349663u);
    }
};

struct DirectedEdge {
    std::size_t from;
    std::size_t to;

    bool operator==(const DirectedEdge& other) const noexcept {
        return from == other.from && to == other.to;
    }
};

struct DirectedEdgeHash {
    std::size_t operator()(const DirectedEdge& edge) const noexcept {
        return (edge.from * 2654435761u) ^ (edge.to * 19349663u);
    }
};

struct Triangle {
    std::size_t a;
    std::size_t b;
    std::size_t c;
    double circumX{0.0};
    double circumY{0.0};
    double circumRadiusSq{std::numeric_limits<double>::infinity()};
};

bool computeCircumcircle(const std::vector<Point2D>& points, Triangle& tri) {
    const Point2D& pa = points[tri.a];
    const Point2D& pb = points[tri.b];
    const Point2D& pc = points[tri.c];

    double d = 2.0 * (pa.x * (pb.y - pc.y) + pb.x * (pc.y - pa.y) + pc.x * (pa.y - pb.y));
    if (std::abs(d) < kEpsilon) {
        tri.circumRadiusSq = std::numeric_limits<double>::infinity();
        return false;
    }

    double ax2ay2 = pa.x * pa.x + pa.y * pa.y;
    double bx2by2 = pb.x * pb.x + pb.y * pb.y;
    double cx2cy2 = pc.x * pc.x + pc.y * pc.y;

    tri.circumX = (ax2ay2 * (pb.y - pc.y) + bx2by2 * (pc.y - pa.y) + cx2cy2 * (pa.y - pb.y)) / d;
    tri.circumY = (ax2ay2 * (pc.x - pb.x) + bx2by2 * (pa.x - pc.x) + cx2cy2 * (pb.x - pa.x)) / d;

    Point2D center{tri.circumX, tri.circumY};
    tri.circumRadiusSq = squaredDistance(center, pa);
    return std::isfinite(tri.circumRadiusSq);
}

std::vector<Triangle> delaunayTriangulation(const std::vector<Point2D>& input) {
    const std::size_t n = input.size();
    if (n < 3) {
        return {};
    }

    double minX = input[0].x;
    double minY = input[0].y;
    double maxX = input[0].x;
    double maxY = input[0].y;
    for (const auto& p : input) {
        minX = std::min(minX, p.x);
        minY = std::min(minY, p.y);
        maxX = std::max(maxX, p.x);
        maxY = std::max(maxY, p.y);
    }

    double dx = maxX - minX;
    double dy = maxY - minY;
    double delta = std::max(dx, dy);
    if (delta < 1e-6) {
        delta = 1.0;
    }

    double midX = (minX + maxX) * 0.5;
    double midY = (minY + maxY) * 0.5;

    std::vector<Point2D> points = input;
    points.push_back(Point2D{midX - 20.0 * delta, midY - delta});
    points.push_back(Point2D{midX, midY + 20.0 * delta});
    points.push_back(Point2D{midX + 20.0 * delta, midY - delta});

    const std::size_t superA = n;
    const std::size_t superB = n + 1;
    const std::size_t superC = n + 2;

    std::vector<Triangle> triangles;
    triangles.reserve(n * 2);

    Triangle super{superA, superB, superC};
    computeCircumcircle(points, super);
    triangles.push_back(super);

    for (std::size_t i = 0; i < n; ++i) {
        const Point2D& point = points[i];

        std::vector<std::pair<std::size_t, std::size_t>> polygon;
        polygon.reserve(triangles.size() * 3);
        std::vector<Triangle> remaining;
        remaining.reserve(triangles.size());

        for (const auto& tri : triangles) {
            Point2D center{tri.circumX, tri.circumY};
            double distSq = squaredDistance(center, point);
            if (distSq <= tri.circumRadiusSq + kEpsilon) {
                polygon.emplace_back(tri.a, tri.b);
                polygon.emplace_back(tri.b, tri.c);
                polygon.emplace_back(tri.c, tri.a);
            } else {
                remaining.push_back(tri);
            }
        }

        triangles.swap(remaining);

        std::unordered_map<EdgeKey, std::pair<std::size_t, std::size_t>, EdgeKeyHash> edgeMap;
        for (const auto& edge : polygon) {
            EdgeKey key{std::min(edge.first, edge.second), std::max(edge.first, edge.second)};
            auto it = edgeMap.find(key);
            if (it == edgeMap.end()) {
                edgeMap.emplace(key, edge);
            } else {
                edgeMap.erase(it);
            }
        }

        for (const auto& entry : edgeMap) {
            const auto& oriented = entry.second;
            Triangle tri{oriented.first, oriented.second, i};
            if (computeCircumcircle(points, tri)) {
                triangles.push_back(tri);
            }
        }
    }

    std::vector<Triangle> filtered;
    filtered.reserve(triangles.size());
    for (const auto& tri : triangles) {
        if (tri.a < n && tri.b < n && tri.c < n) {
            filtered.push_back(tri);
        }
    }
    return filtered;
}

double polygonArea(const std::vector<Point2D>& polygon) {
    if (polygon.size() < 3) {
        return 0.0;
    }

    double area = 0.0;
    const std::size_t count = polygon.size();
    for (std::size_t i = 0; i < count; ++i) {
        const Point2D& a = polygon[i];
        const Point2D& b = polygon[(i + 1) % count];
        area += a.x * b.y - b.x * a.y;
    }
    return 0.5 * area;
}

std::vector<Point2D> extractAlphaShape(const std::vector<Point2D>& points,
                                       const std::vector<Triangle>& triangles,
                                       double alpha) {
    if (triangles.empty()) {
        return {};
    }

    double radiusThreshold = std::numeric_limits<double>::infinity();
    if (alpha > 0.0) {
        radiusThreshold = 1.0 / alpha;
    }
    double radiusSqThreshold = radiusThreshold * radiusThreshold;
    const double twoPi = 2.0 * std::acos(-1.0);

    struct OrientedEdge {
        std::size_t from;
        std::size_t to;
    };

    std::unordered_map<EdgeKey, OrientedEdge, EdgeKeyHash> boundaryEdges;

    auto toggleEdge = [&](std::size_t a, std::size_t b) {
        EdgeKey key{std::min(a, b), std::max(a, b)};
        auto it = boundaryEdges.find(key);
        if (it == boundaryEdges.end()) {
            boundaryEdges.emplace(key, OrientedEdge{a, b});
        } else {
            boundaryEdges.erase(it);
        }
    };

    for (const auto& tri : triangles) {
        if (tri.circumRadiusSq <= radiusSqThreshold + kEpsilon) {
            toggleEdge(tri.a, tri.b);
            toggleEdge(tri.b, tri.c);
            toggleEdge(tri.c, tri.a);
        }
    }

    if (boundaryEdges.empty()) {
        return {};
    }

    std::unordered_map<std::size_t, std::vector<std::size_t>> adjacency;
    adjacency.reserve(boundaryEdges.size() * 2);

    std::vector<DirectedEdge> edgeList;
    edgeList.reserve(boundaryEdges.size() * 2);

    for (const auto& entry : boundaryEdges) {
        const auto& edge = entry.second;
        adjacency[edge.from].push_back(edge.to);
        adjacency[edge.to].push_back(edge.from);
        edgeList.push_back(DirectedEdge{edge.from, edge.to});
        edgeList.push_back(DirectedEdge{edge.to, edge.from});
    }

    auto eraseEdge = [](std::unordered_set<DirectedEdge, DirectedEdgeHash>& container,
                        std::size_t from, std::size_t to) {
        DirectedEdge forward{from, to};
        auto it = container.find(forward);
        if (it != container.end()) {
            container.erase(it);
        }
        DirectedEdge reverse{to, from};
        it = container.find(reverse);
        if (it != container.end()) {
            container.erase(it);
        }
    };

    auto findNext = [&](std::size_t prev, std::size_t current,
                        const std::unordered_set<DirectedEdge, DirectedEdgeHash>& available,
                        std::size_t start) -> std::size_t {
        auto it = adjacency.find(current);
        if (it == adjacency.end()) {
            return static_cast<std::size_t>(-1);
        }

        const auto& neighbors = it->second;
        const Point2D& currentPt = points[current];
        const Point2D& prevPt = points[prev];
        double vxPrev = prevPt.x - currentPt.x;
        double vyPrev = prevPt.y - currentPt.y;

        double bestAngle = -4.0;
        std::size_t bestNeighbor = static_cast<std::size_t>(-1);

        for (std::size_t neighbor : neighbors) {
            if (neighbor == current) {
                continue;
            }
            DirectedEdge edge{current, neighbor};
            if (available.find(edge) == available.end()) {
                continue;
            }
            if (neighbor == prev) {
                continue;
            }

            const Point2D& nextPt = points[neighbor];
            double vxNext = nextPt.x - currentPt.x;
            double vyNext = nextPt.y - currentPt.y;
            double angle = std::atan2(cross2D(vxPrev, vyPrev, vxNext, vyNext),
                                      dot2D(vxPrev, vyPrev, vxNext, vyNext));

            if (angle <= 0.0) {
                angle += twoPi;
            }

            if (angle > bestAngle + kEpsilon) {
                bestAngle = angle;
                bestNeighbor = neighbor;
            }
        }

        if (bestNeighbor != static_cast<std::size_t>(-1)) {
            return bestNeighbor;
        }

        DirectedEdge backEdge{current, prev};
        if (available.find(backEdge) != available.end()) {
            return prev;
        }

        DirectedEdge closeEdge{current, start};
        if (available.find(closeEdge) != available.end()) {
            return start;
        }

        return static_cast<std::size_t>(-1);
    };

    std::unordered_set<DirectedEdge, DirectedEdgeHash> remaining;
    remaining.reserve(edgeList.size());
    for (const auto& edge : edgeList) {
        remaining.insert(edge);
    }

    std::vector<Point2D> bestPolygon;
    double bestArea = 0.0;

    for (const auto& startEdge : edgeList) {
        if (remaining.find(startEdge) == remaining.end()) {
            continue;
        }

        std::vector<std::size_t> indices;
        indices.reserve(points.size());
        indices.push_back(startEdge.from);
        indices.push_back(startEdge.to);

        eraseEdge(remaining, startEdge.from, startEdge.to);

        std::size_t startVertex = startEdge.from;
        std::size_t prev = startEdge.from;
        std::size_t current = startEdge.to;

        bool closed = false;
        while (true) {
            std::size_t next = findNext(prev, current, remaining, startVertex);
            if (next == static_cast<std::size_t>(-1)) {
                break;
            }

            if (next == startVertex) {
                eraseEdge(remaining, current, startVertex);
                closed = true;
                break;
            }

            if (indices.size() > points.size() + 5) {
                break;
            }

            indices.push_back(next);
            eraseEdge(remaining, current, next);
            prev = current;
            current = next;
        }

        if (!closed || indices.size() < 3) {
            continue;
        }

        std::vector<Point2D> polygon;
        polygon.reserve(indices.size());
        for (std::size_t idx : indices) {
            polygon.push_back(points[idx]);
        }

        double area = polygonArea(polygon);
        double absArea = std::abs(area);
        if (absArea > bestArea + kEpsilon) {
            bestArea = absArea;
            if (area < 0.0) {
                std::reverse(polygon.begin(), polygon.end());
            }
            bestPolygon = std::move(polygon);
        }
    }

    if (bestPolygon.size() < 3) {
        return {};
    }

    return bestPolygon;
}

}  // namespace

GgEnvelope::GgEnvelope(double alpha) {
    setAlpha(alpha);
}

void GgEnvelope::addPoint(const Point2D& point) {
    points_.push_back(point);
}

void GgEnvelope::addPoint(double gx, double gy) {
    points_.push_back(Point2D{gx, gy});
}

void GgEnvelope::clear() {
    points_.clear();
}

void GgEnvelope::setAlpha(double alpha) {
    if (!std::isfinite(alpha) || alpha < 0.0) {
        alpha_ = 0.0;
    } else {
        alpha_ = alpha;
    }
}

std::vector<Point2D> GgEnvelope::calculateEnvelope() const {
    if (points_.size() <= 1) {
        return points_;
    }

    std::vector<Point2D> points = makeSortedUnique(points_);
    if (points.size() <= 2) {
        return points;
    }

    if (alpha_ <= 0.0) {
        return computeConvexHull(points);
    }

    std::vector<Triangle> triangles = delaunayTriangulation(points);
    std::vector<Point2D> alphaShape = extractAlphaShape(points, triangles, alpha_);
    if (alphaShape.empty()) {
        return computeConvexHull(points);
    }
    return alphaShape;
}

}  // namespace gg
