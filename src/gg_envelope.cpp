#include "gg_envelope.hpp"

#include <algorithm>
#include <cmath>

namespace gg {

void GgEnvelope::addPoint(const Point2D& point) {
    points_.push_back(point);
}

void GgEnvelope::addPoint(double gx, double gy) {
    points_.push_back(Point2D{gx, gy});
}

void GgEnvelope::clear() {
    points_.clear();
}

namespace {

inline double cross(const Point2D& o, const Point2D& a, const Point2D& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

inline bool almostEqual(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

inline bool pointsEqual(const Point2D& lhs, const Point2D& rhs) {
    return almostEqual(lhs.x, rhs.x) && almostEqual(lhs.y, rhs.y);
}

}  // namespace

std::vector<Point2D> GgEnvelope::calculateEnvelope() const {
    if (points_.size() <= 1) {
        return points_;
    }

    std::vector<Point2D> sorted = points_;
    std::sort(sorted.begin(), sorted.end(), [](const Point2D& lhs, const Point2D& rhs) {
        if (lhs.x == rhs.x) {
            return lhs.y < rhs.y;
        }
        return lhs.x < rhs.x;
    });

    sorted.erase(std::unique(sorted.begin(), sorted.end(), pointsEqual), sorted.end());

    if (sorted.size() <= 1) {
        return sorted;
    }

    std::vector<Point2D> lower;
    std::vector<Point2D> upper;
    lower.reserve(sorted.size());
    upper.reserve(sorted.size());

    for (const auto& p : sorted) {
        while (lower.size() >= 2 && cross(lower[lower.size() - 2], lower.back(), p) <= 0.0) {
            lower.pop_back();
        }
        lower.push_back(p);
    }

    for (auto it = sorted.rbegin(); it != sorted.rend(); ++it) {
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

}  // namespace gg

