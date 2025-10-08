#pragma once

#include "data_types.hpp"

#include <cmath>
#include <vector>

namespace gg {

class GgEnvelope {
public:
    explicit GgEnvelope(double alpha = 0.0) { setAlpha(alpha); }

    void addPoint(const Point2D& point);
    void addPoint(double gx, double gy);
    void clear();

    void setAlpha(double alpha) {
        if (!std::isfinite(alpha) || alpha < 0.0) {
            alpha_ = 0.0;
        } else {
            alpha_ = alpha;
        }
    }
    double alpha() const { return alpha_; }

    std::vector<Point2D> calculateEnvelope() const;

    const std::vector<Point2D>& rawPoints() const { return points_; }

private:
    std::vector<Point2D> points_;
    double alpha_{0.0};
};

}  // namespace gg
