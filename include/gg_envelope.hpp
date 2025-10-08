#pragma once

#include "data_types.hpp"

#include <vector>

namespace gg {

class GgEnvelope {
public:
    void addPoint(const Point2D& point);
    void addPoint(double gx, double gy);
    void clear();

    std::vector<Point2D> calculateEnvelope() const;

    const std::vector<Point2D>& rawPoints() const { return points_; }

private:
    std::vector<Point2D> points_;
};

}  // namespace gg

