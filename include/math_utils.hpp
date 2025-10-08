#pragma once

#include <cmath>
#include <ostream>

namespace gg {

constexpr double kGravity = 9.80665;  // m/s^2 standard gravity

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3& operator+=(const Vec3& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vec3& operator*=(double s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Vec3& operator/=(double s) {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }
};

inline Vec3 operator+(Vec3 lhs, const Vec3& rhs) {
    lhs += rhs;
    return lhs;
}

inline Vec3 operator-(Vec3 lhs, const Vec3& rhs) {
    lhs -= rhs;
    return lhs;
}

inline Vec3 operator-(const Vec3& v) {
    return Vec3{-v.x, -v.y, -v.z};
}

inline Vec3 operator*(Vec3 v, double s) {
    v *= s;
    return v;
}

inline Vec3 operator*(double s, Vec3 v) {
    v *= s;
    return v;
}

inline Vec3 operator/(Vec3 v, double s) {
    v /= s;
    return v;
}

inline double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

inline double squaredNorm(const Vec3& v) {
    return dot(v, v);
}

inline double norm(const Vec3& v) {
    return std::sqrt(squaredNorm(v));
}

inline Vec3 normalized(const Vec3& v) {
    double n = norm(v);
    if (n == 0.0) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return v / n;
}

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Quaternion() = default;
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quaternion identity() {
        return Quaternion{};
    }

    double squaredNorm() const {
        return w * w + x * x + y * y + z * z;
    }

    double norm() const {
        return std::sqrt(squaredNorm());
    }

    void normalize() {
        double n = norm();
        if (n == 0.0) {
            w = 1.0;
            x = y = z = 0.0;
            return;
        }
        w /= n;
        x /= n;
        y /= n;
        z /= n;
    }

    Quaternion normalized() const {
        Quaternion q = *this;
        q.normalize();
        return q;
    }

    Quaternion conjugate() const {
        return Quaternion{w, -x, -y, -z};
    }

    Vec3 rotate(const Vec3& v) const {
        Quaternion qv{0.0, v.x, v.y, v.z};
        Quaternion result = (*this) * qv * this->conjugate();
        return Vec3{result.x, result.y, result.z};
    }

    Vec3 rotateInverse(const Vec3& v) const {
        return conjugate().rotate(v);
    }

    Quaternion operator*(const Quaternion& rhs) const {
        return Quaternion{
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
        };
    }

    static Quaternion fromAxisAngle(const Vec3& axis, double angle) {
        Vec3 unit = gg::normalized(axis);
        double half = angle * 0.5;
        double s = std::sin(half);
        return Quaternion{std::cos(half), unit.x * s, unit.y * s, unit.z * s};
    }

    static Quaternion fromAngularVelocity(const Vec3& omega, double dt) {
        Vec3 theta = omega * dt;
        double angle = gg::norm(theta);
        if (angle < 1e-12) {
            return Quaternion{1.0, theta.x * 0.5, theta.y * 0.5, theta.z * 0.5};
        }
        return fromAxisAngle(theta, angle);
    }

    static Quaternion fromEuler(double roll, double pitch, double yaw) {
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }
};

inline std::ostream& operator<<(std::ostream& os, const Vec3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

}  // namespace gg
