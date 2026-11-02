// =============================================================================
//  utilize.h  --  rviz_car_sim 通用数学工具 (header-only)
//
//  设计原则:
//    1. header-only, 全部使用 inline / constexpr, 无需链接额外库;
//    2. 不依赖 ROS, 仅依赖标准库 (<cmath> <algorithm> <iostream>);
//    3. 所有函数为纯函数 (无副作用), 便于单元测试;
//    4. 角度统一使用弧度 (rad), 除非函数名显式带 Deg。
//
//  命名空间: util
// =============================================================================
#ifndef UTILIZE_H
#define UTILIZE_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>

// ---------------------------------------------------------------- 打印宏 ---
// 注意: 输出到 std::cout 会与 ROS 日志混排, 仅建议在调试阶段使用。
#define SHOW(x)  std::cout << x;
#define SHOWE(x) std::cout << x << '\n';

// ------------------------------------------------------------------ 常量 ---
constexpr double UTIL_PI       = 3.14159265358979323846;
constexpr double UTIL_2PI      = 2.0 * UTIL_PI;
constexpr double UTIL_EPS      = 1e-9;
constexpr double UTIL_DEG2RAD  = UTIL_PI / 180.0;
constexpr double UTIL_RAD2DEG  = 180.0 / UTIL_PI;

namespace util
{

// ============================ 角度处理 ============================

/// 弧度 -> 角度
inline double rad2deg(double rad)
{
    return rad * UTIL_RAD2DEG;
}

/// 角度 -> 弧度
inline double deg2rad(double deg)
{
    return deg * UTIL_DEG2RAD;
}

/// 将任意角度归一化到 (-pi, pi]
inline double normalizeAngle(double angle)
{
    // 先用 fmod 收敛到 (-2pi, 2pi], 避免大角度时 while 循环过慢
    double a = std::fmod(angle + UTIL_PI, UTIL_2PI);
    if (a < 0.0) {
        a += UTIL_2PI;
    }
    return a - UTIL_PI;
}

/// 归一化到 [0, 2pi)
inline double normalizeAnglePositive(double angle)
{
    double a = std::fmod(angle, UTIL_2PI);
    if (a < 0.0) {
        a += UTIL_2PI;
    }
    return a;
}

/// 两角之间的最短夹角 (带符号, 结果位于 (-pi, pi])
inline double angleDiff(double a, double b)
{
    return normalizeAngle(a - b);
}

/// 两角之间的最小无向夹角 (位于 [0, pi])
inline double angleDist(double a, double b)
{
    return std::fabs(normalizeAngle(a - b));
}

/// 判断目标角是否在 [center - half, center + half] 范围内
inline bool inAngleRange(double angle, double center, double half)
{
    return angleDist(angle, center) <= half;
}

// ============================ 数值基础 ============================

/// 数值限幅
inline double clamp(double v, double lo, double hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/// 符号函数: >0 -> 1, <0 -> -1, ==0 -> 0
inline double sign(double v)
{
    if (v > UTIL_EPS) return 1.0;
    if (v < -UTIL_EPS) return -1.0;
    return 0.0;
}

/// 平方
inline double sq(double x)
{
    return x * x;
}

/// 线性插值, t 会被限幅到 [0, 1]
inline double lerp(double a, double b, double t)
{
    return a + (b - a) * clamp(t, 0.0, 1.0);
}

/// 浮点近似相等
inline bool nearlyEqual(double a, double b, double eps = 1e-6)
{
    return std::fabs(a - b) <= eps;
}

/// 安全除法: 分母接近 0 时返回 fallback, 避免 NaN / Inf
inline double safeDiv(double num, double den, double fallback = 0.0)
{
    return (std::fabs(den) < UTIL_EPS) ? fallback : (num / den);
}

/// 映射: x 从 [in_min, in_max] 线性映射到 [out_min, out_max]
inline double remap(double x, double in_min, double in_max, double out_min, double out_max)
{
    const double span = in_max - in_min;
    if (std::fabs(span) < UTIL_EPS) {
        return out_min;
    }
    return out_min + (x - in_min) * (out_max - out_min) / span;
}

/// 指数平滑滤波 (一阶低通): y = alpha * x + (1 - alpha) * y_prev
/// alpha 越大越跟随输入, 越小越平滑。
inline double smooth(double x, double &y_prev, double alpha)
{
    y_prev = clamp(alpha, 0.0, 1.0) * x + (1.0 - clamp(alpha, 0.0, 1.0)) * y_prev;
    return y_prev;
}

// ============================ 二维几何 ============================

/// 二维向量 / 点
struct Vec2
{
    double x{0.0};
    double y{0.0};

    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2 &o) const { return Vec2(x + o.x, y + o.y); }
    Vec2 operator-(const Vec2 &o) const { return Vec2(x - o.x, y - o.y); }
    Vec2 operator*(double s) const { return Vec2(x * s, y * s); }
    Vec2 &operator+=(const Vec2 &o) { x += o.x; y += o.y; return *this; }
    Vec2 &operator-=(const Vec2 &o) { x -= o.x; y -= o.y; return *this; }
};

/// 点积
inline double dot(const Vec2 &a, const Vec2 &b)
{
    return a.x * b.x + a.y * b.y;
}

/// 二维叉积 (标量)
inline double cross(const Vec2 &a, const Vec2 &b)
{
    return a.x * b.y - a.y * b.x;
}

/// 向量模长
inline double norm(const Vec2 &v)
{
    return std::sqrt(sq(v.x) + sq(v.y));
}

/// 模长平方 (避免开方, 比较距离大小时更高效)
inline double norm2(const Vec2 &v)
{
    return sq(v.x) + sq(v.y);
}

/// 单位化; 零向量返回 fallback (默认零向量)
inline Vec2 normalized(const Vec2 &v, const Vec2 &fallback = Vec2())
{
    const double n = norm(v);
    if (n < UTIL_EPS) {
        return fallback;
    }
    return Vec2(v.x / n, v.y / n);
}

/// 两点欧氏距离
inline double dist(const Vec2 &a, const Vec2 &b)
{
    return norm(a - b);
}

/// 点到线段的最近点
inline Vec2 closestPointOnSegment(const Vec2 &p, const Vec2 &a, const Vec2 &b)
{
    const Vec2 ab = b - a;
    const double len2 = norm2(ab);
    if (len2 < UTIL_EPS) {
        return a;
    }
    double t = dot(p - a, ab) / len2;
    t = clamp(t, 0.0, 1.0);
    return a + ab * t;
}

/// 点到线段的最短距离
inline double distPointToSegment(const Vec2 &p, const Vec2 &a, const Vec2 &b)
{
    return dist(p, closestPointOnSegment(p, a, b));
}

/// 2D 刚体变换: 绕原点旋转 yaw 后再平移 (局部 -> 全局)
inline Vec2 transformLocalToGlobal(const Vec2 &local, double yaw, const Vec2 &origin)
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return Vec2(origin.x + local.x * c - local.y * s,
                origin.y + local.x * s + local.y * c);
}

/// 2D 刚体变换: 全局 -> 局部
inline Vec2 transformGlobalToLocal(const Vec2 &global, double yaw, const Vec2 &origin)
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const Vec2 d = global - origin;
    return Vec2(d.x * c + d.y * s, -d.x * s + d.y * c);
}

/// 纯追踪曲率: kappa = 2 * sin(alpha) / Ld
/// alpha: 目标相对车头的偏角(rad), Ld: 前瞻距离(m)
inline double purePursuitCurvature(double alpha, double lookahead)
{
    return 2.0 * std::sin(alpha) / safeDiv(lookahead, 1.0, 1e6);
}

/// 差速车角速度: w = v * kappa
inline double diffDriveOmega(double v, double kappa)
{
    return v * kappa;
}

}  // namespace util

#endif  // UTILIZE_H
