// =============================================================================
//  utilize.h  --  rviz_car_sim 通用数学工具 (header-only, OOP)
//
//  设计:
//    1. 核心类 MathTool —— 纯数学运算以 static 方法提供 (无需实例化);
//       需要跨调用保存状态的部分 (如低通滤波) 以实例方法提供;
//    2. 二维向量 Vec2 作为独立结构体, 通过 MathTool::Vec2 别名访问;
//    3. header-only, 全部 inline / static constexpr, 无需链接额外库;
//    4. 不依赖 ROS, 仅依赖标准库 (<cmath> <algorithm> <iostream>);
//    5. 角度统一使用弧度 (rad), 除非方法名显式带 Deg。
//
//  用法:
//    double a = MathTool::normalizeAngle(yaw);   // 静态方法, 无需实例
//    MathTool tool;                              // 需要滤波状态时实例化
//    double s = tool.smooth(raw, 0.3);
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
    Vec2 operator/(double s) const { return Vec2(x / s, y / s); }
    Vec2 &operator+=(const Vec2 &o) { x += o.x; y += o.y; return *this; }
    Vec2 &operator-=(const Vec2 &o) { x -= o.x; y -= o.y; return *this; }
    Vec2 &operator*=(double s) { x *= s; y *= s; return *this; }

    bool operator==(const Vec2 &o) const { return x == o.x && y == o.y; }
};

/// 数学工具类: 静态纯函数 + 有状态实例方法
class MathTool
{
public:
    // ================================================================ 常量 ===
    static constexpr double PI      = 3.14159265358979323846;
    static constexpr double TWO_PI  = 2.0 * PI;
    static constexpr double EPS     = 1e-9;
    static constexpr double DEG2RAD = PI / 180.0;
    static constexpr double RAD2DEG = 180.0 / PI;

    /// 便于书写: MathTool::Vec2
    using Vec2 = ::Vec2;

    // ========================================================== 角度处理 ===

    /// 弧度 -> 角度
    static double rad2deg(double rad) { return rad * RAD2DEG; }

    /// 角度 -> 弧度
    static double deg2rad(double deg) { return deg * DEG2RAD; }

    /// 将任意角度归一化到 (-pi, pi]
    static double normalizeAngle(double angle)
    {
        // 先用 fmod 收敛到 (-2pi, 2pi], 避免大角度时 while 循环过慢
        double a = std::fmod(angle + PI, TWO_PI);
        if (a < 0.0) {
            a += TWO_PI;
        }
        return a - PI;
    }

    /// 归一化到 [0, 2pi)
    static double normalizeAnglePositive(double angle)
    {
        double a = std::fmod(angle, TWO_PI);
        if (a < 0.0) {
            a += TWO_PI;
        }
        return a;
    }

    /// 两角之间的最短夹角 (带符号, 结果位于 (-pi, pi])
    static double angleDiff(double a, double b) { return normalizeAngle(a - b); }

    /// 两角之间的最小无向夹角 (位于 [0, pi])
    static double angleDist(double a, double b) { return std::fabs(normalizeAngle(a - b)); }

    /// 判断目标角是否在 [center - half, center + half] 范围内
    static bool inAngleRange(double angle, double center, double half)
    {
        return angleDist(angle, center) <= half;
    }

    /// 角度是否落在 start->end 的圆弧区间内 (自动处理跨界, 如 -170° ~ 170°)
    static bool angleInArc(double angle, double start, double end)
    {
        const double span = normalizeAngle(end - start);
        const double rel  = normalizeAngle(angle - start);
        return (span >= 0.0) ? (rel >= 0.0 && rel <= span)
                             : (rel <= 0.0 && rel >= span);
    }

    // ========================================================== 数值基础 ===

    /// 数值限幅
    static double clamp(double v, double lo, double hi)
    {
        return (v < lo) ? lo : ((v > hi) ? hi : v);
    }

    /// 符号函数: >0 -> 1, <0 -> -1, |v|<eps -> 0
    static double sign(double v)
    {
        if (v > EPS) return 1.0;
        if (v < -EPS) return -1.0;
        return 0.0;
    }

    /// 平方
    static double sq(double x) { return x * x; }

    /// 线性插值, t 会被限幅到 [0, 1]
    static double lerp(double a, double b, double t)
    {
        return a + (b - a) * clamp(t, 0.0, 1.0);
    }

    /// 浮点近似相等
    static bool nearlyEqual(double a, double b, double eps = 1e-6)
    {
        return std::fabs(a - b) <= eps;
    }

    /// 安全除法: 分母接近 0 时返回 fallback, 避免 NaN / Inf
    static double safeDiv(double num, double den, double fallback = 0.0)
    {
        return (std::fabs(den) < EPS) ? fallback : (num / den);
    }

    /// 映射: x 从 [in_min, in_max] 线性映射到 [out_min, out_max]
    static double remap(double x, double in_min, double in_max, double out_min, double out_max)
    {
        const double span = in_max - in_min;
        if (std::fabs(span) < EPS) {
            return out_min;
        }
        return out_min + (x - in_min) * (out_max - out_min) / span;
    }

    /// 归一化到 [0, 1]
    static double normalize01(double v, double v_min, double v_max)
    {
        return clamp(remap(v, v_min, v_max, 0.0, 1.0), 0.0, 1.0);
    }

    /// 移动平均 (取末尾 window 个样本), 用于滤掉尖峰
    static double movingAverage(const double *data, std::size_t n, std::size_t window)
    {
        if (n == 0 || window == 0 || data == nullptr) {
            return 0.0;
        }
        const std::size_t w = std::min(window, n);
        double sum = 0.0;
        for (std::size_t i = n - w; i < n; ++i) {
            sum += data[i];
        }
        return sum / static_cast<double>(w);
    }

    // ========================================================== 二维几何 ===

    /// 点积
    static double dot(const Vec2 &a, const Vec2 &b) { return a.x * b.x + a.y * b.y; }

    /// 二维叉积 (标量)
    static double cross(const Vec2 &a, const Vec2 &b) { return a.x * b.y - a.y * b.x; }

    /// 向量模长
    static double norm(const Vec2 &v) { return std::sqrt(sq(v.x) + sq(v.y)); }

    /// 模长平方 (避免开方, 比较距离大小时更高效)
    static double norm2(const Vec2 &v) { return sq(v.x) + sq(v.y); }

    /// 单位化; 零向量返回 fallback (默认零向量)
    static Vec2 normalized(const Vec2 &v, const Vec2 &fallback = Vec2())
    {
        const double n = norm(v);
        if (n < EPS) {
            return fallback;
        }
        return Vec2(v.x / n, v.y / n);
    }

    /// 两点欧氏距离
    static double dist(const Vec2 &a, const Vec2 &b) { return norm(a - b); }

    /// 两点欧氏距离平方
    static double dist2(const Vec2 &a, const Vec2 &b) { return norm2(a - b); }

    /// 点到线段的最近点
    static Vec2 closestPointOnSegment(const Vec2 &p, const Vec2 &a, const Vec2 &b)
    {
        const Vec2 ab = b - a;
        const double len2 = norm2(ab);
        if (len2 < EPS) {
            return a;
        }
        double t = dot(p - a, ab) / len2;
        t = clamp(t, 0.0, 1.0);
        return a + ab * t;
    }

    /// 点到线段的最短距离
    static double distPointToSegment(const Vec2 &p, const Vec2 &a, const Vec2 &b)
    {
        return dist(p, closestPointOnSegment(p, a, b));
    }

    /// 2D 刚体变换: 局部 -> 全局 (绕原点旋转 yaw 后平移)
    static Vec2 transformLocalToGlobal(const Vec2 &local, double yaw, const Vec2 &origin)
    {
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        return Vec2(origin.x + local.x * c - local.y * s,
                    origin.y + local.x * s + local.y * c);
    }

    /// 2D 刚体变换: 全局 -> 局部
    static Vec2 transformGlobalToLocal(const Vec2 &global, double yaw, const Vec2 &origin)
    {
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        const Vec2 d = global - origin;
        return Vec2(d.x * c + d.y * s, -d.x * s + d.y * c);
    }

    // ============================================================ 纯追踪 ===

    /// 纯追踪曲率: kappa = 2 * sin(alpha) / Ld
    /// alpha: 目标相对车头的偏角(rad), Ld: 前瞻距离(m)
    static double purePursuitCurvature(double alpha, double lookahead)
    {
        return 2.0 * std::sin(alpha) / safeDiv(lookahead, 1.0, 1e6);
    }

    /// 差速车角速度: w = v * kappa
    static double diffDriveOmega(double v, double kappa) { return v * kappa; }

    /// 由目标偏角直接求差速车角速度
    static double purePursuitOmega(double v, double alpha, double lookahead)
    {
        return diffDriveOmega(v, purePursuitCurvature(alpha, lookahead));
    }

    // ================================================ 有状态实例方法 ===

    /// 默认构造 (滤波状态未初始化)
    MathTool() = default;

    /// 显式带初值构造 (预置滤波状态)
    explicit MathTool(double init_value) : prev_value_(init_value), filter_ready_(true) {}

    /// 一阶低通滤波: y = alpha * x + (1 - alpha) * y_prev
    /// alpha 越大越跟随输入, 越小越平滑。
    double smooth(double x, double alpha)
    {
        const double a = clamp(alpha, 0.0, 1.0);
        if (!filter_ready_) {
            prev_value_ = x;
            filter_ready_ = true;
            return prev_value_;
        }
        prev_value_ = a * x + (1.0 - a) * prev_value_;
        return prev_value_;
    }

    /// 一阶低通滤波 (带死区, 变化小于 deadband 时保持不变)
    double smooth(double x, double alpha, double deadband)
    {
        if (filter_ready_ && std::fabs(x - prev_value_) < deadband) {
            return prev_value_;
        }
        return smooth(x, alpha);
    }

    /// 带速率限幅的一阶低通 (限制单步最大变化量)
    double smoothWithRateLimit(double x, double alpha, double max_rate)
    {
        const double filtered = smooth(x, alpha);
        const double delta = clamp(filtered - prev_value_, -max_rate, max_rate);
        prev_value_ += delta;
        return prev_value_;
    }

    /// 当前滤波值
    double value() const { return prev_value_; }

    /// 复位滤波状态
    void reset(double v = 0.0)
    {
        prev_value_ = v;
        filter_ready_ = false;
    }

    /// 滤波是否已初始化
    bool initialized() const { return filter_ready_; }

private:
    double prev_value_{0.0};      ///< 上一次滤波输出
    bool   filter_ready_{false};  ///< 是否已有有效历史值
};

#endif  // UTILIZE_H
