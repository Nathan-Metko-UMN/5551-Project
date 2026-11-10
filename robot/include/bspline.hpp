// wire_spline.hpp — fit a 3D B-spline to your points, then query ring poses
// along it for a robot arm (on a driving base) to follow.
//
// Requires Eigen 3.3+ and C++17.
//
//   wire::BSpline     Knots + control points + degree. fit() builds one from points.
//   wire::Wire        Arc-length parameterized view of a BSpline with ring frames:
//                     pose(s), tangent(s), curvature(s), project(p).
//   wire::flangeTargetInBase()
//                     Converts a ring pose on the (world-fixed) wire into the
//                     flange target in the moving robot base frame.
//
// Conventions
//   - Units are whatever your points are in (mm assumed in comments).
//   - The wire is fixed in the WORLD frame. The robot base moves.
//   - Ring frame at arc length s:  z = wire tangent (ring axis, direction of travel)
//                                  x = ring "up" (e.g. where the handle/stem is)
//                                  y = z × x
//
// Typical use
//   wire::Points pts = ...;                       // your points, one per row
//   wire::FitOptions opt; opt.smoothing = 0.01;
//   wire::Wire w(wire::BSpline::fit(pts, opt));
//   for (double s = 0; s <= w.length(); s += 2.0) {
//       Eigen::Isometry3d T = wire::flangeTargetInBase(w, s, T_world_base, T_flange_ring);
//       // -> IK
//   }
#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace wire
{

using Vec3 = Eigen::Vector3d;
using Points = Eigen::Matrix<double, Eigen::Dynamic, 3>;  // one point per row

inline Points toPoints(const std::vector<Vec3>& v)
{
    Points P(static_cast<Eigen::Index>(v.size()), 3);
    for (std::size_t i = 0; i < v.size(); ++i)
        P.row(static_cast<Eigen::Index>(i)) = v[i].transpose();
    return P;
}

struct FitOptions
{
    int degree = 3;
    // Number of control points. 0 = interpolate (curve passes through every point).
    // Fewer than the number of points = least-squares approximation.
    int numCtrl = 0;
    // > 0 penalizes bending (2nd differences of control points). Scale-free:
    // ~1e-3 barely changes anything, ~1 is very smooth. Not used when interpolating
    // exactly unless numCtrl < number of points.
    double smoothing = 0.0;
    // Point parameterization: 1.0 = chord length, 0.5 = centripetal.
    // Centripetal behaves better when your points are unevenly spaced.
    double paramExponent = 0.5;
    // Curve starts and ends exactly on your first and last point.
    bool pinEnds = true;
};

// ---------------------------------------------------------------------------
// BSpline
// Invariant: knots.size() == ctrl.rows() + degree + 1
// ---------------------------------------------------------------------------
class BSpline
{
public:
    static constexpr int kMaxDegree = 7;

    BSpline() = default;
    BSpline(int degree, Eigen::VectorXd knots, Points ctrl)
        : degree_(degree),
          knots_(std::move(knots)),
          ctrl_(std::move(ctrl))
    {
        if (degree_ < 0 || degree_ > kMaxDegree)
            throw std::invalid_argument("BSpline: degree must be in [0, 7]");
        if (ctrl_.rows() < degree_ + 1)
            throw std::invalid_argument("BSpline: need at least degree+1 control points");
        if (knots_.size() != ctrl_.rows() + degree_ + 1)
            throw std::invalid_argument("BSpline: need knots.size() == ctrl.rows() + degree + 1");
        if (!std::is_sorted(knots_.data(), knots_.data() + knots_.size()))
            throw std::invalid_argument("BSpline: knots must be non-decreasing");
        if (!(uMax() > uMin())) throw std::invalid_argument("BSpline: empty parameter range");
    }

    // ---- Fitting -----------------------------------------------------------
    static BSpline fit(const Points& raw, const FitOptions& opt = {})
    {
        const int p = opt.degree;
        if (p < 1 || p > kMaxDegree) throw std::invalid_argument("fit: degree must be in [1, 7]");

        // Drop consecutive duplicate points (they break the parameterization).
        std::vector<Eigen::Index> keep{0};
        for (Eigen::Index i = 1; i < raw.rows(); ++i)
            if ((raw.row(i) - raw.row(keep.back())).norm() > 1e-9) keep.push_back(i);
        const int M = static_cast<int>(keep.size());
        Points Q(M, 3);
        for (int i = 0; i < M; ++i) Q.row(i) = raw.row(keep[static_cast<std::size_t>(i)]);

        const int nc = opt.numCtrl > 0 ? opt.numCtrl : M;
        if (M < p + 1) throw std::invalid_argument("fit: need at least degree+1 distinct points");
        if (nc < p + 1) throw std::invalid_argument("fit: numCtrl must be >= degree+1");
        if (nc > M)
            throw std::invalid_argument("fit: numCtrl must be <= number of distinct points");

        // Parameter value for each point.
        Eigen::VectorXd ub(M);
        ub[0] = 0.0;
        for (int i = 1; i < M; ++i)
            ub[i] = ub[i - 1] + std::pow((Q.row(i) - Q.row(i - 1)).norm(), opt.paramExponent);
        ub /= ub[M - 1];

        // Clamped knot vector (The NURBS Book eq. 9.8 / 9.68-9.69).
        Eigen::VectorXd U(nc + p + 1);
        U.head(p + 1).setZero();
        U.tail(p + 1).setOnes();
        if (nc == M)
        {
            for (int j = 1; j <= M - p - 1; ++j) U[j + p] = ub.segment(j, p).mean();
        }
        else
        {
            const double d = double(M) / double(nc - p);
            for (int j = 1; j <= nc - p - 1; ++j)
            {
                const int i = static_cast<int>(j * d);
                const double a = j * d - i;
                U[p + j] = (1.0 - a) * ub[i - 1] + a * ub[i];
            }
        }

        // Collocation matrix N(r, c) = basis function c evaluated at ub[r].
        Eigen::MatrixXd N = Eigen::MatrixXd::Zero(M, nc);
        double Nb[kMaxDegree + 1];
        for (int r = 0; r < M; ++r)
        {
            const int span = findSpan(U, p, nc, ub[r]);
            basisFuns(U, p, span, ub[r], Nb);
            for (int j = 0; j <= p; ++j) N(r, span - p + j) = Nb[j];
        }

        // Stack [N; w*D] so least squares minimizes fit error + bending penalty.
        const int nd = (opt.smoothing > 0.0 && nc >= 3) ? nc - 2 : 0;
        Eigen::MatrixXd B(M + nd, nc);
        Points R = Points::Zero(M + nd, 3);
        B.topRows(M) = N;
        R.topRows(M) = Q;
        if (nd > 0)
        {
            Eigen::MatrixXd D = Eigen::MatrixXd::Zero(nd, nc);
            for (int i = 0; i < nd; ++i)
            {
                D(i, i) = 1.0;
                D(i, i + 1) = -2.0;
                D(i, i + 2) = 1.0;
            }
            const double w = std::sqrt(opt.smoothing * N.squaredNorm() / D.squaredNorm());
            B.bottomRows(nd) = w * D;
        }

        Points P(nc, 3);
        if (opt.pinEnds)
        {
            P.row(0) = Q.row(0);
            P.row(nc - 1) = Q.row(M - 1);
            R -= B.col(0) * Q.row(0) + B.col(nc - 1) * Q.row(M - 1);
            if (nc > 2)
                P.middleRows(1, nc - 2) = B.middleCols(1, nc - 2).colPivHouseholderQr().solve(R);
        }
        else
        {
            P = B.colPivHouseholderQr().solve(R);
        }
        return BSpline(p, std::move(U), std::move(P));
    }

    // ---- Access ------------------------------------------------------------
    int degree() const { return degree_; }
    const Eigen::VectorXd& knots() const { return knots_; }
    const Points& ctrl() const { return ctrl_; }
    double uMin() const { return knots_[degree_]; }
    double uMax() const { return knots_[ctrl_.rows()]; }

    // ---- Evaluation --------------------------------------------------------
    // Point at parameter u (clamped to [uMin, uMax]).
    Vec3 eval(double u) const
    {
        u = std::clamp(u, uMin(), uMax());
        const int n = static_cast<int>(ctrl_.rows());
        const int span = findSpan(knots_, degree_, n, u);
        double N[kMaxDegree + 1];
        basisFuns(knots_, degree_, span, u, N);
        Vec3 out = Vec3::Zero();
        for (int j = 0; j <= degree_; ++j) out += N[j] * ctrl_.row(span - degree_ + j).transpose();
        return out;
    }
    Vec3 operator()(double u) const { return eval(u); }

    // dC/du as its own B-spline of degree-1.
    BSpline derivative() const
    {
        if (degree_ == 0)
        {
            Eigen::VectorXd t(2);
            t << uMin(), uMax();
            return BSpline(0, t, Points::Zero(1, 3));
        }
        const int p = degree_;
        const Eigen::Index n = ctrl_.rows();
        Points q(n - 1, 3);
        for (Eigen::Index i = 0; i + 1 < n; ++i)
        {
            const double dt = knots_[i + p + 1] - knots_[i + 1];
            q.row(i) = (dt > 0.0) ? Eigen::RowVector3d((ctrl_.row(i + 1) - ctrl_.row(i)) * (p / dt))
                                  : Eigen::RowVector3d::Zero();
        }
        return BSpline(p - 1, knots_.segment(1, knots_.size() - 2), std::move(q));
    }

    // ---- I/O (plain text) --------------------------------------------------
    //   degree \n num_knots \n t0 t1 ... \n num_ctrl \n x y z (per line)
    void save(const std::string& path) const
    {
        std::ofstream f(path);
        if (!f) throw std::runtime_error("BSpline::save: cannot open " + path);
        f << std::setprecision(17) << degree_ << '\n' << knots_.size() << '\n';
        for (Eigen::Index i = 0; i < knots_.size(); ++i) f << knots_[i] << ' ';
        f << '\n' << ctrl_.rows() << '\n';
        for (Eigen::Index i = 0; i < ctrl_.rows(); ++i)
            f << ctrl_(i, 0) << ' ' << ctrl_(i, 1) << ' ' << ctrl_(i, 2) << '\n';
    }

    static BSpline load(const std::string& path)
    {
        std::ifstream f(path);
        if (!f) throw std::runtime_error("BSpline::load: cannot open " + path);
        int k = 0;
        Eigen::Index nt = 0, nc = 0;
        f >> k >> nt;
        Eigen::VectorXd t(nt);
        for (Eigen::Index i = 0; i < nt; ++i) f >> t[i];
        f >> nc;
        Points c(nc, 3);
        for (Eigen::Index i = 0; i < nc; ++i) f >> c(i, 0) >> c(i, 1) >> c(i, 2);
        if (!f) throw std::runtime_error("BSpline::load: malformed file " + path);
        return BSpline(k, std::move(t), std::move(c));
    }

private:
    // Span i with U[i] <= u < U[i+1], restricted to [p, n-1] (n = #ctrl).
    static int findSpan(const Eigen::VectorXd& U, int p, int n, double u)
    {
        const double* t = U.data();
        if (u >= t[n])
        {
            int i = n - 1;  // last non-empty span, so u == uMax works
            while (i > p && t[i] == t[i + 1]) --i;
            return i;
        }
        if (u <= t[p])
        {
            int i = p;  // first non-empty span
            while (i < n - 1 && t[i] == t[i + 1]) ++i;
            return i;
        }
        return static_cast<int>(std::upper_bound(t + p, t + n + 1, u) - t) - 1;
    }

    // Non-zero basis functions N[0..p] on span i (The NURBS Book A2.2).
    static void basisFuns(const Eigen::VectorXd& U, int p, int i, double u, double* N)
    {
        double left[kMaxDegree + 1], right[kMaxDegree + 1];
        N[0] = 1.0;
        for (int j = 1; j <= p; ++j)
        {
            left[j] = u - U[i + 1 - j];
            right[j] = U[i + j] - u;
            double saved = 0.0;
            for (int r = 0; r < j; ++r)
            {
                const double tmp = N[r] / (right[r + 1] + left[j - r]);
                N[r] = saved + right[r + 1] * tmp;
                saved = left[j - r] * tmp;
            }
            N[j] = saved;
        }
    }

    int degree_ = 3;
    Eigen::VectorXd knots_;
    Points ctrl_;
};

// ---------------------------------------------------------------------------
// Wire: arc-length parameterized curve with ring frames.
// ---------------------------------------------------------------------------
enum class FrameMode
{
    // Ring x-axis stays as close to `up` as possible. Roll stays bounded, which
    // suits a wrist with limited roll. Breaks down only where the wire runs
    // (nearly) parallel to `up`.
    UpVector,
    // Zero twist about the wire (parallel transport). Smoothest wrist motion,
    // but the roll angle can drift over a long, winding wire.
    RotationMinimizing,
};

class Wire
{
public:
    explicit Wire(
        BSpline curve,
        FrameMode mode = FrameMode::UpVector,
        const Vec3& up = Vec3::UnitZ(),
        int samples = 2000)
        : c_(std::move(curve)),
          d1_(c_.derivative()),
          d2_(d1_.derivative())
    {
        if (samples < 2) throw std::invalid_argument("Wire: samples must be >= 2");
        const std::size_t n = static_cast<std::size_t>(samples);
        u_.resize(n);
        s_.resize(n);
        x_.resize(n);
        q_.resize(n);

        const double a = c_.uMin(), b = c_.uMax();
        du_ = (b - a) / double(samples - 1);
        for (std::size_t i = 0; i < n; ++i)
        {
            u_[i] = (i + 1 == n) ? b : a + du_ * double(i);
            x_[i] = c_(u_[i]);
            s_[i] = (i == 0) ? 0.0 : s_[i - 1] + segLength(u_[i - 1], u_[i]);
        }

        const Vec3 upN = up.normalized();
        Vec3 tPrev, rPrev;
        for (std::size_t i = 0; i < n; ++i)
        {
            const Vec3 t = tangentU(u_[i]);
            Vec3 r = (i > 0) ? transport(x_[i - 1], tPrev, rPrev, x_[i], t) : Vec3::Zero();
            if (mode == FrameMode::UpVector || i == 0)
            {
                const Vec3 ru = upN - upN.dot(t) * t;
                if (ru.norm() > 1e-3)
                    r = ru;  // normal case
                else if (i == 0)
                    r = t.unitOrthogonal();  // starts vertical
                // else: wire is parallel to up here -> keep transported frame
            }
            r = (r - r.dot(t) * t).normalized();
            q_[i] = frameQuat(t, r);
            if (i > 0 && q_[i].coeffs().dot(q_[i - 1].coeffs()) < 0.0) q_[i].coeffs() *= -1.0;
            tPrev = t;
            rPrev = r;
        }
    }

    const BSpline& curve() const { return c_; }
    double length() const { return s_.back(); }

    // ---- Arc length <-> parameter -----------------------------------------
    double sAt(double u) const
    {
        u = std::clamp(u, c_.uMin(), c_.uMax());
        const std::size_t i = indexU(u);
        return s_[i] + segLength(u_[i], u);
    }

    double uAt(double s) const
    {
        s = std::clamp(s, 0.0, length());
        std::size_t i =
            static_cast<std::size_t>(std::upper_bound(s_.begin(), s_.end(), s) - s_.begin());
        i = std::clamp<std::size_t>(i, 1, s_.size() - 1) - 1;
        const double ds = s_[i + 1] - s_[i];
        double u = (ds > 0.0) ? u_[i] + (s - s_[i]) / ds * (u_[i + 1] - u_[i]) : u_[i];
        for (int k = 0; k < 4; ++k)
        {  // Newton refine on s(u) = s
            const double err = s_[i] + segLength(u_[i], u) - s;
            const double sp = d1_(u).norm();
            if (std::abs(err) < 1e-10 || sp < 1e-12) break;
            u = std::clamp(u - err / sp, u_[i], u_[i + 1]);
        }
        return u;
    }

    // ---- Queries by arc length s (distance along the wire) ----------------
    Vec3 position(double s) const { return c_(uAt(s)); }
    Vec3 tangent(double s) const { return tangentU(uAt(s)); }

    double curvature(double s) const { return curvatureU(uAt(s)); }
    double maxCurvature() const
    {
        double k = 0.0;
        for (double u : u_) k = std::max(k, curvatureU(u));
        return k;
    }

    Eigen::Quaterniond orientation(double s) const { return orientationU(uAt(s)); }

    // Ring pose in the world frame (z = along wire, x = ring "up").
    Eigen::Isometry3d pose(double s) const
    {
        const double u = uAt(s);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = orientationU(u).toRotationMatrix();
        T.translation() = c_(u);
        return T;
    }

    // Arc length of the point on the wire closest to p.
    // While moving, pass the last s as sGuess plus a window (same units as s) so the
    // search can't jump to a different stretch of wire that passes nearby.
    double project(
        const Vec3& p,
        double sGuess = -1.0,
        double window = std::numeric_limits<double>::infinity()) const
    {
        std::size_t lo = 0, hi = s_.size() - 1;
        if (sGuess >= 0.0 && std::isfinite(window))
        {
            lo = static_cast<std::size_t>(
                std::lower_bound(s_.begin(), s_.end(), sGuess - window) - s_.begin());
            hi = static_cast<std::size_t>(
                std::upper_bound(s_.begin(), s_.end(), sGuess + window) - s_.begin());
            lo = std::min(lo, s_.size() - 1);
            hi = std::clamp<std::size_t>(hi, lo + 1, s_.size()) - 1;
        }
        std::size_t best = lo;
        double bestD = std::numeric_limits<double>::infinity();
        for (std::size_t i = lo; i <= hi; ++i)
        {
            const double d = (x_[i] - p).squaredNorm();
            if (d < bestD)
            {
                bestD = d;
                best = i;
            }
        }
        // Newton on f(u) = (C(u) - p) . C'(u) = 0, bracketed by neighbouring samples.
        const double uLo = u_[best > lo ? best - 1 : lo];
        const double uHi = u_[best < hi ? best + 1 : hi];
        double u = u_[best];
        for (int k = 0; k < 20; ++k)
        {
            const Vec3 d = c_(u) - p, c1 = d1_(u), c2 = d2_(u);
            const double f = d.dot(c1), fp = c1.squaredNorm() + d.dot(c2);
            if (fp <= 0.0) break;
            const double un = std::clamp(u - f / fp, uLo, uHi);
            if (std::abs(un - u) < 1e-13)
            {
                u = un;
                break;
            }
            u = un;
        }
        return sAt(u);
    }

private:
    Vec3 tangentU(double u) const { return d1_(u).normalized(); }

    double curvatureU(double u) const
    {
        const Vec3 a = d1_(u), b = d2_(u);
        const double sp = a.norm();
        return (sp > 1e-12) ? a.cross(b).norm() / (sp * sp * sp) : 0.0;
    }

    Eigen::Quaterniond orientationU(double u) const
    {
        const std::size_t i = indexU(u);
        const double f = std::clamp((u - u_[i]) / (u_[i + 1] - u_[i]), 0.0, 1.0);
        const Eigen::Quaterniond q = q_[i].slerp(f, q_[i + 1]);
        const Vec3 t = tangentU(u);
        Vec3 r = q * Vec3::UnitX();
        r = (r - r.dot(t) * t).normalized();  // snap z exactly onto the true tangent
        return frameQuat(t, r);
    }

    std::size_t indexU(double u) const
    {  // sample i with u_[i] <= u <= u_[i+1]
        const double f = (u - u_.front()) / du_;
        const std::size_t i = f <= 0.0 ? 0 : static_cast<std::size_t>(f);
        return std::min(i, u_.size() - 2);
    }

    // |C'| integrated over [a, b], 3-point Gauss-Legendre.
    double segLength(double a, double b) const
    {
        static const double x[3] = {-0.7745966692414834, 0.0, 0.7745966692414834};
        static const double w[3] = {5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0};
        const double h = 0.5 * (b - a), m = 0.5 * (a + b);
        double sum = 0.0;
        for (int k = 0; k < 3; ++k) sum += w[k] * d1_(m + h * x[k]).norm();
        return sum * h;
    }

    // Double-reflection rotation-minimizing frame step (Wang et al. 2008).
    static Vec3 transport(
        const Vec3& x0,
        const Vec3& t0,
        const Vec3& r0,
        const Vec3& x1,
        const Vec3& t1)
    {
        const Vec3 v1 = x1 - x0;
        const double c1 = v1.squaredNorm();
        if (c1 < 1e-18) return r0;
        const Vec3 rL = r0 - (2.0 / c1) * v1.dot(r0) * v1;
        const Vec3 tL = t0 - (2.0 / c1) * v1.dot(t0) * v1;
        const Vec3 v2 = t1 - tL;
        const double c2 = v2.squaredNorm();
        if (c2 < 1e-18) return rL;
        return rL - (2.0 / c2) * v2.dot(rL) * v2;
    }

    static Eigen::Quaterniond frameQuat(const Vec3& t, const Vec3& r)
    {
        Eigen::Matrix3d R;
        R.col(0) = r;
        R.col(1) = t.cross(r);
        R.col(2) = t;
        return Eigen::Quaterniond(R).normalized();
    }

    BSpline c_, d1_, d2_;
    double du_ = 0.0;
    std::vector<double> u_, s_;
    std::vector<Vec3> x_;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> q_;
};

// ---------------------------------------------------------------------------
// Flange target in the robot base frame for ring position s.
//   T_world_base  : current base pose (odometry / localization), world <- base
//   T_flange_ring : fixed mount of the ring on the end effector, flange <- ring
// Recompute every control cycle while driving: the wire is fixed, the base is not.
// ---------------------------------------------------------------------------
inline Eigen::Isometry3d flangeTargetInBase(
    const Wire& w,
    double s,
    const Eigen::Isometry3d& T_world_base,
    const Eigen::Isometry3d& T_flange_ring = Eigen::Isometry3d::Identity())
{
    return T_world_base.inverse() * w.pose(s) * T_flange_ring.inverse();
}

}  // namespace wire