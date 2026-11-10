#pragma once
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "bspline.hpp"
namespace wire
{
template <typename T, typename NodeT>
T getParam(NodeT& node, const std::string& name, const T& def)
{
    if (!node.has_parameter(name)) node.declare_parameter(name, def);
    return node.get_parameter(name).template get_value<T>();
}

template <typename NodeT>
Wire loadWire(NodeT& node)
{
    const auto flat = getParam<std::vector<double>>(node, "wire.points", {});
    if (flat.size() < 6 || flat.size() % 3 != 0)
        throw std::invalid_argument("wire.points must be x y z triples, at least 2 points");

    Points pts(static_cast<Eigen::Index>(flat.size() / 3), 3);
    for (Eigen::Index i = 0; i < pts.rows(); ++i)
        pts.row(i) << flat[3 * i], flat[3 * i + 1], flat[3 * i + 2];

    FitOptions opt;
    opt.degree = static_cast<int>(getParam<int64_t>(node, "wire.degree", 3));
    opt.numCtrl = static_cast<int>(getParam<int64_t>(node, "wire.num_ctrl", 0));
    opt.smoothing = getParam<double>(node, "wire.smoothing", 0.0);
    opt.paramExponent = getParam<double>(node, "wire.param_exponent", 0.5);
    opt.pinEnds = getParam<bool>(node, "wire.pin_ends", true);
    const int samples = static_cast<int>(getParam<int64_t>(node, "wire.samples", 2000));

    return Wire(BSpline::fit(pts, opt), FrameMode::UpVector, Vec3::UnitZ(), samples);
}
}  // namespace wire
