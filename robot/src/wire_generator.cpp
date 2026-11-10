// Prints an SDF model of the wire to stdout, built from the same wire.yaml
// parameters (and the same wire::loadWire) the controller uses.
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include "wire_config.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wire_generator");

    try
    {
        const wire::Wire w = wire::loadWire(*node);
        const double radius = wire::getParam<double>(*node, "wire.radius", 0.005);
        const double seg_len = wire::getParam<double>(*node, "wire.segment_length", 0.005);

        const int n = std::max(1, static_cast<int>(std::ceil(w.length() / seg_len)));

        std::ostringstream sdf;
        sdf << "<?xml version='1.0'?>\n<sdf version='1.9'>\n<model "
               "name='wire'>\n<static>true</static>\n";
        for (int i = 0; i < n; ++i)
        {
            const wire::Vec3 a = w.position(w.length() * i / n);
            const wire::Vec3 b = w.position(w.length() * (i + 1) / n);
            const wire::Vec3 mid = 0.5 * (a + b);
            const double len = (b - a).norm();
            // Cylinders lie along their local z axis: rotate z onto the segment direction
            const Eigen::Quaterniond q =
                Eigen::Quaterniond::FromTwoVectors(wire::Vec3::UnitZ(), (b - a).normalized());

            sdf << "<link name='seg_" << i << "'>\n"
                << "<pose rotation_format='quat_xyzw'>" << mid.x() << ' ' << mid.y() << ' '
                << mid.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
                << "</pose>\n"
                << "<visual name='v'><geometry><cylinder><radius>" << radius << "</radius><length>"
                << len << "</length></cylinder></geometry>"
                << "<material><diffuse>0.8 0.4 0.1 1</diffuse><ambient>0.8 0.4 0.1 "
                   "1</ambient></material>"
                << "</visual>\n</link>\n";
        }
        sdf << "</model>\n</sdf>\n";
        std::cout << sdf.str();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to build wire: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
