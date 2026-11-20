#ifndef IK_SOLVER_HPP_
#define IK_SOLVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace Controller
{
/**
 * Kinematics and IK for one KDL chain.
 **/
class IkSolver
{
public:
    IkSolver() = default;
    IkSolver(const IkSolver &) = delete;
    IkSolver &operator=(const IkSolver &) = delete;
    IkSolver(IkSolver &&) = delete;
    IkSolver &operator=(IkSolver &&) = delete;

    /** Build the chain base -> tip from URDF text. On failure returns false and sets error. */
    bool init(
        const std::string &urdf,
        const std::string &base,
        const std::string &tip,
        std::string &error);

    /** Use an existing chain, e.g. one built by hand in a test. */
    bool init(const KDL::Chain &chain, std::string &error);

    bool initialized() const { return fk_ != nullptr; }
    unsigned int numJoints() const { return chain_.getNrOfJoints(); }

    /** Movable joints in chain order (base to tip). q vectors use this order. */
    const std::vector<std::string> &jointNames() const { return joint_names_; }

    /** Pose of the tip in the base frame. False on a KDL error (e.g. wrong q size). */
    bool fk(const KDL::JntArray &q, KDL::Frame &pose) const;

    /** 6 x n Jacobian of the tip origin, expressed in the base frame (rows vx vy vz wx wy wz). */
    bool jacobian(const KDL::JntArray &q, KDL::Jacobian &J) const;

    // IK methods (DLS, null-space, etc.) get added here, using fk() and jacobian().

private:
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_;
    std::vector<std::string> joint_names_;
};

}  // namespace Controller

#endif  // IK_SOLVER_HPP_