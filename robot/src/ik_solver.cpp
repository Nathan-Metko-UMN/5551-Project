#include "ik_solver.hpp"

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace Controller
{
bool IkSolver::init(
    const std::string &urdf,
    const std::string &base,
    const std::string &tip,
    std::string &error)
{
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf, tree))
    {
        error = "Failed to parse the robot description into a KDL tree";
        return false;
    }
    KDL::Chain chain;
    if (!tree.getChain(base, tip, chain))
    {
        error = "No KDL chain from '" + base + "' to '" + tip + "'";
        return false;
    }
    return init(chain, error);
}

bool IkSolver::init(const KDL::Chain &chain, std::string &error)
{
    // Solvers hold a reference to chain_, so drop them before replacing it.
    fk_.reset();
    jac_.reset();
    joint_names_.clear();

    chain_ = chain;
    if (chain_.getNrOfJoints() == 0)
    {
        error = "KDL chain has no movable joints";
        return false;
    }

    for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
    {
        const KDL::Joint &joint = chain_.getSegment(i).getJoint();
        if (joint.getType() != KDL::Joint::Fixed) joint_names_.push_back(joint.getName());
    }

    fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    jac_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    return true;
}

bool IkSolver::fk(const KDL::JntArray &q, KDL::Frame &pose) const
{
    return fk_ && fk_->JntToCart(q, pose) >= 0;
}

bool IkSolver::jacobian(const KDL::JntArray &q, KDL::Jacobian &J) const
{
    return jac_ && jac_->JntToJac(q, J) >= 0;
}

}  // namespace Controller