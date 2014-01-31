#ifndef __CONMAN_BLOCKS_VECTOR_SUM_H
#define __CONMAN_BLOCKS_VECTOR_SUM_H

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <Eigen/Dense>

#include <conman/hook.h>

namespace conman_blocks {
  class VectorSum : public RTT::TaskContext
  {
    // RTT properties
    int dim_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> addends_in_;
    RTT::OutputPort<Eigen::VectorXd> sum_out_;

  public:
    VectorSum(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:

    // Working variables
    Eigen::VectorXd sum_;

    // Conman interface
    boost::shared_ptr<conman::Hook> conman_hook_;
  };
}


#endif // ifndef __CONMAN_BLOCKS_VECTOR_SUM_H
