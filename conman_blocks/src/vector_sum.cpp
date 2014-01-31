
#include <Eigen/Dense>

#include <conman/hook.h>
#include <rtt_rosparam/rosparam.h>

#include "vector_sum.h"

using namespace conman_blocks;

VectorSum::VectorSum(std::string const& name) :
  TaskContext(name)
  ,dim_(0)
  ,sum_()
  // Haha... dim sum.
  ,addends_in_("addends_in",RTT::ConnPolicy::buffer(17))
{
  // Declare properties
  this->addProperty("dim",dim_)
    .doc("The gravity vector in the root link frame.");

  // Configure data ports
  this->ports()->addPort("addends_in", addends_in_);
  this->ports()->addPort("sum_out", sum_out_);

  // Load Conman interface
  conman_hook_ = conman::Hook::GetHook(this);
  conman_hook_->setInputExclusivity("addends_in", conman::Exclusivity::UNRESTRICTED);
}

bool VectorSum::configureHook()
{
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
    this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  rosparam->getComponentPrivate("dim");

  sum_.resize(dim_);
  return true;
}

bool VectorSum::startHook()
{
  return true;
}

void VectorSum::updateHook()
{
  // Reset the accumulator
  sum_.setZero();

  // Get the addends
  bool has_new_data = false;
  Eigen::VectorXd addend;
  while(addends_in_.read( addend, false ) == RTT::NewData) {
    if(addend.size() == dim_) {
      sum_ += addend;
      has_new_data = true;
    } else {
      RTT::log(RTT::Error) << "Input to VectorSum component does not have the correct dimension. All inputs should have dimension "<<dim_<<" but this input had dimension " << addend.size();
      this->error();
    }
  }

  // Write the sum
  if(has_new_data) {
    sum_out_.write( sum_ );
  }
}

void VectorSum::stopHook()
{
}

void VectorSum::cleanupHook()
{
}
