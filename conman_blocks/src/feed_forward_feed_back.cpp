
#include <Eigen/Dense>

#include <conman/hook.h>
#include <rtt_rosparam/rosparam.h>
#include <rtt_roscomm/rostopic.h>

#include "feed_forward_feed_back.h"

using namespace conman_blocks;

FeedForwardFeedBack::FeedForwardFeedBack(std::string const& name) :
  TaskContext(name)
  ,dim_(0)
  ,sum_()
  ,feedback_effort_()
  ,require_heartbeat_(false)
  ,heartbeat_max_period_(0.01)
  // Haha... dim sum.
  ,feedforward_in_("feedforward_in",RTT::ConnPolicy::buffer(17))
{
  // Declare properties
  this->addProperty("dim",dim_)
    .doc("The dimension of the .");
  this->addProperty("require_heartbeat", require_heartbeat_)
    .doc("If true, feedback effort will be disabled if there is no heartbeat heartbeat.");
  this->addProperty("heartbeat_max_period", heartbeat_max_period_)
    .doc("This is the maximum period between heartbeats before feedback control will be disabled.");

  // Configure data ports
  this->ports()->addPort("feedforward_in", feedforward_in_)
    .doc("The feed-forward term(s). All inputs to this port will be read at each step and added to the output term.");
  this->ports()->addPort("feedback_in", feedback_in_)
    .doc("The feed-back term. This port can only have one input each cycle.");
  this->ports()->addPort("sum_out", sum_out_)
    .doc("The sum of the feed-forward term(s) and feed-back term at each cycle.");

  this->ports()->addPort("heartbeats_in", heartbeats_in_)
    .doc("Heartbeat pulses.");
  this->ports()->addPort("heartbeats_ros_in", heartbeats_ros_in_)
    .doc("Heartbeat pulses from ROS.");

  // Get an instance of the rtt_rostopic service requester
  rtt_rostopic::ROSTopic rostopic;

  // Add the port and stream it to a ROS topic
  heartbeats_ros_in_.createStream(rostopic.connection("~/"+this->getName()+"/heartbeats"));

  // Load Conman interface
  conman_hook_ = conman::Hook::GetHook(this);
  conman_hook_->setInputExclusivity("feedforward_in", conman::Exclusivity::UNRESTRICTED);
  conman_hook_->setInputExclusivity("feedback_in", conman::Exclusivity::EXCLUSIVE);
  conman_hook_->setInputExclusivity("heartbeats_in", conman::Exclusivity::EXCLUSIVE);
}

bool FeedForwardFeedBack::configureHook()
{
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
    this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  rosparam->getComponentPrivate("dim");
  rosparam->getComponentPrivate("require_heartbeat");
  rosparam->getComponentPrivate("heartbeat_max_period");

  sum_.resize(dim_);
  feedback_effort_.resize(dim_);
  return true;
}

bool FeedForwardFeedBack::startHook()
{
  /*
   *interpolate_effort = true;
   *interpolation_scale = 0.0;
   */
  return true;
}

void FeedForwardFeedBack::updateHook()
{
  // Reset the accumulator
  sum_.setZero();
  bool has_new_data = false;

  // Get the feedforward
  Eigen::VectorXd addend;
  while(feedforward_in_.read( addend, false ) == RTT::NewData) {
    if(addend.size() == dim_) {
      sum_ += addend;
      has_new_data = true;
    } else {
      RTT::log(RTT::Error) << "Feed-forward input to FeedForwardFeedBack component does not have the correct dimension. All inputs should have dimension "<<dim_<<" but this input had dimension " << addend.size();
      this->error();
    }
  }

  // Listen for a pulse
  if(heartbeats_in_.readNewest(heartbeat_) == RTT::NewData || heartbeats_ros_in_.readNewest(heartbeat_ros_) == RTT::NewData) {
    last_heartbeat_time_ = rtt_rosclock::host_rt_now();
    heartbeat_warning_ = false;
  }

  // Check heartbeats
  if(!require_heartbeat_ || (rtt_rosclock::host_rt_now() - last_heartbeat_time_).toSec() < heartbeat_max_period_) { 
    // Get the fedback
    if(feedback_in_.readNewest( feedback_effort_, false) == RTT::NewData) {
      if(addend.size() == dim_) {
        sum_ += feedback_effort_;
        has_new_data = true;
/*
 *        if(interpolate_effort) {
 *          joint_effort = joint_effort_last + interpolation_scale * (joint_effort_raw - joint_effort_last);
 *          interpolation_scale = std::min(1.0,std::max(0.0,(interpolation_scale*interpolation_time + period)/interpolation_time));
 *
 *          if(fabs(interpolation_scale-1.0) < 1E-6) {
 *            interpolate_effort = false;
 *            interpolation_scale = 0.0;
 *          }
 *        }
 */
      } else {
        RTT::log(RTT::Error) << "Feed-back input to FeedForwardFeedBack component does not have the correct dimension. All inputs should have dimension "<<dim_<<" but this input had dimension " << addend.size();
        this->error();
      }
    }
  } else {
    if(!heartbeat_warning_) { 
      RTT::log(RTT::Warning) << "Heartbeats are not being sent often enough (should be < " << heartbeat_max_period_ << " s). Disabling feedback effort." << addend.size();
      heartbeat_warning_ = true;
    }
  }

  // Write the sum
  if(has_new_data) {
    sum_out_.write( sum_ );
  }
}

void FeedForwardFeedBack::stopHook()
{
}

void FeedForwardFeedBack::cleanupHook()
{
}
