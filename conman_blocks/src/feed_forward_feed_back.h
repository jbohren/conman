#ifndef __CONMAN_BLOCKS_FEED_FORWARD_FEED_BACK_H
#define __CONMAN_BLOCKS_FEED_FORWARD_FEED_BACK_H

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <Eigen/Dense>

#include <conman/hook.h>

#include <std_msgs/Empty.h>
#include <rtt_rosclock/rtt_rosclock.h>

namespace conman_blocks {
  class FeedForwardFeedBack : public RTT::TaskContext
  {
    // RTT properties
    int dim_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> feedforward_in_;
    RTT::InputPort<Eigen::VectorXd> feedback_in_;
    RTT::InputPort<int> heartbeats_in_;
    RTT::InputPort<std_msgs::Empty> heartbeats_ros_in_;
    RTT::OutputPort<Eigen::VectorXd> sum_out_;

  public:
    FeedForwardFeedBack(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:

    // Working variables
    Eigen::VectorXd 
      sum_,
      feedback_effort_;

    bool require_heartbeat_;
    int heartbeat_;
    std_msgs::Empty heartbeat_ros_;
    double heartbeat_max_period_;
    ros::Time last_heartbeat_time_;
    bool heartbeat_warning_;
    bool enable_feedback_;

    // Conman interface
    boost::shared_ptr<conman::Hook> conman_hook_;
  };
}


#endif // ifndef __CONMAN_BLOCKS_FEED_FORWARD_FEED_BACK_H
