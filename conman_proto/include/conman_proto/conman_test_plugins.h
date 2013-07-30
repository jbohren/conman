#ifndef __CONMAN_TEST_PLUGINS_H
#define __CONMAN_TEST_PLUGINS_H

#include <conman_proto/conman.h>
#include <conman_proto/block.h>

class TestEffortController : public conman::Block 
{
public:
  TestEffortController(std::string const& name);
  bool startHook();
  bool configureHook();
  virtual void compute_control( RTT::os::TimeService::Seconds secs,
                                RTT::os::TimeService::Seconds period);
private:

  RTT::InputPort<double> effort_in_;
  RTT::OutputPort<double> effort_out_;
};

#endif // ifndef __CONMAN_TEST_PLUGINS_H
