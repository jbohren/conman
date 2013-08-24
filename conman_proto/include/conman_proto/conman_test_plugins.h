#ifndef __CONMAN_TEST_PLUGINS_H
#define __CONMAN_TEST_PLUGINS_H

#include <conman_proto/conman.h>

class TestEffortController : public RTT::TaskContext
{
public:
  TestEffortController(std::string const& name);
  bool startHook();
  bool configureHook();

  void computeControlHook( RTT::os::TimeService::Seconds secs, RTT::os::TimeService::Seconds period);

private:

  RTT::InputPort<double> effort_in_;
  RTT::OutputPort<double> effort_out_;
};

#endif // ifndef __CONMAN_TEST_PLUGINS_H
