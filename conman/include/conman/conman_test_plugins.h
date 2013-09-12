/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#ifndef __CONMAN_TEST_PLUGINS_H
#define __CONMAN_TEST_PLUGINS_H

#include <conman/conman.h>

class TestEffortController : public RTT::TaskContext
{
public:
  TestEffortController(std::string const& name);
  bool startHook();
  bool configureHook();
  void updateHook();

private:
  RTT::InputPort<double> effort_in_;
  RTT::OutputPort<double> effort_out_;

  // Reference to the service requester
  boost::shared_ptr<conman::Hook> conman_hook_;
};

#endif // ifndef __CONMAN_TEST_PLUGINS_H
