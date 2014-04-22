/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#ifndef __CONMAN_HOOK_H
#define __CONMAN_HOOK_H

#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/Logger.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <conman/conman.h>

namespace conman {

  /** \brief ServiceRequester for using the conman HookServiec in C++
   *
   * This ServiceRequester will connect to the conman::HookService of a given
   * RTT TaskContext for easier use in C++.
   *
   */
  class Hook : public RTT::ServiceRequester 
  {
  public:

    typedef boost::shared_ptr<Hook> Ptr;
    typedef boost::shared_ptr<const Hook> ConstPtr;

    Hook(RTT::TaskContext *owner) :
      RTT::ServiceRequester("conman_hook", owner),
      setDesiredMinPeriod("setDesiredMinPeriod"),
      getDesiredMinPeriod("getDesiredMinPeriod"),
      setInputExclusivity("setInputExclusivity"),
      getInputExclusivity("getInputExclusivity"),
      getTime("getTime"),
      getPeriod("getPeriod"),
      getPeriodAvg("getPeriodAvg"),
      getPeriodMin("getPeriodMin"),
      getPeriodMax("getPeriodMax"),
      getPeriodVar("getPeriodVar"),
      getDuration("getDuration"),
      getDurationAvg("getDurationAvg"),
      getDurationMin("getDurationMin"),
      getDurationMax("getDurationMax"),
      getDurationVar("getDurationVar"),
      init("init"),
      update("update")
    { 
      this->addOperationCaller(setDesiredMinPeriod);
      this->addOperationCaller(getDesiredMinPeriod);
      this->addOperationCaller(setInputExclusivity);
      this->addOperationCaller(getInputExclusivity);

      this->addOperationCaller(getTime);

      this->addOperationCaller(getPeriod);
      this->addOperationCaller(getPeriodAvg);
      this->addOperationCaller(getPeriodMin);
      this->addOperationCaller(getPeriodMax);
      this->addOperationCaller(getPeriodVar);

      this->addOperationCaller(getDuration);
      this->addOperationCaller(getDurationAvg);
      this->addOperationCaller(getDurationMin);
      this->addOperationCaller(getDurationMax);
      this->addOperationCaller(getDurationVar);

      this->addOperationCaller(init);
      this->addOperationCaller(update);
    }

    RTT::OperationCaller<bool(const RTT::Seconds)>
      setDesiredMinPeriod;
    RTT::OperationCaller<RTT::Seconds(void)>
      getDesiredMinPeriod;
    RTT::OperationCaller<bool(const std::string&, const Exclusivity::Mode)>
      setInputExclusivity;
    RTT::OperationCaller<conman::Exclusivity::Mode(const std::string&)>
      getInputExclusivity;

    RTT::OperationCaller<RTT::Seconds(void)>
      getTime;

    RTT::OperationCaller<RTT::Seconds(void)>
      getPeriod;
    RTT::OperationCaller<RTT::Seconds(void)>
      getPeriodAvg;
    RTT::OperationCaller<RTT::Seconds(void)>
      getPeriodMin;
    RTT::OperationCaller<RTT::Seconds(void)>
      getPeriodMax;
    RTT::OperationCaller<RTT::Seconds(void)>
      getPeriodVar;

    RTT::OperationCaller<RTT::Seconds(void)>
      getDuration;
    RTT::OperationCaller<RTT::Seconds(void)>
      getDurationAvg;
    RTT::OperationCaller<RTT::Seconds(void)>
      getDurationMin;
    RTT::OperationCaller<RTT::Seconds(void)>
      getDurationMax;
    RTT::OperationCaller<RTT::Seconds(void)>
      getDurationVar;

    RTT::OperationCaller<bool(const RTT::Seconds)>
      init;
    RTT::OperationCaller<bool(const RTT::Seconds)>
      update;
    
    //! Checks if an RTT task has the conman Hook RTT service
    static bool HasHook(RTT::TaskContext *task)
    {
      return task->provides()->hasService("conman_hook");
    }

    //! Get the conman Hook service and return 
    static boost::shared_ptr<Hook> GetHook(RTT::TaskContext *tc)
    {
      // Return the service
      return tc->getProvider<Hook>("conman_hook");
    }
  };

}

#endif // ifndef __CONMAN_HOOK_H
