// -*- mode: c++ -*-
#ifndef __OPEN_CONTROLLERS_INTERFACE_H__
#define __OPEN_CONTROLLERS_INTERFACE_H__
#include <iostream>
#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <boost/shared_ptr.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/thread.hpp>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <pr2_controller_manager/controller_manager.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>


namespace OpenControllersInterface {
  // if groovy
  //typedef std::vector<pr2_mechanism_model::Transmission>::iterator TransmissionIterator;
  // if hydro
  typedef std::vector<boost::shared_ptr<pr2_mechanism_model::Transmission> >::iterator TransmissionIterator;
  //
  typedef boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::max, boost::accumulators::tag::mean> > DoubleAccumulatorSet;
  struct Stat {
    DoubleAccumulatorSet ec_acc;
    DoubleAccumulatorSet cm_acc;
    DoubleAccumulatorSet loop_acc;
    DoubleAccumulatorSet jitter_acc;
    int overruns;
    int recent_overruns;
    int last_overrun;
    int last_severe_overrun;
    unsigned long loop_count;
    double overrun_loop_sec;
    double overrun_ec;
    double overrun_cm;
    
    // These values are set when realtime loop does not meet performace expections
    bool rt_loop_not_making_timing; 
    double halt_rt_loop_frequency;
    double rt_loop_frequency;
  };


  
  class OpenController {
  public:
    OpenController();
    virtual ~OpenController();
    virtual bool updateJoints(struct timespec*) = 0;
    virtual void finalizeHW() = 0;

    static bool initRT();
    
    void setAllowUnprogrammedP(bool val) {
      allow_unprogrammed_p = val;
    }
    
    void setStatsPublishP(bool val) {
      stats_publish_p = val;
    }
    
    void setRobotXMLFile(std::string val) {
      robot_xml_file = val;
    }

    bool setDryRun(bool _dryrun) {
      dryrunp = _dryrun;
      return _dryrun;
    }

    void start();

    int waitThreadJoin() {
      return 0;
    }

    // calling haltMotorsService
    virtual void quitRequest();

    bool resetMotorsService(std_srvs::Empty::Request &req,
          std_srvs::Empty::Response &resp);
    bool haltMotorsService(std_srvs::Empty::Request &req,
         std_srvs::Empty::Response &resp);
    bool publishTraceService(std_srvs::Empty::Request &req,
           std_srvs::Empty::Response &resp);
    
    // virtual function you need to impelement
    void initialize();
    void startMain();
    void finalize();

    void diagnosticLoop();

    int setupPidFile ();
    void cleanupPidFile ();
    void parseArguments(int argc, char** argv);
    void Usage(std::string msg = "");
  protected:
    int lock_fd(int fd);
    virtual void initializeROS(ros::NodeHandle& node) = 0;
    virtual void initializeHW() = 0;
    
    void publishDiagnostics();
    double now();
    void timespecInc(struct timespec &tick, int nsec);
    double publishJitter(double start);

    std::string piddir;
    std::string pidfile;
    bool dryrunp;
    bool not_sleep_clock;
    bool allow_unprogrammed_p;
    bool stats_publish_p;
    bool initialized_p;
    bool g_reset_motors;
    bool g_quit;
    bool g_halt_requested;
    bool g_halt_motors;
    bool g_publish_trace_requested;
    std::string robot_xml_file;
    double min_acceptable_rt_loop_frequency;
    double period;
    std::string g_robot_desc;
    pr2_hardware_interface::HardwareInterface* hw;
    boost::shared_ptr<pr2_controller_manager::ControllerManager> cm;
    realtime_tools::RealtimePublisher<std_msgs::Float64> *rtpublisher;
    realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> *publisher;
    Stat g_stats;

    ros::ServiceServer reset_service;
    ros::ServiceServer halt_service;
    ros::ServiceServer publishTrace_service;
    
  private:
  };

  class RTLoopHistory {
  public:
    RTLoopHistory(unsigned length, double default_value) :
      index_(0), 
      length_(length),
      history_(new double[length]) {
      for (unsigned i=0; i<length_; ++i) 
        history_[i] = default_value;
    }

    ~RTLoopHistory() {
      delete[] history_;
      history_ = NULL;
    }
  
    void sample(double value) {
      index_ = (index_+1) % length_;
      history_[index_] = value;
    }

    double average() const {
      double sum(0.0);
      for (unsigned i=0; i<length_; ++i) 
        sum+=history_[i];
      return sum / double(length_);
    }

  protected:
    unsigned index_;
    unsigned length_;
    double *history_;
  };

  class Finalizer {
  protected:
    OpenController* controller;
  public:
    Finalizer(OpenController* controller_): controller(controller_) { };
    virtual ~Finalizer();
  };

}
#endif
