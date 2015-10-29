/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *                2013, Ryohei Ueda, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <stdio.h>
#include <getopt.h>
#include <execinfo.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/mman.h>
#include <boost/cast.hpp>

#include <open_controllers_interface/open_controllers_interface.h>

using namespace pr2_hardware_interface;
using namespace std;
using namespace boost::accumulators;

static const int NSEC_PER_SECOND = 1e+9;
static const int USEC_PER_SECOND = 1e6;

namespace OpenControllersInterface {
  OpenController::OpenController():
    not_sleep_clock_(false) { // constructor
  }

  OpenController::~OpenController() {

  }
  bool OpenController::resetMotorsService(std_srvs::Empty::Request &req,
                                          std_srvs::Empty::Response &resp) {
    g_reset_motors_ = true;
    return true;
  }

  bool OpenController::haltMotorsService(std_srvs::Empty::Request &req,
                                         std_srvs::Empty::Response &resp) {
    quitRequest();
    return true;
  }
  
  bool OpenController::publishTraceService(std_srvs::Empty::Request &req,
                                           std_srvs::Empty::Response &resp) {
    g_publish_trace_requested_ = true;
    return true;
  }

  double timespecDiff(struct timespec* a, struct timespec* b) {
    return (a->tv_sec - b->tv_sec) + double(a->tv_nsec - b->tv_nsec) / NSEC_PER_SECOND;
  }

  void OpenController::start() {
    while(!g_quit_) {
      startMain();
      if (g_reset_motors_)
      {
        ControllerStatusPtr st = recoverController();
        if (!st || !st->isHealthy())
        {
          ROS_FATAL("oh no, no way to recover! quit!");
          g_quit_ = true;
        }
        g_reset_motors_ = false;
      }
    }
    ROS_INFO("good bye start()");
  }

  void OpenController::publishDiagnostics() {
    if (!!publisher_ && publisher_->trylock())
    {
      accumulator_set<double, stats<tag::max, tag::mean> > zero;
      vector<diagnostic_msgs::DiagnosticStatus> statuses;
      diagnostic_updater::DiagnosticStatusWrapper status;

      static double max_ec = 0, max_cm = 0, max_loop = 0, max_jitter = 0;
      double avg_ec, avg_cm, avg_loop, avg_jitter;

      avg_ec           = extract_result<tag::mean>(g_stats_.ec_acc);
      avg_cm           = extract_result<tag::mean>(g_stats_.cm_acc);
      avg_loop         = extract_result<tag::mean>(g_stats_.loop_acc);
      max_ec           = std::max(max_ec, extract_result<tag::max>(g_stats_.ec_acc));
      max_cm           = std::max(max_cm, extract_result<tag::max>(g_stats_.cm_acc));
      max_loop         = std::max(max_loop, extract_result<tag::max>(g_stats_.loop_acc));
      g_stats_.ec_acc   = zero;
      g_stats_.cm_acc   = zero;
      g_stats_.loop_acc = zero;

      // Publish average loop jitter
      avg_jitter         = extract_result<tag::mean>(g_stats_.jitter_acc);
      max_jitter         = std::max(max_jitter, extract_result<tag::max>(g_stats_.jitter_acc));
      g_stats_.jitter_acc = zero;

      static bool first = true;
      if (first)
      {
        first = false;
        status.add("Robot Description", g_robot_desc_);
      }

      status.addf("Max EtherCAT roundtrip (us)", "%.2f", max_ec*USEC_PER_SECOND);
      status.addf("Avg EtherCAT roundtrip (us)", "%.2f", avg_ec*USEC_PER_SECOND);
      status.addf("Max Controller Manager roundtrip (us)", "%.2f", max_cm*USEC_PER_SECOND);
      status.addf("Avg Controller Manager roundtrip (us)", "%.2f", avg_cm*USEC_PER_SECOND);
      status.addf("Max Total Loop roundtrip (us)", "%.2f", max_loop*USEC_PER_SECOND);
      status.addf("Avg Total Loop roundtrip (us)", "%.2f", avg_loop*USEC_PER_SECOND);
      status.addf("Max Loop Jitter (us)", "%.2f", max_jitter * USEC_PER_SECOND);
      status.addf("Avg Loop Jitter (us)", "%.2f", avg_jitter * USEC_PER_SECOND);
      status.addf("Control Loop Overruns", "%d", g_stats_.overruns);
      status.addf("Total Loop Count", "%ul", g_stats_.loop_count);
      status.addf("Recent Control Loop Overruns", "%d", g_stats_.recent_overruns);
      status.addf("Last Control Loop Overrun Cause", "ec: %.2fus, cm: %.2fus", 
                  g_stats_.overrun_ec*USEC_PER_SECOND, g_stats_.overrun_cm*USEC_PER_SECOND);
      status.addf("Last Overrun Loop Time (us)", "%.2f", g_stats_.overrun_loop_sec * USEC_PER_SECOND);
      status.addf("Realtime Loop Frequency", "%.4f", g_stats_.rt_loop_frequency);

      status.name = "Realtime Control Loop";
      if (g_stats_.overruns > 0 && g_stats_.last_overrun < 30)
      {
        if (g_stats_.last_severe_overrun < 30)
          status.level = 1;
        else
          status.level = 0;
        status.message = "Realtime loop used too much time in the last 30 seconds.";
      }
      else
      {
        status.level = 0;
        status.message = "OK";
      }
      g_stats_.recent_overruns = 0;
      g_stats_.last_overrun++;
      g_stats_.last_severe_overrun++;

      if (g_stats_.rt_loop_not_making_timing)
      {
        status.mergeSummaryf(status.ERROR, "Halting, realtime loop only ran at %.4f Hz", g_stats_.halt_rt_loop_frequency);
      }

      statuses.push_back(status);
      publisher_->msg_.status = statuses;
      publisher_->msg_.header.stamp = ros::Time::now();
      publisher_->unlockAndPublish();
    }
  }

  void OpenController::parseArguments(int argc, char** argv) {
    while (1) {
      static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"stats", no_argument, 0, 's'},
        {"allow_unprogrammed", no_argument, 0, 'u'},
        {"interface", required_argument, 0, 'i'},
        {"xml", required_argument, 0, 'x'},
        {"dryrun", no_argument, 0, 'd'}
      };
    int option_index = 0;
    int c = getopt_long(argc, argv, "hi:usdx:", long_options, &option_index);
    if (c == -1) break;
    switch (c) {
    case 'h':
      Usage();
      break;
    case 'u':
      setAllowUnprogrammedP(true);
      break;
    case 'x':
      setRobotXMLFile(std::string(optarg));
      break;
    case 's':
      setStatsPublishP(true);
      break;
    case 'd':
      setDryRun(true);
      break;
    }
    }
  }
  
  
  void OpenController::initialize() {
    ros::NodeHandle node("~");
    initializeROS(node);            // initialize ros anyway
    reset_service_
      = node.advertiseService("reset_motors",
                              &OpenController::resetMotorsService,
                              this);
    halt_service_
      = node.advertiseService("halt_motors",
                              &OpenController::haltMotorsService,
                              this);
    publishTrace_service_
      = node.advertiseService("publish_trace",
                              &OpenController::publishTraceService,
                              this);
    
    publisher_.reset(new realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray>(node, "/diagnostics", 2));

    if (!node.getParam("not_sleep_clock", not_sleep_clock_)) {
      not_sleep_clock_ = false;
    }

    if (!node.getParam("min_acceptable_rt_loop_frequency", min_acceptable_rt_loop_frequency_))
    {
      min_acceptable_rt_loop_frequency_ = 750.0; 
    }
    else 
    {
      ROS_WARN("min_acceptable_rt_loop_frequency changed to %f", min_acceptable_rt_loop_frequency_);
    }
    
    // read period
    if (!node.getParam("rt_period", period_))
    {
      ROS_WARN("failed to read rt_period parameter");
      period_ = 1e+6;            // 1ms in nanoseconds
    }
    ROS_INFO("realtime loop period is %f", period_);

    if (!node.getParam("pidfile", pidfile_)) {
      pidfile_ = "open_controller.pid";
    }
    ROS_INFO("pidfile is: %s", pidfile_.c_str());

    if (!node.getParam("piddir", piddir_)) {
      piddir_ = "/var/tmp/run/";
    }
    ROS_INFO("piddir is: %s", piddir_.c_str());
    
    if (stats_publish_p_) {
      rtpublisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(node, "realtime", 2));
    }
    
    // initialize pid file
    if (setupPidFile() < 0) {
      ROS_FATAL("pid file (%s/%s) already exists", piddir_.c_str(), pidfile_.c_str());
      exit(1);
    }
    
    // Initialize the hardware interface
    //EthercatHardware ec(name);
    //ec.init(g_options.interface_, g_options.allow_unprogrammed_);
    initializeCM();
    loadRobotDescription();
    initializeHW();
    
    for (size_t i = 0; i < cm_->state_->joint_states_.size(); i++) {
      cm_->state_->joint_states_[i].calibrated_ = true;
    }

   // Publish one-time before entering real-time to pre-allocate message vectors
    publishDiagnostics(); //ueda

    //Start Non-realtime diagonostic thread
    boost::thread t(&OpenController::diagnosticLoop, this);

    // Set to realtime scheduler for this thread
#if 0                           // disable
    struct sched_param thread_param;
    policy = SCHED_FIFO;
    thread_param.sched_priority = sched_get_priority_max(policy);
    if ( pthread_setschedparam(pthread_self(), policy, &thread_param) < -1 ) {
      perror("sched_setscheduler");
    }
#endif
  }

  void OpenController::loadRobotDescription() {
    // Load robot description
    TiXmlElement *root;
    TiXmlElement *root_element;
    
    TiXmlDocument xml;
    struct stat st;
    if (0 == stat(robot_xml_file_.c_str(), &st))
    {
      xml.LoadFile(robot_xml_file_.c_str());
    }
    else
    {
      ROS_INFO("Xml file not found, reading from parameter server");
      ros::NodeHandle top_level_node;
      if (top_level_node.getParam(robot_xml_file_.c_str(), g_robot_desc_))
        xml.Parse(g_robot_desc_.c_str());
      else
      {
        ROS_FATAL("Could not load the xml from parameter server: %s", robot_xml_file_.c_str());
        throw "end";
      }
    }
    root_element = xml.RootElement();
    root = xml.FirstChildElement("robot");
    if (!root || !root_element)
    {
      ROS_FATAL("Could not parse the xml from %s", robot_xml_file_.c_str());
      throw "end";
    }

    // Initialize the controller manager from robot description
    if (!cm_->initXml(root))
    {
      ROS_FATAL("Could not initialize the controller manager");
      throw "end";
    }
    else {
      ROS_INFO("success to initialize the controller manager");
    }
  }

  double OpenController::now() {
    struct timespec n;
    clock_gettime(CLOCK_MONOTONIC, &n);
    return double(n.tv_nsec) / NSEC_PER_SECOND + n.tv_sec;
  }

  void OpenController::timespecInc(struct timespec &tick, int nsec)
  {
    tick.tv_nsec += nsec;
    while (tick.tv_nsec >= NSEC_PER_SECOND)
    {
      tick.tv_nsec -= NSEC_PER_SECOND;
      tick.tv_sec++;
    }
  }
  double OpenController::publishJitter(double start) 
  {
    double jitter = now() - start;
    g_stats_.jitter_acc(jitter);
    // Publish realtime loops statistics, if requested
    if (!!rtpublisher_)
      {
        if (rtpublisher_->trylock())
          {
            rtpublisher_->msg_.data  = jitter;
            rtpublisher_->unlockAndPublish();
          }
      }
    return jitter;
  }


  void OpenController::startMain() {

    // setup the priority of the thread
    struct sched_param thread_param;
      int policy = SCHED_FIFO;
      thread_param.sched_priority = sched_get_priority_max(policy);
      if ( pthread_setschedparam(pthread_self(), policy, &thread_param) < -1 ) {
        perror("sched_setscheduler");
        ROS_ERROR("failed to sched_setscheduler");
      }

    // Keep history of last 3 calculation intervals.
    // the value is Hz we supporsed for realtime loop to be in
    RTLoopHistory rt_loop_history(3, 1.0 / (period_ / NSEC_PER_SECOND)); 
    double rt_loop_monitor_period_ = 0.6 / 3;
    unsigned long rt_cycle_count = 0;
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);
    // Snap to the nearest second
    timespecInc(tick, period_);
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    double last_published = now();
    double last_loop_start = now();
    double last_rt_monitor_time = now();
    struct timespec last_exec_time;
    clock_gettime(CLOCK_REALTIME, &last_exec_time);
    g_stats_.loop_count = 0;
    while (!g_quit_ && !g_reset_motors_) {
      g_stats_.loop_count++;
      // Track how long the actual loop takes
      double this_loop_start = now();
      g_stats_.loop_acc(this_loop_start - last_loop_start);
      last_loop_start = this_loop_start;
      bool success_update_joint = false;
      double start = now();
      if (g_reset_motors_) {
        // Also, clear error flags when motor reset is requested
        g_stats_.rt_loop_not_making_timing = false;
        break; // go to recoverController
      }
      else {
        // struct timespec current_time;
        // clock_gettime(CLOCK_REALTIME, &current_time);
        // timespecInc(exec_time, period_);
        // if (((exec_time.tv_sec - current_time.tv_sec) + double(exec_time.tv_nsec - current_time.tv_nsec) / NSEC_PER_SECOND) > 0) {
        //   ROS_INFO("sleep %f", ((exec_time.tv_sec - current_time.tv_sec) + double(exec_time.tv_nsec - current_time.tv_nsec) / NSEC_PER_SECOND));
        //   clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &exec_time, NULL);
        // }
        // clock_gettime(CLOCK_REALTIME, &exec_time);

        struct timespec exec_time;

        ControllerStatusPtr status = updateJoints(&exec_time);
        success_update_joint = status->isHealthy();
        g_reset_motors_ = !success_update_joint;
        // double diff = timespecDiff(&exec_time, &last_exec_time);
        // ROS_INFO("diff: %f", diff);
        last_exec_time.tv_sec = exec_time.tv_sec;
        last_exec_time.tv_nsec = exec_time.tv_nsec;
      }

      if (g_publish_trace_requested_)
      {
        g_publish_trace_requested_ = false;
#if 0
        ec.publishTrace(-1,"",0,0);
#endif
      }
      double after_ec = now();
      if (success_update_joint) {
        cm_->update();
      }
      double end = now();

      g_stats_.ec_acc(after_ec - start);
      g_stats_.cm_acc(end - after_ec);

      if ((end - last_published) > 1.0)
      {
        publishDiagnostics();
        last_published = end;
      }

      // Realtime loop should run about 1000Hz.  
      // Missing timing on a control cycles usually causes a controller glitch and actuators to jerk.
      // When realtime loop misses a lot of cycles controllers will perform poorly and may cause robot to shake.
      // Halt motors if realtime loop does not run enough cycles over a given period.
      ++rt_cycle_count;
      if ((start - last_rt_monitor_time) > rt_loop_monitor_period_)
      {
        // Calculate new average rt loop frequency       
        double rt_loop_frequency = double(rt_cycle_count) / rt_loop_monitor_period_;

        // Use last X samples of frequency when deciding whether or not to halt
        rt_loop_history.sample(rt_loop_frequency);
        double avg_rt_loop_frequency = rt_loop_history.average();
        if (avg_rt_loop_frequency < min_acceptable_rt_loop_frequency_)
        {
          if (!g_stats_.rt_loop_not_making_timing)
          {
            // Only update this value if motors when this first occurs (used for diagnostics error message)
            g_stats_.halt_rt_loop_frequency = avg_rt_loop_frequency;
          }
          g_stats_.rt_loop_not_making_timing = true;
        }
        g_stats_.rt_loop_frequency = avg_rt_loop_frequency;
        rt_cycle_count = 0;
        last_rt_monitor_time = start;
      }
      // Compute end of next period
      timespecInc(tick, period_);

      struct timespec before; 
      clock_gettime(CLOCK_REALTIME, &before); 
      double overrun_time = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) - 
        (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);
      if (overrun_time > 0.0)
      {
        ROS_WARN("  overrun: %f", overrun_time);
        double jitter = publishJitter(start);
        ROS_WARN("  jitter: %f", jitter);
        ROS_WARN("  loop:   %d", g_stats_.loop_count);
        // Total amount of time the loop took to run
        g_stats_.overrun_loop_sec = overrun_time;

        // We overran, snap to next "period"
        // ueda
        //timespecInc(tick, (before.tv_nsec / period_ + 1) * period_);
        tick.tv_sec = before.tv_sec;
        // tick.tv_nsec = (before.tv_nsec / period_) * period_;
        tick.tv_nsec = before.tv_nsec;
        timespecInc(tick, period_);

        // initialize overruns
        if (g_stats_.overruns == 0){
          g_stats_.last_overrun = 1000;
          g_stats_.last_severe_overrun = 1000;
        }
        // check for overruns
        if (g_stats_.recent_overruns > 10)
          g_stats_.last_severe_overrun = 0;
        g_stats_.last_overrun = 0;

        g_stats_.overruns++;
        g_stats_.recent_overruns++;
        g_stats_.overrun_ec = after_ec - start;
        g_stats_.overrun_cm = end - after_ec;
      }

      struct timespec sleep_before;
      clock_gettime(CLOCK_REALTIME, &sleep_before);
      // Sleep until end of period
      if (!not_sleep_clock_)
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
      if (overrun_time <= 0.0)
        publishJitter(start);
      // Calculate RT loop jitter
      struct timespec sleep_after;
      clock_gettime(CLOCK_REALTIME, &sleep_after);
      double sleep_time = (sleep_after.tv_sec - sleep_before.tv_sec
                           + double(sleep_after.tv_nsec-sleep_before.tv_nsec)/NSEC_PER_SECOND);
      //double jitter = (after.tv_sec - tick.tv_sec + double(after.tv_nsec-tick.tv_nsec)/NSEC_PER_SECOND);
      if (overrun_time > 0.0) {
        ROS_WARN("  sleep_time: %f", sleep_time);
      }

      // Halt the motors, if requested by a service call
      if (g_halt_requested_)
      {
        fprintf(stderr, "detect halt request\n");
        g_quit_ = true;
        g_halt_requested_ = false;
      }
    }
    ROS_INFO("good bye startMain()");
  }

  void OpenController::finalize() {
    ROS_WARN("finalizing");
    finalizeHW();
    /* Shutdown all of the motors on exit */
    for (pr2_hardware_interface::ActuatorMap::const_iterator it = hw_->actuators_.begin(); it != hw_->actuators_.end(); ++it)
    {
      it->second->command_.enable_ = false;
      it->second->command_.effort_ = 0;
    }
#if 0
    ec.update(false, true);
#endif
    //pthread_join(diagnosticThread, 0);
    if (!!publisher_) {
      publisher_->stop();
      publisher_.reset();
    }
    rtpublisher_.reset();
    //ros::shutdown();
    //return (void *)rv;
    fprintf(stderr, "exiting from finalize\n");
  }

  void OpenController::diagnosticLoop() {
    //EthercatHardware *ec((EthercatHardware *) args);
    struct timespec tick;
    clock_gettime(CLOCK_MONOTONIC, &tick);
    while (!g_quit_) {
      //ec->collectDiagnostics();
      tick.tv_sec += 1;
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
    }
  }

  int OpenController::lock_fd(int fd) {
    struct flock lock;
    int rv;
    
    lock.l_type = F_WRLCK;
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = 0;
    
    rv = fcntl(fd, F_SETLK, &lock);
    return rv;
  }
  
  int OpenController::setupPidFile() {
    int rv = -1;
    pid_t pid;
    int fd;
    FILE *fp = NULL;
    boost::filesystem::path fullpath = boost::filesystem::path(piddir_) / boost::filesystem::path(pidfile_);
    umask(0);
    mkdir(piddir_.c_str(), 0777);
    fd = open(fullpath.c_str(), O_RDWR | O_CREAT | O_EXCL, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IWOTH | S_IROTH);
    if (fd == -1)
    {
      if (errno != EEXIST)
      {
        ROS_FATAL("Unable to create pid file '%s': %s", fullpath.c_str(), strerror(errno));
        goto end;
      }

      if ((fd = open(fullpath.c_str(), O_RDWR)) < 0)
      {
        ROS_FATAL("Unable to open pid file '%s': %s", fullpath.c_str(), strerror(errno));
        goto end;
      }

      if ((fp = fdopen(fd, "rw")) == NULL)
      {
        ROS_FATAL("Can't read from '%s': %s", fullpath.c_str(), strerror(errno));
        goto end;
      }
      pid = -1;
      if ((fscanf(fp, "%d", &pid) != 1) || (pid == getpid()) || (lock_fd(fileno(fp)) == 0))
      {
        int rc;

        if ((rc = unlink(fullpath.c_str())) == -1)
        {
          ROS_FATAL("Can't remove stale pid file '%s': %s", fullpath.c_str(), strerror(errno));
          goto end;
        }
      } else {
        ROS_FATAL("Another instance of pr2_etherCAT is already running with pid: %d", pid);
        goto end;
      }
    }

    unlink(fullpath.c_str());
    fd = open(fullpath.c_str(), O_RDWR | O_CREAT | O_EXCL, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IWOTH | S_IROTH);

    if (fd == -1)
    {
      ROS_FATAL("Unable to open pid file '%s': %s", fullpath.c_str(), strerror(errno));
      goto end;
    }

    if (lock_fd(fd) == -1)
    {
      ROS_FATAL("Unable to lock pid file '%s': %s", fullpath.c_str(), strerror(errno));
      goto end;
    }

    if ((fp = fdopen(fd, "w")) == NULL)
    {
      ROS_FATAL("fdopen failed: %s", strerror(errno));
      goto end;
    }

    fprintf(fp, "%d\n", getpid());

    /* We do NOT close fd, since we want to keep the lock. */
    fflush(fp);
    fcntl(fd, F_SETFD, (long) 1);
    rv = 0;
  end:
    return rv;
  }

  void OpenController::cleanupPidFile() {
    boost::filesystem::path path = boost::filesystem::path(piddir_) / boost::filesystem::path(pidfile_);
    unlink(path.c_str());
  }
  
  void OpenController::Usage(std::string msg) {
    fprintf(stderr, "Usage: main [options]\n");
    fprintf(stderr, "  Available options\n");
    fprintf(stderr, "    -s, --stats                 Publish statistics on the RT loop jitter on \"pr2_etherCAT/realtime\" in seconds\n");
    fprintf(stderr, "    -x, --xml <file|param>      Load the robot description from this file or parameter name\n");
    fprintf(stderr, "    -d, --dryrun                Run in dry run mode, not communicating with the controller.\n");
    fprintf(stderr, "    -h, --help                  Print this message and exit\n");
    if (msg != "")
    {
      fprintf(stderr, "Error: %s\n", msg.c_str());
    }
  }

  void OpenController::quitRequest() {
    g_halt_requested_ = true;
  }

  bool OpenController::initRT() {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
      perror("mlockall");
      return false;
    }
    else {
      return true;
    }
  }

  Finalizer::~Finalizer() {
    ROS_WARN("Finalizer::~Finalizer is called");
    controller->finalize();
  }
}

