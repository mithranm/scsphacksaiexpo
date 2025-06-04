#ifndef GAZEBO_TRICK_EXECUTOR_PLUGIN_HH
#define GAZEBO_TRICK_EXECUTOR_PLUGIN_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>      

#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <zmq.hpp> 
#include <nlohmann/json.hpp>

#include <ignition/math/Pose3.hh>    
#include <ignition/math/Vector3.hh>  
#include <ignition/math/Quaternion.hh> 

namespace gazebo
{
  using json = nlohmann::json;

  class TrickExecutorPlugin : public ModelPlugin
  {
    public:
      TrickExecutorPlugin();
      virtual ~TrickExecutorPlugin();
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    protected:
      virtual void OnUpdate();

    private:
      void ZmqServerLoop();
      json HandleGazeboCommand(const json& cmd_json);

      physics::ModelPtr model;
      event::ConnectionPtr updateConnection;

      zmq::context_t zmq_context;
      zmq::socket_t zmq_socket;
      std::thread zmq_thread;
      std::atomic<bool> terminate_zmq_thread{false};
      std::string zmq_port_str = "5555";

      std::atomic<bool> executing_trick{false};
      
      double trick_start_time_sim = 0.0;
      double trick_current_time_sim_offset = 0.0;
      double trick_duration = 0.0;
      double trick_time_step = 0.01; 
      std::vector<std::vector<double>> rotor_thrust_sequence;
      size_t current_thrust_idx = 0; 
      std::mutex trick_execution_mutex;

      std::vector<json> recorded_trajectory; 

      physics::LinkPtr body_link;
      std::vector<physics::LinkPtr> rotor_links;

      std::condition_variable trick_completion_cv;
      std::mutex trick_completion_mutex;
      std::atomic<bool> trick_sim_complete_flag{false};

      // Members for Test Mode (Circle Flight)
      std::atomic<bool> is_test_mode_active{false}; 
      double test_mode_radius = 2.0;
      double test_mode_angular_velocity = 0.5;
      double test_mode_altitude = 1.0;
      // Test mode will use trick_duration from constants for its duration
  };
}
#endif