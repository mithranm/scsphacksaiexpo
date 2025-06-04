#include "TrickExecutorPlugin.hh"

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/common/Time.hh>      // For ignition::common::Time
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Events.hh>

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>
#include <functional> // For std::bind

// For ZMQ C API options if needed
extern "C" {
#include <zmq.h>
}

namespace gazebo
{
  // Default parameters for the test mode if not specified in SDF
  const bool DEFAULT_RUN_TEST_MODE_PARAM = false;
  const double DEFAULT_TEST_RADIUS_PARAM = 2.0;
  const double DEFAULT_TEST_ANGULAR_VELOCITY_PARAM = 0.5; // rad/s
  const double DEFAULT_TEST_ALTITUDE_PARAM = 1.0;     // meters
  const double DEFAULT_TEST_MODE_DURATION_PARAM = 60.0; // seconds

  TrickExecutorPlugin::TrickExecutorPlugin() :
    ModelPlugin(),
    zmq_context(1),
    zmq_socket(zmq_context, zmq::socket_type::rep)
  {
    terminate_zmq_thread.store(false);
    executing_trick.store(false);
    trick_sim_complete_flag.store(false);
    is_test_mode_active.store(false);
    std::cout << "[TrickPlugin CONSTRUCTOR] Plugin instance created. (stdout)" << std::endl;
    gzmsg << "[TrickPlugin CONSTRUCTOR] Plugin instance created. (gzmsg)" << std::endl;
  }

  TrickExecutorPlugin::~TrickExecutorPlugin()
  {
    std::string model_name = (this->model ? this->model->GetName() : "UnknownModel");
    gzmsg << "[" << model_name << " TrickPlugin DESTRUCTOR] Called." << std::endl;

    this->terminate_zmq_thread.store(true);
    if (this->updateConnection) {
        this->updateConnection.reset();
        gzmsg << "[" << model_name << " TrickPlugin] Disconnected WorldUpdateBegin event." << std::endl;
    }

    if (zmq_socket.connected()) { // This check might not be meaningful for REP sockets
        int linger_val = 0;
        // zmq_socket.set(zmq::sockopt::linger, 0); // C++ ZMQ way
        zmq_socket.setsockopt(ZMQ_LINGER, &linger_val, sizeof(linger_val)); // C API way
        // zmq_socket.close(); // Consider if needed, or if join handles it.
    }


    if (this->zmq_thread.joinable()) {
      gzmsg << "[" << model_name << " TrickPlugin] Waiting for ZMQ thread to join..." << std::endl;
      this->zmq_thread.join();
      gzmsg << "[" << model_name << " TrickPlugin] ZMQ thread joined." << std::endl;
    }
    // zmq_context.close(); // Or zmq_term(zmq_context.ptr) if managing context manually
    gzmsg << "[" << model_name << " TrickPlugin] Destructor finished." << std::endl;
  }

  void TrickExecutorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    std::cout << "[TrickPlugin LOAD_ENTRY] Entered Load function. (stdout)" << std::endl;
    gzmsg << "[TrickPlugin LOAD_ENTRY] Entered Load function. (gzmsg)" << std::endl;

    this->model = _model;
    if (!this->model) {
        gzerr << "[TrickPlugin LOAD_ERROR] Model pointer is NULL in Load!" << std::endl;
        std::cerr << "[TrickPlugin LOAD_ERROR] Model pointer is NULL in Load! (stderr)" << std::endl;
        return;
    }
    gzmsg << "[" << this->model->GetName() << " TrickPlugin] Load function called. Model name confirmed." << std::endl;

    if (!_sdf) {
        gzerr << "[" << this->model->GetName() << " TrickPlugin LOAD_ERROR] SDF pointer is NULL in Load!" << std::endl;
        std::cerr << "[" << this->model->GetName() << " TrickPlugin LOAD_ERROR] SDF pointer is NULL in Load! (stderr)" << std::endl;
        return;
    }
    gzmsg << "[" << this->model->GetName() << " TrickPlugin] SDF pointer seems valid." << std::endl;

    if (_sdf->HasElement("zmq_port")) {
        this->zmq_port_str = _sdf->Get<std::string>("zmq_port");
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] ZMQ Port from SDF: " << this->zmq_port_str << std::endl;
    } else {
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] zmq_port not specified in SDF, using default: " << this->zmq_port_str << std::endl;
    }
    std::string bind_addr = "tcp://*:" + this->zmq_port_str;

    gzmsg << "[" << this->model->GetName() << " TrickPlugin] Attempting to find 'hull' link..." << std::endl;
    this->body_link = this->model->GetLink("hull");
    if (!this->body_link) {
      gzerr << "[" << this->model->GetName() << " TrickPlugin] CRITICAL: 'hull' link not found!" << std::endl;
      std::cerr << "[" << this->model->GetName() << " TrickPlugin] CRITICAL: 'hull' link not found! (stderr)" << std::endl;
      return;
    }
    gzmsg << "[" << this->model->GetName() << " TrickPlugin] 'hull' link FOUND." << std::endl;

    std::string rotor_link_base_name = "rotor_";
    if (_sdf->HasElement("rotor_link_base_name")) {
        rotor_link_base_name = _sdf->Get<std::string>("rotor_link_base_name");
    }
    gzmsg << "[" << this->model->GetName() << " TrickPlugin] Using rotor_link_base_name: " << rotor_link_base_name << std::endl;


    this->rotor_links.clear();
    for (int i = 0; i < 4; ++i) {
      std::string current_rotor_name = rotor_link_base_name + std::to_string(i);
      gzmsg << "[" << this->model->GetName() << " TrickPlugin] Attempting to find '" << current_rotor_name << "' link..." << std::endl;
      physics::LinkPtr rotor = this->model->GetLink(current_rotor_name);
      if (rotor) {
        this->rotor_links.push_back(rotor);
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] '" << current_rotor_name << "' link FOUND." << std::endl;
      } else {
        gzerr << "[" << this->model->GetName() << " TrickPlugin] CRITICAL: '" << current_rotor_name << "' link not found!" << std::endl;
        std::cerr << "[" << this->model->GetName() << " TrickPlugin] CRITICAL: '" << current_rotor_name << "' link not found! (stderr)" << std::endl;
        return;
      }
    }

    if (this->rotor_links.size() != 4) {
      gzerr << "[" << this->model->GetName() << " TrickPlugin] CRITICAL: Did not find all 4 rotor links! Found: " << this->rotor_links.size() << std::endl;
      std::cerr << "[" << this->model->GetName() << " TrickPlugin] CRITICAL: Did not find all 4 rotor links! Found: " << this->rotor_links.size() << " (stderr)" << std::endl;
      return;
    }
    gzmsg << "[" << this->model->GetName() << " TrickPlugin] All required links (hull, 4 rotors) found." << std::endl;

    try {
      gzmsg << "[" << this->model->GetName() << " TrickPlugin] Attempting ZMQ setup on " << bind_addr << std::endl;
      void* zmq_socket_ptr = static_cast<void*>(zmq_socket);
      int linger_value = 0;
      int rc_opt = zmq_setsockopt(zmq_socket_ptr, ZMQ_LINGER, &linger_value, sizeof(linger_value));
      if (rc_opt != 0) {
          gzerr << "[" << this->model->GetName() << " TrickPlugin] ZMQ SETSOCKOPT ZMQ_LINGER FAILED: " << zmq_strerror(zmq_errno()) << std::endl;
          std::cerr << "[" << this->model->GetName() << " TrickPlugin] ZMQ SETSOCKOPT ZMQ_LINGER FAILED: " << zmq_strerror(zmq_errno()) << " (stderr)" << std::endl;
      } else {
          gzmsg << "[" << this->model->GetName() << " TrickPlugin] ZMQ_LINGER set to 0." << std::endl;
      }

      zmq_socket.bind(bind_addr.c_str());
      gzmsg << "[" << this->model->GetName() << " TrickPlugin] ZMQ REP Server listening on " << bind_addr << std::endl;
      this->zmq_thread = std::thread(&TrickExecutorPlugin::ZmqServerLoop, this);
      gzmsg << "[" << this->model->GetName() << " TrickPlugin] ZMQ thread launched." << std::endl;
    } catch (const zmq::error_t& e) {
      gzerr << "[" << this->model->GetName() << " TrickPlugin] ZMQ Operation FAILED: " << e.what() << std::endl;
      std::cerr << "[" << this->model->GetName() << " TrickPlugin] ZMQ Operation FAILED: " << e.what() << " (stderr)" << std::endl;
      return;
    }

    gzmsg << "[" << this->model->GetName() << " TrickPlugin] Checking for 'run_test_mode' in SDF..." << std::endl;
    if (!_sdf->HasElement("run_test_mode")) {
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] 'run_test_mode' element NOT FOUND in SDF. Test mode will be OFF by default." << std::endl;
    }

    bool run_test_from_sdf = _sdf->Get<bool>("run_test_mode", DEFAULT_RUN_TEST_MODE_PARAM).first;
    gzmsg << "[" << this->model->GetName() << " TrickPlugin] Value of 'run_test_mode' from SDF (or default if not found/parseable): " << (run_test_from_sdf ? "true" : "false") << std::endl;

    if (run_test_from_sdf) {
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] TEST MODE (Circle Flight) ATTEMPTING ACTIVATION from SDF." << std::endl;
        std::unique_lock<std::mutex> lock(this->trick_execution_mutex);
        this->is_test_mode_active.store(true);
        this->test_mode_radius = _sdf->Get<double>("test_mode_radius", DEFAULT_TEST_RADIUS_PARAM).first;
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] Test mode radius: " << this->test_mode_radius << std::endl;
        this->test_mode_angular_velocity = _sdf->Get<double>("test_mode_angular_velocity", DEFAULT_TEST_ANGULAR_VELOCITY_PARAM).first;
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] Test mode angular velocity: " << this->test_mode_angular_velocity << std::endl;
        this->test_mode_altitude = _sdf->Get<double>("test_mode_altitude", DEFAULT_TEST_ALTITUDE_PARAM).first;
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] Test mode altitude: " << this->test_mode_altitude << std::endl;

        this->executing_trick.store(true);
        this->trick_duration = DEFAULT_TEST_MODE_DURATION_PARAM;
        this->trick_time_step = 0.02;

        this->rotor_thrust_sequence.clear();
        this->recorded_trajectory.clear();
        this->trick_start_time_sim = -1.0;
        this->trick_current_time_sim_offset = 0.0;
        this->current_thrust_idx = 0;
        this->trick_sim_complete_flag.store(false);
        lock.unlock();
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] TEST MODE (Circle Flight) ACTIVATED from SDF. Flags set." << std::endl;
    } else {
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] Normal operation: Test mode is OFF based on SDF. Waiting for ZMQ commands." << std::endl;
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&TrickExecutorPlugin::OnUpdate, this));
    if(this->updateConnection) {
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] Connected WorldUpdateBegin event successfully." << std::endl;
    } else {
        gzerr << "[" << this->model->GetName() << " TrickPlugin] FAILED to connect WorldUpdateBegin event!" << std::endl;
        std::cerr << "[" << this->model->GetName() << " TrickPlugin] FAILED to connect WorldUpdateBegin event! (stderr)" << std::endl;
    }

    gzmsg << "[" << this->model->GetName() << " TrickPlugin] Load complete." << std::endl;
  }

  void TrickExecutorPlugin::ZmqServerLoop() {
    gzmsg << "[" << (this->model ? this->model->GetName() : "DetachedModel") << " TrickPlugin] ZMQ Thread Started." << std::endl;
    while (!this->terminate_zmq_thread.load()) {
      zmq::message_t request_msg;
      try {
        zmq::pollitem_t items[] = { { static_cast<void*>(zmq_socket), 0, ZMQ_POLLIN, 0 } };
        int rc = zmq::poll(&items[0], 1, std::chrono::milliseconds(200));

        if (this->terminate_zmq_thread.load()) {
            gzmsg << "[" << (this->model ? this->model->GetName() : "DetachedModel") << " TrickPlugin] ZMQ Thread: Termination signal received during poll wait." << std::endl;
            break;
        }

        if (rc > 0 && (items[0].revents & ZMQ_POLLIN)) {
          auto recvd_bytes = zmq_socket.recv(request_msg, zmq::recv_flags::none);
          if (!recvd_bytes.has_value() || recvd_bytes.value() == 0) {
             gzdbg << "[" << (this->model ? this->model->GetName() : "DetachedModel") << " TrickPlugin] ZMQ recv returned no/zero bytes." << std::endl;
             continue;
          }

          std::string request_str = request_msg.to_string();
          gzmsg << "[" << (this->model ? this->model->GetName() : "DetachedModel") << " TrickPlugin] Received ZMQ request (len " << request_str.length() << ")" << std::endl;

          json response_json;
          json request_json_parsed;
          try {
            request_json_parsed = json::parse(request_str);
            if (this->is_test_mode_active.load() && request_json_parsed.value("command", "") == "execute_rotor_thrust_sequence") {
                response_json["request_id"] = request_json_parsed.value("request_id", "N/A_test_mode_busy");
                response_json["status"] = "error";
                response_json["error_message"] = "Drone " + (this->model ? this->model->GetName() : "N/A") + " is in autonomous test mode, ZMQ commands ignored.";
                gzmsg << "[" << (this->model ? this->model->GetName() : "DetachedModel") << " TrickPlugin] Ignoring ZMQ command due to active test mode." << std::endl;
            } else {
                 response_json = HandleGazeboCommand(request_json_parsed);
            }
          } catch (const json::parse_error& e) {
            gzerr << "[" << (this->model ? this->model->GetName() : "DetachedModel") << " TrickPlugin] JSON Parse Error: " << e.what()
                  << " for request: " << request_str.substr(0, 200) << std::endl;
            response_json["request_id"] = request_json_parsed.value("request_id", "unknown_json_parse_error");
            response_json["status"] = "error";
            response_json["error_message"] = "Invalid JSON request: " + std::string(e.what());
          } catch (const std::exception& e) {
            gzerr << "[" << (this->model ? this->model->GetName() : "DetachedModel") << " TrickPlugin] Std Exception in HandleGazeboCommand: " << e.what() << std::endl;
            response_json["request_id"] = request_json_parsed.value("request_id", "unknown_std_exception_handler");
            response_json["status"] = "error";
            response_json["error_message"] = "Server error during command handling: " + std::string(e.what());
          }
          std::string response_str = response_json.dump();
          zmq_socket.send(zmq::buffer(response_str), zmq::send_flags::none);
        }
      } catch (const zmq::error_t& e) {
        if (e.num() == ETERM || e.num() == EINTR || this->terminate_zmq_thread.load()) {
          gzmsg << "[" << (this->model ? this->model->GetName() : "DetachedModel") << " TrickPlugin] ZMQ Interrupted/Terminated in recv/poll: " << e.what() << std::endl;
          break;
        }
        if (e.num() != EAGAIN) {
            gzerr << "[" << (this->model ? this->model->GetName() : "DetachedModel") << " TrickPlugin] ZMQ Error in ServerLoop (num " << e.num() << "): " << e.what() << std::endl;
        }
        if (!this->terminate_zmq_thread.load()) {
             if (e.num() != EAGAIN) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    }
    gzmsg << "[" << (this->model ? this->model->GetName() : "DetachedModel") << " TrickPlugin] ZMQ Thread Exiting." << std::endl;
  }

  json TrickExecutorPlugin::HandleGazeboCommand(const json& cmd_json) {
    json response;
    std::string req_id_str = cmd_json.value("request_id", "unknown_req_id");
    response["request_id"] = req_id_str;

    std::string command = cmd_json.value("command", "");
    gzmsg << "[" << this->model->GetName() << " TrickPlugin] HandleGazeboCommand: Received '" << command << "' for request ID: " << req_id_str << std::endl;

    if (command == "execute_rotor_thrust_sequence") {
      if (this->executing_trick.load()) {
        response["status"] = "error";
        response["error_message"] = "Model " + this->model->GetName() + " is already executing a maneuver (ZMQ or test mode).";
        gzerr << "[" << this->model->GetName() << " TrickPlugin] " << response["error_message"] << std::endl;
        return response;
      }

      std::unique_lock<std::mutex> lock(this->trick_execution_mutex);
      this->is_test_mode_active.store(false);

      ignition::math::Pose3d initial_pose;
      ignition::math::Vector3d initial_lin_vel;
      ignition::math::Vector3d initial_ang_vel;

      try {
        const json& state = cmd_json.at("initial_state");
        initial_pose.Pos().Set(state.at("pos")[0].get<double>(), state.at("pos")[1].get<double>(), state.at("pos")[2].get<double>());
        initial_pose.Rot().Set(state.at("orient")[3].get<double>(), state.at("orient")[0].get<double>(), state.at("orient")[1].get<double>(), state.at("orient")[2].get<double>());
        initial_lin_vel.Set(state.at("lin_vel")[0].get<double>(), state.at("lin_vel")[1].get<double>(), state.at("lin_vel")[2].get<double>());
        initial_ang_vel.Set(state.at("ang_vel")[0].get<double>(), state.at("ang_vel")[1].get<double>(), state.at("ang_vel")[2].get<double>());
      } catch (const json::exception& e) {
        response["status"] = "error";
        response["error_message"] = "Failed to parse initial_state: " + std::string(e.what());
        gzerr << "[" << this->model->GetName() << " TrickPlugin] " << response["error_message"] << std::endl;
        return response;
      }

      this->model->SetWorldPose(initial_pose);
      this->model->SetLinearVel(initial_lin_vel);
      this->model->SetAngularVel(initial_ang_vel);
      if(this->body_link) this->body_link->ResetPhysicsStates();
      for(const auto& rotor_link : this->rotor_links) { if(rotor_link) rotor_link->ResetPhysicsStates(); }
      gzmsg << "[" << this->model->GetName() << " TrickPlugin] Initial state set for drone." << std::endl;

      const json& thrust_info = cmd_json.value("rotor_thrusts_info", json::object());
      this->trick_duration = thrust_info.value("duration", 0.0);
      this->trick_time_step = thrust_info.value("time_step", 0.01);
      this->rotor_thrust_sequence.clear();

      try {
        const json& thrusts = thrust_info.at("rotor_thrusts");
        for (const auto& frame : thrusts) {
          this->rotor_thrust_sequence.push_back(frame.get<std::vector<double>>());
        }
      } catch (const json::exception& e) {
        response["status"] = "error";
        response["error_message"] = "Failed to parse rotor_thrusts_info: " + std::string(e.what());
        gzerr << "[" << this->model->GetName() << " TrickPlugin] " << response["error_message"] << std::endl;
        return response;
      }

      if (this->rotor_thrust_sequence.empty() || this->trick_duration <= 0 || this->trick_time_step <=0) {
        response["status"] = "error";
        response["error_message"] = "Invalid rotor thrust sequence: empty, zero duration, or zero time step.";
        gzerr << "[" << this->model->GetName() << " TrickPlugin] " << response["error_message"] << std::endl;
        return response;
      }
      gzmsg << "[" << this->model->GetName() << " TrickPlugin] Rotor thrust sequence parsed. Frames: " << this->rotor_thrust_sequence.size() << ", Duration: " << this->trick_duration << ", Timestep: " << this->trick_time_step << std::endl;

      this->recorded_trajectory.clear();
      this->trick_start_time_sim = -1.0;
      this->trick_current_time_sim_offset = 0.0;
      this->current_thrust_idx = 0;
      this->trick_sim_complete_flag.store(false);
      this->executing_trick.store(true);
      lock.unlock();

      gzmsg << "[" << this->model->GetName() << " TrickPlugin] Accepted ZMQ trick. ID: " << req_id_str << ". Waiting for completion..." << std::endl;
      std::unique_lock<std::mutex> cv_lock(this->trick_completion_mutex);
      double wait_timeout_seconds = this->trick_duration + 20.0;
      if (!this->trick_completion_cv.wait_for(cv_lock, std::chrono::duration<double>(wait_timeout_seconds),
                                         [this]{ return this->trick_sim_complete_flag.load(); })) {
        gzerr << "[" << this->model->GetName() << " TrickPlugin] ZMQ Trick TIMEOUT! ID: " << req_id_str << ". Waited " << wait_timeout_seconds << "s." << std::endl;
        std::unique_lock<std::mutex> timeout_lock(this->trick_execution_mutex);
        this->executing_trick.store(false);
        this->trick_sim_complete_flag.store(false);
        timeout_lock.unlock();

        response["status"] = "error";
        response["error_message"] = "ZMQ Trick timed out in Gazebo plugin (CV wait).";
      } else {
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] ZMQ Trick completed signal received. ID: " << req_id_str << std::endl;
        response["status"] = "success";
        std::unique_lock<std::mutex> trajectory_lock(this->trick_execution_mutex);
        response["trajectory_frames"] = this->recorded_trajectory;
        this->recorded_trajectory.clear();
      }
    } else {
      response["status"] = "error";
      response["error_message"] = "Unknown command: " + command;
      gzerr << "[" << this->model->GetName() << " TrickPlugin] " << response["error_message"] << std::endl;
    }
    return response;
  }

  void TrickExecutorPlugin::OnUpdate() {
    if (!this->executing_trick.load()) {
        if (this->body_link && this->model->GetWorld()->SimTime().Double() > 0.1) {
            for (const auto& rotor_link : this->rotor_links) {
                if (rotor_link) rotor_link->SetForce(ignition::math::Vector3d::Zero);
            }
            this->body_link->SetTorque(ignition::math::Vector3d::Zero);
        }
        return;
    }

    bool current_is_active_test_mode;
    {
        std::unique_lock<std::mutex> read_lock(this->trick_execution_mutex);
        current_is_active_test_mode = this->is_test_mode_active.load();
    }

    std::unique_lock<std::mutex> lock(this->trick_execution_mutex);

    if (!this->model || !this->model->GetWorld() || this->model->GetWorld()->SimTime() < gazebo::common::Time::Zero) {
        gzdbg << "[" << (this->model ? this->model->GetName() : "DetachedModel") << " TrickPlugin] OnUpdate: World or Model not ready yet." << std::endl;
        return;
    }

    if (this->trick_start_time_sim < 0) {
        this->trick_start_time_sim = this->model->GetWorld()->SimTime().Double();
        this->trick_current_time_sim_offset = 0.0;
        this->current_thrust_idx = 0;
        this->recorded_trajectory.clear();
        gzmsg << "[" << this->model->GetName() << " TrickPlugin] OnUpdate: Starting "
              << (current_is_active_test_mode ? "TEST MODE Circle Flight" : "ZMQ Trick")
              << " at sim time " << this->trick_start_time_sim << ". Duration: " << this->trick_duration << std::endl;
    }

    double current_sim_time_world = this->model->GetWorld()->SimTime().Double();
    if (current_sim_time_world < this->trick_start_time_sim) {
        gzwarn << "[" << this->model->GetName() << " TrickPlugin] Sim time (" << current_sim_time_world
              << ") is less than trick start time (" << this->trick_start_time_sim
              << "). Resetting start time." << std::endl;
        this->trick_start_time_sim = current_sim_time_world;
    }
    this->trick_current_time_sim_offset = current_sim_time_world - this->trick_start_time_sim;

    bool trick_should_end_by_duration = (this->trick_current_time_sim_offset >= this->trick_duration - (this->trick_time_step / 2.0) );

    if (trick_should_end_by_duration) {
      this->executing_trick.store(false);
      gzmsg << "[" << this->model->GetName() << " TrickPlugin] OnUpdate: Sequence/Test ended. Duration: "
            << this->trick_current_time_sim_offset << " / " << this->trick_duration << std::endl;

      for (const auto& rotor_link : this->rotor_links) {
        if (rotor_link) rotor_link->SetForce(ignition::math::Vector3d::Zero);
      }
      if (this->body_link) {
         this->body_link->SetTorque(ignition::math::Vector3d::Zero);
      }

      if (!current_is_active_test_mode) {
          this->trick_sim_complete_flag.store(true);
          lock.unlock();
          this->trick_completion_cv.notify_one();
      } else {
          this->is_test_mode_active.store(false);
          gzmsg << "[" << this->model->GetName() << " TrickPlugin] OnUpdate: Test mode finished and flag reset." << std::endl;
          lock.unlock();
      }
      return;
    }

    ignition::math::Vector3d net_reaction_torque_body_local(0,0,0);
    double k_reaction_torque_coeff = 0.02;

    if (current_is_active_test_mode) {
        gzmsg << "[" << this->model->GetName() << " DBG_OnUpdate] Test mode entered. SimTimeOffset: " << this->trick_current_time_sim_offset << std::endl;

        if (!this->body_link) {
            gzerr << "[" << this->model->GetName() << " TrickPlugin] OnUpdate: Body link is null in test mode!" << std::endl;
            this->executing_trick.store(false);
            this->is_test_mode_active.store(false);
            return;
        }

        ignition::math::Pose3d current_pose = this->model->WorldPose();
        ignition::math::Vector3d current_vel_world = this->model->WorldLinearVel();
        ignition::math::Vector3d current_ang_vel_world = this->model->WorldAngularVel();

        double angle = this->trick_current_time_sim_offset * this->test_mode_angular_velocity;
        ignition::math::Vector3d target_pos_on_circle(
            this->test_mode_radius * cos(angle),
            this->test_mode_radius * sin(angle),
            this->test_mode_altitude
        );

        ignition::math::Vector3d pos_error = target_pos_on_circle - current_pose.Pos();
        double Kp_pos = 10.0, Kd_pos = 5.0;
        ignition::math::Vector3d target_force_world_pd = pos_error * Kp_pos - current_vel_world * Kd_pos;
        
        // --- Hardcoded mass for testing ---
        double model_mass = 0.0835; // Approximate total mass from previous calculation
        // if (this->body_link && this->body_link->GetInertial()) { // Original logic commented out
        //     model_mass = this->body_link->GetInertial()->Mass();
        // } else {
        //     gzwarn << "[" << this->model->GetName() << " TrickPlugin] Could not get model mass, using default " << model_mass << std::endl;
        // }
        gzmsg << "[" << this->model->GetName() << " DBG_OnUpdate] Using model_mass: " << model_mass << std::endl;


        ignition::math::Vector3d gravity_comp_force(0, 0, model_mass * std::abs(this->model->GetWorld()->Gravity().Z()));
        ignition::math::Vector3d total_target_force_world = target_force_world_pd + gravity_comp_force;
        
        ignition::math::Quaterniond body_orientation_inv = current_pose.Rot().Inverse();
        ignition::math::Vector3d total_target_force_body = body_orientation_inv * total_target_force_world;
        double required_total_thrust_body_z = total_target_force_body.Z();
        double thrust_per_rotor = std::max(0.0, required_total_thrust_body_z / 4.0);
        gzmsg << "[" << this->model->GetName() << " DBG_OnUpdate] TargetForce_BodyZ: " << total_target_force_body.Z() << " ThrustPerRotor: " << thrust_per_rotor << std::endl;


        ignition::math::Vector3d current_ang_vel_body = body_orientation_inv * current_ang_vel_world;
        ignition::math::Vector3d target_ang_vel_body(0, 0, 0);
        ignition::math::Vector3d ang_vel_error_body = target_ang_vel_body - current_ang_vel_body;
        double Kp_att_rate = 0.05; 
        ignition::math::Vector3d corrective_torque_body = ang_vel_error_body * Kp_att_rate;
        
        net_reaction_torque_body_local += corrective_torque_body;

        for (size_t i = 0; i < this->rotor_links.size(); ++i) {
            if (!this->rotor_links[i]) continue;
            ignition::math::Vector3d thrust_force_local(0, 0, thrust_per_rotor);
            ignition::math::Vector3d thrust_force_world_rotor = this->rotor_links[i]->WorldPose().Rot() * thrust_force_local;
            this->rotor_links[i]->SetForce(thrust_force_world_rotor);
            gzmsg << "[" << this->model->GetName() << " DBG_OnUpdate] Rotor " << i << " SetForce_World: " << thrust_force_world_rotor << std::endl;


            double single_rotor_reaction_z = k_reaction_torque_coeff * thrust_per_rotor;
            if (i == 0 || i == 2) { net_reaction_torque_body_local.Z(net_reaction_torque_body_local.Z() - single_rotor_reaction_z); }
            else { net_reaction_torque_body_local.Z(net_reaction_torque_body_local.Z() + single_rotor_reaction_z); }
        }

    } else { // ZMQ commanded trick
        if (this->rotor_thrust_sequence.empty()) {
            gzerr << "[" << this->model->GetName() << " TrickPlugin] OnUpdate: Rotor thrust sequence is empty for ZMQ trick!" << std::endl;
            this->executing_trick.store(false);
            this->trick_sim_complete_flag.store(true);
            lock.unlock();
            this->trick_completion_cv.notify_one();
            return;
        }

        size_t frame_idx = this->current_thrust_idx;
        while(frame_idx < this->rotor_thrust_sequence.size() - 1 &&
              this->rotor_thrust_sequence[frame_idx+1][0] <= this->trick_current_time_sim_offset + (this->trick_time_step / 2.0) ) {
            frame_idx++;
        }
        this->current_thrust_idx = frame_idx;

        const auto& thrust_frame = this->rotor_thrust_sequence[frame_idx];
        if (thrust_frame.size() != 5) {
             gzerr << "[" << this->model->GetName() << " TrickPlugin] OnUpdate: Invalid thrust frame size: " << thrust_frame.size() << std::endl;
             this->executing_trick.store(false);
             this->trick_sim_complete_flag.store(true);
             lock.unlock();
             this->trick_completion_cv.notify_one();
             return;
        }

        for (size_t i = 0; i < this->rotor_links.size(); ++i) {
            if (!this->rotor_links[i]) continue;
            double thrust_val = thrust_frame[i + 1];
            ignition::math::Vector3d thrust_force_local(0, 0, thrust_val);
            ignition::math::Vector3d thrust_force_world = this->rotor_links[i]->WorldPose().Rot() * thrust_force_local;
            this->rotor_links[i]->SetForce(thrust_force_world);

            double single_rotor_reaction_z = k_reaction_torque_coeff * thrust_val;
            if (i == 0 || i == 2) { net_reaction_torque_body_local.Z(net_reaction_torque_body_local.Z() - single_rotor_reaction_z); }
            else { net_reaction_torque_body_local.Z(net_reaction_torque_body_local.Z() + single_rotor_reaction_z); }
        }
    }

    if (this->body_link) {
        ignition::math::Vector3d final_torque_world = this->body_link->WorldPose().Rot() * net_reaction_torque_body_local;
        this->body_link->SetTorque(final_torque_world);
    }

    bool should_record_this_update = this->recorded_trajectory.empty() ||
                                     (this->trick_current_time_sim_offset - this->recorded_trajectory.back()["t"].get<double>()) >= this->trick_time_step - 1e-4;

    if (should_record_this_update) {
        json frame_data;
        frame_data["t"] = this->trick_current_time_sim_offset;
        if (this->body_link) { // Should always be true if we reached here and it's not a ZMQ error case
            ignition::math::Pose3d pose = this->model->WorldPose();
            ignition::math::Vector3d lin_vel = this->model->WorldLinearVel();
            ignition::math::Vector3d ang_vel = this->model->WorldAngularVel();

            frame_data["pos"] = {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()};
            frame_data["orient"] = {pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()};
            frame_data["lin_vel"] = {lin_vel.X(), lin_vel.Y(), lin_vel.Z()};
            frame_data["ang_vel"] = {ang_vel.X(), ang_vel.Y(), ang_vel.Z()};
        }
        this->recorded_trajectory.push_back(frame_data);
    }

    lock.unlock();
  }
  GZ_REGISTER_MODEL_PLUGIN(TrickExecutorPlugin)
}