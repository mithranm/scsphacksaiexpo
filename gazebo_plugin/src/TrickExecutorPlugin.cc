#include "TrickExecutorPlugin.hh" 
#include <gazebo/common/Events.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh> // <<< ADDED THIS INCLUDE
#include <ignition/math/Pose3.hh>      
#include <ignition/math/PID.hh>        
#include <ignition/math/Angle.hh>      
#include <iostream>
#include <thread>
#include <chrono> // Ensure this is included for std::chrono::duration
#include <zmq.h> 

using namespace gazebo;
using json = nlohmann::json;

static constexpr double GRAVITY_MAGNITUDE = 9.81;

//////////////////////////////////////////////////
TrickExecutorPlugin::TrickExecutorPlugin()
  : ModelPlugin(),
    zmqContext_(1),
    zmqSocket_(zmqContext_, zmq::socket_type::rep),
    runTestMode_(false), 
    testModeRadius_(2.0),
    testModeAngularVel_(0.5),
    testModeAltitude_(1.5),
    dragCoeff_(0.1),
    zmqPort_("5555"),
    sequenceStartTime_(0.0),
    sequenceDuration_(0.0),
    nextThrustIndex_(0)
{
    pidX_.Init(20.0, 0.1, 2.5);
    pidY_.Init(20.0, 0.1, 2.5);
    pidZ_.Init(30.0, 1.0, 5.0);
    pidYaw_.Init(2.5, 0.05, 0.25);

    pidX_.SetCmdMax(50.0); pidX_.SetCmdMin(-50.0);
    pidY_.SetCmdMax(50.0); pidY_.SetCmdMin(-50.0);
    pidZ_.SetCmdMax(100.0); pidZ_.SetCmdMin(-20.0);
    pidYaw_.SetCmdMax(2.0); pidYaw_.SetCmdMin(-2.0);
}

//////////////////////////////////////////////////
TrickExecutorPlugin::~TrickExecutorPlugin()
{
  this->zmqThreadRunning_.store(false);
  if (this->zmqSocket_ && this->zmqSocket_.handle()) {
      int linger_option_value = 0;
      try {
        this->zmqSocket_.setsockopt(ZMQ_LINGER, &linger_option_value, sizeof(linger_option_value));
      } catch (const zmq::error_t& e) {
        gzerr << "[TrickExecutorPlugin] Destructor: Failed to set ZMQ_LINGER: " << e.what() << std::endl;
      }
  }
  if (this->zmqThread_.joinable())
    this->zmqThread_.join();
}

//////////////////////////////////////////////////
void TrickExecutorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzmsg << "[TrickExecutorPlugin] Load called for model: " << _model->GetName() << std::endl;
  this->model_ = _model;
  this->world_ = this->model_->GetWorld(); 
  this->hullLink_ = this->model_->GetLink("hull");
  if (!this->hullLink_) {
    gzerr << "[TrickExecutorPlugin] ERROR: Could not find link 'hull'.\n"; return;
  }

  this->rotorBaseName_ = _sdf->Get<std::string>("rotor_link_base_name", "rotor_").first;
  this->rotorCount_ = _sdf->Get<unsigned int>("rotor_count", 4).first;
  this->runTestMode_ = _sdf->Get<bool>("run_test_mode", false).first;
  this->testModeRadius_ = _sdf->Get<double>("test_mode_radius", 2.0).first;
  this->testModeAngularVel_ = _sdf->Get<double>("test_mode_angular_velocity", 0.5).first;
  this->testModeAltitude_ = _sdf->Get<double>("test_mode_altitude", 1.5).first;
  this->dragCoeff_ = _sdf->Get<double>("drag_coefficient", 0.1).first;
  this->zmqPort_ = _sdf->Get<std::string>("zmq_port", "5555").first;

  this->rotorLinks_.clear();
  for (unsigned int i = 0; i < this->rotorCount_; ++i) {
    physics::LinkPtr rotorLink = this->model_->GetLink(this->rotorBaseName_ + std::to_string(i));
    if (!rotorLink) {
      gzerr << "[TrickExecutorPlugin] ERROR: Could not find rotor link: " << this->rotorBaseName_ + std::to_string(i) << "\n";
    } else {
      this->rotorLinks_.push_back(rotorLink);
    }
  }
  if (this->rotorLinks_.size() != this->rotorCount_) {
    gzerr << "[TrickExecutorPlugin] WARNING: Rotor link count (" << this->rotorLinks_.size() 
          << ") does not match expected rotor_count (" << this->rotorCount_ << ").\n"; 
    return;
  }

  if (!this->zmqThreadRunning_.load()) { 
    try {
      std::string endpoint = "tcp://*:" + this->zmqPort_;
      this->zmqSocket_.bind(endpoint); 
      gzmsg << "[TrickExecutorPlugin] ZMQ REP bound to " << endpoint << "\n";
      this->zmqThreadRunning_.store(true);
      this->zmqThread_ = std::thread(&TrickExecutorPlugin::ZmqServerLoop, this);
    } catch (const zmq::error_t &e) {
      gzerr << "[TrickExecutorPlugin] ZMQ bind failed (" << e.what() << "). Forcing test mode.\n";
      this->runTestMode_ = true;
      this->zmqThreadRunning_.store(false);
    }
  } else {
    gzerr << "[TrickExecutorPlugin] Load called again but ZMQ thread already running. Forcing test mode for this instance.\n";
    this->runTestMode_ = true; 
  }

  this->lastUpdateTime_ = this->world_->SimTime(); 

  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&TrickExecutorPlugin::OnUpdate, this));

  gzmsg << "[TrickExecutorPlugin] " << this->model_->GetName() << " initialized. Test mode: " << (this->runTestMode_ ? "ON" : "OFF") << std::endl;
}

//////////////////////////////////////////////////
void TrickExecutorPlugin::OnUpdate()
{
  if (!this->model_ || !this->hullLink_ || !this->world_) return;

  common::Time currentTime = this->world_->SimTime();
  double dt_double = (currentTime - this->lastUpdateTime_).Double(); // dt as double
  if (dt_double <= 1e-6) { 
      this->lastUpdateTime_ = currentTime; 
      return;
  }
  // Convert dt_double to std::chrono::duration<double> for PID::Update
  std::chrono::duration<double> dt_chrono(dt_double);


  if (this->runTestMode_) {
    ignition::math::Pose3 currentPose = this->hullLink_->WorldPose();
    ignition::math::Vector3d currentPos = currentPose.Pos();
    ignition::math::Quaterniond currentOrient = currentPose.Rot();

    double t = currentTime.Double(); 
    double targetX = this->testModeRadius_ * std::cos(this->testModeAngularVel_ * t);
    double targetY = this->testModeRadius_ * std::sin(this->testModeAngularVel_ * t);
    double targetZ = this->testModeAltitude_;

    double targetVx = -this->testModeRadius_ * this->testModeAngularVel_ * std::sin(this->testModeAngularVel_ * t);
    double targetVy =  this->testModeRadius_ * this->testModeAngularVel_ * std::cos(this->testModeAngularVel_ * t);

    ignition::math::Vector3d posError = ignition::math::Vector3d(targetX, targetY, targetZ) - currentPos;

    double forceX = pidX_.Update(posError.X(), dt_chrono); // Corrected: use dt_chrono
    double forceY = pidY_.Update(posError.Y(), dt_chrono); // Corrected: use dt_chrono
    double forceZ = pidZ_.Update(posError.Z(), dt_chrono) + this->GetModelMass() * GRAVITY_MAGNITUDE; // Corrected: use dt_chrono

    ignition::math::Vector3d forceCmdWorld(forceX, forceY, forceZ);

    double targetYawRad = atan2(targetVy, targetVx);
    double currentYawRad = currentOrient.Yaw();
    
    // Use ignition::math::Angle::Normalized() to ensure angles are in [-Pi, Pi] range before subtraction, then normalize difference.
    // A simpler approach for well-behaved angles is often just (target - current), then normalize.
    // Angle::NormalizePiAboutZero ensures the angle is in [-pi, pi].
    double yawErrorRadians = ignition::math::Angle(targetYawRad - currentYawRad).Normalized().Radian();
    double torqueZ_yaw = pidYaw_.Update(yawErrorRadians, dt_chrono); // Corrected: use dt_chrono

    ignition::math::Vector3d torqueCmdBody(0, 0, torqueZ_yaw); 

    this->hullLink_->AddForce(forceCmdWorld); 
    this->hullLink_->AddRelativeTorque(torqueCmdBody); 

  } else { // ZMQ Sequence Mode
    if (this->sequenceActive_.load()) {
      double now = currentTime.Double(); 
      double relT = now - this->sequenceStartTime_;

      if (this->nextThrustIndex_ < this->rotorThrustSequence_.size() &&
          relT >= this->rotorThrustSequence_[this->nextThrustIndex_].first) {
        this->currentThrusts_ = this->rotorThrustSequence_[this->nextThrustIndex_].second;
        this->nextThrustIndex_++;
      }

      if (!this->currentThrusts_.empty()) {
        this->ApplyRotorThrusts(this->currentThrusts_);
      }

      Frame f;
      f.t = relT;
      f.pos = this->hullLink_->WorldPose().Pos();
      f.orient = this->hullLink_->WorldPose().Rot();
      f.linVel = this->hullLink_->WorldLinearVel();
      f.angVel = this->hullLink_->WorldAngularVel();
      this->capturedFrames_.push_back(f);

      if (relT >= this->sequenceDuration_) {
        this->sequenceActive_.store(false);
        this->currentThrusts_.assign(this->rotorCount_, 0.0); 
      }
    } else {
        this->ApplyRotorThrusts(std::vector<double>(this->rotorCount_, 0.0));
    }
  }

  ApplyDrag(); 
  this->lastUpdateTime_ = currentTime; 
}

//////////////////////////////////////////////////
void TrickExecutorPlugin::ApplyRotorThrusts(const std::vector<double> &thrusts)
{
  if (thrusts.size() != this->rotorCount_ || this->rotorLinks_.size() != this->rotorCount_) {
    return;
  }
  for (unsigned int i = 0; i < this->rotorCount_; ++i) {
    if (this->rotorLinks_[i]) { 
        double T = thrusts[i];
        ignition::math::Vector3d forceLocal(0, 0, T); 
        this->rotorLinks_[i]->AddRelativeForce(forceLocal);
    }
  }
}

//////////////////////////////////////////////////
void TrickExecutorPlugin::ApplyDrag()
{
  if (!this->hullLink_) return;
  ignition::math::Vector3d vel = this->hullLink_->WorldLinearVel();
  double speed = vel.Length();
  if (speed > 1e-3 && this->dragCoeff_ > 1e-6) { 
    ignition::math::Vector3d drag_force = vel.Normalized() * (-this->dragCoeff_ * speed * speed);
    this->hullLink_->AddForce(drag_force); 
  }
  ignition::math::Vector3d angVelWorld = this->hullLink_->WorldAngularVel(); 
  double angSpeed = angVelWorld.Length();
  if (angSpeed > 1e-3 && this->dragCoeff_ > 1e-6) { 
      double angularDragCoeffFactor = 0.01; 
      double actualAngularDragCoeff = this->dragCoeff_ * angularDragCoeffFactor;
      ignition::math::Vector3d angular_drag_torque_world = angVelWorld.Normalized() * (-actualAngularDragCoeff * angSpeed * angSpeed);
      this->hullLink_->AddTorque(angular_drag_torque_world); 
  }
}

//////////////////////////////////////////////////
double TrickExecutorPlugin::GetModelMass() const
{
  if (!this->model_) return 0.0;
  double total_mass = 0.0;
  for (auto const &link : this->model_->GetLinks()) {
    if (link) { total_mass += link->GetInertial()->Mass(); }
  }
  return total_mass;
}

//////////////////////////////////////////////////
void TrickExecutorPlugin::ZmqServerLoop()
{
  gzmsg << "[TrickExecutorPlugin] ZMQ server loop started for model " << (this->model_ ? this->model_->GetName() : "N/A") << std::endl;
  while (this->zmqThreadRunning_.load()) {
    zmq::message_t reqMsg;
    auto recvd_len = this->zmqSocket_.recv(reqMsg, zmq::recv_flags::dontwait);

    if (recvd_len && recvd_len.value() > 0) { 
        std::string reqStr(static_cast<char*>(reqMsg.data()), recvd_len.value());
        try {
            this->HandleJsonRequest(reqStr);
        }
        catch (const json::parse_error& e) {
            gzerr << "[TrickExecutorPlugin] JSON parse error: " << e.what() << " at byte " << e.byte << "\nInput (first 200 chars): " << reqStr.substr(0,200) << "\n";
            json errJ; errJ["status"]  = "error"; errJ["error_message"] = "JSON parse error: " + std::string(e.what());
            this->zmqSocket_.send(zmq::buffer(errJ.dump()), zmq::send_flags::none);
        }
        catch (const std::exception &ex) {
            gzerr << "[TrickExecutorPlugin] JSON handling/runtime error: " << ex.what() << "\n";
            json errJ; errJ["status"]  = "error"; errJ["error_message"] = ex.what();
            try { 
                json parsed_for_id = json::parse(reqStr);
                if (parsed_for_id.contains("request_id")) errJ["request_id"] = parsed_for_id["request_id"];
            } catch(...) { /* ignore */ }
            this->zmqSocket_.send(zmq::buffer(errJ.dump()), zmq::send_flags::none);
        }
    } else { 
        if (!recvd_len && errno != EAGAIN) { 
            gzerr << "[TrickExecutorPlugin] ZMQ recv error: " << zmq_strerror(errno) << std::endl;
            if (errno == ETERM) { 
                 this->zmqThreadRunning_.store(false); 
            }
        } 
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
    }
  }
  gzmsg << "[TrickExecutorPlugin] ZMQ server loop ended for model " << (this->model_ ? this->model_->GetName() : "N/A") << std::endl;
}

//////////////////////////////////////////////////
void TrickExecutorPlugin::HandleJsonRequest(const std::string &requestStr)
{
  json j = json::parse(requestStr); 

  if (!j.contains("request_id") || !j.contains("initial_state") || !j.contains("rotor_thrusts_info") ||
      !j["rotor_thrusts_info"].contains("rotor_thrusts") || !j["rotor_thrusts_info"].contains("duration") ||
      !j["rotor_thrusts_info"].contains("time_step")) {
    throw std::runtime_error("Missing required fields in request: request_id, initial_state, or rotor_thrusts_info (with subfields rotor_thrusts, duration, time_step)");
  }
  std::string requestId = j.at("request_id").get<std::string>();

  if (j.contains("drone_name")) {
    std::string droneName = j.at("drone_name").get<std::string>();
    if (this->model_ && droneName != this->model_->GetName()) {
      gzwarn << "[TrickExecutorPlugin] Requested drone '" << droneName
            << "', but plugin is on '" << this->model_->GetName() << "'\n";
    }
  }

  this->SetInitialState(j.at("initial_state"));

  const json &thrustsInfo = j.at("rotor_thrusts_info");
  const json &thrustArr = thrustsInfo.at("rotor_thrusts");
  this->rotorThrustSequence_.clear();
  this->rotorThrustSequence_.reserve(thrustArr.size());

  for (const auto &entry : thrustArr) {
    if (!entry.is_array() || entry.size() != (1u + this->rotorCount_)) { 
      std::string error_msg = "Each rotor_thrusts entry must be an array of [time_offset, T0, ..., T" + 
                              std::to_string(this->rotorCount_ -1) + "]. Received array of size " + std::to_string(entry.size());
      throw std::runtime_error(error_msg);
    }
    double tOff = entry.at(0).get<double>();
    std::vector<double> thrustVec;
    thrustVec.reserve(this->rotorCount_);
    for (unsigned int i = 0; i < this->rotorCount_; ++i) {
      thrustVec.push_back(entry.at(1 + i).get<double>());
    }
    this->rotorThrustSequence_.push_back({tOff, thrustVec});
  }

  if (this->rotorThrustSequence_.empty() && thrustsInfo.at("duration").get<double>() > 1e-3) { 
    throw std::runtime_error("rotor_thrusts sequence is empty but duration is > 0");
  }
  
  this->sequenceDuration_ = thrustsInfo.at("duration").get<double>();
  this->sequenceStartTime_ = this->world_->SimTime().Double(); 
  this->nextThrustIndex_ = 0;
  this->capturedFrames_.clear();
  // Check if world_ and Physics() are valid before calling GetMaxStepSize()
  double maxStepSize = 0.001; // Default if cannot get from physics engine
  if (this->world_ && this->world_->Physics()) {
      maxStepSize = this->world_->Physics()->GetMaxStepSize();
  }
  if (maxStepSize > 1e-9) { // Avoid division by zero
    this->capturedFrames_.reserve(static_cast<size_t>(this->sequenceDuration_ / maxStepSize) + 100);
  } else {
     this->capturedFrames_.reserve(static_cast<size_t>(this->sequenceDuration_ / 0.001) + 100); 
  }
  this->currentThrusts_.assign(this->rotorCount_, 0.0); 

  this->runTestMode_ = false; 
  this->sequenceActive_.store(true); 

  common::Time loopStartTime = this->world_->SimTime();
  double maxWaitTime = this->sequenceDuration_ + 5.0; 

  while (this->sequenceActive_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
    if ((this->world_->SimTime() - loopStartTime).Double() > maxWaitTime) {
        gzerr << "[TrickExecutorPlugin] Timeout waiting for sequence to complete for request_id: " << requestId << std::endl;
        this->sequenceActive_.store(false); 
        json errJ;
        errJ["status"] = "error";
        errJ["request_id"] = requestId;
        errJ["error_message"] = "Simulation sequence timeout.";
        std::string errStr = errJ.dump();
        this->zmqSocket_.send(zmq::buffer(errStr), zmq::send_flags::none);
        return; 
    }
  }

  json replyJ;
  replyJ["status"]      = "success";
  replyJ["request_id"]  = requestId;
  replyJ["trajectory_frames"]  = json::array(); 

  for (const Frame &f : this->capturedFrames_) {
    json fJ;
    fJ["t"]       = f.t;
    fJ["pos"]     = { f.pos.X(),   f.pos.Y(),   f.pos.Z()   };
    fJ["orient"]  = { f.orient.W(), f.orient.X(), f.orient.Y(), f.orient.Z() }; 
    fJ["lin_vel"] = { f.linVel.X(),f.linVel.Y(),f.linVel.Z() };
    fJ["ang_vel"] = { f.angVel.X(),f.angVel.Y(),f.angVel.Z() };
    replyJ["trajectory_frames"].push_back(fJ);
  }

  std::string replyStr = replyJ.dump();
  this->zmqSocket_.send(zmq::buffer(replyStr), zmq::send_flags::none);
  gzmsg << "[TrickExecutorPlugin] Sent ZMQ response for " << requestId << " (" << this->capturedFrames_.size() << " frames)." << std::endl;
}

//////////////////////////////////////////////////
void TrickExecutorPlugin::SetInitialState(const json &initState)
{
  if (!this->model_ || !this->hullLink_) throw std::runtime_error("SetInitialState: model or hullLink null.");
  if (!initState.contains("pos") || !initState.contains("orient") || !initState.contains("lin_vel") || !initState.contains("ang_vel")) {
    throw std::runtime_error("initial_state missing keys.");
  }
  auto p = initState.at("pos"), o = initState.at("orient"), l = initState.at("lin_vel"), a = initState.at("ang_vel");
  if (!p.is_array() || p.size()!=3 || !o.is_array() || o.size()!=4 || !l.is_array() || l.size()!=3 || !a.is_array() || a.size()!=3) {
    throw std::runtime_error("initial_state array sizes incorrect.");
  }
  ignition::math::Pose3 pose( 
      ignition::math::Vector3d(p.at(0).get<double>(), p.at(1).get<double>(), p.at(2).get<double>()),
      ignition::math::Quaterniond(o.at(3).get<double>(), o.at(0).get<double>(), o.at(1).get<double>(), o.at(2).get<double>())
  );
  ignition::math::Vector3d linVel(l.at(0).get<double>(), l.at(1).get<double>(), l.at(2).get<double>());
  ignition::math::Vector3d angVel(a.at(0).get<double>(), a.at(1).get<double>(), a.at(2).get<double>());

  this->model_->SetWorldPose(pose); 
  this->model_->SetLinearVel(linVel);
  this->model_->SetAngularVel(angVel);
  if (this->world_ && this->world_->Physics()) { // Ensure physics is available before resetting states
    this->model_->ResetPhysicsStates(); 
  }


  gzmsg << "[TrickExecutorPlugin] Set initial state for " << this->model_->GetName()
        << ": Pos(" << pose.Pos().X() << "," << pose.Pos().Y() << "," << pose.Pos().Z() << ")"
        << " Orient(" << pose.Rot().W() << "," << pose.Rot().X() << "," << pose.Rot().Y() << "," << pose.Rot().Z() << ")" << std::endl;
}

GZ_REGISTER_MODEL_PLUGIN(TrickExecutorPlugin)