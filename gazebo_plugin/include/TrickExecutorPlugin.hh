#ifndef TRICK_EXECUTOR_PLUGIN_HH
#define TRICK_EXECUTOR_PLUGIN_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>   // For WorldPtr
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>     // For common::Time

#include <ignition/math/Pose3.hh>    // For ignition::math::Pose3
#include <ignition/math/Vector3.hh>  // For ignition::math::Vector3d (typedef'd)
#include <ignition/math/Quaternion.hh>// For ignition::math::Quaterniond (typedef'd)
#include <ignition/math/PID.hh>      // For ignition::math::PID
#include <ignition/math/Angle.hh>    // For ignition::math::Angle

#include <zmq.hpp>
#include <nlohmann/json.hpp>

#include <thread>
#include <atomic>
#include <vector>
#include <string>

namespace gazebo
{
  class TrickExecutorPlugin : public ModelPlugin
  {
  public:
    TrickExecutorPlugin();
    virtual ~TrickExecutorPlugin();
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    void OnUpdate();

  private:
    void ZmqServerLoop();
    void HandleJsonRequest(const std::string &requestStr);
    void SetInitialState(const nlohmann::json &initState);
    void ApplyRotorThrusts(const std::vector<double> &thrusts);
    void ApplyDrag();
    double GetModelMass() const;

    // Structure to hold a single frame of trajectory data
    struct Frame
    {
      double                     t;       // Time offset from start of sequence
      ignition::math::Vector3d   pos;     // World-frame position
      ignition::math::Quaterniond orient; // World-frame orientation (W,X,Y,Z)
      ignition::math::Vector3d   linVel;  // World-frame linear velocity
      ignition::math::Vector3d   angVel;  // World-frame angular velocity
    };

  private:
    // Gazebo Pointers
    physics::ModelPtr               model_;
    physics::WorldPtr               world_;         // Pointer to the world
    physics::LinkPtr                hullLink_;      // Link for applying forces/torques directly
    std::vector<physics::LinkPtr>   rotorLinks_;    // Links for applying rotor thrusts

    // Plugin Parameters from SDF
    unsigned int                    rotorCount_;
    std::string                     rotorBaseName_;
    bool                            runTestMode_;
    double                          testModeRadius_;
    double                          testModeAngularVel_;
    double                          testModeAltitude_;
    double                          dragCoeff_;
    std::string                     zmqPort_;

    // ZMQ Networking
    zmq::context_t                  zmqContext_;
    zmq::socket_t                   zmqSocket_;
    std::thread                     zmqThread_;
    std::atomic<bool>               zmqThreadRunning_{false}; // Initialized

    // Sequence Execution State
    std::atomic<bool>                               sequenceActive_{false}; // Initialized
    double                                          sequenceStartTime_;
    double                                          sequenceDuration_;
    size_t                                          nextThrustIndex_;
    std::vector<std::pair<double,std::vector<double>>> rotorThrustSequence_;
    std::vector<double>                             currentThrusts_;
    std::vector<Frame>                              capturedFrames_;

    // Event Connection & Timing
    event::ConnectionPtr            updateConnection_;
    common::Time                    lastUpdateTime_; // For calculating dt

    // PID Controllers for Test Mode
    ignition::math::PID             pidX_;
    ignition::math::PID             pidY_;
    ignition::math::PID             pidZ_;
    ignition::math::PID             pidYaw_;
  };
}

#endif // TRICK_EXECUTOR_PLUGIN_HH