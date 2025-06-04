# Specification.md (Gazebo Orchestrator Node.js Repository Perspective)

**Project Title:** Gazebo On-Demand Trick Orchestration Service (Node.js)

**Version:** 3.0 (Parameterized Tricks, CSV Logging, Drone Config Endpoint)

**1. Overview & Purpose**

This document specifies the Node.js server responsible for orchestrating on-demand physics simulations within a Gazebo environment. It acts as a bridge between Roblox clients (communicating via a `RoSocket` backend proxy) and a custom Gazebo C++ plugin.
Key functions:
1.  Receive "Trick Requests" (including drone's initial state and desired trick) from Roblox.
2.  Process parameterized trick definitions (e.g., JSON files) to generate specific rotor thrust sequences.
3.  Command the Gazebo C++ plugin (via ZeroMQ) to simulate the trick using these thrust sequences, starting from the provided initial state. Gazebo may run this simulation faster than real-time.
4.  Receive the resulting high-frequency trajectory data (pose, velocity) from Gazebo.
5.  Log this trajectory data to CSV files for analysis.
6.  Transmit the trajectory data in bursts (respecting `RoSocket` characteristics) back to the requesting Roblox client.
7.  Provide an endpoint to accept custom drone parameter configurations from Roblox clients (initial implementation: acknowledge; future: trigger dynamic model generation).

**2. System Architecture**

```
Roblox Client (Luau)
      │ (HTTP via RoSocket Lua Lib)
      ▼
RoSocket Node.js Backend Proxy (Manages actual WebSocket to Orchestrator)
      │ (WebSocket: wss://<orchestrator_host>:<port>/ws_gazebo_orchestrator)
      ▼
Gazebo Orchestrator Node.js Server (this repository's deliverable)
      │ - WebSocket Server (`ws` path: /ws_gazebo_orchestrator) for RoSocket Backend
      │ - Processes trick definitions (parameterized JSONs)
      │ - ZeroMQ REQ client to Gazebo Plugin
      │ - Logs trajectories to CSV
      │
      │ (ZeroMQ REQ/REP: tcp://<gazebo_host>:<gazebo_zmq_port>)
      ▼
Gazebo Docker Container
      │ - gzserver with custom C++ TrickExecutorPlugin (ZeroMQ REP server)
```

**3. Functional Requirements**

**3.1. WebSocket Server (for RoSocket Backend Communication)**
    *   Listen for WebSocket connections on a configurable path (e.g., `/ws_gazebo_orchestrator`) and port (e.g., `28642`).
    *   Handle incoming messages (JSON format) from the RoSocket backend (which are originated by Roblox clients). Expected message types:
        *   `REQUEST_TRICK { payload: { roblox_client_id, original_request_id, drone_name, trick_type, initial_state: {pos, orient, lin_vel, ang_vel, mass?} } }`
        *   `HEARTBEAT`
    *   Send outgoing messages (JSON format) back to the RoSocket backend for relay to the Roblox client:
        *   `TRICK_PENDING { payload: { original_request_id, system_request_id } }`
        *   `TRICK_DATA_START { payload: { original_request_id, burst_id, total_frames, ... } }`
        *   `TRICK_DATA_CHUNK { payload: { original_request_id, burst_id, chunk_id, frames: [...] } }`
        *   `TRICK_DATA_END { payload: { original_request_id, burst_id } }`
        *   `TRICK_ERROR { payload: { original_request_id, burst_id?, message } }`
        *   `HEARTBEAT_ACK`

**3.2. Trick Definition Processing**
    *   Load parameterized trick definitions from local JSON files (e.g., `trick_sequences/barrel_roll_right.json`).
    *   These definitions include base parameters (e.g., `T_HOVER`) and a timed sequence structure using these parameters.
    *   On `REQUEST_TRICK`:
        *   Calculate actual thrust values by substituting parameters (e.g., `T_HOVER` can be derived from `initial_state.mass` if provided by Roblox client).
        *   Interpolate/expand the timed sequence into a flat list of `[sim_time_offset, T0, T1, T2, T3]` thrusts at the `time_step` specified in the trick definition.

**3.3. Gazebo Plugin Communication (ZeroMQ REQ/REP)**
    *   Act as a ZeroMQ REQ client, connecting to the Gazebo C++ plugin's REP server (e.g., `tcp://localhost:5555`).
    *   Send Gazebo commands (JSON format) for `execute_rotor_thrust_sequence`, including:
        *   `request_id` (unique for this orchestrator-Gazebo interaction).
        *   `drone_name` (e.g., "drone1").
        *   `initial_state` (pose, velocities).
        *   `rotor_thrusts_info` (the fully processed, flat sequence of timed rotor thrusts, duration, and time_step).
    *   Receive response from Gazebo containing the `status` and `trajectory_frames`. This is a blocking request-reply.

**3.4. Trajectory Data Logging**
    *   Upon successful receipt of `trajectory_frames` from Gazebo:
        *   Log the complete trajectory to a CSV file in a configurable directory (`TRAJECTORY_LOG_DIR`).
        *   Filename convention: `<timestamp>_<drone_name>_<trick_type>_<roblox_original_request_id>.csv`.
        *   CSV Headers: `sim_time_offset, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w, lin_vel_x, ..., ang_vel_z`.

**3.5. Error Handling & Resilience**
    *   Handle timeouts and errors in ZMQ communication with Gazebo.
    *   Handle WebSocket connection errors/disconnections from RoSocket Backend.
    *   Gracefully report errors back to the requesting Roblox client.

**4. Non-Functional Requirements**
    *   **Performance:** Process trick requests and relay data with minimal added latency. ZMQ communication with Gazebo should be fast.
    *   **Scalability:** Handle multiple concurrent WebSocket connections from RoSocket backends (representing multiple Roblox game servers/clients). The blocking ZMQ REQ/REP to Gazebo for each trick is a potential bottleneck for very high concurrency and might need a worker pool or async ZMQ pattern in future.
    *   **Configurability:** Ports, ZMQ endpoints, log directories should be configurable via environment variables.
    *   **Maintainability:** Clean, well-commented Node.js (JavaScript or TypeScript) code.

**5. Dependencies**
    *   `express` (HTTP framework)
    *   `ws` (WebSocket server implementation)
    *   `zeromq` (npm package for ZeroMQ)
    *   `uuid` (for generating unique IDs)
    *   `fast-csv` (for CSV writing)