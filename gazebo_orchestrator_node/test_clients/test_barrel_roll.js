const WebSocket = require('ws');

const orchestratorWsUrl = process.env.ORCHESTRATOR_WS_URL || 'ws://localhost:28642/ws_gazebo_orchestrator';
const ws = new WebSocket(orchestratorWsUrl);

const TARGET_DRONE_NAME = process.argv[2] || "drone1"; // Pass drone name as arg: node test_barrel_roll.js drone2
const ROLL_DIRECTION = (process.argv[3] || "RIGHT").toUpperCase();

ws.on('open', function open() {
  console.log(`TestClient (Barrel Roll): Connected to Orchestrator at ${orchestratorWsUrl}.`);

  const clientReqId = `barrelRollTest_${TARGET_DRONE_NAME}_${Date.now()}`;
  const trickPayload = {
    type: "REQUEST_TRICK",
    payload: {
      roblox_client_id: "node_test_client_barrel_roll", // Can be any identifier
      original_request_id: clientReqId,
      drone_name: TARGET_DRONE_NAME,
      trick_type: "barrel_roll", // Must match a .json file in trick_sequences
      direction: ROLL_DIRECTION,
      initial_state: {
        pos: [TARGET_DRONE_NAME === "drone2" ? 1.0 : 0.0, 0.0, 0.5], // Start drone2 at x=1
        orient: [0.0, 0.0, 0.0, 1.0], // No rotation (X,Y,Z,W order for this payload)
        lin_vel: [0.0, 0.0, 0.0],
        ang_vel: [0.0, 0.0, 0.0],
        mass_kg: 0.05 // Example mass
      }
    }
  };
  console.log(`TestClient: Sending REQUEST_TRICK for ${TARGET_DRONE_NAME}, Direction: ${ROLL_DIRECTION}, ID: ${clientReqId}`);
  ws.send(JSON.stringify(trickPayload));
});

ws.on('message', function incoming(data) {
  const messageString = data.toString();
  console.log('TestClient: Received from Orchestrator: %s', messageString.substring(0, 300) + (messageString.length > 300 ? "..." : ""));
  try {
    const parsed = JSON.parse(messageString);
    if (parsed.type === 'TRICK_DATA_END' || parsed.type === 'TRICK_ERROR') {
        console.log(`TestClient: Trick sequence for ${parsed.payload?.original_request_id} finished or errored. Closing client.`);
        ws.close();
    }
  } catch(e) {
    // console.error("TestClient: Error parsing message or not JSON", e);
  }
});

ws.on('close', function close() {
  console.log('TestClient: Disconnected from Orchestrator.');
});

ws.on('error', function error(err) {
  console.error('TestClient: WebSocket error:', err.message);
});