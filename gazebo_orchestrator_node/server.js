const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const zmq = require('zeromq');
const { v4: uuidv4 } = require('uuid');
const fs = require('fs');
const path = require('path');
const csv = require('fast-csv');
const xml2js = require('xml2js');

const PORT = process.env.NODE_ORCHESTRATOR_PORT || 28642;
const GAZEBO_ZMQ_ENDPOINT = process.env.GAZEBO_ZMQ_ENDPOINT || 'tcp://localhost:5555';
const TRICK_SEQUENCES_DIR = path.join(__dirname, 'trick_sequences');
const TRAJECTORY_LOG_DIR = process.env.TRAJECTORY_LOG_DIR || path.join(__dirname, '..', 'trajectory_logs');

// Paths for world and drone spec files - ensure these are correct for your container setup
const WORLD_SDF_PATH = process.env.GAZEBO_WORLD_FILE_PATH || path.join(__dirname, '..', 'world.sdf');
const DRONE_SPECS_PATH = process.env.DRONE_SPECS_FILE_PATH || path.join(__dirname, '..', 'drone_params.json');


if (!fs.existsSync(TRAJECTORY_LOG_DIR)) {
    try {
        fs.mkdirSync(TRAJECTORY_LOG_DIR, { recursive: true });
        console.log(`Created trajectory log directory: ${TRAJECTORY_LOG_DIR}`);
    } catch (err) {
        console.error(`Error creating trajectory log directory ${TRAJECTORY_LOG_DIR}:`, err);
    }
}

const app = express();
app.use(express.json({ limit: '5mb' }));
const server = http.createServer(app);
const wss = new WebSocket.Server({ server, path: '/ws_gazebo_orchestrator' });

let gazeboRequester = new zmq.Request(); // Changed to let for potential re-creation
let gazeboConnected = false;
let isConnectingToGazebo = false;

async function connectToGazebo() {
    if (isConnectingToGazebo || gazeboConnected) return;
    isConnectingToGazebo = true;
    try {
        console.log(`Orchestrator: Attempting to connect to Gazebo ZMQ REP server at ${GAZEBO_ZMQ_ENDPOINT}...`);
        if (gazeboRequester && typeof gazeboRequester.close === 'function' && !gazeboRequester.closed) {
            try { 
                console.log("Orchestrator: Closing existing ZMQ requester before reconnecting...");
                await gazeboRequester.close();
            } catch (closeErr) {
                console.warn("Orchestrator: Error closing ZMQ requester during reconnect attempt:", closeErr.message);
            }
        }
        gazeboRequester = new zmq.Request(); // Re-initialize
        gazeboRequester.receiveTimeout = 7000;
        gazeboRequester.sendTimeout = 7000;
        await gazeboRequester.connect(GAZEBO_ZMQ_ENDPOINT);
        gazeboConnected = true;
        isConnectingToGazebo = false;
        console.log(`Orchestrator: Successfully connected to Gazebo ZMQ REP server.`);
    } catch (err) {
        console.error(`Orchestrator: Failed to connect to Gazebo ZMQ: ${err.message}. Retrying in 5s...`);
        gazeboConnected = false;
        isConnectingToGazebo = false;
        setTimeout(connectToGazebo, 5000);
    }
}
connectToGazebo();

const activeGazeboRequests = new Map();

function loadTrickDefinition(trickName) {
    const filepath = path.join(TRICK_SEQUENCES_DIR, `${trickName}.json`);
    try {
        if (fs.existsSync(filepath)) {
            const data = fs.readFileSync(filepath, 'utf8');
            return JSON.parse(data);
        }
        console.warn(`Trick definition file not found: ${filepath}`);
    } catch (err) {
        console.error(`Error loading trick definition ${trickName} from ${filepath}:`, err);
    }
    return null;
}

function logTrajectoryToCsv(robloxRequestId, droneName, trickType, trajectoryFrames) {
    if (!trajectoryFrames || trajectoryFrames.length === 0) {
        console.log("No trajectory frames to log.");
        return;
    }
    const timestamp = new Date().toISOString().replace(/:/g, '-').replace(/\..+/, '');
    const filename = `${timestamp}_${droneName}_${trickType}_${robloxRequestId}.csv`;
    const filepath = path.join(TRAJECTORY_LOG_DIR, filename);

    const csvStream = csv.format({ headers: true, writeHeaders: true });
    const writeStream = fs.createWriteStream(filepath);
    
    csvStream.pipe(writeStream)
        .on('finish', () => console.log(`Trajectory logged to ${filepath}`))
        .on('error', (err) => console.error(`Error writing CSV ${filepath}:`, err));

    trajectoryFrames.forEach(frame => {
        const row = { sim_time_offset: frame.t != null ? frame.t.toFixed(3) : 'N/A' };
        if (frame.pos) { row.pos_x = frame.pos[0]; row.pos_y = frame.pos[1]; row.pos_z = frame.pos[2]; }
        if (frame.orient) { row.orient_x = frame.orient[0]; row.orient_y = frame.orient[1]; row.orient_z = frame.orient[2]; row.orient_w = frame.orient[3]; }
        if (frame.lin_vel) { row.lin_vel_x = frame.lin_vel[0]; row.lin_vel_y = frame.lin_vel[1]; row.lin_vel_z = frame.lin_vel[2]; }
        if (frame.ang_vel) { row.ang_vel_x = frame.ang_vel[0]; row.ang_vel_y = frame.ang_vel[1]; row.ang_vel_z = frame.ang_vel[2]; }
        csvStream.write(row);
    });
    csvStream.end();
}

function generateFullThrustSequence(trickDef, droneMassKgOverride, rollDirection = "RIGHT") {
    const params = { ...trickDef.parameters };
    const GRAVITY_ACCEL = 9.80665;

    let actualDroneMassKg = 0.050; 
    if (typeof droneMassKgOverride === 'number' && droneMassKgOverride > 0) {
        actualDroneMassKg = droneMassKgOverride;
    }

    params.T_HOVER_ACTUAL = (actualDroneMassKg * GRAVITY_ACCEL) / 4.0;
    params.T_PUNCH_ACTUAL = params.T_HOVER_ACTUAL * params.PUNCH_FACTOR;
    const rollDifferentialMagnitude = params.T_HOVER_ACTUAL * params.ROLL_DIFFERENTIAL_FACTOR;
    params.T_ROLL_LOW_ACTUAL = Math.max(0, params.T_HOVER_ACTUAL - rollDifferentialMagnitude);
    params.T_ROLL_HIGH_ACTUAL = params.T_HOVER_ACTUAL + rollDifferentialMagnitude;
    params.T_MAINTAIN_COLLECTIVE_ACTUAL = params.T_HOVER_ACTUAL * params.MAINTAIN_COLLECTIVE_FACTOR;

    const getThrustsForPhase = (phaseName) => {
        let t0 = params.T_HOVER_ACTUAL, t1 = params.T_HOVER_ACTUAL, t2 = params.T_HOVER_ACTUAL, t3 = params.T_HOVER_ACTUAL;
        switch (phaseName) {
            case "HOVER": case "FINAL_HOVER": case "STABILIZE_HOVER":
                break;
            case "PUNCH":
                t0 = t1 = t2 = t3 = params.T_PUNCH_ACTUAL;
                break;
            case "INITIATE_ROLL": case "MAX_ROLL_INPUT": case "MAINTAIN_MAX_ROLL_INPUT": case "RECOVER_ROLL":
                t0 = params.T_MAINTAIN_COLLECTIVE_ACTUAL; 
                t2 = params.T_MAINTAIN_COLLECTIVE_ACTUAL; 
                if (rollDirection === "RIGHT") {
                    t1 = params.T_ROLL_LOW_ACTUAL;  
                    t3 = params.T_ROLL_HIGH_ACTUAL; 
                } else { 
                    t1 = params.T_ROLL_HIGH_ACTUAL; 
                    t3 = params.T_ROLL_LOW_ACTUAL;  
                }
                if (phaseName === "INITIATE_ROLL" || phaseName === "RECOVER_ROLL") {
                    const midHoverMaintain = (params.T_HOVER_ACTUAL + params.T_MAINTAIN_COLLECTIVE_ACTUAL) / 2;
                    t0 = midHoverMaintain;
                    t2 = midHoverMaintain;
                    if (rollDirection === "RIGHT") {
                        t1 = (params.T_ROLL_LOW_ACTUAL + params.T_HOVER_ACTUAL) / 2;
                        t3 = (params.T_ROLL_HIGH_ACTUAL + params.T_HOVER_ACTUAL) / 2;
                    } else {
                        t1 = (params.T_ROLL_HIGH_ACTUAL + params.T_HOVER_ACTUAL) / 2;
                        t3 = (params.T_ROLL_LOW_ACTUAL + params.T_HOVER_ACTUAL) / 2;
                    }
                }
                break;
            default: console.warn(`Unknown phase name: ${phaseName}, defaulting to hover.`);
        }
        return [ parseFloat(t0.toFixed(4)), parseFloat(t1.toFixed(4)), parseFloat(t2.toFixed(4)), parseFloat(t3.toFixed(4)) ];
    };

    const fullSequence = [];
    const keyframes = trickDef.keyframes;
    const timeStep = trickDef.time_step;

    for (let i = 0; i < keyframes.length; i++) {
        const currentKeyframe = keyframes[i];
        const currentTargetThrusts = getThrustsForPhase(currentKeyframe.phase);
        if (i === 0) {
            if (currentKeyframe.time > 0) {
                const initialHoverThrusts = getThrustsForPhase("HOVER");
                for (let t = 0; t < currentKeyframe.time; t += timeStep) {
                    fullSequence.push([parseFloat(t.toFixed(3)), ...initialHoverThrusts]);
                }
            }
            fullSequence.push([parseFloat(currentKeyframe.time.toFixed(3)), ...currentTargetThrusts]);
        } else {
            const prevKeyframe = keyframes[i - 1];
            const prevActualThrusts = fullSequence.length > 0 ? fullSequence[fullSequence.length - 1].slice(1) : getThrustsForPhase(prevKeyframe.phase);
            const timeDiff = currentKeyframe.time - prevKeyframe.time;
            const numStepsInSegment = Math.max(1, Math.round(timeDiff / timeStep));
            for (let step = 1; step <= numStepsInSegment; step++) {
                let t = prevKeyframe.time + step * timeStep;
                if (step < numStepsInSegment) {
                     t = Math.min(t, currentKeyframe.time - timeStep/10); 
                } else {
                     t = currentKeyframe.time; 
                }
                const interpFactor = timeDiff === 0 ? 1 : (t - prevKeyframe.time) / timeDiff; // Avoid division by zero
                const interpolatedThrusts = prevActualThrusts.map((prevT, motorIdx) =>
                    parseFloat((prevT + (currentTargetThrusts[motorIdx] - prevT) * interpFactor).toFixed(4))
                );
                fullSequence.push([parseFloat(t.toFixed(3)), ...interpolatedThrusts]);
            }
        }
    }
    const lastTimeInGeneratedSeq = fullSequence.length > 0 ? fullSequence[fullSequence.length - 1][0] : 0;
    if (lastTimeInGeneratedSeq < trickDef.duration) {
        const finalThrusts = getThrustsForPhase(keyframes[keyframes.length - 1].phase);
        for (let t = lastTimeInGeneratedSeq + timeStep; t <= trickDef.duration + timeStep/2; t += timeStep) {
            if (t > trickDef.duration + timeStep/2) break; 
            fullSequence.push([parseFloat(t.toFixed(3)), ...finalThrusts]);
        }
    }
    const finalFilteredSequence = fullSequence.filter(frame => frame[0] <= trickDef.duration + 1e-5); // Add small tolerance for float comparison
    return { duration: trickDef.duration, time_step: timeStep, rotor_thrusts: finalFilteredSequence };
}

function parseSdfPose(poseStr) {
    if (!poseStr || typeof poseStr !== 'string') return { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
    const parts = poseStr.split(/\s+/).map(Number);
    return {
        x: parts[0] || 0, y: parts[1] || 0, z: parts[2] || 0,
        roll: parts[3] || 0, pitch: parts[4] || 0, yaw: parts[5] || 0
    };
}

function parseSdfSize(sizeStr) {
    if (!sizeStr || typeof sizeStr !== 'string') return { x: 0, y: 0, z: 0 };
    const parts = sizeStr.split(/\s+/).map(Number);
    return { x: parts[0] || 0, y: parts[1] || 0, z: parts[2] || 0 };
}

function getMaterialColor(material) {
    if (!material || !material[0] || !material[0].script || !material[0].script[0] || !material[0].script[0].name) {
        return 'Gray';
    }
    const materialName = material[0].script[0].name[0];
    if (typeof materialName === 'string') {
        const parts = materialName.split('/');
        return parts.length > 1 ? parts[1] : parts[0];
    }
    return 'Gray';
}

app.get('/world_obstacles', async (req, res) => {
    try {
        console.log(`Orchestrator: Received request for /world_obstacles. Reading from ${WORLD_SDF_PATH}`);
        if (!fs.existsSync(WORLD_SDF_PATH)) {
            console.error(`Orchestrator: World SDF file not found at ${WORLD_SDF_PATH}`);
            return res.status(404).json({ error: 'World SDF file not found on server.' });
        }

        const sdfData = fs.readFileSync(WORLD_SDF_PATH, 'utf8');
        const parser = new xml2js.Parser();
        const result = await parser.parseStringPromise(sdfData);

        const obstacles = [];
        if (result.sdf && result.sdf.world && result.sdf.world[0] && result.sdf.world[0].model) {
            const models = result.sdf.world[0].model;
            models.forEach(model => {
                const modelName = model.$.name;
                if (modelName === 'ground_plane' || modelName === 'sun' || modelName.includes('drone') || modelName === 'active_drone') {
                    return;
                }

                const modelPoseStr = model.pose && model.pose[0];
                const modelPose = parseSdfPose(modelPoseStr);

                if (model.link) {
                    model.link.forEach(link => {
                        const linkName = link.$.name;
                        const linkPoseStr = link.pose && link.pose[0];
                        const linkRelativePose = parseSdfPose(linkPoseStr);
                        
                        const obstaclePose = {
                            x: modelPose.x + linkRelativePose.x,
                            y: modelPose.y + linkRelativePose.y,
                            z: modelPose.z + linkRelativePose.z,
                            roll: modelPose.roll + linkRelativePose.roll,
                            pitch: modelPose.pitch + linkRelativePose.pitch,
                            yaw: modelPose.yaw + linkRelativePose.yaw
                        };

                        let geometryType = 'unknown';
                        let geometrySize = { x: 1, y: 1, z: 1 };
                        let materialColor = 'Gray';

                        if (link.collision && link.collision[0] && link.collision[0].geometry && link.collision[0].geometry[0]) {
                            const geom = link.collision[0].geometry[0];
                            if (geom.box && geom.box[0] && geom.box[0].size && geom.box[0].size[0]) {
                                geometryType = 'box';
                                geometrySize = parseSdfSize(geom.box[0].size[0]);
                            } else if (geom.cylinder && geom.cylinder[0]) {
                                geometryType = 'cylinder';
                                geometrySize = {
                                    radius: parseFloat(geom.cylinder[0].radius[0]),
                                    length: parseFloat(geom.cylinder[0].length[0])
                                };
                            } else if (geom.sphere && geom.sphere[0]) {
                                geometryType = 'sphere';
                                geometrySize = {
                                    radius: parseFloat(geom.sphere[0].radius[0])
                                };
                            }
                        }
                        
                        if (link.visual && link.visual[0] && link.visual[0].material) {
                           materialColor = getMaterialColor(link.visual[0].material);
                        }

                        obstacles.push({
                            name: `${modelName}_${linkName}`,
                            model_name: modelName,
                            link_name: linkName,
                            type: geometryType,
                            pose: obstaclePose,
                            size: geometrySize,
                            color: materialColor,
                            is_static: (model.static && model.static[0] === 'true') || false
                        });
                    });
                }
            });
        }
        res.json({ obstacles });
    } catch (error) {
        console.error("Orchestrator: Error processing /world_obstacles request:", error);
        res.status(500).json({ error: 'Failed to parse world SDF or extract obstacles.', details: error.message });
    }
});

app.get('/drone_specs', (req, res) => {
    try {
        console.log(`Orchestrator: Received request for /drone_specs. Reading from ${DRONE_SPECS_PATH}`);
        if (fs.existsSync(DRONE_SPECS_PATH)) {
            const droneData = fs.readFileSync(DRONE_SPECS_PATH, 'utf8');
            // The file content is already JSON, so just send it as such.
            // express.json() middleware only parses request bodies, not for sending responses.
            res.setHeader('Content-Type', 'application/json');
            res.status(200).send(droneData);
        } else {
            console.error("Orchestrator: drone_params.json not found at", DRONE_SPECS_PATH);
            res.status(404).json({ error: 'Drone specification file not found.' });
        }
    } catch (error) {
        console.error("Orchestrator: Error serving /drone_specs:", error);
        res.status(500).json({ error: 'Failed to read drone specifications.', details: error.message });
    }
});


wss.on('connection', (wsClient, req) => {
    const clientIp = req.socket.remoteAddress || req.headers['x-forwarded-for'];
    const internalClientId = uuidv4();
    console.log(`Orchestrator: RoSocket Backend Proxy ${internalClientId} connected from ${clientIp}`);

    wsClient.on('message', async (message) => {
        let parsedMessage;
        try {
            parsedMessage = JSON.parse(message.toString());
        } catch (error) {
            console.error(`Orchestrator: Failed to parse message from RoSocket Backend ${internalClientId}:`, message.toString(), error);
            wsClient.send(JSON.stringify({ type: 'ERROR', payload: { message: 'Invalid JSON message from orchestrator' } }));
            return;
        }
        console.log(`Orchestrator: Received from RoSocket Backend ${internalClientId} (Roblox Client ${parsedMessage.payload?.roblox_client_id || 'unknown'}): Type: ${parsedMessage.type}`);

        if (parsedMessage.type === 'REQUEST_TRICK') {
            const { drone_name, trick_type, initial_state, roblox_client_id, original_request_id, direction } = parsedMessage.payload;

            if (!drone_name || !trick_type || !initial_state || !roblox_client_id || !original_request_id) {
                wsClient.send(JSON.stringify({ type: 'TRICK_ERROR', payload: { original_request_id, message: 'Missing fields in REQUEST_TRICK' } }));
                return;
            }
            if (!gazeboConnected) {
                console.warn("Orchestrator: Gazebo not connected, cannot process trick request.");
                wsClient.send(JSON.stringify({ type: 'TRICK_ERROR', payload: { original_request_id, message: 'Gazebo simulation service temporarily unavailable.' } }));
                return;
            }

            const trickDefinition = loadTrickDefinition(trick_type);
            if (!trickDefinition) {
                wsClient.send(JSON.stringify({ type: 'TRICK_ERROR', payload: { original_request_id, message: `Trick type '${trick_type}' not found.` } }));
                return;
            }

            const droneMassKg = initial_state.mass_kg; 
            const rollDirection = (trick_type === "barrel_roll" && direction) ? direction.toUpperCase() : "RIGHT";
            const processedSequenceInfo = generateFullThrustSequence(trickDefinition, droneMassKg, rollDirection);

            const gazeboSystemRequestId = `gz_sys_${uuidv4()}`;
            activeGazeboRequests.set(gazeboSystemRequestId, { wsClient, original_request_id, drone_name, trick_type, roblox_client_id });


            const gazeboCommand = {
                command: "execute_rotor_thrust_sequence",
                request_id: gazeboSystemRequestId,
                drone_name: drone_name,
                initial_state: initial_state,
                rotor_thrusts_info: processedSequenceInfo
            };

            console.log(`Orchestrator: Sending command to Gazebo (ID: ${gazeboSystemRequestId}) for Roblox request ${original_request_id}. Sequence steps: ${processedSequenceInfo.rotor_thrusts.length}`);
            wsClient.send(JSON.stringify({ type: 'TRICK_PENDING', payload: { original_request_id, system_request_id: gazeboSystemRequestId, roblox_client_id: roblox_client_id } }));


            try {
                if(gazeboRequester.pending) { 
                    console.warn("Orchestrator: ZMQ Requester was pending. Previous request might have timed out without reply.");
                }
                await gazeboRequester.send(JSON.stringify(gazeboCommand));
                const [responseBuffer] = await gazeboRequester.receive();
                const responseFromGazebo = JSON.parse(responseBuffer.toString());
                
                const requestInfo = activeGazeboRequests.get(responseFromGazebo.request_id);
                activeGazeboRequests.delete(responseFromGazebo.request_id);

                if (!requestInfo) {
                    console.error("Orchestrator: Received Gazebo response for an unknown or timed-out request:", responseFromGazebo.request_id);
                    return; 
                }
                console.log(`Orchestrator: Received response from Gazebo for ${responseFromGazebo.request_id}`);
                
                const responsePayload = {
                    original_request_id: requestInfo.original_request_id,
                    burst_id: responseFromGazebo.request_id,
                    roblox_client_id: requestInfo.roblox_client_id // Ensure roblox_client_id is in response
                };

                if (responseFromGazebo.status === "success") {
                    const trajectoryFrames = responseFromGazebo.trajectory_frames || [];
                    logTrajectoryToCsv(requestInfo.original_request_id, requestInfo.drone_name, requestInfo.trick_type, trajectoryFrames);

                    requestInfo.wsClient.send(JSON.stringify({
                        type: 'TRICK_DATA_START',
                        payload: { ...responsePayload, total_frames: trajectoryFrames.length, trick_duration_estimate: processedSequenceInfo.duration }
                    }));
                    const chunkSize = 20; 
                    for (let i = 0; i < trajectoryFrames.length; i += chunkSize) {
                        const chunk = trajectoryFrames.slice(i, i + chunkSize);
                        requestInfo.wsClient.send(JSON.stringify({
                            type: 'TRICK_DATA_CHUNK',
                            payload: { ...responsePayload, chunk_id: Math.floor(i / chunkSize) + 1, frames: chunk }
                        }));
                    }
                    requestInfo.wsClient.send(JSON.stringify({
                        type: 'TRICK_DATA_END',
                        payload: responsePayload
                    }));
                } else {
                     const errorMsg = responseFromGazebo.error_message || "Unknown error from Gazebo";
                     console.error(`Orchestrator: Error from Gazebo for ${responseFromGazebo.request_id}: ${errorMsg}`);
                     requestInfo.wsClient.send(JSON.stringify({ type: 'TRICK_ERROR', payload: { ...responsePayload, message: `Gazebo simulation error: ${errorMsg}` } }));
                }
            } catch (err) {
                console.error(`Orchestrator: Error during Gazebo ZMQ communication for ${gazeboSystemRequestId}: ${err.message}, Code: ${err.code}, Errno: ${err.errno}`);
                const requestInfoOnError = activeGazeboRequests.get(gazeboSystemRequestId);
                if (requestInfoOnError) {
                    requestInfoOnError.wsClient.send(JSON.stringify({ type: 'TRICK_ERROR', payload: { 
                        original_request_id: requestInfoOnError.original_request_id, 
                        burst_id: gazeboSystemRequestId, 
                        roblox_client_id: requestInfoOnError.roblox_client_id,
                        message: `Gazebo communication error: ${err.message}` 
                    } }));
                    activeGazeboRequests.delete(gazeboSystemRequestId);
                }
                // More robust ZMQ error handling: recreate socket and reconnect
                if (!isConnectingToGazebo) { // Avoid concurrent connection attempts
                    console.warn("Orchestrator: ZMQ error detected. Attempting to reset Gazebo connection.");
                    gazeboConnected = false; // Mark as disconnected
                    connectToGazebo(); // Attempt to reconnect
                }
            }
        } else if (parsedMessage.type === 'HEARTBEAT') {
            wsClient.send(JSON.stringify({ type: 'HEARTBEAT_ACK', payload: { timestamp: Date.now(), roblox_client_id: parsedMessage.payload?.roblox_client_id } }));
        } else {
             console.warn(`Orchestrator: Unknown message type from RoSocket Backend ${internalClientId}: ${parsedMessage.type}`);
        }
    });
    wsClient.on('close', () => {
        console.log(`Orchestrator: RoSocket Backend Proxy ${internalClientId} disconnected`);
        activeGazeboRequests.forEach((value, key) => {
            if (value.wsClient === wsClient) {
                activeGazeboRequests.delete(key);
                console.log(`Cleaned up pending Gazebo request ${key} due to RoSocket Backend disconnect.`);
            }
        });
    });
    wsClient.on('error', (error) => {
        console.error(`Orchestrator: WebSocket error for RoSocket Backend Proxy ${internalClientId}:`, error);
    });
});

app.get('/health', (req, res) => res.status(200).send('Gazebo Orchestrator is healthy'));
server.listen(PORT, () => {
    console.log(`Gazebo Orchestration Server listening on http://localhost:${PORT}`);
    console.log(`WebSocket endpoint for RoSocket Backend: ws://localhost:${PORT}/ws_gazebo_orchestrator`);
    console.log(`World obstacles endpoint: http://localhost:${PORT}/world_obstacles`);
    console.log(`Drone specs endpoint: http://localhost:${PORT}/drone_specs`);
});

async function gracefulShutdown() {
    console.log('Orchestrator: Initiating graceful shutdown...');
    gazeboConnected = false; 

    wss.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) client.terminate();
    });
    wss.close(() => console.log('WebSocket server closed.'));
    
    try {
        if (gazeboRequester && typeof gazeboRequester.close === 'function' && !gazeboRequester.closed) {
            console.log('Closing Gazebo ZMQ Requester...');
            await gazeboRequester.close();
            console.log('Gazebo ZMQ Requester closed.');
        } else {
            console.log('Gazebo ZMQ Requester already closed or not initialized for shutdown.');
        }
    } catch (e) { console.error('Error closing ZMQ Requester during shutdown:', e); }

    server.close(() => {
        console.log('HTTP server closed.');
        console.log('Orchestrator shutdown complete.');
        process.exit(0);
    });

    setTimeout(() => {
        console.error('Forcing shutdown due to timeout.');
        process.exit(1);
    }, 5000); 
}

process.on('SIGINT', gracefulShutdown);
process.on('SIGTERM', gracefulShutdown);