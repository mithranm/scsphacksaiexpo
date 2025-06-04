#!/usr/bin/env bash
# ... (initial comments and log setup as before) ...

LOG_DIR="/home/vscode/logs"
GZ_SERVER_LOG="${LOG_DIR}/gzserver_script.log"
GZ_ACTUAL_SERVER_OUTPUT_LOG="${LOG_DIR}/gzserver_instance.log"

mkdir -p "${LOG_DIR}"
echo "--- Gazebo Server Startup Script (Focus on Install Dir) ---" > "${GZ_SERVER_LOG}"
echo "" > "${GZ_ACTUAL_SERVER_OUTPUT_LOG}"

exec 3>>"${GZ_SERVER_LOG}"
echo "Log FD 3 opened." >&3

date >&3
echo "User: $(whoami)" >&3
echo "PWD: $(pwd)" >&3
echo "Initial GAZEBO_PLUGIN_PATH: ${GAZEBO_PLUGIN_PATH}" >&3
echo "Initial GAZEBO_MODEL_PATH: ${GAZEBO_MODEL_PATH}" >&3
echo "Initial LD_LIBRARY_PATH: ${LD_LIBRARY_PATH}" >&3
echo "Shell: $SHELL, Shell opts: $-" >&3

echo "STEP 1: About to source Gazebo environment (/usr/share/gazebo/setup.sh)..." >&3
if [ -f "/usr/share/gazebo/setup.sh" ]; then
    source /usr/share/gazebo/setup.sh
    SOURCE_EXIT_CODE=$?
    echo "STEP 2: Sourced Gazebo's setup.sh. Exit code: ${SOURCE_EXIT_CODE}" >&3
    # ... (error check for SOURCE_EXIT_CODE) ...
    echo "GAZEBO_PLUGIN_PATH after Gazebo setup: ${GAZEBO_PLUGIN_PATH}" >&3
    echo "GAZEBO_MODEL_PATH after Gazebo setup: ${GAZEBO_MODEL_PATH}" >&3
    echo "LD_LIBRARY_PATH after Gazebo setup: ${LD_LIBRARY_PATH}" >&3
else
    echo "ERROR: Gazebo setup.sh not found! This is critical." >&3
fi

PLUGIN_INSTALL_DIR="/usr/local/lib/gazebo-11/plugins"
PLUGIN_BUILD_DIR="/home/vscode/workspace/gazebo_plugin/build"

# --- Construct and Export GAZEBO_PLUGIN_PATH ---
# Let's be VERY explicit and ensure the install directory is primary and clean.
# The system default Gazebo plugin path is usually handled by Gazebo's own setup.
echo "STEP 3: Constructing GAZEBO_PLUGIN_PATH..." >&3
# Get the path that /usr/share/gazebo/setup.sh sets
GZ_SYSTEM_PLUGIN_PATH_FROM_SETUP="${GAZEBO_PLUGIN_PATH}"
export GAZEBO_PLUGIN_PATH="${PLUGIN_INSTALL_DIR}:${PLUGIN_BUILD_DIR}:${GZ_SYSTEM_PLUGIN_PATH_FROM_SETUP}"
# Remove potential duplicate empty fields from excessive colons if GZ_SYSTEM_PLUGIN_PATH_FROM_SETUP was empty or ended with :
export GAZEBO_PLUGIN_PATH=$(echo "$GAZEBO_PLUGIN_PATH" | awk -F: '{for (i=1;i<=NF;i++) if ($i != "") printf "%s%s", $i, (i<NF && $(i+1) != "" ? FS : "")} {if (substr($0,length($0),1)==":") print ""; else print "\n"}')
# Trim trailing newline if any from awk
GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH%%$'\n'}
echo "EFFECTIVE GAZEBO_PLUGIN_PATH set to: [${GAZEBO_PLUGIN_PATH}]" >&3


# --- Construct and Export GAZEBO_MODEL_PATH (no change) ---
echo "INFO: Attempting to clear Gazebo model cache..." >&3
if [ -d "${HOME}/.gazebo/models" ]; then
    echo "INFO: Found Gazebo model cache directory: ${HOME}/.gazebo/models. Clearing its contents." >&3
    rm -rf "${HOME}/.gazebo/models/"*
    echo "INFO: Contents of ${HOME}/.gazebo/models/ cleared." >&3
else
    echo "INFO: Gazebo model cache directory ${HOME}/.gazebo/models/ not found. Skipping clear." >&3
fi
# Ensure the directory exists for Gazebo and user-specific models
mkdir -p "${HOME}/.gazebo/models"
echo "INFO: Ensured Gazebo model cache directory ${HOME}/.gazebo/models/ exists." >&3

echo "STEP 4: Constructing GAZEBO_MODEL_PATH..." >&3
WORKSPACE_MODEL_DIR="${HOME}/workspace/.gazebo/models"
USER_GAZEBO_MODEL_DIR="${HOME}/.gazebo/models"
CURRENT_GZ_MODEL_PATH="${GAZEBO_MODEL_PATH}"
export GAZEBO_MODEL_PATH="${WORKSPACE_MODEL_DIR}:${USER_GAZEBO_MODEL_DIR}:${CURRENT_GZ_MODEL_PATH}"
# Clean up model path too
export GAZEBO_MODEL_PATH=$(echo "$GAZEBO_MODEL_PATH" | awk -F: '{for (i=1;i<=NF;i++) if ($i != "") printf "%s%s", $i, (i<NF && $(i+1) != "" ? FS : "")} {if (substr($0,length($0),1)==":") print ""; else print "\n"}')
GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH%%$'\n'}
echo "EFFECTIVE GAZEBO_MODEL_PATH set to: [${GAZEBO_MODEL_PATH}]" >&3


# --- Check file existence (simplified, relying on ls output from previous debug) ---
echo "STEP 5: Verifying plugin existence based on prior logs..." >&3
echo "Build dir content at script run time:" >> "${GZ_SERVER_LOG}"
ls -la "${PLUGIN_BUILD_DIR}/" >> "${GZ_SERVER_LOG}" 2>&1
echo "Install dir content at script run time:" >> "${GZ_SERVER_LOG}"
ls -la "${PLUGIN_INSTALL_DIR}/" >> "${GZ_SERVER_LOG}" 2>&1


# --- Generate Model (no change) ---
echo "STEP 6: Generating model..." >&3
cd "${HOME}/workspace"
if [ -f "./generate_model.py" ]; then
    python3 ./generate_model.py >> "${GZ_SERVER_LOG}" 2>&1
    echo "Model generation script finished." >&3
    # Add path for locally generated models
    export GAZEBO_MODEL_PATH="${HOME}/workspace/generated_models:${GAZEBO_MODEL_PATH}"
    echo "GAZEBO_MODEL_PATH updated for generated models: [${GAZEBO_MODEL_PATH}]" >&3

    GENERATED_SDF_PATH="${HOME}/workspace/generated_models/x_quad_drone_generated/model.sdf"
    if [ -f "${GENERATED_SDF_PATH}" ]; then
        echo "--- Content of generated ${GENERATED_SDF_PATH} ---" >&3
        cat "${GENERATED_SDF_PATH}" >> "${GZ_SERVER_LOG}"
        echo "
--- End of ${GENERATED_SDF_PATH} content ---" >&3
    else
        echo "ERROR: Generated SDF file ${GENERATED_SDF_PATH} not found after script execution!" >&3
    fi
else
    echo "WARNING: generate_model.py not found." >&3
fi

GAZEBO_WORLD_FILE="${HOME}/workspace/world.sdf"
echo "STEP 7: Preparing to launch Gazebo Server (gzserver)..." >&3
# ... (rest of script as before) ...
echo "Using world file: $GAZEBO_WORLD_FILE" >&3
echo "Command: gzserver --verbose ${GAZEBO_WORLD_FILE}" >&3

echo "STEP 8: Launching gzserver..." >&3
gzserver --verbose "${GAZEBO_WORLD_FILE}" >> "${GZ_ACTUAL_SERVER_OUTPUT_LOG}" 2>&1

GZSERVER_EXIT_CODE=$?
echo "gzserver process finished with exit code ${GZSERVER_EXIT_CODE}." >&3
echo "--- Gazebo Server Startup Script END ---" >&3
exit ${GZSERVER_EXIT_CODE}