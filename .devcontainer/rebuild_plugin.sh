#!/bin/bash
set -e

PLUGIN_DIR="/home/vscode/workspace/gazebo_plugin"
BUILD_DIR="$PLUGIN_DIR/build"

echo "--- Rebuilding Gazebo Plugin ---"
echo "Plugin Directory: $PLUGIN_DIR"
echo "Build Directory: $BUILD_DIR"

# Clean the build directory first for a true rebuild (optional but often good practice)
echo "Cleaning previous build artifacts (if any)..."
if [ -d "$BUILD_DIR" ]; then
  # Be careful with rm -rf; ensure BUILD_DIR is correctly set
  # Create a temporary subdirectory inside build to delete, then recreate build
  # This avoids deleting build if it's a symlink or something unexpected.
  TEMP_CONTENTS_DIR="${BUILD_DIR}/_temp_rebuild_contents"
  mkdir -p "$TEMP_CONTENTS_DIR" # Ensure build dir exists at least
  rm -rf "${BUILD_DIR:?}"/* # Delete contents of build directory, :? errors if var is unset/null
fi
mkdir -p "$BUILD_DIR" # Ensure build directory exists

# Navigate to the plugin directory first, then to build for out-of-source build
cd "$PLUGIN_DIR"

# Run CMake to configure the project in the build directory
echo "Running CMake from $PLUGIN_DIR into $BUILD_DIR..."
cmake -S . -B "$BUILD_DIR" # Configure from current dir (PLUGIN_DIR) into BUILD_DIR

# Navigate into the build directory for make and install commands
cd "$BUILD_DIR"

# Run Make
echo "Running Make in $BUILD_DIR..."
make -j$(nproc) # You can keep -j$(nproc) for manual rebuilds if your system handles it

# Install the plugin using cmake --install for correct prefix handling
echo "Running Sudo CMake Install..."
sudo cmake --install . --prefix /usr/local

echo "--- Gazebo Plugin Rebuild Complete ---"
echo "Plugin should now be in /usr/local/lib/gazebo-11/plugins/ and $BUILD_DIR/"
echo "You may need to restart the Gazebo server (gzserver) for changes to take effect."
echo "If 'sim_services' tmux session is running:"
echo "  tmux attach-session -t sim_services"
echo "  Ctrl+b, 0 (to go to gzserver window)"
echo "  Ctrl+c (to stop gzserver)"
echo "  /usr/local/bin/start_gazebo.sh (to restart gzserver)"