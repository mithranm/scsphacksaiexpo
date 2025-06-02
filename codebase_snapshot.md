# Snapshot

## Filesystem Tree

```
gazebo-devcontainer/
├── .devcontainer/
│   ├── Dockerfile
│   └── devcontainer.json
├── .gazebo/
│   └── models/
│       └── tesseract_nano/
│           ├── model.config
│           └── model.sdf
├── .gitignore
├── README.md
├── drone_params.json
├── generate_model.py
├── pyproject.toml
└── world.sdf
```

## File Contents

Files are ordered alphabetically by path.

### File: .devcontainer\devcontainer.json

```json
{
    "name": "Gazebo 11 Dev Container",
    "build": {
      "dockerfile": "Dockerfile",
      "context": ".."
    },
  
    //  ⬇ Bind your host 'gazebo-devcontainer' directly into /home/vscode in the container
    "workspaceMount": "source=${localWorkspaceFolder},target=/home/vscode,type=bind,consistency=cached",
    "workspaceFolder": "/home/vscode",
  
    // Make sure each new terminal is a login shell so ~/.bashrc gets sourced
  
    // Extensions you might want (e.g. Python)
    "customizations": {
      "vscode": {
        "settings": {
          "terminal.integrated.shell.linux": "/bin/bash",
          "terminal.integrated.shellArgs.linux": ["-l"]
        },
        "extensions": [
          "ms-python.python"
        ]
      }
    },
  
    // Forward Gazebo’s default port for remote gzclient if needed
    "forwardPorts": [11345],
  
    // Run as unprivileged user 'vscode'
    "remoteUser": "vscode"
  }
  
```

---
### File: .devcontainer\Dockerfile

```
# 1. Base: Official Gazebo 11 server image (Ubuntu 20.04 "focal" for amd64)
FROM gazebo:gzserver11-focal

# 2. Suppress interactive prompts in apt-get
ENV DEBIAN_FRONTEND=noninteractive

# 3. Install Python 3 and NumPy (for our generator script)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3 python3-pip && \
    pip3 install numpy && \
    rm -rf /var/lib/apt/lists/*

# 4. Create an unprivileged 'vscode' user (UID 1000)
RUN useradd --create-home --shell /bin/bash vscode

# 5. Ensure Gazebo is sourced in every new shell for both root and vscode
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> /home/vscode/.bashrc && \
    chown vscode:vscode /home/vscode/.bashrc

# 6. Create the folder for custom models (so the Python generator can write there)
USER root
RUN mkdir -p /home/vscode/.gazebo/models/tesseract_nano && \
    chown -R vscode:vscode /home/vscode/.gazebo

# 7. Switch to the unprivileged user for subsequent work
USER vscode
WORKDIR /home/vscode

# 8. Open bash as a login shell (so ~/.bashrc runs automatically)
CMD [ "bash", "-l" ]
```

---
### File: .gazebo\models\tesseract_nano\model.config

```
<model>
  <name>tesseract_nano</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>Hackathon Team</name>
    <email>mithran.mohanraj@gmail.com</email>
  </author>
  <description>Tesseract Nano FPV Drone (static SDF)</description>
</model>
```

---
### File: .gazebo\models\tesseract_nano\model.sdf

```
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="tesseract_nano">

    <link name="hull">
      <pose>0 0 0.005 0 0 0</pose>
      <inertial>
        <mass>0.02</mass>
        <inertia>
          <ixx>0.00000121</ixx>
          <iyy>0.00000433</iyy>
          <izz>0.00000521</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="hull_visual">
        <geometry>
          <box><size>0.050 0.025 0.010</size></box>
        </geometry>
        <material><ambient>0.7 0.7 0.7 1</ambient></material>
      </visual>
      <collision name="hull_collision">
        <geometry>
          <box><size>0.050 0.025 0.010</size></box>
        </geometry>
      </collision>
    </link>

    <link name="battery">
      <pose>0 0 0.004 0 0 0</pose>
      <inertial>
        <mass>0.007</mass>
        <inertia>
          <ixx>0.00000027</ixx>
          <iyy>0.00000056</iyy>
          <izz>0.00000076</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="bat_visual">
        <geometry>
          <box><size>0.030 0.020 0.008</size></box>
        </geometry>
        <material><ambient>0.2 0.2 1 1</ambient></material>
      </visual>
      <collision name="bat_collision">
        <geometry>
          <box><size>0.030 0.020 0.008</size></box>
        </geometry>
      </collision>
    </link>
    <!-- Joint battery → hull -->
    <joint name="joint_battery_hull" type="fixed">
      <parent>hull</parent>
      <child>battery</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>

    <link name="camera">
      <pose>0.031 0 0 0 0 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia>
          <ixx>0.00000010</ixx>
          <iyy>0.00000010</iyy>
          <izz>0.00000012</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="cam_visual">
        <geometry>
          <box><size>0.012 0.012 0.01</size></box>
        </geometry>
        <material><ambient>1 0.2 0.2 1</ambient></material>
      </visual>
      <collision name="cam_collision">
        <geometry>
          <box><size>0.012 0.012 0.01</size></box>
        </geometry>
      </collision>
    </link>
    <!-- Joint camera → hull -->
    <joint name="joint_cam_hull" type="fixed">
      <parent>hull</parent>
      <child>camera</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>

    <link name="arm_0">
      <pose>0.03000000 0.00000000 0.005 0 0 0.00000000</pose>
      <inertial>
        <mass>0.00086400</mass>
        <inertia>
          <ixx>0.00000000</ixx>
          <iyy>0.00000026</iyy>
          <izz>0.00000026</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="arm_vis_0">
        <geometry>
          <box><size>0.060 0.003 0.003</size></box>
        </geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient></material>
      </visual>
      <collision name="arm_col_0">
        <geometry>
          <box><size>0.060 0.003 0.003</size></box>
        </geometry>
      </collision>
    </link>
    <joint name="joint_arm_0" type="fixed">
      <parent>hull</parent>
      <child>arm_0</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
    
    <link name="rotor_0">
      <pose>0.06000000 0.00000000 0.015 0 0 0</pose>
      <inertial>
        <mass>0.01500000</mass>
        <inertia>
          <ixx>0.00000093</ixx>
          <iyy>0.00000093</iyy>
          <izz>0.00000180</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="rotor_vis_0">
        <geometry>
          <cylinder><radius>0.0155</radius><length>0.005</length></cylinder>
        </geometry>
        <material><ambient>0.6 0.6 0.6 1</ambient></material>
      </visual>
      <collision name="rotor_col_0">
        <geometry>
          <cylinder><radius>0.0155</radius><length>0.005</length></cylinder>
        </geometry>
      </collision>
      <plugin name="rotor_plugin_0" filename="libRotorPlugin.so">
        <rotor>
          <jointName>joint_arm_0</jointName>
          <linkName>rotor_0</linkName>
          <torqueConstant>1.200000e-07</torqueConstant>
          <momentConstant>2.500000e-08</momentConstant>
          <direction>1</direction>
          <bladeArea>0.000755</bladeArea>
        </rotor>
      </plugin>
    </link>
    <joint name="joint_rotor_0" type="fixed">
      <parent>arm_0</parent>
      <child>rotor_0</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
    
    <link name="arm_1">
      <pose>0.00000000 0.03000000 0.005 0 0 1.57079633</pose>
      <inertial>
        <mass>0.00086400</mass>
        <inertia>
          <ixx>0.00000000</ixx>
          <iyy>0.00000026</iyy>
          <izz>0.00000026</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="arm_vis_1">
        <geometry>
          <box><size>0.060 0.003 0.003</size></box>
        </geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient></material>
      </visual>
      <collision name="arm_col_1">
        <geometry>
          <box><size>0.060 0.003 0.003</size></box>
        </geometry>
      </collision>
    </link>
    <joint name="joint_arm_1" type="fixed">
      <parent>hull</parent>
      <child>arm_1</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
    
    <link name="rotor_1">
      <pose>0.00000000 0.06000000 0.015 0 0 0</pose>
      <inertial>
        <mass>0.01500000</mass>
        <inertia>
          <ixx>0.00000093</ixx>
          <iyy>0.00000093</iyy>
          <izz>0.00000180</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="rotor_vis_1">
        <geometry>
          <cylinder><radius>0.0155</radius><length>0.005</length></cylinder>
        </geometry>
        <material><ambient>0.6 0.6 0.6 1</ambient></material>
      </visual>
      <collision name="rotor_col_1">
        <geometry>
          <cylinder><radius>0.0155</radius><length>0.005</length></cylinder>
        </geometry>
      </collision>
      <plugin name="rotor_plugin_1" filename="libRotorPlugin.so">
        <rotor>
          <jointName>joint_arm_1</jointName>
          <linkName>rotor_1</linkName>
          <torqueConstant>1.200000e-07</torqueConstant>
          <momentConstant>2.500000e-08</momentConstant>
          <direction>1</direction>
          <bladeArea>0.000755</bladeArea>
        </rotor>
      </plugin>
    </link>
    <joint name="joint_rotor_1" type="fixed">
      <parent>arm_1</parent>
      <child>rotor_1</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
    
    <link name="arm_2">
      <pose>-0.03000000 0.00000000 0.005 0 0 3.14159265</pose>
      <inertial>
        <mass>0.00086400</mass>
        <inertia>
          <ixx>0.00000000</ixx>
          <iyy>0.00000026</iyy>
          <izz>0.00000026</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="arm_vis_2">
        <geometry>
          <box><size>0.060 0.003 0.003</size></box>
        </geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient></material>
      </visual>
      <collision name="arm_col_2">
        <geometry>
          <box><size>0.060 0.003 0.003</size></box>
        </geometry>
      </collision>
    </link>
    <joint name="joint_arm_2" type="fixed">
      <parent>hull</parent>
      <child>arm_2</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
    
    <link name="rotor_2">
      <pose>-0.06000000 0.00000000 0.015 0 0 0</pose>
      <inertial>
        <mass>0.01500000</mass>
        <inertia>
          <ixx>0.00000093</ixx>
          <iyy>0.00000093</iyy>
          <izz>0.00000180</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="rotor_vis_2">
        <geometry>
          <cylinder><radius>0.0155</radius><length>0.005</length></cylinder>
        </geometry>
        <material><ambient>0.6 0.6 0.6 1</ambient></material>
      </visual>
      <collision name="rotor_col_2">
        <geometry>
          <cylinder><radius>0.0155</radius><length>0.005</length></cylinder>
        </geometry>
      </collision>
      <plugin name="rotor_plugin_2" filename="libRotorPlugin.so">
        <rotor>
          <jointName>joint_arm_2</jointName>
          <linkName>rotor_2</linkName>
          <torqueConstant>1.200000e-07</torqueConstant>
          <momentConstant>2.500000e-08</momentConstant>
          <direction>1</direction>
          <bladeArea>0.000755</bladeArea>
        </rotor>
      </plugin>
    </link>
    <joint name="joint_rotor_2" type="fixed">
      <parent>arm_2</parent>
      <child>rotor_2</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
    
    <link name="arm_3">
      <pose>-0.00000000 -0.03000000 0.005 0 0 4.71238898</pose>
      <inertial>
        <mass>0.00086400</mass>
        <inertia>
          <ixx>0.00000000</ixx>
          <iyy>0.00000026</iyy>
          <izz>0.00000026</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="arm_vis_3">
        <geometry>
          <box><size>0.060 0.003 0.003</size></box>
        </geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient></material>
      </visual>
      <collision name="arm_col_3">
        <geometry>
          <box><size>0.060 0.003 0.003</size></box>
        </geometry>
      </collision>
    </link>
    <joint name="joint_arm_3" type="fixed">
      <parent>hull</parent>
      <child>arm_3</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
    
    <link name="rotor_3">
      <pose>-0.00000000 -0.06000000 0.015 0 0 0</pose>
      <inertial>
        <mass>0.01500000</mass>
        <inertia>
          <ixx>0.00000093</ixx>
          <iyy>0.00000093</iyy>
          <izz>0.00000180</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="rotor_vis_3">
        <geometry>
          <cylinder><radius>0.0155</radius><length>0.005</length></cylinder>
        </geometry>
        <material><ambient>0.6 0.6 0.6 1</ambient></material>
      </visual>
      <collision name="rotor_col_3">
        <geometry>
          <cylinder><radius>0.0155</radius><length>0.005</length></cylinder>
        </geometry>
      </collision>
      <plugin name="rotor_plugin_3" filename="libRotorPlugin.so">
        <rotor>
          <jointName>joint_arm_3</jointName>
          <linkName>rotor_3</linkName>
          <torqueConstant>1.200000e-07</torqueConstant>
          <momentConstant>2.500000e-08</momentConstant>
          <direction>1</direction>
          <bladeArea>0.000755</bladeArea>
        </rotor>
      </plugin>
    </link>
    <joint name="joint_rotor_3" type="fixed">
      <parent>arm_3</parent>
      <child>rotor_3</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
    
    <static>false</static>
  </model>
</sdf>
```

---
### File: .gitignore

```
.cache/
.vscode-server/
.dotnet/
```

---
### File: drone_params.json

```json
{
    "hull": {
        "length": 0.05,
        "width": 0.025,
        "height": 0.01,
        "mass": 0.02
    },
    "arms": {
        "count": 4,
        "length": 0.06,
        "cross_section": [0.003, 0.003],
        "density": 1600
    },
    "rotors": {
        "prop_diameter": 0.031,
        "prop_mass": 0.001,
        "motor_mass": 0.014,
        "torque_constant": 1.2e-7,
        "moment_constant": 2.5e-8
    },
    "battery": {
        "mass": 0.007,
        "dimensions": [0.03, 0.02, 0.008]
    },
    "camera": {
        "mass": 0.005
    }
}
```

---
### File: generate_model.py

```python
#!/usr/bin/env python3
import json
import os
import numpy as np

# Load parameters from JSON
with open("drone_params.json", "r") as f:
    params = json.load(f)

# Helper to compute inertia for a box
def box_inertia(mass, x, y, z):
    # Ixx = (1/12)*m*(y^2 + z^2), etc.
    Ixx = (1.0/12.0) * mass * (y*y + z*z)
    Iyy = (1.0/12.0) * mass * (x*x + z*z)
    Izz = (1.0/12.0) * mass * (x*x + y*y)
    return Ixx, Iyy, Izz

# Helper to compute inertia for a solid cylinder (propeller + motor)
def cylinder_inertia(mass, radius, length=0.005):
    # Cylinder axis = z-axis; Ixx = Iyy = (1/12)*m*(3r^2 + h^2), Izz = (1/2)*m*r^2
    Ixx = Iyy = (1.0/12.0) * mass * (3*radius*radius + length*length)
    Izz = 0.5 * mass * radius*radius
    return Ixx, Iyy, Izz

# Create model.sdf content
model_sdf = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="tesseract_nano">
"""

# 1. Hull link
h = params["hull"]
hull_name = "hull"
h_Ixx, h_Iyy, h_Izz = box_inertia(h["mass"], h["length"], h["width"], h["height"])
model_sdf += f"""
    <link name="{hull_name}">
      <pose>0 0 {h['height']/2} 0 0 0</pose>
      <inertial>
        <mass>{h['mass']}</mass>
        <inertia>
          <ixx>{h_Ixx:.8f}</ixx>
          <iyy>{h_Iyy:.8f}</iyy>
          <izz>{h_Izz:.8f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="hull_visual">
        <geometry>
          <box><size>{h['length']:.3f} {h['width']:.3f} {h['height']:.3f}</size></box>
        </geometry>
        <material><ambient>0.7 0.7 0.7 1</ambient></material>
      </visual>
      <collision name="hull_collision">
        <geometry>
          <box><size>{h['length']:.3f} {h['width']:.3f} {h['height']:.3f}</size></box>
        </geometry>
      </collision>
    </link>
"""

# 2. Add battery as a small box under hull
b = params["battery"]
b_mass = b["mass"]
b_dim = b["dimensions"]  # [x, y, z]
b_Ixx, b_Iyy, b_Izz = box_inertia(b_mass, b_dim[0], b_dim[1], b_dim[2])
model_sdf += f"""
    <link name="battery">
      <pose>0 0 {b_dim[2]/2} 0 0 0</pose>
      <inertial>
        <mass>{b_mass}</mass>
        <inertia>
          <ixx>{b_Ixx:.8f}</ixx>
          <iyy>{b_Iyy:.8f}</iyy>
          <izz>{b_Izz:.8f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="bat_visual">
        <geometry>
          <box><size>{b_dim[0]:.3f} {b_dim[1]:.3f} {b_dim[2]:.3f}</size></box>
        </geometry>
        <material><ambient>0.2 0.2 1 1</ambient></material>
      </visual>
      <collision name="bat_collision">
        <geometry>
          <box><size>{b_dim[0]:.3f} {b_dim[1]:.3f} {b_dim[2]:.3f}</size></box>
        </geometry>
      </collision>
    </link>
    <!-- Joint battery → hull -->
    <joint name="joint_battery_hull" type="fixed">
      <parent>{hull_name}</parent>
      <child>battery</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
"""

# 3. Camera as a small box on front of hull
cam_mass = params["camera"]["mass"]
cam_Ixx, cam_Iyy, cam_Izz = box_inertia(cam_mass, 0.012, 0.012, 0.01)
model_sdf += f"""
    <link name="camera">
      <pose>{h['length']/2 + 0.006} 0 0 0 0 0</pose>
      <inertial>
        <mass>{cam_mass}</mass>
        <inertia>
          <ixx>{cam_Ixx:.8f}</ixx>
          <iyy>{cam_Iyy:.8f}</iyy>
          <izz>{cam_Izz:.8f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="cam_visual">
        <geometry>
          <box><size>0.012 0.012 0.01</size></box>
        </geometry>
        <material><ambient>1 0.2 0.2 1</ambient></material>
      </visual>
      <collision name="cam_collision">
        <geometry>
          <box><size>0.012 0.012 0.01</size></box>
        </geometry>
      </collision>
    </link>
    <!-- Joint camera → hull -->
    <joint name="joint_cam_hull" type="fixed">
      <parent>{hull_name}</parent>
      <child>camera</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
"""

# 4. Arms and rotors
arm_count = params["arms"]["count"]
arm_len = params["arms"]["length"]
arm_x, arm_y = params["arms"]["cross_section"]
arm_density = params["arms"]["density"]
r_params = params["rotors"]
for i in range(arm_count):
    theta = (2.0 * np.pi / arm_count) * i
    cos_t, sin_t = np.cos(theta), np.sin(theta)

    # Arm mass & inertia
    arm_mass = arm_density * arm_len * arm_x * arm_y
    a_Ixx, a_Iyy, a_Izz = box_inertia(arm_mass, arm_len, arm_x, arm_y)
    link_arm = f"arm_{i}"
    # Position arm so center at (arm_len/2, 0, 0) rotated by theta
    ax = (arm_len/2) * cos_t
    ay = (arm_len/2) * sin_t

    model_sdf += f"""
    <link name="{link_arm}">
      <pose>{ax:.8f} {ay:.8f} {h['height']/2} 0 0 {theta:.8f}</pose>
      <inertial>
        <mass>{arm_mass:.8f}</mass>
        <inertia>
          <ixx>{a_Ixx:.8f}</ixx>
          <iyy>{a_Iyy:.8f}</iyy>
          <izz>{a_Izz:.8f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="arm_vis_{i}">
        <geometry>
          <box><size>{arm_len:.3f} {arm_x:.3f} {arm_y:.3f}</size></box>
        </geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient></material>
      </visual>
      <collision name="arm_col_{i}">
        <geometry>
          <box><size>{arm_len:.3f} {arm_x:.3f} {arm_y:.3f}</size></box>
        </geometry>
      </collision>
    </link>
    <joint name="joint_arm_{i}" type="fixed">
      <parent>{hull_name}</parent>
      <child>{link_arm}</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
    """

    # Rotor link (cylinder)
    rotor_mass = r_params["prop_mass"] + r_params["motor_mass"]
    rotor_rad = r_params["prop_diameter"] / 2
    c_Ixx, c_Iyy, c_Izz = cylinder_inertia(rotor_mass, rotor_rad)
    link_rotor = f"rotor_{i}"
    rx = arm_len * cos_t
    ry = arm_len * sin_t

    model_sdf += f"""
    <link name="{link_rotor}">
      <pose>{rx:.8f} {ry:.8f} {h['height'] + 0.005} 0 0 0</pose>
      <inertial>
        <mass>{rotor_mass:.8f}</mass>
        <inertia>
          <ixx>{c_Ixx:.8f}</ixx>
          <iyy>{c_Iyy:.8f}</iyy>
          <izz>{c_Izz:.8f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="rotor_vis_{i}">
        <geometry>
          <cylinder><radius>{rotor_rad:.4f}</radius><length>0.005</length></cylinder>
        </geometry>
        <material><ambient>0.6 0.6 0.6 1</ambient></material>
      </visual>
      <collision name="rotor_col_{i}">
        <geometry>
          <cylinder><radius>{rotor_rad:.4f}</radius><length>0.005</length></cylinder>
        </geometry>
      </collision>
      <plugin name="rotor_plugin_{i}" filename="libRotorPlugin.so">
        <rotor>
          <jointName>joint_arm_{i}</jointName>
          <linkName>{link_rotor}</linkName>
          <torqueConstant>{r_params['torque_constant']:.6e}</torqueConstant>
          <momentConstant>{r_params['moment_constant']:.6e}</momentConstant>
          <direction>1</direction>
          <bladeArea>{np.pi * rotor_rad**2:.6f}</bladeArea>
        </rotor>
      </plugin>
    </link>
    <joint name="joint_rotor_{i}" type="fixed">
      <parent>arm_{i}</parent>
      <child>rotor_{i}</child>
      <pose>0 0 0 0 0 0</pose>
    </joint>
    """

# Close the model tag and add static tag if desired
model_sdf += """
    <static>false</static>
  </model>
</sdf>
"""

# Write model.sdf into ~/.gazebo/models/tesseract_nano/
home = os.path.expanduser("~")
model_dir = os.path.join(home, ".gazebo", "models", "tesseract_nano")
os.makedirs(model_dir, exist_ok=True)
with open(os.path.join(model_dir, "model.sdf"), "w") as f:
    f.write(model_sdf)

print(f"Written model.sdf to {model_dir}/model.sdf")
```

---
### File: pyproject.toml

```toml
[tool.vibelint]
include_globs = [
    "**/*",
]

exclude_globs = [
    ".git/*",
    ".vscode-server/*",
    ".dotnet/*",
    ".cache/*",
]
```

---
### File: README.md

```markdown

```

---
### File: world.sdf

```
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="default">
    <!-- Default lighting and ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Spawn Tesseract Nano at origin -->
    <include>
      <uri>model://tesseract_nano</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <!-- Optional: add a static obstacle or marker -->
    <model name="box_marker">
      <pose>1 0 0.25 0 0 0</pose>
      <link name="link">
        <pose>0 0 0 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

---

