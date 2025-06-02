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
