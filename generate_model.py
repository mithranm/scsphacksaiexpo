#!/usr/bin/env python3
"""
generate_model.py

Reads drone_params.json from the workspace root, then writes out
generated_models/x_quad_drone_generated/model.sdf
with:
  - one "hull" link (box)
  - four "arm" links (boxes)
  - four "rotor" links (cylinders)
  - a <plugin> block using parameters from drone_params.json
"""

import json
import math
import os
import sys

CONFIG_PATH = "drone_params.json" 
if not os.path.isfile(CONFIG_PATH):
    print(f"Error: cannot find config file at {os.path.abspath(CONFIG_PATH)}", file=sys.stderr)
    sys.exit(1)

with open(CONFIG_PATH, "r") as f:
    cfg = json.load(f)

required_top_level = [
    "central_frame_structure", "electronics_stack", "arms", "rotors", "battery",
    "vehicle_total_mass_kg_AUW", "motor_positions_relative_to_cog_m",
    "motor_spin_directions", "gazebo_plugin_run_test_mode", "gazebo_plugin_test_radius",
    "gazebo_plugin_test_angular_velocity", "gazebo_plugin_test_altitude",
    "drag_coefficient", "initial_lin_vel", "initial_ang_vel"
]
for key in required_top_level:
    if key not in cfg:
        print(f"Error: missing top‐level key \"{key}\" in {CONFIG_PATH}", file=sys.stderr)
        sys.exit(1)

# Hull mass: central_frame + electronics + battery
hull_mass = cfg["central_frame_structure"]["mass_kg"] + cfg["electronics_stack"]["mass_kg"] + cfg["battery"]["mass_kg"]
hull_x, hull_y, hull_z = cfg["electronics_stack"]["dimensions_m"] 

Ixx_hull = (1/12) * hull_mass * (hull_y**2 + hull_z**2)
Iyy_hull = (1/12) * hull_mass * (hull_x**2 + hull_z**2)
Izz_hull = (1/12) * hull_mass * (hull_x**2 + hull_y**2)

# Sanity check total mass
arms_cfg_val = cfg["arms"]
rotors_cfg_val = cfg["rotors"]
calculated_total_mass = (hull_mass +
                         arms_cfg_val["mass_per_arm_kg"] * arms_cfg_val["count"] +
                         (rotors_cfg_val["prop_mass_kg"] + rotors_cfg_val["motor_mass_kg"]) * rotors_cfg_val["count"])
if abs(calculated_total_mass - cfg["vehicle_total_mass_kg_AUW"]) > 0.01: 
    print(
        f"Warning: Calculated total mass ({calculated_total_mass:.6f} kg) differs from "
        f"vehicle_total_mass_kg_AUW ({cfg['vehicle_total_mass_kg_AUW']:.6f} kg) in drone_params.json "
        f"by {abs(calculated_total_mass - cfg['vehicle_total_mass_kg_AUW']):.6f} kg. "
        "Ensure AUW is correct.",
        file=sys.stderr
    )

sdf_lines = [
    "<?xml version='1.0' encoding='UTF-8'?>",
    '<sdf version="1.7">',
    '  <model name="x_quad_drone_generated">',
    "    <static>false</static>",
    "    <pose>0 0 1.0 0 0 0</pose>", 
    f'    <link name="hull">',
    f'      <pose>0 0 0 0 0 0</pose>', 
    f'      <inertial>',
    f'        <mass>{hull_mass:.6f}</mass>',
    f'        <inertia>',
    f'          <ixx>{Ixx_hull:.8e}</ixx>',
    f'          <iyy>{Iyy_hull:.8e}</iyy>',
    f'          <izz>{Izz_hull:.8e}</izz>',
    f'          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>',
    f'        </inertia>',
    f'      </inertial>',
    f'      <collision name="hull_geom_col">',
    f'        <geometry><box><size>{hull_x:.6f} {hull_y:.6f} {hull_z:.6f}</size></box></geometry>',
    f'      </collision>',
    f'      <visual name="hull_geom_vis">',
    f'        <geometry><box><size>{hull_x:.6f} {hull_y:.6f} {hull_z:.6f}</size></box></geometry>',
    f'        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>',
    f'      </visual>',
    f'    </link>'
]

motor_positions_val = cfg["motor_positions_relative_to_cog_m"]

arm_width = arms_cfg_val["width_m"]
arm_thickness = arms_cfg_val["thickness_m"]
mass_per_arm = arms_cfg_val["mass_per_arm_kg"]
motor_keys = list(motor_positions_val.keys())

for idx, motor_name_json_key in enumerate(motor_keys):
    px, py, pz_motor = motor_positions_val[motor_name_json_key] 
    arm_length_xy = math.sqrt(px**2 + py**2)

    Ixx_arm = (1/12) * mass_per_arm * (arm_width**2 + arm_thickness**2)
    Iyy_arm = (1/12) * mass_per_arm * (arm_length_xy**2 + arm_thickness**2)
    Izz_arm = (1/12) * mass_per_arm * (arm_length_xy**2 + arm_width**2)
    
    arm_yaw_radians = math.atan2(py, px)
    arm_com_x = px / 2.0
    arm_com_y = py / 2.0
    arm_com_z = pz_motor / 2.0 

    arm_link_name = f"arm_{idx}"
    sdf_lines.extend([
        f'    <link name="{arm_link_name}">',
        f'      <pose>{arm_com_x:.6f} {arm_com_y:.6f} {arm_com_z:.6f} 0 0 {arm_yaw_radians:.6f}</pose>',
        f'      <inertial><mass>{mass_per_arm:.6f}</mass><inertia><ixx>{Ixx_arm:.8e}</ixx><iyy>{Iyy_arm:.8e}</iyy><izz>{Izz_arm:.8e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>',
        f'      <collision name="{arm_link_name}_geom_col"><geometry><box><size>{arm_length_xy:.6f} {arm_width:.6f} {arm_thickness:.6f}</size></box></geometry></collision>',
        f'      <visual name="{arm_link_name}_geom_vis"><geometry><box><size>{arm_length_xy:.6f} {arm_width:.6f} {arm_thickness:.6f}</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/DarkGrey</name></script></material></visual>',
        f'    </link>',
        f'    <joint name="joint_hull_to_{arm_link_name}" type="fixed">',
        f'      <parent>hull</parent><child>{arm_link_name}</child><pose>0 0 0 0 0 0</pose>', # This pose is relative to parent link frame
        f'    </joint>'
    ])

    rotor_mass = rotors_cfg_val["prop_mass_kg"] + rotors_cfg_val["motor_mass_kg"]
    rotor_radius = rotors_cfg_val["prop_diameter_m"] / 2.0
    rotor_height = 0.010 

    Ixx_rot = (1/12) * rotor_mass * (3 * rotor_radius**2 + rotor_height**2)
    Iyy_rot = Ixx_rot
    Izz_rot = (1/2) * rotor_mass * rotor_radius**2
    
    rotor_link_name = f"rotor_{idx}" 
    sdf_lines.extend([
        f'    <link name="{rotor_link_name}">',
        f'      <pose>{px:.6f} {py:.6f} {pz_motor:.6f} 0 0 0</pose>', # Rotor CoM in model frame, cylinder Z is up
        f'      <inertial><mass>{rotor_mass:.6f}</mass><inertia><ixx>{Ixx_rot:.8e}</ixx><iyy>{Iyy_rot:.8e}</iyy><izz>{Izz_rot:.8e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>',
        f'      <collision name="{rotor_link_name}_geom_col"><geometry><cylinder><radius>{rotor_radius:.6f}</radius><length>{rotor_height:.6f}</length></cylinder></geometry></collision>',
        f'      <visual name="{rotor_link_name}_geom_vis"><geometry><cylinder><radius>{rotor_radius:.6f}</radius><length>{rotor_height:.6f}</length></cylinder></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Red</name></script></material></visual>',
        f'    </link>',
        f'    <joint name="joint_{arm_link_name}_to_{rotor_link_name}" type="revolute">',
        f'      <parent>{arm_link_name}</parent>', # Arm is parent
        f'      <child>{rotor_link_name}</child>',   # Rotor is child
        # Pose of the joint frame, relative to the CHILD link's origin frame.
        # If rotor_link_name <pose> is its CoM in model frame, and arm <pose> is its CoM in model frame,
        # this joint pose needs to be defined carefully.
        # Alternative: pose child (rotor) relative to parent (arm) in joint.
        # Let's try placing rotor relative to arm tip.
        # Arm tip in arm's local frame (X along length) is (arm_length_xy/2, 0, 0)
        # Rotor should be placed at this tip, with its Z axis (rotation axis) aligned with model Z.
        # The joint's <pose> tag specifies the pose of the child link frame relative to the parent link frame.
        # So, this is the offset from arm's CoM to where the rotor is mounted.
        # Rotor CoM is (px, py, pz_motor). Arm CoM is (arm_com_x, arm_com_y, arm_com_z).
        # Vector from arm CoM to rotor CoM in world frame: (px-arm_com_x, py-arm_com_y, pz_motor-arm_com_z)
        # This vector needs to be expressed in the arm's frame.
        # Since arm's local X points to motor, this vector is (arm_length_xy/2, 0, pz_motor - arm_com_z)
        # if arm_com_z is the Z of the arm's centerline.
        # If pz_motor is 0 (motors in XY plane of hull), and arm_com_z is 0, then this is (arm_length_xy/2, 0, 0)
        f'      <pose>{arm_length_xy/2.0:.6f} 0 {(pz_motor - arm_com_z):.6f} 0 0 0</pose>', 
        f'      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit><dynamics><damping>0.0001</damping></dynamics></axis>', # Reduced damping slightly
        f'    </joint>'
    ])

run_test_str = "true" if cfg["gazebo_plugin_run_test_mode"] else "false"
sdf_lines.extend([
    f'    <plugin name="trick_executor_plugin" filename="libtrick_executor_plugin.so">',
    f'      <rotor_count>{arms_cfg_val["count"]}</rotor_count>',
    f'      <rotor_link_base_name>rotor_</rotor_link_base_name>',
    f'      <zmq_port>5555</zmq_port>',
    f'      <run_test_mode>{run_test_str}</run_test_mode>',
    f'      <test_mode_radius>{cfg["gazebo_plugin_test_radius"]:.6f}</test_mode_radius>',
    f'      <test_mode_angular_velocity>{cfg["gazebo_plugin_test_angular_velocity"]:.6f}</test_mode_angular_velocity>',
    f'      <test_mode_altitude>{cfg["gazebo_plugin_test_altitude"]:.6f}</test_mode_altitude>',
    f'      <drag_coefficient>{cfg["drag_coefficient"]:.6f}</drag_coefficient>',
    f'    </plugin>',
    f'  </model>',
    f'</sdf>'
])

output_directory = os.path.join("generated_models", "x_quad_drone_generated")
os.makedirs(output_directory, exist_ok=True)
output_sdf_filepath = os.path.join(output_directory, "model.sdf")
with open(output_sdf_filepath, "w") as out:
    out.write("\n".join(sdf_lines))
print(f"Generated SDF: {os.path.abspath(output_sdf_filepath)}")

model_config_content = f"""<?xml version="1.0"?>
<model>
  <name>x_quad_drone_generated</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author><name>Parametric Script</name><email>user@example.com</email></author>
  <description>X-frame quadcopter. Test mode: {run_test_str.capitalize()}</description>
</model>
"""
model_config_path = os.path.join(output_directory, "model.config")
with open(model_config_path, "w") as f_cfg:
    f_cfg.write(model_config_content) # Corrected variable name
print(f"Wrote model.config: {os.path.abspath(model_config_path)}")