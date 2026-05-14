#!/usr/bin/env python3

"""
Generate the model.sdf file by substituting strings of the form "@foo" with calculated values.

We use the COMMAND method to send commands to the Gazebo Sim ThrusterPlugin. The ThrusterPlugin
supports 2 control methods:
      control thrust via /cmd_thrust
      control angular velocity via /cmd_vel

The ThrusterPlugin uses the Fossen equation to relate thrust to angular velocity, and will apply
thrust force to the joint and spin the propeller. Propellers have bounding boxes and inertia, so
spinning the propeller does affect the simulation.

Typical usage:
scripts/generate_model.py models/bluerov2/model.sdf.in models/bluerov2/model.sdf 0
"""

import math
import re
import sys

# SDF 1.9 supports degrees="true"; provide some nice vars for earlier versions
d180 = math.pi
d90 = d180 / 2
d45 = d90 / 2
d135 = d90 + d45

visual_x = 0.457
visual_y = 0.338
visual_z = 0.4

# Propellers
propeller_size = "0.1 0.02 0.01"
propeller_mass = 0.002
propeller_ixx = 0.001
propeller_iyy = 0.001
propeller_izz = 0.001

# Passive stereo camera system
passive_cam_mass = 0.1
passive_cam_ixx = 0.001
passive_cam_iyy = 0.001
passive_cam_izz = 0.001

passive_cam_x = 0.195
passive_cam_y = -0.00728
passive_cam_z = -0.284
p_cam_baseline = 0.2 / 2 # The cameras are 200 mm apart, so each camera is 100 mm from the center
p_cam_fov = 1.6
p_cam_far_clip =  7
p_img_width = 1600
p_img_height = 1200
p_noise_mean = 0.0
p_noise_std = 0.001


# IMU position
imu_x = 0
imu_y = 0
imu_z = 0

# Waterlinked Sonar 3D-15
sonar_mass = 0.105
sonar_x = 0.11375
sonar_y = 0.0
sonar_z = 0.1133
sonar_ixx = 0.001
sonar_iyy = 0.001
sonar_izz = 0.001

sonar_hz = 5 # Low freq: 5 Hz, High freq: 20 Hz
sonar_h_samples = 250
sonar_v_samples = 66
sonar_h_angle = 45 *(math.pi/180) #Low Freq: 45, High freq: 20
sonar_v_angle = 20 *(math.pi/180) #Low Freq: 20, High freq: 20
sonar_max_range = 15 #Low freq: 15 m, High freq: 4 m
sonar_resolution = 0.0015
sonar_noise_mean = 0.002
sonar_noise_std = 0.05
# Waterlinked DVL A50
dvl_x = -0.075
dvl_y = 0.0
dvl_z = -0.3213
dvl_mass = 0.17
# Visual DVL (cylinder)
dvl_height = 0.025
dvl_radius = 0.033
# DVL noise (Gaussian)
dvl_noise_mean = 0.0
dvl_noise_std = 0.001
dvl_max_range = 50.0
# Mass
base_mass = 14
total_mass = base_mass + 8 * propeller_mass + 2 * passive_cam_mass + sonar_mass + dvl_mass

# The ROV should be positively buoyant
buoyancy_adjustment = 0.05
displaced_mass = total_mass + buoyancy_adjustment

# The collision box is used by the BuoyancyPlugin
# collision_x * collision_y * collision_z * density == displaced_mass
# We have a target buoyancy and displaced_mass, so collision_z is the dependent variable
# You can view the collision box in Gazebo
fluid_density = 1000
collision_x = visual_x
collision_y = visual_y
collision_z = displaced_mass / (visual_x * visual_y * fluid_density)
print(f'total_mass = {total_mass :.4f}, displaced_mass = {displaced_mass :.4f}, collision_z = {collision_z :.4f}')

# The center of mass is just above the origin
mass_z = 0.0113

# The center of volume is directly above the center of mass, resulting in a restoring force
volume_z = 0.06

ixx = total_mass / 12 * (collision_y * collision_y + collision_z * collision_z)
iyy = total_mass / 12 * (collision_x * collision_x + collision_z * collision_z)
izz = total_mass / 12 * (collision_x * collision_x + collision_y * collision_y)

# 2nd order stability for the HydrodynamicsPlugin
xUabsU = -0.5 * visual_y * visual_z * 0.8 * fluid_density
yVabsV = -0.5 * visual_x * visual_z * 0.95 * fluid_density
zWabsW = -0.5 * visual_x * visual_y * 0.95 * fluid_density
kPabsP = -0.5 * 0.008 * fluid_density
mQabsQ = -0.5 * 0.008 * fluid_density
nRabsR = -0.5 * 0.008 * fluid_density

# Thruster placement
thruster_v_x = 0.12
thruster_v_y = 0.218
thruster_v_z = 0.048

thruster_h_x = 0.138
thruster_h_y = 0.098
thruster_h_z = 0.0137

# ThrusterPlugin parameters
propeller_diameter = 0.1
thrust_coefficient = 0.02

# Max thrust force, N
# Both forward and reverse thrust must be the same
max_thrust = 50

# ArduPilotPlugin control parameters
servo_min = 1100
servo_max = 1900
control_offset = -0.5

# From the command line
use_angvel_cmd = False

# Set by update_globals()
# cw_control_multiplier = 0   # Thrusters 3, 4 and 6
cw_control_multiplier = 100 
# ccw_control_multiplier = 0  # Thrusters 1, 2 and 5
ccw_control_multiplier = 100
thruster1_topic = "/model/bluerov2/joint/thruster1_joint/cmd_"
thruster2_topic = "/model/bluerov2/joint/thruster2_joint/cmd_"
thruster3_topic = "/model/bluerov2/joint/thruster3_joint/cmd_"
thruster4_topic = "/model/bluerov2/joint/thruster4_joint/cmd_"
thruster5_topic = "/model/bluerov2/joint/thruster5_joint/cmd_"
thruster6_topic = "/model/bluerov2/joint/thruster6_joint/cmd_"
thruster7_topic = "/model/bluerov2/joint/thruster7_joint/cmd_"
thruster8_topic = "/model/bluerov2/joint/thruster8_joint/cmd_"

# Fossen equation, see "Guidance and Control of Ocean Vehicles" p. 246
def thrust_to_ang_vel(thrust):
    assert thrust >= 0
    assert thrust_coefficient >= 0
    return math.sqrt(thrust / (fluid_density * thrust_coefficient * pow(propeller_diameter, 4)))


def update_globals():
    global cw_control_multiplier
    global ccw_control_multiplier
    global thruster1_topic
    global thruster2_topic
    global thruster3_topic
    global thruster4_topic
    global thruster5_topic
    global thruster6_topic
    global thruster7_topic
    global thruster8_topic

    if use_angvel_cmd:
        print("control method: angular velocity")
        thruster1_topic += "vel"
        thruster2_topic += "vel"
        thruster3_topic += "vel"
        thruster4_topic += "vel"
        thruster5_topic += "vel"
        thruster6_topic += "vel"
        thruster7_topic += "vel"
        thruster8_topic += "vel"

        # Angular velocity range in rad/s
        # Thrust ~ sqrt(angular velocity), so the curves are quite different
        # Reverse the angular velocity for thrusters 3, 4 and 6
        cw_control_multiplier = -thrust_to_ang_vel(max_thrust) * 2
        ccw_control_multiplier = thrust_to_ang_vel(max_thrust) * 2
    else:
        print("control method: thrust force")
        thruster1_topic += "thrust"
        thruster2_topic += "thrust"
        thruster3_topic += "thrust"
        thruster4_topic += "thrust"
        thruster5_topic += "thrust"
        thruster6_topic += "thrust"
        thruster7_topic += "thrust"
        thruster8_topic += "thrust"

        # Force range [-50, 50] in N
        cw_control_multiplier = max_thrust * 2
        ccw_control_multiplier = max_thrust * 2


def generate_model(input_path, output_path):
    s = open(input_path, "r").read()
    pattern = re.compile(r"@(\w+)")
    # globals()['foo'] will return the value of foo
    s = re.sub(pattern, lambda m: str(globals()[m.group(1)]), s)
    open(output_path, "w").write(s)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage:")
        print("generate_model.py infile outfile 0|1")
        print("0: control thrust force")
        print("1: control angular velocity")
        exit(-100)

    use_angvel_cmd = bool(int(sys.argv[3]))

    update_globals()

    generate_model(sys.argv[1], sys.argv[2])
