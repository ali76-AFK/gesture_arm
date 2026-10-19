#!/usr/bin/env python3
"""Generates src/gesture_arm_sim/worlds/pick_place.sdf"""
import pathlib

TABLE_TOP = 0.20  # table surface height (m); the robot base is at z=0

def box(x, y, z):  return f"<box><size>{x} {y} {z}</size></box>"
def cyl(r, h):     return f"<cylinder><radius>{r}</radius><length>{h}</length></cylinder>"

def model(name, geom, pose, rgba, mass=None, inertia=(1e-4, 1e-4, 1e-4)):
    inertial = "" if mass is None else (
        f"<inertial><mass>{mass}</mass><inertia><ixx>{inertia[0]:.3e}</ixx>"
        f"<iyy>{inertia[1]:.3e}</iyy><izz>{inertia[2]:.3e}</izz></inertia></inertial>")
    return f"""
    <model name="{name}">
      <static>{'true' if mass is None else 'false'}</static>
      <pose>{pose}</pose>
      <link name="link">
        {inertial}
        <collision name="c"><geometry>{geom}</geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry>{geom}</geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual>
      </link>
    </model>"""

def bin_model(name, cx, cy, z0, w=0.16, h=0.06, t=0.01):
    parts = [("floor", (0, 0, t/2), (w, w, t)),
             ("wall_n", (0, w/2 - t/2, t + h/2), (w, t, h)),
             ("wall_s", (0, -w/2 + t/2, t + h/2), (w, t, h)),
             ("wall_e", (w/2 - t/2, 0, t + h/2), (t, w - 2*t, h)),
             ("wall_w", (-w/2 + t/2, 0, t + h/2), (t, w - 2*t, h))]
    body = "".join(
        f"""<collision name="{n}_c"><pose>{p[0]} {p[1]} {p[2]} 0 0 0</pose><geometry>{box(*s)}</geometry></collision>
        <visual name="{n}_v"><pose>{p[0]} {p[1]} {p[2]} 0 0 0</pose><geometry>{box(*s)}</geometry>
          <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material></visual>"""
        for n, p, s in parts)
    return f"""
    <model name="{name}"><static>true</static><pose>{cx} {cy} {z0} 0 0 0</pose>
      <link name="link">{body}</link></model>"""

def z_on_table(half_h): return TABLE_TOP + half_h + 0.001

objects = "".join([
    model("red_cube",      box(.05, .05, .05),  f"0.45  0.15 {z_on_table(.025)} 0 0 0", "0.9 0.1 0.1 1",
          0.1, (4.2e-5, 4.2e-5, 4.2e-5)),
    model("green_cube",    box(.05, .05, .05),  f"0.65  0.18 {z_on_table(.025)} 0 0 0", "0.1 0.7 0.2 1",
          0.1, (4.2e-5, 4.2e-5, 4.2e-5)),
    model("blue_cylinder", cyl(.025, .06),      f"0.55 -0.08 {z_on_table(.03)} 0 0 0",  "0.1 0.2 0.9 1",
          0.1, (4.56e-5, 4.56e-5, 3.13e-5)),
    model("yellow_block",  box(.04, .04, .08),  f"0.70 -0.12 {z_on_table(.04)} 0 0 0",  "0.95 0.85 0.1 1",
          0.1, (6.67e-5, 6.67e-5, 2.67e-5)),
])

table = model("table", box(0.8, 1.0, TABLE_TOP), f"0.6 0 {TABLE_TOP/2} 0 0 0", "0.55 0.4 0.25 1")
the_bin = bin_model("bin", 0.55, -0.33, TABLE_TOP)

camera = """
    <model name="overhead_cam">
      <static>true</static>
      <pose>0.6 0 1.0 0 1.5708 0</pose>
      <link name="link">
        <visual name="v"><geometry><box><size>0.06 0.06 0.04</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
        <sensor name="rgbd" type="rgbd_camera">
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image><width>640</width><height>480</height></image>
            <clip><near>0.1</near><far>3.0</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>15</update_rate>
          <visualize>false</visualize>
          <topic>camera</topic>
          <gz_frame_id>camera_link</gz_frame_id>
        </sensor>
      </link>
    </model>"""

sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="pick_place">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>0 0 -9.8</gravity>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.8 0.85 0.9 1</background></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane"><static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link></model>
    {table}{the_bin}{objects}{camera}
  </world>
</sdf>
"""
out = pathlib.Path(__file__).resolve().parent.parent / "src/gesture_arm_sim/worlds/pick_place.sdf"
out.write_text(sdf)
print("wrote", out)
