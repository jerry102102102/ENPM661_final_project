import json
import math

def generate_world(json_file, output_file):
    try:
        with open(json_file, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Error: {json_file} not found.")
        return

    # XML Header and Environment Setup
    world_content = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>15.0 -15.0 10.0 0 0.5 2.3</pose>
      </camera>
    </gui>
"""

    # Process Static Walls
    for wall in data.get('static_entities', []):
        yaw = wall.get('rotation', 0) * (math.pi / 180.0)
        world_content += f"""
    <model name='{wall['name']}'>
      <static>1</static>
      <pose>{wall['position']['x']} {wall['position']['y']} {wall['position']['z']} 0 0 {yaw}</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{wall['scale']['length']} {wall['scale']['width']} {wall['scale']['height']}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{wall['scale']['length']} {wall['scale']['width']} {wall['scale']['height']}</size></box></geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    # Process Moving Orange Balls (Actors)
    for ball in data.get('dynamic_entities', []):
        world_content += f"""
    <actor name='{ball['name']}'>
      <pose>{ball['start_position']['x']} {ball['start_position']['y']} {ball['start_position']['z']} 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><sphere><radius>{ball['radius']}</radius></sphere></geometry>
          <material>
            <ambient>1 0.65 0 1</ambient> <!-- Orange RGBA -->
            <diffuse>1 0.65 0 1</diffuse>
          </material>
        </visual>
      </link>
      <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id="0" type="linear">"""
        
        for wp in ball['waypoints']:
            world_content += f"""
          <waypoint>
            <time>{wp['time']}</time>
            <pose>{wp['pos']['x']} {wp['pos']['y']} {wp['pos']['z']} 0 0 0</pose>
          </waypoint>"""
            
        world_content += """
        </trajectory>
      </script>
    </actor>"""

    world_content += "\n  </world>\n</sdf>"

    with open(output_file, 'w') as f:
        f.write(world_content)
    print(f"Success! Generated '{output_file}' with 4 walls and 2 moving orange balls.")

if __name__ == "__main__":
    generate_world('world_config.json', 'output.world')
