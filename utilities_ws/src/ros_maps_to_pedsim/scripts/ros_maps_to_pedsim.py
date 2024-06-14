#!/usr/bin/env python

import rospy
import yaml
import os.path

import numpy as np
import xml.etree.ElementTree as xml
import skimage.io as io

from xml.dom import minidom


def get_window(image, x, y):
    """
    Returns a window around a pixel.

    The windows is a 3x3 window centererd around pixel (x, y). If the pixel is
    close to the edges of the image, the window will be smaller, accordingly
    (e.g., the method will return only a 2x2 window for pixel (0, 0)).

        Parameters:
            image (array_like): an image from which the window is extracted
            x (int): x coordinate of the pixel
            y (int): y coordinate of the pixel

        Returns:
            window (array_like): a window around the pixel (x, y)
    """
    sz = image.shape
    assert (x >= 0 and x < sz[0] and y >= 0 and y <
            sz[1]), "Pixel indeces out of image bounds (%d, %d)" % (x, y)

    x_min = np.maximum(0, x-1)
    x_max = np.minimum(sz[0], x+2)
    y_min = np.maximum(0, y-1)
    y_max = np.minimum(sz[1], y+2)

    return image[x_min:x_max, y_min:y_max]


def add_waypoint(scenario, id, x, y, r):
    """Adds to a scenario a waypoint named 'id' in (x, y) with radius 'r'"""
    waypoint = xml.SubElement(scenario, 'waypoint')
    waypoint.set('id', str(id))
    waypoint.set('x', str(x))
    waypoint.set('y', str(y))
    waypoint.set('r', str(r))


def add_agent(scenario, x, y, waypoints, n=2, dx=0.5, dy=0.5, type=1):
    """Adds to a scenario n agents going from (x, y) through the waypoints"""
    agent = xml.SubElement(scenario, 'agent')
    agent.set('x', str(x))
    agent.set('y', str(y))
    agent.set('n', str(n))
    agent.set('dx', str(dx))
    agent.set('dy', str(dy))
    agent.set('type', str(type))
    for id in waypoints:
        addwaypoint = xml.SubElement(agent, 'addwaypoint')
        addwaypoint.set('id', str(id))


def add_waypoints_and_agent(scenario, agents_info):
    """Adds to a scenario a set of waypoints and agents going through them"""
    waypoints = agents_info["waypoints"]
    for id in waypoints.keys():
        w = waypoints[id]
        add_waypoint(scenario, id, w[0], w[1], w[2])

    agents_keys = agents_info.keys()
    agents_keys.remove('waypoints')
    for key in agents_keys:
        agent = agents_info[key]
        agent_dx = agent['dx'] if 'dx' in agent else 0.5
        agent_dy = agent['dy'] if 'dy' in agent else 0.5
        agent_type = agent['type'] if 'type' in agent else 1
        add_agent(scenario, agent['x'], agent['y'], agent['w'], n=agent['n'],
                  dx=agent_dx, dy=agent_dy, type=agent_type)


def add_obstacle(scenario, x1, y1, x2, y2):
    """Adds to a scenario an obstacle going from (x1, y1) to (x2, y2)"""
    obstacle = xml.SubElement(scenario, 'obstacle')
    obstacle.set('x1', str(x1))
    obstacle.set('y1', str(y1))
    obstacle.set('x2', str(x2))
    obstacle.set('y2', str(y2))


def add_pixel_obstacle(scenario, x, y, resolution):
    """Adds to a scenario a 1x1 obstacle at location (x, y)"""
    add_obstacle(scenario, x + resolution / 2, y - resolution / 2,
                 x - resolution / 2, y + resolution / 2)


def scenario_from_map(map_image, map_metadata, use_map_origin=False):
    """
    Builds a pedsim scenario having obstacles to separate free space in the map
    from unknown and occupied space. Everything below 'free_thresh' (in the map
    metadata) is considered free space.

        Parameters:
            map_image (array_like): the map ternary image
            map_metadata (dictionary): the metadata extracted from the map YAML
                file
            use_map_origin (bool): if True reads the map origin from
                map_metadata, otherwise sets it to [0, 0, 0] (default).
                Integration with pedsim_ros works better in the latter case.

        Returns:
            scenario (ElementTree): a pedsim scenario as xml element tree
            map_walls (array_like): a binary image showing the locations on the
                map where obstacles have been placed
    """
    resolution = map_metadata['resolution']
    negate = map_metadata['negate']
    free_thresh = map_metadata['free_thresh'] * 255
    origin = map_metadata['origin'] if use_map_origin else [0.0, 0.0, 0.0]

    # ROS maps have white (255) as free space for visualization, colors need to
    # be inverted before comparing with thresholds (if negate == 0)
    if ~negate:
        map_binary = 255-map_image < free_thresh
    else:
        map_binary = map_image < free_thresh

    scenario = xml.Element('scenario')

    sz = map_binary.shape
    map_walls = np.zeros(sz, dtype=bool)

    # reduce the search space to only the area where there is free space
    x_free = np.nonzero(np.sum(map_binary, axis=1))[0]
    x_min = np.maximum(0, x_free[0]-1)
    x_max = np.minimum(sz[0], x_free[-1]+2)
    y_free = np.nonzero(np.sum(map_binary, axis=0))[0]
    y_min = np.maximum(0, y_free[0]-1)
    y_max = np.minimum(sz[1], y_free[-1]+2)

    for x in range(x_min, x_max):
        for y in range(y_min, y_max):
            is_free = map_binary[x, y]
            window = get_window(map_binary, x, y)
            if ~is_free and np.any(window) and np.any(~window):
                # conversion between world coordinates and pixel coordinates
                # (x and y coordinates are inverted, and y is also flipped)
                world_x = origin[0] + y * resolution
                world_y = origin[1] - (x - sz[0]) * resolution

                add_pixel_obstacle(scenario, world_x, world_y, resolution)
                map_walls[x, y] = True

    return scenario, map_walls


def write_xml(tree, file_path, indent="  "):
    """Takes an xml tree and writes it to a file, indented"""
    indented_xml = minidom.parseString(
        xml.tostring(tree)).toprettyxml(indent=indent)

    with open(file_path, "w") as f:
        f.write(indented_xml)


if __name__ == '__main__':
    rospy.init_node('ros_maps_to_pedsim', anonymous=True)

    map_path = rospy.get_param("~map_path", ".")
    map_name = rospy.get_param("~map_name", "map.yaml")
    scenario_path = rospy.get_param("~scenario_path", ".")
    scenario_name = rospy.get_param("~scenario_name", "scene.xml")
    use_map_origin = rospy.get_param("~use_map_origin", False)
    add_agents = rospy.get_param("~add_agents", True)
    agents_info_path = rospy.get_param("~agents_info_path", ".")
    agents_info_name = rospy.get_param("~agents_info_name", "agents.yaml")

    with open(os.path.join(map_path, map_name)) as file:
        map_metadata = yaml.safe_load(file)

    map_image = io.imread(os.path.join(map_path, map_metadata['image']))

    print("Loaded map in " + os.path.join(map_path, map_name)
          + " with metadata:")
    print(map_metadata)

    scenario, map_walls = scenario_from_map(
        map_image, map_metadata, use_map_origin)

    # uncomment for a visualization of where the obstacles have been placed
    # io.imsave(os.path.join(scenario_path, 'walls.png'), map_walls*255)

    if add_agents:
        with open(os.path.join(agents_info_path, agents_info_name)) as file:
            agents_info = yaml.safe_load(file)
            print(agents_info)
        add_waypoints_and_agent(scenario, agents_info)

    print("Writing scene in " + os.path.join(scenario_path, scenario_name)
          + "...")

    write_xml(scenario, os.path.join(scenario_path, scenario_name))

    print("Done.")
