#!/usr/bin/env python
import rospkg
from PIL import Image, ImageDraw
import json, yaml
from numpy import average
import utm

rospack = rospkg.RosPack()
package_path = rospack.get_path('swan_navigation')
json_fn = "geo_fence.json"
resolution = 1

def draw_polygon_map(pkg_path, resolution, json_filename):
    with open(pkg_path + "/maps/" + json_filename, "r") as geo_fence_file:
        geo_fence_points_gps = json.load(geo_fence_file)
        geo_fence_file.close()

    centre_x = 0
    centre_y = 0

    geo_fence_points_utm = []
    for point_gps in geo_fence_points_gps:
        point_full_utm = utm.from_latlon(point_gps[0], point_gps[1])
        point_utm = [point_full_utm[0], point_full_utm[1]]
        geo_fence_points_utm.append(point_utm)
        centre_x += point_full_utm[0];
        centre_y += point_full_utm[1];

    total_points = len(geo_fence_points_utm)
    centre_x /= total_points
    centre_y /= total_points

    centre_geo_fence_utm = (centre_x, centre_y)

    points_in_map = []
    max_x_m = 0;
    max_y_m = 0;
    for point_utm in geo_fence_points_utm:
        x_m = point_utm[0] - centre_geo_fence_utm[0]
        y_m = point_utm[1] - centre_geo_fence_utm[1]
        if abs(x_m) > max_x_m:
            max_x_m = abs(x_m)
        if abs(y_m) > max_y_m:
            max_y_m = abs(y_m)
        points_in_map.append([x_m / resolution, -y_m / resolution])

    width_m = max_x_m * 5
    height_m = max_y_m * 5
    width = int(width_m / resolution)
    height = int(height_m / resolution)

    for i in range(len(points_in_map)):
        points_in_map[i] = (int(points_in_map[i][0] + width / 2), int(points_in_map[i][1] + height / 2))

    filename = json_filename.replace(".json", "")
    im_filename = filename + ".pgm"
    yaml_filename = filename + ".yaml"
    centre_yaml_filename = "centre.yaml"

    im = Image.new("L", (width, height), 205)
    ImageDraw.Draw(im).polygon(points_in_map, 254, 0)
    im.save(pkg_path + "/maps/geo_fence.pgm")

    param = dict()
    param["image"] = im_filename
    param["resolution"] = resolution
    param["origin"] = [( - width_m / 2.0), ( - height_m / 2.0), 0]
    param["negate"] = 0
    param["occupied_thresh"] = 0.65
    param["free_thresh"] = 0.196


    param_file = open(pkg_path + "/maps/" + yaml_filename,"w+")
    param_file.write(yaml.dump(param, default_flow_style=False))
    param_file.close()

    centre_param = dict()
    centre_param["x"] = centre_x
    centre_param["y"] = centre_y
    centre_param["z"] = 0
    centre_param["yaw"] = 0
    centre_param["pitch"] = 0
    centre_param["roll"] = 0
    centre_param_file = open(pkg_path + "/maps/" + centre_yaml_filename, "w+")
    centre_param_file.write(yaml.dump(centre_param, default_flow_style=False))
    centre_param_file.close()

draw_polygon_map(package_path, resolution, json_fn)
