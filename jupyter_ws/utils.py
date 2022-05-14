from dataclasses import dataclass
import json
import os
import yaml
import pandas as pd
import numpy as np
from scipy import integrate
from tf.transformations import euler_from_quaternion


def bag_path(map, algorithm):
    repo = "/".join(os.path.abspath(__file__).split("/")[:-2])
    rosbag_path = os.path.join(repo, "catkin_ws", "rosbag", map, f"{algorithm}_bag")
    return rosbag_path


def bag_list(path):
    for (_, _, files) in os.walk(path):
        return list(map(lambda file: os.path.join(path, file), sorted(files)))


def parse_yaml(string):
    return yaml.load(string, Loader=yaml.FullLoader)


def parse_poses_to_path(poses):
    poses = map(parse_yaml, poses[1:-1].split(", "))
    path = pd.DataFrame(map(lambda pose: pose["pose"]["position"], poses))
    path.drop("z", axis="columns", inplace=True)
    return path


def path_length(path):
    return np.sqrt(np.square(path.diff().drop(columns=["t", "theta"])).sum(axis=1)).sum()


def path_deviation(path, distance):
    integral = integrate.simps(distance.dist, distance.t)
    return (integral, integral/path_length(path))


def travel_time(cmd_vel):
    non_zero = cmd_vel[~((cmd_vel["linear.x"].abs() < 1e-6) & (cmd_vel["angular.z"].abs() < 1e-6))]
    return non_zero.iloc[-1].Time - non_zero.iloc[0].Time


def simplify_odom(odoms: pd.DataFrame):
    specific = odoms[[
        "Time",
        "pose.pose.position.x",
        "pose.pose.position.y",
        "pose.pose.orientation.x",
        "pose.pose.orientation.y",
        "pose.pose.orientation.z",
        "pose.pose.orientation.w",
    ]].rename(columns={
        "Time": "t",
        "pose.pose.position.x": "x",
        "pose.pose.position.y": "y",
        "pose.pose.orientation.x": "qx",
        "pose.pose.orientation.y": "qy",
        "pose.pose.orientation.z": "qz",
        "pose.pose.orientation.w": "qw",
    })

    specific["theta"] = specific[["qx", "qy", "qz", "qw"]].apply(lambda r: euler_from_quaternion(r.to_numpy())[-1], axis=1)
    specific.drop(columns=["qx", "qy", "qz", "qw"], inplace=True)
    return specific

def simplify_cmd_vel(cmd_vel: pd.DataFrame):
    return cmd_vel[~((cmd_vel["linear.x"].abs() < 1e-6) & (cmd_vel["angular.z"].abs() < 1e-6))]\
        .rename(columns={"linear.x": "linear", "angular.z": "angular", "Time": "t"})\
        [["t", "linear", "angular"]]

def evaluate_smoothness(data: pd.DataFrame, x_name, y_name, z_name=None, angle_fix=False):

    norm_x = (data[x_name] - data[x_name].min())/(data[x_name].max() - data[x_name].min())
    norm_y = (data[y_name] - data[y_name].min())/(data[y_name].max() - data[y_name].min())

    diff_y = norm_y.diff().replace({0: 1e-10})
    if (angle_fix):
        for i, ang in enumerate(diff_y):
            if np.abs(ang) >= np.pi*2:
                diff_y[i] = (diff_y[i-1] + diff_y[i+1])/2

    if z_name is not None:
        norm_z = (data[z_name] - data[z_name].min())/(data[z_name].max() - data[z_name].min())
        shrinked = np.sqrt(np.square(diff_y) + np.square(norm_z.diff()))
        der = shrinked / norm_x.diff()
    else:
        der = diff_y / norm_x.diff()
    
    for i, vel in der.items():
            if np.abs(vel) >= 1e+3:
                der.drop(i, inplace=True)

    return der[~der.isin([np.inf, -np.inf])].std()


def odom_to_vel(simple_odom: pd.DataFrame):
    diff = simple_odom.diff()
    diff["linear"] = np.sqrt(np.power(diff.x, 2) + np.power(diff.y, 2))/diff.t
    diff["angular"] = diff.theta/diff.t

    for i, ang in enumerate(diff.angular):
        if np.abs(ang) >= np.pi*2:
            diff["angular"][i] = (diff["angular"][i-1] + diff["angular"][i+1])/2

    diff.dropna(inplace=True)
    diff.index -= 1
    diff.t /=2

    diff.t += simple_odom[:-1].t
    diff.drop(columns=["x", "y", "theta"], inplace=True)

    return diff


def local_to_global_distance(local_path: pd.DataFrame, global_path: pd.DataFrame):
    distance_list = []
    for _, local_point in local_path.iterrows():
        np_local_point = local_point[["x", "y"]].to_numpy()
        np_global_point = global_path.iloc[0].to_numpy()
        distance = np.linalg.norm(np_local_point - np_global_point)
        for _, global_point in global_path.iterrows():
            np_global_point = global_point.to_numpy()
            distance = min(distance, np.linalg.norm(np_local_point - np_global_point))
        distance_list += [{"t": local_point.t, "dist": distance}]

    return pd.DataFrame(distance_list)

def load_data(dir, map, approach):
    @dataclass
    class Deviation:
        absolute: float
        relative: float

    @dataclass
    class DataObject:
        map_name: str
        approach_name: str
        global_path: pd.DataFrame
        local_path: pd.DataFrame
        velocity: pd.DataFrame
        odom_velocity: pd.DataFrame
        global_path_deviation: pd.DataFrame
        measured_deviation: Deviation
        travel_time: float
        freq: float

    with open(os.path.join(dir, map, f"{approach}.json"), "r") as f:
        data = json.load(f)
        return DataObject(
            map_name=map,
            approach_name=approach,
            global_path=pd.DataFrame(data["global_path"]),
            local_path=pd.DataFrame(data["local_path"]),
            velocity=pd.DataFrame(data["velocity"]),
            odom_velocity=pd.DataFrame(data["odom_velocity"]),
            global_path_deviation=pd.DataFrame(data["global_path_deviation"]),
            measured_deviation=Deviation(**data["measured_deviation"]),
            travel_time=data["travel_time"],
            freq=data["freq"]
        )