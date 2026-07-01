#!/usr/bin/env python3

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import math
import rospy
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from fmm_global_planner.srv import FMMPlan, FMMPlanResponse


from fmm_grad_utils import (
    compute_fmm_distance_map,
    get_fmm_path_bilinear,
)


def occ_grid_to_binary_grid(msg, occupied_threshold=50):
    """
    Converts nav_msgs/OccupancyGrid to numpy grid.
    FMM expects:
      0 = free
      1 = obstacle
    """
    data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))

    grid = np.zeros_like(data, dtype=np.int8)

    # Unknown cells are -1. Treat them as obstacles for safety.
    grid[data < 0] = 1
    grid[data >= occupied_threshold] = 1

    return grid


def world_to_grid(x, y, map_msg):
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    res = map_msg.info.resolution

    col = int(round((x - origin_x) / res))
    row = int(round((y - origin_y) / res))

    return row, col


def make_pose(x, y, frame_id):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose


def handle_fmm_plan(req):
    try:
        map_msg = req.map
        grid = occ_grid_to_binary_grid(map_msg)

        res = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        width = map_msg.info.width
        height = map_msg.info.height

        world_bounds = (
            origin_x,
            origin_y,
            origin_x + width * res,
            origin_y + height * res,
        )

        start_world = (
            req.start.pose.position.x,
            req.start.pose.position.y,
        )
        goal_world = (
            req.goal.pose.position.x,
            req.goal.pose.position.y,
        )

        goal_row, goal_col = world_to_grid(goal_world[0], goal_world[1], map_msg)

        dist_map = compute_fmm_distance_map(
            grid,
            goal_row,
            goal_col,
            resolution=res,
        )

        path_np = get_fmm_path_bilinear(
            dist_map,
            start_world=start_world,
            goal_world=goal_world,
            resolution=res,
            world_bounds=world_bounds,
            grid=grid,
            base_step=0.5,
            wall_repulsion_radius=1.0,
            wall_repulsion_strength=4.0,
        )

        if path_np is None or len(path_np) == 0:
            return FMMPlanResponse(
                path=Path(),
                success=False,
                message="FMM failed to produce path",
            )

        path_msg = Path()
        path_msg.header.frame_id = map_msg.header.frame_id
        path_msg.header.stamp = rospy.Time.now()

        for x, y in path_np:
            path_msg.poses.append(make_pose(x, y, map_msg.header.frame_id))

        return FMMPlanResponse(
            path=path_msg,
            success=True,
            message="success",
        )

    except Exception as e:
        rospy.logerr("FMM planning failed: %s", str(e))
        return FMMPlanResponse(
            path=Path(),
            success=False,
            message=str(e),
        )


def main():
    rospy.init_node("fmm_plan_server")
    service_name = rospy.get_param("~service_name", "/fmm_make_plan")

    rospy.Service(service_name, FMMPlan, handle_fmm_plan)

    rospy.loginfo("FMM plan server ready on %s", service_name)
    rospy.spin()


if __name__ == "__main__":
    main()