"""ROS-map route assessment: reject unsafe and excessive global paths.

This module never computes an alternative path or relaxes obstacle clearance.
Nav2 remains the source of the collision-aware plan.
"""
import math


def assess_path(poses, goal_x, goal_y, geofence=None, margin=0.0,
                max_detour_ratio=4.0, min_detour_limit_m=12.0,
                max_goal_error_m=0.3, start_x=None, start_y=None,
                max_start_error_m=0.6):
    points = []
    for stamped in poses:
        p = stamped.pose.position
        x, y = float(p.x), float(p.y)
        if not (math.isfinite(x) and math.isfinite(y)):
            return {'ok': False, 'message': 'Planner returned a non-finite path point.', 'points': []}
        points.append({'x': x, 'y': y})
    if not points:
        return {'ok': False, 'message': 'Planner returned an empty path.', 'points': []}
    if len(points) < 2:
        return {'ok': False, 'message': 'Planner returned an incomplete path.', 'points': points}
    if start_x is not None and start_y is not None:
        start_error = math.hypot(points[0]['x'] - start_x, points[0]['y'] - start_y)
        if start_error > max_start_error_m:
            return {'ok': False, 'message': f'Planner path starts {start_error:.2f} m from localized robot.', 'points': points}
    goal_error = math.hypot(points[-1]['x'] - goal_x, points[-1]['y'] - goal_y)
    if goal_error > max_goal_error_m:
        return {'ok': False, 'message': f'Planner endpoint differs from requested goal by {goal_error:.2f} m.', 'points': points}
    if geofence is not None and geofence.enabled:
        if any(not geofence.is_robot_inside(p['x'], p['y'], margin) for p in points):
            return {'ok': False, 'message': 'Planned route crosses the permitted geofence boundary.', 'points': points}
    length = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y'])
                 for a, b in zip(points, points[1:]))
    direct = math.hypot(goal_x - points[0]['x'], goal_y - points[0]['y'])
    if length > max(min_detour_limit_m, direct * max_detour_ratio):
        return {
            'ok': False,
            'message': f'Excessive detour: {length:.1f} m route for {direct:.1f} m goal distance; navigation inhibited.',
            'points': points, 'length_m': length, 'direct_m': direct,
        }
    return {'ok': True, 'message': f'Route verified: {length:.1f} m for {direct:.1f} m direct distance.',
            'points': points, 'length_m': length, 'direct_m': direct}
