#!/usr/bin/env python3
import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path


@dataclass
class VehicleParams:
    length: float
    width: float
    wheelbase: float
    max_steering: float
    motion_step: float
    obstacle_inflation: float
    allow_reverse: bool

    @property
    def min_turning_radius(self) -> float:
        return self.wheelbase / math.tan(self.max_steering)


@dataclass
class State:
    x: float
    y: float
    yaw: float

    def distance_to(self, other: "State") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass
class Node:
    state: State
    g: float
    f: float
    parent: Optional["Node"] = None
    trajectory: Optional[List[State]] = None

    def __lt__(self, other: "Node") -> bool:
        return self.f < other.f


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class GridCollisionChecker:
    def __init__(self, vehicle: VehicleParams):
        self.vehicle = vehicle
        self.map: Optional[OccupancyGrid] = None
        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0
        self.obstacle_threshold = rospy.get_param("~obstacle_threshold", 50)
        self.treat_unknown_as_obstacle = rospy.get_param("~treat_unknown_as_obstacle", True)
        self.distance_field: List[float] = []

    def update_map(self, msg: OccupancyGrid) -> None:
        self.map = msg
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.origin_yaw = yaw_from_quaternion(msg.info.origin.orientation)
        self.update_distance_field()

    def update_distance_field(self) -> None:
        if self.map is None or self.width <= 0 or self.height <= 0:
            self.distance_field = []
            return

        cell_count = self.width * self.height
        self.distance_field = [float("inf")] * cell_count
        queue: List[Tuple[float, int, int]] = []

        for my in range(self.height):
            row = my * self.width
            for mx in range(self.width):
                if self.is_cell_occupied(mx, my):
                    index = row + mx
                    self.distance_field[index] = 0.0
                    heapq.heappush(queue, (0.0, mx, my))

        if not queue:
            return

        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
        ]
        while queue:
            dist_cells, mx, my = heapq.heappop(queue)
            index = my * self.width + mx
            if dist_cells > self.distance_field[index]:
                continue
            for dx, dy, step in neighbors:
                nx = mx + dx
                ny = my + dy
                if not (0 <= nx < self.width and 0 <= ny < self.height):
                    continue
                next_index = ny * self.width + nx
                next_dist = dist_cells + step
                if next_dist < self.distance_field[next_index]:
                    self.distance_field[next_index] = next_dist
                    heapq.heappush(queue, (next_dist, nx, ny))

    def world_to_map(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.map is None:
            return None
        dx = x - self.origin_x
        dy = y - self.origin_y
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        mx = int(math.floor(lx / self.resolution))
        my = int(math.floor(ly / self.resolution))
        if 0 <= mx < self.width and 0 <= my < self.height:
            return mx, my
        return None

    def is_cell_occupied(self, mx: int, my: int) -> bool:
        if self.map is None:
            return True
        value = self.map.data[my * self.width + mx]
        if value < 0:
            return self.treat_unknown_as_obstacle
        return value >= self.obstacle_threshold

    def is_world_occupied(self, x: float, y: float) -> bool:
        cell = self.world_to_map(x, y)
        if cell is None:
            return True
        mx, my = cell
        radius_cells = int(math.ceil(self.vehicle.obstacle_inflation / self.resolution))
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                nx = mx + dx
                ny = my + dy
                if not (0 <= nx < self.width and 0 <= ny < self.height):
                    return True
                if math.hypot(dx, dy) * self.resolution <= self.vehicle.obstacle_inflation:
                    if self.is_cell_occupied(nx, ny):
                        return True
        return False

    def collides(self, state: State) -> bool:
        half_l = self.vehicle.length * 0.5
        half_w = self.vehicle.width * 0.5
        step = max(self.resolution, 0.04)
        c = math.cos(state.yaw)
        s = math.sin(state.yaw)

        nx = max(2, int(math.ceil(self.vehicle.length / step)))
        ny = max(2, int(math.ceil(self.vehicle.width / step)))

        for ix in range(nx + 1):
            lx = -half_l + self.vehicle.length * ix / nx
            for iy in range(ny + 1):
                ly = -half_w + self.vehicle.width * iy / ny
                wx = state.x + lx * c - ly * s
                wy = state.y + lx * s + ly * c
                if self.is_world_occupied(wx, wy):
                    return True
        return False

    def trajectory_collides(self, trajectory: List[State], skip_first: bool = False) -> bool:
        states = trajectory[1:] if skip_first else trajectory
        return any(self.collides(state) for state in states)

    def distance_to_obstacle(self, x: float, y: float) -> float:
        cell = self.world_to_map(x, y)
        if cell is None or not self.distance_field:
            return 0.0
        mx, my = cell
        dist_cells = self.distance_field[my * self.width + mx]
        if not math.isfinite(dist_cells):
            return float("inf")
        return dist_cells * self.resolution

    def clearance_margin(self, state: State) -> float:
        footprint_radius = 0.5 * math.hypot(self.vehicle.length, self.vehicle.width)
        return self.distance_to_obstacle(state.x, state.y) - footprint_radius

    def clearance_gradient(self, state: State) -> Tuple[float, float]:
        step = max(self.resolution, 0.025)
        dx = self.distance_to_obstacle(state.x + step, state.y) - self.distance_to_obstacle(state.x - step, state.y)
        dy = self.distance_to_obstacle(state.x, state.y + step) - self.distance_to_obstacle(state.x, state.y - step)
        return dx / (2.0 * step), dy / (2.0 * step)


class HybridParkingPlanner:
    def __init__(self, vehicle: VehicleParams, collision: GridCollisionChecker):
        self.vehicle = vehicle
        self.collision = collision
        self.grid_resolution = rospy.get_param("~grid_resolution", 0.08)
        self.angle_resolution = math.radians(rospy.get_param("~angle_resolution_deg", 10.0))
        self.goal_xy_tolerance = rospy.get_param("~goal_xy_tolerance", 0.12)
        self.goal_yaw_tolerance = math.radians(rospy.get_param("~goal_yaw_tolerance_deg", 12.0))
        self.max_iterations = rospy.get_param("~max_iterations", 25000)
        self.reverse_penalty = rospy.get_param("~reverse_penalty", 0.25)
        self.steering_penalty = rospy.get_param("~steering_penalty", 0.06)
        self.allow_in_place_rotation = rospy.get_param("~allow_in_place_rotation", True)
        self.in_place_yaw_step = math.radians(rospy.get_param("~in_place_yaw_step_deg", 20.0))
        self.steering_angles = [
            -vehicle.max_steering,
            -0.6 * vehicle.max_steering,
            -0.25 * vehicle.max_steering,
            0.0,
            0.25 * vehicle.max_steering,
            0.6 * vehicle.max_steering,
            vehicle.max_steering,
        ]
        self.directions = [1.0, -1.0] if vehicle.allow_reverse else [1.0]

    def plan(self, start: State, goal: State) -> List[State]:
        if self.collision.collides(start):
            rospy.logwarn("Parking start pose is in collision with the current map. Continuing and ignoring only the first sampled state.")
        if self.collision.collides(goal):
            rospy.logerr("Parking goal pose is in collision.")
            return []

        start_node = Node(start, 0.0, self.heuristic(start, goal))
        open_set = [start_node]
        closed = set()
        best_g: Dict[Tuple[int, int, int], float] = {self.grid_key(start): 0.0}
        best_node = start_node
        best_dist = start.distance_to(goal)

        for iteration in range(1, self.max_iterations + 1):
            if not open_set:
                break
            current = heapq.heappop(open_set)
            key = self.grid_key(current.state)
            if key in closed:
                continue
            closed.add(key)

            dist = current.state.distance_to(goal)
            yaw_err = abs(normalize_angle(goal.yaw - current.state.yaw))
            if dist < best_dist:
                best_dist = dist
                best_node = current

            if dist <= self.goal_xy_tolerance and yaw_err <= self.goal_yaw_tolerance:
                rospy.loginfo("Hybrid parking path found in %d iterations.", iteration)
                return self.reconstruct_path(current)

            if iteration % 60 == 0 and dist < 0.8:
                shortcut = self.straight_shortcut(current.state, goal)
                if shortcut and not self.collision.trajectory_collides(shortcut, skip_first=True):
                    goal_node = Node(goal, current.g + self.path_length(shortcut), 0.0, current, shortcut)
                    rospy.loginfo("Hybrid parking shortcut accepted in %d iterations.", iteration)
                    return self.reconstruct_path(goal_node)

            for steering in self.steering_angles:
                for direction in self.directions:
                    trajectory = self.integrate(current.state, steering, direction)
                    if self.collision.trajectory_collides(trajectory, skip_first=True):
                        continue
                    child_state = trajectory[-1]
                    child_key = self.grid_key(child_state)
                    if child_key in closed:
                        continue
                    step_cost = self.path_length(trajectory)
                    step_cost += abs(steering) * self.steering_penalty
                    if direction < 0.0:
                        step_cost += self.reverse_penalty
                    g = current.g + step_cost
                    if g >= best_g.get(child_key, float("inf")):
                        continue
                    best_g[child_key] = g
                    f = g + self.heuristic(child_state, goal)
                    heapq.heappush(open_set, Node(child_state, g, f, current, trajectory))

            if self.allow_in_place_rotation:
                for yaw_delta in (-self.in_place_yaw_step, self.in_place_yaw_step):
                    trajectory = self.rotate_in_place(current.state, yaw_delta)
                    if self.collision.trajectory_collides(trajectory, skip_first=True):
                        continue
                    child_state = trajectory[-1]
                    child_key = self.grid_key(child_state)
                    if child_key in closed:
                        continue
                    g = current.g + abs(yaw_delta) * 0.10
                    if g >= best_g.get(child_key, float("inf")):
                        continue
                    best_g[child_key] = g
                    f = g + self.heuristic(child_state, goal)
                    heapq.heappush(open_set, Node(child_state, g, f, current, trajectory))

            if iteration % 2000 == 0:
                rospy.loginfo("Hybrid parking search: iter=%d, open=%d, best_dist=%.2f",
                              iteration, len(open_set), best_dist)

        rospy.logwarn("Hybrid parking search stopped; returning best partial path, best_dist=%.2f", best_dist)
        return self.reconstruct_path(best_node)

    def grid_key(self, state: State) -> Tuple[int, int, int]:
        return (
            int(round(state.x / self.grid_resolution)),
            int(round(state.y / self.grid_resolution)),
            int(round(normalize_angle(state.yaw) / self.angle_resolution)),
        )

    def heuristic(self, state: State, goal: State) -> float:
        dist = state.distance_to(goal)
        yaw_err = abs(normalize_angle(goal.yaw - state.yaw))
        return dist + 0.35 * self.vehicle.min_turning_radius * yaw_err

    def integrate(self, state: State, steering: float, direction: float) -> List[State]:
        step = self.vehicle.motion_step
        substeps = max(2, int(math.ceil(step / 0.025)))
        ds = direction * step / substeps
        current = State(state.x, state.y, state.yaw)
        trajectory = [current]
        for _ in range(substeps):
            x = current.x + ds * math.cos(current.yaw)
            y = current.y + ds * math.sin(current.yaw)
            yaw = normalize_angle(current.yaw + ds * math.tan(steering) / self.vehicle.wheelbase)
            current = State(x, y, yaw)
            trajectory.append(current)
        return trajectory

    def rotate_in_place(self, state: State, yaw_delta: float) -> List[State]:
        steps = max(2, int(math.ceil(abs(yaw_delta) / math.radians(5.0))))
        trajectory = [state]
        for i in range(1, steps + 1):
            ratio = float(i) / float(steps)
            trajectory.append(State(state.x, state.y, normalize_angle(state.yaw + ratio * yaw_delta)))
        return trajectory

    def straight_shortcut(self, start: State, goal: State) -> List[State]:
        dist = start.distance_to(goal)
        if dist < 1e-6:
            return [start, goal]
        if abs(normalize_angle(math.atan2(goal.y - start.y, goal.x - start.x) - start.yaw)) > math.radians(45):
            return []
        steps = max(2, int(math.ceil(dist / 0.04)))
        path = []
        for i in range(steps + 1):
            t = float(i) / float(steps)
            path.append(State(
                start.x + t * (goal.x - start.x),
                start.y + t * (goal.y - start.y),
                normalize_angle(start.yaw + t * normalize_angle(goal.yaw - start.yaw)),
            ))
        return path

    @staticmethod
    def path_length(path: List[State]) -> float:
        return sum(path[i - 1].distance_to(path[i]) for i in range(1, len(path)))

    @staticmethod
    def reconstruct_path(node: Node) -> List[State]:
        chunks = []
        current = node
        while current is not None:
            chunks.append(current.trajectory if current.trajectory else [current.state])
            current = current.parent
        path: List[State] = []
        for chunk in reversed(chunks):
            for state in chunk:
                if not path or path[-1].distance_to(state) > 0.01:
                    path.append(state)
        return path


class PathPostProcessor:
    def __init__(self, collision: GridCollisionChecker):
        self.collision = collision
        self.enabled = rospy.get_param("~enable_path_smoothing", True)
        self.shortcut_enabled = rospy.get_param("~enable_path_shortcut", True)
        self.shortcut_max_skip = rospy.get_param("~shortcut_max_skip", 20)
        self.resample_spacing = rospy.get_param("~smooth_resample_spacing", 0.05)
        self.smooth_iterations = rospy.get_param("~smooth_iterations", 80)
        self.smooth_weight_data = rospy.get_param("~smooth_weight_data", 0.25)
        self.smooth_weight_smooth = rospy.get_param("~smooth_weight_smooth", 0.45)
        self.smooth_tolerance = rospy.get_param("~smooth_tolerance", 1e-4)
        self.clearance_enabled = rospy.get_param("~enable_obstacle_clearance", True)
        self.min_obstacle_clearance = rospy.get_param("~min_obstacle_clearance", 0.08)
        self.clearance_weight = rospy.get_param("~obstacle_clearance_weight", 0.35)
        self.max_clearance_push = rospy.get_param("~max_clearance_push", 0.03)

    def process(self, raw_path: List[State]) -> List[State]:
        if not self.enabled or len(raw_path) < 4:
            return raw_path

        path = raw_path
        if self.shortcut_enabled:
            path = self.shortcut_path(path)
        path = self.resample_path(path, self.resample_spacing)
        path = self.smooth_path(path)

        raw_clearance = self.path_min_clearance(raw_path)
        optimized_clearance = self.path_min_clearance(path)
        clearance_ok = (
            not self.clearance_enabled
            or optimized_clearance >= self.min_obstacle_clearance
            or optimized_clearance >= raw_clearance - 0.01
        )
        if self.path_collision_free(path, skip_first=True) and clearance_ok:
            rospy.loginfo(
                "Path smoothing accepted: raw=%d, optimized=%d poses, min_clearance %.3f -> %.3f m.",
                len(raw_path), len(path), raw_clearance, optimized_clearance
            )
            return path

        rospy.logwarn(
            "Path smoothing rejected; raw min_clearance=%.3f m, optimized min_clearance=%.3f m.",
            raw_clearance, optimized_clearance
        )
        return raw_path

    def shortcut_path(self, path: List[State]) -> List[State]:
        if len(path) < 3:
            return path
        result = [path[0]]
        i = 0
        while i < len(path) - 1:
            farthest = min(len(path) - 1, i + self.shortcut_max_skip)
            next_index = i + 1
            for j in range(farthest, i + 1, -1):
                segment = self.interpolate_segment(path[i], path[j], self.resample_spacing)
                if self.path_collision_free(segment, skip_first=True):
                    next_index = j
                    break
            result.append(path[next_index])
            i = next_index
        return result

    def resample_path(self, path: List[State], spacing: float) -> List[State]:
        if len(path) < 2:
            return path
        spacing = max(0.01, spacing)
        resampled = [path[0]]
        for i in range(1, len(path)):
            prev = path[i - 1]
            current = path[i]
            dist = prev.distance_to(current)
            steps = max(1, int(math.ceil(dist / spacing)))
            for step in range(1, steps + 1):
                ratio = float(step) / float(steps)
                resampled.append(self.interpolate_state(prev, current, ratio))
        return resampled

    def smooth_path(self, path: List[State]) -> List[State]:
        if len(path) < 4:
            return path
        original = [State(state.x, state.y, state.yaw) for state in path]
        smoothed = [State(state.x, state.y, state.yaw) for state in path]

        for _ in range(self.smooth_iterations):
            total_change = 0.0
            for i in range(1, len(smoothed) - 1):
                old_x = smoothed[i].x
                old_y = smoothed[i].y
                smoothed[i].x += self.smooth_weight_data * (original[i].x - smoothed[i].x)
                smoothed[i].y += self.smooth_weight_data * (original[i].y - smoothed[i].y)
                smoothed[i].x += self.smooth_weight_smooth * (
                    smoothed[i - 1].x + smoothed[i + 1].x - 2.0 * smoothed[i].x
                )
                smoothed[i].y += self.smooth_weight_smooth * (
                    smoothed[i - 1].y + smoothed[i + 1].y - 2.0 * smoothed[i].y
                )
                if self.clearance_enabled:
                    margin = self.collision.clearance_margin(smoothed[i])
                    if margin < self.min_obstacle_clearance:
                        grad_x, grad_y = self.collision.clearance_gradient(smoothed[i])
                        grad_norm = math.hypot(grad_x, grad_y)
                        if grad_norm > 1e-6:
                            push = min(
                                self.max_clearance_push,
                                self.clearance_weight * (self.min_obstacle_clearance - margin)
                            )
                            smoothed[i].x += push * grad_x / grad_norm
                            smoothed[i].y += push * grad_y / grad_norm
                total_change += abs(old_x - smoothed[i].x) + abs(old_y - smoothed[i].y)
            if total_change < self.smooth_tolerance:
                break

        return smoothed

    def interpolate_segment(self, start: State, end: State, spacing: float) -> List[State]:
        dist = start.distance_to(end)
        steps = max(2, int(math.ceil(dist / max(0.01, spacing))))
        return [self.interpolate_state(start, end, float(i) / float(steps)) for i in range(steps + 1)]

    @staticmethod
    def interpolate_state(start: State, end: State, ratio: float) -> State:
        return State(
            start.x + ratio * (end.x - start.x),
            start.y + ratio * (end.y - start.y),
            normalize_angle(start.yaw + ratio * normalize_angle(end.yaw - start.yaw)),
        )

    def path_collision_free(self, path: List[State], skip_first: bool = False) -> bool:
        return not self.collision.trajectory_collides(path, skip_first=skip_first)

    def path_min_clearance(self, path: List[State]) -> float:
        if not self.clearance_enabled or not path:
            return float("inf")
        return min(self.collision.clearance_margin(state) for state in path)


class HybridParkingPlannerNode:
    def __init__(self):
        self.global_frame = rospy.get_param("~global_frame", "tianbot_mini/map")
        self.base_frame = rospy.get_param("~base_frame", "tianbot_mini/base_link")
        self.map_topic = rospy.get_param("~map_topic", "/tianbot_mini/map")
        self.pose_topic = rospy.get_param("~pose_topic", "/tianbot_mini/amcl_pose")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.path_topic = rospy.get_param("~path_topic", "/parking_path")
        self.adjust_colliding_goal = rospy.get_param("~adjust_colliding_goal", True)
        self.goal_adjust_search_radius = rospy.get_param("~goal_adjust_search_radius", 0.35)
        self.goal_adjust_step = rospy.get_param("~goal_adjust_step", 0.05)

        vehicle = VehicleParams(
            length=rospy.get_param("~vehicle_length", 0.28),
            width=rospy.get_param("~vehicle_width", 0.22),
            wheelbase=rospy.get_param("~wheelbase", 0.18),
            max_steering=math.radians(rospy.get_param("~max_steering_deg", 55.0)),
            motion_step=rospy.get_param("~motion_step", 0.10),
            obstacle_inflation=rospy.get_param("~obstacle_inflation", 0.16),
            allow_reverse=rospy.get_param("~allow_reverse", True),
        )
        self.collision = GridCollisionChecker(vehicle)
        self.planner = HybridParkingPlanner(vehicle, self.collision)
        self.post_processor = PathPostProcessor(self.collision)
        self.current_pose: Optional[State] = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self.pose_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=1)
        rospy.loginfo("Hybrid parking planner ready. Send a 2D Nav Goal to %s.", self.goal_topic)

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.collision.update_map(msg)

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self.current_pose = State(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def goal_callback(self, msg: PoseStamped) -> None:
        if self.collision.map is None:
            rospy.logwarn("No map received yet; cannot plan parking path.")
            self.publish_empty_path()
            return
        start = self.current_pose or self.lookup_pose_from_tf()
        if start is None:
            rospy.logwarn("No current pose received yet; cannot plan parking path.")
            self.publish_empty_path()
            return

        goal = State(msg.pose.position.x, msg.pose.position.y, yaw_from_quaternion(msg.pose.orientation))
        rospy.loginfo("Planning parking path: start=(%.2f, %.2f, %.1f deg), goal=(%.2f, %.2f, %.1f deg)",
                      start.x, start.y, math.degrees(start.yaw),
                      goal.x, goal.y, math.degrees(goal.yaw))
        self.publish_empty_path()
        goal = self.resolve_goal_collision(goal)
        if goal is None:
            rospy.logerr("Parking planner failed because no nearby collision-free goal pose was found.")
            self.publish_empty_path()
            return
        path_states = self.planner.plan(start, goal)
        if not path_states:
            rospy.logerr("Parking planner failed to produce a path.")
            self.publish_empty_path()
            return
        path_states = self.post_processor.process(path_states)
        self.path_pub.publish(self.to_path_msg(path_states))
        rospy.loginfo("Published parking path with %d poses.", len(path_states))

    def lookup_pose_from_tf(self) -> Optional[State]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.05)
            )
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Planner TF pose lookup failed: %s", exc)
            return None
        t = tf_msg.transform.translation
        return State(t.x, t.y, yaw_from_quaternion(tf_msg.transform.rotation))

    def resolve_goal_collision(self, goal: State) -> Optional[State]:
        if not self.adjust_colliding_goal or not self.collision.collides(goal):
            return goal

        rospy.logwarn(
            "Parking goal pose is in collision; searching nearby free goal within %.2f m.",
            self.goal_adjust_search_radius
        )
        best: Optional[State] = None
        best_score = float("inf")
        yaw_offsets = [0.0, math.radians(10.0), -math.radians(10.0), math.radians(20.0), -math.radians(20.0)]
        radius_steps = max(1, int(math.ceil(self.goal_adjust_search_radius / max(self.goal_adjust_step, 0.01))))

        for ri in range(1, radius_steps + 1):
            radius = ri * self.goal_adjust_step
            samples = max(12, int(math.ceil(2.0 * math.pi * radius / max(self.goal_adjust_step, 0.01))))
            for si in range(samples):
                theta = 2.0 * math.pi * float(si) / float(samples)
                x = goal.x + radius * math.cos(theta)
                y = goal.y + radius * math.sin(theta)
                for yaw_offset in yaw_offsets:
                    candidate = State(x, y, normalize_angle(goal.yaw + yaw_offset))
                    if self.collision.collides(candidate):
                        continue
                    score = radius + 0.05 * abs(yaw_offset)
                    clearance = self.collision.clearance_margin(candidate)
                    score -= 0.02 * max(0.0, clearance)
                    if score < best_score:
                        best = candidate
                        best_score = score
            if best is not None:
                rospy.logwarn(
                    "Adjusted parking goal to nearby free pose: (%.2f, %.2f, %.1f deg).",
                    best.x, best.y, math.degrees(best.yaw)
                )
                return best

        return None

    def to_path_msg(self, states: List[State]) -> Path:
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.global_frame
        for state in states:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.orientation = quaternion_from_yaw(state.yaw)
            msg.poses.append(pose)
        return msg

    def publish_empty_path(self) -> None:
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.global_frame
        self.path_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("hybrid_parking_planner")
    HybridParkingPlannerNode()
    rospy.spin()
