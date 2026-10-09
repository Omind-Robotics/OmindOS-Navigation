"""ROS-independent algorithm core used by the native ROS 2 nodes."""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Iterable, List, Optional, Sequence, Tuple


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def bresenham(x0: int, y0: int, x1: int, y1: int) -> Iterable[Tuple[int, int]]:
    dx, sx = abs(x1 - x0), 1 if x0 < x1 else -1
    dy, sy = -abs(y1 - y0), 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def transform_pose(transform: Pose2D, pose: Pose2D) -> Pose2D:
    """Apply an SE(2) transform to a pose."""
    c, s = math.cos(transform.yaw), math.sin(transform.yaw)
    return Pose2D(
        transform.x + c * pose.x - s * pose.y,
        transform.y + s * pose.x + c * pose.y,
        normalize_angle(transform.yaw + pose.yaw),
    )


def transform_between(target_pose: Pose2D, source_pose: Pose2D) -> Pose2D:
    """Return the SE(2) transform that maps source_pose onto target_pose."""
    yaw = normalize_angle(target_pose.yaw - source_pose.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    return Pose2D(
        target_pose.x - (c * source_pose.x - s * source_pose.y),
        target_pose.y - (s * source_pose.x + c * source_pose.y),
        yaw,
    )


def inverse_transform(transform: Pose2D) -> Pose2D:
    c, s = math.cos(transform.yaw), math.sin(transform.yaw)
    return Pose2D(
        -(c * transform.x + s * transform.y),
        -(-s * transform.x + c * transform.y),
        normalize_angle(-transform.yaw),
    )


class LogOddsGrid:
    def __init__(self, width: int, height: int, resolution: float,
                 origin_x: float, origin_y: float):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        size = width * height
        self.log_odds = [0] * size
        self.seen = [0] * size

    def index(self, gx: int, gy: int) -> int:
        return gy * self.width + gx

    def inside(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        return int(math.floor((x - self.origin_x) / self.resolution)
                   ), int(math.floor((y - self.origin_y) / self.resolution))

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        return (self.origin_x + (gx + 0.5) * self.resolution,
                self.origin_y + (gy + 0.5) * self.resolution)

    def update_scan(self, pose: Pose2D, ranges: Sequence[float], angle_min: float,
                    angle_increment: float, range_min: float, range_max: float) -> None:
        sx, sy = self.world_to_grid(pose.x, pose.y)
        if not self.inside(sx, sy):
            return
        for beam, measured in enumerate(ranges):
            if not math.isfinite(measured) or measured < range_min:
                continue
            hit = measured < range_max * 0.995
            distance = min(measured, range_max)
            angle = pose.yaw + angle_min + beam * angle_increment
            ex = pose.x + distance * math.cos(angle)
            ey = pose.y + distance * math.sin(angle)
            gx, gy = self.world_to_grid(ex, ey)
            cells = list(bresenham(sx, sy, gx, gy))
            free_cells = cells[:-1] if hit else cells
            for cx, cy in free_cells:
                if self.inside(cx, cy):
                    idx = self.index(cx, cy)
                    self.log_odds[idx] = max(-20, self.log_odds[idx] - 2)
                    self.seen[idx] += 1
            if hit and cells:
                cx, cy = cells[-1]
                if self.inside(cx, cy):
                    idx = self.index(cx, cy)
                    self.log_odds[idx] = min(20, self.log_odds[idx] + 6)
                    self.seen[idx] += 1

    def occupancy(self) -> List[int]:
        result: List[int] = []
        for observed, value in zip(self.seen, self.log_odds):
            if observed == 0:
                result.append(-1)
            elif value >= 3:
                result.append(100)
            elif value <= 0:
                result.append(0)
            else:
                result.append(50)
        return result


def occupied_near(data: Sequence[int], width: int, height: int,
                  gx: int, gy: int, radius: int = 1) -> bool:
    for y in range(max(0, gy - radius), min(height, gy + radius + 1)):
        offset = y * width
        for x in range(max(0, gx - radius), min(width, gx + radius + 1)):
            if data[offset + x] >= 65:
                return True
    return False


def scan_match(data: Sequence[int], width: int, height: int, resolution: float,
               origin_x: float, origin_y: float, predicted: Pose2D,
               ranges: Sequence[float], angle_min: float, angle_increment: float,
               range_max: float, xy_window: float = 0.6,
               yaw_window: float = 0.16) -> Tuple[Pose2D, float]:
    best = predicted
    best_score = -1.0e30
    xy_steps = int(round(xy_window / resolution))
    yaw_step = 0.04
    yaw_steps = int(round(yaw_window / yaw_step))
    sampled = [(i, r) for i, r in enumerate(ranges[::4])
               if math.isfinite(r) and 0.08 < r < range_max * 0.995]
    if len(sampled) < 8:
        return best, best_score
    for ix in range(-xy_steps, xy_steps + 1):
        x = predicted.x + ix * resolution
        for iy in range(-xy_steps, xy_steps + 1):
            y = predicted.y + iy * resolution
            for it in range(-yaw_steps, yaw_steps + 1):
                yaw = normalize_angle(predicted.yaw + it * yaw_step)
                score = -0.12 * (ix * ix + iy * iy) - 0.2 * (it * it)
                for sample_index, measured in sampled:
                    beam_index = sample_index * 4
                    angle = yaw + angle_min + beam_index * angle_increment
                    ex = x + measured * math.cos(angle)
                    ey = y + measured * math.sin(angle)
                    gx = int(math.floor((ex - origin_x) / resolution))
                    gy = int(math.floor((ey - origin_y) / resolution))
                    if not (0 <= gx < width and 0 <= gy < height):
                        score -= 1.0
                    elif occupied_near(data, width, height, gx, gy, 1):
                        score += 3.0
                    elif data[gy * width + gx] < 0:
                        score -= 0.3
                    else:
                        score -= 0.8
                if score > best_score:
                    best_score = score
                    best = Pose2D(x, y, yaw)
    return best, best_score


def inflate_obstacles(data: Sequence[int], width: int,
                      height: int, radius_cells: int) -> List[bool]:
    blocked = [False] * (width * height)
    occupied = [(i % width, i // width) for i, value in enumerate(data) if value >= 65]
    r2 = radius_cells * radius_cells
    for ox, oy in occupied:
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= r2:
                    x, y = ox + dx, oy + dy
                    if 0 <= x < width and 0 <= y < height:
                        blocked[y * width + x] = True
    return blocked


def astar(data: Sequence[int], width: int, height: int,
          start: Tuple[int, int], goal: Tuple[int, int],
          inflation_radius: int = 2) -> List[Tuple[int, int]]:
    if not (0 <= start[0] < width and 0 <= start[1] < height and 0 <=
            goal[0] < width and 0 <= goal[1] < height):
        return []
    blocked = inflate_obstacles(data, width, height, inflation_radius)
    # A pose estimate may lie just inside an inflated safety band. Clear a
    # robot-sized disk around both endpoints so replanning can leave/enter it.
    for center in (start, goal):
        for dy in range(-inflation_radius, inflation_radius + 1):
            for dx in range(-inflation_radius, inflation_radius + 1):
                x, y = center[0] + dx, center[1] + dy
                inside_map = 0 <= x < width and 0 <= y < height
                inside_disk = (
                    dx * dx + dy * dy <= inflation_radius * inflation_radius)
                if inside_map and inside_disk:
                    blocked[y * width + x] = False
    frontier = [(0.0, start)]
    came_from = {start: None}
    cost = {start: 0.0}
    motions = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
               (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            break
        for dx, dy, step in motions:
            nxt = current[0] + dx, current[1] + dy
            if not (0 <= nxt[0] < width and 0 <= nxt[1] < height):
                continue
            idx = nxt[1] * width + nxt[0]
            if blocked[idx]:
                continue
            unknown_penalty = 2.5 if data[idx] < 0 else 0.0
            new_cost = cost[current] + step + unknown_penalty
            if nxt not in cost or new_cost < cost[nxt]:
                cost[nxt] = new_cost
                heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(frontier, (new_cost + heuristic, nxt))
                came_from[nxt] = current
    if goal not in came_from:
        return []
    path = []
    current: Optional[Tuple[int, int]] = goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def simplify_path(points: Sequence[Tuple[float, float]],
                  min_spacing: float = 0.25) -> List[Tuple[float, float]]:
    if len(points) <= 2:
        return list(points)
    spaced = [points[0]]
    for point in points[1:-1]:
        if math.hypot(point[0] - spaced[-1][0], point[1] - spaced[-1][1]) >= min_spacing:
            spaced.append(point)
    spaced.append(points[-1])
    if len(spaced) <= 2:
        return spaced
    smooth = [spaced[0]]
    for i in range(1, len(spaced) - 1):
        smooth.append(((spaced[i - 1][0] + 2.0 * spaced[i][0] + spaced[i + 1][0]) / 4.0,
                       (spaced[i - 1][1] + 2.0 * spaced[i][1] + spaced[i + 1][1]) / 4.0))
    smooth.append(spaced[-1])
    return smooth


def pure_pursuit(pose: Pose2D, path: Sequence[Tuple[float, float]], lookahead: float = 0.30,
                 max_linear: float = 0.55, max_angular: float = 1.4) -> Tuple[float, float, bool]:
    if not path:
        return 0.0, 0.0, True
    if math.hypot(path[-1][0] - pose.x, path[-1][1] - pose.y) < 0.22:
        return 0.0, 0.0, True
    nearest = min(
        range(
            len(path)),
        key=lambda i: math.hypot(
            path[i][0] -
            pose.x,
            path[i][1] -
            pose.y))
    target = path[-1]
    for point in path[nearest:]:
        if math.hypot(point[0] - pose.x, point[1] - pose.y) >= lookahead:
            target = point
            break
    heading = math.atan2(target[1] - pose.y, target[0] - pose.x)
    error = normalize_angle(heading - pose.yaw)
    angular = clamp(2.2 * error, -max_angular, max_angular)
    # Do not creep forward while the target is behind the robot. Squaring the
    # heading factor slows corner entry and prevents lookahead corner cutting.
    heading_factor = max(0.0, math.cos(error))
    linear = max_linear * heading_factor * heading_factor
    return linear, angular, False
