import math
from nav3d_native.algorithms import (
    LogOddsGrid,
    Pose2D,
    astar,
    normalize_angle,
    pure_pursuit,
    simplify_path,
    transform_between,
    transform_pose)


def test_angle_normalization():
    assert math.isclose(normalize_angle(3 * math.pi), math.pi, abs_tol=1e-9)


def test_log_odds_scan_marks_free_and_occupied():
    grid = LogOddsGrid(40, 40, 0.1, -2.0, -2.0)
    for _ in range(3):
        grid.update_scan(Pose2D(0.0, 0.0, 0.0), [1.0], 0.0, 1.0, 0.05, 2.0)
    data = grid.occupancy()
    free = grid.world_to_grid(0.5, 0.0)
    hit = grid.world_to_grid(1.0, 0.0)
    assert data[free[1] * grid.width + free[0]] == 0
    assert data[hit[1] * grid.width + hit[0]] == 100


def test_astar_avoids_wall_gap():
    width = height = 20
    data = [0] * (width * height)
    for y in range(height):
        if y != 10:
            data[y * width + 9] = 100
    path = astar(data, width, height, (2, 2), (17, 17), inflation_radius=0)
    assert path
    assert (9, 10) in path
    assert all(data[y * width + x] < 65 for x, y in path)


def test_smoothing_and_controller():
    path = simplify_path([(0.0, 0.0), (0.1, 0.0), (1.0, 0.0), (2.0, 0.0)])
    linear, angular, done = pure_pursuit(Pose2D(0.0, 0.0, 0.0), path)
    assert not done and linear > 0.0 and abs(angular) < 1e-6


def test_se2_transform_round_trip():
    source = Pose2D(2.0, -1.0, 0.4)
    target = Pose2D(-0.5, 3.0, -0.2)
    transform = transform_between(target, source)
    result = transform_pose(transform, source)
    assert math.isclose(result.x, target.x, abs_tol=1e-9)
    assert math.isclose(result.y, target.y, abs_tol=1e-9)
    assert math.isclose(result.yaw, target.yaw, abs_tol=1e-9)


def test_controller_rotates_before_driving_toward_rear_target():
    linear, angular, done = pure_pursuit(
        Pose2D(0.0, 0.0, math.pi), [(0.0, 0.0), (1.0, 0.0)])
    assert not done
    assert linear == 0.0
    assert abs(angular) > 1.0
