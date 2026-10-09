"""Bounded, coupled dual-TCP inverse kinematics and rigid-grasp path tracking.

FK and collision callbacks use the same fixed-base robot model. This is a local
geometric planner; it neither searches around obstacles nor commands hardware.
"""
from dataclasses import asdict, dataclass
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class PlanningFailure(RuntimeError):
    """No complete path satisfying the requested sampled checks was found."""


def transform(value, name='pose'):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-8, rtol=0)):
        raise ValueError(f'{name} must be a finite SE(3) transform')
    return t.copy()


def inverse(t):
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ t[:3, 3]
    return result


def interpolate(a, b, fraction):
    """Linear translation and shortest SO(3) geodesic, including pi rotations."""
    result = np.eye(4)
    result[:3, 3] = a[:3, 3] + fraction * (b[:3, 3] - a[:3, 3])
    delta = Rotation.from_matrix(b[:3, :3] @ a[:3, :3].T).as_rotvec()
    result[:3, :3] = Rotation.from_rotvec(fraction * delta).as_matrix() @ a[:3, :3]
    return result


def pose_error(actual, target):
    return (float(np.linalg.norm(actual[:3, 3] - target[:3, 3])),
            float(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude()))


def _positive(value, name):
    if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return float(value)


@dataclass(frozen=True)
class TrackingSettings:
    # Explicit numerical tolerances, not sensor accuracy or execution guarantees.
    ik_position_m: float = .0001
    ik_rotation_rad: float = .001
    path_position_m: float = .001
    path_rotation_rad: float = .01
    relative_position_m: float = .002
    relative_rotation_rad: float = .02
    translation_step_m: float = .005
    rotation_step_rad: float = .025
    joint_step_rad: float = .01
    max_nfev: int = 200
    max_samples: int = 10000

    def __post_init__(self):
        for name in ('ik_position_m', 'ik_rotation_rad', 'path_position_m',
                     'path_rotation_rad', 'relative_position_m', 'relative_rotation_rad',
                     'translation_step_m', 'rotation_step_rad', 'joint_step_rad'):
            _positive(getattr(self, name), name)
        for name in ('max_nfev', 'max_samples'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f'{name} must be a positive integer')


class DualTaskPlanner:
    """Joint optimization of both TCPs, including their shared waist coordinate.

    poses_at(q) returns (world_T_left_tcp, world_T_right_tcp).
    collision_free(q) MUST explicitly check self/environment collision and return
    bool; it is required even for an empty environment. Joint order is caller-defined.
    """

    def __init__(self, poses_at, limits_rad, collision_free, settings=None):
        self.poses_at, self.collision_free = poses_at, collision_free
        limits = np.asarray(limits_rad, dtype=float)
        if (limits.ndim != 2 or limits.shape[1] != 2 or not len(limits)
                or not np.isfinite(limits).all() or np.any(limits[:, 0] >= limits[:, 1])):
            raise ValueError('Expected finite ordered joint limits, shape (dof, 2)')
        if not callable(poses_at) or not callable(collision_free):
            raise ValueError('FK and collision callbacks are required')
        self.limits = limits.copy()
        self.settings = settings or TrackingSettings()
        if not isinstance(self.settings, TrackingSettings):
            raise ValueError('Expected TrackingSettings')

    def _q(self, values):
        q = np.asarray(values, dtype=float)
        if (q.shape != (len(self.limits),) or not np.isfinite(q).all()
                or np.any(q < self.limits[:, 0]) or np.any(q > self.limits[:, 1])):
            raise ValueError('Joint positions must be finite and within limits')
        return q.copy()

    def _poses(self, q):
        poses = self.poses_at(q.copy())
        if len(poses) != 2:
            raise ValueError('FK must return exactly two TCP poses')
        return tuple(transform(t, 'FK pose') for t in poses)

    def _clear(self, q):
        clear = self.collision_free(q.copy())
        if not isinstance(clear, (bool, np.bool_)):
            raise ValueError('Collision callback must return bool')
        if not clear:
            raise PlanningFailure('Collision at a checked configuration')

    def solve(self, target_poses, seed):
        """Find one local bounded solution; unreachable/colliding results raise.

        Optimizer trial states are not a motion path. Only the final candidate is
        collision checked here; use plan_object_motion for a checked path.
        """
        q0 = self._q(seed)
        if len(target_poses) != 2:
            raise ValueError('Exactly two target TCP poses are required')
        targets = tuple(transform(t, 'TCP target') for t in target_poses)
        s = self.settings

        def residual(q):
            errors = []
            for actual, target in zip(self._poses(q), targets):
                errors.extend((actual[:3, 3] - target[:3, 3]) / s.ik_position_m)
                errors.extend(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
                              / s.ik_rotation_rad)
            return np.asarray(errors)

        # Two six-dimensional tasks leave redundant joint directions. LSMR's
        # regularized trust-region solve avoids the exact-SVD solver's slow
        # convergence in this underdetermined system; scale joint columns too.
        solution = least_squares(residual, q0, bounds=self.limits.T, method='trf',
                                 tr_solver='lsmr', x_scale='jac',
                                 max_nfev=s.max_nfev, ftol=1e-10, xtol=1e-10, gtol=1e-10)
        q = self._q(solution.x)
        errors = [pose_error(a, b) for a, b in zip(self._poses(q), targets)]
        if any(p > s.ik_position_m or r > s.ik_rotation_rad for p, r in errors):
            raise PlanningFailure(f'Coupled IK did not meet both TCP tolerances: {errors}')
        self._clear(q)
        return q

    def plan_object_motion(self, start, object_poses, grasp_transforms):
        """Track world object poses with fixed object-to-left/right TCP transforms.

        Each object segment is subdivided, warm-started coupled IK is solved, then
        every joint-space edge is independently sampled for collision AND both TCP
        constraints. Any failure raises without returning a partial success.
        Returned positions are untimed geometric samples, not controller commands.
        """
        q = self._q(start)
        objects = [transform(t, 'object waypoint') for t in object_poses]
        if not objects or len(objects) > self.settings.max_samples:
            raise ValueError('Provide a nonempty bounded list of object waypoints')
        if len(grasp_transforms) != 2:
            raise ValueError('Exactly two fixed grasp transforms are required')
        grasps = [transform(t, 'object-to-TCP grasp') for t in grasp_transforms]
        expected_relative = inverse(grasps[0]) @ grasps[1]
        samples, object_samples, maximum = [], [], np.zeros(4)
        s = self.settings

        def check(qcheck, obj):
            if len(samples) >= s.max_samples:
                raise PlanningFailure('Path sample budget exceeded')
            poses = self._poses(qcheck)
            self._clear(qcheck)
            errors = [pose_error(a, obj @ g) for a, g in zip(poses, grasps)]
            if any(p > s.path_position_m or r > s.path_rotation_rad for p, r in errors):
                raise PlanningFailure(f'Interpolated path leaves TCP tracking tolerances: {errors}')
            relative = pose_error(inverse(poses[0]) @ poses[1], expected_relative)
            if relative[0] > s.relative_position_m or relative[1] > s.relative_rotation_rad:
                raise PlanningFailure(f'Interpolated path leaves relative TCP tolerances: {relative}')
            maximum[:] = np.maximum(maximum, [max(p for p, _ in errors),
                                              max(r for _, r in errors), *relative])
            samples.append(qcheck.tolist())
            object_samples.append(obj.tolist())

        check(q, objects[0])
        for begin, end in zip(objects, objects[1:]):
            distance, angle = pose_error(begin, end)
            count = max(1, math.ceil(distance / s.translation_step_m),
                        math.ceil(angle / s.rotation_step_rad))
            if count > s.max_samples:
                raise PlanningFailure('Object segment exceeds sample budget')
            previous_object = begin
            for k in range(1, count + 1):
                obj = interpolate(begin, end, k / count)
                next_q = self.solve([obj @ g for g in grasps], q)
                edges = max(1, math.ceil(float(np.max(np.abs(next_q - q))) / s.joint_step_rad))
                if edges + len(samples) > s.max_samples:
                    raise PlanningFailure('Joint edge exceeds sample budget')
                for j in range(1, edges + 1):
                    u = j / edges
                    check(q + u * (next_q - q), interpolate(previous_object, obj, u))
                q, previous_object = next_q, obj
        return dict(schema='omindos.community.dual_task.v1', simulation_only=True,
                    collision_check='sampled', continuous_collision_checked=False,
                    timed=False, hardware_executed=False,
                    settings=asdict(s),
                    positions_rad=samples, object_poses=object_samples,
                    object_to_tcp=[g.tolist() for g in grasps],
                    validated_samples=len(samples),
                    max_tcp_position_error_m=float(maximum[0]),
                    max_tcp_rotation_error_rad=float(maximum[1]),
                    max_relative_position_error_m=float(maximum[2]),
                    max_relative_rotation_error_rad=float(maximum[3]))


def plan_robot_object_motion(robot, start, object_poses, grasp_transforms, *, obstacles=(), settings=None):
    """MobileDualPitch adapter. Restore the caller's joints on success AND failure.

    The base is fixed. Grasp transforms constrain the TCPs; any payload collision
    geometry must already be registered on the robot by the caller.
    """
    if robot.cc is None:
        raise ValueError('A robot with collision checking enabled is required')
    if np.asarray(robot.get_jnt_ranges(robot.component)).shape != (15, 2):
        raise ValueError('MobileDualPitch adapter requires exactly 15 joint limits')
    saved = robot.get_jnt_values(robot.component).copy()
    obstacle_list = list(obstacles)

    def poses_at(q):
        robot.fk(robot.component, q)
        result = []
        for arm in ('lft_arm', 'rgt_arm'):
            position, rotation = robot.get_gl_tcp(arm)
            t = np.eye(4)
            t[:3, :3], t[:3, 3] = rotation, position
            result.append(t)
        return result

    def collision_free(q):
        robot.fk(robot.component, q)
        return not robot.is_collided(obstacle_list=obstacle_list)

    try:
        planner = DualTaskPlanner(poses_at, robot.get_jnt_ranges(robot.component), collision_free, settings)
        result = planner.plan_object_motion(start, object_poses, grasp_transforms)
        result['joint_names'] = (['waist_pitch_joint'] + [f'left_joint_{i}' for i in range(1, 8)]
                                 + [f'right_joint_{i}' for i in range(1, 8)])
        result['base_position_m'] = np.asarray(robot.pos).tolist()
        result['base_rotation'] = np.asarray(robot.rotmat).tolist()
        return result
    finally:
        robot.fk(robot.component, saved)
