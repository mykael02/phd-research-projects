import math
from pathlib import Path
import unittest

from diffdrive import (
    Limits, Pose, cell_center, cell_in_collision, clip_command,
    disc_in_collision, point_in_collision, rollout, step, wrap_angle,
)
from gridnav import load_grid

ROOT = Path(__file__).resolve().parents[1]
CIRCLE_TOLERANCE = 1e-9


def load(name):
    return load_grid(ROOT / "maps" / (name + ".txt"))


class GeometryReferenceTests(unittest.TestCase):
    """Shared reference cases for rule G on maps/detour.txt (width 3, height 5)."""

    def setUp(self):
        self.grid = load("detour")

    def test_point_reference_cases(self):
        cases = [
            ((0.5, 2.5), False),  # free cell centre
            ((1.0, 2.5), True),  # touches the obstacle in column 1
            ((0.0, 2.5), True),  # on the left outer edge
            ((3.0, 2.5), True),  # on the right outer edge
            ((1.5, 4.5), False),  # free cell under the wall
        ]
        for point, expected in cases:
            with self.subTest(point=point):
                self.assertEqual(point_in_collision(self.grid, point), expected)

    def test_disc_reference_cases(self):
        cases = [(0.4, False), (0.5, True), (0.6, True)]
        for radius, expected in cases:
            with self.subTest(radius=radius):
                self.assertEqual(disc_in_collision(self.grid, (0.5, 2.5), radius), expected)

    def test_zero_radius_matches_point_rule(self):
        reference = [(0.5, 2.5), (1.0, 2.5), (0.0, 2.5), (3.0, 2.5), (1.5, 4.5)]
        # Quarter-unit sweep, exact in binary floating point, covering edges,
        # obstacle faces and corners, and points outside the map.
        sweep = [(i / 4.0, j / 4.0) for i in range(-2, 15) for j in range(-2, 23)]
        for name in ("detour", "open", "disconnected"):
            grid = load(name)
            for point in reference + sweep:
                with self.subTest(map=name, point=point):
                    self.assertEqual(
                        disc_in_collision(grid, point, 0.0),
                        point_in_collision(grid, point),
                    )

    def test_narrow_passage_fails_with_footprint(self):
        # Row 4 is a one-unit passage below the wall: free for a point and a
        # small disc, in collision once the disc reaches the wall and the edge.
        self.assertFalse(disc_in_collision(self.grid, (1.5, 4.5), 0.4))
        self.assertTrue(disc_in_collision(self.grid, (1.5, 4.5), 0.5))

    def test_cell_mapping_and_g5(self):
        self.assertEqual(cell_center((2, 0)), (0.5, 2.5))
        self.assertEqual(cell_center((4, 1)), (1.5, 4.5))
        self.assertFalse(cell_in_collision(self.grid, (2, 0)))
        self.assertTrue(cell_in_collision(self.grid, (2, 0), 0.5))
        for cell in ((0, 1), (3, 1), (-1, 0), (0, -1), (5, 0), (0, 3)):
            with self.subTest(cell=cell):
                self.assertTrue(cell_in_collision(self.grid, cell))
                self.assertTrue(point_in_collision(self.grid, cell_center(cell)))

    def test_invalid_radius_and_non_finite_points(self):
        for radius in (-0.1, math.inf, math.nan):
            with self.subTest(radius=radius), self.assertRaises(ValueError):
                disc_in_collision(self.grid, (0.5, 2.5), radius)
        self.assertTrue(point_in_collision(self.grid, (math.nan, 2.5)))
        self.assertTrue(disc_in_collision(self.grid, (0.5, math.inf), 0.1))


class KinematicsTests(unittest.TestCase):
    def test_straight_line(self):
        pose = step(Pose(0.5, 2.5, 0.0), 1.0, 0.0, 0.5)
        self.assertEqual(pose, Pose(1.0, 2.5, 0.0))
        diagonal = step(Pose(0.0, 0.0, math.pi / 4), math.sqrt(2.0), 0.0, 1.0)
        self.assertAlmostEqual(diagonal.x, 1.0, places=12)
        self.assertAlmostEqual(diagonal.y, 1.0, places=12)
        self.assertAlmostEqual(diagonal.theta, math.pi / 4, places=12)

    def test_turn_on_the_spot(self):
        start = Pose(1.5, 2.5, 0.3)
        pose = step(start, 0.0, 1.0, 0.25)
        self.assertEqual((pose.x, pose.y), (start.x, start.y))
        self.assertAlmostEqual(pose.theta, 0.55, places=12)
        pose = step(start, 0.0, -2.0, 1.0)
        self.assertEqual((pose.x, pose.y), (start.x, start.y))
        self.assertAlmostEqual(pose.theta, wrap_angle(0.3 - 2.0), places=12)

    def test_full_circle_returns_to_start(self):
        # v = 1, omega = 1: radius 1, centre one unit to the left of the
        # heading (+y here). 100 exact arc steps make one full circle.
        grid = load("open")
        start = Pose(2.5, 1.5, 0.0)
        steps = 100
        dt = 2.0 * math.pi / steps
        limits = Limits(0.0, 1.0, 1.0)
        result = rollout(grid, start, [(1.0, 1.0)] * steps, dt, limits, radius=0.3)
        self.assertFalse(result.collided)
        self.assertEqual(result.clipped_steps, ())
        self.assertEqual(len(result.poses), steps + 1)
        for pose in result.poses:
            self.assertLess(abs(math.hypot(pose.x - 2.5, pose.y - 2.5) - 1.0), CIRCLE_TOLERANCE)
        half = result.poses[steps // 2]
        self.assertLess(abs(half.x - 2.5), CIRCLE_TOLERANCE)
        self.assertLess(abs(half.y - 3.5), CIRCLE_TOLERANCE)
        self.assertLess(abs(wrap_angle(half.theta - math.pi)), CIRCLE_TOLERANCE)
        end = result.poses[-1]
        self.assertLess(abs(end.x - start.x), CIRCLE_TOLERANCE)
        self.assertLess(abs(end.y - start.y), CIRCLE_TOLERANCE)
        self.assertLess(abs(wrap_angle(end.theta - start.theta)), CIRCLE_TOLERANCE)

    def test_arc_update_is_exact_under_step_splitting(self):
        start = Pose(1.0, 1.0, 0.3)
        whole = step(start, 1.0, 0.5, 1.0)
        split = start
        for _ in range(10):
            split = step(split, 1.0, 0.5, 0.1)
        self.assertAlmostEqual(whole.x, split.x, places=12)
        self.assertAlmostEqual(whole.y, split.y, places=12)
        self.assertAlmostEqual(whole.theta, split.theta, places=12)

    def test_invalid_step_inputs(self):
        for dt in (0.0, -0.1, math.inf, math.nan):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                step(Pose(0.0, 0.0, 0.0), 1.0, 0.0, dt)
        with self.assertRaises(ValueError):
            step(Pose(0.0, 0.0, 0.0), math.nan, 0.0, 0.1)


class LimitTests(unittest.TestCase):
    def setUp(self):
        self.limits = Limits(-0.5, 1.0, 2.0)

    def test_clip_command(self):
        cases = [
            ((3.0, 0.0), (1.0, 0.0, True)),
            ((-2.0, 0.0), (-0.5, 0.0, True)),
            ((0.5, 5.0), (0.5, 2.0, True)),
            ((0.5, -5.0), (0.5, -2.0, True)),
            ((9.0, -9.0), (1.0, -2.0, True)),
            ((1.0, -2.0), (1.0, -2.0, False)),  # exactly at the limits
            ((0.2, 0.3), (0.2, 0.3, False)),
        ]
        for (v, omega), expected in cases:
            with self.subTest(v=v, omega=omega):
                command = clip_command(v, omega, self.limits)
                self.assertEqual((command.v, command.omega, command.clipped), expected)

    def test_rollout_uses_clipped_commands_and_flags_them(self):
        grid = load("open")
        commands = [(0.1, 0.0), (5.0, 0.0), (0.1, 9.0)]
        result = rollout(grid, Pose(2.5, 2.0, 0.0), commands, 0.1, self.limits)
        self.assertEqual(result.clipped_steps, (2, 3))
        self.assertFalse(result.collided)
        self.assertAlmostEqual(result.poses[2].x, 2.5 + 0.01 + 0.1, places=12)
        self.assertAlmostEqual(result.poses[3].theta, 0.2, places=12)

    def test_invalid_limits_and_commands(self):
        for args in ((1.0, 0.0, 1.0), (0.0, 1.0, -1.0), (0.0, math.nan, 1.0), (0.0, 1.0, math.inf)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                Limits(*args)
        with self.assertRaises(ValueError):
            clip_command(math.nan, 0.0, self.limits)


class RolloutCollisionTests(unittest.TestCase):
    def setUp(self):
        self.grid = load("detour")
        self.limits = Limits(0.0, 2.0, 1.0)
        self.start = Pose(0.5, 2.5, 0.0)

    def test_large_dt_jumps_the_wall_unflagged(self):
        # Known limitation: only poses after each step are checked. One step
        # of length 2 carries the disc from column 0 across the 1-unit wall in
        # column 1 to column 2, and the end pose is free.
        result = rollout(self.grid, self.start, [(2.0, 0.0)], 1.0, self.limits, radius=0.2)
        self.assertFalse(result.collided)
        self.assertIsNone(result.collision_step)
        self.assertEqual(result.poses[-1], Pose(2.5, 2.5, 0.0))
        # The straight motion between the two poses does pass through the wall.
        self.assertTrue(point_in_collision(self.grid, (1.5, 2.5)))

    def test_small_dt_flags_the_wall(self):
        commands = [(2.0, 0.0)] * 10
        result = rollout(self.grid, self.start, commands, 0.1, self.limits, radius=0.2)
        self.assertTrue(result.collided)
        self.assertEqual(result.collision_step, 2)
        self.assertEqual(len(result.poses), 3)  # stops at the first colliding pose
        self.assertFalse(disc_in_collision(self.grid, (result.poses[1].x, result.poses[1].y), 0.2))
        self.assertTrue(disc_in_collision(self.grid, (result.poses[2].x, result.poses[2].y), 0.2))

    def test_start_in_collision_and_empty_commands(self):
        result = rollout(self.grid, Pose(1.5, 2.5, 0.0), [(1.0, 0.0)], 0.1, self.limits)
        self.assertEqual(result.collision_step, 0)
        self.assertEqual(result.poses, (Pose(1.5, 2.5, 0.0),))
        result = rollout(self.grid, self.start, [], 0.1, self.limits, radius=0.4)
        self.assertFalse(result.collided)
        self.assertEqual(result.poses, (self.start,))
        with self.assertRaises(ValueError):
            rollout(self.grid, self.start, [], 0.1, self.limits, radius=-1.0)

    def test_rollout_is_deterministic(self):
        commands = [(1.0, 0.7), (0.5, -0.3), (2.5, 0.0), (0.0, 1.0)] * 5
        start = Pose(1.5, 4.5, 0.0)
        first = rollout(self.grid, start, commands, 0.05, self.limits, radius=0.1)
        second = rollout(self.grid, start, commands, 0.05, self.limits, radius=0.1)
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
