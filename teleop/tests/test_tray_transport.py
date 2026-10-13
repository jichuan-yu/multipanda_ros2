"""Offline geometry, playback and nominal collision regressions."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tray_transport_plan import (
    build_plan, DEFAULT_SCENE, DEFAULT_MODEL, HOME_POSITION, home_poses,
    validate_reachability, SegmentPlayer, errors, GraspArrival, grasp_region_status, transit_region_status, angle, quaternion_wxyz_to_matrix,
)


class TrayPlanTests(unittest.TestCase):
    def setUp(self):
        self.plan = build_plan()

    def test_model_derived_contacts_and_support_heights(self):
        self.assertEqual(self.plan.walls, ['back_wall', 'front_wall'])
        stages = {s.name: s for s in self.plan.stages}
        np.testing.assert_allclose(stages['close'].target[:, :3],
                                   [[.600104189, .495, .229409115], [.600104189, .205, .229409115]], atol=1e-6)
        np.testing.assert_allclose(stages['place'].target[:, :3],
                                   [[.600104189, -.205, .379409115], [.600104189, -.495, .379409115]], atol=1e-6)
        self.assertAlmostEqual(self.plan.metadata['seat_top'], .35)
        self.assertAlmostEqual(self.plan.open_width, .08)

    def test_center_contact_and_no_half_turn(self):
        grasp = self.plan.stages[3].target
        for arm in range(2):
            rot = quaternion_wxyz_to_matrix(np.roll(grasp[arm, 3:], 1))
            offset = self.plan.metadata['grasp_acceptance']['contacts'][arm]['pad_offset_tcp']
            center = grasp[arm, :3]+rot @ offset
            np.testing.assert_allclose(center, [.6, [.495, .205][arm], .23], atol=1e-8)
            self.assertLess(angle(grasp[arm, 3:], home_poses()[arm, 3:]), np.deg2rad(11))
        # Live planning picks independently from both symmetric gripper orientations.
        start = home_poses(); start[1, 3:] = [0., 1., 0., 0.]
        changed = build_plan(start=start)
        self.assertLess(angle(changed.stages[3].target[1, 3:], start[1, 3:]), np.deg2rad(11))

    def test_intermediate_gates_allow_tracking_residual_but_require_clearance(self):
        stages = {s.name: s for s in self.plan.stages}
        actual = stages['lift'].target.copy(); actual[:, 2] -= .025
        self.assertGreater(errors(actual, stages['lift'].target)[0], .003)
        self.assertTrue(transit_region_status(self.plan, stages['lift'], actual)[0])
        actual[1, 2] -= .02
        self.assertFalse(transit_region_status(self.plan, stages['lift'], actual)[0])
        actual = stages['transport'].target.copy(); actual[:, 1] += .015; actual[:, 2] -= .012
        self.assertTrue(transit_region_status(self.plan, stages['transport'], actual)[0])
        actual[0, 1] += .025
        self.assertFalse(transit_region_status(self.plan, stages['transport'], actual)[0])
        actual = stages['place'].target.copy()
        self.assertFalse(transit_region_status(self.plan, stages['transport'], actual)[0])
        self.assertFalse(transit_region_status(self.plan, stages['place'], actual)[0])
        # Reanchoring after a high grasp must not invent tray clearance.
        for stage in self.plan.stages:
            if stage.cooperative:
                stage.target[:, 2] += .01
        actual = stages['place'].target.copy(); actual[:, 2] += .02
        self.assertFalse(transit_region_status(self.plan, stages['lift'], actual)[0])

    def test_relative_pose_preserved_while_carrying(self):
        expected = self.plan.stages[3].target[1, :3]-self.plan.stages[3].target[0, :3]
        for stage in self.plan.stages:
            if stage.cooperative:
                np.testing.assert_allclose(stage.target[1, :3]-stage.target[0, :3], expected)
                np.testing.assert_allclose(stage.target[0, 3:], stage.target[1, 3:])
        names = [s.name for s in self.plan.stages if s.cooperative]
        self.assertEqual(names, ['enable_cooperation', 'lift', 'transport', 'place', 'release'])

    def test_scene_changes_recompute_points(self):
        scene = yaml.safe_load(DEFAULT_SCENE.read_text())
        for obj in scene['collision_objects']:
            obj['pose']['position'][0] += .02
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'scene.yaml'
            path.write_text(yaml.safe_dump(scene))
            changed = build_plan(path)
        for original, new in zip(self.plan.stages, changed.stages):
            np.testing.assert_allclose(new.target[:, 0]-original.target[:, 0], .02)

    def test_all_segments_reachable_and_bad_target_rejected(self):
        count = validate_reachability(self.plan, home_poses(), np.tile(HOME_POSITION, (2, 1)))
        self.assertGreater(count, 300)
        bad = copy.deepcopy(self.plan)
        bad.stages[0].target[:, 0] = 2.
        with self.assertRaisesRegex(ValueError, 'unreachable'):
            validate_reachability(bad, home_poses(), np.tile(HOME_POSITION, (2, 1)), spacing=.1)

    def test_high_grasp_retains_carry_reach_margin(self):
        raised = copy.deepcopy(self.plan)
        for stage in raised.stages:
            if stage.name in ('lift', 'transport', 'place', 'release', 'disable_cooperation', 'retreat'):
                stage.target[:, 2] += .018
        validate_reachability(raised, home_poses(), np.tile(HOME_POSITION, (2, 1)))

    def test_feedback_governor_keeps_advancing_and_recovers_smoothly(self):
        start = home_poses(); end = start.copy(); end[:, 0] += .1
        player = SegmentPlayer(start, end, .04)
        for _ in range(60):
            player.advance(player.target, .01)
        before = player.elapsed
        delayed = player.target.copy(); delayed[1, 0] -= .4
        previous_target = player.target.copy()
        player.advance(delayed, .1)
        self.assertAlmostEqual(player.scale, .2)
        self.assertAlmostEqual(player.elapsed-before, .02)
        self.assertGreater(np.linalg.norm(player.target-previous_target), 0.)
        player.advance(player.target, .01)
        self.assertAlmostEqual(player.scale, .22)

    def test_large_position_and_rotation_errors_cannot_freeze_reference(self):
        start = home_poses(); end = start.copy(); end[:, 0] += .1
        player = SegmentPlayer(start, end, .04, min_playback_scale=.1)
        blocked = start.copy(); blocked[:, 1] -= .5; blocked[:, 3:] = [0, 0, 0, 1]
        last_elapsed = 0.
        for _ in range(int((player.duration/.1+1)/.01)):
            player.advance(blocked, .01)
            self.assertGreater(player.elapsed, last_elapsed)
            last_elapsed = player.elapsed
            if player.done:
                break
        self.assertTrue(player.done)
        np.testing.assert_allclose(player.target, end, atol=1e-10)
        # This only completes the reference, not actual robot arrival.
        self.assertGreater(errors(blocked, player.goal)[0], .4)
        for scale in [0., -.1, 1.1, float('nan')]:
            with self.assertRaisesRegex(ValueError, 'min_playback_scale'):
                SegmentPlayer(start, end, .04, min_playback_scale=scale)

    def test_speed_bound_and_both_arm_endpoint(self):
        start = home_poses(); end = start.copy(); end[0, 0] += .1; end[1, 0] += .05
        player = SegmentPlayer(start, end, .04)
        previous = start.copy()
        for _ in range(2000):
            target = player.advance(previous, .01)
            self.assertLessEqual(max(np.linalg.norm(target[:, :3]-previous[:, :3], axis=1))/.01,
                                 .04001)
            previous = target.copy()
            if player.done:
                break
        self.assertTrue(player.done)
        np.testing.assert_allclose(player.target, end, atol=1e-10)

    def test_short_ramps_remove_long_endpoint_crawl(self):
        start = home_poses(); end = start.copy(); end[:, 2] += .23
        player = SegmentPlayer(start, end, .008, ramp_time=.5)
        self.assertAlmostEqual(player.duration, 29.25)
        self.assertAlmostEqual(player.progress_at(3.)*.23, .022)
        self.assertAlmostEqual(player.progress_at(.5), .5*.5*player.progress_rate)
        self.assertAlmostEqual(player.progress_at(player.duration-.5),
                               1-.5*.5*player.progress_rate)
        # Smooth ramp endpoints: zero velocity at either endpoint, speed cap at join.
        h = 1e-5
        self.assertLess(player.progress_at(h)/h, 1e-7)
        self.assertLess((1-player.progress_at(player.duration-h))/h, 1e-7)
        velocity = (player.progress_at(.5+h)-player.progress_at(.5-h))/(2*h)*.23
        self.assertAlmostEqual(velocity, .008, places=7)

    def test_zero_and_short_segments_remain_finite(self):
        start = home_poses()
        same = SegmentPlayer(start, start, .04)
        self.assertTrue(same.done)
        np.testing.assert_allclose(same.advance(start, .01), start)
        short = start.copy(); short[:, 0] += .0001
        player = SegmentPlayer(start, short, .04)
        self.assertAlmostEqual(player.duration, 1.)
        values = [player.progress_at(t) for t in np.linspace(0, player.duration, 101)]
        self.assertTrue(np.isfinite(values).all())
        self.assertTrue(np.all(np.diff(values) >= 0))
        for ramp in [0., -.1, float('nan')]:
            with self.assertRaises(ValueError):
                SegmentPlayer(start, short, .04, ramp_time=ramp)

    def test_raised_grasp_requires_both_pad_overlaps_and_alignment(self):
        nominal = next(s.target for s in self.plan.stages if s.name == 'descend_to_grasp')
        raised = nominal.copy(); raised[:, 2] += .010
        valid, overlaps = grasp_region_status(self.plan, raised)
        self.assertTrue(valid)
        self.assertGreater(min(overlaps), .013)
        too_high = raised.copy(); too_high[1, 2] += .012
        self.assertFalse(grasp_region_status(self.plan, too_high)[0])
        lateral = raised.copy(); lateral[0, 0] += .005
        self.assertFalse(grasp_region_status(self.plan, lateral)[0])
        below = nominal.copy(); below[:, 2] -= .004
        self.assertFalse(grasp_region_status(self.plan, below)[0])
        rotated = raised.copy(); rotated[0, 3:] = [0., 0., 0., 1.]
        self.assertFalse(grasp_region_status(self.plan, rotated)[0])

    def test_grasp_requires_stationary_dwell_and_resets_outside_region(self):
        nominal = next(s.target for s in self.plan.stages if s.name == 'descend_to_grasp')
        actual = nominal.copy(); actual[:, 2] += .009
        arrival = GraspArrival(self.plan)
        self.assertFalse(arrival.update(actual, 0.))
        self.assertFalse(arrival.update(actual, .2))
        self.assertTrue(arrival.update(actual, .5))
        invalid = actual.copy(); invalid[1, 2] += .015
        self.assertFalse(arrival.update(invalid, .6))
        self.assertFalse(arrival.update(actual, .7))
        self.assertFalse(arrival.update(actual, .9))
        self.assertTrue(arrival.update(actual, 1.2))
        moving = GraspArrival(self.plan)
        for i in range(50):
            point = actual.copy(); point[:, 2] -= i*.00008
            self.assertFalse(moving.update(point, i*.01))

    def test_grasp_window_tracks_wall_geometry_and_rejects_impossible_overlap(self):
        scene = yaml.safe_load(DEFAULT_SCENE.read_text())
        tray = next(o for o in scene['collision_objects'] if o['id'] == 'tray')
        for wall in tray['parts']:
            if wall['id'].endswith('_wall'):
                wall['dimensions'][2] = .06
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'scene.yaml'; path.write_text(yaml.safe_dump(scene))
            plan = build_plan(path)
            actual = next(s.target.copy() for s in plan.stages if s.name == 'descend_to_grasp')
            actual[:, 2] += .020
            self.assertTrue(grasp_region_status(plan, actual)[0])
            scene['tray_transport_input']['grasp_min_overlap'] = .03
            path.write_text(yaml.safe_dump(scene))
            with self.assertRaisesRegex(ValueError, 'exceeds wall/pad height'):
                build_plan(path)

    def test_discrete_nominal_mesh_contacts_and_obstacle_rejection(self):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest('MuJoCo unavailable')
        from tray_transport_collision import ReferenceCollisionCheck
        checker = ReferenceCollisionCheck(self.plan, DEFAULT_SCENE, DEFAULT_MODEL)
        validate_reachability(self.plan, home_poses(), np.tile(HOME_POSITION, (2, 1)),
                              collision_check=checker)
        self.assertGreater(checker.count, 300)
        scene = yaml.safe_load(DEFAULT_SCENE.read_text())
        scene['collision_objects'].append(dict(id='blocking_box', type='Box',
            dimensions=[.15, .15, .2], pose=dict(position=[.535, .495, .4]), movable=False))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'blocked.yaml'; path.write_text(yaml.safe_dump(scene))
            checker = ReferenceCollisionCheck(self.plan, path, DEFAULT_MODEL)
            with self.assertRaisesRegex(ValueError, 'collision'):
                validate_reachability(self.plan, home_poses(), np.tile(HOME_POSITION, (2, 1)),
                                      collision_check=checker)


if __name__ == '__main__':
    unittest.main()
