"""v0.14: verify a real timed MuJoCo external wrench is scheduled and cleared.

The fake stepping test only checks backend wiring; actual contact physics is
verified by the full MuJoCo experiment on Codespaces.
"""
from types import SimpleNamespace

import numpy as np

from turtlebot_integrated.config import Config
from turtlebot_integrated.scenarios import get_scenario, disturbance_wrench_world
from turtlebot_integrated.mujoco_backend import MuJoCoBackend


def test_scenario_schedule_and_no_leak_to_other_scenarios():
    s = get_scenario('disturbance', 7)
    assert 0 < s.disturbance_at < 22.4, 'disturbance must occur before nominal docking'
    assert np.all(disturbance_wrench_world(s, s.disturbance_at - .001) == 0)
    middle = disturbance_wrench_world(s, s.disturbance_at + .2)
    assert middle[1] == s.disturbance_force_y_n
    assert middle[5] == s.disturbance_yaw_torque_nm
    assert middle[0] == middle[2] == middle[3] == middle[4] == 0.0
    assert np.all(disturbance_wrench_world(s, s.disturbance_at + s.disturbance_duration_s) == 0)
    for name in ('nominal', 'noise', 'blocked', 'camera_loss'):
        assert np.all(disturbance_wrench_world(get_scenario(name), 9.1) == 0)


def test_backend_sets_force_then_clears_without_real_mujoco():
    class FakeMujoco:
        @staticmethod
        def mj_step(model, data):
            data.time += model.opt.timestep

    sim = MuJoCoBackend.__new__(MuJoCoBackend)
    sim.c = Config(dt=.08)
    sim.s = get_scenario('disturbance')
    sim.model = SimpleNamespace(opt=SimpleNamespace(timestep=.004))
    sim.data = SimpleNamespace(time=sim.s.disturbance_at,
                               ctrl=np.zeros(2), xfrc_applied=np.zeros((2, 6)))
    sim.body_id = 1
    sim.time = sim.s.disturbance_at
    sim.tick = 0
    sim.mj = FakeMujoco()
    sim.disturbance_applied_steps = 0
    sim.collision = False
    sim.get_truth = lambda: np.array([-1.75, -1.10, 0.0])
    sim.step(.10, .00)
    assert np.allclose(sim.data.xfrc_applied[1],
                       disturbance_wrench_world(sim.s, sim.s.disturbance_at))
    assert sim.disturbance_applied_steps == 1
    sim.data.time = sim.s.disturbance_at + sim.s.disturbance_duration_s + .01
    sim.time = sim.data.time
    sim.step(.10, .00)
    assert np.all(sim.data.xfrc_applied[1] == 0)
    assert sim.disturbance_applied_steps == 1


def test_nominal_wrench_always_zero():
    for t in (0., 9., 21., 40.):
        assert np.array_equal(disturbance_wrench_world(get_scenario('nominal'), t), np.zeros(6))
