"""PyBullet quadruped driven by rope-like tendon tensions.

This script builds a minimalist quadruped with two hinge joints per leg and
maps inverse-kinematics targets into antagonistic tendon forces. Each tendon
produces torque about a joint via a fixed moment arm, so tension control acts
as the only source of actuation instead of direct motor position targets.
"""

import argparse
import math
from dataclasses import dataclass
from typing import Dict, Sequence, Tuple

import pybullet as p
import pybullet_data


@dataclass
class LegKinematics:
    hip_joint: int
    knee_joint: int
    foot_link: int
    default_target: Tuple[float, float, float]


@dataclass
class TendonGains:
    moment_arm: float = 0.03
    stiffness: float = 80.0
    damping: float = 2.0
    min_tension: float = 5.0
    max_tension: float = 250.0


class TendonQuadruped:
    def __init__(self, gui: bool = False):
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        flags = p.URDF_USE_SELF_COLLISION
        self.robot = p.loadURDF("quadruped.urdf", [0, 0, 0.35], useFixedBase=False, flags=flags)
        self.leg_defs = self._build_leg_defs()
        self.tendon_gains = TendonGains()
        self._disable_default_motors()

    def _build_leg_defs(self) -> Dict[str, LegKinematics]:
        name_to_index = {p.getJointInfo(self.robot, i)[1].decode(): i for i in range(p.getNumJoints(self.robot))}
        link_name_to_index = {p.getJointInfo(self.robot, i)[12].decode(): p.getJointInfo(self.robot, i)[0] for i in range(p.getNumJoints(self.robot))}
        return {
            "fl": LegKinematics(name_to_index["fl_hip"], name_to_index["fl_knee"], link_name_to_index["fl_shin"], (0.2, 0.12, -0.35)),
            "fr": LegKinematics(name_to_index["fr_hip"], name_to_index["fr_knee"], link_name_to_index["fr_shin"], (0.2, -0.12, -0.35)),
            "rl": LegKinematics(name_to_index["rl_hip"], name_to_index["rl_knee"], link_name_to_index["rl_shin"], (-0.2, 0.12, -0.35)),
            "rr": LegKinematics(name_to_index["rr_hip"], name_to_index["rr_knee"], link_name_to_index["rr_shin"], (-0.2, -0.12, -0.35)),
        }

    def _disable_default_motors(self):
        joint_indices = [p.getJointInfo(self.robot, i)[0] for i in range(p.getNumJoints(self.robot))]
        p.setJointMotorControlArray(
            self.robot,
            joint_indices,
            p.VELOCITY_CONTROL,
            forces=[0.0] * len(joint_indices),
        )

    def compute_leg_ik(self, leg: LegKinematics, target_pos_world: Sequence[float]) -> Tuple[float, float]:
        q = p.calculateInverseKinematics(
            self.robot,
            leg.foot_link,
            target_pos_world,
            lowerLimits=[-1.57, -1.57],
            upperLimits=[1.57, 1.57],
            jointRanges=[3.14, 3.14],
            restPoses=[0.0, 0.0],
            jointDamping=[0.1, 0.1],
        )
        return float(q[leg.hip_joint]), float(q[leg.knee_joint])

    def _tension_from_torque(self, desired_tau: float) -> Tuple[float, float]:
        r = self.tendon_gains.moment_arm
        tension_a = self.tendon_gains.min_tension
        tension_b = self.tendon_gains.min_tension
        if desired_tau >= 0:
            tension_a += desired_tau / r
        else:
            tension_b += -desired_tau / r
        return (
            max(self.tendon_gains.min_tension, min(self.tendon_gains.max_tension, tension_a)),
            max(self.tendon_gains.min_tension, min(self.tendon_gains.max_tension, tension_b)),
        )

    def _joint_torque_from_tension(self, tension_pair: Tuple[float, float]) -> float:
        return (tension_pair[0] - tension_pair[1]) * self.tendon_gains.moment_arm

    def apply_leg_control(self, leg: LegKinematics, target_angles: Tuple[float, float]):
        hip_state = p.getJointState(self.robot, leg.hip_joint)
        knee_state = p.getJointState(self.robot, leg.knee_joint)

        hip_error = target_angles[0] - hip_state[0]
        knee_error = target_angles[1] - knee_state[0]

        hip_tau = self.tendon_gains.stiffness * hip_error - self.tendon_gains.damping * hip_state[1]
        knee_tau = self.tendon_gains.stiffness * knee_error - self.tendon_gains.damping * knee_state[1]

        hip_tensions = self._tension_from_torque(hip_tau)
        knee_tensions = self._tension_from_torque(knee_tau)

        hip_command = self._joint_torque_from_tension(hip_tensions)
        knee_command = self._joint_torque_from_tension(knee_tensions)

        p.setJointMotorControlArray(
            self.robot,
            [leg.hip_joint, leg.knee_joint],
            controlMode=p.TORQUE_CONTROL,
            forces=[hip_command, knee_command],
        )

    def swing_trajectory(self, leg_name: str, step_count: int, amplitude: float = 0.04) -> Tuple[float, float, float]:
        base_pos = self.leg_defs[leg_name].default_target
        phase = 0 if leg_name in ("fl", "rr") else math.pi
        lift = amplitude if math.sin(0.02 * step_count + phase) > 0 else -amplitude
        return (
            base_pos[0] + amplitude * math.sin(0.02 * step_count + phase),
            base_pos[1],
            base_pos[2] + lift,
        )

    def step(self, step_count: int):
        for name, leg in self.leg_defs.items():
            target_world = self.swing_trajectory(name, step_count)
            desired_hip, desired_knee = self.compute_leg_ik(leg, target_world)
            self.apply_leg_control(leg, (desired_hip, desired_knee))
        p.stepSimulation()

    def shutdown(self):
        p.disconnect(self.client)



def run_simulation(gui: bool, steps: int, realtime: bool):
    robot = TendonQuadruped(gui=gui)
    try:
        if realtime:
            p.setRealTimeSimulation(1)
        for i in range(steps):
            robot.step(i)
            if gui and not realtime:
                p.setTimeStep(1.0 / 240.0)
        if gui:
            print("Simulation finished. Close the window to exit.")
            while p.isConnected():
                pass
    finally:
        robot.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Quadruped with rope-driven tendon control")
    parser.add_argument("--gui", action="store_true", help="Render the simulation window")
    parser.add_argument("--steps", type=int, default=2400, help="Number of simulation steps to run")
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="Use PyBullet real-time stepping (ignored in DIRECT mode)",
    )
    args = parser.parse_args()
    run_simulation(gui=args.gui, steps=args.steps, realtime=args.realtime)
