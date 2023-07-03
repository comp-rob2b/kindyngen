# SPDX-License-Identifier: MPL-2.0
from rdflib import Literal, RDF
from kindynsyn.rdflib_tools import uuid_ref
from kindynsyn.namespaces import KC_STAT
from kindynsyn.synthesizer import Traverser, Dispatcher
from kindynsyn.synthesizer.steps import JointState, q_expand
from .namespace import MY_IF


class RobotInterfaceMeasurementStep:
    def __init__(self, g, algo, jnt_idx):
        self.g = g
        self.algo = algo
        self.jnt_idx = jnt_idx

    def traverse(self):
        return Traverser(
            expander=q_expand,
            edge=[Dispatcher(None, None, self.compute_edge)]
        )

    def compute_edge(self, state, parent, child):
        jmot = state[child][JointState]

        pos = self.joint_position_to_solver(self.jnt_idx[jmot.q], jmot.q)
        vel = self.joint_velocity_to_solver(self.jnt_idx[jmot.qd], jmot.qd)

        self.algo["func"].extend([pos, vel])

    def joint_position_to_solver(self, src_index, destination):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], MY_IF["JointPositionToSolver"]))
        self.g.add((id_, MY_IF["source-index"], Literal(src_index)))
        self.g.add((id_, MY_IF["destination"], destination))
        return id_

    def joint_velocity_to_solver(self, src_index, destination):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], MY_IF["JointVelocityToSolver"]))
        self.g.add((id_, MY_IF["source-index"], Literal(src_index)))
        self.g.add((id_, MY_IF["destination"], destination))
        return id_


class RobotInterfaceCommandStep:
    def __init__(self, g, algo, src_cls, jnt_idx):
        self.g = g
        self.algo = algo
        self.src_cls = src_cls
        self.jnt_idx = jnt_idx

    def traverse(self):
        return Traverser(
            expander=q_expand,
            edge=[Dispatcher(None, None, self.compute_edge)]
        )

    def compute_edge(self, state, parent, child):
        jnt = state[child][JointState]

        tau = self.joint_force_from_solver(jnt.tau, self.jnt_idx[jnt.tau])

        self.algo["func"].extend([tau])

    def joint_force_from_solver(self, source, dst_index):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], MY_IF["JointForceFromSolver"]))
        self.g.add((id_, MY_IF["source"], source))
        self.g.add((id_, MY_IF["destination-index"], Literal(dst_index)))
        return id_
