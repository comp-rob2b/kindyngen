# SPDX-License-Identifier: MPL-2.0
from dataclasses import dataclass, field
from rdflib import URIRef, RDF
from kindynsyn.namespaces import KC_STAT
from kindynsyn.synthesizer.synthesizer import Traverser, Dispatcher
from kindynsyn.synthesizer.steps import ChainIndexState, q_expand


@dataclass
class JointState:
    q: URIRef | None = field(default=None)   # Joint position
    qd: URIRef | None = field(default=None)  # Joint velocity
    qdd: URIRef | None = field(default=None) # Joint acceleration
    tau: URIRef | None = field(default=None) # Joint force


class JointStep:
    def __init__(self, g, load, algo):
        self.g = g
        self.algo = algo
        self.sel_jnt_stat = load("select_joint_state.rq")

    def traverse(self):
        return Traverser(
            expander=q_expand,
            edge=[Dispatcher(None, self.configure_expand, None)])

    def configure_expand(self, state, parent, child):
        idx = state[child][ChainIndexState]

        joint_state = list(self.g.query(self.sel_jnt_stat, initBindings={
            "joint": idx.joint
        }))[0]

        s = JointState()
        s.q = joint_state["q"]
        s.qd = joint_state["qd"]
        s.qdd = joint_state["qdd"]
        s.tau = joint_state["tau"]
        state[child][JointState] = s

        self.algo["data"].extend([s.q, s.qd, s.qdd, s.tau])



@dataclass
class JointDynamicsState:
    inertia: URIRef | None = field(default=None)   # Joint inertia


class JointDynamicsStep:
    def __init__(self, g, load, kc_stat, algo):
        self.g = g
        self.kc_stat = kc_stat
        self.algo = algo

    def traverse(self):
        return Traverser(
            expander=q_expand,
            edge=[Dispatcher(None, self.configure_expand, None)])

    def configure_expand(self, state, parent, child):
        joint = state[child][ChainIndexState].joint

        inertia = self.kc_stat.joint_inertia(joint=joint)

        s = JointDynamicsState()
        s.inertia = inertia
        state[child][JointDynamicsState] = s

        self.algo["data"].extend([s.inertia])
