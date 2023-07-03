# SPDX-License-Identifier: MPL-2.0
from dataclasses import dataclass, field
from rdflib import URIRef
import numpy as np
from kindynsyn.namespaces import GEOM_COORD, RBDYN_COORD
from kindynsyn.synthesizer.synthesizer import Traverser, Dispatcher
from kindynsyn.synthesizer.steps import ChainIndexState, JointDynamicsState, \
    PositionPropagationState, q_expand


@dataclass
class RigidBodyInertiaState:
    m_scr_prox: URIRef | None = field(default=None)     # Screw/rigid-body inertia of segment's body about local root's origin
    # Intermediate
    m_scr_com: URIRef | None = field(default=None)      # Screw/rigid-body inertia about centre-of-mass point
    r_com: URIRef | None = field(default=None)          # Position of centre-of-mass point w.r.t. proximal link frame's origin as seen by proximal link frame


def skew(v):
    return np.array([[ 0.0 , -v[2],  v[1]],
                     [ v[2],  0.0 , -v[0]],
                     [-v[1],  v[0],  0.0 ]])

class RigidBodyInertiaStep:
    q_inertia = """
    SELECT ?inertia WHERE {
        ?current ^geom-ent:simplices / ^rbdyn-ent:of-body ?inertia .
    }
    """

    def __init__(self, g, load, algo, dyn):
        self.g = g
        self.dyn = dyn
        self.sel_inr = load("select_inertia.rq")
        self.algo = algo

    def traverse(self):
        return Traverser(
            expander=q_expand,
            node=[Dispatcher(None, self.configure, None)]
        )

    def configure(self, state, node):
        idx = state[node][ChainIndexState]

        inertia = list(self.g.query(self.sel_inr, initBindings={
            "frame": idx.frm_prox
        }))[0]

        mass = float(self.g.value(inertia["rbi"], RBDYN_COORD["mass"]))

        com_x = float(self.g.value(inertia["tx_com"], GEOM_COORD["x"]))
        com_y = float(self.g.value(inertia["tx_com"], GEOM_COORD["y"]))
        com_z = float(self.g.value(inertia["tx_com"], GEOM_COORD["z"]))

        ixx_com = float(self.g.value(inertia["rbi"], RBDYN_COORD["ixx"]))
        iyy_com = float(self.g.value(inertia["rbi"], RBDYN_COORD["iyy"]))
        izz_com = float(self.g.value(inertia["rbi"], RBDYN_COORD["izz"]))
        ixy_com = float(self.g.value(inertia["rbi"], RBDYN_COORD["ixy"]))
        ixz_com = float(self.g.value(inertia["rbi"], RBDYN_COORD["ixz"]))
        iyz_com = float(self.g.value(inertia["rbi"], RBDYN_COORD["iyz"]))

        # Translate rigid-body inertia:
        # - mass remains the same
        # - moment of mass: m c
        # - rotational inertia: I_c + m cx cx^T
        tx_prox_com = np.array([com_x, com_y, com_z])
        moment_of_mass_prox = mass * tx_prox_com
        cx = skew(tx_prox_com)
        m_rot_com = np.array([[ixx_com, ixy_com, ixz_com],
                              [ixy_com, iyy_com, iyz_com],
                              [ixz_com, iyz_com, izz_com]])
        m_rot_prox = m_rot_com + mass * cx @ cx.T
        ixx = m_rot_prox[0, 0]
        iyy = m_rot_prox[1, 1]
        izz = m_rot_prox[2, 2]
        ixy = m_rot_prox[0, 1]
        ixz = m_rot_prox[0, 2]
        iyz = m_rot_prox[1, 2]

        m_scr_prox = self.dyn.rigid_body_inertia(of=idx.bdy, as_seen_by=idx.frm_prox,
                moment_of_inertia=[ixx, iyy, izz], product_of_inertia=[ixy, ixz, iyz],
                moment_of_mass=list(moment_of_mass_prox), mass=mass)

        s = RigidBodyInertiaState()
        s.m_scr_com = inertia["rbi"]
        s.r_com = inertia["tx_com"]
        s.m_scr_prox = m_scr_prox
        state[node][RigidBodyInertiaState] = s

        self.algo["data"].extend([s.m_scr_prox])
