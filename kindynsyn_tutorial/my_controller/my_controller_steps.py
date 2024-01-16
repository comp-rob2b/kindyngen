# SPDX-License-Identifier: MPL-2.0
from dataclasses import dataclass, field
from rdflib import URIRef, Literal, RDF
from kindynsyn.rdflib_tools import uuid_ref
from kindynsyn.synthesizer import Traverser, Dispatcher
from kindynsyn.synthesizer.steps import ChainIndexState, \
    PositionAccumulationState, VelocityPropagationState
from kindynsyn.namespaces import GEOM_ENT, SPEC
from .namespace import EX_CTRL


ctrl_expand = """
PREFIX ex-ctrl: <https://example.org/ctrl#>

SELECT ?child ?parent WHERE {
    ?node ^ex-ctrl:attached-to ?child .
    BIND(?node as ?parent)
}
"""


@dataclass
class MyCartesianControllerState:
    velocity_root: URIRef | None = field(default=None)
    wrench_root: URIRef | None = field(default=None)
    wrench: URIRef | None = field(default=None)


class MyCartesianControllerStep:

    def __init__(self, g, algo, geom, geom_coord, dyn, dyn_coord):
        self.g = g
        self.algo = algo
        self.geom = geom
        self.geom_coord = geom_coord
        self.dyn = dyn
        self.dyn_coord = dyn_coord

    def traverse(self):
        return Traverser(
            expander=ctrl_expand,
            edge=[Dispatcher(None, self.configure_edge, self.compute_edge)]
        )

    def configure_edge(self, state, parent, child):
        par = state[parent][ChainIndexState]

        # Get hold of the frame's origin that will be the damping's reference
        # point and, hence, also the associated velocity's and wrench's
        # reference point
        frm_prox_org = self.g.value(par.frm_prox, GEOM_ENT["origin"])

        # Declare required data
        velocity_root = self.geom.velocity_twist(of=par.bdy,
            with_respect_to=par.bdy_root, reference_point=frm_prox_org,
            as_seen_by=par.frm_root)
        wrench_root = self.dyn.wrench(acts_on=par.bdy,
            reference_point=frm_prox_org, as_seen_by=par.frm_root,
            number_of_wrenches=1)
        wrench = self.dyn.wrench(acts_on=par.bdy, as_seen_by=par.frm_prox,
            number_of_wrenches=1)

        # Tag as external force specification
        ext = uuid_ref()
        self.g.add((ext, RDF["type"], SPEC["ExternalForce"]))
        self.g.add((ext, SPEC["force"], wrench))

        # Setup state
        s = MyCartesianControllerState()
        s.velocity_root = velocity_root
        s.wrench_root = wrench_root
        s.wrench = wrench

        # Register state
        state[child][MyCartesianControllerState] = s

        # Register data
        self.algo["data"].extend([velocity_root, wrench_root])

    def compute_edge(self, state, parent, child):
        pos = state[parent][PositionAccumulationState]
        vel = state[parent][VelocityPropagationState]
        ctrl = state[child][MyCartesianControllerState]

        rot_vel = self.geom_coord.rotate_velocity_twist_to_proximal_with_pose(
            pose=pos.x_tot,
            frm=vel.xd_tot,
            to=ctrl.velocity_root)
        damp = self.my_damper(
            velocity_twist=ctrl.velocity_root,
            wrench=ctrl.wrench_root)
        rot_wrench = self.dyn_coord.rotate_wrench_to_distal_with_pose(
            pose=pos.x_tot,
            frm=ctrl.wrench_root,
            to=ctrl.wrench,
            number_of_wrenches=1,
            at_index=0)

        self.algo["func"].extend([rot_vel, damp, rot_wrench])

    def my_damper(self, velocity_twist, wrench):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], EX_CTRL["Damping"]))
        self.g.add((id_, EX_CTRL["max-velocity"], Literal(0.1)))
        self.g.add((id_, EX_CTRL["max-force"], Literal(2.0)))
        self.g.add((id_, EX_CTRL["velocity-twist"], velocity_twist))
        self.g.add((id_, EX_CTRL["wrench"], wrench))
        return id_
