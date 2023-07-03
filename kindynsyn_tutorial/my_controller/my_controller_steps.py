# SPDX-License-Identifier: MPL-2.0
from dataclasses import dataclass, field
from rdflib import URIRef, Literal, RDF
from kindynsyn.rdflib_tools import uuid_ref
from kindynsyn.synthesizer import Traverser, Dispatcher
from kindynsyn.synthesizer.steps import ChainIndexState, \
    VelocityPropagationState
from kindynsyn.namespaces import SPEC
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
    wrench: URIRef | None = field(default=None)


class MyCartesianControllerStep:

    def __init__(self, g, algo, dyn):
        self.g = g
        self.algo = algo
        self.dyn = dyn

    def traverse(self):
        return Traverser(
            expander=ctrl_expand,
            edge=[Dispatcher(None, self.configure_edge, self.compute_edge)]
        )

    def configure_edge(self, state, parent, child):
        par = state[parent][ChainIndexState]

        # Declare required data
        wrench = self.dyn.wrench(acts_on=par.bdy, as_seen_by=par.frm_prox,
            number_of_wrenches=1)

        # Tag as external force specification
        ext = uuid_ref()
        self.g.add((ext, RDF["type"], SPEC["ExternalForce"]))
        self.g.add((ext, SPEC["force"], wrench))

        # Setup state
        s = MyCartesianControllerState()
        s.wrench = wrench

        # Register state
        state[child][MyCartesianControllerState] = s

        # Register data
        self.algo["data"].extend([wrench])

    def compute_edge(self, state, parent, child):
        vel = state[parent][VelocityPropagationState]
        ctrl = state[child][MyCartesianControllerState]

        damp = self.my_damper(
            velocity_twist=vel.xd_tot,
            wrench=ctrl.wrench)

        self.algo["func"].extend([damp])

    def my_damper(self, velocity_twist, wrench):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], EX_CTRL["Damping"]))
        self.g.add((id_, EX_CTRL["max-velocity"], Literal(0.1)))
        self.g.add((id_, EX_CTRL["max-force"], Literal(2.0)))
        self.g.add((id_, EX_CTRL["velocity-twist"], velocity_twist))
        self.g.add((id_, EX_CTRL["wrench"], wrench))
        return id_
