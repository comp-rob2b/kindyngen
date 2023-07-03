# SPDX-License-Identifier: MPL-2.0
from dataclasses import dataclass, field
from rdflib import URIRef
from kindynsyn.namespaces import GEOM_ENT, KC_ENT
from kindynsyn.synthesizer.synthesizer import Traverser, Dispatcher
from .queries import q_expand, q_root


@dataclass
class ChainIndexState:
    frm_prox: URIRef | None = field(default=None)
    frm_par_dist: URIRef | None = field(default=None)
    org_prox: URIRef | None = field(default=None)
    bdy: URIRef | None = field(default=None)
    bdy_root: URIRef | None = field(default=None)
    frm_par_dist: URIRef | None = field(default=None)
    frm_root: URIRef | None = field(default=None)

    joint: URIRef | None = field(default=None)          # Joint that connects to the parent


class ChainIndexStep:
    def __init__(self, g, load, frm_root):
        self.g = g
        self.frm_root = frm_root
        self.sel_sib = load("select_sibling_frame.rq")

    def traverse(self):
        return Traverser(
            expander=q_expand,
            edge=[Dispatcher(configure=self.configure_branch)],
            node=[Dispatcher(condition=q_root, configure=self.configure_node)]
        )

    def configure_node(self, state, node):
        s = ChainIndexState()
        s.frm_prox = node
        s.frm_par_dist = None   # Does not exist for root
        s.org_prox = self.g.value(node, GEOM_ENT["origin"])
        s.bdy = self.g.value(node, ~GEOM_ENT["simplices"])
        s.bdy_root = self.g.value(self.frm_root, ~GEOM_ENT["simplices"])
        s.frm_root = self.frm_root
        s.joint = self.g.value(node, ~KC_ENT["between-attachments"])
        state[node][ChainIndexState] = s

    def configure_branch(self, state, parent, child):
        self.configure_node(state, child)

        res = self.g.query(self.sel_sib, initBindings={"frame": child})
        state[child][ChainIndexState].frm_par_dist = list(res)[0]["sibling"]
