# SPDX-License-Identifier: MPL-2.0
from rdflib.namespace import RDF
from kindynsyn.rdflib_tools import qname
from kindynsyn.ir_gen import escape
from .namespace import EX_CTRL


class MyCartesianControllerTranslator:
    @staticmethod
    def is_applicable(g, node):
        return set([EX_CTRL["Damping"]]) <= set(g[node : RDF["type"]])

    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "operator": "ex-damping",
            "dimensions": 3,
            "max-velocity": float(g.value(node, EX_CTRL["max-velocity"])),
            "max-force": float(g.value(node, EX_CTRL["max-force"])),
            "velocity-twist": escape(qname(g, g.value(node, EX_CTRL["velocity-twist"]))),
            "wrench": escape(qname(g, g.value(node, EX_CTRL["wrench"])))
        }
