# SPDX-License-Identifier: MPL-2.0
from rdflib.namespace import RDF
from kindynsyn.rdflib_tools import qname
from kindynsyn.ir_gen import escape
from .namespace import MY_IF


class JointConfigurationToSolverTranslator:
    def __init__(self, rdf_type, operator_name):
        self.rdf_type = rdf_type
        self.operator_name = operator_name

    def is_applicable(self, g, node):
        return set([self.rdf_type]) <= set(g[node : RDF["type"]])

    def translate(self, g, node):
        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "operator": self.operator_name,
            "source-index": int(g.value(node, MY_IF["source-index"])),
            "destination": escape(qname(g, g.value(node, MY_IF["destination"])))
        }


class JointConfigurationFromSolverTranslator:
    def __init__(self, rdf_type, operator_name):
        self.rdf_type = rdf_type
        self.operator_name = operator_name

    def is_applicable(self, g, node):
        return set([self.rdf_type]) <= set(g[node : RDF["type"]])

    def translate(self, g, node):
        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "operator": self.operator_name,
            "destination-index": int(g.value(node, MY_IF["destination-index"])),
            "source": escape(qname(g, g.value(node, MY_IF["source"])))
        }
