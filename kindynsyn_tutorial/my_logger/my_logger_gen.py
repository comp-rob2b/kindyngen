# SPDX-License-Identifier: MPL-2.0
from rdflib import collection, RDF
from kindynsyn.namespaces import GEOM_COORD
from kindynsyn.rdflib_tools import qname
from kindynsyn.ir_gen import escape
from .namespace import MY_LOG


class MyLoggerTranslator:

    @staticmethod
    def is_applicable(g, node):
        return set([MY_LOG["Logger"]]) <= set(g[node : RDF["type"]])

    @staticmethod
    def translate(g, node):
        l = []
        for e in g.objects(node, MY_LOG["quantity"]):
            l.append(escape(qname(g, e)))

        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "operator": "ex-logger",
            "quantity": l
        }
