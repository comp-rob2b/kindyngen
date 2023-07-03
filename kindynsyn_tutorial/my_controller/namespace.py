# SPDX-License-Identifier: MPL-2.0
import rdflib

class EX_CTRL(rdflib.namespace.DefinedNamespace):
    Damping: rdflib.URIRef

    wrench: rdflib.URIRef

    _extras = [
        "attached-to",
        "velocity-twist",
        "max-velocity",
        "max-force"
    ]

    _NS = rdflib.Namespace("https://example.org/ctrl#")
