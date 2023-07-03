# SPDX-License-Identifier: MPL-2.0
import rdflib

class MY_IF(rdflib.namespace.DefinedNamespace):
    JointPositionToSolver: rdflib.URIRef
    JointVelocityToSolver: rdflib.URIRef
    JointForceFromSolver: rdflib.URIRef

    source: rdflib.URIRef
    destination: rdflib.URIRef

    _extras = [
        "source-index",
        "destination-index"
    ]

    _NS = rdflib.Namespace("https://example.org/robot-interface#")
