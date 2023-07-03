# SPDX-License-Identifier: MPL-2.0
from rdflib import URIRef, Namespace
from rdflib.namespace import DefinedNamespace

class MY_SLV(DefinedNamespace):
    # The operation type to accumulate joint forces. It requires two
    # properties/predicates:
    # - sources: a list of symbolic pointers to input joint forces
    # - destination: the symbolic pointer to the accumulated joint force
    AccumulateJointForce: URIRef

    # The property to represent the list of symbolic pointers to the input joint
    # forces for the AccumulateJointForce operation.
    sources: URIRef
    # The property to represent the symbolic pointer to the accumulated joint
    # force in the AccumulateJointForce operation.
    destination: URIRef

    # The fully-qualified namespace
    _NS = Namespace("https://example.org/my-solver#")
