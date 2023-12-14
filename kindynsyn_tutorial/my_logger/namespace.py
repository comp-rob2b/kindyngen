# SPDX-License-Identifier: MPL-2.0
from rdflib import URIRef, Namespace
from rdflib.namespace import DefinedNamespace

class MY_LOG(DefinedNamespace):
    # The operation type to log some quantity. It requires a single 
    # property/predicate:
    # - quantity: a symbolic pointer to the quantity
    Logger: URIRef

    # The property to represent the symbolic pointer to quantity for the Logger
    # operation.
    quantity: URIRef

    # The fully-qualified namespace
    _NS = Namespace("https://example.org/logging#")
