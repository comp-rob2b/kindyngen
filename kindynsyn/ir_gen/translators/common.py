# SPDX-License-Identifier: MPL-2.0
import re
from rdflib import collection, Graph, URIRef, BNode, Literal, RDF
from kindynsyn.rdflib_tools import local_name


def for_type(*types):
    """
    Decorator that checks if a translator is applicable to a set of RDF types.
    The decorator must be applied to a class and injects a static method called
    "is_applicable" that accepts two arguments (an RDFLib graph and a node in
    that graph). If the types provided as arguments to the decorator are a
    subset of the node's rdf:type the function return true else false.
    """
    def decorator_for_type(cls):
        @staticmethod
        def is_applicable(g: Graph, node: URIRef) -> bool:
            return set(types) <= set(g[node : RDF["type"]])
        cls.is_applicable = is_applicable
        return cls
    return decorator_for_type


def escape(s):
    s = re.sub("[:-]", "_", s)
    s = re.sub("[<>]", "", s)
    return s



# Literals, URIRef and BNode are disjunct
def is_literal(x):
    return isinstance(x, Literal)

def is_uriref(x):
    return isinstance(x, URIRef)

def is_bnode(x):
    return isinstance(x, BNode)

# A list must have an rdf:first predicate (excludes empty lists!) and be a blank
# node
def is_list(g, x):
    return RDF["first"] in g.predicates(x) and is_bnode(x)


def embed_data(g, obj):
    """
    Return ...:
    - lists by recursive traversal
    - literals by their Python representations
    - IRIs as their representation (local part of QName), no recursion
    """
    if is_list(g, obj):
        l = []
        for e in collection.Collection(g, obj):
            l.append(embed_data(g, e))
        return l

    if is_literal(obj):
        return obj.toPython()

    if is_uriref(obj):
        return local_name(g, obj)

    return None



def parse_scalar(node):
    return node.value if node is not None else None


def parse_vector3(g, node):
    l = list(collection.Collection(g, node))
    if len(l) != 3:
        return None

    # Get the Python representation of the associated RDF literal
    return list(map(lambda v: v.value, l))
