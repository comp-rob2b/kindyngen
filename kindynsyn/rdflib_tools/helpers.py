# SPDX-License-Identifier: MPL-2.0
import operator
from functools import reduce
import uuid
import rdflib
from rdflib.plugins.sparql.parser import parseUpdate, parseQuery
from rdflib.plugins.sparql.algebra import translateUpdate, translateQuery


def uuid_ref():
    """
    Return an rdflib reference for a randomly-generated UUID.
    """
    return rdflib.URIRef(uuid.uuid4().urn)


def expand_to_named_graph(closure, g, named_graph):
    """
    By default owlrl expands into the default graph. This functions hooks the
    'add' method to inject a named graph that the triples will be inserted
    into.
    There's an associated issue: https://github.com/RDFLib/OWL-RL/issues/42
    """
    if type(g) is not rdflib.Graph:
        add_original = g.add
        g.add = lambda t: add_original((t[0], t[1], t[2], named_graph))
        closure.expand(g)
        g.add = add_original
    else:
        closure.expand(g)


def flatten(g):
    """
    Flatten a conjunctive graph into a graph.
    """
    if isinstance(g, rdflib.ConjunctiveGraph):
        return reduce(operator.iadd, g.contexts(), rdflib.Graph())

    return g


def prepare_query(sparql_str, base={}, initNs={}):
    """
    Pre-compile a SPARQL query so that it executes faster.
    """
    return translateQuery(parseQuery(sparql_str), base=base, initNs=initNs)

def prepare_update(sparql_str, base={}, initNs={}):
    """
    Pre-compile a SPARQL update so that it executes faster.
    """
    return translateUpdate(parseUpdate(sparql_str), base=base, initNs=initNs)


def prefixed(g, node):
    """
    Return the CURIE (Compact URI) for the given node in the given graph.
    """
    return node.n3(g.namespace_manager)


def local_name(g, x):
    """
    Return the local name of a URIRef. In the context of QNames
    (<prefix>:<local-name>) the local name is the part after the colon.
    """
    try:
        q = g.compute_qname(x)
        return q[2]
    except:
        return x


def qname(g, x):
    """
    Return the QName of a URIRef. The QName has the shape <prefix>:<local-name>
    """
    try:
        q = g.compute_qname(x)
        return q[0] + ":" + q[2]
    except:
        return x
