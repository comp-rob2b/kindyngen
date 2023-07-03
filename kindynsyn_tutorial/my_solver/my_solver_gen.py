# SPDX-License-Identifier: MPL-2.0
from rdflib import collection, RDF
from kindynsyn.rdflib_tools import qname
from kindynsyn.ir_gen import escape
from .namespace import MY_SLV


class AccumulateJointForceTranslator:
    """
    Translate an MY_SLV:AccumulateJointForce operator into the intermediate
    representation.
    """

    @staticmethod
    def is_applicable(g, node):
        """
        Check if this translator is applicable for the given "node". The node's
        type must contain MY_SLV:AccumulateJointForce as defined in the
        namespace associated with this module.

        Parameters
        ----------
        g : rdflib.Graph
            The overall graph from which to obtain data
        node : rdflib.URIRef
            The identifier of the candidate node to be translated
        
        Returns
        -------
        bool
            true if this translator can handle the node, false otherwise
        """
        return set([MY_SLV["AccumulateJointForce"]]) <= set(g[node : RDF["type"]])

    @staticmethod
    def translate(g, node):
        """
        Perform the actual translation to the intermediate representation.

        Parameters
        ----------
        g : rdflib.Graph
            The overall graph from which to obtain data
        node : rdflib.URIRef
            The identifier of the node to be translated
        
        Returns
        -------
        dict
            The operator translated to the intermediate representation
        """

        # Obtain the "sources" list from the graph.
        l = list(collection.Collection(g, g.value(node, MY_SLV["sources"])))

        # Transform each list entry (a source to accumulate from) to the form
        # required by the intermediate representation.
        sources = list(map(lambda v: escape(qname(g, v)), l))
        destintation = escape(qname(g, g.value(node, MY_SLV["destination"])))

        return {
            # Retain the original ID of the operation for traceability
            "represents": str(node),
            # Same as before but escaped
            "name": escape(qname(g, node)),
            # The operator type (it will be used to find the correct rule in the
            # code generator)
            "operator": "ex-accumulate-joint-force",
            # The list of sources to accumulate from
            "sources": sources,
            # The destination where the accumulated forces should be store to
            "destination": destintation
        }
