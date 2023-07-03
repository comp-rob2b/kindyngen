# SPDX-License-Identifier: MPL-2.0
from rdflib import collection, BNode, Literal, RDF
from kindynsyn.namespaces import ALGO
from kindynsyn.rdflib_tools.helpers import uuid_ref


class Algorithm:
    def __init__(self, g):
        self.g = g

    def schedule(self, operation_list, name=None):
        trigger_chain_id = BNode()
        collection.Collection(self.g, trigger_chain_id, operation_list)

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], ALGO["Schedule"]))
        self.g.add((id_, ALGO["trigger-chain"], trigger_chain_id))
        if name:
            self.g.add((id_, ALGO["name"], Literal(name)))

        return id_

    def algorithm(self, data=[], func=[], sched=[], conn=[], algo=[], dep=[], type_=None):
        id_ = uuid_ref()

        self.g.add((id_, RDF["type"], ALGO["Algorithm"]))
        if type_:
            self.g.add((id_, RDF["type"], type_))
        for d in data:
            self.g.add((id_, ALGO["data"], d))
        for f in func:
            self.g.add((id_, ALGO["function"], f))
        for s in sched:
            self.g.add((id_, ALGO["schedule"], s))
        for a in algo:
            self.g.add((id_, ALGO["algorithm"], a))

        return id_
