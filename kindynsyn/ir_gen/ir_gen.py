# SPDX-License-Identifier: MPL-2.0
import rdflib
from rdflib import RDF
from kindynsyn.utility import log
from kindynsyn.namespaces import ALGO
from kindynsyn.rdflib_tools import qname
from kindynsyn.ir_gen.translators import escape


class IRGenerator:
    def __init__(self, g, translators):
        self.g = g
        self.translators = translators


    def generate_data_types(self, algo):
        res = {}
        for data in self.g[algo : ALGO["data"]]:
            name = escape(qname(self.g, data))
            res[name] = { "data-type": "primitive" }

        # Descent into child algorithms
        for a in self.g[algo : ALGO["algorithm"]]:
            res |= self.generate_variables(a)

        return res

    def generate_variables(self, algo):
        res = {}
        for data in self.g[algo : ALGO["data"]]:
            for t in self.translators:
                if t.is_applicable(self.g, data):
                    obj = t.translate(self.g, data)
                    name = escape(qname(self.g, data))
                    res[name] = obj
                    break
            else:
                log("No data translator found:", data)
                continue

        # Descent into child algorithms
        for a in self.g[algo : ALGO["algorithm"]]:
            res |= self.generate_variables(a)

        return res

    def generate_local(self, algo):
        res = []
        for data in self.g[algo : ALGO["data"]]:
            name = escape(qname(self.g, data))
            res.append(name)

        # Descent into child algorithms
        for a in self.g[algo : ALGO["algorithm"]]:
            res.extend(self.generate_variables(a))

        return res

    def generate_closures(self, sched):
        res = {}

        chain = rdflib.collection.Collection(self.g, self.g.value(sched, ALGO["trigger-chain"]))
        for trig in chain:
            if not self.g[trig : RDF["type"] : ALGO["Schedule"]]:
                for t in self.translators:
                    if t.is_applicable(self.g, trig):
                        obj = t.translate(self.g, trig)
                        name = escape(qname(self.g, trig))
                        res[name] = obj
                        break
                else:
                    log("No function translator found:", trig)
                    continue
            else:
                # Descent into child schedules
                res |= self.generate_closures(trig)

        return res

    def generate_schedule(self, sched):
        seq = []

        chain = rdflib.collection.Collection(self.g, self.g.value(sched, ALGO["trigger-chain"]))
        for trig in chain:
            if not self.g[trig : RDF["type"] : ALGO["Schedule"]]:
                name = escape(qname(self.g, trig))
                seq.append(name)
            else:
                # Descent into child schedules
                seq.extend(self.generate_schedule(trig))

        return seq


    def generate(self, sched, algo):
        data_types = self.generate_data_types(algo)
        variables = self.generate_variables(algo)
        local = self.generate_local(algo)
        closures = self.generate_closures(sched)
        schedule = self.generate_schedule(sched)

        return {
            "data-types": data_types,
            "variables": variables,
            "input": None,
            "output": None,
            "local": local,
            "closures": closures,
            "schedule": schedule
        }
