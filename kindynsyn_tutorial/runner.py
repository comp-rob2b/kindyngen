# SPDX-License-Identifier: MPL-2.0
import json
import rdflib

from kindynsyn.namespaces import UUID
from kindynsyn.utility import resolver, loader
from kindynsyn.rdflib_tools import sparql_prepare, sparql_cache
from kindynsyn.ir_gen.translators import translator_list
from kindynsyn.ir_gen import IRGenerator

from kindynsyn.synthesizer.synthesizer import SolverSynthesizer
from kindynsyn.synthesizer.graph_factories import Algorithm

import sys
import importlib

try:
    # Dynamically load the tutorial module as specified via the command line
    funcs = ["solver_configurator", "translator_configurator"]
    _mod = importlib.__import__(name=sys.argv[1], fromlist=funcs)
    solver_configurator = _mod.solver_configurator
    translator_configurator = _mod.translator_configurator
except:
    print("Usage:")
    print("#", sys.argv[0], "<tutorial>")
    print("where <tutorial> is one of:")
    print("- fpk")
    print("- rne")
    print("- rne_slv_robif")
    print("- rne_slv_robif_ctrl")
    sys.exit()


def main():
    OUT_FILE = "gen/solver.gen-ir.json"
    SPARQL_PATH = "models/sparql"

    ROB = rdflib.Namespace("https://comp-rob2b.github.io/robots/kinova/gen3/7dof/")
    METAMODELS = "https://comp-rob2b.github.io/metamodels/"
    MODELS = "https://comp-rob2b.github.io/robot-models/"

    url_map = {
        METAMODELS: "comp-rob2b/metamodels/",
        MODELS: "comp-rob2b/robot-models/"
    }
    resolver.install(resolver.IriToFileResolver(url_map))

    sparql_loader = loader(SPARQL_PATH)
    cache = sparql_cache(sparql_loader, sparql_prepare)


    #
    # Load graph
    #
    g = rdflib.ConjunctiveGraph()
    g.bind("uuid", UUID)

    g.parse(MODELS + "kinova/gen3/7dof/robot.geom.json", format="json-ld")
    g.parse(MODELS + "kinova/gen3/7dof/robot.kin-chain.json", format="json-ld")
    g.parse(MODELS + "kinova/gen3/7dof/robot.dyn.json", format="json-ld")
    g.parse(MODELS + "kinova/gen3/7dof/mounting-upright.geom.json", format="json-ld")


    #
    # Synthesize solver
    #

    # Identify robot's connection to the "world"
    frm_root = ROB["link0-root"]
    base_x = ROB["pose-coord-link0-root-wrt-world-frame"]
    base_v = ROB["velocity-twist-coord-link0-wrt-world-body"]
    base_a = ROB["acceleration-twist-coord-link0-wrt-world-body"]

    slv_algo = { "data": [ base_x, base_v, base_a ], "func": [] }
    slv_conf = solver_configurator(g, cache, ROB, slv_algo)

    # Run synthesis
    s = SolverSynthesizer(g, slv_conf)
    s.execute(frm_root, ["configure", "compute"])

    # Create algorithm representation
    algo = Algorithm(g)
    sched = algo.schedule(slv_algo["func"])
    algo_id = algo.algorithm(data=slv_algo["data"], func=slv_algo["func"], sched=[sched])


    #
    # Generate intermediate representation
    #
    tr = translator_configurator()
    tr.extend(translator_list)
    ir = IRGenerator(g, tr)
    ir_prog = ir.generate(sched, algo_id)

    with open(OUT_FILE, "w") as f:
        json.dump(ir_prog, f, indent=4)


if __name__ == "__main__":
    main()
