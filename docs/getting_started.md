# Getting started

Here, we provide information about common steps that are required for all tutorials and also all tools that reuse the kindynsyn library. For the tutorials, those common steps reside in the file [`runner.py`](https://github.com/comp-rob2b/kindyngen/kindynsyn_tutorial/runner.py) and will be explained step-by-step in this document.

## Resolving IRIs locally
A benefit of using IRIs as identifiers is that they do not have to resolve to a concrete resource. In other words, the identification of that resource remains separated from further information about that resource. We exploit this to achieve fast access to models by retaining a local copy of remote models and then adapting the file resolver of [urllib](https://docs.python.org/3/library/urllib.html), which is used internally by rdflib.

```python
from kindynsyn.utility import resolver
...
METAMODELS = "https://comp-rob2b.github.io/metamodels/"
MODELS = "https://comp-rob2b.github.io/robot-models/"

url_map = {
    METAMODELS: "comp-rob2b/metamodels/",
    MODELS: "comp-rob2b/robot-models/"
}
resolver.install(resolver.IriToFileResolver(url_map))
```

In the above example, the `url_map` dictionary defines which IRIs (either prefixes or fully-qualified names) - the keys in the dictionary - map to which local files or directorys - the values in the dictionary. Here, any resource under [https://comp-rob2b.github.io/robot-models/](https://comp-rob2b.github.io/robot-models/) would remap to the local directory [comp-rob2b/robot-models/](comp-rob2b/robot-models/). Note, that in this example the local directory is given relative to current working directory.

Hence, the metamodels repository and the robot models repository should be cloned ...
```bash
cd <repo>
git clone git@github.com:comp-rob2b/metamodels.git
git clone git@github.com:comp-rob2b/robot-models.git
```

... and symlinked to those locations:
```bash
cd <repo>/kindyngen
mkdir comp-rob2b
ln -s <repo>/metamodels comp-rob2b/metamodels
ln -s <repo>/robot-models comp-rob2b/robot-models
```

Alternatively, one can (i) remove the local file resolution to rely on the remote models; or (ii) adapt the locations in the `url_map`.


## Caching SPARQL queries
We advocate using [SPARQL](https://www.w3.org/TR/sparql11-query/) to represent any non-trivial queries on the knowledge graph. Here, we decide to keep all queries in a separated directory and load them on request in the associated modules. However, many of the SPARQL queries are reused across different modules. To this end it is worthwhile to [prepare](https://en.wikipedia.org/wiki/Prepared_statement) and cache those queries. The following code snippet demonstrates the loading and caching mechanism. Here, the `SPARQL_PATH` variable points to the directory that contains the SPARQL query models.

```python
from kindynsyn.utility import loader
from kindynsyn.rdflib_tools import sparql_prepare
...
SPARQL_PATH = "models/sparql"
...
sparql_loader = loader(SPARQL_PATH)
cache = sparql_cache(sparql_loader, sparql_prepare)
```


## Loading models
First, we instantiate the rdflib-specific, in-memory graph representation (here, a so-called conjunctive graph to represent [named graphs and quads](https://en.wikipedia.org/wiki/Named_graph#Named_graphs_and_quads)). Next, the `parse` function loads all required models. The concrete selection of which models to load is a highly application-specific choice. In this case we select the [models](https://github.com/comp-rob2b/robot-models) of the [Kinova Gen3](https://www.kinovarobotics.com/product/gen3-robots) arm as input kinematic chain. To be compatible with the `kindynsyn` tool, the models must conform to our [metamodels](https://github.com/comp-rob2b/metamodels) as described in the associated [tutorial](https://github.com/comp-rob2b/modelling-tutorial).

```python
import rdflib
...
g = rdflib.ConjunctiveGraph()
...
g.parse(MODELS + "kinova/gen3/7dof/robot.geom.json", format="json-ld")
g.parse(MODELS + "kinova/gen3/7dof/robot.kin-chain.json", format="json-ld")
g.parse(MODELS + "kinova/gen3/7dof/robot.dyn.json", format="json-ld")
g.parse(MODELS + "kinova/gen3/7dof/mounting-upright.geom.json", format="json-ld")
```

Notice, how the models are loaded via their URL which is then remapped to a local file as described [above](#local-file-resolution).

With the `bind` function we tell rdflib to use the `uuid` prefix for any [CURIE](https://en.wikipedia.org/wiki/CURIE) associated with the UUID namespace. This is an optional step and only meant to increase the human readability of serialized models.
```python
from kindynsyn.namespaces import UUID
...
g.bind("uuid", UUID)
```


## Synthesizing a solver algorithm
The main step is that of synthesizing a solver algorithm. An algorithm consists of (i) a collection of data blocks, to represent the data in the algorithm; (ii) a collection of function blocks, to represent closures, i.e. the binding of functions to their data; and (iii) a collection of schedule blocks, to represent the order in which the function blocks should be executed or triggered.

The synthesizer's behaviour is influenced by its _configuration_ (the sweeps to be executed and the computations involved in each sweep), as determined via the `solver_configurator` function and to be discussed in-depth in the following tutorials. Computations involved in the synthesis may contribute data blocks or function blocks to the algorithm model that will be stored in the `slv_algo` dictionary. The initial data blocks are those that represent the kinematic chains' connection to the "world" as determined by the robot's pose (`base_x`), velocity twist (`base_v`) and acceleration twist (`base_a`) with respect to the world. Those originate from the `mounting-upright.geom.json` model that was loaded before. Given the configuration, the synthesizer starts its sweeps (`s.execute`) through the kinematic chain model at the selected root frame (`frm_root`).

```python
from kindynsyn.synthesizer.synthesizer import SolverSynthesizer
...
frm_root = ROB["link0-root"]
base_x = ROB["pose-coord-link0-root-wrt-world-frame"]
base_v = ROB["velocity-twist-coord-link0-root-wrt-world-frame"]
base_a = ROB["acceleration-twist-coord-link0-root-wrt-world-frame"]

slv_algo = { "data": [ base_x, base_v, base_a ], "func": [] }
slv_conf = solver_configurator(g, cache, ROB, slv_algo)

s = SolverSynthesizer(g, slv_conf)
s.execute(frm_root, ["configure", "compute"])
```

Finally, we emit a representation of the generated algorithm into the graph:
```python
from kindynsyn.synthesizer.graph_factories import Algorithm
...
algo = Algorithm(g)
sched = algo.schedule(slv_algo["func"])
algo_id = algo.algorithm(data=slv_algo["data"], func=slv_algo["func"], sched=[sched])
```


## Storing the solver's intermediate representation
The step consists of transforming the algorithm's graph model to a JSON-based (tree-structured) intermediate representation via the `IRGenerator`. The IR generator supports configuration via a list of translators that extract and convert information from the graph to the required JSON representation. The translator configuration is another variation point to be discussed in-depth in the dedicated tutorials.

```python
from kindynsyn.ir_gen.translators import translator_list
from kindynsyn.ir_gen import IRGenerator
...
OUT_FILE = "gen/solver.gen-ir.json"
...
tr = translator_configurator()
tr.extend(translator_list)
ir = IRGenerator(g, tr)
ir_prog = ir.generate(sched, algo_id)

with open(OUT_FILE, "w") as f:
    json.dump(ir_prog, f, indent=4)
```


## Executing the synthesizer and the code generator

To execute the `kindynsyn` tutorials run the following command:
```bash
python kindynsyn_tutorial/runner.py <tutorial>
```

Where `<tutorial>` selects the desired solver configuration (as described in the following tutorials):

* `fpk`: synthesize a forward position kinematics algorithm
* `rne`: synthesize an RNE inverse dynamics solver
* `rne_slv_robif`: same as the previous one, but adds a solver sweep and a robot interface model
* `rne_slv_robif_ctrl`: same as the previous one, but adds a model of a Cartesian-space impedance controller

Afterwards, the code generator can be executed via:
```bash
cd <kindyngen>/code_generator
make <backend>
```

Where `<backend>` selects the desired generation target and must be compatible with the previous selection of the `<tutorial>`:

* `tutorial-dyn2b`: for `fpk` or `rne`
* `tutorial-dyn2b-slv-print`: for `rne_slv_robif`
* `tutorial-dyn2b-slv-robif2b`: for `rne_slv_robif`
* `tutorial-dyn2b-slv-print-ctrl`: for `rne_slv_robif_ctrl`
* `tutorial-dyn2b-slv-robif2b-ctrl`: for `rne_slv_robif_ctrl`

This generates the code in the `gen/` directory which can now be compiled and executed as follows:
```bash
cd <kindyngen>/gen
cmake .
make
./main
```
