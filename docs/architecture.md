# kindyngen's architecture


## Synthesizer: kindynsyn

`kindynsyn` is a Python library that relies on [RDFlib](https://rdflib.dev/) to represent composable kinematic chain models in memory and perform queries on those graphs. Especially, for non-trivial queries the [SPARQL](https://www.w3.org/TR/sparql11-query/) interface provides a powerful and standardized way to interact effectively with the graph. Hence, `kindynsyn` is a [graph rewriting](https://en.wikipedia.org/wiki/Graph_rewriting) software, in that it consumes one graph model (of a kinematic chain and a query thereupon) to produce another graph model (the solver algorithm). By exploiting domain-specific knowledge about kinematic chains and their solvers, it can efficiently synthesize the solver algorithms. As output, the synthesizer produces a JSON-based [intermediate representation](https://en.wikipedia.org/wiki/Intermediate_representation) (IR) of the synthesized algorithm.

The core of `kindynsyn` is a graph traversal which can be controlled by extension modules. Inspired by terminology of the [Gremlin](https://tinkerpop.apache.org/docs/current/reference/#the-graph-process) graph query language, we call these extensions _steps_ because they represent the individual "instructions" of the overall graph traversal program. In `kindynsyn` a step declares an expansion query (for example, to define how to find the "root" frame on the next segment in a kinematic chain), or _expander_, to control which nodes and edges the graph traversal should visit next. Additionally, the step associates domain-specific computations (for instance, "map a joint position to a Cartesian pose") pertaining to the nodes and edges with the expander. Note, that steps only _declare_ but don't execute the expansion queries. This design originates from our observation that the query execution is on of the slower computations. Hence, `kindynsyn` groups all steps with the same expander to reduce the number of overall query executions. This approach resembles modern web development where [React components](https://react.dev/reference/react/components) declare their required data via [GraphQL queries](https://graphql.org/) with the overall objective of minimizing the slow client-server interaction.


## Code generator

A template-based code generator backend consumes `kindynsyn`'s IR. In `kindyngen` we opt for [StringTemplate](https://www.stringtemplate.org/) as the template engine and the [StringTemplate Standalone Tool](https://github.com/jsnyders/STSTv4) as its frontend. Its author has reported on StringTemplate's benefits[^Parr2004] over the more popular template engines such as [Cheetha](https://cheetahtemplate.org/), [Django's](https://www.djangoproject.com/) templating system, [Genshi](https://genshi.edgewall.org/) or [Jinja](https://palletsprojects.com/p/jinja/). A more hands-on paper draft is also available[^Parr2006].

To summarize, the main reasons for choosing StringTemplate are:

* It enforces the separation of "business logic" from "display" by only introducing a minimal amount of control flow (if-then-else conditional, template application/[map](https://en.wikipedia.org/wiki/Map_(higher-order_function)) operator, template inclusion) in the templates. This contrasts with the afore-mentioned, popular template engines that often embed Turing-complete languages.
* A template is composed of rules (akin to, or dual to, rules in [formal grammars](https://en.wikipedia.org/wiki/Formal_grammar)) where each rule has a unique identifier. This aligns with our modelling approach for [composable models](https://github.com/comp-rob2b/modelling-tutorial).

However, the benefit of the IR is that it decouples the synthesizer from the code generator. Hence, one could swap over the code generator to a more popular template engine.

[^Parr2004]: Terence John Parr, "Enforcing strict model-view separation in template engines", in Proc. of the International Conference On World Wide Web (WWW), 2004.

[^Parr2006]: Terence John Parr, "A Functional Language For Generating Structured Text", available [https://www.cs.usfca.edu/~parrt/papers/ST.pdf](https://www.cs.usfca.edu/~parrt/papers/ST.pdf), 2006.
