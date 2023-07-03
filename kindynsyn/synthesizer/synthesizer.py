# SPDX-License-Identifier: MPL-2.0
import typing
import enum
from dataclasses import dataclass, field
import rdflib
from rdflib.plugins.sparql.sparql import Query
from kindynsyn.rdflib_tools.helpers import prepare_query
from kindynsyn.rdflib_tools.traversal import BreadthFirst, \
    traverse_nodes_with_parent_user
from kindynsyn.utility import log


class SweepDirection(enum.Enum):
    OUTWARD = 1
    INWARD = 2


@dataclass
class Dispatcher:
    # Condition on node or edge (depending on this dispatcher's role in the
    # traverser) when the dispatcher gets activated
    condition: str | None = field(default=None)
    configure: typing.Callable | None = field(default=None)
    compute: typing.Callable | None = field(default=None)

    def __repr__(self):
        return f"Dispatcher({self.configure}, {self.compute})"

@dataclass
class Traverser:
    '''
    Depending on the traversal (inward or outward) we visit in the following way
    - outward: parent node, edge (args: parent, child)
    - inward: each child node, edge (args: parent, all children)
    '''
    expander: str | None = field(default=None)
    node: list[Dispatcher] = field(default_factory=list)    # Node visitor (before edge)
    edge: list[Dispatcher] = field(default_factory=list)    # Edge visitor (after each node)

    def __repr__(self):
        return f"Traverser({self.node}, {self.edge})"



class Step(typing.Protocol):
    def traverse(self) -> Traverser:
        ...


@dataclass
class SweepConfig:
    direction: SweepDirection
    steps: list[Step]

    def __hash__(self):
        return hash(repr(self))

@dataclass
class SolverConfig:
    sweeps: list[SweepConfig]



class ConditionCache:
    """
    The conditions of node visitors are independent of the concrete traverser
    and hence can be cached.
    """
    def __init__(self, g):
        self.g = g
        self.node = {}

    def register(self, node, condition):
        if node not in self.node:
            self.node[node] = {}
        if condition not in self.node[node]:
            res = self.g.query(condition, initBindings={"node": node})
            self.node[node][condition] = res


class CompiledExpander:
    """
    The overall design is based on the following assumptions:
    1. Steps are the major extension point (i.e. developers will want to
       contribute custom steps)
    2. Steps may still want to use custom queries
    3. Query compilation and query execution are rather slow
    4. Queries are shared by multiple steps
    
    Hence, we allow steps to declare (but not to execute) queries to (i) expand
    the traversal's open set; and (ii) dispatch to graph-specific computations,
    for example, special processing at a tree's root. The pure declaration of
    queries enables (i) to prepare or pre-compile them; and (ii) to
    memoize/cache shared queries and defer their execution to reduce the amount
    of executed queries to the bare minimum.
    """

    def __init__(self, query: str):
        self.compiled: Query = prepare_query(query)
        self.traversers: dict[SweepConfig, list[Traverser]] = {}

    def append(self, sweep: SweepConfig, traverser: Traverser):
        if sweep not in self.traversers:
            self.traversers[sweep] = []
        self.traversers[sweep].append(traverser)


class TraverserRegistry:
    def __init__(self):
        self.expanders: dict[str, CompiledExpander] = {} # str -> expander query

    def register(self, query: str, sweep: SweepConfig, traverser: Traverser):
        if query not in self.expanders:
            self.expanders[query] = CompiledExpander(query)
        self.expanders[query].append(sweep, traverser)



class TreeExpander:
    def __init__(self, graph: rdflib.Graph, expanders: dict[str, CompiledExpander]):
        self.graph = graph
        self.expanders = expanders

    def root(self, node: rdflib.URIRef) -> dict[str, list[CompiledExpander]]:
        # Expand the root once separately. Any user data that applies to the
        # neighbours also applies to the root.
        ret = {}
        for u in self.expand(node).values():
            for q, ex in u.items():
                if q not in ret:
                    ret[q] = []
                ret[q].extend(ex)
        return ret

    def expand(self, node: rdflib.URIRef) -> dict[rdflib.URIRef, dict[str, list[CompiledExpander]]]:
        # Try applying any expander query and if it returns neighbour nodes
        # attach them to the final result list together with the dispatcher
        # functions.
        ret = {}
        for query, expander in self.expanders.items():
            res = self.graph.query(query, initBindings={"node": node})
            for row in res:
                child = row["child"]

                # There can be multiple queries leading to the _same_ child, so
                # the associated expanders/traversers are collected in a list
                if child not in ret:
                    ret[child] = {}
                if query not in ret[child]:
                    ret[child][query] = []
                ret[child][query].append(expander)
        return ret


class SolverSynthesizer:
    def __init__(self, g: rdflib.Graph, conf: SolverConfig):
        self.g = g
        self.conf = conf
        self.traversal = None
        self.conditions = None
        self.children = None
        self.state = None

    def execute(self, root: rdflib.URIRef, funcs: list[str]):
        # Compute (serial) breadth-first traversal of graph using expanders to expand the fringe.
        # The same expander query may be used for (i) multiple steps; in (ii) different sweeps.
        # Hence, we need to keep track of the sweep and the dispatch function per expansion step
        self.traversal = self._compute_traversal(root)

        # Execute one outward traversal to fill the condition cache
        self.conditions = self._cache_conditions(self.traversal)

        # Compute children of all nodes
        self.children = self._compute_children(self.traversal)

        # Initialize the state for each node
        self.state = self._init_state(self.traversal)

        # Execute functions
        for func in funcs:
            for sweep in self.conf.sweeps:
                self._execute_sweep(sweep, func)

    def _compute_traversal(self, root):
        registry = TraverserRegistry()
        for sweep in self.conf.sweeps:
            for step in sweep.steps:
                traverser = step.traverse()
                registry.register(traverser.expander, sweep, traverser)
        # expanders[expand_query][dispatch_query][sweep] -> list of configure/compute/... functions

        log(registry)

        open_set = BreadthFirst
        ex = TreeExpander(self.g, registry.expanders)
        entity_with_parent = list(traverse_nodes_with_parent_user(open_set, root, ex))

        return entity_with_parent

    def _cache_conditions(self, traversal):
        conditions = ConditionCache(self.g)
        for (node, _, expander_dict) in traversal:
            for expander_list in expander_dict.values():
                for expander in expander_list:
                    for traverser_list in expander.traversers.values():
                        for traverser in traverser_list:
                            for node_dispatcher in traverser.node:
                                if not node_dispatcher.condition:
                                    continue

                                conditions.register(node, node_dispatcher.condition)
                            for edge_dispatcher in traverser.edge:
                                if not edge_dispatcher.condition:
                                    continue

                                # TODO: is it fine to register it under the "node"?
                                conditions.register(node, edge_dispatcher.condition)

        return conditions

    @staticmethod
    def _compute_children(traversal: list) -> dict[rdflib.URIRef, list[rdflib.URIRef]]:
        child_map = {}
        for (node, _, _) in traversal:
            child_map[node] = {}

        # TODO: decompose by sweep or query
        for (node, parent, expander_dict) in traversal:
            if not parent:
                continue

            for q_expander, expander_list in expander_dict.items():
                if q_expander not in child_map[parent]:
                    child_map[parent][q_expander] = []
                child_map[parent][q_expander].append(node)

        return child_map

    @staticmethod
    def _init_state(traversal):
        state = {}
        for (node, _, _) in traversal:
            state[node] = {}
        return state

    def _execute_sweep(self, sweep, func):
        if sweep.direction == SweepDirection.OUTWARD:
            self._outward(sweep, func)
        else:
            self._inward(sweep, func)

    def _outward(self, sweep, func):
        for (current, parent, expander_dict) in self.traversal:
            log(current)
            for expander_list in expander_dict.values():
                for expander in expander_list:
                    # The current node has not been expanded/discovered by any
                    # step that is associated with the current sweep -> just skip it
                    if sweep not in expander.traversers:
                        continue

                    for traverser in expander.traversers[sweep]:
                        self._trig_dispatchers_out(traverser, current, parent, func)
            log()

    def _inward(self, sweep, func):
        for (current, _, expander_dict) in reversed(self.traversal):
            log(current)
            for expander_list in expander_dict.values():
                for expander in expander_list:
                    # The current node has not been expanded/discovered by any
                    # step that is associated with the current sweep -> just skip it
                    if sweep not in expander.traversers:
                        continue

                    for traverser in expander.traversers[sweep]:
                        self._trig_dispatchers_in(traverser, current, func)
            log()

    def _trig_dispatchers_out(self, traverser, current, parent, func):
        for dispatcher in traverser.node:
            self._dispatch_to_node(current, dispatcher, func)

        # Always visit the current _node_ (see above), but visit the edge only
        # if we are not at the root (which does not have a parent)
        if not parent:
            return

        for dispatcher in traverser.edge:
            if not self._should_dispatch(current, dispatcher):
                continue

            fn = getattr(dispatcher, func)
            log("edge:", fn)
            if fn:
                fn(self.state, parent, current)

    def _trig_dispatchers_in(self, traverser, current, func):
        for dispatcher in traverser.node:
            self._dispatch_to_node(current, dispatcher, func)

        for dispatcher in traverser.edge:
            if not self._should_dispatch(current, dispatcher):
                continue

            fn = getattr(dispatcher, func)
            log("edge:", fn)

            # Only trigger the function if it exists (not None) and there are
            # children associated with the expander query that we are currently
            # handling
            if fn and traverser.expander in self.children[current]:
                fn(self.state, current, self.children[current][traverser.expander])

    def _dispatch_to_node(self, node, dispatcher, func):
        if not self._should_dispatch(node, dispatcher):
            return

        fn = getattr(dispatcher, func)
        log("node:", fn)
        if fn:
            fn(self.state, node)

    def _should_dispatch(self, node, dispatcher):
        # None is a wildcard
        if not dispatcher.condition:
            return True

        return bool(self.conditions.node[node][dispatcher.condition])
