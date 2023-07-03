# SPDX-License-Identifier: MPL-2.0
import typing
import enum
import collections
import itertools
import rdflib.plugins.sparql

class Direction(enum.Enum):
    IN = 1
    OUT = 2
    IN_OUT = 3


class OpenSet(typing.Protocol):
    def is_empty(self) -> bool:
        ...

    def insert(self, node: rdflib.URIRef) -> None:
        ...

    def remove(self) -> rdflib.URIRef:
        ...

class Expander(typing.Protocol):
    def root(self, node: rdflib.URIRef) -> list:
        ...

    def expand(self, node: rdflib.URIRef) -> dict:
        ...


class BreadthFirst:
    def __init__(self):
        self.queue = collections.deque()

    def is_empty(self):
        return len(self.queue) <= 0

    def insert(self, node):
        self.queue.append(node)

    def remove(self):
        return self.queue.popleft()

class DepthFirst:
    def __init__(self):
        self.stack = []

    def is_empty(self):
        return len(self.stack) <= 0

    def insert(self, node):
        self.stack.append(node)

    def remove(self):
        return self.stack.pop()


def traverse_nodes_with_parent_user(open_set_ds: type[OpenSet],
        root: rdflib.URIRef, expander: Expander):
    visited = {}
    open_set = open_set_ds()
    open_set.insert(root)
    parent = {root: None}
    user = {root: expander.root(root)}

    while not open_set.is_empty():
        node = open_set.remove()
        if node in visited:
            continue

        yield (node, parent[node], user[node])
        visited[node] = True

        for child, u in expander.expand(node).items():
            parent[child] = node
            user[child] = u
            open_set.insert(child)

