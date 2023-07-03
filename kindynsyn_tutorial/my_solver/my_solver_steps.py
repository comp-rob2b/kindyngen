# SPDX-License-Identifier: MPL-2.0
from rdflib import collection, BNode, RDF
from kindynsyn.rdflib_tools import uuid_ref
from kindynsyn.synthesizer import Traverser, Dispatcher
from kindynsyn.synthesizer.steps import JointState, q_expand
from .namespace import MY_SLV


class MySolverStep:
    """
    This step accumulates joint forces ...
    - ... from all states given in the "source_classes" (they must have a "tau"
      variable)...
    - ... to the accumulator "JointState.tau"

    This step must be deployed in an outward sweep.
    """

    def __init__(self, g, algo, source_classes):
        self.g = g
        self.algo = algo
        self.source_classes = source_classes


    def traverse(self):
        """
        Set up the traverser for this step.
        """

        # Unconditionally dispatch to computation (no configuration required)
        disp = Dispatcher(condition=None, configure=None,
            compute=self.compute_edge)

        # Reuse the "default" root-to-root expander. Only dispatch on the edge
        # between the two segments' root frames because that's where the joint
        # lies (and we want to compute a joint-level quantity). This also
        # ensures that we don't visit more nodes than there are joints in the
        # kinematic chain (a serial chain has one more links than joints).
        return Traverser(expander=q_expand, edge=[disp])


    def compute_edge(self, state, parent, child):
        """
        The "parent" parameter is not accessed because we don't propagate any
        quantity from parent to child (also see comment about the edge
        dispatcher above).
        """

        # Collect the joint forces from each source
        sources = []
        for source_class in self.source_classes:
            sources.append(state[child][source_class].tau)

        # Get a handle to the joint state which is used as the destination, i.e.
        # it will contain the accumulated joint forces
        jnt = state[child][JointState]

        # Emit an operation to accumulate joint forces
        acc = self.accumulate_joint_force(sources, jnt.tau)

        # Register the operation so that it will be included in the final
        # schedule of the synthesized algorithm
        self.algo["func"].extend([acc])


    def accumulate_joint_force(self, sources, destination):
        # Create an RDF list from the provided sources
        lst = BNode()
        collection.Collection(self.g, lst, sources)

        # Compute a unique identifier (here, a UUID) that represents the
        # accumulation operation
        id_ = uuid_ref()

        # Assign the type to the identifier (here, the custom type defined in
        # the MY_SLV namespace associated with this module)
        self.g.add((id_, RDF["type"], MY_SLV["AccumulateJointForce"]))

        # Associate the source list and the (single) destination with the
        # operation, using the "sources" and "destination" predicates as defined
        # in the MY_SLV namespace
        self.g.add((id_, MY_SLV["sources"], lst))
        self.g.add((id_, MY_SLV["destination"], destination))

        # Return the unique identifier as representative of the operation
        return id_
