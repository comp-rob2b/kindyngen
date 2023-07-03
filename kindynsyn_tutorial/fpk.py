# SPDX-License-Identifier: MPL-2.0
from kindynsyn.synthesizer.synthesizer import SweepDirection, SweepConfig, \
    SolverConfig
from kindynsyn.synthesizer.graph_factories import (
    SpatialRelations, SpatialRelationsCoordinates,
    SpatialRelationsWithCoordinates,
    KinematicChainOperators
)
from kindynsyn.synthesizer.steps import (
    ChainIndexStep, JointStep,
    PositionPropagationStep, PositionAccumulationStep
)


def solver_configurator(g, cache, ROB, slv_algo):
    frm_world = ROB["world-frame"]

    # Instantiate factories
    geom_rel = SpatialRelations(g)
    geom_coord = SpatialRelationsCoordinates(g)
    geom = SpatialRelationsWithCoordinates(geom_rel, geom_coord)
    kc = KinematicChainOperators(g)

    # Instantiate "standard" solver steps
    index = ChainIndexStep(g, cache, frm_world)
    j_mot = JointStep(g, cache, slv_algo)
    pos_prop = PositionPropagationStep(g, cache, slv_algo, geom, geom_coord, kc)
    pos_acc = PositionAccumulationStep(g, cache, slv_algo, geom, geom_coord)

    # Configure solver
    out_1 = SweepConfig(
        direction=SweepDirection.OUTWARD,
        steps=[index, j_mot, pos_prop, pos_acc])

    return SolverConfig(sweeps=[out_1])


def translator_configurator():
    return []
