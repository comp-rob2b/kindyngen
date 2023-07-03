# SPDX-License-Identifier: MPL-2.0
from kindynsyn.synthesizer.synthesizer import SweepDirection, SweepConfig, \
    SolverConfig
from kindynsyn.synthesizer.graph_factories import (
    SpatialRelations, SpatialRelationsCoordinates,
    SpatialRelationsWithCoordinates, KinematicChainState,
    KinematicChainOperators, DynamicsEntities, DynamicsEntitiesCoordinates,
    DynamicsEntitiesWithCoordinates
)
from kindynsyn.synthesizer.steps import (
    ChainIndexStep,
    JointState, JointStep, JointDynamicsStep,
    PositionPropagationStep, PositionAccumulationStep,
    VelocityPropagationStep, AccelerationPropagationStep,
    RigidBodyInertiaStep,
    InertialForceStep, QuasiStaticInertialForcePropagationState,
    QuasiStaticInertialForcePropagationStep,
    QuasiStaticExternalForcePropagationState,
)


def solver_configurator(g, cache, ROB, slv_algo):
    frm_world = ROB["world-frame"]

    # Instantiate factories
    geom_rel = SpatialRelations(g)
    geom_coord = SpatialRelationsCoordinates(g)
    geom = SpatialRelationsWithCoordinates(geom_rel, geom_coord)
    dyn_ent = DynamicsEntities(g)
    dyn_coord = DynamicsEntitiesCoordinates(g)
    dyn = DynamicsEntitiesWithCoordinates(dyn_ent, dyn_coord)
    kc_stat = KinematicChainState(g)
    kc = KinematicChainOperators(g)

    # Instantiate "standard" solver steps
    index = ChainIndexStep(g, cache, frm_world)
    j_mot = JointStep(g, cache, slv_algo)
    j_dyn = JointDynamicsStep(g, cache, kc_stat, slv_algo)
    pos_prop = PositionPropagationStep(g, cache, slv_algo, geom, geom_coord, kc)
    pos_acc = PositionAccumulationStep(g, cache, slv_algo, geom, geom_coord)
    vel_prop = VelocityPropagationStep(g, cache, slv_algo, geom, geom_coord, kc)
    acc_prop = AccelerationPropagationStep(g, cache, slv_algo, geom, geom_coord, kc)
    rbi = RigidBodyInertiaStep(g, cache, slv_algo, dyn)
    f_nrt = InertialForceStep(slv_algo, dyn_coord, dyn)
    nrt_prop = QuasiStaticInertialForcePropagationStep(slv_algo, dyn_coord, dyn, kc, kc_stat)

    # Configure solver
    out_1 = SweepConfig(
        direction=SweepDirection.OUTWARD,
        steps=[index, j_mot, j_dyn, pos_prop, pos_acc, vel_prop, acc_prop])
    in_1 = SweepConfig(
        direction=SweepDirection.INWARD,
        steps=[rbi, f_nrt, nrt_prop])

    return SolverConfig(sweeps=[out_1, in_1])


def translator_configurator():
    return []
