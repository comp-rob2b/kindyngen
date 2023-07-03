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
    QuasiStaticExternalForcePropagationState
)
from kindynsyn_tutorial.my_solver import MySolverStep, \
    AccumulateJointForceTranslator
from kindynsyn_tutorial.my_robot_interface import (
    MY_IF, RobotInterfaceMeasurementStep, RobotInterfaceCommandStep,
    JointConfigurationToSolverTranslator, JointConfigurationFromSolverTranslator
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

    # Instantiate solver extension
    my_slv = MySolverStep(g, slv_algo, [
        QuasiStaticInertialForcePropagationState
    ])

    # Instantiate robot interface extension
    joint_index = {
        ROB["q1"]: 0, ROB["qd1"]: 0, ROB["tau1"]: 0,
        ROB["q2"]: 1, ROB["qd2"]: 1, ROB["tau2"]: 1,
        ROB["q3"]: 2, ROB["qd3"]: 2, ROB["tau3"]: 2,
        ROB["q4"]: 3, ROB["qd4"]: 3, ROB["tau4"]: 3,
        ROB["q5"]: 4, ROB["qd5"]: 4, ROB["tau5"]: 4,
        ROB["q6"]: 5, ROB["qd6"]: 5, ROB["tau6"]: 5,
        ROB["q7"]: 6, ROB["qd7"]: 6, ROB["tau7"]: 6
    }
    if_msr = RobotInterfaceMeasurementStep(g, slv_algo, joint_index)
    if_cmd = RobotInterfaceCommandStep(g, slv_algo, JointState, joint_index)

    # Configure solver
    out_1 = SweepConfig(
        direction=SweepDirection.OUTWARD,
        steps=[index, j_mot, j_dyn, if_msr, pos_prop, pos_acc, vel_prop, acc_prop])
    in_1 = SweepConfig(
        direction=SweepDirection.INWARD,
        steps=[rbi, f_nrt, nrt_prop])
    out_2 = SweepConfig(
        direction=SweepDirection.OUTWARD,
        steps=[my_slv, if_cmd])

    return SolverConfig(sweeps=[out_1, in_1, out_2])


def translator_configurator():
    return [
        AccumulateJointForceTranslator(),
        JointConfigurationToSolverTranslator(MY_IF["JointPositionToSolver"], "joint-position-to-solver"),
        JointConfigurationToSolverTranslator(MY_IF["JointVelocityToSolver"], "joint-velocity-to-solver"),
        JointConfigurationFromSolverTranslator(MY_IF["JointForceFromSolver"], "joint-force-from-solver")
    ]
