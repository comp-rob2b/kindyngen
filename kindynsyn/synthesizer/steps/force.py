# SPDX-License-Identifier: MPL-2.0
from dataclasses import dataclass, field
from rdflib import URIRef
from kindynsyn.namespaces import RBDYN_COORD
from kindynsyn.synthesizer.synthesizer import Traverser, Dispatcher
from kindynsyn.synthesizer.steps import (
    ChainIndexState,
    PositionPropagationState, VelocityPropagationState,
    AccelerationPropagationState, RigidBodyInertiaState,
    q_expand, q_leaf
)


@dataclass
class InertialForceState:
    f_nrt_prox: URIRef | None = field(default=None)     # Rigid-body inertial force acting on segment in sgement's proximal link frame


class InertialForceStep:
    """
    F_{b,i} = Xd_i x* (M_i Xd_i)
    """
    def __init__(self, algo, dyn_coord, dyn):
        self.dyn_coord = dyn_coord
        self.dyn = dyn
        self.algo = algo

    def traverse(self):
        return Traverser(
            expander=q_expand,
            node=[Dispatcher(None, self.configure, self.compute)]
        )

    def configure(self, state, node):
        idx = state[node][ChainIndexState]

        f_nrt_prox = self.dyn.wrench(acts_on=idx.bdy, as_seen_by=idx.frm_prox, number_of_wrenches=1)

        s = InertialForceState()
        s.f_nrt_prox = f_nrt_prox
        state[node][InertialForceState] = s

        self.algo["data"].extend([s.f_nrt_prox])

    def compute(self, state, node):
        rbi = state[node][RigidBodyInertiaState]
        vel = state[node][VelocityPropagationState]
        frc = state[node][InertialForceState]

        to_fb = self.dyn_coord.inertial_wrench(
            rigid_body_inertia=rbi.m_scr_prox,
            velocity_twist=vel.xd_tot,
            wrench=frc.f_nrt_prox)

        self.algo["func"].extend([to_fb])



@dataclass
class QuasiStaticInertialForcePropagationState:
    f_cur_prox: URIRef | None = field(default=None)     # Inertial force acting on segment's link (including propagated child forces)
    tau: URIRef | None = field(default=None)            # Joint force (computed when visting the parent segment)
    # Intermediate
    f_par_tf: URIRef | None = field(default=None)       # Inertial force acting on segment's link (transformed to parent's proximal link frame, computed when visiting the parent segment)


class QuasiStaticInertialForcePropagationStep:
    """
    tau_c = S_c^T F_c
    F_p = (M_p Xdd_p + v_p x* M_p v_p) + \sum_c X^T F_c
        = (M_p Xdd_p +     F_{b,p}   ) + \sum_c X^T F_c
    """
    def __init__(self, algo, dyn_coord, dyn, kc, kc_stat):
        self.dyn_coord = dyn_coord
        self.dyn = dyn
        self.algo = algo
        self.kc = kc
        self.kc_stat = kc_stat

    def traverse(self):
        return Traverser(
            expander=q_expand,
            edge=[Dispatcher(None, self.configure_branch, self.compute_branch)],
            node=[Dispatcher(None, self.configure_node, self.compute_node)]
        )

    def configure_node(self, state, node):
        idx = state[node][ChainIndexState]

        f_cur_prox = self.dyn.wrench(acts_on=idx.bdy, as_seen_by=idx.frm_prox, number_of_wrenches=1)

        s = QuasiStaticInertialForcePropagationState()
        s.f_cur_prox = f_cur_prox
        state[node][QuasiStaticInertialForcePropagationState] = s

        self.algo["data"].extend([s.f_cur_prox])

    def configure_branch(self, state, parent, children):
        par = state[parent][ChainIndexState]

        # The children have already been visited before, so their state exists
        for child in children:
            cld = state[child][ChainIndexState]

            # TODO: do we need an "acts_on" and if so which one to use here!
            #f_par_tf = self.dyn.wrench(acts_on=cld.bdy, as_seen_by=par.frm_prox, number_of_wrenches=1)
            f_par_tf = self.dyn.wrench(acts_on=par.bdy, as_seen_by=par.frm_prox, number_of_wrenches=1)
            tau = self.kc_stat.joint_force(joint=cld.joint, number_of_elements=1)

            s = state[child][QuasiStaticInertialForcePropagationState]
            s.f_par_tf = f_par_tf
            s.tau = tau

            self.algo["data"].extend([s.f_par_tf, s.tau])

    def compute_node(self, state, node):
        rbi = state[node][RigidBodyInertiaState]
        acc = state[node][AccelerationPropagationState]
        nrt = state[node][InertialForceState]
        prp = state[node][QuasiStaticInertialForcePropagationState]

        # M Xdd + F_b
        to_f = self.dyn_coord.acceleration_twist_to_wrench_with_rigid_body_inertia(
            rigid_body_inertia=rbi.m_scr_prox,
            acceleration_twist=acc.xdd_tot,
            wrench=prp.f_cur_prox)
        accu = self.dyn_coord.accumulate_wrench(
            aggregate=prp.f_cur_prox,
            new_element=nrt.f_nrt_prox,
            number_of_wrenches=1)

        self.algo["func"].extend([to_f, accu])

    def compute_branch(self, state, parent, children):
        par = state[parent][QuasiStaticInertialForcePropagationState]

        for child in children:
            idx = state[child][ChainIndexState]
            pos = state[child][PositionPropagationState]
            frc = state[child][QuasiStaticInertialForcePropagationState]

            # tau_c = S_c^T F_c
            to_jnt = self.kc.joint_force_from_wrench(
                joint=idx.joint,
                wrench=frc.f_cur_prox,
                joint_force=frc.tau,
                number_of_wrenches=1)
            # F'_c = X* F_c
            tf = self.dyn_coord.transform_wrench_to_proximal(
                pose=pos.x_seg,
                frm=frc.f_cur_prox,
                to=frc.f_par_tf,
                number_of_wrenches=1,
                at_index=0)
            # F_p += F'_c
            acc = self.dyn_coord.accumulate_wrench(
                aggregate=par.f_cur_prox,
                new_element=frc.f_par_tf,
                number_of_wrenches=1)

            self.algo["func"].extend([to_jnt, tf, acc])



@dataclass
class QuasiStaticExternalForcePropagationState:
    f_cur_prox: URIRef | None = field(default=None)     # External force acting on segment's link (including propagated child forces)
    f_ext: URIRef | None = field(default=None)          # Only the specified external forces on the current link
    tau: URIRef | None = field(default=None)            # Joint force (computed when visting the parent segment)
    size: int = field(default=0)                        # Number of external forces specified for the current link
    size_acc: int = field(default=0)                    # Number of external forces accumulated from leaves and intermediate links down to current link
    idx_par: int = field(default = 0)                   # Index to where the external forces should be copied in the parent's array


class QuasiStaticExternalForcePropagationStep:
    """
    tau_c = S_c^T F_c
    F_p = F_ext + \sum_c X^T F_c

    Layout of f_cur_prox:
    [f_{ext,p}, f_{ext,c_1}, ..., f_{ext,c_n}]
    """
    def __init__(self, g, load, algo, dyn_coord, dyn, kc, kc_stat):
        self.g = g
        self.dyn_coord = dyn_coord
        self.dyn = dyn
        self.algo = algo
        self.kc = kc
        self.kc_stat = kc_stat
        self.sel_ext = load("select_external_force.rq")

    def traverse(self):
        return Traverser(
            expander=q_expand,
            edge=[Dispatcher(None, self.configure_branch, self.compute_branch)],
            node=[Dispatcher(q_leaf, self.configure_leaf, self.compute_leaf)]
        )

    def configure_leaf(self, state, node):
        idx = state[node][ChainIndexState]

        # TODO: Handle specification in root frame
        res = self.g.query(self.sel_ext, initBindings={"frame": idx.frm_prox, "body": idx.bdy})
        if not res:
            # There was no external force specified for this link
            return

        f_ext = list(res)[0]["force"]

        size = int(self.g.value(f_ext, RBDYN_COORD["number-of-wrenches"]))
        f_cur_prox = self.dyn.wrench(acts_on=idx.bdy, as_seen_by=idx.frm_prox, number_of_wrenches=size)

        s = QuasiStaticExternalForcePropagationState()
        s.f_cur_prox = f_cur_prox
        s.f_ext = f_ext
        s.size = size
        s.size_acc = size
        state[node][QuasiStaticExternalForcePropagationState] = s

        self.algo["data"].extend([s.f_cur_prox, s.f_ext])

    def configure_branch(self, state, parent, children):
        '''
        Strategy:
        1. Find out how many "local" external forces have been specified on the
           current link.
        2. Iterate over children to:
           a) setup the transformed wrenches & the joint force
           b) accumulate how many wrenches are propagated for all children
        3. Allocate the required amount of wrenches (sum of "local" and
           propagated count)
        '''
        par = state[parent][ChainIndexState]

        size = 0    # TODO: extract from graph
        size_acc = size

        # The children have already been visited before, so their state exists
        for child in children:
            cld = state[child][ChainIndexState]

            try:
                s = state[child][QuasiStaticExternalForcePropagationState]
            except KeyError:
                # There was no external force specified for that child
                continue

            tau = self.kc_stat.joint_force(joint=cld.joint, number_of_elements=s.size_acc)
            s.tau = tau
            s.idx_par = size_acc

            # Accumulate the number of wrenches specified on the children
            size_acc += s.size_acc

            self.algo["data"].extend([s.tau])

        # Setup the state for the parent
        f_cur_prox = self.dyn.wrench(acts_on=par.bdy, as_seen_by=par.frm_prox, number_of_wrenches=size_acc)

        s = QuasiStaticExternalForcePropagationState()
        s.f_cur_prox = f_cur_prox
        s.size = size
        s.size_acc = size_acc
        state[parent][QuasiStaticExternalForcePropagationState] = s

        self.algo["data"].extend([s.f_cur_prox])

    def compute_leaf(self, state, node):
        try:
            prp = state[node][QuasiStaticExternalForcePropagationState]
        except KeyError:
            # There was no external force specified for this link
            return

        cpy = self.dyn_coord.assign_wrench(
            frm=prp.f_ext,
            to=prp.f_cur_prox)

        self.algo["func"].extend([cpy])

    def compute_branch(self, state, parent, children):
        acc = state[parent][AccelerationPropagationState]
        nrt = state[parent][InertialForceState]
        prp = state[parent][QuasiStaticExternalForcePropagationState]

        for child in children:
            idx = state[child][ChainIndexState]
            pos = state[child][PositionPropagationState]

            try:
                frc = state[child][QuasiStaticExternalForcePropagationState]
            except KeyError:
                # There was no external force specified for that child
                continue

            # tau_c = S_c^T F_c
            to_jnt = self.kc.joint_force_from_wrench(
                joint=idx.joint,
                wrench=frc.f_cur_prox,
                joint_force=frc.tau,
                number_of_wrenches=frc.size_acc)
            # F'_c = X* F_c
            tf = self.dyn_coord.transform_wrench_to_proximal(
                pose=pos.x_seg,
                frm=frc.f_cur_prox,
                to=prp.f_cur_prox,
                number_of_wrenches=frc.size_acc,
                at_index=frc.idx_par)

            self.algo["func"].extend([to_jnt, tf])
