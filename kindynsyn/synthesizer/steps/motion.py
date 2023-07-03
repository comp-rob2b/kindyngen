# SPDX-License-Identifier: MPL-2.0
from dataclasses import dataclass, field
from rdflib import URIRef
from kindynsyn.synthesizer.synthesizer import Traverser, Dispatcher
from kindynsyn.synthesizer.steps import ChainIndexState, JointState, q_expand, \
    q_root


@dataclass
class PositionPropagationState:
    x_seg: URIRef | None = field(default=None)           # Pose across segment (this.frm_prox w.r.t. parent.frm_prox)
    # Intermediate
    x_jnt: URIRef | None = field(default=None)           # Pose across segment's joint (this.frm_prox w.r.t. this.frm_par_dist)
    x_lnk: URIRef | None = field(default=None)           # Pose across segment's link (this.frm_par_dist w.r.t. parent.frm_prox)


class PositionPropagationStep:
    """
    {i+1}X_i = X_{j,i}(q) X_{l,i}
    """
    def __init__(self, g, load, algo, geom, geom_coord, kc):
        self.g = g
        self.geom = geom
        self.geom_coord = geom_coord
        self.kc = kc
        self.algo = algo
        self.sel_tf = load("select_transform_from_frames.rq")

    def traverse(self):
        return Traverser(
            expander=q_expand,
            edge=[Dispatcher(None, self.configure, self.compute)])

    def configure(self, state, parent, child):
        cur = state[child][ChainIndexState]
        par = state[parent][ChainIndexState]

        tf = list(self.g.query(self.sel_tf, initBindings={
            "of_frame": cur.frm_par_dist,
            "with_respect_to_frame": par.frm_prox
        }))[0]

        x_jnt = self.geom.pose(of=cur.frm_prox, with_respect_to=cur.frm_par_dist)
        x_seg = self.geom.pose(of=cur.frm_prox, with_respect_to=par.frm_prox)

        s = PositionPropagationState()
        s.x_lnk = tf["tf"]
        s.x_jnt = x_jnt
        s.x_seg = x_seg
        state[child][PositionPropagationState] = s

        self.algo["data"].extend([s.x_lnk, s.x_jnt, s.x_seg])

    def compute(self, state, parent, child):
        idx = state[child][ChainIndexState]
        jnt = state[child][JointState]
        pos = state[child][PositionPropagationState]

        to_cart = self.kc.joint_position_to_pose(
            joint=idx.joint,
            joint_position=jnt.q,
            pose=pos.x_jnt)
        comp_seg = self.geom_coord.compose_pose(
            in1=pos.x_lnk,
            in2=pos.x_jnt,
            composite=pos.x_seg)

        self.algo["func"].extend([to_cart, comp_seg])



@dataclass
class PositionAccumulationState:
    x_tot: URIRef | None = field(default=None)           # Pose of segment's link w.r.t. to root (this.frm_prox w.r.t. root.frm_prox)


class PositionAccumulationStep:
    """
    {i+1}X_0 = {i+1}X_{i} {i}X_{0}
    """
    def __init__(self, g, load, algo, geom, geom_coord):
        self.g = g
        self.geom = geom
        self.geom_coord = geom_coord
        self.algo = algo
        self.sel_tf = load("select_transform_from_frames.rq")

    def traverse(self):
        return Traverser(
            expander=q_expand,
            edge=[Dispatcher(compute=self.compute)],
            node=[Dispatcher(configure=self.configure)]
        )

    def configure(self, state, node):
        idx = state[node][ChainIndexState]

        res = list(self.g.query(self.sel_tf, initBindings={
            "of_frame": idx.frm_prox,
            "with_respect_to_frame": idx.frm_root
        }))

        if res:
            # The transform already exists (use that one)
            x_tot = res[0]["tf"]
        else:
            # Create a new transform for the root
            x_tot = self.geom.pose(of=idx.frm_prox, with_respect_to=idx.frm_root)

        s = PositionAccumulationState()
        s.x_tot = x_tot
        state[node][PositionAccumulationState] = s

        self.algo["data"].extend([s.x_tot])

    def compute(self, state, parent, child):
        par = state[parent]
        cur = state[child]

        comp_tot = self.geom_coord.compose_pose(
            in1=par[PositionAccumulationState].x_tot,
            in2=cur[PositionPropagationState].x_seg,
            composite=cur[PositionAccumulationState].x_tot)

        self.algo["func"].extend([comp_tot])



@dataclass
class VelocityPropagationState:
    xd_jnt: URIRef | None = field(default=None)          # Velocity across segment's joint
    xd_tot: URIRef | None = field(default=None)          # Velocity of segment's body w.r.t. root
    # Intermediate
    xd_tot_par_tf: URIRef | None = field(default=None)   # Parent link's velocity w.r.t. root transformed to segment's proximal link frame


class VelocityPropagationStep:
    """
    Xd_{i+1} = {i+1}X_i Xd_i + S_i qd_i
    """
    def __init__(self, g, load, algo, geom, geom_coord, kc):
        self.g = g
        self.geom = geom
        self.geom_coord = geom_coord
        self.algo = algo
        self.kc = kc
        self.sel_mot_lnk = load("select_body_motion_state.rq")

    def traverse(self):
        return Traverser(
            expander=q_expand,
            edge=[Dispatcher(None, self.configure, self.compute)],
            node=[Dispatcher(q_root, self.configure_root, None)]
        )

    def configure_root(self, state, node):
        cur = state[node][ChainIndexState]

        res = list(self.g.query(self.sel_mot_lnk, initBindings={
            "of_body": cur.bdy,
            "with_respect_to_body": cur.bdy_root
        }))

        if res:
            # The motion state already exists (use that one)
            xd_tot = res[0]["velocity_twist"]
        else:
            # Create a new motion state for the root
            xd_tot = self.geom.velocity_twist(of=cur.bdy, with_respect_to=cur.bdy, as_seen_by=cur.frm_prox)

        s = VelocityPropagationState()
        s.xd_tot = xd_tot
        state[node][VelocityPropagationState] = s

        self.algo["data"].extend([s.xd_tot])

    def configure(self, state, parent, child):
        cur = state[child][ChainIndexState]
        par = state[parent][ChainIndexState]

        xd_jnt        = self.geom.velocity_twist(of=cur.bdy, with_respect_to=par.bdy,      as_seen_by=cur.frm_prox)
        xd_tot_par_tf = self.geom.velocity_twist(of=par.bdy, with_respect_to=cur.bdy_root, as_seen_by=cur.frm_prox)
        xd_tot        = self.geom.velocity_twist(of=cur.bdy, with_respect_to=cur.bdy_root, as_seen_by=cur.frm_prox)

        s = VelocityPropagationState()
        s.xd_jnt        = xd_jnt
        s.xd_tot_par_tf = xd_tot_par_tf
        s.xd_tot        = xd_tot
        state[child][VelocityPropagationState] = s

        self.algo["data"].extend([s.xd_jnt, s.xd_tot_par_tf, s.xd_tot])

    def compute(self, state, parent, child):
        par = state[parent]
        cur = state[child]

        to_cart = self.kc.joint_velocity_to_velocity_twist(
            joint=cur[ChainIndexState].joint,
            joint_velocity=cur[JointState].qd,
            velocity_twist=cur[VelocityPropagationState].xd_jnt)
        tf = self.geom_coord.transform_velocity_twist_to_distal(
            pose=cur[PositionPropagationState].x_seg,
            frm=par[VelocityPropagationState].xd_tot,
            to=cur[VelocityPropagationState].xd_tot_par_tf)
        comp = self.geom_coord.add_velocity_twist(
            in1=cur[VelocityPropagationState].xd_tot_par_tf,
            in2=cur[VelocityPropagationState].xd_jnt,
            composite=cur[VelocityPropagationState].xd_tot)

        self.algo["func"].extend([to_cart, tf, comp])



@dataclass
class AccelerationPropagationState:
    xdd_tot: URIRef | None = field(default=None)        # Acceleration of segment's link w.r.t. root
    # Intermediate
    xdd_jnt: URIRef | None = field(default=None)        # Acceleration across segment's joint
    xdd_tot_par_tf: URIRef | None = field(default=None) # Parent link's acceleration w.r.t. root transformed to segment's proximal link frame


class AccelerationPropagationStep:
    """
    Xdd_{i+1} = {i+1}X_i Xdd_i + S_i qdd_i + Xd_i x S_i qd_i
    """
    def __init__(self, g, load, algo, geom, geom_coord, kc):
        self.g = g
        self.geom = geom
        self.geom_coord = geom_coord
        self.algo = algo
        self.kc = kc
        self.sel_mot_lnk = load("select_body_motion_state.rq")

    def traverse(self):
        return Traverser(
            expander=q_expand,
            edge=[Dispatcher(None, self.configure, self.compute)],
            node=[Dispatcher(q_root, self.configure_root, None)]
        )

    def configure_root(self, state, node):
        cur = state[node][ChainIndexState]

        res = list(self.g.query(self.sel_mot_lnk, initBindings={
            "of_body": cur.bdy,
            "with_respect_to_body": cur.bdy_root
        }))

        if res:
            # The motion state already exists (use that one)
            xdd_tot = res[0]["acceleration_twist"]
        else:
            # Create a new motion state for the root
            xdd_tot = self.geom.acceleration_twist(of=cur.bdy, with_respect_to=cur.bdy, as_seen_by=cur.frm_prox)


        s = AccelerationPropagationState()
        s.xdd_tot = xdd_tot
        state[node][AccelerationPropagationState] = s

        self.algo["data"].extend([s.xdd_tot])

    def configure(self, state, parent, child):
        cur = state[child][ChainIndexState]
        par = state[parent][ChainIndexState]

        xdd_jnt        = self.geom.acceleration_twist(of=cur.bdy, with_respect_to=par.bdy,      as_seen_by=cur.frm_prox)
        xdd_tot_par_tf = self.geom.acceleration_twist(of=par.bdy, with_respect_to=cur.bdy_root, as_seen_by=cur.frm_prox)
        xdd_tot        = self.geom.acceleration_twist(of=cur.bdy, with_respect_to=cur.bdy_root, as_seen_by=cur.frm_prox)

        s = AccelerationPropagationState()
        s.xdd_jnt        = xdd_jnt
        s.xdd_tot_par_tf = xdd_tot_par_tf
        s.xdd_tot        = xdd_tot
        state[child][AccelerationPropagationState] = s

        self.algo["data"].extend([s.xdd_jnt, s.xdd_tot_par_tf, s.xdd_tot])

    def compute(self, state, parent, child):
        par = state[parent]
        cur = state[child]

        to_cart = self.kc.joint_acceleration_to_acceleration_twist(
            joint=cur[ChainIndexState].joint,
            joint_acceleration=cur[JointState].qdd,
            acceleration_twist=cur[AccelerationPropagationState].xdd_jnt)
        tf = self.geom_coord.transform_acceleration_twist_to_distal(
            pose=cur[PositionPropagationState].x_seg,
            absolute_velocity=par[VelocityPropagationState].xd_tot,
            relative_velocity=cur[VelocityPropagationState].xd_jnt,
            frm=par[AccelerationPropagationState].xdd_tot,
            to=cur[AccelerationPropagationState].xdd_tot_par_tf)
        comp = self.geom_coord.add_acceleration_twist(
            in1=cur[AccelerationPropagationState].xdd_tot_par_tf,
            in2=cur[AccelerationPropagationState].xdd_jnt,
            composite=cur[AccelerationPropagationState].xdd_tot)

        self.algo["func"].extend([to_cart, tf, comp])
