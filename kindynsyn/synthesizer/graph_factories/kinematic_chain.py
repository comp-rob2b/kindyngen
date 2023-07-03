# SPDX-License-Identifier: MPL-2.0
from rdflib import Literal, RDF
from kindynsyn.namespaces import GEOM_ENT, GEOM_REL, GEOM_COORD, RBDYN_COORD, \
    KC_ENT, KC_STAT, KC_OP, QUDT_SCHEMA, QUDT_QKIND, QUDT_UNIT
from kindynsyn.rdflib_tools.helpers import uuid_ref


class KinematicChainState:
    def __init__(self, g):
        self.g = g

    def joint_position(self, joint):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], KC_STAT["JointPosition"]))
        self.g.add((id_, RDF["type"], KC_STAT["JointPositionCoordinate"]))
        self.g.add((id_, KC_STAT["joint"], joint))
        return id_

    def joint_force(self, joint, number_of_elements):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], KC_STAT["JointForce"]))
        self.g.add((id_, RDF["type"], KC_STAT["JointForceCoordinate"]))
        self.g.add((id_, KC_STAT["of-joint"], joint))
        self.g.add((id_, KC_STAT["number-of-elements"], Literal(number_of_elements)))
        return id_

    def joint_inertia(self, joint):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], KC_STAT["JointInertia"]))
        self.g.add((id_, RDF["type"], KC_STAT["JointInertiaCoordinate"]))
        self.g.add((id_, KC_STAT["of-joint"], joint))
        return id_


class KinematicChainOperators:
    def __init__(self, g):
        self.g = g

    def joint_position_to_pose(self, joint, joint_position, pose):
        assert joint == self.g.value(joint_position, KC_STAT["of-joint"])
        pose_frames = {
            self.g.value(pose, GEOM_COORD["of-pose"] / GEOM_REL["of"]),
            self.g.value(pose, GEOM_COORD["of-pose"] / GEOM_REL["with-respect-to"])}
        assert pose_frames == set(self.g[joint : KC_ENT["between-attachments"]])

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], KC_OP["JointPositionToPose"]))
        self.g.add((id_, KC_OP["joint"], joint))
        self.g.add((id_, KC_OP["joint-position"], joint_position))
        self.g.add((id_, KC_OP["pose"], pose))
        return id_

    def joint_velocity_to_velocity_twist(self, joint, joint_velocity, velocity_twist):
        assert joint == self.g.value(joint_velocity, KC_STAT["of-joint"])
        assert self.g.value(velocity_twist, GEOM_COORD["as-seen-by"]) in set(self.g[joint : KC_ENT["between-attachments"]])

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], KC_OP["JointVelocityToVelocityTwist"]))
        self.g.add((id_, KC_OP["joint"], joint))
        self.g.add((id_, KC_OP["joint-velocity"], joint_velocity))
        self.g.add((id_, KC_OP["velocity-twist"], velocity_twist))
        return id_

    def joint_acceleration_to_acceleration_twist(self, joint, joint_acceleration, acceleration_twist):
        assert joint == self.g.value(joint_acceleration, KC_STAT["of-joint"])
        assert self.g.value(acceleration_twist, GEOM_COORD["as-seen-by"]) in set(self.g[joint : KC_ENT["between-attachments"]])

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], KC_OP["JointAccelerationToAccelerationTwist"]))
        self.g.add((id_, KC_OP["joint"], joint))
        self.g.add((id_, KC_OP["joint-acceleration"], joint_acceleration))
        self.g.add((id_, KC_OP["acceleration-twist"], acceleration_twist))
        return id_

    def joint_force_from_wrench(self, joint, joint_force, wrench, number_of_wrenches):
        assert joint == self.g.value(joint_force, KC_STAT["of-joint"])
        assert self.g.value(wrench, RBDYN_COORD["as-seen-by"]) in set(self.g[joint : KC_ENT["between-attachments"]])
        assert number_of_wrenches <= self.g.value(joint_force, KC_STAT["number-of-elements"])
        assert number_of_wrenches <= self.g.value(wrench, RBDYN_COORD["number-of-wrenches"])

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], KC_OP["JointForceFromWrench"]))
        self.g.add((id_, KC_OP["joint"], joint))
        self.g.add((id_, KC_OP["number-of-wrenches"], Literal(number_of_wrenches)))
        self.g.add((id_, KC_OP["joint-force"], joint_force))
        self.g.add((id_, KC_OP["wrench"], wrench))
        return id_
