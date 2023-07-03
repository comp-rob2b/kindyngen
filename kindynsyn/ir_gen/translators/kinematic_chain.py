# SPDX-License-Identifier: MPL-2.0
from rdflib import RDF
from kindynsyn.rdflib_tools import qname
from kindynsyn.namespaces import GEOM_ENT, GEOM_REL, KC_ENT, KC_STAT, KC_OP, \
    QUDT_SCHEMA
from kindynsyn.ir_gen.translators import for_type, escape, embed_data


def joint_axis(g, node) -> str | None:
    vx = g[node : KC_OP["joint"] / KC_ENT["common-axis"] / GEOM_REL["lines"] / ~GEOM_ENT["vector-x"]]
    vy = g[node : KC_OP["joint"] / KC_ENT["common-axis"] / GEOM_REL["lines"] / ~GEOM_ENT["vector-y"]]
    vz = g[node : KC_OP["joint"] / KC_ENT["common-axis"] / GEOM_REL["lines"] / ~GEOM_ENT["vector-z"]]

    if len(list(vx)) == 2:
        return "x"
    if len(list(vy)) == 2:
        return "y"
    if len(list(vz)) == 2:
        return "z"

    print("No joint axis found")
    return None



@for_type(KC_STAT["JointPositionCoordinate"])
class JointPositionTranslator:
    @staticmethod
    def is_applicable(g, node):
        is_rev_jnt = g[node : KC_STAT["of-joint"] / RDF["type"] : KC_ENT["RevoluteJoint"]]
        return is_rev_jnt

    @staticmethod
    def translate(g, node):
        coord = embed_data(g, g.value(node, QUDT_SCHEMA["value"]))

        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "quantity": "joint-position",
            "size": 1,
            "coordinates": coord
        }

@for_type(KC_STAT["JointVelocityCoordinate"])
class JointVelocityTranslator:
    @staticmethod
    def is_applicable(g, node) -> bool:
        is_rev_jnt = g[node : KC_STAT["of-joint"] / RDF["type"] : KC_ENT["RevoluteJoint"]]
        return is_rev_jnt

    @staticmethod
    def translate(g, node):
        coord = embed_data(g, g.value(node, QUDT_SCHEMA["value"]))

        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "quantity": "joint-velocity",
            "size": 1,
            "coordinates": coord
        }

@for_type(KC_STAT["JointAccelerationCoordinate"])
class JointAccelerationTranslator:
    @staticmethod
    def is_applicable(g, node):
        is_rev_jnt = g[node : KC_STAT["of-joint"] / RDF["type"] : KC_ENT["RevoluteJoint"]]
        return is_rev_jnt

    @staticmethod
    def translate(g, node):
        coord = embed_data(g, g.value(node, QUDT_SCHEMA["value"]))

        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "quantity": "joint-acceleration",
            "size": 1,
            "coordinates": coord
        }

@for_type(KC_STAT["JointForceCoordinate"])
class JointForceTranslator:
    @staticmethod
    def is_applicable(g, node):
        is_rev_jnt = g[node : KC_STAT["of-joint"] / RDF["type"] : KC_ENT["RevoluteJoint"]]
        return is_rev_jnt

    @staticmethod
    def translate(g, node):
        coord = embed_data(g, g.value(node, QUDT_SCHEMA["value"]))

        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "quantity": "joint-force",
            "size": int(g.value(node, KC_STAT["number-of-elements"])),
            "coordinates": coord
        }

@for_type(KC_STAT["JointInertiaCoordinate"])
class JointInertiaTranslator:
    @staticmethod
    def is_applicable(g, node):
        is_rev_jnt = g[node : KC_STAT["of-joint"] / RDF["type"] : KC_ENT["RevoluteJoint"]]
        return is_rev_jnt

    @staticmethod
    def translate(g, node):
        coord = embed_data(g, g.value(node, QUDT_SCHEMA["value"]))

        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "quantity": "joint-inertia",
            "size": 1,
            "coordinates": coord
        }



@for_type(KC_OP["JointPositionToPose"])
class JointPositionToPoseTranslator:
    @staticmethod
    def is_applicable(g, node):
        is_rev_jnt = g[node : KC_OP["joint"] / RDF["type"] : KC_ENT["RevoluteJoint"]]
        return is_rev_jnt

    @staticmethod
    def translate(g, node):
        axis = joint_axis(g, node)

        return {
            "represents": str(node),
            "operator": "joint-position-to-pose",
            "dimensions": 3,
            "joint": "rev_" + axis,
            "joint-position": escape(qname(g, g.value(node, KC_OP["joint-position"]))),
            "pose": escape(qname(g, g.value(node, KC_OP["pose"])))
        }

@for_type(KC_OP["JointVelocityToVelocityTwist"])
class JointVelocityToVelocityTwistTranslator:
    @staticmethod
    def is_applicable(g, node):
        is_rev_jnt = g[node : KC_OP["joint"] / RDF["type"] : KC_ENT["RevoluteJoint"]]
        return is_rev_jnt

    @staticmethod
    def translate(g, node):
        axis = joint_axis(g, node)

        return {
            "represents": str(node),
            "operator": "joint-velocity-to-velocity-twist",
            "dimensions": 3,
            "joint": "rev_" + axis,
            "joint-velocity": escape(qname(g, g.value(node, KC_OP["joint-velocity"]))),
            "velocity-twist": escape(qname(g, g.value(node, KC_OP["velocity-twist"])))
        }

@for_type(KC_OP["JointAccelerationToAccelerationTwist"])
class JointAccelerationToAccelerationTwistTranslator:
    @staticmethod
    def is_applicable(g, node):
        is_rev_jnt = g[node : KC_OP["joint"] / RDF["type"] : KC_ENT["RevoluteJoint"]]
        return is_rev_jnt

    @staticmethod
    def translate(g, node):
        axis = joint_axis(g, node)

        return {
            "represents": str(node),
            "operator": "joint-acceleration-to-acceleration-twist",
            "dimensions": 3,
            "joint": "rev_" + axis,
            "joint-acceleration": escape(qname(g, g.value(node, KC_OP["joint-acceleration"]))),
            "acceleration-twist": escape(qname(g, g.value(node, KC_OP["acceleration-twist"])))
        }

@for_type(KC_OP["JointForceFromWrench"])
class JointForceFromWrenchTranslator:
    @staticmethod
    def is_applicable(g, node):
        is_rev_jnt = g[node : KC_OP["joint"] / RDF["type"] : KC_ENT["RevoluteJoint"]]
        return is_rev_jnt

    @staticmethod
    def translate(g, node):
        axis = joint_axis(g, node)

        return {
            "represents": str(node),
            "operator": "joint-force-from-wrench",
            "dimensions": 3,
            "joint": "rev_" + axis,
            "number-of-wrenches": int(g.value(node, KC_OP["number-of-wrenches"])),
            "joint-force": escape(qname(g, g.value(node, KC_OP["joint-force"]))),
            "wrench": escape(qname(g, g.value(node, KC_OP["wrench"])))
        }
