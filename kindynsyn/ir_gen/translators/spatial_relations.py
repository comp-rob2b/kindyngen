# SPDX-License-Identifier: MPL-2.0
from kindynsyn.rdflib_tools import qname
from kindynsyn.namespaces import GEOM_COORD, GEOM_OP
from kindynsyn.ir_gen.translators import for_type, escape, parse_vector3


def parse_direction_cosine_xyz(g, node):
    cos_x = parse_vector3(g, g.value(node, GEOM_COORD["direction-cosine-x"]))
    cos_y = parse_vector3(g, g.value(node, GEOM_COORD["direction-cosine-y"]))
    cos_z = parse_vector3(g, g.value(node, GEOM_COORD["direction-cosine-z"]))

    if cos_x is None or cos_y is None or cos_z is None:
        return None

    mat_col_major = []
    for col in [cos_x, cos_y, cos_z]:
        mat_col_major.extend(col)

    return mat_col_major


def parse_xyz(g, node):
    x = g.value(node, GEOM_COORD["x"])
    y = g.value(node, GEOM_COORD["y"])
    z = g.value(node, GEOM_COORD["z"])

    if x is None or y is None or z is None:
        return None

    return [x.value, y.value, z.value]


@for_type(GEOM_COORD["PoseCoordinate"], GEOM_COORD["DirectionCosineXYZ"], GEOM_COORD["VectorXYZ"])
class PoseTranslator:
    @staticmethod
    def translate(g, node):
        dc_x = parse_vector3(g, g.value(node, GEOM_COORD["direction-cosine-x"]))
        dc_y = parse_vector3(g, g.value(node, GEOM_COORD["direction-cosine-y"]))
        dc_z = parse_vector3(g, g.value(node, GEOM_COORD["direction-cosine-z"]))
        pos = parse_xyz(g, node)

        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "quantity": "pose",
            "dimensions": 3,
            "direction-cosine-x": dc_x,
            "direction-cosine-y": dc_y,
            "direction-cosine-z": dc_z,
            "position": pos
        }

@for_type(GEOM_COORD["VelocityTwistCoordinate"], GEOM_COORD["AngularVelocityVectorXYZ"], GEOM_COORD["LinearVelocityVectorXYZ"])
class VelocityTwistTranslator:
    @staticmethod
    def translate(g, node):
        vel_ang = parse_vector3(g, g.value(node, GEOM_COORD["angular-velocity"]))
        vel_lin = parse_vector3(g, g.value(node, GEOM_COORD["linear-velocity"]))
        vel_ang = [vel_ang] if vel_ang else None
        vel_lin = [vel_lin] if vel_lin else None

        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "quantity": "velocity-twist",
            "dimensions": 3,
            "number-of-velocities": 1,
            "angular-velocity": vel_ang,
            "linear-velocity": vel_lin
        }

@for_type(GEOM_COORD["AccelerationTwistCoordinate"], GEOM_COORD["AngularAccelerationVectorXYZ"], GEOM_COORD["LinearAccelerationVectorXYZ"])
class AccelerationTwistTranslator:
    @staticmethod
    def translate(g, node):
        acc_ang = parse_vector3(g, g.value(node, GEOM_COORD["angular-acceleration"]))
        acc_lin = parse_vector3(g, g.value(node, GEOM_COORD["linear-acceleration"]))
        acc_ang = [acc_ang] if acc_ang else None
        acc_lin = [acc_lin] if acc_lin else None

        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "quantity": "acceleration-twist",
            "dimensions": 3,
            "number-of-accelerations": 1,
            "angular-acceleration": acc_ang,
            "linear-acceleration": acc_lin
        }



@for_type(GEOM_OP["TransformVelocityTwistToDistal"])
class TransformVelocityTwistToDistalTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "operator": "transform-velocity-twist-to-distal",
            "dimensions": 3,
            "number-of-velocities": 1,
            "pose": escape(qname(g, g.value(node, GEOM_OP["pose"]))),
            "from": escape(qname(g, g.value(node, GEOM_OP["from"]))),
            "to": escape(qname(g, g.value(node, GEOM_OP["to"])))
        }

@for_type(GEOM_OP["RotateVelocityTwistToProximalWithPose"])
class RotateVelocityTwistToProximalWithPoseTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "operator": "rotate-velocity-twist-to-proximal-with-pose",
            "dimensions": 3,
            "number-of-velocities": 1,
            "pose": escape(qname(g, g.value(node, GEOM_OP["pose"]))),
            "from": escape(qname(g, g.value(node, GEOM_OP["from"]))),
            "to": escape(qname(g, g.value(node, GEOM_OP["to"])))
        }

@for_type(GEOM_OP["TransformAccelerationTwistToDistal"])
class TransformAccelerationTwistToDistalTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "operator": "transform-acceleration-twist-to-distal",
            "dimensions": 3,
            "pose": escape(qname(g, g.value(node, GEOM_OP["pose"]))),
            "absolute-velocity": escape(qname(g, g.value(node, GEOM_OP["absolute-velocity"]))),
            "relative-velocity": escape(qname(g, g.value(node, GEOM_OP["relative-velocity"]))),
            "from": escape(qname(g, g.value(node, GEOM_OP["from"]))),
            "to": escape(qname(g, g.value(node, GEOM_OP["to"])))
        }

@for_type(GEOM_OP["ComposePose"])
class ComposePoseTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "operator": "compose-pose",
            "dimensions": 3,
            "in1": escape(qname(g, g.value(node, GEOM_OP["in1"]))),
            "in2": escape(qname(g, g.value(node, GEOM_OP["in2"]))),
            "out": escape(qname(g, g.value(node, GEOM_OP["composite"])))
        }

@for_type(GEOM_OP["AddVelocityTwist"])
class AddVelocityTwistTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "operator": "add-velocity-twist",
            "dimensions": 3,
            "in1": escape(qname(g, g.value(node, GEOM_OP["in1"]))),
            "in2": escape(qname(g, g.value(node, GEOM_OP["in2"]))),
            "out": escape(qname(g, g.value(node, GEOM_OP["composite"])))
        }

@for_type(GEOM_OP["AddAccelerationTwist"])
class AddAccelerationTwistTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "operator": "add-acceleration-twist",
            "dimensions": 3,
            "in1": escape(qname(g, g.value(node, GEOM_OP["in1"]))),
            "in2": escape(qname(g, g.value(node, GEOM_OP["in2"]))),
            "out": escape(qname(g, g.value(node, GEOM_OP["composite"])))
        }
