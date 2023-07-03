# SPDX-License-Identifier: MPL-2.0
from kindynsyn.rdflib_tools import qname
from kindynsyn.namespaces import RBDYN_COORD, RBDYN_OP
from kindynsyn.ir_gen.translators import for_type, escape, parse_scalar, \
    parse_vector3


@for_type(RBDYN_COORD["RigidBodyInertiaCoordinate"], RBDYN_COORD["MassScalar"], RBDYN_COORD["FirstMomentOfMassVectorXYZ"], RBDYN_COORD["MomentOfInertiaXYZ"], RBDYN_COORD["ProductOfInertiaXYZ"])
class RigidBodyInertiaTranslator:
    @staticmethod
    def translate(g, node):
        m0 = parse_scalar(g.value(node, RBDYN_COORD["mass"]))
        m1 = parse_vector3(g, g.value(node, RBDYN_COORD["first-moment-of-mass"]))
        m2 = []
        var = [ "ixx", "ixy", "ixz", "ixy", "iyy", "iyz", "ixz", "iyz", "izz" ]
        for i in var:
            m = parse_scalar(g.value(node, RBDYN_COORD[i]))
            if m is None:
                m2 = None
                break
            m2.append(m)

        if m0 is not None and m1 is not None and m2 is not None:
            init = m2.copy()
            init.extend(m1)
            init.append(m0)
        else:
            init = None

        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "quantity": "rigid-body-inertia",
            "dimensions": 3,
            "rotational-inertia": m2,
            "moment-of-mass": m1,
            "mass": m0
        }

@for_type(RBDYN_COORD["WrenchCoordinate"], RBDYN_COORD["ForceVectorXYZ"], RBDYN_COORD["TorqueVectorXYZ"])
class WrenchTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "name": escape(qname(g, node)),
            "quantity": "wrench",
            "dimensions": 3,
            "number-of-wrenches": int(g.value(node, RBDYN_COORD["number-of-wrenches"])),
            "torque": None,
            "force": None
        }



@for_type(RBDYN_OP["InvertWrench"])
class InvertWrenchTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "operator": "invert-wrench",
            "dimensions": 3,
            "number-of-wrenches": int(g.value(node, RBDYN_OP["number-of-wrenches"])),
            "original": escape(qname(g, g.value(node, RBDYN_OP["original"]))),
            "inverse": escape(qname(g, g.value(node, RBDYN_OP["inverse"])))
        }

@for_type(RBDYN_OP["TransformWrenchToProximal"])
class TransformWrenchToProximalTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "operator": "transform-wrench-to-proximal",
            "dimensions": 3,
            "number-of-wrenches": int(g.value(node, RBDYN_OP["number-of-wrenches"])),
            "at-index": int(g.value(node, RBDYN_OP["at-index"])),
            "pose": escape(qname(g, g.value(node, RBDYN_OP["pose"]))),
            "from": escape(qname(g, g.value(node, RBDYN_OP["from"]))),
            "to": escape(qname(g, g.value(node, RBDYN_OP["to"])))
        }

@for_type(RBDYN_OP["InertialWrench"])
class InertialWrenchTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "operator": "inertial-wrench",
            "dimensions": 3,
            "number-of-wrenches": 1,
            "rigid-body-inertia": escape(qname(g, g.value(node, RBDYN_OP["rigid-body-inertia"]))),
            "velocity-twist": escape(qname(g, g.value(node, RBDYN_OP["velocity-twist"]))),
            "wrench": escape(qname(g, g.value(node, RBDYN_OP["wrench"])))
        }

@for_type(RBDYN_OP["AccelerationTwistToWrenchWithRigidBodyInertia"])
class AccelerationTwistToWrenchWithRigidBodyInertiaTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "operator": "acceleration-twist-to-wrench-with-rigid-body-inertia",
            "dimensions": 3,
            "rigid-body-inertia": escape(qname(g, g.value(node, RBDYN_OP["rigid-body-inertia"]))),
            "acceleration-twist": escape(qname(g, g.value(node, RBDYN_OP["acceleration-twist"]))),
            "wrench": escape(qname(g, g.value(node, RBDYN_OP["wrench"])))
        }

@for_type(RBDYN_OP["AssignWrench"])
class AssignWrenchTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "operator": "assign-wrench",
            "dimensions": 3,
            "number-of-wrenches": 1,
            "from": escape(qname(g, g.value(node, RBDYN_OP["from"]))),
            "to": escape(qname(g, g.value(node, RBDYN_OP["to"])))
        }

@for_type(RBDYN_OP["AccumulateWrench"])
class AccumulateWrenchTranslator:
    @staticmethod
    def translate(g, node):
        return {
            "represents": str(node),
            "operator": "accumulate-wrench",
            "dimensions": 3,
            "number-of-wrenches": int(g.value(node, RBDYN_OP["number-of-wrenches"])),
            "element": escape(qname(g, g.value(node, RBDYN_OP["element"]))),
            "aggregate": escape(qname(g, g.value(node, RBDYN_OP["aggregate"])))
        }
