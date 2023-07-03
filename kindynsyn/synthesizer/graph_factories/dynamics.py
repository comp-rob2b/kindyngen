# SPDX-License-Identifier: MPL-2.0
from rdflib import collection, BNode, Literal, RDF
from kindynsyn.namespaces import GEOM_ENT, GEOM_REL, GEOM_COORD, RBDYN_ENT, \
    RBDYN_COORD, RBDYN_OP, QUDT_SCHEMA, QUDT_QKIND, QUDT_UNIT
from kindynsyn.rdflib_tools.helpers import uuid_ref


class DynamicsEntities:
    def __init__(self, g):
        self.g = g

    def wrench(self, acts_on, reference_point):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], RBDYN_ENT["Wrench"]))
        self.g.add((id_, RBDYN_ENT["acts-on"], acts_on))
        self.g.add((id_, RBDYN_ENT["reference-point"], reference_point))
        self.g.add((id_, QUDT_SCHEMA["quantityKind"], QUDT_QKIND["Torque"]))
        self.g.add((id_, QUDT_SCHEMA["quantityKind"], QUDT_QKIND["Force"]))
        return id_

    def rigid_body_inertia(self, of_body, about):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], RBDYN_ENT["RigidBodyInertia"]))
        self.g.add((id_, RBDYN_ENT["of-body"], of_body))
        self.g.add((id_, RBDYN_ENT["about"], about))
        self.g.add((id_, QUDT_SCHEMA["quantityKind"], QUDT_QKIND["Mass"]))
        self.g.add((id_, QUDT_SCHEMA["quantityKind"], QUDT_QKIND["LengthMass"]))
        self.g.add((id_, QUDT_SCHEMA["quantityKind"], QUDT_QKIND["MomentOfInertia"]))
        return id_


class DynamicsEntitiesCoordinates:
    def __init__(self, g):
        self.g = g

    def wrench(self, of_force, as_seen_by, number_of_wrenches):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], RBDYN_COORD["WrenchCoordinate"]))
        self.g.add((id_, RDF["type"], RBDYN_COORD["WrenchReference"]))
        self.g.add((id_, RDF["type"], RBDYN_COORD["ForceVectorXYZ"]))
        self.g.add((id_, RDF["type"], RBDYN_COORD["TorqueVectorXYZ"]))
        self.g.add((id_, RBDYN_COORD["of-wrench"], of_force))
        self.g.add((id_, RBDYN_COORD["as-seen-by"], as_seen_by))
        self.g.add((id_, QUDT_SCHEMA["unit"], QUDT_UNIT["N-M"]))
        self.g.add((id_, QUDT_SCHEMA["unit"], QUDT_UNIT["N"]))
        self.g.add((id_, RBDYN_COORD["number-of-wrenches"], Literal(number_of_wrenches)))
        return id_

    def rigid_body_inertia(self, of_inertia, as_seen_by, moment_of_inertia=None, product_of_inertia=None, moment_of_mass=None, mass=None):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], RBDYN_COORD["RigidBodyInertiaCoordinate"]))
        self.g.add((id_, RDF["type"], RBDYN_COORD["InertiaReference"]))
        self.g.add((id_, RDF["type"], RBDYN_COORD["MassScalar"]))
        self.g.add((id_, RDF["type"], RBDYN_COORD["FirstMomentOfMassVectorXYZ"]))
        self.g.add((id_, RDF["type"], RBDYN_COORD["MomentOfInertiaXYZ"]))
        self.g.add((id_, RDF["type"], RBDYN_COORD["ProductOfInertiaXYZ"]))
        self.g.add((id_, RBDYN_COORD["of-inertia"], of_inertia))
        self.g.add((id_, RBDYN_COORD["as-seen-by"], as_seen_by))
        self.g.add((id_, QUDT_SCHEMA["unit"], QUDT_UNIT["KiloGM"]))
        self.g.add((id_, QUDT_SCHEMA["unit"], QUDT_UNIT["M-KiloGM"]))
        self.g.add((id_, QUDT_SCHEMA["unit"], QUDT_UNIT["KiloGM-M2"]))

        if moment_of_inertia:
            self.g.add((id_, RBDYN_COORD["ixx"], Literal(moment_of_inertia[0])))
            self.g.add((id_, RBDYN_COORD["iyy"], Literal(moment_of_inertia[1])))
            self.g.add((id_, RBDYN_COORD["izz"], Literal(moment_of_inertia[2])))

        if product_of_inertia:
            self.g.add((id_, RBDYN_COORD["ixy"], Literal(product_of_inertia[0])))
            self.g.add((id_, RBDYN_COORD["ixz"], Literal(product_of_inertia[1])))
            self.g.add((id_, RBDYN_COORD["iyz"], Literal(product_of_inertia[2])))

        if moment_of_mass:
            mom_id = BNode()
            l = [Literal(a) for a in moment_of_mass]
            collection.Collection(self.g, mom_id, l)
            self.g.add((id_, RBDYN_COORD["first-moment-of-mass"], mom_id))

        if mass:
            self.g.add((id_, RBDYN_COORD["mass"], Literal(mass)))

        return id_


    def assign_wrench(self, frm, to):
        assert self.g.value(frm, RBDYN_COORD["as-seen-by"]) == self.g.value(to, RBDYN_COORD["as-seen-by"])
        assert self.g.value(frm, RBDYN_COORD["of-wrench"] / RBDYN_ENT["reference-point"]) == self.g.value(to, RBDYN_COORD["of-wrench"] / RBDYN_ENT["reference-point"])
        assert self.g.value(frm, RBDYN_COORD["of-wrench"] / RBDYN_ENT["acts-on"]) == self.g.value(to, RBDYN_COORD["of-wrench"] / RBDYN_ENT["acts-on"])
        assert QUDT_UNIT["N-M"] in self.g[frm : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N"] in self.g[frm : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N-M"] in self.g[to : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N"] in self.g[to : QUDT_SCHEMA["unit"]]

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], RBDYN_OP["AssignWrench"]))
        self.g.add((id_, RBDYN_OP["from"], frm))
        self.g.add((id_, RBDYN_OP["to"], to))
        return id_

    def invert_wrench(self, original, inverse, number_of_wrenches):
        assert self.g.value(original, RBDYN_COORD["as-seen-by"]) == self.g.value(inverse, RBDYN_COORD["as-seen-by"])
        assert self.g.value(original, RBDYN_COORD["of-wrench"] / RBDYN_ENT["reference-point"]) == self.g.value(inverse, RBDYN_COORD["of-wrench"] / RBDYN_ENT["reference-point"])
        assert self.g.value(original, RBDYN_COORD["of-wrench"] / RBDYN_ENT["acts-on"]) == self.g.value(inverse, RBDYN_COORD["of-wrench"] / RBDYN_ENT["acts-on"])
        assert number_of_wrenches <= self.g.value(original, RBDYN_COORD["number-of-wrenches"])
        assert number_of_wrenches <= self.g.value(inverse, RBDYN_COORD["number-of-wrenches"])
        assert QUDT_UNIT["N-M"] in self.g[original : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N"] in self.g[original : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N-M"] in self.g[inverse : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N"] in self.g[inverse : QUDT_SCHEMA["unit"]]

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], RBDYN_OP["InvertWrench"]))
        self.g.add((id_, RBDYN_OP["original"], original))
        self.g.add((id_, RBDYN_OP["inverse"], inverse))
        self.g.add((id_, RBDYN_OP["number-of-wrenches"], Literal(number_of_wrenches)))
        return id_

    def transform_wrench_to_proximal(self, pose, frm, to, number_of_wrenches, at_index):
        assert self.g.value(pose, GEOM_COORD["of-pose"] / GEOM_REL["of"]) == self.g.value(frm, RBDYN_COORD["as-seen-by"])
        assert self.g.value(pose, GEOM_COORD["of-pose"] / GEOM_REL["with-respect-to"]) == self.g.value(to, RBDYN_COORD["as-seen-by"])
        assert number_of_wrenches <= self.g.value(frm, RBDYN_COORD["number-of-wrenches"])
        assert number_of_wrenches <= self.g.value(to, RBDYN_COORD["number-of-wrenches"])
        assert number_of_wrenches + at_index <= self.g.value(to, RBDYN_COORD["number-of-wrenches"])
        assert QUDT_UNIT["UNITLESS"] in self.g[pose : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["M"] in self.g[pose : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N-M"] in self.g[frm : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N"] in self.g[frm : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N-M"] in self.g[to : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N"] in self.g[to : QUDT_SCHEMA["unit"]]

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], RBDYN_OP["TransformWrenchToProximal"]))
        self.g.add((id_, RBDYN_OP["pose"], pose))
        self.g.add((id_, RBDYN_OP["from"], frm))
        self.g.add((id_, RBDYN_OP["to"], to))
        self.g.add((id_, RBDYN_OP["number-of-wrenches"], Literal(number_of_wrenches)))
        self.g.add((id_, RBDYN_OP["at-index"], Literal(at_index)))
        return id_

    def accumulate_wrench(self, aggregate, new_element, number_of_wrenches):
        assert self.g.value(aggregate, RBDYN_COORD["as-seen-by"]) == self.g.value(new_element, RBDYN_COORD["as-seen-by"])
        assert self.g.value(aggregate, RBDYN_COORD["of-wrench"] / RBDYN_ENT["reference-point"]) == self.g.value(new_element, RBDYN_COORD["of-wrench"] / RBDYN_ENT["reference-point"])
        assert self.g.value(aggregate, RBDYN_COORD["of-wrench"] / RBDYN_ENT["acts-on"]) == self.g.value(new_element, RBDYN_COORD["of-wrench"] / RBDYN_ENT["acts-on"])
        assert QUDT_UNIT["N-M"] in self.g[aggregate : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N"] in self.g[aggregate : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N-M"] in self.g[new_element : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N"] in self.g[new_element : QUDT_SCHEMA["unit"]]

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], RBDYN_OP["AccumulateWrench"]))
        self.g.add((id_, RBDYN_OP["aggregate"], aggregate))
        self.g.add((id_, RBDYN_OP["element"], new_element))
        self.g.add((id_, RBDYN_OP["number-of-wrenches"], Literal(number_of_wrenches)))
        return id_

    def acceleration_twist_to_wrench_with_rigid_body_inertia(self, rigid_body_inertia, acceleration_twist, wrench):
        assert self.g.value(rigid_body_inertia, RBDYN_COORD["as-seen-by"]) == self.g.value(acceleration_twist, GEOM_COORD["as-seen-by"])
        assert self.g.value(rigid_body_inertia, RBDYN_COORD["as-seen-by"]) == self.g.value(wrench, RBDYN_COORD["as-seen-by"])
        assert self.g.value(rigid_body_inertia, RBDYN_COORD["of-inertia"] / RBDYN_ENT["about"]) == self.g.value(acceleration_twist, GEOM_COORD["of-acceleration"] / GEOM_REL["reference-point"])
        assert self.g.value(rigid_body_inertia, RBDYN_COORD["of-inertia"] / RBDYN_ENT["about"]) == self.g.value(wrench, RBDYN_COORD["of-wrench"] / RBDYN_ENT["reference-point"])
        assert self.g.value(rigid_body_inertia, RBDYN_COORD["of-inertia"] / RBDYN_ENT["of-body"]) == self.g.value(acceleration_twist, GEOM_COORD["of-acceleration"] / GEOM_REL["of"])
        assert self.g.value(rigid_body_inertia, RBDYN_COORD["of-inertia"] / RBDYN_ENT["of-body"]) == self.g.value(wrench, RBDYN_COORD["of-wrench"] / RBDYN_ENT["acts-on"])
        assert QUDT_UNIT["KiloGM"] in self.g[rigid_body_inertia : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["M-KiloGM"] in self.g[rigid_body_inertia : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["KiloGM-M2"] in self.g[rigid_body_inertia : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["RAD-PER-SEC2"] in self.g[acceleration_twist : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["M-PER-SEC2"] in self.g[acceleration_twist : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N-M"] in self.g[wrench : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N"] in self.g[wrench : QUDT_SCHEMA["unit"]]

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], RBDYN_OP["AccelerationTwistToWrenchWithRigidBodyInertia"]))
        self.g.add((id_, RBDYN_OP["rigid-body-inertia"], rigid_body_inertia))
        self.g.add((id_, RBDYN_OP["acceleration-twist"], acceleration_twist))
        self.g.add((id_, RBDYN_OP["wrench"], wrench))
        return id_

    def inertial_wrench(self, rigid_body_inertia, velocity_twist, wrench):
        assert self.g.value(rigid_body_inertia, RBDYN_COORD["as-seen-by"]) == self.g.value(velocity_twist, GEOM_COORD["as-seen-by"])
        assert self.g.value(rigid_body_inertia, RBDYN_COORD["as-seen-by"]) == self.g.value(wrench, RBDYN_COORD["as-seen-by"])
        assert self.g.value(rigid_body_inertia, RBDYN_COORD["of-inertia"] / RBDYN_ENT["about"]) == self.g.value(velocity_twist, GEOM_COORD["of-velocity"] / GEOM_REL["reference-point"])
        assert self.g.value(rigid_body_inertia, RBDYN_COORD["of-inertia"] / RBDYN_ENT["about"]) == self.g.value(wrench, RBDYN_COORD["of-wrench"] / RBDYN_ENT["reference-point"])
        assert self.g.value(rigid_body_inertia, RBDYN_COORD["of-inertia"] / RBDYN_ENT["of-body"]) == self.g.value(velocity_twist, GEOM_COORD["of-velocity"] / GEOM_REL["of"])
        assert self.g.value(rigid_body_inertia, RBDYN_COORD["of-inertia"] / RBDYN_ENT["of-body"]) == self.g.value(wrench, RBDYN_COORD["of-wrench"] / RBDYN_ENT["acts-on"])
        assert QUDT_UNIT["KiloGM"] in self.g[rigid_body_inertia : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["M-KiloGM"] in self.g[rigid_body_inertia : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["KiloGM-M2"] in self.g[rigid_body_inertia : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["RAD-PER-SEC"] in self.g[velocity_twist : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["M-PER-SEC"] in self.g[velocity_twist : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N-M"] in self.g[wrench : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["N"] in self.g[wrench : QUDT_SCHEMA["unit"]]

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], RBDYN_OP["InertialWrench"]))
        self.g.add((id_, RBDYN_OP["rigid-body-inertia"], rigid_body_inertia))
        self.g.add((id_, RBDYN_OP["velocity-twist"], velocity_twist))
        self.g.add((id_, RBDYN_OP["wrench"], wrench))
        return id_


class DynamicsEntitiesWithCoordinates:
    def __init__(self, ent: DynamicsEntities, coord: DynamicsEntitiesCoordinates):
        assert ent.g == coord.g

        self.ent = ent
        self.coord = coord

    def wrench(self, acts_on, as_seen_by, number_of_wrenches):
        assert GEOM_ENT["Frame"] in self.ent.g[as_seen_by : RDF["type"]]

        reference_point = self.ent.g.value(as_seen_by, GEOM_ENT["origin"])
        of_wrench = self.ent.wrench(acts_on, reference_point)
        return self.coord.wrench(of_wrench, as_seen_by, number_of_wrenches)

    def rigid_body_inertia(self, of, as_seen_by,
            moment_of_inertia=None, product_of_inertia=None, moment_of_mass=None, mass=None):
        assert GEOM_ENT["Frame"] in self.ent.g[as_seen_by : RDF["type"]]

        reference_point = self.ent.g.value(as_seen_by, GEOM_ENT["origin"])
        of_inertia = self.ent.rigid_body_inertia(of, reference_point)
        return self.coord.rigid_body_inertia(of_inertia, as_seen_by,
            moment_of_inertia, product_of_inertia, moment_of_mass, mass)
