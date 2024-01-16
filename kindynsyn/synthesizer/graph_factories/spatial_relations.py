# SPDX-License-Identifier: MPL-2.0
from rdflib import collection, BNode, Literal, RDF
from kindynsyn.namespaces import GEOM_ENT, GEOM_REL, GEOM_COORD, GEOM_OP, \
    QUDT_SCHEMA, QUDT_QKIND, QUDT_UNIT
from kindynsyn.rdflib_tools.helpers import uuid_ref


class SpatialRelations:
    def __init__(self, g):
        self.g = g

    def pose(self, of, with_respect_to):
        assert GEOM_ENT["Frame"] in self.g[of : RDF["type"]]
        assert GEOM_ENT["Frame"] in self.g[with_respect_to : RDF["type"]]

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], GEOM_REL["Pose"]))
        self.g.add((id_, GEOM_REL["of"], of))
        self.g.add((id_, GEOM_REL["with-respect-to"], with_respect_to))
        self.g.add((id_, QUDT_SCHEMA["quantityKind"], QUDT_QKIND["Angle"]))
        self.g.add((id_, QUDT_SCHEMA["quantityKind"], QUDT_QKIND["Length"]))
        return id_

    def velocity(self, of, with_respect_to, reference_point):
        assert GEOM_ENT["RigidBody"] in self.g[of : RDF["type"]]
        assert GEOM_ENT["RigidBody"] in self.g[with_respect_to : RDF["type"]]
        assert GEOM_ENT["Point"] in self.g[reference_point : RDF["type"]]

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], GEOM_REL["VelocityTwist"]))
        self.g.add((id_, GEOM_REL["of"], of))
        self.g.add((id_, GEOM_REL["with-respect-to"], with_respect_to))
        self.g.add((id_, GEOM_REL["reference-point"], reference_point))
        self.g.add((id_, QUDT_SCHEMA["quantityKind"], QUDT_QKIND["AngularVelocity"]))
        self.g.add((id_, QUDT_SCHEMA["quantityKind"], QUDT_QKIND["LinearVelocity"]))
        return id_

    def acceleration(self, of, with_respect_to, reference_point):
        assert GEOM_ENT["RigidBody"] in self.g[of : RDF["type"]]
        assert GEOM_ENT["RigidBody"] in self.g[with_respect_to : RDF["type"]]
        assert GEOM_ENT["Point"] in self.g[reference_point : RDF["type"]]

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], GEOM_REL["AccelerationTwist"]))
        self.g.add((id_, GEOM_REL["of"], of))
        self.g.add((id_, GEOM_REL["with-respect-to"], with_respect_to))
        self.g.add((id_, GEOM_REL["reference-point"], reference_point))
        self.g.add((id_, QUDT_SCHEMA["quantityKind"], QUDT_QKIND["AngularAcceleration"]))
        self.g.add((id_, QUDT_SCHEMA["quantityKind"], QUDT_QKIND["LinearAcceleration"]))
        return id_


class SpatialRelationsCoordinates:
    def __init__(self, g):
        self.g = g

    def pose(self, of_pose, as_seen_by, orientation=None, position=None):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], GEOM_COORD["PoseCoordinate"]))
        self.g.add((id_, RDF["type"], GEOM_COORD["PoseReference"]))
        self.g.add((id_, RDF["type"], GEOM_COORD["DirectionCosineXYZ"]))
        self.g.add((id_, RDF["type"], GEOM_COORD["VectorXYZ"]))
        self.g.add((id_, GEOM_COORD["of-pose"], of_pose))
        self.g.add((id_, GEOM_COORD["as-seen-by"], as_seen_by))
        self.g.add((id_, QUDT_SCHEMA["unit"], QUDT_UNIT["UNITLESS"]))
        self.g.add((id_, QUDT_SCHEMA["unit"], QUDT_UNIT["M"]))

        if orientation:
            for name, idx in [("direction-cosine-x", 0), ("direction-cosine-y", 1), ("direction-cosine-z", 2)]:
                dc = BNode()
                l = [Literal(a) for a in orientation[idx]]
                collection.Collection(self.g, dc, l)
                self.g.add((id_, GEOM_COORD[name], dc))

        if position:
            self.g.add((id_, GEOM_COORD["x"], Literal(position[0])))
            self.g.add((id_, GEOM_COORD["y"], Literal(position[1])))
            self.g.add((id_, GEOM_COORD["z"], Literal(position[2])))

        return id_

    def velocity_twist(self, of_velocity, as_seen_by, angular_velocity=None, linear_velocity=None):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], GEOM_COORD["VelocityTwistCoordinate"]))
        self.g.add((id_, RDF["type"], GEOM_COORD["VelocityReference"]))
        self.g.add((id_, RDF["type"], GEOM_COORD["AngularVelocityVectorXYZ"]))
        self.g.add((id_, RDF["type"], GEOM_COORD["LinearVelocityVectorXYZ"]))
        self.g.add((id_, GEOM_COORD["of-velocity"], of_velocity))
        self.g.add((id_, GEOM_COORD["as-seen-by"], as_seen_by))
        self.g.add((id_, QUDT_SCHEMA["unit"], QUDT_UNIT["RAD-PER-SEC"]))
        self.g.add((id_, QUDT_SCHEMA["unit"], QUDT_UNIT["M-PER-SEC"]))

        if angular_velocity:
            ang_acc_id = BNode()
            l = [Literal(a) for a in angular_velocity]
            collection.Collection(self.g, ang_acc_id, l)
            self.g.add((id_, GEOM_COORD["angular-velocity"], ang_acc_id))

        if linear_velocity:
            lin_acc_id = BNode()
            l = [Literal(a) for a in linear_velocity]
            collection.Collection(self.g, lin_acc_id, l)
            self.g.add((id_, GEOM_COORD["linear-velocity"], lin_acc_id))

        return id_

    def acceleration_twist(self, of_acceleration, as_seen_by, angular_acceleration=None, linear_acceleration=None):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], GEOM_COORD["AccelerationTwistCoordinate"]))
        self.g.add((id_, RDF["type"], GEOM_COORD["AccelerationReference"]))
        self.g.add((id_, RDF["type"], GEOM_COORD["AngularAccelerationVectorXYZ"]))
        self.g.add((id_, RDF["type"], GEOM_COORD["LinearAccelerationVectorXYZ"]))
        self.g.add((id_, GEOM_COORD["of-acceleration"], of_acceleration))
        self.g.add((id_, GEOM_COORD["as-seen-by"], as_seen_by))
        self.g.add((id_, QUDT_SCHEMA["unit"], QUDT_UNIT["RAD-PER-SEC2"]))
        self.g.add((id_, QUDT_SCHEMA["unit"], QUDT_UNIT["M-PER-SEC2"]))

        if angular_acceleration:
            ang_acc_id = BNode()
            l = [Literal(a) for a in angular_acceleration]
            collection.Collection(self.g, ang_acc_id, l)
            self.g.add((id_, GEOM_COORD["angular-acceleration"], ang_acc_id))

        if linear_acceleration:
            lin_acc_id = BNode()
            l = [Literal(a) for a in linear_acceleration]
            collection.Collection(self.g, lin_acc_id, l)
            self.g.add((id_, GEOM_COORD["linear-acceleration"], lin_acc_id))

        return id_


    def compose_pose(self, in1, in2, composite):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], GEOM_OP["ComposePose"]))
        self.g.add((id_, GEOM_OP["in1"], in1))
        self.g.add((id_, GEOM_OP["in2"], in2))
        self.g.add((id_, GEOM_OP["composite"], composite))
        return id_

    def add_velocity_twist(self, in1, in2, composite):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], GEOM_OP["AddVelocityTwist"]))
        self.g.add((id_, GEOM_OP["in1"], in1))
        self.g.add((id_, GEOM_OP["in2"], in2))
        self.g.add((id_, GEOM_OP["composite"], composite))
        return id_

    def add_acceleration_twist(self, in1, in2, composite):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], GEOM_OP["AddAccelerationTwist"]))
        self.g.add((id_, GEOM_OP["in1"], in1))
        self.g.add((id_, GEOM_OP["in2"], in2))
        self.g.add((id_, GEOM_OP["composite"], composite))
        return id_

    def transform_velocity_twist_to_distal(self, pose, frm, to):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], GEOM_OP["TransformVelocityTwistToDistal"]))
        self.g.add((id_, GEOM_OP["pose"], pose))
        self.g.add((id_, GEOM_OP["from"], frm))
        self.g.add((id_, GEOM_OP["to"], to))
        return id_

    def rotate_velocity_twist_to_proximal_with_pose(self, pose, frm, to):
        assert self.g.value(pose, GEOM_COORD["of-pose"] / GEOM_REL["with-respect-to"]) == self.g.value(to, GEOM_COORD["as-seen-by"])
        assert self.g.value(pose, GEOM_COORD["of-pose"] / GEOM_REL["of"]) == self.g.value(frm, GEOM_COORD["as-seen-by"])
        assert QUDT_UNIT["UNITLESS"] in self.g[pose : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["M"] in self.g[pose : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["RAD-PER-SEC"] in self.g[frm : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["M-PER-SEC"] in self.g[frm : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["RAD-PER-SEC"] in self.g[to : QUDT_SCHEMA["unit"]]
        assert QUDT_UNIT["M-PER-SEC"] in self.g[to : QUDT_SCHEMA["unit"]]

        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], GEOM_OP["RotateVelocityTwistToProximalWithPose"]))
        self.g.add((id_, GEOM_OP["pose"], pose))
        self.g.add((id_, GEOM_OP["from"], frm))
        self.g.add((id_, GEOM_OP["to"], to))
        return id_

    def transform_acceleration_twist_to_distal(self, pose, absolute_velocity, relative_velocity, frm, to):
        id_ = uuid_ref()
        self.g.add((id_, RDF["type"], GEOM_OP["TransformAccelerationTwistToDistal"]))
        self.g.add((id_, GEOM_OP["pose"], pose))
        self.g.add((id_, GEOM_OP["absolute-velocity"], absolute_velocity))
        self.g.add((id_, GEOM_OP["relative-velocity"], relative_velocity))
        self.g.add((id_, GEOM_OP["from"], frm))
        self.g.add((id_, GEOM_OP["to"], to))
        return id_


class SpatialRelationsWithCoordinates:
    def __init__(self, rel: SpatialRelations, coord: SpatialRelationsCoordinates):
        assert rel.g == coord.g

        self.rel = rel
        self.coord = coord

    def pose(self, of, with_respect_to, orientation=None, position=None):
        of_pose = self.rel.pose(of, with_respect_to)
        return self.coord.pose(of_pose, with_respect_to, orientation, position)

    def velocity_twist(self, of, with_respect_to, as_seen_by, reference_point=None, angular_velocity=None, linear_velocity=None):
        assert GEOM_ENT["Frame"] in self.rel.g[as_seen_by : RDF["type"]]

        if not reference_point:
            reference_point = self.rel.g.value(as_seen_by, GEOM_ENT["origin"])
        of_velocity = self.rel.velocity(of, with_respect_to, reference_point)
        return self.coord.velocity_twist(of_velocity, as_seen_by, angular_velocity, linear_velocity)

    def acceleration_twist(self, of, with_respect_to, as_seen_by, angular_acceleration=None, linear_acceleration=None):
        assert GEOM_ENT["Frame"] in self.rel.g[as_seen_by : RDF["type"]]

        reference_point = self.rel.g.value(as_seen_by, GEOM_ENT["origin"])
        of_acceleration = self.rel.acceleration(of, with_respect_to, reference_point)
        return self.coord.acceleration_twist(of_acceleration, as_seen_by, angular_acceleration, linear_acceleration)
