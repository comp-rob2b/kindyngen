# SPDX-License-Identifier: MPL-2.0
from rdflib.namespace import DefinedNamespace, Namespace
from rdflib.term import URIRef


class GEOM_ENT(DefinedNamespace):
    Euclidean: URIRef
    Point: URIRef
    Vector: URIRef
    BoundVector: URIRef
    UnitLength: URIRef
    Frame: URIRef
    Orthonormal: URIRef
    RigidBody: URIRef
    RightHanded: URIRef
    SimplicialComplex: URIRef

    origin: URIRef
    simplices: URIRef

    _extras = [
        "vector-x",
        "vector-y",
        "vector-z"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/geometry/structural-entities#")


class GEOM_REL(DefinedNamespace):
    Pose: URIRef
    VelocityTwist: URIRef
    AccelerationTwist: URIRef

    of: URIRef                      # Spatial relation _of_ a body with respect to another
    lines: URIRef                   # The _lines_ that appear in LineCollinearity

    _extras = [
        "with-respect-to",          # Spatial relation of a body _with respect to_ another
        "reference-point"           # _Reference point_ of linear velocity/velocity twist/linear acceleration/acceleration twist
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/geometry/spatial-relations#")


class GEOM_COORD(DefinedNamespace):
    PoseCoordinate: URIRef
    PoseReference: URIRef
    VelocityTwistCoordinate: URIRef
    VelocityReference: URIRef
    AccelerationTwistCoordinate: URIRef
    AccelerationReference: URIRef
    DirectionCosineXYZ: URIRef
    VectorXYZ: URIRef
    AngularVelocityVectorXYZ: URIRef
    LinearVelocityVectorXYZ: URIRef
    AngularAccelerationVectorXYZ: URIRef
    LinearAccelerationVectorXYZ: URIRef

    x: URIRef
    y: URIRef
    z: URIRef
    coord: URIRef

    _extras = [
        "of-pose",
        "of-velocity",
        "of-acceleration",
        "as-seen-by",
        "direction-cosine-x",
        "direction-cosine-y",
        "direction-cosine-z",
        "angular-velocity",
        "linear-velocity",
        "angular-acceleration",
        "linear-acceleration"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/geometry/coordinates#")


class GEOM_OP(DefinedNamespace):
    AddVelocityTwist: URIRef
    AddAccelerationTwist: URIRef
    ComposePose: URIRef
    TransformVelocityTwistToDistal: URIRef
    RotateVelocityTwistToProximalWithPose: URIRef
    TransformAccelerationTwistToDistal: URIRef

    in1: URIRef
    in2: URIRef
    composite: URIRef
    pose: URIRef
    to: URIRef                  # Transform a vector from one space _to_ another

    _extras = [
        "from",                 # Transform a vector _from_ one space to another
        "absolute-velocity",    # The _absolute velocity_ twist required for transforming an acceleration twist (the v_1 in "v_1 x v_2")
        "relative-velocity"     # The _relative velocity_ twist required for transforming an acceleration twist (the v_2 in "v_1 x v_2")
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/geometry/spatial-operators#")


class RBDYN_ENT(DefinedNamespace):
    Wrench: URIRef
    RigidBodyInertia: URIRef

    about: URIRef               # The point _about_ which rotational inertia/rigid-body inertia is measured

    _extras = [
        "acts-on",              # Force/torque/force wrench _acts on_ a body
        "of-body",              # Inertia _of_ a body
        "reference-point"       # _Reference point_ of angular torque/force wrench/inertia
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/newtonian-rigid-body-dynamics/structural-entities#")


class RBDYN_COORD(DefinedNamespace):
    WrenchCoordinate: URIRef
    WrenchReference: URIRef
    InertiaReference: URIRef
    RigidBodyInertiaCoordinate: URIRef
    ForceVectorXYZ: URIRef
    TorqueVectorXYZ: URIRef
    MassScalar: URIRef
    MomentOfInertiaXYZ: URIRef
    ProductOfInertiaXYZ: URIRef
    FirstMomentOfMassVectorXYZ: URIRef

    mass: URIRef
    ixx: URIRef
    ixy: URIRef
    ixz: URIRef
    iyy: URIRef
    iyz: URIRef
    izz: URIRef

    _extras = [
        "of-wrench",
        "of-inertia",
        "as-seen-by",
        "first-moment-of-mass",
        "number-of-wrenches"            # The number of wrench instances in a wrench coordinate vector
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/newtonian-rigid-body-dynamics/coordinates#")


class RBDYN_OP(DefinedNamespace):
    AssignWrench: URIRef                # Assign/copy from one wrench to another
    InvertWrench: URIRef
    AccelerationTwistToWrenchWithRigidBodyInertia: URIRef
    AccumulateWrench: URIRef            # _Accumulate_ wrench
    InertialWrench: URIRef
    TransformWrenchToProximal: URIRef
    RotateWrenchToDistalWithPose: URIRef

    original: URIRef                    # The _original_ wrench to be inverted
    inverse: URIRef                     # The _inverse_ of the original wrench
    aggregate: URIRef                   # The _aggregate_'d value in the accumulate operator
    element: URIRef                     # The new _element_ to be added to the aggregate in the accumulate operator
    wrench: URIRef
    pose: URIRef                        # The _pose_ required for transforming any rigid-body entity
    to: URIRef                          # The "destination" wrench _to_ which to transform

    _extras = [
        "from",                         # The "source" wrench _from_ which to transform
        "rigid-body-inertia",           # The _rigid-body inertia_ to map a twist to a momentum in the computation of inertial force
        "velocity-twist",
        "acceleration-twist",
        "number-of-wrenches",           # The number of wrench instances to process
        "at-index"                      # The _index at_ which the transformed wrench should be stored
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/newtonian-rigid-body-dynamics/operators#")


class KC_ENT(DefinedNamespace):
    _extras = [
        "between-attachments",
        "common-axis"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/kinematic-chain/structural-entities#")


class KC_STAT(DefinedNamespace):
    JointInertia: URIRef
    JointForce: URIRef
    JointPositionCoordinate: URIRef
    JointVelocityCoordinate: URIRef
    JointAccelerationCoordinate: URIRef
    JointForceCoordinate: URIRef
    JointInertiaCoordinate: URIRef

    _extras = [
        "of-joint",
        "as-seen-by",
        "number-of-elements"            # The number of elements/instances in a joint-space quantity
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/kinematic-chain/state#")


class KC_OP(DefinedNamespace):
    JointPositionToPose: URIRef
    JointVelocityToVelocityTwist: URIRef
    JointAccelerationToAccelerationTwist: URIRef
    JointForceFromWrench: URIRef

    joint: URIRef
    pose: URIRef
    wrench: URIRef
    aggregate: URIRef                   # The _aggregate_'d value in the accumulate operator
    element: URIRef                     # The new _element_ to be added to the aggregate in the accumulate operator
    to: URIRef                          # The "destination" wrench _to_ which to transform

    _extras = [
        "joint-position",
        "joint-velocity",
        "joint-acceleration",
        "joint-force",
        "velocity-twist",
        "acceleration-twist",
        "number-of-wrenches",           # The number of wrench instances to process
        "from"                          # The "source" wrench _from_ which to transform
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/kinematic-chain/operators#")


class SPEC(DefinedNamespace):
    ExternalForce: URIRef

    force: URIRef

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/kinematic-chain/solver-specification#")


class ALGO(DefinedNamespace):
    Schedule: URIRef            # Scheduler block
    Algorithm: URIRef           # Algorithm block

    data: URIRef                # The data blocks contained in an algorithm block
    schedule: URIRef            # The scheduler blocks contained in an algorithm block
    function: URIRef            # The function blocks contained in an algorithm block
    algorithm: URIRef           # The algorithm blocks contained in another algorithm block

    _extras = [
        "trigger-chain"         # Sequence of function blocks to be triggered from a scheduler block
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/algorithm#")


class QUDT_SCHEMA(DefinedNamespace):
    quantityKind: URIRef
    unit: URIRef
    value: URIRef

    _NS = Namespace("http://qudt.org/schema/qudt/")


class QUDT_QKIND(DefinedNamespace):
    Angle: URIRef
    Length: URIRef
    AngularVelocity: URIRef
    LinearVelocity: URIRef
    AngularAcceleration: URIRef
    LinearAcceleration: URIRef
    Torque: URIRef
    Force: URIRef
    MomentOfInertia: URIRef
    Mass: URIRef
    LengthMass: URIRef

    _NS = Namespace("http://qudt.org/vocab/quantitykind/")


class QUDT_UNIT(DefinedNamespace):
    UNITLESS: URIRef
    M: URIRef
    N: URIRef
    KiloGM: URIRef

    _extras = [
        "KiloGM-M2",
        "M-KiloGM",
        "M-PER-SEC",
        "M-PER-SEC2",
        "N-M",
        "RAD-PER-SEC",
        "RAD-PER-SEC2"
    ]

    _NS = Namespace("http://qudt.org/vocab/unit/")


class UUID(DefinedNamespace):
    _NS = Namespace("urn:uuid:")
