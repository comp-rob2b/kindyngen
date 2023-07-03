# SPDX-License-Identifier: MPL-2.0

# There exists a chain: frame -> link -> frame -> joint -> frame
# Returns multiple rows/matches at a branching point
# This also can match at the root (but not at a leaf)!
q_expand = """
PREFIX geom-ent: <https://comp-rob2b.github.io/metamodels/geometry/structural-entities#>
PREFIX kc-ent: <https://comp-rob2b.github.io/metamodels/kinematic-chain/structural-entities#>

SELECT ?child ?parent WHERE {
    ?node ^geom-ent:simplices / geom-ent:simplices ?joint_prox .
    FILTER (?node != ?joint_prox)
    ?joint_prox ^kc-ent:between-attachments / kc-ent:between-attachments ?child .
    FILTER (?joint_prox != ?child)
    BIND(?node as ?parent)
}
"""

# There only exists a frame, but no incoming chain:
# [frame -> link -> frame -> joint ->] frame
q_root = """
PREFIX geom-ent: <https://comp-rob2b.github.io/metamodels/geometry/structural-entities#>
PREFIX kc-ent: <https://comp-rob2b.github.io/metamodels/kinematic-chain/structural-entities#>

ASK {
    ?node a geom-ent:Frame .
    FILTER NOT EXISTS {
        ?root_prox ^geom-ent:simplices / geom-ent:simplices ?joint_prox .
        FILTER (?root_prox != ?joint_prox)
        ?joint_prox ^kc-ent:between-attachments / kc-ent:between-attachments ?node .
        FILTER (?joint_prox != ?node)
    }
}
"""

# There only exists a frame, but no outgoing chain:
# frame [-> link -> frame -> joint -> frame]
q_leaf = """
PREFIX geom-ent: <https://comp-rob2b.github.io/metamodels/geometry/structural-entities#>
PREFIX kc-ent: <https://comp-rob2b.github.io/metamodels/kinematic-chain/structural-entities#>

ASK {
    ?node a geom-ent:Frame .
    FILTER NOT EXISTS {
        ?node ^geom-ent:simplices / geom-ent:simplices ?joint_prox .
        FILTER (?node != ?joint_prox)
        ?joint_prox ^kc-ent:between-attachments / kc-ent:between-attachments ?joint_dist .
        FILTER (?joint_prox != ?joint_dist)
    }
}
"""


q_joint_motion = """
PREFIX geom-ent: <https://comp-rob2b.github.io/metamodels/geometry/structural-entities#>
PREFIX kc: <https://comp-rob2b.github.io/metamodels/kinematic-chain/structural-entities#>
PREFIX kc-ent: <https://comp-rob2b.github.io/metamodels/kinematic-chain/structural-entities#>
PREFIX kc-stat: <https://comp-rob2b.github.io/metamodels/kinematic-chain/state#>

SELECT ?child ?parent WHERE {
    ?node ^kc-ent:between-attachments / ^kc-stat:of-joint ?child .
    { ?child a kc-stat:JointPosition }
    UNION
    { ?child a kc-stat:JointVelocity }
    UNION
    { ?child a kc-stat:JointAcceleration }
    BIND(?node as ?parent)
}
"""
