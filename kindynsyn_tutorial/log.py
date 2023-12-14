from kindynsyn_tutorial.my_logger import MY_LOG, MyLoggerTranslator

q = """ 
PREFIX rob: <https://comp-rob2b.github.io/robots/kinova/gen3/7dof/>
PREFIX algo: <https://comp-rob2b.github.io/metamodels/algorithm#>
PREFIX geom_rel: <https://comp-rob2b.github.io/metamodels/geometry/spatial-relations#>
PREFIX geom_coord: <https://comp-rob2b.github.io/metamodels/geometry/coordinates#>
PREFIX geom_op: <https://comp-rob2b.github.io/metamodels/geometry/spatial-operators#>
PREFIX rob: <https://comp-rob2b.github.io/robots/kinova/gen3/7dof/>
PREFIX log: <https://example.org/logging#>

DELETE {
    ?ins_ptr rdf:rest ?rest .
} INSERT {
    ?log a log:Logger ;
        log:quantity ?quant .
    _:b1 rdf:first ?log ;
        rdf:rest ?rest .
    ?ins_ptr rdf:rest _:b1 .
} WHERE {
    ?schedule a algo:Schedule ;
        algo:trigger-chain / rdf:rest* / rdf:first ?op .
    ?ins_ptr rdf:first ?op ;
        rdf:rest ?rest .
    ?quant a geom_coord:PoseCoordinate ;
        geom_coord:as-seen-by rob:world-frame ;
        geom_coord:of-pose / geom_rel:of rob:link7-root ;
        geom_coord:of-pose / geom_rel:with-respect-to rob:world-frame .
    ?op a geom_op:ComposePose ;
        geom_op:composite ?quant .
    BIND(UUID() AS ?log)
}
"""

def postprocessor(g):
    g.update(q)

def translator_configurator():
    return [MyLoggerTranslator()]
