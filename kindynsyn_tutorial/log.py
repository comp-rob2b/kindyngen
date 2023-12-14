from kindynsyn_tutorial.my_logger import MY_LOG, MyLoggerTranslator

q = """
PREFIX rob: <https://comp-rob2b.github.io/robots/kinova/gen3/7dof/>
PREFIX algo: <https://comp-rob2b.github.io/metamodels/algorithm#>
PREFIX ctrl: <https://example.org/ctrl#>
PREFIX log: <https://example.org/logging#>

DELETE {
    ?ins_ptr rdf:rest ?rest .
} INSERT {
    ?log a log:Logger ;
        log:quantity ?twist, ?wrench, rob:q2 .
    _:b1 rdf:first ?log ;
        rdf:rest ?rest .
    ?ins_ptr rdf:rest _:b1 .
} WHERE {
    ?schedule a algo:Schedule ;
        algo:trigger-chain / rdf:rest* / rdf:first ?op .
    ?ins_ptr rdf:first ?op ;
        rdf:rest ?rest .
    ?op a ctrl:Damping ;
        ctrl:velocity-twist ?twist ;
        ctrl:wrench ?wrench .
    BIND(UUID() AS ?log)
}
"""

def postprocessor(g):
    g.update(q)

def translator_configurator():
    return [MyLoggerTranslator()]
