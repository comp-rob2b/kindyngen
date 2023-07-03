# SPDX-License-Identifier: MPL-2.0
import os
from rdflib.plugins.sparql import prepareQuery, prepareUpdate


def sparql_prepare(filename, data):
    _, extension = os.path.splitext(filename)

    if extension == ".ru":
        return prepareUpdate(data)

    if extension == ".rq":
        return prepareQuery(data)

    return data


def sparql_cache(file_loader, process = lambda name, data: data):
    files = {}

    def load(file):
        if file in files:
            return files[file]

        q = file_loader(file)
        files[file] = process(file, file_loader(file))
        return q

    return load
