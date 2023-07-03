# SPDX-License-Identifier: MPL-2.0
import os

def loader(directory):
    """
    Return a function that loads a file from the specified directory.
    """
    def load(file):
        with open(os.path.join(directory, file)) as f:
            return f.read()
        return ""
    return load


def log(*args):
    """
    Logging function that can quickly be deactivated over the whole code base.
    """
    #print(" ".join(map(str, args)))
    pass
