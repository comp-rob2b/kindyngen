# SPDX-License-Identifier: MPL-2.0
import urllib
import pathlib
import json

#
# For rdflib
#
class IriToFileResolver(urllib.request.OpenerDirector):
    '''
    This urllib OpenerDirectory remaps specific IRIs to local files. The url_map
    dictionary defines which IRIs (prefix or complete; the key in the
    dictionary) map to which local file or directory (the value in the
    dictionary). For example, `{ "http://example.org/": "foo/bar/" }` would
    remap any urllib open request for any resource under "http://example.org/"
    to a local directory "foo/bar/". In this example the local directory is
    given relative to current working directory.
    '''
    def __init__(self, url_map):
        super().__init__()
        self.default_opener = urllib.request.build_opener()
        self.url_map = url_map

    def open(self, fullurl, data=None, timeout=None):
        url = pathlib.Path(fullurl.full_url)

        # If the requested URL starts with any key in the url_map
        # fetch the file from a local file that is derived from
        # the URL and the value in the map
        for prefix, directory in self.url_map.items():
            if url.is_relative_to(prefix):
                # Wrap the directory in a pathlib.Path to get access to
                # convenience functions
                path = pathlib.Path(directory).joinpath(url.relative_to(prefix))

                # Open the file and wrap it in an urllib response
                fp = open(path, "rb")
                resp = urllib.response.addinfourl(fp,
                        headers={},
                        url=fullurl.full_url,
                        code=200)

                return resp

        # If we did not find any match above just continue with
        # the default opener which has the behaviour as initially
        # expected by rdflib.
        return self.default_opener.open(fullurl, data, timeout)

def install(resolver):
    '''
    Note that only a single opener can be globally installed in urllib. Only the
    latest installed resolver will be active.
    '''
    urllib.request.install_opener(resolver)



#
# For PyLD
#
def pyld_loader(url_map):
    def load(url_str, options={}):
        url = pathlib.Path(url_str)

        # If the requested URL starts with any key in the url_map
        # fetch the file from a local file that is derived from
        # the URL and the value in the map
        for prefix, directory in url_map.items():
            if url.is_relative_to(prefix):
                # Wrap the directory in a pathlib.Path to get access to
                # convenience functions
                path = pathlib.Path(directory).joinpath(url.relative_to(prefix))

                # Open the file and wrap it in an urllib response
                fp = open(path, "r", encoding="utf-8")
                doc = {
                    "contentType": "application/ld+json",
                    "contextUrl": None,
                    "documentUrl": url_str,
                    "document": json.load(fp)
                }

                return doc

        # Fail and print the URL that caused the problem
        print("No file found for:", url_str)
        assert False

    return load
