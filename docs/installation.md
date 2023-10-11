# Installation

## Synthesizer

The following dependencies are required and can either be installed with the operating system's package manager or pip (see below):

* [Python](https://www.python.org/)
* [rdflib](https://github.com/RDFLib/rdflib)
* [pySHACL](https://github.com/RDFLib/pySHACL)
* [numpy](https://numpy.org/)

First, clone the repository and change into the cloned folder:
```bash
git clone https://github.com/comp-rob2b/kindyngen.git
cd kindynsyn
```

(Optional) To avoid cluttering the operating system's Python installation it is advisable to use a virtual environment:

```bash
python3 -m venv .venv
source ./.venv/bin/activate
```

Now, install the synthesizer via pip (the `-e` flag indicates an editable installation to avoid repeated installation steps after changes to the code). This will also install the required dependencies:
```bash
pip3 install -e .
```

Note: the virtual environment can be deactivated again, at the very end, via:
```bash
deactivate
```


## Code generator

The following dependencies are required for the code generator:

* [Java](https://openjdk.org/)
* [Apache Ant](https://ant.apache.org/)
* [StringTemplate](https://www.stringtemplate.org/)
* [STSTv4](https://github.com/jsnyders/STSTv4) (from Git!)
* [GNU Make](https://www.gnu.org/software/make/)

For convenience, we provide a step-by-step installation guide for the latter two dependencies. Note, that STSTv4 comes with a pre-bundled version of StringTemplate. Hence, the steps for StringTemplate can be considered optional and are only relevant if one plans to use a more recent StringTemplate version.

StringTemplate can either be [installed](https://github.com/antlr/stringtemplate4/blob/master/doc/java.md#installation) from the [pre-compiled version](https://www.stringtemplate.org/download.html) or it can be built from source:

1. Download the latest version and extract it to a directory `<st>`
2. Change to the directory `cd <st>`
2. For version 4.3.3 execute `sed "s/1.6/1.8/g" -i build.xml`
3. Compile using `ant`
4. This creates a JAR file `<jar>` (e.g. `ST-4.3.4.jar`) in the `<st>/dist` folder

STSTv4 must be [built](https://github.com/jsnyders/STSTv4#install-instructions) from the Git version (to support [nested JSON arrays](https://github.com/jsnyders/STSTv4/commit/6f72c8cc19b773bab015ef9cf58cabd2cb2984c8)):

1. Clone the source code: `git clone https://github.com/jsnyders/STSTv4.git`
2. Change into the repository: `cd STSTv4`
3. Build with `ant`
4. Copy the launch script template: `cp stst.sh.init stst.sh`
5. Adapt the `STST_HOME` variable in the launch script
6. (Optional) To use StringTemplate from above adapt the `CP` variable: `sed "s#\$STST_HOME/lib/ST-4.0.8.jar#<st>/dist/<jar>#g" -i stst.sh`
7. Fix the launch script: `sed "s#lib/stst.jar#build/jar/stst.jar#g" -i stst.sh`
8. Make the launch script executable: `chmod +x stst.sh`


## Generated code

The following dependencies are required to build and execute the generated code:

* [GCC](https://gcc.gnu.org/)
* [CMake](https://cmake.org/)
* [dyn2b](https://github.com/comp-rob2b/dyn2b)
* [robif2b](https://github.com/rosym-project/robif2b)

Make sure that the latter dependencies are accessible to the generated CMake script. That can be achieved via a local or system-wide installation, but also using CMake's package registry (to avoid the installation). To this end, both projects can be configured with a `-DENABLE_PACKAGE_REGISTRY=On` option.
