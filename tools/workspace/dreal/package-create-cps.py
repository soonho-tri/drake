#!/usr/bin/env python2

from drake.tools.install.cpsutils import read_defs

defs = read_defs("#define DREAL_(VERSION_STRING)\s+([^\s]+)")

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "dreal",
  "Description": "SMT solver for nonlinear theories of reals",
  "License": "Apache-2.0",
  "Version": "%(VERSION_STRING)s",
  "Default-Components": [":dreal"]
}
""" % defs

print(content[1:])
