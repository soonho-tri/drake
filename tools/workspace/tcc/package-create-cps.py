#!/usr/bin/env python2

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "tcc",
  "Description": "Tiny C Compiler",
  "License": "LGPL-2.1",
  "Version": "0.9.27",
  "Default-Components": [":tcc"],
  "Components": {
    "tcc": {
      "Type": "interface",
      "Includes": ["@prefix@/include/tcc"]
    }
  }
}
""" % defs

print(content[1:])
