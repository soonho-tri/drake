# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def tcc_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # repository = "TinyCC/tinycc",
        # commit = "release_0_9_27",
        repository = "dreal-deps/tcc",
        commit = "43f279f0af16ed33d206e5077dbe3b0bf9ce09a6",
        sha256 = "7ba0ae283ecea61c46f02c7e1465021126195108d1f501c09b3a53dbe597c4c7",
        build_file = "@drake//tools/workspace/tcc:package.BUILD.bazel",
        mirrors = mirrors,
    )
