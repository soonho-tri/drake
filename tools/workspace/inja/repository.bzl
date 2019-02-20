# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def inja_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "pantor/inja",
        commit = "v2.1.0",
        sha256 = "038ecde8f6dbad5d3cedb6ceb0853fd0e488d5dc57593a869633ecb30b0dfa6e",  # noqa
        build_file = "@drake//tools/workspace/inja:package.BUILD.bazel",
        mirrors = mirrors,
    )
