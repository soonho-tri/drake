# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def spdlog_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gabime/spdlog",
        commit = "v1.4.1",
        sha256 = "3291958eb54ed942d1bd3aef1b4f8ccf70566cbc04d34296ec61eb96ceb73cff",  # noqa
        build_file = "@drake//tools/workspace/spdlog:package.BUILD.bazel",
        mirrors = mirrors,
    )
