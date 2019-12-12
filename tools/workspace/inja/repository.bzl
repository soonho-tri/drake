load("@drake//tools/workspace:github.bzl", "github_archive")

def inja_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        build_file = "@drake//tools/workspace/inja:package.BUILD.bazel",
        commit = "c36cbac39d21f79f515935242d66cffc9505a410",  # 20191203
        mirrors = mirrors,
        repository = "pantor/inja",
        sha256 = "ab354ab0b8b3fe2c59efbf1eb2ebfaa13babaddea13c833c2248fbe626756919",  # noqa
    )
