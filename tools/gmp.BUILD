# We build GMP by shelling out to autotools.

# A prefix-string for genrule cmd attributes, which uses the Kythe cdexec tool,
# in quiet mode, to execute in the genrule output directory.
CDEXEC = "$(location @//tools/third_party/kythe/tools/cdexec:cdexec) -q $(@D)"

# We run autotools in a genrule, and only files explicitly identified as outputs
# of that genrule can be made available to other rules. Therefore, we need a
# list of every file in the GMP install.
# See https://github.com/bazelbuild/bazel/issues/281.

# find include/coin -name "*.h" -o -name "*.hpp" -o -name "*.hdd" | sort |
# sed 's/$/",/g'| sed 's/^/"/g'
GMP_HDRS = [
    "external/gmp/include/gmp.h"
]

# ls lib | grep "\.a$" | sed 's/$/",/g'| sed 's/^/"lib\//g'
# These are artisanally topo-sorted: demand before supply.
# If you change the order, you may get undefined-reference linker errors.
GMP_LIBS = [
    "external/gmp/lib/libgmp.a",
]

# Invokes ./configure, make, and make install to build GMP. We arbitrarily
# use make -j 32 and hope for the best in terms of overall CPU consumption, since
# Bazel has no way to tell a genrule how many cores it should use.
#
# We emit static libraries because dynamic libraries would have different names
# on OS X and on Linux, and Bazel genrules don't allow platform-dependent outs.
# https://github.com/bazelbuild/bazel/issues/281
BUILD_GMP_CMD = (
    CDEXEC + " `pwd`/external/gmp/configure" + " && " +
    CDEXEC + " make -j 32" + " && " +
    CDEXEC + " make install"
    # CDEXEC + " `pwd`/external/gmp/configure 2> /dev/null" +
    # " && " + CDEXEC + " make -j 32 2> /dev/null" +
    # " && " + CDEXEC + " make install 2> /dev/null"
)

genrule(
    name = "build_with_autotools",
    srcs = glob(["**/*"]),
    outs = GMP_HDRS + GMP_LIBS,
    cmd = BUILD_GMP_CMD,
    tools = ["@//tools/third_party/kythe/tools/cdexec:cdexec"],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "lib",
    srcs = GMP_LIBS,
    hdrs = GMP_HDRS,
    includes = ["include/coin"],
    linkstatic = 1,
    visibility = ["//visibility:public"],
    # deps = [
    #     "@gfortran//:lib",
    # ],
    alwayslink = 1,
)
