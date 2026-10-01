#!/usr/bin/env python3
"""GCC 9 rejects `cond ? *abstractBaseRef : throw ...` in gtsam/constrained/QpCost.h
(a ternary whose branches are an abstract-base reference and a throw-expression
has no common type GCC 9 accepts, even though the standard permits it and newer
GCC versions compile it). This replaces the ternary with an equivalent
null-check helper -- no logic change, just restructures the null check so it
is not a ternary.

Idempotent: if the ternary pattern is not found (e.g. a newer/older GTSAM
release already avoids it, or already uses this exact fix), the file is left
untouched and this prints a warning instead of failing the build, since the
underlying GCC-9-vs-abstract-ternary problem may simply not apply there.
"""
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "gtsam/constrained/QpCost.h"

with open(path, "r") as f:
    src = f.read()

pattern = re.compile(
    r"explicit QpCost\(const GaussianFactor::shared_ptr& factor\)\s*"
    r":\s*QpCost\(factor\s*\?\s*\*factor\s*:\s*throw std::invalid_argument\(\s*"
    r'"QpCost: shared Gaussian factor is null\."\)\)\s*\{\}',
    re.DOTALL,
)

replacement = (
    "explicit QpCost(const GaussianFactor::shared_ptr& factor)\n"
    "      : QpCost(checkNotNull(factor)) {}\n"
    "\n"
    " private:\n"
    "  // GCC 9 rejects `cond ? *abstractBaseRef : throw ...` (needs a common,\n"
    "  // non-abstract type for the conditional expression) -- this helper\n"
    "  // avoids that pattern without changing behavior.\n"
    "  static const GaussianFactor& checkNotNull(\n"
    "      const GaussianFactor::shared_ptr& factor) {\n"
    "    if (!factor) {\n"
    '      throw std::invalid_argument("QpCost: shared Gaussian factor is null.");\n'
    "    }\n"
    "    return *factor;\n"
    "  }\n"
    "\n"
    " public:"
)

new_src, n = pattern.subn(replacement, src)

if n == 0:
    print(
        f"WARNING: {path}: the GCC-9-incompatible ternary pattern was not found "
        "(GTSAM release may already avoid it, or its exact formatting differs). "
        "Leaving the file untouched -- if the build later fails on this file with "
        "GCC rejecting an abstract type in a ?: expression, this script needs updating.",
        file=sys.stderr,
    )
    sys.exit(0)

with open(path, "w") as f:
    f.write(new_src)

print(f"{path}: patched {n} occurrence(s).")
