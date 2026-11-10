#!/usr/bin/env bash
# Fail if a tracked file contains something that identifies the authors, their machines, or
# their private infrastructure.
#
# WHY THIS EXISTS AS A SCRIPT RATHER THAN A RULE IN A DOC. This repo becomes a
# double-anonymous review capsule, and the leaks found in Aug 2026 were not careless — they
# were in places nobody thinks of as prose: the `maintainer` field of 17 package.xml files
# that ROS tooling generates for you, a benchmark filename that `hostname -s` produced
# automatically on every run, and a structural review document that had been committed into
# docs/ while itself listing the leaks it contained. A rule in CONTRIBUTING.md would not have
# caught any of those, because none of them were written by hand with intent.
#
# SCOPE IS TRACKED FILES ONLY, deliberately, via `git ls-files`. `$ARTS_LOCAL`, the
# `reference/` symlinks and `code_intake/` are where identifying material is ALLOWED to live —
# that separation is the whole point of the two-tree layout, so scanning the working directory
# instead of the index would report the design as a defect.
#
# PUBLISHED PRIOR ART IS NOT A LEAK and must not be added to the patterns below. Citing MOPS,
# dVRK or any other published platform by name is correct and wanted; the rule is about
# identifying the authors OF THIS WORK. The one in-house exception is the prior system's
# acronym, which is banned because it maps to this lab's own earlier publications.
#
# Usage:  ./deploy/ci/check_anonymity.sh [--list]
#         --list  print every match instead of just the summary
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."

LIST=0
[ "${1:-}" = "--list" ] && LIST=1

# One pattern per line: "<label>|<extended regex>". Case-insensitive.
#
# ADD A PATTERN WHENEVER A NEW CLASS OF LEAK IS FOUND, and add it here rather than fixing the
# file quietly — the file is one instance, the pattern is the class.
PATTERNS=$(cat <<'EOF'
maintainer name|jiawei|jge9
institution email|@jh\.edu|@jhu\.edu
collaborator name|mcdonough|dmcdono|\bdoug\b
collaborator package name|sarrts
personal account|github\.com/[A-Za-z0-9_-]+/(arts|esu_tap)
machine hostname|alienware|sonamyeshe|imerse
prior system acronym|\bastr\b
EOF
)

FAILED=0
TRACKED=$(git ls-files)

# The checker names the banned acronym in its own PATTERNS block, and a benchmark report may
# legitimately quote a CPU model. Exempt only files that must contain a pattern to do their
# job, and name each one so the exemption list cannot quietly grow.
EXEMPT='^(deploy/ci/check_anonymity\.sh|CONTRIBUTING\.md|docs/repo_structure_triage\.md)$'

while IFS='|' read -r label rest; do
    [ -z "${label:-}" ] && continue
    # Remaining fields are alternatives joined by '|' in the source line.
    regex="$rest"
    hits=$(printf '%s\n' "$TRACKED" \
        | grep -Ev "$EXEMPT" \
        | xargs -r -d '\n' grep -IlniE -- "$regex" 2>/dev/null || true)
    if [ -n "$hits" ]; then
        FAILED=1
        count=$(printf '%s\n' "$hits" | wc -l)
        echo "FAIL  $label — $count tracked file(s)"
        if [ "$LIST" -eq 1 ]; then
            # -H because grep omits the filename when handed exactly one file, which is the
            # common case here and the one where you most need to know which file it was.
            printf '%s\n' "$hits" | xargs -r -d '\n' grep -IHniE -- "$regex" | sed 's/^/        /'
        else
            printf '%s\n' "$hits" | sed 's/^/        /'
        fi
    else
        echo "ok    $label"
    fi
done <<< "$PATTERNS"

if [ "$FAILED" -ne 0 ]; then
    echo
    echo "Identity leaked into tracked files. Re-run with --list to see the lines."
    echo "Identifying material belongs in \$ARTS_LOCAL or behind a reference/ symlink,"
    echo "never in the index — see CONTRIBUTING.md, 'Anonymity'."
    exit 1
fi

echo
echo "PASS — no identity found in tracked files."
