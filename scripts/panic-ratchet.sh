#!/usr/bin/env bash
# Counts the places non-test code can panic (unwrap, expect, panic!,
# unreachable!, todo!, unimplemented!) per file, with clippy, and compares
# them with `scripts/panic-baseline.txt`, which may only shrink.
#
#   scripts/panic-ratchet.sh           check: fails if a file has more
#   scripts/panic-ratchet.sh --update  write the baseline (after removing some)
set -euo pipefail
cd "$(dirname "$0")/.."
baseline=scripts/panic-baseline.txt
lints=(unwrap_used expect_used panic unreachable todo unimplemented)
flags=()
for lint in "${lints[@]}"; do flags+=(-W "clippy::$lint"); done

current=$(cargo clippy --workspace --exclude msla_format --message-format=short -- "${flags[@]}" 2>&1 |
    grep -E '^[^ ]+\.rs:[0-9]+:[0-9]+: warning: (used `(unwrap|expect)\(\)`|usage of the `unreachable!`|`(panic|todo|unimplemented)` should not)' |
    cut -d: -f1 | sort | uniq -c | awk '{print $2, $1}' || true)

if [[ ${1:-} == --update ]]; then
    printf '%s\n' "$current" >"$baseline"
    echo "baseline: $(awk '{s+=$2} END {print s}' "$baseline") panic sites"
    exit 0
fi

failed=0
while read -r file count; do
    [[ -z $file ]] && continue
    allowed=$(awk -v f="$file" '$1 == f {print $2}' "$baseline")
    if (( count > ${allowed:-0} )); then
        echo "$file: $count panic sites, baseline ${allowed:-0}" >&2
        failed=1
    fi
done <<<"$current"
total=$(printf '%s\n' "$current" | awk '{s+=$2} END {print s+0}')
echo "panic sites: $total (baseline $(awk '{s+=$2} END {print s+0}' "$baseline"))"
exit $failed
