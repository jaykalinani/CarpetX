#!/bin/bash

if [ -z "${CACTUSX:-}" ]; then
  echo "✗ missing required env var: CACTUSX" >&2
  exit 1
fi

log=$(mktemp)
trap 'rm -f "$log"' EXIT
# PROMPT=no makes RunTest.pl take the default answer for every prompt,
# so the testsuite runs non-interactively regardless of TTY
if (
  cd "$CACTUSX" &&
  make carpetx-testsuite PROMPT=no
) > "$log" 2>&1; then
  if grep -q 'Number failed            -> 0' "$log"; then
    echo "✓ test"
  else
    cat "$log"
    echo "✗ test failed" >&2
    exit 1
  fi
else
  cat "$log"
  echo "✗ test failed" >&2
  exit 1
fi
