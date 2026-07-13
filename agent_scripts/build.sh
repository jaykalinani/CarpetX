#!/bin/bash

if [ -z "${CACTUSX:-}" ]; then
  echo "✗ missing required env var: CACTUSX" >&2
  exit 1
fi

if [ -z "${ETKGUIDE:-}" ]; then
  echo "✗ missing required env var: ETKGUIDE" >&2
  exit 1
fi

log=$(mktemp)
trap 'rm -f "$log"' EXIT
if (
  cd "$CACTUSX" &&
  Compile-ETK -e carpetx -j8 \
    -c "$ETKGUIDE/macos.cfg" \
    -t "$ETKGUIDE/../ThornList/carpetx.th"
) > "$log" 2>&1; then
  echo "✓ build"
else
  cat "$log"
  echo "✗ build failed" >&2
  exit 1
fi
