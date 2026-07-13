#!/bin/bash

if [ -z "${CACTUSX:-}" ]; then
  echo "✗ missing required env var: CACTUSX" >&2
  exit 1
fi

if [ -z "${ETKGUIDE:-}" ]; then
  echo "✗ missing required env var: ETKGUIDE" >&2
  exit 1
fi

log="$CACTUSX/last-build.log"
if (
  cd "$CACTUSX" &&
  Compile-ETK -e carpetx -j8 \
    -c "$ETKGUIDE/macos.cfg" \
    -t "$ETKGUIDE/../ThornList/carpetx.th"
) > "$log" 2>&1; then
  echo "✓ build"
else
  echo "--- first error lines ---"
  grep -n -m 12 -E 'error:|Error|\*\*\*' "$log" || echo "(no obvious error lines)"
  echo "--- last 40 lines ---"
  tail -40 "$log"
  echo "✗ build failed — full log: $log" >&2
  exit 1
fi
