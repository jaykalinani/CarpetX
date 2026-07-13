# How to access test logs and build artifacts

Both scripts build and test directly on the host in the Cactus tree at `$CACTUSX` (pre-set in the host shell), so logs and artifacts are ordinary host files.

Per-test logs: `$CACTUSX/TEST/carpetx/<Thorn>/<test>.log` (testsuite summary: `$CACTUSX/TEST/carpetx/summary.log`).
Build artifacts: `$CACTUSX/configs/carpetx/`.
Executable: `$CACTUSX/exe/cactus_carpetx`.
