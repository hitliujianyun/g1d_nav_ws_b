#!/usr/bin/env bash
# Installs the systemd units (expects the workspace at /opt/g1d_ws, built into /opt/g1d_ws/install).
#   sudo deploy/install.sh [--enable]
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ "$ROOT" != "/opt/g1d_ws" ]; then
  echo "note: units reference /opt/g1d_ws (this checkout is $ROOT); symlink or edit deploy/g1d.env" >&2
fi
install -m 644 "$ROOT"/deploy/systemd/g1d-*.service /etc/systemd/system/
mkdir -p "$ROOT/logs"
systemctl daemon-reload
if [ "${1:-}" = "--enable" ]; then
  systemctl enable g1d-guard.service g1d-lio.service g1d-nav.service
fi
echo "installed: systemctl start g1d-guard g1d-lio g1d-nav ; journalctl -fu g1d-nav"
