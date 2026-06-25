#!/usr/bin/env bash
# Update repo, restart venator-lora.service, optionally follow logs.

set -euo pipefail

SERVICE_NAME="venator-lora.service"
BRANCH="main"
REMOTE="origin"

cd "$(dirname "$0")"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "Error: not a git repository ($(pwd))"
    exit 1
fi

echo "==> Repository: $(pwd)"
echo "==> Updating from ${REMOTE}/${BRANCH}..."

if [[ -n "$(git status --porcelain)" ]]; then
    echo
    echo "Local changes detected:"
    git status --short
    echo
    read -r -p "Discard local changes and reset to ${REMOTE}/${BRANCH}? [y/N] " reset_ans
    if [[ "${reset_ans}" =~ ^[Yy]$ ]]; then
        git fetch "${REMOTE}" "${BRANCH}"
        git reset --hard "${REMOTE}/${BRANCH}"
        echo "Reset to ${REMOTE}/${BRANCH}."
    else
        read -r -p "Try git pull anyway (may fail or merge)? [y/N] " pull_ans
        if [[ "${pull_ans}" =~ ^[Yy]$ ]]; then
            git pull "${REMOTE}" "${BRANCH}"
        else
            echo "Aborted — no git changes applied."
            exit 1
        fi
    fi
else
    git pull "${REMOTE}" "${BRANCH}"
fi

echo
echo "==> Reloading systemd and restarting ${SERVICE_NAME}..."
sudo systemctl daemon-reload
sudo systemctl restart "${SERVICE_NAME}"

echo
echo "Service restarted. Status:"
sudo systemctl --no-pager status "${SERVICE_NAME}" || true

echo
read -r -p "Follow service logs (journalctl -f)? [Y/n] " log_ans
if [[ ! "${log_ans}" =~ ^[Nn]$ ]]; then
    echo "==> Following logs (Ctrl+C to stop)..."
    sudo journalctl -u "${SERVICE_NAME}" -f
fi
