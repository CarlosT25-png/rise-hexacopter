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
echo "==> Fetching ${REMOTE}/${BRANCH}..."
git fetch "${REMOTE}" "${BRANCH}"

REMOTE_REF="${REMOTE}/${BRANCH}"
LOCAL_SHA="$(git rev-parse HEAD)"
REMOTE_SHA="$(git rev-parse "${REMOTE_REF}")"

reset_to_remote() {
    git reset --hard "${REMOTE_REF}"
    echo "Reset to ${REMOTE_REF}."
}

pull_ff_only() {
    if git merge-base --is-ancestor "${LOCAL_SHA}" "${REMOTE_SHA}"; then
        git merge --ff-only "${REMOTE_REF}"
        echo "Fast-forwarded to ${REMOTE_REF}."
    else
        echo "Error: cannot fast-forward (branches diverged)."
        return 1
    fi
}

if [[ -n "$(git status --porcelain)" ]]; then
    echo
    echo "Uncommitted local changes:"
    git status --short
    echo
    read -r -p "Discard changes and reset to ${REMOTE_REF}? [y/N] " reset_ans
    if [[ "${reset_ans}" =~ ^[Yy]$ ]]; then
        reset_to_remote
    else
        read -r -p "Try git pull --rebase anyway? [y/N] " pull_ans
        if [[ "${pull_ans}" =~ ^[Yy]$ ]]; then
            git pull --rebase "${REMOTE}" "${BRANCH}"
        else
            echo "Aborted — no git changes applied."
            exit 1
        fi
    fi
elif [[ "${LOCAL_SHA}" == "${REMOTE_SHA}" ]]; then
    echo "Already up to date with ${REMOTE_REF}."
elif git merge-base --is-ancestor "${LOCAL_SHA}" "${REMOTE_SHA}"; then
    echo "Behind ${REMOTE_REF} — fast-forwarding..."
    pull_ff_only
elif git merge-base --is-ancestor "${REMOTE_SHA}" "${LOCAL_SHA}"; then
    echo "Ahead of ${REMOTE_REF} (local commits only). Skipping pull."
else
    echo
    echo "Branch diverged from ${REMOTE_REF}:"
    git log --oneline --left-right "HEAD...${REMOTE_REF}" | head -10
    echo
    read -r -p "Reset to ${REMOTE_REF} (discard local commits)? [y/N] " reset_ans
    if [[ "${reset_ans}" =~ ^[Yy]$ ]]; then
        reset_to_remote
    else
        read -r -p "Try git pull --rebase? [y/N] " rebase_ans
        if [[ "${rebase_ans}" =~ ^[Yy]$ ]]; then
            git pull --rebase "${REMOTE}" "${BRANCH}"
        else
            echo "Aborted — no git changes applied."
            exit 1
        fi
    fi
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
