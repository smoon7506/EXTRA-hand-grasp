#!/usr/bin/env bash
# 파이(데몬 노드)로 코드를 보낸다. 보낼 목록은 pi_manifest.txt 에 있다.
#
#   ./deploy_pi.sh                      기본 주소로
#   ./deploy_pi.sh wearlab@192.168.0.5  다른 주소로
#   ./deploy_pi.sh --dry-run            보낼 목록만 확인
#
# --- 왜 scp -r 을 안 쓰나 ---
# `scp -r detection hand_control pi:~/repo/` 는 __pycache__ 와 logs/
# 까지 통째로 보낸다. 2026-08-21 에 실제로 그러다 전송이 실패했다.
# logs/ 만 7MB · 139파일이고, 파이는 자기 로그를 따로 쓴다.
#
# --- 왜 목록을 별도 파일에 두나 ---
# git diff 에 "무엇을 파이로 보내는가"의 변화가 보이게 하려고. 스크립트
# 안의 --exclude 플래그로만 두면 새 폴더가 생겼을 때 아무도 못 알아챈다.
#
# --- PowerShell 에서 파이프하지 말 것 ---
# PowerShell 은 파이프를 텍스트로 다뤄서 tar 스트림을 망가뜨린다.
# 그래서 파이프 대신 파일로 만들어 scp 한다 -- 어느 셸에서든 같다.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="$HERE/pi_manifest.txt"
REMOTE_DIR="EXTRA-hand-grasp"

DRY_RUN=0
TARGET="wearlab@192.168.137.236"
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        *) TARGET="$arg" ;;
    esac
done

[ -f "$MANIFEST" ] || { echo "[ERROR] $MANIFEST 가 없다."; exit 1; }

# 매니페스트를 tar 인자로 바꾼다.
INCLUDES=()
EXCLUDES=()
while read -r verb value; do
    case "${verb:-}" in
        include) INCLUDES+=("$value") ;;
        exclude) EXCLUDES+=("--exclude=$value") ;;
    esac
done < <(grep -v '^\s*#' "$MANIFEST" | grep -v '^\s*$')

[ ${#INCLUDES[@]} -gt 0 ] || { echo "[ERROR] 매니페스트에 include 가 없다."; exit 1; }

cd "$HERE"
missing=0
for path in "${INCLUDES[@]}"; do
    [ -e "$path" ] || { echo "[ERROR] 없는 경로: $path"; missing=1; }
done
[ "$missing" -eq 0 ] || exit 1

if [ "$DRY_RUN" -eq 1 ]; then
    echo "[대상] $TARGET:~/$REMOTE_DIR"
    echo "[보냄]"; printf '  %s\n' "${INCLUDES[@]}"
    echo "[제외]"; printf '  %s\n' "${EXCLUDES[@]#--exclude=}"
    echo "[파일]"
    tar cf - "${EXCLUDES[@]}" "${INCLUDES[@]}" | tar tf - | sed 's/^/  /'
    exit 0
fi

# 확장자를 안 붙인다. mktemp 가 만든 파일과 다른 이름을 쓰면 원본이
# 그대로 남아서 임시 디렉터리에 빈 파일이 쌓인다. 원격 이름은 어차피
# 아래에서 deploy.tgz 로 고정한다.
BUNDLE="$(mktemp -t haram_deploy_XXXXXX)"
trap 'rm -f "$BUNDLE"' EXIT
tar czf "$BUNDLE" "${EXCLUDES[@]}" "${INCLUDES[@]}"

echo "[INFO] 묶음 $(du -h "$BUNDLE" | cut -f1) -> $TARGET:~/$REMOTE_DIR"
scp "$BUNDLE" "$TARGET:~/deploy.tgz"

# 파이에 이미 있는 것을 지우지 않는다 -- tar 는 덮어쓰기만 한다.
# 파이에서만 고친 파일(예: tactile_sensor 의 aarch64 패치)은 매니페스트에
# 없으므로 손대지 않는다.
ssh "$TARGET" "mkdir -p ~/$REMOTE_DIR \
    && tar xzf ~/deploy.tgz -C ~/$REMOTE_DIR \
    && rm -f ~/deploy.tgz \
    && find ~/$REMOTE_DIR -name '__pycache__' -type d -prune -exec rm -rf {} + \
    && echo '[INFO] 배포 완료' && ls -1 ~/$REMOTE_DIR"

echo
echo "[다음] 파이에서 데몬을 다시 띄우세요:"
echo "  ssh $TARGET"
echo "  source ~/venv/bin/activate && cd ~/$REMOTE_DIR/detection && python grasp_daemon.py"
