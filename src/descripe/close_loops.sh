#!/usr/bin/env bash
set -euo pipefail

topics=(
  close_loop_121
  close_loop_211
  close_loop_221
  close_loop_311
  close_loop_321
  close_loop_41111_to_411111
)

for t in "${topics[@]}"; do
  full_topic="/model/descripe/${t}"

  echo "[WAIT] ${full_topic}"

  found=0
  for _ in $(seq 1 150); do
    if gz topic -l | grep -Fxq "${full_topic}"; then
      found=1
      break
    fi

    sleep 0.1
  done

  if [[ "${found}" -ne 1 ]]; then
    echo "[ERROR] Không tìm thấy topic: ${full_topic}" >&2
    exit 1
  fi

  echo "[ATTACH] ${full_topic}"

  gz topic \
    -t "${full_topic}" \
    -m gz.msgs.Empty \
    -p ''
done

echo "[OK] Đã đóng toàn bộ vòng động học."
