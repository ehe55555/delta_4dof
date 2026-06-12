#!/usr/bin/env bash
set -e

topics=(
  close_loop_121
  close_loop_211
  close_loop_221
  close_loop_311
  close_loop_321
  close_loop_41111_to_411111
)

for t in "${topics[@]}"; do
  echo "Attach $t"

  gz topic \
    -t "/model/descripe/${t}" \
    -m gz.msgs.Empty \
    -p ''
done