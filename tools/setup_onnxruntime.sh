#!/usr/bin/env bash
# Project-local CPU runtime. Does not change system packages or shell settings.
set -euo pipefail
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  echo 'This pinned runtime is for Linux x86_64. Set ONNXRUNTIME_ROOT for another platform.' >&2
  exit 1
fi
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_name=onnxruntime-linux-x64-1.23.2
archive_sha=1fa4dcaef22f6f7d5cd81b28c2800414350c10116f5fdd46a2160082551c5f9b
if [[ -f "$project_dir/third_party/$runtime_name/include/onnxruntime_cxx_api.h" &&
      -f "$project_dir/third_party/$runtime_name/lib/libonnxruntime.so.1.23.2" ]]; then
  echo "Runtime already available: $project_dir/third_party/$runtime_name"
  exit 0
fi
temporary_dir="$(mktemp -d)"
trap 'rm -rf "$temporary_dir"' EXIT
curl --fail --location --max-time 120 \
  "https://github.com/microsoft/onnxruntime/releases/download/v1.23.2/$runtime_name.tgz" \
  --output "$temporary_dir/runtime.tgz"
printf '%s  %s\n' "$archive_sha" "$temporary_dir/runtime.tgz" | sha256sum --check
mkdir -p "$project_dir/third_party"
tar -xzf "$temporary_dir/runtime.tgz" -C "$project_dir/third_party"
echo "Runtime ready: $project_dir/third_party/$runtime_name"
