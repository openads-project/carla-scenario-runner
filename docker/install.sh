#!/usr/bin/env bash
set -euo pipefail

: "${SCENARIO_RUNNER_ROOT:?SCENARIO_RUNNER_ROOT must point to the Scenario Runner checkout}"

if [[ -z "${CARLA_ARTIFACTS_URL:-}" && -z "${CARLA_ARTIFACTS_PATH:-}" ]]; then
    echo "Either CARLA_ARTIFACTS_URL or CARLA_ARTIFACTS_PATH must point to the CARLA artifacts" >&2
    exit 1
fi

CARLA_PATH="${CARLA_PATH:-/opt/carla}"
CARLA_API_PATH="${CARLA_API_PATH:-$CARLA_PATH/PythonAPI}"
CARLA_CACHE_DIR="${CARLA_CACHE_DIR:-/tmp/carlaCache}"
CARLA_SETUP_SCRIPT="${CARLA_SETUP_SCRIPT:-/opt/carla/setup.bash}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

artifacts_dir="$(mktemp -d)"
requirements_raw="$(mktemp)"
requirements_deduped="$(mktemp)"
cleanup() {
    rm -rf "$artifacts_dir" "$requirements_raw" "$requirements_deduped"
}
trap cleanup EXIT

# Install Scenario Runner Python dependencies from the copied repository.
"$PYTHON_BIN" -m pip install -r "$SCENARIO_RUNNER_ROOT/requirements.txt"

# Download or install CARLA PythonAPI artifacts.
mkdir -p "$CARLA_PATH"
if [[ -n "${CARLA_ARTIFACTS_PATH:-}" ]]; then
    [[ -d "$CARLA_ARTIFACTS_PATH" ]] || {
        echo "CARLA_ARTIFACTS_PATH must point to an unpacked PythonAPI directory" >&2
        exit 1
    }
    mv "$CARLA_ARTIFACTS_PATH" "$CARLA_API_PATH"
else
    curl --location --output "$artifacts_dir/artifacts.zip" "$CARLA_ARTIFACTS_URL"
    unzip -q "$artifacts_dir/artifacts.zip" -d "$artifacts_dir"
    mv "$artifacts_dir/artifacts/PythonAPI" "$CARLA_API_PATH"
fi

# Install PythonAPI requirements, keeping the version of the first occurrence.
find "$CARLA_API_PATH" -type f -name "requirements.txt" -print0 | xargs -0 cat > "$requirements_raw"
awk -F '==' '{print $1}' "$requirements_raw" | awk '!visited[$1]++' > "$requirements_deduped"
"$PYTHON_BIN" -m pip install -r "$requirements_deduped"

# Install the CARLA wheel that matches the current Python minor version.
pyver=$("$PYTHON_BIN" -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')")
shopt -s nullglob
wheels=("$CARLA_API_PATH"/carla/dist/*"$pyver"*.whl)
shopt -u nullglob
if [[ ${#wheels[@]} -eq 0 ]]; then
    echo "No CARLA wheel found for Python $pyver in $CARLA_API_PATH/carla/dist" >&2
    exit 1
fi
"$PYTHON_BIN" -m pip install --no-cache-dir "${wheels[0]}"

mkdir -p "$(dirname "$CARLA_SETUP_SCRIPT")" "$CARLA_CACHE_DIR"
chmod 1777 "$CARLA_CACHE_DIR"

# Create a script to append necessary paths to PYTHONPATH.
{
    echo "export PYTHONPATH=\$PYTHONPATH:$CARLA_API_PATH/carla/agents"
    echo "export PYTHONPATH=\$PYTHONPATH:$CARLA_API_PATH/carla"
    echo "export PYTHONPATH=\$PYTHONPATH:$SCENARIO_RUNNER_ROOT"
    echo "export SCENARIO_RUNNER_ROOT=$SCENARIO_RUNNER_ROOT"
    echo "export CARLA_CACHE_DIR=$CARLA_CACHE_DIR"
} >> "$CARLA_SETUP_SCRIPT"
