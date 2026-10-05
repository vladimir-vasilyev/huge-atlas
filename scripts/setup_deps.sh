#!/usr/bin/env bash
# Adds dependencies as git submodules in external/.
# Can be run from anywhere; the script is idempotent.
set -euo pipefail
cd "$(dirname "$0")/.."

if [ ! -d .git ]; then
    git init
fi

add_sub() {
    local url="$1" path="$2" ref="${3:-}"
    if [ -d "$path/.git" ] || [ -f "$path/.git" ]; then
        echo "== $path already present, skipping"
    else
        echo "== adding $path"
        git submodule add "$url" "$path"
    fi
    if [ -n "$ref" ]; then
        git -C "$path" fetch --tags --quiet
        git -C "$path" checkout --quiet "$ref"
        git add "$path"   # record the chosen ref in the index, otherwise submodule update reverts it
    fi
}

# Stage 0
add_sub https://github.com/ocornut/imgui.git external/imgui docking
add_sub https://github.com/glfw/glfw.git     external/glfw  3.4

# File dialog (stage 1)
add_sub https://github.com/aiekick/ImGuiFileDialog.git external/ImGuiFileDialog

# Needed from stage 1-2 (added right away so the layout does not change)
add_sub https://gitlab.vci.rwth-aachen.de:9000/OpenMesh/OpenMesh.git external/OpenMesh
add_sub https://gitlab.com/libeigen/eigen.git                         external/eigen 3.4.0

git submodule update --init --recursive
echo "Done."
