#!/usr/bin/env bash
# Подключает зависимости как git submodules в external/.
# Запускать из любого места; скрипт идемпотентен.
set -euo pipefail
cd "$(dirname "$0")/.."

if [ ! -d .git ]; then
    git init
fi

add_sub() {
    local url="$1" path="$2" ref="${3:-}"
    if [ -d "$path/.git" ] || [ -f "$path/.git" ]; then
        echo "== $path уже есть, пропускаю"
    else
        echo "== добавляю $path"
        git submodule add "$url" "$path"
    fi
    if [ -n "$ref" ]; then
        git -C "$path" fetch --tags --quiet
        git -C "$path" checkout --quiet "$ref"
        git add "$path"   # зафиксировать выбранный ref в индексе, иначе submodule update его откатит
    fi
}

# Этап 0
add_sub https://github.com/ocornut/imgui.git external/imgui docking
add_sub https://github.com/glfw/glfw.git     external/glfw  3.4

# Диалог выбора файлов (этап 1)
add_sub https://github.com/aiekick/ImGuiFileDialog.git external/ImGuiFileDialog

# Понадобятся с этапа 1-2 (подключаем сразу, чтобы структура не менялась)
add_sub https://gitlab.vci.rwth-aachen.de:9000/OpenMesh/OpenMesh.git external/OpenMesh
add_sub https://gitlab.com/libeigen/eigen.git                         external/eigen 3.4.0

git submodule update --init --recursive
echo "Готово."
