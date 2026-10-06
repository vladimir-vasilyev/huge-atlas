# VSA + LinABF

Сегментация меша методом Variational Shape Approximation (Cohen-Steiner, Alliez, Desbrun 2004)
и развёртка полученных патчей методом Linear Angle Based Parameterization (Zayer, Lévy, Seidel 2007).

Стек: C++17, OpenGL 4.1 Core, GLFW, Dear ImGui (docking), OpenMesh, Eigen. Платформа: macOS.

## Требования

- Xcode (или Command Line Tools)
- CMake ≥ 3.20 (`brew install cmake`)

## Сборка

```bash
./scripts/setup_deps.sh                 # git submodules в external/
cmake -G Xcode -B build-xcode
open build-xcode/VSAParam.xcodeproj     # схема vsa_app → Run
```

Release (в разы быстрее, нужен для больших моделей): `cmake --build build-xcode --config Release --target vsa_app`,
бинарник `build-xcode/Release/vsa_app`.

Без Xcode:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/vsa_app
```

## Структура

```
external/      зависимости (submodules)
scripts/       служебные скрипты
src/app/       окно, главный цикл, UI-логика
src/render/    обёртки OpenGL, шейдеры, отрисовка
src/core/      (этап 1+) меш, VSA — без зависимостей от GL/ImGui
src/param/     (этап 6+) патчи, LinABF, UV, атлас
tests/         (этап 2+) тесты ядра
```

## Этап 0 — критерии готовности

- [ ] окно открывается, в консоли и панели Info видно `4.1 ... Metal` и GLSL 4.10
- [ ] на Retina framebuffer в 2 раза больше окна, треугольник занимает центр, а не угол
- [ ] треугольник не искажается при изменении размеров окна
- [ ] панели ImGui перетаскиваются и пристыковываются к краям (docking)
- [ ] «ImGui wants mouse» = yes над панелью и no над пустой областью
- [ ] в консоли нет сообщений `[GL]` и `[GLFW]`
