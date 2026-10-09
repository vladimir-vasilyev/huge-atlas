# huge-atlas

Unwrap 100 million triangles seamlessly—with the market's only tool built for massive meshes.

A macOS tool (C++17, OpenGL 4.1 Core, Dear ImGui) that segments a triangle mesh into patches with
**Variational Shape Approximation** (Cohen-Steiner, Alliez, Desbrun, SIGGRAPH 2004), unwraps every patch with
**Linear Angle Based Parameterization** (Zayer, Lévy, Seidel, SGP 2007), packs the charts into one UV atlas that is safe
for mip mapping (Horizon pack, Lévy et al. 2002) and bakes the texture of the source mesh into the new atlas.

## Two modes

- **Make atlas** — the mesh is loaded as a half-edge mesh (OpenMesh); VSA, patch extraction, repair ladder
  (LinABF → active set → slits → LSCM → Tutte), automatic splitting of patches that do not unfold, packing, texture baking.
- **Make atlas (huge)** — for dense scans (tens of millions of faces) on flat arrays (about 100 bytes per face): the mesh is
  simplified (meshoptimizer, parallel), segmented and unwrapped on the simplified copy, the charts are lifted back onto the
  full mesh, patches that do not lift are repaired or re-unwrapped, the atlas is packed and the texture moved to it.
  The size of the simplified copy is a percentage of the faces (10% = 10 times fewer).

Both modes compare the result with the UV layout stored in the file (distortion, seams, texel density, gaps between charts,
mip levels the layout can use) and save OBJ / PLY with the atlas UV and the baked texture.

## Requirements

- macOS, Xcode (or Command Line Tools)
- CMake ≥ 3.20 (`brew install cmake`)

## Build

```bash
git clone --recurse-submodules https://github.com/vladimir-vasilyev/huge-atlas.git
cd huge-atlas
# if the clone has no submodules: ./scripts/setup_deps.sh
cmake -G Xcode -B build-xcode
cmake --build build-xcode --config Release --target vsa_app
./build-xcode/Release/vsa_app
```

Without Xcode: `cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j`.
Tests (their own group in the IDE project): `cmake --build build-xcode --config Release --target all_tests`, then `ctest`.
Use a Release build for big models: a Debug build is very slow on large meshes.

## Models

Large test models are not part of the repository. `data/` has only small ones (a Stanford bunny, a cube, a hemisphere).
The scenes used for the measurements (a 3.2M face scan of a combat scene and a 5.7M face car, both by
[Artec 3D](https://www.artec3d.com), CC BY 3.0) have to be downloaded separately and put in `data/doom-combat-scene_obj/`
and `data/car/`; `data/*/license.txt` keep their attribution.

## Testing without clicks

Environment hooks run the pipeline and exit, for example:

```bash
VSA_AUTO_ATLAS=1 VSA_MESH=data/bunny.ply VSA_K=8 ./build-xcode/Release/vsa_app
VSA_AUTO_HUGE=data/car/FIN_CAR_D.obj VSA_K=200 VSA_HUGE_PERCENT=10 VSA_ATLAS_RES=8192 VSA_ATLAS_MIP=1 VSA_AUTO_BAKE=1 ./build-xcode/Release/vsa_app
```

See `CLAUDE.md` for the list of hooks and the development log (in Russian) and `docs/PLAN.md` for the plan.

## Layout

```
external/      dependencies (git submodules)
scripts/       helper scripts
src/app/       window, main loop, UI
src/render/    OpenGL wrappers, shaders, drawing
src/core/      mesh, VSA (no GL / ImGui dependency)
src/param/     patches, LinABF, UV, repair, huge pipeline, atlas packing
tests/         tests of the core
```

## References

- Cohen-Steiner, Alliez, Desbrun. *Variational Shape Approximation.* SIGGRAPH 2004.
- Zayer, Lévy, Seidel. *Linear Angle Based Parameterization.* SGP 2007.
- Sheffer, Lévy, Mogilnitsky, Bogomyakov. *ABF++: Fast and Robust Angle Based Flattening.* ACM TOG 2005.
- Lévy, Petitjean, Ray, Maillot. *Least Squares Conformal Maps for Automatic Texture Atlas Generation.* SIGGRAPH 2002.

## License

MIT, see `LICENSE`. The dependencies in `external/` have their own licenses.
