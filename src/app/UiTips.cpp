#include "app/UiTips.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace {

const std::unordered_map<std::string, const char*>& table()
{
    static const std::unordered_map<std::string, const char*> t = {
        // ---- File / View menus, Info and Mesh panels
        {"Open...", "Load a mesh (OBJ, PLY, OFF, STL) into the half-edge mesh. For scans that do not fit, use Make atlas (huge)."},
        {"Save", "Write the mesh with the atlas UV and the baked texture next to the file it came from (OBJ or PLY)."},
        {"Save As...", "Choose the file name and format (OBJ or PLY) for the mesh with the atlas UV and texture."},
        {"Binary PLY", "Write PLY files in binary form: smaller and much faster for big meshes than the text form."},
        {"Quit", "Close the program."},
        {"Wireframe", "Draw the triangle edges over the shaded surface. Slow on meshes of millions of faces."},
        {"Reset camera", "Return the orbit camera to the starting view of the whole model."},
        {"ImGui demo", "Open the demo window of the interface library (for developers)."},
        {"Background", "Colour of the 3D view behind the model."},
        {"Debug triangle", "Show the test triangle drawn by the first stage of the program (checks the graphics context)."},
        {"Rotate", "Spin the test triangle."},
        {"Cull back faces (faster on scans)", "Skip triangles that face away from the camera: about half of a closed scan, so the view is faster. An open surface seen from behind shows holes."},
        {"Anti-aliasing (MSAA)", "Smooth the edges with multisampling. Costs speed and memory on big meshes (it is switched off automatically above 1.5 million faces)."},
        {"Gray", "Plain light gray surface."},
        {"Face normals", "Colour every face by its normal direction."},
        {"Texture", "Show the texture of the loaded mesh (needs UV and an image next to the file)."},
        {"Regions", "Colour every face by the proxy (region) of the VSA partition it belongs to."},
        {"Weights", "Colour faces by the weight painted on them: heavier faces pull the partition towards them."},

        // ---- VSA panel
        {"Metric", "How the distance of a face to a proxy plane is measured. L2,1 compares normals (fast, good for flat regions); L2 measures the squared distance of the points to the plane."},
        {"Proxies (k)", "Number of regions (planar proxies) the surface is cut into. More regions follow the shape more closely but mean more charts."},
        {"Proxies (k) ", "Number of regions the surface is cut into."},
        {"Seeding", "How the first face of every region is chosen: randomly, or by farthest point (spread evenly over the surface, usually better)."},
        {"Random seed", "Number that fixes the random choices, so that a run can be repeated exactly."},
        {"Seed", "Choose the starting faces of the regions with the current method and k."},
        {"Partition", "One flooding pass: every face joins the region whose proxy fits it best, regions grow from their seeds."},
        {"Fit", "Recompute the best proxy plane of every region from the faces it holds."},
        {"Step", "One Lloyd iteration: partition, smooth the borders, fit the proxies."},
        {"Run", "Repeat Lloyd iterations until the energy stops changing or the iteration limit is reached."},
        {"Stop", "Stop the running iterations."},
        {"Max iterations", "Upper limit of Lloyd iterations for Run."},
        {"Patience", "How many iterations in a row must change the energy by less than the tolerance before Run stops."},
        {"Rel. tolerance", "Relative change of the energy below which an iteration counts as 'no progress'."},
        {"Lambda (perimeter)", "Border smoothing: how much a shorter, straighter region border is worth against the fit error. 0 keeps the ragged border of the flooding; 0.1 is a good start."},
        {"Kappa (crease)", "Makes the smoothing cheaper along creases: borders prefer to run where the surface bends. 0 ignores the bend."},
        {"Max sweeps", "Upper limit of passes over all faces in one smoothing step."},
        {"Smooth now", "Apply the border smoothing once to the current partition."},
        {"Insert", "Add a region: take the face with the largest error out of the worst region and make it a new seed."},
        {"Delete cheapest", "Remove the region whose merge with a neighbour costs the least error."},
        {"Teleport", "Move a proxy from the region where it is cheapest to lose to the region where it helps the most; kept only if the total error drops."},
        {"Grow to k", "Insert regions one by one until the partition has k of them."},
        {"Run + teleport", "Lloyd iterations alternated with teleportation of proxies: escapes some local minima."},
        {"Teleport ratio", "How much better a move must be, relative to the worst region, to be tried."},
        {"Max teleports", "Upper limit of successful teleports in one run."},
        {"Paint (Alt+drag rotates)", "Paint on the model with the mouse: painted faces get a weight, so the partition spends more regions there. Alt+drag still rotates the camera."},
        {"Brush radius (px)", "Radius of the paint brush on the screen."},
        {"Brush weight", "Weight given to the painted faces (1 = normal, larger = more detail wanted there)."},
        {"Erase (set to 1)", "Painting sets the weight back to 1 instead."},
        {"Reset weights", "Set the weight of every face back to 1."},

        // ---- Patches panel
        {"Merge regions smaller than", "Regions with fewer faces than this are merged into the neighbour they share the longest border with."},
        {"Cut multi-boundary patches into disks", "A patch with a hole is cut along the shortest path from its outer border to the hole, so that it becomes a disk and can be flattened."},
        {"Extract patches", "Turn the regions into separate meshes (patches) with their own vertices along the seams."},
        {"Explode", "Move the patches apart in the 3D view to see their borders."},
        {"Show patch boundaries", "Draw the borders between patches (and the cuts) over the model."},
        {"Boundary width (px)", "Thickness of the drawn patch borders."},
        {"Patches", "Colour every face by its patch."},
        {"Patch kind", "Colour patches by topology: disk (green), with holes, with handles, closed, other."},

        // ---- LinABF panel
        {"Max linearization steps", "The angle constraints around a vertex are not linear; the solver linearizes them again up to this many times until they hold."},
        {"Wheel tolerance", "A vertex counts as satisfied when the angles around it sum to 2 pi within this error."},
        {"Fan deficit threshold", "Vertices whose angle sum is more than this many radians away from 2 pi are scaled before the first solve."},
        {"Solve selected", "Compute the corrected angles (LinABF) of the selected patch."},
        {"Solve all disks", "Compute the corrected angles of every patch that is a disk."},
        {"Angle error", "Colour the faces by how much LinABF had to change their angles."},
        {"Greedy (debug) instead of least squares", "Build the UV by walking from triangle to triangle instead of solving one least squares problem. Errors pile up on big patches; for debugging only."},
        {"UV selected", "Compute the UV coordinates of the selected patch from its solved angles."},
        {"UV all solved", "Compute the UV of every patch whose angles are solved."},

        // ---- Parameterize panel
        {"Active set (pin angles, limit boundary fans)", "Second try when plain LinABF is not injective: angles that went out of range are pinned, and the angle sum at the border is limited."},
        {"Fold delta (rad)", "The angles around a border vertex must sum to less than 2 pi minus this amount, otherwise the border folds over itself."},
        {"Angle margin (rad)", "Angles are kept at least this far away from 0 and pi."},
        {"Slits from the worst vertex", "Third try: cut a slit from the vertex with the largest angle defect to the border, which relieves the stretch."},
        {"Max slits", "How many slits may be cut in one patch."},
        {"Tutte fallback (always injective)", "Last resort: embed the patch with a convex combination map. Never overlaps, but distorts angles and areas a lot, so the patch is marked for splitting."},
        {"Split above L2 stretch", "A triangle stretched more than this (1 = no stretch) counts as too distorted; a patch with too many of them is split by the refinement."},
        {"Parameterize selected", "Flatten the selected patch with the repair ladder: LinABF, active set, slits, Tutte."},
        {"Parameterize all disks", "Flatten every patch that is a disk, in parallel."},
        {"Stretch", "Colour faces by L2 stretch: blue is isometric, red is badly stretched."},
        {"Problems", "Highlight faces that are flipped, on crossings, or on folds of the border; orange patches need a split."},
        {"UV checker", "Show a checkerboard through the UV of the atlas (or of the patch): even squares of equal size mean an even texel density."},
        {"Horizon pack (texel exact, mip aware)", "Pack the charts into a square texture texel by texel, with a gap that keeps mip mapped lookups from reading a neighbour."},
        {"Shelf pack (quick)", "Simple packing of the charts' rectangles in rows. Fast, but wastes more space."},
        {"Texture size", "Side of the square atlas texture in texels."},
        {"Max mip level", "Highest mip level that must stay clean: the gap between charts grows with it (level 3 needs 22 texels). Lower it if the texture is never viewed from far away."},
        {"Extra margin (texels)", "Additional space around every chart on top of what the mip level needs."},
        {"Atlas border (texels)", "Texels kept free around the whole atlas. 0 is safe even with texture wrapping."},
        {"Rotations tried per chart", "Orientations tried for every chart when placing it on the horizon: more gives a tighter packing and takes longer."},
        {"Fill holes under overhangs", "Put charts into the free space below the horizon, left by earlier charts, before raising the horizon."},
        {"Rotations tried per hole", "Orientations tried when looking for a hole. Doom-sized scans pack as well with 1-2 as with 4, and much faster."},
        {"Threads (0 = all cores)", "Threads used for the hole search. The result does not depend on the number."},
        {"Longest first by diameter (else by area)", "Order of placement: the longest chart first (as in the paper) or the largest area first."},
        {"Gutter", "Gap between charts, as a part of the atlas side."},
        {"Rotate to minimum rectangle", "Turn every chart so that its bounding rectangle is as small as possible."},
        {"Checker squares", "Size of the checkerboard squares of the UV checker: the model is this many squares across. The squares are whole texels of the atlas and equal on the surface."},
        {"Pack atlas", "Arrange all flattened patches in one atlas."},
        {"Export OBJ...", "Write an OBJ with the atlas texture coordinates (one set per patch vertex, seams are duplicated)."},
        {"Export mask PNG...", "Write the occupancy of the atlas as a PNG: the alpha channel marks the texels that charts touch."},

        // ---- Refinement panel
        {"Fix first (ladder, then split)", "Try the whole repair ladder on a bad patch before splitting it. Fewer patches, but slow on big ones."},
        {"Split immediately (plain LinABF)", "Only plain LinABF; whatever fails is split at once. About three times faster, with more patches."},
        {"Max depth", "How many times a face may be split again and again."},
        {"Min faces per part", "Both parts of a split must have at least this many faces."},
        {"Split at once above (faces)", "A patch with more faces than this is only tried with plain LinABF and split at once if that fails, without the slow repair ladder. 0 turns it off (every patch gets the whole ladder: fewer patches, much slower on big scans)."},
        {"Max rounds", "Upper limit of parameterize-and-split rounds."},
        {"Local smoothing lambda", "Border smoothing of the local partition made when a patch is split."},
        {"Refine bad patches", "Split every patch that is not a disk, overlaps itself or is too stretched, parameterize the parts, and repeat. Runs in the background."},
        {"Cancel", "Stop after the current step."},

        // ---- Make atlas panel
        {"Patches to start from (k)", "Number of regions of the first partition. The refinement then splits the patches that cannot be flattened, so the final count is larger."},
        {"Improve the partition with teleportation (slower)", "Alternate Lloyd iterations with moving proxies to better places: a slightly better partition for more time."},
        {"Make atlas", "Do everything in one go: partition the mesh (VSA), cut patches, flatten and refine them, pack the atlas."},
        {"Bake atlas texture", "Create the new atlas texture from the source texture through the old and the new UV."},
        {"Compare with the original layout", "Measure the new atlas against the UV the file came with: charts, seams, stretch, texel density, coverage, colour error of the retexturing."},
        {"View the baked atlas texture", "Show the baked texture on the model."},

        // ---- Huge mode
        {"Simplified mesh, faces", "Size of the simplified copy that is segmented and unwrapped first. About 300 thousand faces works for most scans; bigger is slower and more exact."},
        {"Patches to start from", "Number of regions on the simplified mesh. The patch count of the result is larger (patches that do not unfold are split)."},
        {"Make atlas (huge)", "Huge mode for dense scans (tens of millions of faces). It works on compact arrays (about 100 bytes per face): with a mesh loaded this button takes it and its texture from memory, no file dialog. The mesh is simplified, segmented and unwrapped on the simplified copy, the charts are lifted back onto the full mesh, the ones that do not lift are repaired, the atlas is packed and the texture moved to it. It cannot be cancelled."},
        {"Make atlas (huge)...", "Huge mode for dense scans (tens of millions of faces). Pick a PLY or OBJ file: it is read straight into compact arrays, simplified, segmented and unwrapped on the simplified copy, the charts are lifted back onto the full mesh, the atlas is packed and the texture moved to it. It cannot be cancelled."},
        {"from a file...", "Process another mesh straight from a PLY or OBJ file (read into compact arrays) instead of the loaded one."},
        {"patches", "Colour every face by the patch it belongs to."},
        {"atlas checker", "Checkerboard through the new atlas UV: equal squares mean an even texel density."},
        {"plain", "Plain gray surface."},
        {"texture", "The old texture moved onto the new atlas."},
        {"##squares", "Size of the checker squares: the model is this many squares across. The squares are whole texels of the atlas."},
        {"Show the huge result in 3D", "Show the result of the huge mode in the 3D view."},
        {"Save the huge result (PLY)...", "Write a binary PLY with the atlas UV (s, t) and, if there was a texture, the baked atlas as a PNG next to it."},

        // ---- UV view
        {"Edges", "Draw the triangle edges (3D view of patches) or the chart borders over the atlas."},
        {"Fit view", "Zoom and centre the UV view on the atlas."},
        {"Atlas", "Show all charts as packed in the atlas instead of the selected patch."},
    };
    return t;
}

} // namespace

void uiTip(const char* label)
{
    if (!ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) return;
    const auto it = table().find(label);
    if (it != table().end()) {
        ImGui::SetTooltip("%s", it->second);
        return;
    }
    // a control without a text: tell a developer once
    static std::unordered_set<std::string> reported;
    if (std::getenv("VSA_VERBOSE") && reported.insert(label).second) std::fprintf(stderr, "[ui] no tooltip for \"%s\"\n", label);
}
