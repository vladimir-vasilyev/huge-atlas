 #pragma once

#include "core/BoundaryMetrics.h"
#include "core/MeshPrep.h"
#include "core/Operations.h"
#include "core/Smoothing.h"
#include "core/Partition.h"
#include "core/Proxy.h"
#include "param/AnglesToUV.h"
#include "param/AtlasBake.h"
#include "param/AtlasMask.h"
#include "param/MipSafety.h"
#include "param/UVMetrics.h"
#include "param/AtlasPacker.h"
#include "param/HorizonPacker.h"
#include "param/LinABF.h"
#include "param/ParamPipeline.h"
#include "param/PatchExtractor.h"
#include "param/FlatBake.h"
#include "param/FlatPipeline.h"
#include "param/Refinement.h"
#include "render/Camera.h"
#include "render/DebugTriangle.h"
#include "core/ObjLoader.h"
#include "render/MeshRenderer.h"
#include "render/SceneFramebuffer.h"

#include <imgui.h>

#include <atomic>
#include <memory>
#include <thread>
#include <string>
#include <vector>

struct GLFWwindow;

// Window, main loop, ImGui. Algorithmic modules (VSA, LinABF) do not live here:
// App only owns their state and calls them from the UI.
class App {
public:
    bool init(int width, int height, const char* title);
    void run();
    void shutdown();

private:
    void drawUI();
    void drawMainMenu();
    void renderScene(int fbWidth, int fbHeight);
    void handleSceneInput();
    void openMesh(const std::string& path);
    void uploadMesh();
    void drawVsaPanel();
    void rebuildMoments();                 // face moments = base moments * weights; refit if partitioned
    void startJob(RefineJob::Kind kind);
    void jobTick();                        // one step of the running job (once per frame)
    void paintWeights(int fbW, int fbH);
    void drawPatchPanel();
    void drawLinabfPanel();
    void solvePatches(bool onlySelected);
    void computeUV(bool onlySelected);
    void drawUvPanel();
    void drawParamPanel();
    void runParam(bool onlySelected);
    void packAtlasNow();
    void drawRefinePanel();
    void extractPatchesNow();              // patches of vsa_.label (the Extract button)
    void drawExportDialogs();
    void bakeAtlasPreview();               // retexture: the source texture baked into the atlas layout
    void matchAtlasToOriginal();           // texture size and max mip level of the packer = what the original layout allows
    void measureLayouts();                 // original layout vs ours, and the quality of the retexturing
    std::vector<BakeChart> bakeCharts() const;
    const Patch& patchAt(size_t i) const;  // the patch to draw / solve: the parameterized (maybe cut) one, else the extracted one
    void requestSave();                    // File > Save: to the last path, or Save As if there is none
    bool performSave(const std::string& path);              // OBJ / mask PNG dialogs, drawn every frame
    void drawMakeAtlasPanel();
    // Huge meshes: the file is read straight into flat arrays and processed on a background thread (flat pipeline)
    struct HugeJob;
    void startHuge(const std::string& path);
    void startHugeFromLoaded();
    void runHugeJob(const std::shared_ptr<HugeJob>& job);
    void drawHugeButtons();
    void compareHuge();                    // original layout vs ours for the huge result (a thread; fills HugeJob::layout*)
    void stopRefine(bool wait);
    void drawRefineModal();
    void drawHugeAtlas();
    void saveBadPatches();
    void loadBadPatch(const std::string& path);
    void drawBadPatchDialog();
    struct Preset currentPreset() const;
    void applyPreset(const struct Preset& preset);
    bool saveHuge(const std::string& path);   // PLY with the atlas UV, and the baked texture next to it
    void showHugeResult();                 // the result of the huge mode in the 3D view (flat arrays -> GPU)
    void setHugeColors();
    float hugeCheckerCells() const;        // checker cells across the atlas uv, tied to the texel density of the atlas
    void drawHugeUI();                     // modal progress window, result summary, file dialogs
    void startMakeAtlas();
    void makeAtlasTick();                  // one step of the one-button pipeline (once per frame)
    void cancelMakeAtlas(const char* why);
    void startRefine();
    void refineTick();                     // one round of a running refinement (once per frame)
    void applyRefineResult();
    std::vector<Vec3> patchExplodeOffsets() const;   // per patch; empty if the view is not exploded
    void rebuildBoundaryLines();
    VSAInput vsaInput() const { return {mesh_, faceData_, faceMoments_}; }

    GLFWwindow* window_ = nullptr;
    bool imguiReady_ = false;

    // Context info, collected once
    std::string glVersion_, glRenderer_, glslVersion_;

    // Stage 0 UI state
    float clearColor_[3] = {0.16f, 0.17f, 0.20f};
    bool  showDemo_ = false;
    bool  showTriangle_ = true;
    bool  rotate_ = true;
    float angle_ = 0.0f;

    DebugTriangle triangle_;

    // Stage 1: mesh and its display
    TriMesh          mesh_;
    FaceData         faceData_;
    MeshDiagnostics  diag_;
    std::vector<Moments> faceMoments_;
    Moments          wholeMesh_;       // moments of all faces (stage 2 check)
    MeshRenderer     meshRenderer_;
    SceneFramebuffer scene_;
    int              msaaSamples_ = 4;          // 0 / 2 / 4 / 8, switchable at run time
    bool             cullBackFaces_ = false;    // on by default for big scans
    GLuint           gpuQuery_[2] = {0, 0};     // GL_TIME_ELAPSED around the mesh draw, double buffered
    int              gpuQueryFrame_ = 0;
    double           gpuDrawMs_ = 0.0;          // last measured GPU time of the mesh draw
    double           gpuDrawMsAvg_ = 0.0;
    ObjExtras        objExtras_;                // texture coordinates / materials of the loaded OBJ
    int              textureMaterial_ = -1;     // the material whose map is shown (-1: no texture)
    bool             hasSourceTexture_ = false;
    std::string      textureNote_;
    std::vector<unsigned char> sourcePixels_;   // the decoded source texture (RGBA, row 0 = v = 0), kept for baking
    int              sourceW_ = 0, sourceH_ = 0;
    std::string      meshPath_, savePath_;

    // Retexturing: the source texture baked into the atlas, and how well the new layout compares to the old one
    std::vector<unsigned char> bakedRgba_;      // resolution x resolution RGBA, row 0 = v = 0
    int              bakedResolution_ = 0;
    std::vector<float> bakedCornerUV_;           // the atlas as corner uv of the source faces (NaN: no uv)
    unsigned         atlasVersion_ = 0, bakedVersion_ = ~0u;
    int              textureOwner_ = 0;          // what the renderer holds: 0 nothing, 1 the source, 2 the baked atlas texture
    bool             layoutMeasured_ = false;
    UVLayoutStats    layoutOriginal_, layoutOurs_;
    MipSafetyResult  mipOriginal_, mipOurs_;
    RetextureReport  retexture_;
    bool             saveAsRequested_ = false;
    bool             binaryPly_ = true;
    Camera           camera_;
    std::string      lastDir_ = ".";
    std::string      status_;
    bool             wireframe_ = true;
    int              colorMode_ = 0;   // 0 - gray, 1 - face normal, 2 - VSA regions, 3 - weights, 4 - patches, 5 - patch kind, 6 - LinABF angle error, 10 - source texture,
                                       // 7 - stretch, 8 - UV checker, 9 - injectivity problems, 11 - baked atlas texture
    // Stage 3: VSA
    VSAState         vsa_;
    int              vsaK_ = 6;
    int              vsaRngSeed_ = 1;
    int              vsaMetric_ = 0;       // 0 - L2,1  1 - L2
    int              vsaAdded_ = 0;        // proxies added by the last partition for unseeded components
    int              vsaSteps_ = 0;
    LloydParams      lloydParams_;
    TeleportParams   teleportParams_;
    int              seedMethod_ = 1;      // 0 - random, 1 - farthest point
    std::unique_ptr<RefineJob> job_;       // running Lloyd / grow / teleport job
    std::string      lloydStatus_;

    // Border smoothing (applied after every partition of a Lloyd step) and raggedness of the borders
    SmoothParams     smoothParams_{0.1, 0.0, 10};
    NormalSmoothing  normalSmoothing_;      // smoothing of the face normal field before the VSA (0 iterations = off)
    void applyNormalSmoothing();            // recompute the face normals of the loaded mesh with normalSmoothing_, rebuild moments
    BoundaryStats    boundaryStats_;
    bool             boundaryStatsValid_ = false;

    // Stage 5: per-face weights (error density), painted with a brush
    std::vector<float>   faceWeights_;
    bool             paintMode_ = false;
    bool             eraseMode_ = false;
    int              brushRadius_ = 20;    // framebuffer pixels
    float            brushWeight_ = 4.0f;
    bool             painted_ = false;     // weights changed during the current stroke

    // Stage 6: patches
    ExtractionResult patches_;
    PatchOptions     patchOptions_;
    bool             patchesValid_ = false;
    std::vector<int> patchedLabels_;       // labels the patches were extracted from (to detect a stale result)
    int              selectedPatch_ = -1;
    float            explode_ = 0.0f;

    // Stage 7: LinABF angles per patch (parallel to patches_.patches; an entry is empty if not solved)
    LinABFParams     linabfParams_;
    std::vector<LinABFResult> linabf_;
    std::vector<char> linabfSolved_;
    double           linabfSeconds_ = 0.0;

    // Stage 9: parameterization ladder, distortion, atlas
    ParamOptions     paramOptions_;
    std::vector<PatchParam> params_;      // parallel to patches_.patches
    std::vector<char> paramDone_;
    double           paramSeconds_ = 0.0;
    AtlasOptions     atlasOptions_;        // shelf packer
    HorizonOptions   horizonOptions_;      // horizon packer
    int              atlasMethod_ = 0;     // 0 - horizon (texel based, mip aware), 1 - shelf
    AtlasResult      atlas_;              // parallel to params_
    bool             atlasValid_ = false;
    bool             uvShowAtlas_ = false;
    int              checkerSquares_ = 40;        // UV checker: size of a square = 1 / this of the model
    std::string      paramStatus_;

    // One-button pipeline: VSA (Lloyd) -> patches -> refinement (parameterization of every patch) -> atlas
    enum class AtlasStage { Idle, Lloyd, Refine };
    AtlasStage       atlasStage_ = AtlasStage::Idle;
    std::unique_ptr<RefineJob> atlasJob_;
    bool             makeAtlasTeleport_ = false;
    int              makeAtlasIter_ = 0;
    std::string      makeAtlasStatus_;
    bool             exportObjRequested_ = false, exportMaskRequested_ = false;

    // "Make atlas (huge)"
    std::shared_ptr<HugeJob> hugeJob_;
    bool             hugeOpenRequested_ = false, hugeSaveRequested_ = false;
    bool             loadBadRequested_ = false;
    bool             hugeFromLoaded_ = false;     // the shown result comes from the loaded TriMesh, not from a file
    // Presentation: the patches in colors, then the same patches translucent and moved along their normals
    bool             presentation_ = false;
    float            presentOffset_ = 0.06f, presentAlpha_ = 0.4f;
    int              presentFirst_ = 0, presentSecond_ = 1;   // what each pass shows: 0 patch colors, 1 texture, 2 plain shading
    bool             presentAuto_ = true;                      // until the user chooses: textured mesh = colors + texture, else shading + colors
    int              presentPass(int which) const;             // 0 first, 1 second: what the pass shows now
    int              uvOverride_ = -1;                        // 10 / 11: the geometry carries the uv of the source / the atlas whatever the color mode
    bool             presentTextureReady() const;             // is there a texture to show (and its uv in the geometry)
    void             updatePresentationNormals();   // the normals of the patches for the second pass
    void             uploadMeshImpl();
    bool             hugeShown_ = false;          // the 3D view shows the huge result, not mesh_
    int              hugeCheckerSquares_ = 40;    // size of a checker square: 1 / this of the model
    int              hugeView_ = 0;               // 0 patches, 1 uv checker, 2 plain
    double           hugeSimplifyPercent_ = 10.0; // size of the simplified mesh, % of the faces (10 = 10 times fewer); 0 = hugeCoarseFaces_
    int              hugeCoarseFaces_ = 300000;   // size of the simplified mesh when the percent is 0 (testing hook)
    int              hugeK_ = 1000;               // patches to start from on the simplified mesh
    std::string      hugeStatus_;

    // Stage 10: refinement loop
    RefineOptions    refineOptions_;
    std::unique_ptr<Refiner> refiner_;
    std::thread      refineThread_;                // one round of refiner_->step() at a time
    std::atomic<bool> refineBusy_{false}, refineStop_{false}, refineMore_{false};
    int              refineRounds_ = 0;
    RefineStats      refineStats_;
    bool             refineHasStats_ = false;
    bool             patchesRefined_ = false;  // the patches come from the refinement, not from the VSA labels

    // Stage 8: UV per patch (parallel to patches_.patches)
    std::vector<UVResult> uv_;
    std::vector<UVStats>  uvStats_;
    std::vector<char>     uvDone_;
    bool             uvGreedy_ = false;       // debug: depth-first reconstruction instead of least squares
    bool             uvShowEdges_ = true;
    float            uvZoom_ = 1.0f;
    ImVec2           uvPan_{0.0f, 0.0f};
    bool             showBoundaries_ = true;
    float            boundaryWidth_ = 3.0f;   // pixels

    bool             dragging_ = false;
    bool             panning_ = false;
};
