// huge-atlas, the native shell (a prototype): an AppKit window with a sidebar of settings, a toolbar, a menu bar and a 3D view. The
// work is done by AtlasSession (the same library as the ImGui build); the rules of the Human Interface Guidelines for the Mac that
// this window follows are in docs/native-ui.md.
#import <Cocoa/Cocoa.h>
#import <OpenGL/OpenGL.h>
#import <QuartzCore/QuartzCore.h>

#include "render/GL.h"
#include "render/SceneFramebuffer.h"
#include "param/AtlasChecker.h"
#include "render/Camera.h"
#include "render/MeshRenderer.h"
#include "render/GridRenderer.h"
#include "app/BadPatch.h"
#include "app/NativeUI.h"
#include "app/Paths.h"
#include "native/HugeSession.h"
#import "native/ChatPanel.h"
#import "native/DeveloperPanel.h"
#import "native/VsaLab.h"
#import "native/Settings.h"
#include "native/Session.h"
#include "chat/Agent.h"
#include "chat/Provider.h"
#include "chat/Tools.h"

#include <array>
#include <atomic>
#include <memory>
#include <thread>

// ------------------------------------------------------------------------------------------------------------------ the 3D view

@interface SceneView : NSOpenGLView {
    MeshRenderer _renderer;
    GridRenderer _grid;
    Camera _camera;
    BOOL _ready;
    int _mode;              // 0 patches, 1 plain shading, 2 presentation, 3 texture
    BOOL _hasMesh;
    BOOL _showGrid;         // the coordinate grid, under the mesh when there is one
    float _floorY;          // where the floor of the grid is: the lowest point of the mesh
    BOOL _hasTexture;
    SceneFramebuffer _fbo;     // the multisample target (the developer mode)
    int _msaa;
    float _checkerCells;       // > 0: the UV checker with this many squares along the atlas
    GLuint _gpuQuery[2];
    int _gpuFrame;
    BOOL _wire;             // the developer mode: the wire of the triangles over the mesh
    BOOL _cull;             // big meshes: the back faces are not drawn
    float _offset, _alpha;  // presentation: how far the translucent copies of the patches move, how opaque they are
    int _pass1, _pass2;     // presentation: what a pass shows (0 patch colors, 1 texture, 2 shading); -1 = decided by the mesh
    NSPoint _last;
    NSPoint _downPoint;     // where the left button went down: a click is a release that did not move
    BOOL _dragged;
    NSTimer* _flyTimer;
    Camera::State _flyFrom, _flyTo;
    CFTimeInterval _flyStart;
}
@property (nonatomic, copy) void (^onDropURL)(NSURL*);
@property (nonatomic, copy) void (^onGpuTime)(double ms);
// The brush of the Segmentation Lab: while it is on, the left button paints (the faces under the disc go to onBrush, the end of the stroke to onBrushEnd); ⌥ turns.
@property (nonatomic) BOOL brushActive;
@property (nonatomic) int brushRadius;
@property (nonatomic, copy) void (^onBrush)(const std::vector<int>& faces);
@property (nonatomic, copy) void (^onBrushEnd)(void);
// A click (no drag) on the model gives the face under the cursor, -1 for the empty space or Esc (the patch of the face is chosen).
@property (nonatomic, copy) void (^onPickFace)(int face);
- (int)faceAtViewPoint:(NSPoint)p;
// The camera goes to the ray out of the surface at `centre` along `normal` and looks back at the centre, far enough for the points to fit
// the view; the world up stays up on the screen (the view is not rolled). Animated, and any mouse action in the view stops the flight.
- (void)lookAtCentre:(const double*)centre normal:(const double*)normal points:(const std::vector<float>&)points animated:(BOOL)animated;   // p in the coordinates of the view (origin bottom left)
- (void)setMsaa:(int)samples;
- (void)setCull:(BOOL)on;
- (BOOL)cull;
- (void)setCheckerCells:(float)cells;
- (void)showMesh:(const TriMesh&)mesh colors:(const std::vector<float>&)colors faceUV:(const std::vector<float>&)uv;
- (void)showFlat:(const FlatMesh&)mesh cornerUV:(const std::vector<float>&)uv rgba:(const std::vector<unsigned char>&)rgba;
- (void)clearMesh;
- (void)setShowGrid:(BOOL)on;
- (void)setFaceColors:(const std::vector<float>&)rgb;
- (void)setFaceRgba:(const std::vector<unsigned char>&)rgba;
- (BOOL)showGrid;
- (void)setTexture:(int)side pixels:(const unsigned char*)pixels;
- (void)setTextureWidth:(int)w height:(int)h pixels:(const unsigned char*)pixels;
- (void)setPatchNormals:(const std::vector<float>&)normals;
- (void)setMode:(int)mode;
- (void)setOffset:(float)offset alpha:(float)alpha;
- (void)setPass1:(int)p1 pass2:(int)p2;
- (void)resetCamera;
- (void)setWireframe:(BOOL)on;
@end

@implementation SceneView

- (instancetype)initWithFrame:(NSRect)frame
{
    NSOpenGLPixelFormatAttribute attrs[] = {
        NSOpenGLPFAOpenGLProfile, NSOpenGLProfileVersion4_1Core,
        NSOpenGLPFADoubleBuffer, NSOpenGLPFAColorSize, 24, NSOpenGLPFADepthSize, 24,
        NSOpenGLPFAMultisample, NSOpenGLPFASampleBuffers, 1, NSOpenGLPFASamples, 4,
        0};
    NSOpenGLPixelFormat* pf = [[NSOpenGLPixelFormat alloc] initWithAttributes:attrs];
    self = [super initWithFrame:frame pixelFormat:pf];
    if (self) {
        self.wantsBestResolutionOpenGLSurface = YES;
        [self registerForDraggedTypes:@[NSPasteboardTypeFileURL]];
        _offset = 0.06f; _alpha = 0.4f; _pass1 = _pass2 = -1;
        self.accessibilityLabel = @"3D view of the mesh";
        self.accessibilityRole = NSAccessibilityGroupRole;
    }
    return self;
}

// The renderer is made on first use: a mesh can be opened before the view has been drawn for the first time.
- (void)ensureReady
{
    [self.openGLContext makeCurrentContext];
    if (_ready) return;
    GLint swap = 1;
    [self.openGLContext setValues:&swap forParameter:NSOpenGLContextParameterSwapInterval];
    std::string log;
    _ready = _renderer.init(&log);
    if (!_ready) NSLog(@"MeshRenderer: %s", log.c_str());
    if (!_grid.init(&log)) NSLog(@"GridRenderer: %s", log.c_str());
    _camera.resetAbove();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
}

- (void)prepareOpenGL
{
    [super prepareOpenGL];
    [self ensureReady];
}

- (BOOL)acceptsFirstResponder { return YES; }
- (BOOL)isOpaque { return YES; }

- (void)viewDidChangeEffectiveAppearance { [self setNeedsDisplay:YES]; }

- (void)showMesh:(const TriMesh&)mesh colors:(const std::vector<float>&)colors faceUV:(const std::vector<float>&)uv
{
    [self ensureReady];
    _renderer.invalidateGeometry();
    _renderer.upload(mesh, colors, {}, uv);
    float lo = 0.0f;
    bool first = true;
    for (auto v : mesh.vertices()) { const float y = float(mesh.point(v)[1]); if (first || y < lo) lo = y; first = false; }
    _floorY = lo;
    if (!_hasMesh) _camera.reset();   // from the grid view to the model
    _hasMesh = YES;
    _cull = NO;
    [self setNeedsDisplay:YES];
}

- (void)showFlat:(const FlatMesh&)mesh cornerUV:(const std::vector<float>&)uv rgba:(const std::vector<unsigned char>&)rgba
{
    [self ensureReady];
    _renderer.uploadFlat(mesh, uv, rgba);
    float lo = 0.0f;
    for (size_t i = 1; i < mesh.pos.size(); i += 3) lo = (i == 1 || mesh.pos[i] < lo) ? mesh.pos[i] : lo;
    _floorY = lo;
    if (!_hasMesh) _camera.reset();
    _hasMesh = YES;
    _cull = mesh.faceCount() > 500000;   // as in the other build: a scan is drawn without its back faces
    [self setNeedsDisplay:YES];
}

- (void)setFaceColors:(const std::vector<float>&)rgb
{
    [self ensureReady];
    _renderer.setFaceColors(rgb);
    [self setNeedsDisplay:YES];
}

- (void)setFaceRgba:(const std::vector<unsigned char>&)rgba
{
    [self ensureReady];
    _renderer.setFaceColorsRgba(rgba);
    [self setNeedsDisplay:YES];
}

- (void)setMsaa:(int)samples { _msaa = samples; [self setNeedsDisplay:YES]; }
- (void)setCull:(BOOL)on { _cull = on; [self setNeedsDisplay:YES]; }
- (BOOL)cull { return _cull; }
- (void)setCheckerCells:(float)cells { _checkerCells = cells; [self setNeedsDisplay:YES]; }

- (BOOL)showGrid { return _showGrid; }
- (void)setShowGrid:(BOOL)on { _showGrid = on; [self setNeedsDisplay:YES]; }

- (void)clearMesh { _floorY = 0; _hasMesh = NO; _hasTexture = NO; _camera.resetAbove(); [self setNeedsDisplay:YES]; }

- (void)setTexture:(int)side pixels:(const unsigned char*)pixels { [self setTextureWidth:side height:side pixels:pixels]; }

- (void)setTextureWidth:(int)w height:(int)h pixels:(const unsigned char*)pixels
{
    [self ensureReady];
    _hasTexture = w > 0 && h > 0 && pixels && _renderer.setTexture(w, h, pixels);
    [self setNeedsDisplay:YES];
}

- (void)setPatchNormals:(const std::vector<float>&)normals
{
    [self ensureReady];
    _renderer.setPatchNormals(normals);
}

- (void)setMode:(int)mode { _mode = mode; [self setNeedsDisplay:YES]; }
- (void)setOffset:(float)offset alpha:(float)alpha { _offset = offset; _alpha = alpha; [self setNeedsDisplay:YES]; }
- (void)setPass1:(int)p1 pass2:(int)p2 { _pass1 = p1; _pass2 = p2; [self setNeedsDisplay:YES]; }
- (void)setWireframe:(BOOL)on { _wire = on; [self setNeedsDisplay:YES]; }
- (void)resetCamera { if (_hasMesh) _camera.reset(); else _camera.resetAbove(); [self setNeedsDisplay:YES]; }

- (void)drawRect:(NSRect)dirty
{
    [self ensureReady];
    const NSRect b = [self convertRectToBacking:self.bounds];
    _fbo.begin(int(b.size.width), int(b.size.height), _msaa);   // the default framebuffer when there is no multisampling
    glViewport(0, 0, GLsizei(b.size.width), GLsizei(b.size.height));
    glEnable(GL_DEPTH_TEST);
    __block float r = 0.12f, g = 0.12f, bl = 0.13f;
    [self.effectiveAppearance performAsCurrentDrawingAppearance:^{   // the color of the window follows light / dark
        NSColor* c = [[NSColor underPageBackgroundColor] colorUsingColorSpace:NSColorSpace.sRGBColorSpace];
        if (c) { r = float(c.redComponent); g = float(c.greenComponent); bl = float(c.blueComponent); }
    }];
    glClearColor(r, g, bl, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (_ready && _showGrid && b.size.height > 0) {
        float vp[16];
        _camera.viewProj(float(b.size.width / b.size.height), vp);
        for (int i = 0; i < 4; ++i) vp[12 + i] += _floorY * vp[4 + i];   // the floor moves to the bottom of the mesh
        _grid.draw(vp, 0.2126f * r + 0.7152f * g + 0.0722f * bl < 0.5f);
    }
    if (_ready && _hasMesh && b.size.height > 0) {
        float vp[16], v[16];
        _camera.viewProj(float(b.size.width / b.size.height), vp);
        _camera.view(v);
        const BOOL tex = _hasTexture;
        // the GPU time of the mesh, read one frame late so that nothing stalls
        if (!_gpuQuery[0]) glGenQueries(2, _gpuQuery);
        const int cur = _gpuFrame & 1, prev = cur ^ 1;
        if (_gpuFrame > 1 && _onGpuTime) {
            GLint available = 0;
            glGetQueryObjectiv(_gpuQuery[prev], GL_QUERY_RESULT_AVAILABLE, &available);
            if (available) {
                GLuint64 ns = 0;
                glGetQueryObjectui64v(_gpuQuery[prev], GL_QUERY_RESULT, &ns);
                const double ms = double(ns) * 1e-6;
                dispatch_async(dispatch_get_main_queue(), ^{ if (self->_onGpuTime) self->_onGpuTime(ms); });
            }
        }
        glBeginQuery(GL_TIME_ELAPSED, _gpuQuery[cur]);
        ++_gpuFrame;
        if (_mode == 2 && _renderer.hasPatchNormals()) {
            // presentation: the patches as they are, then the same patches translucent and moved along their normals. The defaults:
            // a textured mesh is colors + texture, a mesh without a texture is shading + colors
            auto fix = [&](int p) { return (p == 1 && !tex) ? 0 : p; };
            const int p1 = fix(_pass1 >= 0 ? _pass1 : (tex ? 0 : 2)), p2 = fix(_pass2 >= 0 ? _pass2 : (tex ? 1 : 0));
            const BOOL skipFirst = _offset <= 1e-6f, secondFirst = _alpha >= 0.999f;   // offset 0: one pass; opaque: the second goes first
            auto first = [&] { _renderer.draw(vp, v, false, 0.0f, p1 == 1, _cull, p1 == 2); };
            if (!skipFirst && !secondFirst) first();
            _renderer.drawGhosts(vp, v, _offset, _alpha, _cull, p2 == 1, p2 == 2);
            if (!skipFirst && secondFirst) first();
        } else {
            _renderer.draw(vp, v, _wire && !_cull, _checkerCells, _mode == 3 && tex && _checkerCells <= 0.0f, _cull, _mode == 1);
        }
        glEndQuery(GL_TIME_ELAPSED);
    }
    _fbo.end();
    [self.openGLContext flushBuffer];
}

// ---- mouse: the left button turns, the right or middle one moves, the wheel and the pinch zoom towards the cursor
- (NSPoint)local:(NSEvent*)e
{
    const NSPoint p = [self convertPoint:e.locationInWindow fromView:nil];
    return NSMakePoint(p.x, self.bounds.size.height - p.y);   // y down, as the camera expects
}

- (void)paintWith:(NSEvent*)e
{
    [self.openGLContext makeCurrentContext];
    const NSPoint bp = [self convertPointToBacking:[self convertPoint:e.locationInWindow fromView:nil]];   // origin bottom left
    const NSRect bb = [self convertRectToBacking:self.bounds];
    if (bb.size.height <= 0) return;
    float vp[16];
    _camera.viewProj(float(bb.size.width / bb.size.height), vp);
    std::vector<int> ids;
    _renderer.pickFaces(vp, int(bb.size.width), int(bb.size.height), int(bp.x), int(bp.y), std::max(2, _brushRadius), ids);
    if (!ids.empty() && _onBrush) _onBrush(ids);
}

- (void)stopFlight { [_flyTimer invalidate]; _flyTimer = nil; }

- (void)lookAtCentre:(const double*)c normal:(const double*)n points:(const std::vector<float>&)pts animated:(BOOL)animated
{
    const NSRect b = self.bounds;
    const double aspect = b.size.height > 0 ? b.size.width / b.size.height : 1.0;
    const double up[3] = {0, 1, 0};
    double cx[3], cy[3], cz[3];
    _camera.axes(cx, cy, cz);
    // the axes of the new view first (they do not depend on the distance), then the distance that fits the patch
    Camera probe;
    probe.lookAt(c, n, 1.0, up, cy);
    double x[3], y[3], z[3];
    probe.axes(x, y, z);
    double halfW = 0.0, halfH = 0.0, toward = 0.0;   // the extent across the view, and how far the patch comes out towards the camera
    for (size_t i = 0; i + 2 < pts.size(); i += 3) {
        const double d[3] = {pts[i] - c[0], pts[i + 1] - c[1], pts[i + 2] - c[2]};
        halfW = std::max(halfW, std::abs(d[0] * x[0] + d[1] * x[1] + d[2] * x[2]));
        halfH = std::max(halfH, std::abs(d[0] * y[0] + d[1] * y[1] + d[2] * y[2]));
        toward = std::max(toward, d[0] * z[0] + d[1] * z[1] + d[2] * z[2]);
    }
    const double t = Camera::halfFieldTan(aspect);
    double distance = std::max(halfH * 1.15 / t, halfW * 1.15 / (t * aspect)) + toward;   // a margin of 15%; the nearest part of the patch must fit too
    distance = std::max(std::max(0.35, toward + 0.12), std::min(12.0, distance));       // not closer than the near plane allows, not lost in the distance
    if (getenv("HUGE_NATIVE_VERBOSE")) printf("LOOKAT: centre %.3f %.3f %.3f normal %.3f %.3f %.3f halfW %.3f halfH %.3f toward %.3f aspect %.2f t %.3f distance %.3f points %zu\n", c[0], c[1], c[2], n[0], n[1], n[2], halfW, halfH, toward, aspect, t, distance, pts.size() / 3);
    const Camera::State target = Camera::lookAtState(c, n, distance, up, cy);
    [self stopFlight];
    if (!animated) {
        _camera.setState(target);
        if (getenv("HUGE_NATIVE_VERBOSE")) {   // where the points of the patch fall on the screen: all of them must be inside [-1, 1]
            float vp[16];
            _camera.viewProj(float(aspect), vp);
            double mx = 0, my = 0;
            for (size_t i = 0; i + 2 < pts.size(); i += 3) {
                const double w = vp[3] * pts[i] + vp[7] * pts[i + 1] + vp[11] * pts[i + 2] + vp[15];
                mx = std::max(mx, std::abs((vp[0] * pts[i] + vp[4] * pts[i + 1] + vp[8] * pts[i + 2] + vp[12]) / w));
                my = std::max(my, std::abs((vp[1] * pts[i] + vp[5] * pts[i + 1] + vp[9] * pts[i + 2] + vp[13]) / w));
            }
            printf("LOOKAT: the points fall within x +-%.2f, y +-%.2f of the screen\n", mx, my);
        }
        [self setNeedsDisplay:YES];
        return;
    }
    _flyFrom = _camera.state();
    _flyTo = target;
    _flyStart = CACurrentMediaTime();
    __weak SceneView* weak = self;
    _flyTimer = [NSTimer scheduledTimerWithTimeInterval:1.0 / 60.0 repeats:YES block:^(NSTimer* timer) {
        SceneView* me = weak;
        if (!me) { [timer invalidate]; return; }
        double s = (CACurrentMediaTime() - me->_flyStart) / 0.35;
        const BOOL last = s >= 1.0;
        s = std::min(1.0, s);
        s = s * s * (3.0 - 2.0 * s);   // ease in and out
        me->_camera.setState(Camera::blend(me->_flyFrom, me->_flyTo, s));
        [me setNeedsDisplay:YES];
        if (last) [me stopFlight];
    }];
}

- (int)faceAtViewPoint:(NSPoint)viewPoint
{
    if (!_hasMesh) return -1;
    [self.openGLContext makeCurrentContext];
    const NSPoint bp = [self convertPointToBacking:viewPoint];
    const NSRect bb = [self convertRectToBacking:self.bounds];
    if (bb.size.height <= 0) return -1;
    float vp[16];
    _camera.viewProj(float(bb.size.width / bb.size.height), vp);
    std::vector<int> ids;
    _renderer.pickFaces(vp, int(bb.size.width), int(bb.size.height), int(bp.x), int(bp.y), 1, ids);
    return ids.empty() ? -1 : ids.front();
}

- (void)mouseUp:(NSEvent*)e
{
    if (_brushActive) { if (_onBrushEnd) _onBrushEnd(); return; }
    if (!_dragged && e.clickCount < 2 && _onPickFace) _onPickFace([self faceAtViewPoint:[self convertPoint:e.locationInWindow fromView:nil]]);
}

- (void)keyDown:(NSEvent*)e
{
    if (e.keyCode == 53 && _onPickFace) { _onPickFace(-1); return; }   // Esc: no patch chosen
    [super keyDown:e];
}

- (void)resetCursorRects { if (_brushActive) [self addCursorRect:self.bounds cursor:NSCursor.crosshairCursor]; }
- (void)setBrushActive:(BOOL)on { _brushActive = on; [self.window invalidateCursorRectsForView:self]; }

- (void)mouseDown:(NSEvent*)e
{
    [self stopFlight];
    [self.window makeFirstResponder:self];
    if (_brushActive && !(e.modifierFlags & NSEventModifierFlagOption)) { [self paintWith:e]; return; }
    _dragged = NO;
    _downPoint = [self convertPoint:e.locationInWindow fromView:nil];
    if (e.clickCount == 2) { [self resetCamera]; return; }
    const NSPoint p = [self local:e];
    _last = p;
    _camera.beginRotate(p.x, p.y, int(self.bounds.size.width), int(self.bounds.size.height));
}

- (void)mouseDragged:(NSEvent*)e
{
    if (_brushActive && !(e.modifierFlags & NSEventModifierFlagOption)) { [self paintWith:e]; return; }
    {
        const NSPoint q = [self convertPoint:e.locationInWindow fromView:nil];
        if (std::hypot(q.x - _downPoint.x, q.y - _downPoint.y) > 3.0) _dragged = YES;
    }
    const NSPoint p = [self local:e];
    _camera.rotateTo(p.x, p.y, int(self.bounds.size.width), int(self.bounds.size.height));
    [self setNeedsDisplay:YES];
}

- (void)panWith:(NSEvent*)e
{
    const NSPoint p = [self local:e];
    const double w = self.bounds.size.width, h = self.bounds.size.height;
    _camera.pan(p.x - _last.x, p.y - _last.y, int(h), w / h);
    _last = p;
    [self setNeedsDisplay:YES];
}
- (void)rightMouseDown:(NSEvent*)e { _last = [self local:e]; }
- (void)rightMouseDragged:(NSEvent*)e { [self panWith:e]; }
- (void)otherMouseDown:(NSEvent*)e { _last = [self local:e]; }
- (void)otherMouseDragged:(NSEvent*)e { [self panWith:e]; }

- (void)zoom:(double)amount at:(NSEvent*)e
{
    const NSPoint p = [self local:e];
    const double w = self.bounds.size.width, h = self.bounds.size.height;
    _camera.zoomAt(amount, 2.0 * p.x / w - 1.0, 1.0 - 2.0 * p.y / h, w / h);
    [self setNeedsDisplay:YES];
}
- (void)scrollWheel:(NSEvent*)e { [self stopFlight]; [self zoom:(e.hasPreciseScrollingDeltas ? e.scrollingDeltaY * 0.1 : e.scrollingDeltaY) at:e]; }
- (void)magnifyWithEvent:(NSEvent*)e { [self zoom:e.magnification * 8.0 at:e]; }

- (NSMenu*)menuForEvent:(NSEvent*)e
{
    NSMenu* m = [[NSMenu alloc] initWithTitle:@""];
    [m addItemWithTitle:@"Reset Camera" action:@selector(resetCamera:) keyEquivalent:@""];
    [m addItem:[NSMenuItem separatorItem]];
    [m addItemWithTitle:@"Show Patches" action:@selector(showPatches:) keyEquivalent:@""];
    [m addItemWithTitle:@"Show Plain Shading" action:@selector(showPlain:) keyEquivalent:@""];
    [m addItemWithTitle:@"Show Presentation" action:@selector(showPresentation:) keyEquivalent:@""];
    [m addItemWithTitle:@"Show Texture" action:@selector(showTexture:) keyEquivalent:@""];
    return m;
}

// ---- files dropped on the view
- (NSDragOperation)draggingEntered:(id<NSDraggingInfo>)sender { return NSDragOperationCopy; }
- (BOOL)performDragOperation:(id<NSDraggingInfo>)sender
{
    NSArray<NSURL*>* urls = [sender.draggingPasteboard readObjectsForClasses:@[NSURL.class]
                                                                      options:@{NSPasteboardURLReadingFileURLsOnlyKey: @YES}];
    if (urls.count == 0 || !self.onDropURL) return NO;
    self.onDropURL(urls.firstObject);
    return YES;
}

@end

// ------------------------------------------------------------------------------------------------------------------ the UV view

// The atlas as a picture: the square of the texture, the baked texture in it (or the charts filled with the color of their patch),
// the border of every chart. The wheel zooms towards the cursor, a drag moves, a double click fits the square to the view.
@interface UvView : NSView {
    std::vector<UvChart> _charts;
    CGImageRef _image;
    double _zoom;
    NSPoint _pan;
    NSPoint _last;
    NSPoint _downPoint;
    BOOL _dragged;
}
@property (nonatomic) NSInteger selectedPatch;   // its chart is drawn heavier (-1: none)
@property (nonatomic, copy) void (^onPickPatch)(NSInteger patch);   // a click on a chart; -1 for the empty space or Esc
- (NSInteger)patchAtViewPoint:(NSPoint)p;
- (void)setCharts:(std::vector<UvChart>)charts texture:(const unsigned char*)rgba side:(int)side;
- (NSRect)square;
- (void)clear;
@end

@implementation UvView

- (instancetype)initWithFrame:(NSRect)frame
{
    self = [super initWithFrame:frame];
    if (self) {
        _zoom = 1.0;
        self.accessibilityLabel = @"UV layout of the atlas";
        self.accessibilityRole = NSAccessibilityGroupRole;
    }
    return self;
}

- (void)dealloc { if (_image) CGImageRelease(_image); }

- (void)clear
{
    _charts.clear();
    if (_image) { CGImageRelease(_image); _image = nullptr; }
    [self setNeedsDisplay:YES];
}

- (void)setCharts:(std::vector<UvChart>)charts texture:(const unsigned char*)rgba side:(int)side
{
    _charts = std::move(charts);
    if (_image) { CGImageRelease(_image); _image = nullptr; }
    if (rgba && side > 0) {
        CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
        NSData* data = [NSData dataWithBytes:rgba length:size_t(side) * side * 4];   // a copy: the session may change
        CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
        _image = CGImageCreate(side, side, 8, 32, size_t(side) * 4, cs, kCGImageAlphaLast | kCGBitmapByteOrderDefault, provider, nullptr, true,
                               kCGRenderingIntentDefault);
        CGDataProviderRelease(provider);
        CGColorSpaceRelease(cs);
    }
    _zoom = 1.0;
    _pan = NSZeroPoint;
    [self setNeedsDisplay:YES];
}

- (NSRect)square
{
    const double s = std::max(32.0, std::min(self.bounds.size.width, self.bounds.size.height) - 32.0) * _zoom;
    return NSMakeRect(self.bounds.size.width / 2 - s / 2 + _pan.x, self.bounds.size.height / 2 - s / 2 + _pan.y, s, s);
}

- (void)drawRect:(NSRect)dirty
{
    [NSColor.underPageBackgroundColor setFill];   // the pane is gray, the square of the atlas stands out from it
    NSRectFill(dirty);
    const NSRect sq = [self square];
    CGContextRef g = NSGraphicsContext.currentContext.CGContext;
    CGContextSaveGState(g);
    CGContextClipToRect(g, NSRectToCGRect(NSIntersectionRect(sq, self.bounds)));
    [NSColor.controlBackgroundColor setFill];
    NSRectFill(sq);
    if (_image) {
        CGContextSaveGState(g);   // the first row of the data is v = 0, the bottom of the square
        CGContextTranslateCTM(g, sq.origin.x, sq.origin.y + sq.size.height);
        CGContextScaleCTM(g, 1.0, -1.0);
        CGContextDrawImage(g, CGRectMake(0, 0, sq.size.width, sq.size.height), _image);
        CGContextRestoreGState(g);
    }
    for (const UvChart& c : _charts) {
        float rgb[3];
        AtlasSession::patchColor(c.patch, rgb);
        auto pt = [&](unsigned v) { return CGPointMake(sq.origin.x + c.uv[2 * v] * sq.size.width, sq.origin.y + c.uv[2 * v + 1] * sq.size.height); };
        CGMutablePathRef fill = CGPathCreateMutable();
        for (size_t t = 0; t + 2 < c.triangles.size(); t += 3) {
            CGPathMoveToPoint(fill, nullptr, pt(c.triangles[t]).x, pt(c.triangles[t]).y);
            CGPathAddLineToPoint(fill, nullptr, pt(c.triangles[t + 1]).x, pt(c.triangles[t + 1]).y);
            CGPathAddLineToPoint(fill, nullptr, pt(c.triangles[t + 2]).x, pt(c.triangles[t + 2]).y);
            CGPathCloseSubpath(fill);
        }
        const BOOL chosen = _selectedPatch >= 0 && c.patch == _selectedPatch;
        CGContextSetRGBFillColor(g, rgb[0], rgb[1], rgb[2], chosen ? 0.9 : (_image ? 0.10 : 0.55));
        CGContextAddPath(g, fill);
        CGContextFillPath(g);
        CGPathRelease(fill);
        if (!c.flipped.empty()) {   // the faces whose uv are flipped
            CGMutablePathRef bad = CGPathCreateMutable();
            for (unsigned f : c.flipped) {
                if (size_t(f) * 3 + 2 >= c.triangles.size()) continue;
                CGPathMoveToPoint(bad, nullptr, pt(c.triangles[3 * f]).x, pt(c.triangles[3 * f]).y);
                CGPathAddLineToPoint(bad, nullptr, pt(c.triangles[3 * f + 1]).x, pt(c.triangles[3 * f + 1]).y);
                CGPathAddLineToPoint(bad, nullptr, pt(c.triangles[3 * f + 2]).x, pt(c.triangles[3 * f + 2]).y);
                CGPathCloseSubpath(bad);
            }
            CGContextSetRGBFillColor(g, 1.0, 0.1, 0.1, 0.8);
            CGContextAddPath(g, bad);
            CGContextFillPath(g);
            CGPathRelease(bad);
        }
        CGContextSetRGBStrokeColor(g, rgb[0], rgb[1], rgb[2], 0.95);
        CGContextSetLineWidth(g, chosen ? 3.5 : 1.5);
        for (size_t i = 0; i + 1 < c.segments.size(); i += 2) {   // the huge mode: only the border edges
            CGContextMoveToPoint(g, pt(c.segments[i]).x, pt(c.segments[i]).y);
            CGContextAddLineToPoint(g, pt(c.segments[i + 1]).x, pt(c.segments[i + 1]).y);
        }
        CGContextStrokePath(g);
        for (const auto& loop : c.loops) {
            if (loop.size() < 2) continue;
            CGContextMoveToPoint(g, pt(loop[0]).x, pt(loop[0]).y);
            for (size_t i = 1; i < loop.size(); ++i) CGContextAddLineToPoint(g, pt(loop[i]).x, pt(loop[i]).y);
            CGContextClosePath(g);
            CGContextStrokePath(g);
        }
        if (!c.crossings.empty()) {   // the border edges that cross another one
            CGContextSetRGBStrokeColor(g, 0.95, 0.1, 0.9, 1.0);
            CGContextSetLineWidth(g, 3.0);
            for (size_t i = 0; i + 1 < c.crossings.size(); i += 2) {
                CGContextMoveToPoint(g, pt(c.crossings[i]).x, pt(c.crossings[i]).y);
                CGContextAddLineToPoint(g, pt(c.crossings[i + 1]).x, pt(c.crossings[i + 1]).y);
            }
            CGContextStrokePath(g);
        }
    }
    CGContextRestoreGState(g);
    [[NSColor.labelColor colorWithAlphaComponent:0.35] setStroke];
    NSFrameRect(sq);
}

- (BOOL)acceptsFirstResponder { return YES; }
// the chart under a point of the view: its triangles when the chart has them, else its border (the huge mode) by the even-odd rule
- (NSInteger)patchAtViewPoint:(NSPoint)p
{
    const NSRect sq = [self square];
    if (sq.size.width <= 0 || !NSPointInRect(p, sq)) return -1;
    const double u = (p.x - sq.origin.x) / sq.size.width, v = (p.y - sq.origin.y) / sq.size.height;
    for (size_t ci = _charts.size(); ci-- > 0;) {
        const UvChart& c = _charts[ci];
        auto at = [&](unsigned i, double* xy) { xy[0] = c.uv[2 * i]; xy[1] = c.uv[2 * i + 1]; };
        if (!c.triangles.empty()) {
            for (size_t t = 0; t + 2 < c.triangles.size(); t += 3) {
                double a[2], b[2], d[2];
                at(c.triangles[t], a); at(c.triangles[t + 1], b); at(c.triangles[t + 2], d);
                const double d1 = (u - b[0]) * (a[1] - b[1]) - (a[0] - b[0]) * (v - b[1]);
                const double d2 = (u - d[0]) * (b[1] - d[1]) - (b[0] - d[0]) * (v - d[1]);
                const double d3 = (u - a[0]) * (d[1] - a[1]) - (d[0] - a[0]) * (v - a[1]);
                const bool neg = d1 < 0 || d2 < 0 || d3 < 0, pos = d1 > 0 || d2 > 0 || d3 > 0;
                if (!(neg && pos)) return c.patch;
            }
        } else if (!c.segments.empty()) {
            int crossings = 0;
            for (size_t i = 0; i + 1 < c.segments.size(); i += 2) {
                double a[2], b[2];
                at(c.segments[i], a); at(c.segments[i + 1], b);
                if ((a[1] > v) != (b[1] > v) && u < a[0] + (v - a[1]) * (b[0] - a[0]) / (b[1] - a[1])) ++crossings;
            }
            if (crossings & 1) return c.patch;
        }
    }
    return -1;
}

- (void)keyDown:(NSEvent*)e { if (e.keyCode == 53 && _onPickPatch) _onPickPatch(-1); else [super keyDown:e]; }

- (void)mouseDown:(NSEvent*)e
{
    [self.window makeFirstResponder:self];
    _dragged = NO;
    _downPoint = [self convertPoint:e.locationInWindow fromView:nil];
    if (e.clickCount == 2) { _zoom = 1.0; _pan = NSZeroPoint; [self setNeedsDisplay:YES]; return; }
    _last = _downPoint;
}
- (void)mouseUp:(NSEvent*)e
{
    if (!_dragged && e.clickCount < 2 && _onPickPatch) _onPickPatch([self patchAtViewPoint:[self convertPoint:e.locationInWindow fromView:nil]]);
}
- (void)mouseDragged:(NSEvent*)e
{
    const NSPoint p = [self convertPoint:e.locationInWindow fromView:nil];
    if (std::hypot(p.x - _downPoint.x, p.y - _downPoint.y) > 3.0) _dragged = YES;
    _pan.x += p.x - _last.x; _pan.y += p.y - _last.y;
    _last = p;
    [self setNeedsDisplay:YES];
}
- (void)zoom:(double)factor at:(NSPoint)p
{
    const double z = std::max(0.2, std::min(60.0, _zoom * factor));
    const double k = z / _zoom;
    const NSPoint c = NSMakePoint(self.bounds.size.width / 2, self.bounds.size.height / 2);   // keep the point under the cursor in place
    _pan.x = (p.x - c.x) - ((p.x - c.x) - _pan.x) * k;
    _pan.y = (p.y - c.y) - ((p.y - c.y) - _pan.y) * k;
    _zoom = z;
    [self setNeedsDisplay:YES];
}
- (void)scrollWheel:(NSEvent*)e
{
    const double d = e.hasPreciseScrollingDeltas ? e.scrollingDeltaY * 0.01 : e.scrollingDeltaY * 0.1;
    [self zoom:std::pow(1.0 + d, 1.0) at:[self convertPoint:e.locationInWindow fromView:nil]];
}
- (void)magnifyWithEvent:(NSEvent*)e { [self zoom:1.0 + e.magnification at:[self convertPoint:e.locationInWindow fromView:nil]]; }

@end

// ------------------------------------------------------------------------------------------------------------------ the application

// The document of the sidebar's scroll view: the content starts at the top.
@interface FlippedView : NSView
@end
@implementation FlippedView
- (BOOL)isFlipped { return YES; }
@end

static NSTextField* label(NSString* text, CGFloat size = 13, BOOL secondary = NO, BOOL bold = NO)
{
    NSTextField* t = [NSTextField labelWithString:text];
    t.font = bold ? [NSFont boldSystemFontOfSize:size] : [NSFont systemFontOfSize:size];
    t.textColor = secondary ? NSColor.secondaryLabelColor : NSColor.labelColor;
    t.lineBreakMode = NSLineBreakByTruncatingMiddle;
    return t;
}

@class AppController;
// the hands of the assistant: it works on the thread of the agent and moves what must be done to the main thread
std::unique_ptr<AtlasHost> makeNativeHost(AppController* controller);

@interface AppController : NSObject <NSApplicationDelegate, NSToolbarDelegate, NSToolbarItemValidation, NSMenuItemValidation, NSWindowDelegate> {
    NSWindow* _window;
    NSSplitViewController* _split;
    NSSplitViewItem* _uvItem;
    ChatPanel* _chat;
    NSSplitViewItem* _chatItem;
    ChatSession _chatSession;
    std::thread _chatThread;
    std::atomic<bool> _chatStop;
    BOOL _chatBusy;
    dispatch_semaphore_t _openSignal;    // released when a file the assistant opened is shown (or failed)
    dispatch_semaphore_t _atlasSignal;   // released when a run the assistant started has ended
    BOOL _lastOK;
    BOOL _checkerOn;            // the UV checker is drawn (the renderer holds the atlas coordinates)
    BOOL _lookAtPatch;          // the option: choosing a patch in the atlas or the table turns the camera to it
    NSInteger _selectedPatch;   // the row chosen in the patch table (-1: none)
    BOOL _sourceTextureShown;   // the mesh is shown with the texture of its file (no atlas yet)
    NSString* _lastError;
    UvView* _uv;
    SceneView* _scene;
    NSSegmentedControl* _viewSwitch;

    NSTextField* _meshLabel;
    NSPopUpButton* _quality;
    NSButton* _manual;
    NSTextField* _patchField;
    NSStepper* _patchStepper;
    NSPopUpButton* _textureSize;
    NSSlider* _mip;
    NSTextField* _mipLabel;
    NSButton* _makeButton;
    NSButton* _cancelButton;
    NSProgressIndicator* _progress;
    NSTextField* _status;
    NSTextField* _result;
    NSStackView* _presentGroup;
    NSSlider* _offset;
    NSSlider* _alpha;
    NSTextField* _offsetLabel;
    NSTextField* _alphaLabel;
    NSPopUpButton* _pass1;
    NSPopUpButton* _pass2;
    int _mode;

    AtlasSession _session;
    HugeSession _huge;
    BOOL _hookPending;
    BOOL _useHuge;              // the mesh is a big scan: it is read by the huge mode when the atlas is made
    BOOL _textureTouched;       // the user chose a texture size: it is not suggested from the size of the mesh any more
    NSButton* _repackButton;
    DeveloperPanel* _dev;
    NSMenuItem* _wireItem;
    VsaLab* _lab;
    NSMenuItem* _debugMenuItem;   // the Debug menu: only in the developer mode
    BOOL _developer;
    BOOL _wire;
    prefs::Values _prefs;
    NSString* _pendingOpen;     // a file the system asked to open before the window was made
    std::atomic<bool> _cancel;
    std::thread _worker;
    BOOL _running;
}
- (void)showVsaLab:(id)sender;
- (void)selectPatch:(NSInteger)patch focus:(BOOL)focus;
- (void)lookAtSelectedPatch:(id)sender;
- (Json)hostOpen:(NSString*)path;
- (Json)hostMakeAtlas:(const Json&)args;
- (Json)hostRepack:(int)size mip:(int)mip;
- (Json)hostState;
- (Json)hostLayout;
- (Json)hostReport;
- (Json)hostSave:(NSString*)name;
- (void)hostCancel;
@end

@implementation AppController

- (BOOL)hasMeshToWork { return _useHuge ? !_huge.path().empty() : _session.hasMesh(); }
- (BOOL)hasResult { return _useHuge ? _huge.hasResult() : _session.hasResult(); }
- (BOOL)hasTextureView { return [self hasBaked] || (_sourceTextureShown && ![self hasResult]); }
- (BOOL)hasBaked { return _useHuge ? _huge.hasBakedTexture() : _session.hasBakedTexture(); }

- (SessionSettings)settings
{
    SessionSettings s;
    s.quality = int(_quality.indexOfSelectedItem);
    s.manualPatches = _manual.state == NSControlStateValueOn;
    s.patches = _patchField.intValue;
    s.resolution = int(_textureSize.selectedItem.tag);
    s.maxMipLevel = int(_mip.integerValue);
    if (_developer) s.dev = [_dev options];
    return s;
}

// ---- the sidebar: the steps of the work as a form
- (NSView*)buildSidebar
{
    NSVisualEffectView* fx = [[NSVisualEffectView alloc] init];
    fx.material = NSVisualEffectMaterialSidebar;
    fx.blendingMode = NSVisualEffectBlendingModeBehindWindow;
    fx.state = NSVisualEffectStateFollowsWindowActiveState;

    _meshLabel = label(@"No mesh", 13, YES);

    _quality = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    [_quality addItemsWithTitles:@[@"Fast", @"Balanced", @"Detailed"]];
    [_quality selectItemAtIndex:1];
    _quality.accessibilityLabel = @"Quality";

    _manual = [NSButton checkboxWithTitle:@"Set the number of patches" target:self action:@selector(manualChanged:)];
    _patchField = [NSTextField textFieldWithString:@"100"];
    _patchField.alignment = NSTextAlignmentRight;
    _patchField.enabled = NO;
    _patchField.accessibilityLabel = @"Number of patches";
    _patchStepper = [[NSStepper alloc] init];
    _patchStepper.minValue = 1; _patchStepper.maxValue = 20000; _patchStepper.increment = 10; _patchStepper.intValue = 100;
    _patchStepper.enabled = NO;
    _patchStepper.target = self; _patchStepper.action = @selector(stepperChanged:);
    NSStackView* patchRow = [NSStackView stackViewWithViews:@[_patchField, _patchStepper]];
    patchRow.spacing = 4;
    [_patchField.widthAnchor constraintEqualToConstant:80].active = YES;

    _textureSize = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    for (int r : {256, 512, 1024, 2048, 4096, 8192}) {
        [_textureSize addItemWithTitle:[NSString stringWithFormat:@"%d × %d", r, r]];
        _textureSize.lastItem.tag = r;
    }
    [_textureSize selectItemWithTag:2048];
    _textureSize.accessibilityLabel = @"Texture size";
    _textureSize.target = self; _textureSize.action = @selector(textureChanged:);

    _mip = [NSSlider sliderWithValue:3 minValue:0 maxValue:8 target:self action:@selector(mipChanged:)];
    _mip.numberOfTickMarks = 9;
    _mip.allowsTickMarkValuesOnly = YES;
    _mip.accessibilityLabel = @"Mip levels";
    _mipLabel = label(@"", 11, YES);
    _mipLabel.lineBreakMode = NSLineBreakByWordWrapping;
    _mipLabel.maximumNumberOfLines = 2;
    _mipLabel.preferredMaxLayoutWidth = 260;

    _makeButton = [NSButton buttonWithTitle:@"Make Atlas" target:self action:@selector(makeAtlas:)];
    _makeButton.keyEquivalent = @"\r";   // the default button: Return, in the accent color of the system
    _makeButton.controlSize = NSControlSizeLarge;
    _cancelButton = [NSButton buttonWithTitle:@"Cancel" target:self action:@selector(cancelAtlas:)];
    _cancelButton.keyEquivalent = @".";
    _cancelButton.keyEquivalentModifierMask = NSEventModifierFlagCommand;
    _cancelButton.hidden = YES;
    NSStackView* makeRow = [NSStackView stackViewWithViews:@[_makeButton, _cancelButton]];
    makeRow.spacing = 8;

    _progress = [[NSProgressIndicator alloc] init];
    _progress.style = NSProgressIndicatorStyleBar;
    _progress.indeterminate = NO;
    _progress.minValue = 0; _progress.maxValue = 1;
    _progress.hidden = YES;
    _status = label(@"", 11, YES);
    _result = label(@"", 12);
    _result.lineBreakMode = NSLineBreakByWordWrapping;
    _result.maximumNumberOfLines = 4;
    _result.preferredMaxLayoutWidth = 260;

    // Presentation: shown only while the presentation is the view
    _offset = [NSSlider sliderWithValue:0.06 minValue:0 maxValue:1 target:self action:@selector(presentationChanged:)];
    _offset.accessibilityLabel = @"Patch offset";
    _alpha = [NSSlider sliderWithValue:0.4 minValue:0.05 maxValue:1 target:self action:@selector(presentationChanged:)];
    _alpha.accessibilityLabel = @"Patch opacity";
    _offsetLabel = label(@"", 11, YES);
    _alphaLabel = label(@"", 11, YES);
    auto passMenu = [&](NSString* what) {
        NSPopUpButton* b = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
        [b addItemsWithTitles:@[@"Automatic", @"Patch Colors", @"Texture", @"Shading"]];
        b.target = self; b.action = @selector(presentationChanged:);
        b.accessibilityLabel = what;
        return b;
    };
    _pass1 = passMenu(@"First pass");
    _pass2 = passMenu(@"Second pass");
    _presentGroup = [NSStackView stackViewWithViews:@[
        label(@"Presentation", 11, YES, YES), label(@"Patch offset", 12), _offset, _offsetLabel, label(@"Patch opacity", 12), _alpha, _alphaLabel,
        label(@"First pass", 12), _pass1, label(@"Second pass", 12), _pass2]];
    _presentGroup.orientation = NSUserInterfaceLayoutOrientationVertical;
    _presentGroup.alignment = NSLayoutAttributeLeading;
    _presentGroup.spacing = 8;
    _presentGroup.hidden = YES;

    _repackButton = [NSButton buttonWithTitle:@"Repack Atlas" target:self action:@selector(repackAtlas:)];
    _repackButton.enabled = NO;
    _repackButton.toolTip = @"Pack the same patches again with the texture size and mip levels set above (⌥⌘R)";
    _dev = [[DeveloperPanel alloc] initWithTarget:self compareAction:@selector(compareLayouts:) matchAction:@selector(matchOriginalLayout:) inspectAction:@selector(inspectPatch:) againAction:@selector(unfoldPatchAgain:)];
    _selectedPatch = -1;
    __weak AppController* weakDev = self;
    _dev.onViewOptions = ^{ [weakDev applyViewOptions]; };
    _scene.onGpuTime = ^(double ms) { AppController* me = weakDev; if (me) [me->_dev setGpuMilliseconds:ms]; };
    _lookAtPatch = [NSUserDefaults.standardUserDefaults objectForKey:@"lookAtPatch"] ? [NSUserDefaults.standardUserDefaults boolForKey:@"lookAtPatch"] : YES;
    _dev.onPatchSelected = ^(NSInteger patch) { AppController* me = weakDev; if (me) [me selectPatch:patch focus:YES]; };
    _scene.onPickFace = ^(int face) { AppController* me = weakDev; if (me) [me selectPatch:[me patchOfFace:face] focus:NO]; };
    _dev.onColorMode = ^(NSInteger mode) { [weakDev recolorPatches]; };
    NSStackView* stack = [NSStackView stackViewWithViews:@[
        label(@"Mesh", 11, YES, YES), _meshLabel,
        label(@"Settings", 11, YES, YES), label(@"Quality", 12), _quality, _manual, patchRow,
        label(@"Texture size", 12), _textureSize, label(@"Mip levels", 12), _mip, _mipLabel,
        label(@"Atlas", 11, YES, YES), makeRow, _progress, _status,
        label(@"Result", 11, YES, YES), _result, _repackButton, _presentGroup, _dev.view]];
    stack.orientation = NSUserInterfaceLayoutOrientationVertical;
    stack.alignment = NSLayoutAttributeLeading;
    stack.spacing = 8;
    stack.edgeInsets = NSEdgeInsetsMake(52, 20, 20, 20);   // the top stays free for the toolbar (the sidebar goes under it)
    stack.translatesAutoresizingMaskIntoConstraints = NO;
    // group spacing: 20 between the groups
    for (NSView* v in @[_meshLabel]) (void)v;
    [stack setCustomSpacing:20 afterView:_meshLabel];
    [stack setCustomSpacing:20 afterView:patchRow];
    [stack setCustomSpacing:20 afterView:_mipLabel];
    [stack setCustomSpacing:20 afterView:_status];
    // the form scrolls when the window is low (the presentation group makes it long)
    FlippedView* doc = [[FlippedView alloc] init];
    doc.translatesAutoresizingMaskIntoConstraints = NO;
    [doc addSubview:stack];
    NSScrollView* scroll = [[NSScrollView alloc] init];
    scroll.translatesAutoresizingMaskIntoConstraints = NO;
    scroll.drawsBackground = NO;
    scroll.hasVerticalScroller = YES;
    scroll.autohidesScrollers = YES;
    scroll.borderType = NSNoBorder;
    scroll.automaticallyAdjustsContentInsets = NO;
    scroll.documentView = doc;
    [fx addSubview:scroll];
    [NSLayoutConstraint activateConstraints:@[
        [scroll.topAnchor constraintEqualToAnchor:fx.topAnchor],
        [scroll.bottomAnchor constraintEqualToAnchor:fx.bottomAnchor],
        [scroll.leadingAnchor constraintEqualToAnchor:fx.leadingAnchor],
        [scroll.trailingAnchor constraintEqualToAnchor:fx.trailingAnchor],
        [doc.topAnchor constraintEqualToAnchor:scroll.contentView.topAnchor],
        [doc.leadingAnchor constraintEqualToAnchor:scroll.contentView.leadingAnchor],
        [doc.trailingAnchor constraintEqualToAnchor:scroll.contentView.trailingAnchor],
        [stack.topAnchor constraintEqualToAnchor:doc.topAnchor],
        [stack.bottomAnchor constraintEqualToAnchor:doc.bottomAnchor],
        [stack.leadingAnchor constraintEqualToAnchor:doc.leadingAnchor],
        [stack.trailingAnchor constraintEqualToAnchor:doc.trailingAnchor]]];
    _meshLabel.lineBreakMode = NSLineBreakByTruncatingMiddle;
    [_meshLabel.widthAnchor constraintLessThanOrEqualToAnchor:stack.widthAnchor constant:-40].active = YES;
    [_status.widthAnchor constraintLessThanOrEqualToAnchor:stack.widthAnchor constant:-40].active = YES;
    for (NSView* v in @[(NSView*)_quality, (NSView*)_textureSize, (NSView*)_mip, (NSView*)_progress, (NSView*)_offset, (NSView*)_alpha,
                        (NSView*)_pass1, (NSView*)_pass2])
        [v.widthAnchor constraintEqualToAnchor:stack.widthAnchor constant:-40].active = YES;
    [stack setCustomSpacing:20 afterView:_repackButton];
    [stack setCustomSpacing:20 afterView:_presentGroup];
    [_dev.view.widthAnchor constraintEqualToAnchor:stack.widthAnchor constant:-40].active = YES;
    [self updateMipLabel];
    [self presentationChanged:nil];
    return fx;
}

- (void)updateMipLabel
{
    const int mip = int(_mip.integerValue), res = int(_textureSize.selectedItem.tag);
    int maxLevel = 0;
    while ((1 << (maxLevel + 1)) <= res) ++maxLevel;
    maxLevel = std::max(0, maxLevel - 2);
    if (mip > maxLevel) _mip.integerValue = maxLevel;
    _mip.maxValue = maxLevel;
    _mip.numberOfTickMarks = maxLevel + 1;
    const int gutter = 2 * ((int)std::ceil(1.5 * double(1 << int(_mip.integerValue))) - 1);
    _mipLabel.stringValue = [NSString stringWithFormat:@"Level %d, gap between the parts of the texture: %d texels",
                                                      int(_mip.integerValue), gutter];
}

- (void)presentationChanged:(id)sender
{
    _offsetLabel.stringValue = [NSString stringWithFormat:@"%.2f of the size of the model", _offset.doubleValue];
    _alphaLabel.stringValue = [NSString stringWithFormat:@"%.0f%%", 100.0 * _alpha.doubleValue];
    [_scene setOffset:float(_offset.doubleValue) alpha:float(_alpha.doubleValue)];
    // the popups: item 0 is "Automatic", then 0 patch colors, 1 texture, 2 shading
    [_scene setPass1:int(_pass1.indexOfSelectedItem) - 1 pass2:int(_pass2.indexOfSelectedItem) - 1];
}

- (void)mipChanged:(id)sender { [self updateMipLabel]; }
- (void)textureChanged:(id)sender { _textureTouched = YES; [self updateMipLabel]; }
- (void)manualChanged:(id)sender
{
    const BOOL on = _manual.state == NSControlStateValueOn;
    _patchField.enabled = on;
    _patchStepper.enabled = on;
}
- (void)stepperChanged:(id)sender { _patchField.intValue = _patchStepper.intValue; }

// ---- the window
- (void)applicationDidFinishLaunching:(NSNotification*)n
{
    if (getenv("HUGE_NATIVE_DARK")) NSApp.appearance = [NSAppearance appearanceNamed:NSAppearanceNameDarkAqua];   // a check of the dark mode
    _prefs = prefs::load();
    if (getenv("HUGE_NATIVE_DEV")) _prefs.developer = true;   // testing hook
    _developer = _prefs.developer;
    [NSNotificationCenter.defaultCenter addObserver:self selector:@selector(prefsChanged:) name:prefs::Changed object:nil];
    [self buildMenus];
    _scene = [[SceneView alloc] initWithFrame:NSMakeRect(0, 0, 800, 600)];
    [_scene setShowGrid:[NSUserDefaults.standardUserDefaults objectForKey:@"showGrid"] ? [NSUserDefaults.standardUserDefaults boolForKey:@"showGrid"] : YES];
    __weak AppController* weakSelf = self;
    _scene.onDropURL = ^(NSURL* url) { [weakSelf loadPath:url.path]; };

    NSViewController* sideVC = [[NSViewController alloc] init];
    sideVC.view = [self buildSidebar];
    NSViewController* sceneVC = [[NSViewController alloc] init];
    sceneVC.view = _scene;
    _split = [[NSSplitViewController alloc] init];
    NSSplitViewItem* sideItem = [NSSplitViewItem sidebarWithViewController:sideVC];
    sideItem.minimumThickness = 280;
    sideItem.maximumThickness = 340;
    sideItem.canCollapse = YES;
    [_split addSplitViewItem:sideItem];
    [_split addSplitViewItem:[NSSplitViewItem splitViewItemWithViewController:sceneVC]];
    // the atlas on the trailing side: hidden until there is one, opened and closed by the toolbar and by View > Show UV Layout
    _uv = [[UvView alloc] initWithFrame:NSMakeRect(0, 0, 360, 600)];
    __weak AppController* weakUv = self;
    _uv.onPickPatch = ^(NSInteger patch) { AppController* me = weakUv; if (me) [me selectPatch:patch focus:YES]; };
    NSViewController* uvVC = [[NSViewController alloc] init];
    uvVC.view = _uv;
    _uvItem = [NSSplitViewItem splitViewItemWithViewController:uvVC];
    _uvItem.minimumThickness = 280;
    _uvItem.canCollapse = YES;
    _uvItem.collapsed = YES;
    [_split addSplitViewItem:_uvItem];
    // the assistant on the far trailing side: closed until it is asked for
    _chat = [[ChatPanel alloc] initWithFrame:NSMakeRect(0, 0, 340, 600)];
    __weak AppController* weak = self;
    _chat.onSend = ^(NSString* t) { [weak chatSend:t]; };
    _chat.onStop = ^{ [weak chatStop]; };
    _chat.onSetup = ^{ [weak chatSetup]; };
    NSViewController* chatVC = [[NSViewController alloc] init];
    chatVC.view = _chat;
    _chatItem = [NSSplitViewItem splitViewItemWithViewController:chatVC];
    _chatItem.minimumThickness = 300;
    _chatItem.canCollapse = YES;
    _chatItem.collapsed = YES;
    [_split addSplitViewItem:_chatItem];

    _window = [NSWindow windowWithContentViewController:_split];
    _window.styleMask |= NSWindowStyleMaskResizable | NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskFullSizeContentView;
    _window.title = @"huge-atlas";
    _window.minSize = NSMakeSize(900, 560);
    [_window setContentSize:NSMakeSize(1200, 760)];
    [_window center];
    [_window setFrameAutosaveName:@"MainWindow"];
    _window.delegate = self;
    _window.collectionBehavior |= NSWindowCollectionBehaviorFullScreenPrimary;
    NSToolbar* tb = [[NSToolbar alloc] initWithIdentifier:@"MainToolbar"];
    tb.delegate = self;
    tb.displayMode = NSToolbarDisplayModeIconAndLabel;
    _window.toolbar = tb;
    _window.toolbarStyle = NSWindowToolbarStyleUnified;
    [_window makeKeyAndOrderFront:nil];
    [NSApp activateIgnoringOtherApps:YES];
    _status.stringValue = @"Open a mesh to begin";
    // the sidebar starts with the defaults of the Settings window
    [_quality selectItemAtIndex:_prefs.quality];
    [_textureSize selectItemWithTag:_prefs.textureSize];
    _mip.integerValue = _prefs.mipLevels;
    [self updateMipLabel];
    [self applyDeveloperMode];
    if (getenv("HUGE_NATIVE_WIRE") && _developer) [self toggleWireframe:nil];   // testing hook
    if (const char* st = getenv("HUGE_NATIVE_SETTINGS")) {   // testing hook: the Settings window (pane 0 or 1), shot of the window itself
        showSettingsWindow(atoi(st));
        [self autoShotAndQuit];
        return;
    }

    if (const char* bad = getenv("HUGE_NATIVE_LOADBAD")) {   // testing hook: Debug > Load Bad Patch on this file, then the shot
        [self loadBadPatchPath:[NSString stringWithUTF8String:bad]];
        [self autoShotAndQuit];
        return;
    }
    if (_pendingOpen) { [self loadPath:_pendingOpen]; _pendingOpen = nil; }
    const char* arg = getenv("HUGE_NATIVE_OPEN");
    if (arg) {
        _hookPending = YES;
        [self loadPath:[NSString stringWithUTF8String:arg]];
    } else if (getenv("HUGE_NATIVE_SHOT")) {
        [self autoShotAndQuit];
    }
}

// a file opened from Finder or dropped on the icon of the Dock (it can come before the window exists)
- (BOOL)application:(NSApplication*)app openFile:(NSString*)filename
{
    if (!_window) { _pendingOpen = filename; return YES; }
    [self loadPath:filename];
    return YES;
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)app { return YES; }
- (void)application:(NSApplication*)app openURLs:(NSArray<NSURL*>*)urls { if (urls.count) [self loadPath:urls.firstObject.path]; }

// ---- toolbar
- (NSArray<NSToolbarItemIdentifier>*)toolbarDefaultItemIdentifiers:(NSToolbar*)t
{
    return @[NSToolbarToggleSidebarItemIdentifier, NSToolbarSidebarTrackingSeparatorItemIdentifier, @"makeAtlas",
             NSToolbarFlexibleSpaceItemIdentifier, @"views", @"save", @"uv", @"assistant"];
}
- (NSArray<NSToolbarItemIdentifier>*)toolbarAllowedItemIdentifiers:(NSToolbar*)t { return [self toolbarDefaultItemIdentifiers:t]; }

- (NSToolbarItem*)toolbar:(NSToolbar*)t itemForItemIdentifier:(NSToolbarItemIdentifier)ident willBeInsertedIntoToolbar:(BOOL)flag
{
    NSToolbarItem* item = [[NSToolbarItem alloc] initWithItemIdentifier:ident];
    auto set = [&](NSString* title, NSString* symbol, SEL action, NSString* tip) {
        item.label = title; item.paletteLabel = title; item.toolTip = tip;
        item.image = [NSImage imageWithSystemSymbolName:symbol accessibilityDescription:title];
        item.target = self; item.action = action;
    };
    if ([ident isEqual:@"open"]) set(@"Open", @"folder", @selector(openMesh:), @"Open a mesh (⌘O)");
    else if ([ident isEqual:@"makeAtlas"]) set(@"Make Atlas", @"wand.and.stars", @selector(makeAtlas:), @"Make the atlas (⌘R)");
    else if ([ident isEqual:@"uv"]) set(@"UV Layout", @"square.grid.2x2", @selector(toggleUVLayout:), @"Show or hide the atlas (⌥⌘U)");
    else if ([ident isEqual:@"assistant"]) set(@"Assistant", @"bubble.left.and.bubble.right", @selector(toggleAssistant:), @"Ask the assistant to make or tune the atlas (⌥⌘I)");
    else if ([ident isEqual:@"save"]) set(@"Save", @"square.and.arrow.down", @selector(saveAs:), @"Save the mesh with the atlas (⇧⌘S)");
    else if ([ident isEqual:@"views"]) {
        _viewSwitch = [NSSegmentedControl segmentedControlWithLabels:@[@"Patches", @"Plain", @"Presentation", @"Texture"]
                                                         trackingMode:NSSegmentSwitchTrackingSelectOne
                                                               target:self action:@selector(viewSwitched:)];
        _viewSwitch.selectedSegment = 1;
        [_viewSwitch setEnabled:NO forSegment:0];
        [_viewSwitch setEnabled:NO forSegment:2];
        [_viewSwitch setEnabled:NO forSegment:3];
        _viewSwitch.accessibilityLabel = @"View";
        item.label = @"View"; item.paletteLabel = @"View"; item.view = _viewSwitch;
    }
    return item;
}

- (BOOL)validateToolbarItem:(NSToolbarItem*)item
{
    if ([item.itemIdentifier isEqual:@"open"]) return !_running;
    if ([item.itemIdentifier isEqual:@"makeAtlas"]) return !_running && [self hasMeshToWork];
    if ([item.itemIdentifier isEqual:@"save"]) return !_running && [self hasResult];
    if ([item.itemIdentifier isEqual:@"uv"]) return [self hasResult];
    return YES;
}

// ---- menus
- (void)buildMenus
{
    NSMenu* bar = [[NSMenu alloc] init];
    NSString* name = @"huge-atlas";
    auto add = [&](NSMenu* m, NSString* title, SEL a, NSString* key, NSEventModifierFlags mods = NSEventModifierFlagCommand, id target = nil) {
        NSMenuItem* it = [m addItemWithTitle:title action:a keyEquivalent:key];
        it.keyEquivalentModifierMask = mods;
        it.target = target;
        return it;
    };
    auto top = [&](NSString* title) {
        NSMenuItem* it = [[NSMenuItem alloc] initWithTitle:title action:nil keyEquivalent:@""];
        NSMenu* m = [[NSMenu alloc] initWithTitle:title];
        it.submenu = m;
        [bar addItem:it];
        return m;
    };
    NSMenu* app = top(name);
    add(app, [@"About " stringByAppendingString:name], @selector(orderFrontStandardAboutPanel:), @"", 0, NSApp);
    [app addItem:[NSMenuItem separatorItem]];
    add(app, @"Settings…", @selector(showSettings:), @",", NSEventModifierFlagCommand, self);
    [app addItem:[NSMenuItem separatorItem]];
    NSMenu* services = [[NSMenu alloc] initWithTitle:@"Services"];
    [app addItemWithTitle:@"Services" action:nil keyEquivalent:@""].submenu = services;
    NSApp.servicesMenu = services;
    [app addItem:[NSMenuItem separatorItem]];
    add(app, [@"Hide " stringByAppendingString:name], @selector(hide:), @"h", NSEventModifierFlagCommand, NSApp);
    add(app, @"Hide Others", @selector(hideOtherApplications:), @"h", NSEventModifierFlagCommand | NSEventModifierFlagOption, NSApp);
    add(app, @"Show All", @selector(unhideAllApplications:), @"", 0, NSApp);
    [app addItem:[NSMenuItem separatorItem]];
    add(app, [@"Quit " stringByAppendingString:name], @selector(terminate:), @"q", NSEventModifierFlagCommand, NSApp);

    NSMenu* file = top(@"File");
    add(file, @"Open…", @selector(openMesh:), @"o", NSEventModifierFlagCommand, self);
    [file addItem:[NSMenuItem separatorItem]];
    add(file, @"Close", @selector(performClose:), @"w");
    add(file, @"Save As…", @selector(saveAs:), @"s", NSEventModifierFlagCommand | NSEventModifierFlagShift, self);

    NSMenu* edit = top(@"Edit");
    add(edit, @"Undo", @selector(undo:), @"z");
    add(edit, @"Redo", @selector(redo:), @"z", NSEventModifierFlagCommand | NSEventModifierFlagShift);
    [edit addItem:[NSMenuItem separatorItem]];
    add(edit, @"Cut", @selector(cut:), @"x");
    add(edit, @"Copy", @selector(copy:), @"c");
    add(edit, @"Paste", @selector(paste:), @"v");
    add(edit, @"Select All", @selector(selectAll:), @"a");

    NSMenu* atlas = top(@"Atlas");
    add(atlas, @"Make Atlas", @selector(makeAtlas:), @"r", NSEventModifierFlagCommand, self);
    add(atlas, @"Cancel", @selector(cancelAtlas:), @".", NSEventModifierFlagCommand, self);
    [atlas addItem:[NSMenuItem separatorItem]];
    add(atlas, @"Repack Atlas", @selector(repackAtlas:), @"r", NSEventModifierFlagCommand | NSEventModifierFlagOption, self);

    NSMenu* view = top(@"View");
    add(view, @"Show Patches", @selector(showPatches:), @"1", NSEventModifierFlagCommand, self);
    add(view, @"Show Plain Shading", @selector(showPlain:), @"2", NSEventModifierFlagCommand, self);
    add(view, @"Show Presentation", @selector(showPresentation:), @"3", NSEventModifierFlagCommand, self);
    add(view, @"Show Texture", @selector(showTexture:), @"4", NSEventModifierFlagCommand, self);
    [view addItem:[NSMenuItem separatorItem]];
    add(view, @"Reset Camera", @selector(resetCamera:), @"0", NSEventModifierFlagCommand, self);
    [view addItem:[NSMenuItem separatorItem]];
    add(view, @"Show UV Layout", @selector(toggleUVLayout:), @"u", NSEventModifierFlagCommand | NSEventModifierFlagOption, self);
    add(view, @"Look at Selected Patch", @selector(lookAtSelectedPatch:), @"f", NSEventModifierFlagCommand | NSEventModifierFlagOption, self);
    add(view, @"Look at a Patch When It Is Chosen in the Atlas or the Table", @selector(toggleLookAtPatch:), @"", 0, self);
    add(view, @"Show Assistant", @selector(toggleAssistant:), @"i", NSEventModifierFlagCommand | NSEventModifierFlagOption, self);
    add(view, @"Show Grid", @selector(toggleGrid:), @"g", NSEventModifierFlagCommand | NSEventModifierFlagOption, self);
    _wireItem = add(view, @"Show Wireframe", @selector(toggleWireframe:), @"w", NSEventModifierFlagCommand | NSEventModifierFlagOption, self);
    add(view, @"Show Sidebar", @selector(toggleSidebar:), @"s", NSEventModifierFlagCommand | NSEventModifierFlagControl);
    add(view, @"Enter Full Screen", @selector(toggleFullScreen:), @"f", NSEventModifierFlagCommand | NSEventModifierFlagControl);

    NSMenu* debug = top(@"Debug");
    _debugMenuItem = bar.itemArray.lastObject;
    add(debug, @"Save Bad Patches", @selector(saveBadPatches:), @"b", NSEventModifierFlagCommand | NSEventModifierFlagOption, self);
    add(debug, @"Segmentation Lab…", @selector(showVsaLab:), @"v", NSEventModifierFlagCommand | NSEventModifierFlagOption, self);
    add(debug, @"Load Bad Patch…", @selector(loadBadPatch:), @"l", NSEventModifierFlagCommand | NSEventModifierFlagOption, self);
    [debug addItem:[NSMenuItem separatorItem]];
    add(debug, @"Export Atlas Mask…", @selector(exportMask:), @"m", NSEventModifierFlagCommand | NSEventModifierFlagOption, self);

    NSMenu* window = top(@"Window");
    add(window, @"Minimize", @selector(performMiniaturize:), @"m");
    add(window, @"Zoom", @selector(performZoom:), @"");
    [window addItem:[NSMenuItem separatorItem]];
    add(window, @"Bring All to Front", @selector(arrangeInFront:), @"", 0, NSApp);
    NSApp.windowsMenu = window;

    NSMenu* help = top(@"Help");
    add(help, @"huge-atlas Help", @selector(showHelp:), @"?", NSEventModifierFlagCommand, self);
    NSApp.helpMenu = help;
    NSApp.mainMenu = bar;
}

- (void)showHelp:(id)sender
{
    [NSWorkspace.sharedWorkspace openURL:[NSURL URLWithString:@"https://github.com/vladimir-vasilyev/huge-atlas#how-to-use"]];
}

- (BOOL)validateMenuItem:(NSMenuItem*)item
{
    const SEL a = item.action;
    if (a == @selector(openMesh:)) return !_running;
    if (a == @selector(makeAtlas:)) return !_running && [self hasMeshToWork];
    if (a == @selector(cancelAtlas:)) return _running;
    if (a == @selector(repackAtlas:)) return !_running && [self hasResult];
    if (a == @selector(saveAs:)) return !_running && [self hasResult];
    if (a == @selector(toggleGrid:)) { item.state = [_scene showGrid] ? NSControlStateValueOn : NSControlStateValueOff; return YES; }
    if (a == @selector(toggleWireframe:)) { item.state = _wire ? NSControlStateValueOn : NSControlStateValueOff; item.hidden = !_developer; return _developer; }
    if (a == @selector(compareLayouts:)) return !_running && [self hasResult];
    if (a == @selector(showPatches:)) { item.state = _mode == 0 ? NSControlStateValueOn : NSControlStateValueOff; return [self hasResult]; }
    if (a == @selector(showPlain:)) item.state = _mode == 1 ? NSControlStateValueOn : NSControlStateValueOff;
    if (a == @selector(showPresentation:)) { item.state = _mode == 2 ? NSControlStateValueOn : NSControlStateValueOff; return [self hasResult]; }
    if (a == @selector(showTexture:)) { item.state = _mode == 3 ? NSControlStateValueOn : NSControlStateValueOff; return [self hasTextureView]; }
    if (a == @selector(toggleUVLayout:)) {
        item.title = _uvItem.collapsed ? @"Show UV Layout" : @"Hide UV Layout";
        return [self hasResult];
    }
    if (a == @selector(lookAtSelectedPatch:)) return _selectedPatch >= 0 && [self hasResult] && !_running;
    if (a == @selector(toggleLookAtPatch:)) { item.state = _lookAtPatch ? NSControlStateValueOn : NSControlStateValueOff; return YES; }
    if (a == @selector(saveBadPatches:)) return !_running && [self hasResult];
    if (a == @selector(loadBadPatch:)) return !_running;
    if (a == @selector(showVsaLab:)) return YES;
    if (a == @selector(exportMask:)) return !_running && [self hasResult] && !_useHuge;
    if (a == @selector(toggleAssistant:)) item.title = _chatItem.collapsed ? @"Show Assistant" : @"Hide Assistant";
    if (a == @selector(toggleSidebar:)) {
        NSSplitViewItem* s = _split.splitViewItems.firstObject;
        item.title = s.collapsed ? @"Show Sidebar" : @"Hide Sidebar";
    }
    return YES;
}

// ---- actions
- (void)suggestTextureSize:(size_t)faces
{
    if (_textureTouched) return;   // the texture size that fits the mesh, until the user chooses one
    [_textureSize selectItemWithTag:(faces < 1000000 ? 2048 : faces < 8000000 ? 4096 : 8192)];
    [self updateMipLabel];
}

- (void)updateEnabled
{
    if ([_dev cullBackFaces] != [_scene cull]) [_dev setCullBackFaces:[_scene cull]];
    _repackButton.enabled = !_running && [self hasResult];
    [_window.toolbar validateVisibleItems];
}

- (void)loadPath:(NSString*)path
{
    std::string err;
    const size_t estimate = HugeSession::estimateFaces(path.UTF8String);
    if (estimate > size_t(_prefs.hugeMillions) * 1000000) {   // a big scan goes straight into flat arrays (no half-edge mesh), also for viewing
        if (_running) return;
        _useHuge = YES;
        _session = AtlasSession();
        _huge.setFile(path.UTF8String);
        _window.title = path.lastPathComponent;
        _window.representedURL = [NSURL fileURLWithPath:path];
        _meshLabel.stringValue = [NSString stringWithFormat:@"%@ — a big scan, about %.1f million faces", path.lastPathComponent, estimate / 1e6];
        _result.stringValue = @"";
        [_scene clearMesh];
        [_uv clear];
        _uvItem.collapsed = YES;
        [self suggestTextureSize:estimate];
        if (_lab) [_lab attachMesh:nullptr];
        _sourceTextureShown = NO;
        [_dev setInfo:@"Reading the mesh…"];
        [_dev setPatchRows:{}];
        [self setViewMode:1];
        [self beginWork:@"Reading the mesh"];
        __weak AppController* weakSelf = self;
        _worker = std::thread([self, weakSelf, path]() {
            std::string e;
            const bool ok = _huge.load(&e);
            dispatch_async(dispatch_get_main_queue(), ^{
                AppController* me = weakSelf;
                if (!me) return;
                [me setRunning:NO];
                me->_progress.indeterminate = NO;
                if (!ok) {
                    me->_status.stringValue = [NSString stringWithFormat:@"Cannot open %@: %s", path.lastPathComponent, e.c_str()];
                    me->_lastError = [NSString stringWithFormat:@"cannot open: %s", e.c_str()];
                    me->_huge.setFile("");
                    [me updateEnabled];
                    [me openFinished];
                    return;
                }
                me->_meshLabel.stringValue = [NSString stringWithFormat:@"%@ — %zu faces%s", path.lastPathComponent, me->_huge.faceCount(),
                                                                       me->_huge.hasSourceTexture() ? ", with a texture" : ""];
                me->_status.stringValue = @"Ready";
                [me->_scene resetCamera];
                // the texture of the file, as it is: the mesh is shown with it until there is an atlas
                const BOOL tex = me->_huge.hasSourceTexture();
                [me->_scene showFlat:me->_huge.mesh() cornerUV:(tex ? me->_huge.sourceCornerUV() : std::vector<float>()) rgba:{}];
                [me->_scene setTextureWidth:(tex ? me->_huge.sourceWidth() : 0) height:me->_huge.sourceHeight() pixels:me->_huge.sourcePixels()];
                me->_sourceTextureShown = tex;
                me->_checkerOn = NO;
                [me->_scene setCheckerCells:0.0f];
                [me->_dev setResultViewsEnabled:NO];
                [me->_dev setInfo:[NSString stringWithFormat:@"Vertices          %zu\nFaces             %zu\n(a big scan: no further diagnostics)", me->_huge.mesh().vertexCount(), me->_huge.faceCount()]];
                [me->_dev setPatchRows:{}];
                [me setViewMode:tex ? 3 : 1];
                [me suggestTextureSize:me->_huge.faceCount()];
                [me updateEnabled];
                [me openFinished];
            });
        });
        return;
    }
    if (!_session.load(path.UTF8String, &err)) {
        _status.stringValue = [NSString stringWithFormat:@"Cannot open %@: %s", path.lastPathComponent, err.c_str()];
        _lastError = [NSString stringWithFormat:@"cannot open: %s", err.c_str()];
        [self openFinished];
        return;
    }
    _useHuge = NO;
    _huge.clear();
    _window.title = path.lastPathComponent;
    _window.representedURL = [NSURL fileURLWithPath:path];
    _meshLabel.stringValue = [NSString stringWithFormat:@"%@ — %zu faces%s", path.lastPathComponent, _session.mesh().n_faces(),
                                                       _session.hasSourceTexture() ? ", with a texture" : ""];
    _result.stringValue = @"";
    _status.stringValue = @"Ready";
    [_scene resetCamera];
    // a mesh with a texture is shown with it (the texture of the file, its own coordinates) until there is an atlas
    const BOOL tex = _session.hasSourceTexture() && _session.sourceCornerUV().size() == _session.mesh().n_faces() * 6;
    [_scene showMesh:_session.mesh() colors:{} faceUV:(tex ? _session.sourceCornerUV() : std::vector<float>())];
    [_scene setTextureWidth:(tex ? _session.sourceWidth() : 0) height:_session.sourceHeight() pixels:_session.sourcePixels()];
    _sourceTextureShown = tex;
    _checkerOn = NO;
    [_scene setCheckerCells:0.0f];
    [_dev setResultViewsEnabled:NO];
    if (_lab) [_lab attachMesh:&_session.mesh()];
    [_dev setInfo:[NSString stringWithUTF8String:_session.meshInfo().c_str()]];
    [_dev setPatchRows:{}];
    [_uv clear];
    _uvItem.collapsed = YES;
    [self suggestTextureSize:_session.mesh().n_faces()];
    [self setViewMode:tex ? 3 : 1];
    [self updateEnabled];
    [self openFinished];
}

// a file is shown (or could not be opened): the assistant that waits for it goes on, and the test hook runs
- (void)openFinished
{
    if (_openSignal) { dispatch_semaphore_signal(_openSignal); _openSignal = nil; }
    [self runOpenHook];
}

// test hook: after HUGE_NATIVE_OPEN has finished loading, make the atlas or take the shot
- (void)runOpenHook
{
    if (!_hookPending) return;
    _hookPending = NO;
    if (const char* chat = getenv("HUGE_NATIVE_CHAT")) {   // testing hook: open the assistant and send this message, shot of the window at the end
        _chatItem.collapsed = NO;
        [self chatSend:[NSString stringWithUTF8String:chat]];
        return;
    }
    if (getenv("HUGE_NATIVE_AUTORUN")) [self makeAtlas:nil];
    else [self autoShotAndQuit];
}

- (void)openMesh:(id)sender
{
    const std::string p = native::openFile("Open a mesh", {"obj", "ply", "off", "stl"}, "");
    if (!p.empty()) [self loadPath:[NSString stringWithUTF8String:p.c_str()]];
}

- (void)setRunning:(BOOL)running
{
    _running = running;
    _makeButton.enabled = !running;
    _cancelButton.hidden = !running;
    _progress.hidden = !running;
    for (NSControl* c in @[(NSControl*)_quality, (NSControl*)_manual, (NSControl*)_textureSize, (NSControl*)_mip]) c.enabled = !running;
    [_dev setEnabled:!running];
    [self updateEnabled];
}

// the work of the huge mode has no fraction to show: the bar goes round, the line under it says what is happening
- (void)beginWork:(NSString*)first
{
    [self setRunning:YES];
    _cancel = false;
    _progress.indeterminate = _useHuge;
    if (_useHuge) [_progress startAnimation:nil]; else _progress.doubleValue = 0.02;
    _status.stringValue = first;
    if (_worker.joinable()) _worker.join();
}

- (void)makeAtlas:(id)sender { [self makeAtlasWithSettings:[self settings]]; }

- (void)makeAtlasWithSettings:(SessionSettings)s
{
    if (_running || ![self hasMeshToWork]) return;
    [self beginWork:@"Starting…"];
    __weak AppController* weakSelf = self;
    _worker = std::thread([self, weakSelf, s]() {
        std::string err;
        bool ok;
        if (_useHuge) {
            ok = _huge.run(s, [weakSelf](const std::string& what) {
                NSString* text = [NSString stringWithUTF8String:what.c_str()];
                dispatch_async(dispatch_get_main_queue(), ^{ AppController* me = weakSelf; if (me) me->_status.stringValue = text; });
            }, &err, &_cancel);
        } else {
            ok = _session.run(s, [weakSelf](double f, const std::string& what) {
                NSString* text = [NSString stringWithUTF8String:what.c_str()];
                dispatch_async(dispatch_get_main_queue(), ^{
                    AppController* me = weakSelf;
                    if (!me) return;
                    me->_progress.doubleValue = f;
                    me->_status.stringValue = text;
                });
            }, _cancel, &err);
        }
        dispatch_async(dispatch_get_main_queue(), ^{ [weakSelf atlasFinished:ok error:[NSString stringWithUTF8String:err.c_str()] keepView:NO]; });
    });
}

// the same patches packed again with the texture size and the mip levels of the sidebar: nothing before the packing is run again
- (void)repackAtlas:(id)sender { [self repackWithSettings:[self settings]]; }

- (void)repackWithSettings:(SessionSettings)s
{
    if (_running || ![self hasResult]) return;
    [self beginWork:@"Packing the atlas…"];
    __weak AppController* weakSelf = self;
    _worker = std::thread([self, weakSelf, s]() {
        std::string err;
        bool ok;
        if (_useHuge) {
            ok = _huge.repack(s, [weakSelf](const std::string& what) {
                NSString* text = [NSString stringWithUTF8String:what.c_str()];
                dispatch_async(dispatch_get_main_queue(), ^{ AppController* me = weakSelf; if (me) me->_status.stringValue = text; });
            }, &err);
        } else {
            ok = _session.repack(s, &err);
        }
        dispatch_async(dispatch_get_main_queue(), ^{ [weakSelf atlasFinished:ok error:[NSString stringWithUTF8String:err.c_str()] keepView:YES]; });
    });
}

- (void)cancelAtlas:(id)sender { if (_running) { _cancel = true; _status.stringValue = @"Cancelling…"; } }

- (void)atlasFinished:(BOOL)ok error:(NSString*)error keepView:(BOOL)keepView
{
    if (_worker.joinable()) _worker.join();
    if (_useHuge) [_progress stopAnimation:nil];
    _progress.indeterminate = NO;
    [self setRunning:NO];
    _lastOK = ok;
    _lastError = ok ? nil : error;
    if (_atlasSignal) { dispatch_semaphore_signal(_atlasSignal); _atlasSignal = nil; }   // the assistant that waits for the run goes on
    if (!ok) {
        _status.stringValue = [error isEqual:@"cancelled"] ? @"Cancelled" : [@"Failed: " stringByAppendingString:error];
        return;
    }
    _status.stringValue = keepView ? @"Packed again" : @"Done";
    [self showResultKeepingView:keepView];
}

// the result goes to the 3D view and the UV view, the numbers to the sidebar
- (void)showResultKeepingView:(BOOL)keepView
{
    _sourceTextureShown = NO;
    NSString* text;
    if (_useHuge) {
        text = [NSString stringWithFormat:@"%zu patches\nAtlas %d × %d, %.0f%% of the texture is used\n%.1f s", _huge.patchCount(), _huge.resolution(),
                                          _huge.resolution(), 100.0 * _huge.efficiency(), _huge.seconds()];
        if (_huge.facesWithoutUV() > 0)
            text = [text stringByAppendingFormat:@"\n%zu faces could not be unfolded and have no texture coordinates", _huge.facesWithoutUV()];
        [_scene showFlat:_huge.mesh() cornerUV:(_huge.hasBakedTexture() || _checkerOn ? _huge.cornerUV() : std::vector<float>()) rgba:_huge.patchRgba()];
        [_scene setTexture:(_huge.hasBakedTexture() ? _huge.bakedResolution() : 0) pixels:_huge.bakedPixels()];
        [_scene setPatchNormals:_huge.patchNormalsPerFace()];
        [_uv setCharts:_huge.uvCharts() texture:(_huge.hasBakedTexture() ? _huge.bakedPixels() : nullptr) side:_huge.bakedResolution()];
    } else {
        text = [NSString stringWithFormat:@"%zu patches\nAtlas %d × %d, %.0f%% of the texture is used\n%.1f s", _session.patchCount(), _session.resolution(),
                                          _session.resolution(), 100.0 * _session.efficiency(), _session.seconds()];
        if (_session.unresolvedPatches() > 0)
            text = [text stringByAppendingFormat:@"\n%d patches could not be unfolded cleanly", _session.unresolvedPatches()];
        [_scene showMesh:_session.mesh() colors:_session.patchColors() faceUV:(_session.hasBakedTexture() || _checkerOn ? _session.atlasFaceUV() : std::vector<float>())];
        [_scene setTexture:(_session.hasBakedTexture() ? _session.bakedResolution() : 0) pixels:_session.bakedPixels()];
        [_scene setPatchNormals:_session.patchNormalsPerFace()];
        [_uv setCharts:_session.uvCharts() texture:(_session.hasBakedTexture() ? _session.bakedPixels() : nullptr) side:_session.bakedResolution()];
    }
    _result.stringValue = text;
    {
        const std::string rep = _useHuge ? _huge.report() : _session.report();
        [_dev setReport:[NSString stringWithUTF8String:rep.c_str()]];
        [_dev setPatchRows:_useHuge ? _huge.patchRows() : _session.patchRows()];
        [_dev setResultViewsEnabled:YES];
        [self updateCheckerCells];
        _selectedPatch = -1;
        _uv.selectedPatch = -1;
        if (getenv("HUGE_NATIVE_AUTORUN") && !keepView) printf("NATIVE REPORT:\n%s", rep.c_str());
    }
    if (_prefs.openUV) [_uvItem.animator setCollapsed:NO];
    [self setViewMode:keepView ? _mode : ([self hasBaked] ? 3 : 0)];
    [self updateEnabled];
    if (getenv("HUGE_NATIVE_AUTORUN")) {
        printf("NATIVE: %s%s\n", text.UTF8String, [self hasBaked] ? " (texture baked)" : "");
        if (getenv("HUGE_NATIVE_CHECKER")) [_dev setUvChecker:YES msaa:atoi(getenv("HUGE_NATIVE_CHECKER"))];   // testing hook: the UV checker, anti-aliasing with this many samples
        static BOOL labRan = NO;
        if (getenv("HUGE_NATIVE_LAB") && !labRan) {   // testing hook: the Segmentation Lab by script; its regions start the atlas that is made next
            labRan = YES;
            [self showVsaLab:nil];
            printf("NATIVE LAB: %s\n", [_lab selfTestWithK:atoi(getenv("HUGE_NATIVE_LAB"))].UTF8String);
            dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 800 * NSEC_PER_MSEC), dispatch_get_main_queue(), ^{
                if (const char* shot = getenv("HUGE_NATIVE_LABSHOT")) {
                    CGImageRef img = CGWindowListCreateImage(CGRectNull, kCGWindowListOptionIncludingWindow, CGWindowID(self->_lab.window.windowNumber), kCGWindowImageBoundsIgnoreFraming);
                    if (img) {
                        NSBitmapImageRep* rep = [[NSBitmapImageRep alloc] initWithCGImage:img];
                        [[rep representationUsingType:NSBitmapImageFileTypePNG properties:@{}] writeToFile:[NSString stringWithUTF8String:shot] atomically:YES];
                        CGImageRelease(img);
                    }
                }
                [self makeAtlas:nil];
            });
            return;
        }
        if (const char* c3 = getenv("HUGE_NATIVE_CLICK3D")) {   // testing hook: a click in the 3D view at this fraction of its width and height
            double fx = 0.5, fy = 0.5;
            sscanf(c3, "%lf,%lf", &fx, &fy);
            const NSPoint vp = NSMakePoint(fx * _scene.bounds.size.width, fy * _scene.bounds.size.height);   // origin bottom left
            const NSPoint wp = [_scene convertPoint:vp toView:nil];
            NSEvent* down = [NSEvent mouseEventWithType:NSEventTypeLeftMouseDown location:wp modifierFlags:0 timestamp:0 windowNumber:_window.windowNumber context:nil eventNumber:0 clickCount:1 pressure:1];
            NSEvent* up = [NSEvent mouseEventWithType:NSEventTypeLeftMouseUp location:wp modifierFlags:0 timestamp:0 windowNumber:_window.windowNumber context:nil eventNumber:0 clickCount:1 pressure:0];
            [_scene mouseDown:down];
            [_scene mouseUp:up];
            printf("NATIVE CLICK3D: bounds %.0fx%.0f, face %d, patch %ld\n", _scene.bounds.size.width, _scene.bounds.size.height, [_scene faceAtViewPoint:vp], long(_selectedPatch));
        }
        if (const char* cu = getenv("HUGE_NATIVE_CLICKUV")) {   // testing hook: a click in the UV pane at this (u, v) of the atlas
            double u = 0.5, v = 0.5;
            sscanf(cu, "%lf,%lf", &u, &v);
            const NSRect sq = [_uv square];
            const NSPoint vp = NSMakePoint(sq.origin.x + u * sq.size.width, sq.origin.y + v * sq.size.height);
            const NSPoint wp = [_uv convertPoint:vp toView:nil];
            NSEvent* down = [NSEvent mouseEventWithType:NSEventTypeLeftMouseDown location:wp modifierFlags:0 timestamp:0 windowNumber:_window.windowNumber context:nil eventNumber:0 clickCount:1 pressure:1];
            NSEvent* up = [NSEvent mouseEventWithType:NSEventTypeLeftMouseUp location:wp modifierFlags:0 timestamp:0 windowNumber:_window.windowNumber context:nil eventNumber:0 clickCount:1 pressure:0];
            [_uv mouseDown:down];
            [_uv mouseUp:up];
            if (getenv("HUGE_NATIVE_REFOCUS")) dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 700 * NSEC_PER_MSEC), dispatch_get_main_queue(), ^{ [self lookAtSelectedPatch:nil]; });   // after the UV pane has opened and the view has its final size
            printf("NATIVE CLICKUV: direct hit %ld, selected %ld\n", long([_uv patchAtViewPoint:vp]), long(_selectedPatch));
        }
        if (getenv("HUGE_NATIVE_INSPECT")) { _selectedPatch = 0; [self showPatchUnfolding:atoi(getenv("HUGE_NATIVE_INSPECT")) > 1]; }   // testing hook: 1 inspect, 2 unfold again
        if (getenv("HUGE_NATIVE_MATCH")) [self matchOriginalLayout:nil];   // testing hook
        if (getenv("HUGE_NATIVE_MASK")) [self exportMask:nil];
        if (getenv("HUGE_NATIVE_SAVEBAD")) [self saveBadPatches:nil];   // testing hook
        if (getenv("HUGE_NATIVE_PATCHES")) {   // testing hook: the patch table, the paint modes and a selection
            const std::vector<PatchRow> rows = _useHuge ? _huge.patchRows() : _session.patchRows();
            size_t bad = 0;
            for (const PatchRow& r : rows) bad += r.bad;
            printf("NATIVE PATCHES: %zu rows, %zu bad; first: #%d %zu faces %s L2 %.3f %s\n", rows.size(), bad, rows.empty() ? -1 : rows[0].index,
                   rows.empty() ? size_t(0) : rows[0].faces, rows.empty() ? "" : rows[0].method.c_str(), rows.empty() ? 0.0 : rows[0].l2Mean,
                   rows.empty() ? "" : rows[0].status.c_str());
            if (!_useHuge) printf("NATIVE INFO:\n%s\n", _session.meshInfo().c_str());
            _selectedPatch = 0;
            _uv.selectedPatch = 0;
            [self recolorPatches];
        }
        if (!keepView) {
            if (const char* r = getenv("HUGE_NATIVE_REPACK")) {   // "resolution:mip": pack again as the button does, then look at the result
                int res = 0, mip = 0;
                if (sscanf(r, "%d:%d", &res, &mip) == 2) {
                    [_textureSize selectItemWithTag:res];
                    [self updateMipLabel];
                    _mip.integerValue = mip;
                    [self updateMipLabel];
                    std::string err;
                    const SessionSettings s = [self settings];
                    const bool ok = _useHuge ? _huge.repack(s, nullptr, &err) : _session.repack(s, &err);
                    printf("NATIVE REPACK: %s %s\n", ok ? "ok" : "failed", err.c_str());
                    if (ok) { [self showResultKeepingView:YES]; return; }
                }
            }
        }
        if (const char* v = getenv("HUGE_NATIVE_VIEW")) [self setViewMode:atoi(v)];
        if (const char* o = getenv("HUGE_NATIVE_OFFSET")) _offset.doubleValue = atof(o);
        if (const char* al = getenv("HUGE_NATIVE_ALPHA")) _alpha.doubleValue = atof(al);
        if (const char* p1 = getenv("HUGE_NATIVE_PASS1")) [_pass1 selectItemAtIndex:atoi(p1)];
        if (const char* p2 = getenv("HUGE_NATIVE_PASS2")) [_pass2 selectItemAtIndex:atoi(p2)];
        [self presentationChanged:nil];
        if (const char* out = getenv("HUGE_NATIVE_SAVE")) {   // the same as Save As, to a given path
            std::string err;
            const bool ok = _useHuge ? _huge.save(out, &err) : _session.save(out, &err);
            printf("NATIVE SAVE: %s %s\n", ok ? "ok" : "failed", err.c_str());
        }
        if (getenv("HUGE_NATIVE_COMPARE")) { [self compareLayouts:nil]; return; }   // testing hook: the comparison, printed; it quits when done
        [self autoShotAndQuit];
    }
}

- (void)saveAs:(id)sender
{
    if (![self hasResult]) return;
    std::string stem = _useHuge ? _huge.path() : _session.path();
    const size_t slash = stem.find_last_of('/');
    if (slash != std::string::npos) stem = stem.substr(slash + 1);
    const size_t dot = stem.find_last_of('.');
    if (dot != std::string::npos) stem.resize(dot);
    const std::string p = _useHuge
        ? native::saveFile("Save the mesh with the atlas (PLY, the texture goes next to it)", stem + "_atlas.ply", {"ply"}, "")
        : native::saveFile("Save the mesh with the atlas (OBJ or PLY, the texture goes next to it)", stem + "_atlas.obj", {"obj", "ply"}, "");
    if (p.empty()) return;
    std::string err;
    const BOOL ok = _useHuge ? _huge.save(p, &err) : _session.save(p, &err);
    _status.stringValue = ok ? [NSString stringWithFormat:@"Saved %@ and its texture", [NSString stringWithUTF8String:p.c_str()].lastPathComponent]
                             : [NSString stringWithFormat:@"Cannot save: %s", err.c_str()];
}

// the views: 0 patches, 1 plain shading, 2 presentation, 3 texture (the segments of the toolbar are in the same order)
- (void)setViewMode:(int)mode
{
    const BOOL result = [self hasResult];
    [_viewSwitch setEnabled:result forSegment:0];
    [_viewSwitch setEnabled:result forSegment:2];
    [_viewSwitch setEnabled:[self hasTextureView] forSegment:3];
    if ((mode == 0 || mode == 2) && !result) mode = 1;
    if (mode == 3 && ![self hasTextureView]) mode = result ? 0 : 1;
    _mode = mode;
    _viewSwitch.selectedSegment = mode;
    _presentGroup.hidden = mode != 2;
    [_scene setMode:mode];
}
- (void)toggleUVLayout:(id)sender { [_uvItem.animator setCollapsed:!_uvItem.collapsed]; }
- (void)showPatches:(id)sender { [self setViewMode:0]; }
- (void)showPlain:(id)sender { [self setViewMode:1]; }
- (void)showPresentation:(id)sender { [self setViewMode:2]; }
- (void)showTexture:(id)sender { [self setViewMode:3]; }
- (void)viewSwitched:(NSSegmentedControl*)s { [self setViewMode:int(s.selectedSegment)]; }
- (void)resetCamera:(id)sender { [_scene resetCamera]; }
- (void)showSettings:(id)sender { showSettingsWindow(0); }
- (void)toggleGrid:(id)sender
{
    const BOOL on = ![_scene showGrid];
    [_scene setShowGrid:on];
    [NSUserDefaults.standardUserDefaults setBool:on forKey:@"showGrid"];
}

- (void)toggleWireframe:(id)sender { _wire = !_wire; [_scene setWireframe:_wire]; }

// the preferences changed (the Settings window): the developer mode, the defaults of the sidebar
- (void)prefsChanged:(NSNotification*)n
{
    const prefs::Values old = _prefs;
    _prefs = prefs::load();
    [self applyDeveloperMode];
    if (_prefs.quality != old.quality) [_quality selectItemAtIndex:_prefs.quality];
    if (_prefs.textureSize != old.textureSize) { [_textureSize selectItemWithTag:_prefs.textureSize]; _textureTouched = YES; }
    if (_prefs.mipLevels != old.mipLevels) _mip.integerValue = _prefs.mipLevels;
    [self updateMipLabel];
}

- (void)applyDeveloperMode
{
    _developer = _prefs.developer;
    _dev.view.hidden = !_developer;
    _debugMenuItem.hidden = !_developer;
    if (!_developer && _wire) { _wire = NO; [_scene setWireframe:NO]; }
}

// the unfolding of the patch chosen in the table, in the UV pane; with the options of the panel when `again`
- (void)showPatchUnfolding:(BOOL)again
{
    if (_running || ![self hasResult]) return;
    if (_useHuge) { showTextWindow(@"Inspect Patch", @"A big scan does not keep the patches as meshes: this works in the regular mode."); return; }
    if (_selectedPatch < 0) { _status.stringValue = @"Choose a patch in the table first"; return; }
    UvChart chart;
    std::string text;
    const DeveloperOptions o = [_dev options];
    const BOOL ok = _session.inspectPatch(int(_selectedPatch), again ? &o : nullptr, &chart, &text);
    NSString* line = [NSString stringWithUTF8String:text.c_str()];
    _status.stringValue = ok ? @"Patch unfolding shown in the UV pane" : line;
    [_dev setReport:line];
    if (getenv("HUGE_NATIVE_INSPECT")) printf("NATIVE INSPECT:\n%s\n", text.c_str());
    if (ok) {
        chart.patch = int(_selectedPatch);
        [_uv setCharts:{chart} texture:nullptr side:0];
        [_uvItem.animator setCollapsed:NO];
    }
}
- (void)inspectPatch:(id)sender { [self showPatchUnfolding:NO]; }
- (void)unfoldPatchAgain:(id)sender { [self showPatchUnfolding:YES]; }

// the texture size and the mip level of the sidebar from the layout of the file
- (void)matchOriginalLayout:(id)sender
{
    if (_running || ![self hasMeshToWork]) return;
    int res = 0, mip = 0;
    std::string text;
    if (!(_useHuge ? _huge.matchOriginal(&res, &mip, &text) : _session.matchOriginal(&res, &mip, &text))) {
        _status.stringValue = [NSString stringWithUTF8String:text.c_str()];
        return;
    }
    [_textureSize selectItemWithTag:res];
    _textureTouched = YES;
    _mip.integerValue = mip;
    [self updateMipLabel];
    NSString* line = [NSString stringWithUTF8String:text.c_str()];
    _status.stringValue = line;
    [_dev setReport:line];
    if (getenv("HUGE_NATIVE_MATCH")) printf("NATIVE MATCH: %s\n", text.c_str());
}

// the coverage of the atlas as a PNG (the alpha channel)
- (void)exportMask:(id)sender
{
    if (_running || ![self hasResult]) return;
    if (_useHuge) { showTextWindow(@"Export Atlas Mask", @"The mask is made for the regular mode only; a big scan keeps just the borders of its charts."); return; }
    std::string stem = _session.path();
    const size_t slash = stem.find_last_of('/');
    if (slash != std::string::npos) stem = stem.substr(slash + 1);
    const size_t dot = stem.find_last_of('.');
    if (dot != std::string::npos) stem.resize(dot);
    const std::string p = getenv("HUGE_NATIVE_MASK") ? std::string(getenv("HUGE_NATIVE_MASK")) : native::saveFile("Export the occupancy mask (alpha channel)", stem + "_mask.png", {"png"}, "");
    if (p.empty()) return;
    std::string err;
    _status.stringValue = _session.writeMask(p, &err) ? [NSString stringWithFormat:@"Mask written: %@", [NSString stringWithUTF8String:p.c_str()].lastPathComponent]
                                                      : [NSString stringWithFormat:@"Cannot write the mask: %s", err.c_str()];
}

// the measures of the original layout and of the new atlas, in a window of text (the measuring takes a while on a big scan)
- (void)compareLayouts:(id)sender
{
    if (_running || ![self hasResult]) return;
    _dev.compareButton.enabled = NO;
    _status.stringValue = @"Measuring the layouts…";
    __weak AppController* weakSelf = self;
    std::thread([self, weakSelf]() {
        std::string err;
        const std::string text = _useHuge ? _huge.compare(&err) : _session.compare(&err);
        dispatch_async(dispatch_get_main_queue(), ^{
            AppController* me = weakSelf;
            if (!me) return;
            me->_dev.compareButton.enabled = YES;
            if (text.empty()) { me->_status.stringValue = [NSString stringWithFormat:@"Cannot compare: %s", err.c_str()]; return; }
            me->_status.stringValue = @"Done";
            showTextWindow(@"Original Layout and New Atlas", [NSString stringWithUTF8String:text.c_str()]);
            if (getenv("HUGE_NATIVE_AUTORUN")) { printf("NATIVE COMPARE:\n%s", text.c_str()); [me autoShotAndQuit]; }
        });
    }).detach();
}

// ---- the assistant: a model that drives the same actions as the buttons
- (void)toggleAssistant:(id)sender { [_chatItem.animator setCollapsed:!_chatItem.collapsed]; }

- (void)chatSetup { showAssistantSetup(_window, ^{ [self->_chat appendRole:@"note" text:@"The provider is saved."]; }); }

- (void)chatStop
{
    _chatStop = true;
    [self hostCancel];
}

- (void)chatSend:(NSString*)text
{
    if (_chatBusy) return;
    ChatConfig config = loadChatConfig();
    if (const char* u = getenv("HUGE_NATIVE_CHAT_URL")) { config.baseUrl = u; config.model = getenv("HUGE_NATIVE_CHAT_MODEL") ?: "test"; }   // testing hook
    if (!config.ready()) {
        [_chat appendRole:@"note" text:@"Set up the model provider first (Set Up…): the address, the model and the key."];
        [self chatSetup];
        return;
    }
    [_chat appendRole:@"user" text:text];
    _chatBusy = YES;
    [_chat setBusy:YES];
    _chatStop = false;
    if (_chatThread.joinable()) _chatThread.join();
    __weak AppController* weakSelf = self;
    const std::string message = text.UTF8String;
    _chatThread = std::thread([self, weakSelf, config, message]() {
        std::unique_ptr<AtlasHost> host = makeNativeHost(self);
        auto onMain = [weakSelf](NSString* role, NSString* line) {
            dispatch_async(dispatch_get_main_queue(), ^{ AppController* me = weakSelf; if (me) [me->_chat appendRole:role text:line]; });
        };
        ChatEvents ev;
        ev.assistant = [&](const std::string& t) { onMain(@"assistant", [NSString stringWithUTF8String:t.c_str()]); };
        ev.toolCall = [&](const std::string& name, const Json& args) {
            const std::string a = args.dump();
            onMain(@"tool", [NSString stringWithFormat:@"%s %s", name.c_str(), a == "{}" ? "" : a.c_str()]);
        };
        ev.toolResult = [&](const std::string& name, const Json& r) {
            if (r.has("error")) onMain(@"tool", [NSString stringWithFormat:@"→ %s", r.getString("error").c_str()]);
        };
        std::string err;
        const bool ok = _chatSession.converse(message, config.model, makeHttpTransport(config, &_chatStop), *host, ev, &_chatStop, &err);
        dispatch_async(dispatch_get_main_queue(), ^{
            AppController* me = weakSelf;
            if (!me) return;
            me->_chatBusy = NO;
            [me->_chat setBusy:NO];
            if (getenv("HUGE_NATIVE_CHAT")) { if (!ok) printf("CHAT FAILED: %s\n", err.c_str()); [me autoShotAndQuit]; return; }
            if (!ok) [me->_chat appendRole:@"note" text:err == "cancelled" ? @"Stopped." : [NSString stringWithFormat:@"The assistant stopped: %s", err.c_str()]];
        });
    });
}

static Json errorJson(NSString* message) { Json e = Json::object(); e["error"] = message.UTF8String; return e; }

// the numbers of the last run (main thread)
- (Json)atlasMetrics
{
    Json m = Json::object();
    m["mode"] = _useHuge ? "huge" : "regular";
    m["patches"] = _useHuge ? _huge.patchCount() : _session.patchCount();
    m["unresolved_patches"] = _useHuge ? _huge.unresolvedPatches() : size_t(std::max(0, _session.unresolvedPatches()));
    m["faces_without_uv"] = _useHuge ? _huge.facesWithoutUV() : size_t(0);
    m["texture_size"] = _useHuge ? _huge.resolution() : _session.resolution();
    m["efficiency"] = _useHuge ? _huge.efficiency() : _session.efficiency();
    m["gutter_texels"] = _useHuge ? _huge.gutter() : _session.gutter();
    m["seconds"] = _useHuge ? _huge.seconds() : _session.seconds();
    m["has_texture"] = [self hasBaked] ? true : false;
    return m;
}

- (Json)hostOpen:(NSString*)path
{
    __block Json result;
    __block BOOL started = NO;
    dispatch_semaphore_t signal = dispatch_semaphore_create(0);
    dispatch_sync(dispatch_get_main_queue(), ^{
        if (self->_running) { result = errorJson(@"a run is going on; wait for it or stop it"); return; }
        if (![NSFileManager.defaultManager fileExistsAtPath:path]) { result = errorJson(@"no such file"); return; }
        self->_openSignal = signal;
        self->_lastError = nil;
        started = YES;
        [self loadPath:path];
    });
    if (!started) return result;
    dispatch_semaphore_wait(signal, DISPATCH_TIME_FOREVER);
    dispatch_sync(dispatch_get_main_queue(), ^{ result = self->_lastError ? errorJson(self->_lastError) : [self hostState]; });
    return result;
}

- (SessionSettings)settingsFromArgs:(const Json&)a
{
    SessionSettings s = [self settings];
    if (a.has("quality")) { s.quality = std::max(0, std::min(2, a.getInt("quality"))); s.manualPatches = false; }
    if (a.has("patches")) { s.manualPatches = true; s.patches = std::max(1, a.getInt("patches")); }
    if (a.has("texture_size")) s.resolution = std::max(256, std::min(8192, a.getInt("texture_size")));
    if (a.has("mip_levels")) s.maxMipLevel = std::max(0, std::min(5, a.getInt("mip_levels")));
    if (a.has("smoothing")) s.dev.smoothing = a.getNumber("smoothing");
    if (a.has("normal_iterations")) s.dev.normalIterations = a.getInt("normal_iterations");
    if (a.has("normal_sigma")) s.dev.normalSigma = a.getNumber("normal_sigma");
    if (a.has("lloyd_iterations")) s.dev.lloydIterations = a.getInt("lloyd_iterations");
    if (a.has("max_depth")) s.dev.maxDepth = a.getInt("max_depth");
    if (a.has("allow_lscm")) s.dev.allowLscm = a.getBool("allow_lscm");
    if (a.has("repair_near_misses")) s.dev.repairNearMisses = a.getBool("repair_near_misses");
    if (a.has("simplify_percent")) s.dev.simplifyPercent = a.getNumber("simplify_percent");
    return s;
}

// what the assistant chose shows in the sidebar too
- (void)reflectSettings:(const SessionSettings&)s
{
    [_quality selectItemAtIndex:s.quality];
    _manual.state = s.manualPatches ? NSControlStateValueOn : NSControlStateValueOff;
    if (s.manualPatches) { _patchField.intValue = s.patches; _patchStepper.intValue = s.patches; }
    _patchField.enabled = s.manualPatches;
    _patchStepper.enabled = s.manualPatches;
    if (NSMenuItem* it = [_textureSize itemAtIndex:[_textureSize indexOfItemWithTag:s.resolution]]) [_textureSize selectItem:it];
    _textureTouched = YES;
    _mip.integerValue = s.maxMipLevel;
    [self updateMipLabel];
}

- (Json)hostMakeAtlas:(const Json&)args
{
    __block Json result;
    __block BOOL started = NO;
    dispatch_semaphore_t signal = dispatch_semaphore_create(0);
    const Json a = args;
    dispatch_sync(dispatch_get_main_queue(), ^{
        if (self->_running) { result = errorJson(@"a run is going on already"); return; }
        if (![self hasMeshToWork]) { result = errorJson(@"no mesh is open: open_mesh first"); return; }
        const SessionSettings s = [self settingsFromArgs:a];
        [self reflectSettings:s];
        self->_atlasSignal = signal;
        started = YES;
        [self makeAtlasWithSettings:s];
    });
    if (!started) return result;
    dispatch_semaphore_wait(signal, DISPATCH_TIME_FOREVER);
    dispatch_sync(dispatch_get_main_queue(), ^{ result = self->_lastOK ? [self atlasMetrics] : errorJson(self->_lastError ?: @"failed"); });
    return result;
}

- (Json)hostRepack:(int)size mip:(int)mip
{
    __block Json result;
    __block BOOL started = NO;
    dispatch_semaphore_t signal = dispatch_semaphore_create(0);
    dispatch_sync(dispatch_get_main_queue(), ^{
        if (self->_running) { result = errorJson(@"a run is going on already"); return; }
        if (![self hasResult]) { result = errorJson(@"there is no atlas yet: make_atlas first"); return; }
        SessionSettings s = [self settings];
        s.resolution = std::max(256, std::min(8192, size));
        if (mip >= 0) s.maxMipLevel = std::min(5, mip);
        [self reflectSettings:s];
        self->_atlasSignal = signal;
        started = YES;
        [self repackWithSettings:s];
    });
    if (!started) return result;
    dispatch_semaphore_wait(signal, DISPATCH_TIME_FOREVER);
    dispatch_sync(dispatch_get_main_queue(), ^{ result = self->_lastOK ? [self atlasMetrics] : errorJson(self->_lastError ?: @"failed"); });
    return result;
}

- (Json)hostState
{
    __block Json s = Json::object();
    void (^build)(void) = ^{
        const BOOL has = [self hasMeshToWork];
        s["mesh_open"] = has ? true : false;
        if (has) {
            s["file"] = (_useHuge ? _huge.path() : _session.path());
            s["faces"] = _useHuge ? (_huge.loaded() ? _huge.faceCount() : size_t(0)) : _session.mesh().n_faces();
            s["has_source_texture"] = _useHuge ? _huge.hasSourceTexture() : _session.hasSourceTexture();
            s["mode"] = _useHuge ? "huge" : "regular";
        }
        s["running"] = _running ? true : false;
        const SessionSettings cur = [self settings];
        Json set = Json::object();
        set["quality"] = cur.quality;
        set["patches"] = cur.manualPatches ? Json(cur.patches) : Json();
        set["texture_size"] = cur.resolution;
        set["mip_levels"] = cur.maxMipLevel;
        s["settings"] = set;
        s["atlas"] = [self hasResult] ? [self atlasMetrics] : Json();
    };
    if ([NSThread isMainThread]) build(); else dispatch_sync(dispatch_get_main_queue(), build);
    return s;
}

- (Json)hostLayout
{
    __block BOOL ok = NO;
    dispatch_sync(dispatch_get_main_queue(), ^{ ok = [self hasResult] && !self->_running; });
    if (!ok) return errorJson(@"there is no atlas (or a run is going on)");
    std::string err;
    LayoutComparison c;
    if (!(_useHuge ? _huge.compareData(&c, &err) : _session.compareData(&c, &err))) return errorJson([NSString stringWithUTF8String:err.c_str()]);
    return layoutToJson(c);
}

- (Json)hostReport
{
    __block std::string text;
    dispatch_sync(dispatch_get_main_queue(), ^{ text = [self hasResult] ? (self->_useHuge ? self->_huge.report() : self->_session.report()) : std::string(); });
    if (text.empty()) return errorJson(@"there is no atlas yet");
    Json r = Json::object();
    r["report"] = text;
    return r;
}

- (Json)hostSave:(NSString*)name
{
    __block BOOL ok = NO;
    dispatch_sync(dispatch_get_main_queue(), ^{ ok = [self hasResult] && !self->_running; });
    if (!ok) return errorJson(@"there is no atlas (or a run is going on)");
    std::string file = chatSafeFileName(name.UTF8String);
    if (_useHuge && file.size() > 4 && file.compare(file.size() - 4, 4, ".obj") == 0) file.replace(file.size() - 4, 4, ".ply");   // the huge mode writes PLY
    const std::string path = chatOutputDirectory() + "/" + file;
    std::string err;
    if (!(_useHuge ? _huge.save(path, &err) : _session.save(path, &err))) return errorJson([NSString stringWithUTF8String:err.c_str()]);
    Json r = Json::object();
    r["saved"] = path;
    return r;
}

// the view options of the developer panel: anti-aliasing, back faces, the UV checker
- (void)applyViewOptions
{
    [_scene setMsaa:_dev.msaa];
    [_scene setCull:_dev.cullBackFaces];
    const BOOL checker = _dev.uvChecker && [self hasResult] && !_running;
    if (checker != _checkerOn) {
        _checkerOn = checker;
        [self showResultKeepingView:YES];   // the renderer gets the atlas coordinates (or the texture ones again)
    }
    [self updateCheckerCells];
}

- (void)updateCheckerCells
{
    float cells = 0.0f;
    if (_checkerOn && [self hasResult]) cells = atlasCheckerCells(_useHuge ? _huge.atlas() : _session.atlas(), _dev.checkerSquares);
    [_scene setCheckerCells:cells];
}

// the patch of a face of the result (-1: none, or no result)
- (NSInteger)patchOfFace:(int)face
{
    if (face < 0 || ![self hasResult] || _running) return -1;
    if (_useHuge) { const std::vector<int32_t>& of = _huge.patchOfFace(); return size_t(face) < of.size() ? of[size_t(face)] : -1; }
    const std::vector<int>& of = _session.faceToPatch();
    return size_t(face) < of.size() ? of[size_t(face)] : -1;
}

// One patch is chosen (-1: none): in the table, white in the 3D view and heavier in the UV pane, whoever chose it: a row, a click in the
// 3D view or a click in the UV pane
- (void)selectPatch:(NSInteger)patch { [self selectPatch:patch focus:NO]; }

- (void)selectPatch:(NSInteger)patch focus:(BOOL)focus
{
    if (![self hasResult] || _running) patch = -1;
    _selectedPatch = patch;
    _uv.selectedPatch = patch;
    [_dev selectPatchIndex:patch];
    [self recolorPatches];
    if (patch < 0) return;
    BOOL found = NO;
    const PatchRow r = [_dev rowForPatch:patch found:&found];
    if (found) _status.stringValue = [NSString stringWithFormat:@"Patch %ld: %zu faces, %s, stretch %.2f, %s", long(patch), r.faces, r.method.c_str(), r.l2Mean, r.status.c_str()];
    if (focus && _lookAtPatch) [self lookAtSelectedPatch:nil];
}

// the camera on the ray out of the middle of the patch, looking back at it
- (void)lookAtSelectedPatch:(id)sender
{
    if (_selectedPatch < 0 || ![self hasResult] || _running) return;
    PatchFrame frame;
    if (!(_useHuge ? _huge.patchFrame(int(_selectedPatch), &frame) : _session.patchFrame(int(_selectedPatch), &frame))) return;
    [_scene lookAtCentre:frame.centre normal:frame.normal points:frame.points animated:!getenv("HUGE_NATIVE_SHOT")];
}

- (void)toggleLookAtPatch:(id)sender
{
    _lookAtPatch = !_lookAtPatch;
    [NSUserDefaults.standardUserDefaults setBool:_lookAtPatch forKey:@"lookAtPatch"];
}

// the patches painted by the mode of the table (patch colors, problems, stretch); the chosen patch is white
- (void)recolorPatches
{
    if (![self hasResult] || _running) return;
    const std::vector<PatchRow> rows = _useHuge ? _huge.patchRows() : _session.patchRows();
    const NSInteger mode = _dev.colorMode;
    auto rgbOf = [&](size_t i, float* c) {
        const PatchRow& r = rows[i];
        if (int(i) == _selectedPatch) { c[0] = c[1] = c[2] = 1.0f; return; }
        if (mode == 1) { if (r.bad) { c[0] = 1.0f; c[1] = 0.25f; c[2] = 0.2f; } else { c[0] = 0.72f; c[1] = 0.76f; c[2] = 0.8f; } return; }
        if (mode == 2) {
            const float t = float(std::max(0.0, std::min(1.0, (r.l2Mean - 1.0) / 1.5)));
            c[0] = 0.2f + 0.8f * t; c[1] = 0.5f * (1.0f - t) + 0.15f; c[2] = 0.95f * (1.0f - t) + 0.15f;
            return;
        }
        AtlasSession::patchColor(int(i), c);
    };
    if (_useHuge) {
        const std::vector<int32_t>& of = _huge.patchOfFace();
        std::vector<unsigned char> rgba(of.size() * 4, 190);
        std::vector<std::array<unsigned char, 3>> table(rows.size());
        for (size_t i = 0; i < rows.size(); ++i) {
            float c[3];
            rgbOf(i, c);
            for (int k = 0; k < 3; ++k) table[i][size_t(k)] = (unsigned char)(255.0f * std::max(0.0f, std::min(1.0f, c[k])));
        }
        for (size_t f = 0; f < of.size(); ++f) {
            rgba[4 * f + 3] = 255;
            if (of[f] >= 0 && size_t(of[f]) < table.size()) { rgba[4 * f] = table[size_t(of[f])][0]; rgba[4 * f + 1] = table[size_t(of[f])][1]; rgba[4 * f + 2] = table[size_t(of[f])][2]; }
            else { rgba[4 * f] = rgba[4 * f + 1] = rgba[4 * f + 2] = 190; }
        }
        [_scene setFaceRgba:rgba];
    } else {
        const std::vector<int>& of = _session.faceToPatch();
        std::vector<float> rgb(of.size() * 3, 0.72f);
        std::vector<std::array<float, 3>> table(rows.size());
        for (size_t i = 0; i < rows.size(); ++i) rgbOf(i, table[i].data());
        for (size_t f = 0; f < of.size(); ++f)
            if (of[f] >= 0 && size_t(of[f]) < table.size()) for (int k = 0; k < 3; ++k) rgb[3 * f + size_t(k)] = table[size_t(of[f])][size_t(k)];
        [_scene setFaceColors:rgb];
    }
    [_uv setNeedsDisplay:YES];
}

- (void)hostCancel { dispatch_async(dispatch_get_main_queue(), ^{ if (self->_running) [self cancelAtlas:nil]; }); }

// ---- Debug: the bad patches. Save writes every patch that is not a disk, overlaps itself or is too stretched (an OBJ with the faulty uv and a
// preset next to it) into DebugOut/BadPatch; Load takes such a file back: it is unfolded again with its preset and the problems are shown.
// the segmentation by hand: its window, its colors in the 3D view, its brush; the regions can start the atlas
- (void)showVsaLab:(id)sender
{
    if (!_lab) {
        _lab = [[VsaLab alloc] init];
        __weak AppController* weak = self;
        _lab.onColors = ^(const std::vector<float>& rgb) { AppController* me = weak; if (me && !me->_useHuge) { [me->_scene setFaceColors:rgb]; [me->_scene setMode:0]; } };
        _lab.onBrush = ^(BOOL on, int radius) { AppController* me = weak; if (me) { me->_scene.brushActive = on; me->_scene.brushRadius = radius; } };
        _scene.onBrush = ^(const std::vector<int>& faces) { AppController* me = weak; if (me) [me->_lab brushFaces:faces]; };
        _scene.onBrushEnd = ^{ AppController* me = weak; if (me) [me->_lab brushEnded]; };
        _lab.onUseLabels = ^(const std::vector<int>& labels) {
            AppController* me = weak;
            if (!me) return;
            me->_session.setStartLabels(labels);
            me->_status.stringValue = @"The regions of the lab will start the atlas: press Make Atlas";
        };
    }
    [_lab attachMesh:(!_useHuge && _session.hasMesh() && !_session.badPatchLoaded()) ? &_session.mesh() : nullptr];
    [_lab showWindow];
}

- (void)saveBadPatches:(id)sender
{
    if (_running || ![self hasResult]) return;
    const std::string folder = paths::debugOutDir() + "/BadPatch";
    const SessionSettings s = [self settings];
    const BadPatchReport r = _useHuge ? ::saveBadPatches(_huge.mesh(), _huge.result(), _huge.preset(s), folder, -1)
                                      : ::saveBadPatches(_session.params(), nullptr, _session.preset(s), folder, -1);
    NSString* text = [NSString stringWithUTF8String:describe(r).c_str()];
    _status.stringValue = text;
    if (getenv("HUGE_NATIVE_SAVEBAD")) printf("NATIVE BAD: %s\n", text.UTF8String);
    else if (r.saved > 0) [NSWorkspace.sharedWorkspace activateFileViewerSelectingURLs:@[[NSURL fileURLWithPath:[NSString stringWithUTF8String:r.folder.c_str()]]]];
    else if (!r.error.empty() || r.bad == 0) showTextWindow(@"Save Bad Patches", text);
}

- (void)loadBadPatch:(id)sender
{
    if (_running) return;
    const std::string start = paths::debugOutDir() + "/BadPatch";
    const std::string p = native::openFile("Load a bad patch (OBJ saved by Save Bad Patches)", {"obj"}, start);
    if (p.empty()) return;
    [self loadBadPatchPath:[NSString stringWithUTF8String:p.c_str()]];
}

- (void)loadBadPatchPath:(NSString*)path
{
    std::string text, err;
    AtlasSession next;
    if (!next.loadBadPatch(path.UTF8String, &text, &err)) {
        _status.stringValue = [NSString stringWithFormat:@"Cannot load: %s", err.c_str()];
        return;
    }
    if (_lab) [_lab attachMesh:nullptr];
    _session = std::move(next);
    _useHuge = NO;
    _huge.clear();
    _window.title = path.lastPathComponent;
    _window.representedURL = [NSURL fileURLWithPath:path];
    _meshLabel.stringValue = [NSString stringWithFormat:@"%@ — %zu faces (a bad patch)", path.lastPathComponent, _session.mesh().n_faces()];
    _result.stringValue = @"";
    _sourceTextureShown = NO;
    [_scene resetCamera];
    [_scene showMesh:_session.mesh() colors:_session.badPatchColors() faceUV:{}];
    [_scene setTexture:0 pixels:nullptr];
    [_scene setMode:0];   // the colors: blue where the stretch is 1, red where it is bad
    _mode = 0;
    if (_session.badPatchLoaded()) {
        [_uv setCharts:{_session.badPatchChart()} texture:nullptr side:0];
        [_uvItem.animator setCollapsed:NO];
    } else {
        [_uv clear];
    }
    NSString* line = [NSString stringWithUTF8String:text.c_str()];
    _status.stringValue = line;
    [_dev setReport:line];
    [_dev setInfo:[NSString stringWithUTF8String:_session.meshInfo().c_str()]];
    [_dev setPatchRows:{}];
    [self updateEnabled];
    printf("%s\n", text.c_str());
}

// ---- a picture of the own window for the checks without a person (own window: needs no screen recording permission)
- (void)autoShotAndQuit
{
    const char* shot = getenv("HUGE_NATIVE_SHOT");
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 1500 * NSEC_PER_MSEC), dispatch_get_main_queue(), ^{
        if (shot) {
            CGImageRef img = CGWindowListCreateImage(CGRectNull, kCGWindowListOptionIncludingWindow, CGWindowID((getenv("HUGE_NATIVE_SETTINGS") && settingsWindow() ? settingsWindow() : self->_window).windowNumber),
                                                     kCGWindowImageBoundsIgnoreFraming);
            if (img) {
                NSBitmapImageRep* rep = [[NSBitmapImageRep alloc] initWithCGImage:img];
                [[rep representationUsingType:NSBitmapImageFileTypePNG properties:@{}] writeToFile:[NSString stringWithUTF8String:shot] atomically:YES];
                CGImageRelease(img);
            }
        }
        [NSApp terminate:nil];
    });
}

@end

namespace {

class NativeHost : public AtlasHost {
public:
    explicit NativeHost(AppController* c) : c_(c) {}
    Json openMesh(const std::string& path) override { return [c_ hostOpen:[NSString stringWithUTF8String:path.c_str()]]; }
    Json makeAtlas(const Json& s) override { return [c_ hostMakeAtlas:s]; }
    Json repack(int size, int mip) override { return [c_ hostRepack:size mip:mip]; }
    Json state() override { return [c_ hostState]; }
    Json layout() override { return [c_ hostLayout]; }
    Json report() override { return [c_ hostReport]; }
    Json save(const std::string& name) override { return [c_ hostSave:[NSString stringWithUTF8String:name.c_str()]]; }
private:
    AppController* c_;
};

} // namespace

std::unique_ptr<AtlasHost> makeNativeHost(AppController* controller) { return std::make_unique<NativeHost>(controller); }

int main(int argc, const char* argv[])
{
    @autoreleasepool {
        NSApplication* app = [NSApplication sharedApplication];
        app.activationPolicy = NSApplicationActivationPolicyRegular;
        AppController* controller = [[AppController alloc] init];
        app.delegate = controller;
        [app run];
    }
    return 0;
}
