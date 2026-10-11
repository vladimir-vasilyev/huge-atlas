// huge-atlas, the native shell (a prototype): an AppKit window with a sidebar of settings, a toolbar, a menu bar and a 3D view. The
// work is done by AtlasSession (the same library as the ImGui build); the rules of the Human Interface Guidelines for the Mac that
// this window follows are in docs/native-ui.md.
#import <Cocoa/Cocoa.h>
#import <OpenGL/OpenGL.h>
#import <QuartzCore/QuartzCore.h>

#include "render/GL.h"
#include "render/Camera.h"
#include "render/MeshRenderer.h"
#include "render/GridRenderer.h"
#include "app/NativeUI.h"
#include "native/HugeSession.h"
#include "native/Session.h"

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
    BOOL _hasTexture;
    BOOL _cull;             // big meshes: the back faces are not drawn
    float _offset, _alpha;  // presentation: how far the translucent copies of the patches move, how opaque they are
    int _pass1, _pass2;     // presentation: what a pass shows (0 patch colors, 1 texture, 2 shading); -1 = decided by the mesh
    NSPoint _last;
}
@property (nonatomic, copy) void (^onDropURL)(NSURL*);
- (void)showMesh:(const TriMesh&)mesh colors:(const std::vector<float>&)colors faceUV:(const std::vector<float>&)uv;
- (void)showFlat:(const FlatMesh&)mesh cornerUV:(const std::vector<float>&)uv rgba:(const std::vector<unsigned char>&)rgba;
- (void)clearMesh;
- (void)setTexture:(int)side pixels:(const unsigned char*)pixels;
- (void)setPatchNormals:(const std::vector<float>&)normals;
- (void)setMode:(int)mode;
- (void)setOffset:(float)offset alpha:(float)alpha;
- (void)setPass1:(int)p1 pass2:(int)p2;
- (void)resetCamera;
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
    if (!_hasMesh) _camera.reset();   // from the grid view to the model
    _hasMesh = YES;
    _cull = NO;
    [self setNeedsDisplay:YES];
}

- (void)showFlat:(const FlatMesh&)mesh cornerUV:(const std::vector<float>&)uv rgba:(const std::vector<unsigned char>&)rgba
{
    [self ensureReady];
    _renderer.uploadFlat(mesh, uv, rgba);
    if (!_hasMesh) _camera.reset();
    _hasMesh = YES;
    _cull = mesh.faceCount() > 500000;   // as in the other build: a scan is drawn without its back faces
    [self setNeedsDisplay:YES];
}

- (void)clearMesh { _hasMesh = NO; _hasTexture = NO; _camera.resetAbove(); [self setNeedsDisplay:YES]; }

- (void)setTexture:(int)side pixels:(const unsigned char*)pixels
{
    [self ensureReady];
    _hasTexture = side > 0 && pixels && _renderer.setTexture(side, side, pixels);
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
- (void)resetCamera { if (_hasMesh) _camera.reset(); else _camera.resetAbove(); [self setNeedsDisplay:YES]; }

- (void)drawRect:(NSRect)dirty
{
    [self ensureReady];
    const NSRect b = [self convertRectToBacking:self.bounds];
    glViewport(0, 0, GLsizei(b.size.width), GLsizei(b.size.height));
    __block float r = 0.12f, g = 0.12f, bl = 0.13f;
    [self.effectiveAppearance performAsCurrentDrawingAppearance:^{   // the color of the window follows light / dark
        NSColor* c = [[NSColor underPageBackgroundColor] colorUsingColorSpace:NSColorSpace.sRGBColorSpace];
        if (c) { r = float(c.redComponent); g = float(c.greenComponent); bl = float(c.blueComponent); }
    }];
    glClearColor(r, g, bl, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (_ready && !_hasMesh && b.size.height > 0) {
        float vp[16];
        _camera.viewProj(float(b.size.width / b.size.height), vp);
        _grid.draw(vp, 0.2126f * r + 0.7152f * g + 0.0722f * bl < 0.5f);
    }
    if (_ready && _hasMesh && b.size.height > 0) {
        float vp[16], v[16];
        _camera.viewProj(float(b.size.width / b.size.height), vp);
        _camera.view(v);
        const BOOL tex = _hasTexture;
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
            _renderer.draw(vp, v, false, 0.0f, _mode == 3 && tex, _cull, _mode == 1);
        }
    }
    [self.openGLContext flushBuffer];
}

// ---- mouse: the left button turns, the right or middle one moves, the wheel and the pinch zoom towards the cursor
- (NSPoint)local:(NSEvent*)e
{
    const NSPoint p = [self convertPoint:e.locationInWindow fromView:nil];
    return NSMakePoint(p.x, self.bounds.size.height - p.y);   // y down, as the camera expects
}

- (void)mouseDown:(NSEvent*)e
{
    [self.window makeFirstResponder:self];
    if (e.clickCount == 2) { [self resetCamera]; return; }
    const NSPoint p = [self local:e];
    _last = p;
    _camera.beginRotate(p.x, p.y, int(self.bounds.size.width), int(self.bounds.size.height));
}

- (void)mouseDragged:(NSEvent*)e
{
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
- (void)scrollWheel:(NSEvent*)e { [self zoom:(e.hasPreciseScrollingDeltas ? e.scrollingDeltaY * 0.1 : e.scrollingDeltaY) at:e]; }
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
}
- (void)setCharts:(std::vector<UvChart>)charts texture:(const unsigned char*)rgba side:(int)side;
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
        CGContextSetRGBFillColor(g, rgb[0], rgb[1], rgb[2], _image ? 0.10 : 0.55);
        CGContextAddPath(g, fill);
        CGContextFillPath(g);
        CGPathRelease(fill);
        CGContextSetRGBStrokeColor(g, rgb[0], rgb[1], rgb[2], 0.95);
        CGContextSetLineWidth(g, 1.5);
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
    }
    CGContextRestoreGState(g);
    [[NSColor.labelColor colorWithAlphaComponent:0.35] setStroke];
    NSFrameRect(sq);
}

- (BOOL)acceptsFirstResponder { return YES; }
- (void)mouseDown:(NSEvent*)e
{
    if (e.clickCount == 2) { _zoom = 1.0; _pan = NSZeroPoint; [self setNeedsDisplay:YES]; return; }
    _last = [self convertPoint:e.locationInWindow fromView:nil];
}
- (void)mouseDragged:(NSEvent*)e
{
    const NSPoint p = [self convertPoint:e.locationInWindow fromView:nil];
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

@interface AppController : NSObject <NSApplicationDelegate, NSToolbarDelegate, NSToolbarItemValidation, NSMenuItemValidation, NSWindowDelegate> {
    NSWindow* _window;
    NSSplitViewController* _split;
    NSSplitViewItem* _uvItem;
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
    BOOL _useHuge;              // the mesh is a big scan: it is read by the huge mode when the atlas is made
    BOOL _textureTouched;       // the user chose a texture size: it is not suggested from the size of the mesh any more
    NSButton* _repackButton;
    std::atomic<bool> _cancel;
    std::thread _worker;
    BOOL _running;
}
@end

@implementation AppController

- (BOOL)hasMeshToWork { return _useHuge ? !_huge.path().empty() : _session.hasMesh(); }
- (BOOL)hasResult { return _useHuge ? _huge.hasResult() : _session.hasResult(); }
- (BOOL)hasBaked { return _useHuge ? _huge.hasBakedTexture() : _session.hasBakedTexture(); }

- (SessionSettings)settings
{
    SessionSettings s;
    s.quality = int(_quality.indexOfSelectedItem);
    s.manualPatches = _manual.state == NSControlStateValueOn;
    s.patches = _patchField.intValue;
    s.resolution = int(_textureSize.selectedItem.tag);
    s.maxMipLevel = int(_mip.integerValue);
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
    NSButton* open = [NSButton buttonWithTitle:@"Open…" target:self action:@selector(openMesh:)];

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
    NSStackView* stack = [NSStackView stackViewWithViews:@[
        label(@"Mesh", 11, YES, YES), _meshLabel, open,
        label(@"Settings", 11, YES, YES), label(@"Quality", 12), _quality, _manual, patchRow,
        label(@"Texture size", 12), _textureSize, label(@"Mip levels", 12), _mip, _mipLabel,
        label(@"Atlas", 11, YES, YES), makeRow, _progress, _status,
        label(@"Result", 11, YES, YES), _result, _repackButton, _presentGroup]];
    stack.orientation = NSUserInterfaceLayoutOrientationVertical;
    stack.alignment = NSLayoutAttributeLeading;
    stack.spacing = 8;
    stack.edgeInsets = NSEdgeInsetsMake(52, 20, 20, 20);   // the top stays free for the toolbar (the sidebar goes under it)
    stack.translatesAutoresizingMaskIntoConstraints = NO;
    // group spacing: 20 between the groups
    for (NSView* v in @[_meshLabel]) (void)v;
    [stack setCustomSpacing:20 afterView:open];
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
    [self buildMenus];
    _scene = [[SceneView alloc] initWithFrame:NSMakeRect(0, 0, 800, 600)];
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
    NSViewController* uvVC = [[NSViewController alloc] init];
    uvVC.view = _uv;
    _uvItem = [NSSplitViewItem splitViewItemWithViewController:uvVC];
    _uvItem.minimumThickness = 280;
    _uvItem.canCollapse = YES;
    _uvItem.collapsed = YES;
    [_split addSplitViewItem:_uvItem];

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

    const char* arg = getenv("HUGE_NATIVE_OPEN");
    if (arg) {
        [self loadPath:[NSString stringWithUTF8String:arg]];
        if (getenv("HUGE_NATIVE_AUTORUN")) [self makeAtlas:nil];
        else [self autoShotAndQuit];
    } else if (getenv("HUGE_NATIVE_SHOT")) {
        [self autoShotAndQuit];
    }
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)app { return YES; }
- (void)application:(NSApplication*)app openURLs:(NSArray<NSURL*>*)urls { if (urls.count) [self loadPath:urls.firstObject.path]; }

// ---- toolbar
- (NSArray<NSToolbarItemIdentifier>*)toolbarDefaultItemIdentifiers:(NSToolbar*)t
{
    return @[NSToolbarToggleSidebarItemIdentifier, NSToolbarSidebarTrackingSeparatorItemIdentifier, @"open", @"makeAtlas",
             NSToolbarFlexibleSpaceItemIdentifier, @"views", @"save", @"uv"];
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
    NSMenuItem* settings = add(app, @"Settings…", @selector(showSettings:), @",", NSEventModifierFlagCommand, self);
    settings.enabled = NO;   // the Settings window comes with the next step; the item has its place and its shortcut already
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
    add(view, @"Show Sidebar", @selector(toggleSidebar:), @"s", NSEventModifierFlagCommand | NSEventModifierFlagControl);
    add(view, @"Enter Full Screen", @selector(toggleFullScreen:), @"f", NSEventModifierFlagCommand | NSEventModifierFlagControl);

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
    if (a == @selector(cancelAtlas:)) return _running && !_useHuge;   // the huge mode cannot be stopped in the middle
    if (a == @selector(repackAtlas:)) return !_running && [self hasResult];
    if (a == @selector(saveAs:)) return !_running && [self hasResult];
    if (a == @selector(showSettings:)) return NO;
    if (a == @selector(showPatches:)) { item.state = _mode == 0 ? NSControlStateValueOn : NSControlStateValueOff; return [self hasResult]; }
    if (a == @selector(showPlain:)) item.state = _mode == 1 ? NSControlStateValueOn : NSControlStateValueOff;
    if (a == @selector(showPresentation:)) { item.state = _mode == 2 ? NSControlStateValueOn : NSControlStateValueOff; return [self hasResult]; }
    if (a == @selector(showTexture:)) { item.state = _mode == 3 ? NSControlStateValueOn : NSControlStateValueOff; return [self hasBaked]; }
    if (a == @selector(toggleUVLayout:)) {
        item.title = _uvItem.collapsed ? @"Show UV Layout" : @"Hide UV Layout";
        return [self hasResult];
    }
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
    _repackButton.enabled = !_running && [self hasResult];
    [_window.toolbar validateVisibleItems];
}

- (void)loadPath:(NSString*)path
{
    std::string err;
    const size_t estimate = HugeSession::estimateFaces(path.UTF8String);
    if (estimate > 3000000) {   // a big scan is not loaded for viewing, the huge mode reads it straight from the file
        _useHuge = YES;
        _huge.setFile(path.UTF8String);
        _session = AtlasSession();
        _window.title = path.lastPathComponent;
        _window.representedURL = [NSURL fileURLWithPath:path];
        _meshLabel.stringValue = [NSString stringWithFormat:@"%@ — a big scan, about %.1f million faces", path.lastPathComponent, estimate / 1e6];
        _result.stringValue = @"";
        _status.stringValue = @"A big scan is read when the atlas is made";
        [_scene clearMesh];
        [_uv clear];
        _uvItem.collapsed = YES;
        [self suggestTextureSize:estimate];
        [self setViewMode:1];
        [self updateEnabled];
        return;
    }
    if (!_session.load(path.UTF8String, &err)) {
        _status.stringValue = [NSString stringWithFormat:@"Cannot open %@: %s", path.lastPathComponent, err.c_str()];
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
    [_scene showMesh:_session.mesh() colors:{} faceUV:{}];
    [_scene setTexture:0 pixels:nullptr];
    [_uv clear];
    _uvItem.collapsed = YES;
    [self suggestTextureSize:_session.mesh().n_faces()];
    [self setViewMode:1];
    [self updateEnabled];
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
    _cancelButton.hidden = !running || _useHuge;   // the huge mode cannot be stopped in the middle
    for (NSControl* c in @[(NSControl*)_quality, (NSControl*)_manual, (NSControl*)_textureSize, (NSControl*)_mip]) c.enabled = !running;
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

- (void)makeAtlas:(id)sender
{
    if (_running || ![self hasMeshToWork]) return;
    const SessionSettings s = [self settings];
    [self beginWork:@"Starting…"];
    __weak AppController* weakSelf = self;
    _worker = std::thread([self, weakSelf, s]() {
        std::string err;
        bool ok;
        if (_useHuge) {
            ok = _huge.run(s, [weakSelf](const std::string& what) {
                NSString* text = [NSString stringWithUTF8String:what.c_str()];
                dispatch_async(dispatch_get_main_queue(), ^{ AppController* me = weakSelf; if (me) me->_status.stringValue = text; });
            }, &err);
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
- (void)repackAtlas:(id)sender
{
    if (_running || ![self hasResult]) return;
    const SessionSettings s = [self settings];
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

- (void)cancelAtlas:(id)sender { if (_running && !_useHuge) { _cancel = true; _status.stringValue = @"Cancelling…"; } }

- (void)atlasFinished:(BOOL)ok error:(NSString*)error keepView:(BOOL)keepView
{
    if (_worker.joinable()) _worker.join();
    if (_useHuge) [_progress stopAnimation:nil];
    _progress.indeterminate = NO;
    [self setRunning:NO];
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
    NSString* text;
    if (_useHuge) {
        text = [NSString stringWithFormat:@"%zu patches\nAtlas %d × %d, %.0f%% of the texture is used\n%.1f s", _huge.patchCount(), _huge.resolution(),
                                          _huge.resolution(), 100.0 * _huge.efficiency(), _huge.seconds()];
        if (_huge.facesWithoutUV() > 0)
            text = [text stringByAppendingFormat:@"\n%zu faces could not be unfolded and have no texture coordinates", _huge.facesWithoutUV()];
        [_scene showFlat:_huge.mesh() cornerUV:(_huge.hasBakedTexture() ? _huge.cornerUV() : std::vector<float>()) rgba:_huge.patchRgba()];
        [_scene setTexture:(_huge.hasBakedTexture() ? _huge.bakedResolution() : 0) pixels:_huge.bakedPixels()];
        [_scene setPatchNormals:_huge.patchNormalsPerFace()];
        [_uv setCharts:_huge.uvCharts() texture:(_huge.hasBakedTexture() ? _huge.bakedPixels() : nullptr) side:_huge.bakedResolution()];
    } else {
        text = [NSString stringWithFormat:@"%zu patches\nAtlas %d × %d, %.0f%% of the texture is used\n%.1f s", _session.patchCount(), _session.resolution(),
                                          _session.resolution(), 100.0 * _session.efficiency(), _session.seconds()];
        if (_session.unresolvedPatches() > 0)
            text = [text stringByAppendingFormat:@"\n%d patches could not be unfolded cleanly", _session.unresolvedPatches()];
        [_scene showMesh:_session.mesh() colors:_session.patchColors() faceUV:(_session.hasBakedTexture() ? _session.atlasFaceUV() : std::vector<float>())];
        [_scene setTexture:(_session.hasBakedTexture() ? _session.bakedResolution() : 0) pixels:_session.bakedPixels()];
        [_scene setPatchNormals:_session.patchNormalsPerFace()];
        [_uv setCharts:_session.uvCharts() texture:(_session.hasBakedTexture() ? _session.bakedPixels() : nullptr) side:_session.bakedResolution()];
    }
    _result.stringValue = text;
    [_uvItem.animator setCollapsed:NO];
    [self setViewMode:keepView ? _mode : ([self hasBaked] ? 3 : 0)];
    [self updateEnabled];
    if (getenv("HUGE_NATIVE_AUTORUN")) {
        printf("NATIVE: %s%s\n", text.UTF8String, [self hasBaked] ? " (texture baked)" : "");
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
    [_viewSwitch setEnabled:[self hasBaked] forSegment:3];
    if ((mode == 0 || mode == 2) && !result) mode = 1;
    if (mode == 3 && ![self hasBaked]) mode = result ? 0 : 1;
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
- (void)showSettings:(id)sender {}

// ---- a picture of the own window for the checks without a person (own window: needs no screen recording permission)
- (void)autoShotAndQuit
{
    const char* shot = getenv("HUGE_NATIVE_SHOT");
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 1500 * NSEC_PER_MSEC), dispatch_get_main_queue(), ^{
        if (shot) {
            CGImageRef img = CGWindowListCreateImage(CGRectNull, kCGWindowListOptionIncludingWindow, CGWindowID(self->_window.windowNumber),
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
