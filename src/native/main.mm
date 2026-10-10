// huge-atlas, the native shell (a prototype): an AppKit window with a sidebar of settings, a toolbar, a menu bar and a 3D view. The
// work is done by AtlasSession (the same library as the ImGui build); the rules of the Human Interface Guidelines for the Mac that
// this window follows are in docs/native-ui.md.
#import <Cocoa/Cocoa.h>
#import <OpenGL/OpenGL.h>
#import <QuartzCore/QuartzCore.h>

#include "render/GL.h"
#include "render/Camera.h"
#include "render/MeshRenderer.h"
#include "app/NativeUI.h"
#include "native/Session.h"

#include <atomic>
#include <memory>
#include <thread>

// ------------------------------------------------------------------------------------------------------------------ the 3D view

@interface SceneView : NSOpenGLView {
    MeshRenderer _renderer;
    Camera _camera;
    BOOL _ready;
    BOOL _plain;            // gray, lit mesh instead of the colors of the patches
    BOOL _hasMesh;
    NSPoint _last;
}
@property (nonatomic, copy) void (^onDropURL)(NSURL*);
- (void)showMesh:(const TriMesh&)mesh colors:(const std::vector<float>&)colors;
- (void)setPlain:(BOOL)plain;
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
        self.accessibilityLabel = @"3D view of the mesh";
        self.accessibilityRole = NSAccessibilityGroupRole;
    }
    return self;
}

- (void)prepareOpenGL
{
    [super prepareOpenGL];
    [self.openGLContext makeCurrentContext];
    GLint swap = 1;
    [self.openGLContext setValues:&swap forParameter:NSOpenGLContextParameterSwapInterval];
    std::string log;
    _ready = _renderer.init(&log);
    if (!_ready) NSLog(@"MeshRenderer: %s", log.c_str());
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
}

- (BOOL)acceptsFirstResponder { return YES; }
- (BOOL)isOpaque { return YES; }

- (void)viewDidChangeEffectiveAppearance { [self setNeedsDisplay:YES]; }

- (void)showMesh:(const TriMesh&)mesh colors:(const std::vector<float>&)colors
{
    [self.openGLContext makeCurrentContext];
    _renderer.invalidateGeometry();
    _renderer.upload(mesh, colors);
    _hasMesh = YES;
    [self setNeedsDisplay:YES];
}

- (void)setPlain:(BOOL)plain { _plain = plain; [self setNeedsDisplay:YES]; }
- (void)resetCamera { _camera.reset(); [self setNeedsDisplay:YES]; }

- (void)drawRect:(NSRect)dirty
{
    [self.openGLContext makeCurrentContext];
    const NSRect b = [self convertRectToBacking:self.bounds];
    glViewport(0, 0, GLsizei(b.size.width), GLsizei(b.size.height));
    __block float r = 0.12f, g = 0.12f, bl = 0.13f;
    [self.effectiveAppearance performAsCurrentDrawingAppearance:^{   // the color of the window follows light / dark
        NSColor* c = [[NSColor underPageBackgroundColor] colorUsingColorSpace:NSColorSpace.sRGBColorSpace];
        if (c) { r = float(c.redComponent); g = float(c.greenComponent); bl = float(c.blueComponent); }
    }];
    glClearColor(r, g, bl, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (_ready && _hasMesh && b.size.height > 0) {
        float vp[16], v[16];
        _camera.viewProj(float(b.size.width / b.size.height), vp);
        _camera.view(v);
        _renderer.draw(vp, v, false, 0.0f, false, false, _plain);
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

// ------------------------------------------------------------------------------------------------------------------ the application

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

    AtlasSession _session;
    std::atomic<bool> _cancel;
    std::thread _worker;
    BOOL _running;
}
@end

@implementation AppController

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

    NSStackView* stack = [NSStackView stackViewWithViews:@[
        label(@"Mesh", 11, YES, YES), _meshLabel, open,
        label(@"Settings", 11, YES, YES), label(@"Quality", 12), _quality, _manual, patchRow,
        label(@"Texture size", 12), _textureSize, label(@"Mip levels", 12), _mip, _mipLabel,
        label(@"Atlas", 11, YES, YES), makeRow, _progress, _status,
        label(@"Result", 11, YES, YES), _result]];
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
    [fx addSubview:stack];
    [NSLayoutConstraint activateConstraints:@[
        [stack.topAnchor constraintEqualToAnchor:fx.topAnchor],
        [stack.leadingAnchor constraintEqualToAnchor:fx.leadingAnchor],
        [stack.trailingAnchor constraintEqualToAnchor:fx.trailingAnchor]]];
    for (NSView* v in @[(NSView*)_quality, (NSView*)_textureSize, (NSView*)_mip, (NSView*)_progress])
        [v.widthAnchor constraintEqualToAnchor:stack.widthAnchor constant:-40].active = YES;
    [self updateMipLabel];
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

- (void)mipChanged:(id)sender { [self updateMipLabel]; }
- (void)textureChanged:(id)sender { [self updateMipLabel]; }
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
    }
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)app { return YES; }
- (void)application:(NSApplication*)app openURLs:(NSArray<NSURL*>*)urls { if (urls.count) [self loadPath:urls.firstObject.path]; }

// ---- toolbar
- (NSArray<NSToolbarItemIdentifier>*)toolbarDefaultItemIdentifiers:(NSToolbar*)t
{
    return @[NSToolbarToggleSidebarItemIdentifier, NSToolbarSidebarTrackingSeparatorItemIdentifier, @"open", @"makeAtlas",
             NSToolbarFlexibleSpaceItemIdentifier, @"views", @"save"];
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
    else if ([ident isEqual:@"save"]) set(@"Save", @"square.and.arrow.down", @selector(saveAs:), @"Save the mesh with the atlas (⇧⌘S)");
    else if ([ident isEqual:@"views"]) {
        _viewSwitch = [NSSegmentedControl segmentedControlWithLabels:@[@"Patches", @"Plain"] trackingMode:NSSegmentSwitchTrackingSelectOne
                                                                target:self action:@selector(viewSwitched:)];
        _viewSwitch.selectedSegment = 0;
        _viewSwitch.accessibilityLabel = @"View";
        item.label = @"View"; item.paletteLabel = @"View"; item.view = _viewSwitch;
    }
    return item;
}

- (BOOL)validateToolbarItem:(NSToolbarItem*)item
{
    if ([item.itemIdentifier isEqual:@"open"]) return !_running;
    if ([item.itemIdentifier isEqual:@"makeAtlas"]) return !_running && _session.hasMesh();
    if ([item.itemIdentifier isEqual:@"save"]) return !_running && _session.hasResult();
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

    NSMenu* view = top(@"View");
    add(view, @"Show Patches", @selector(showPatches:), @"1", NSEventModifierFlagCommand, self);
    add(view, @"Show Plain Shading", @selector(showPlain:), @"2", NSEventModifierFlagCommand, self);
    [view addItem:[NSMenuItem separatorItem]];
    add(view, @"Reset Camera", @selector(resetCamera:), @"0", NSEventModifierFlagCommand, self);
    [view addItem:[NSMenuItem separatorItem]];
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
    if (a == @selector(makeAtlas:)) return !_running && _session.hasMesh();
    if (a == @selector(cancelAtlas:)) return _running;
    if (a == @selector(saveAs:)) return !_running && _session.hasResult();
    if (a == @selector(showSettings:)) return NO;
    if (a == @selector(showPatches:)) item.state = _viewSwitch.selectedSegment == 0 ? NSControlStateValueOn : NSControlStateValueOff;
    if (a == @selector(showPlain:)) item.state = _viewSwitch.selectedSegment == 1 ? NSControlStateValueOn : NSControlStateValueOff;
    if (a == @selector(toggleSidebar:)) {
        NSSplitViewItem* s = _split.splitViewItems.firstObject;
        item.title = s.collapsed ? @"Show Sidebar" : @"Hide Sidebar";
    }
    return YES;
}

// ---- actions
- (void)loadPath:(NSString*)path
{
    std::string err;
    if (!_session.load(path.UTF8String, &err)) {
        _status.stringValue = [NSString stringWithFormat:@"Cannot open %@: %s", path.lastPathComponent, err.c_str()];
        return;
    }
    _window.title = path.lastPathComponent;
    _window.representedURL = [NSURL fileURLWithPath:path];
    _meshLabel.stringValue = [NSString stringWithFormat:@"%@ — %zu faces", path.lastPathComponent, _session.mesh().n_faces()];
    _result.stringValue = @"";
    _status.stringValue = @"Ready";
    [_scene resetCamera];
    [_scene showMesh:_session.mesh() colors:{}];
    _viewSwitch.selectedSegment = 1;
    [_scene setPlain:YES];
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
    [_window.toolbar validateVisibleItems];
}

- (void)makeAtlas:(id)sender
{
    if (_running || !_session.hasMesh()) return;
    const SessionSettings s = [self settings];
    [self setRunning:YES];
    _cancel = false;
    _progress.doubleValue = 0.02;
    _status.stringValue = @"Starting…";
    if (_worker.joinable()) _worker.join();
    __weak AppController* weakSelf = self;
    _worker = std::thread([self, weakSelf, s]() {
        std::string err;
        const bool ok = _session.run(s, [weakSelf](double f, const std::string& what) {
            NSString* text = [NSString stringWithUTF8String:what.c_str()];
            dispatch_async(dispatch_get_main_queue(), ^{
                AppController* me = weakSelf;
                if (!me) return;
                me->_progress.doubleValue = f;
                me->_status.stringValue = text;
            });
        }, _cancel, &err);
        dispatch_async(dispatch_get_main_queue(), ^{ [weakSelf atlasFinished:ok error:[NSString stringWithUTF8String:err.c_str()]]; });
    });
}

- (void)cancelAtlas:(id)sender { if (_running) { _cancel = true; _status.stringValue = @"Cancelling…"; } }

- (void)atlasFinished:(BOOL)ok error:(NSString*)error
{
    if (_worker.joinable()) _worker.join();
    [self setRunning:NO];
    if (!ok) {
        _status.stringValue = [error isEqual:@"cancelled"] ? @"Cancelled" : [@"Failed: " stringByAppendingString:error];
        return;
    }
    _status.stringValue = @"Done";
    NSString* text = [NSString stringWithFormat:@"%zu patches\nAtlas %d × %d, %.0f%% of the texture is used\n%.1f s",
                                                _session.patchCount(), _session.resolution(), _session.resolution(),
                                                100.0 * _session.efficiency(), _session.seconds()];
    if (_session.unresolvedPatches() > 0)
        text = [text stringByAppendingFormat:@"\n%d patches could not be unfolded cleanly", _session.unresolvedPatches()];
    _result.stringValue = text;
    _viewSwitch.selectedSegment = 0;
    [_scene setPlain:NO];
    [_scene showMesh:_session.mesh() colors:_session.patchColors()];
    if (getenv("HUGE_NATIVE_AUTORUN")) {
        printf("NATIVE: %s\n", text.UTF8String);
        [self autoShotAndQuit];
    }
}

- (void)saveAs:(id)sender
{
    if (!_session.hasResult()) return;
    std::string stem = std::string(_session.path());
    const size_t slash = stem.find_last_of('/');
    if (slash != std::string::npos) stem = stem.substr(slash + 1);
    const size_t dot = stem.find_last_of('.');
    if (dot != std::string::npos) stem.resize(dot);
    const std::string p = native::saveFile("Save the mesh with the atlas", stem + "_atlas.obj", {"obj"}, "");
    if (p.empty()) return;
    std::string err;
    _status.stringValue = _session.save(p, &err) ? @"Saved" : [NSString stringWithFormat:@"Cannot save: %s", err.c_str()];
}

- (void)showPatches:(id)sender { _viewSwitch.selectedSegment = 0; [_scene setPlain:NO]; }
- (void)showPlain:(id)sender { _viewSwitch.selectedSegment = 1; [_scene setPlain:YES]; }
- (void)viewSwitched:(NSSegmentedControl*)s { [_scene setPlain:s.selectedSegment == 1]; }
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
