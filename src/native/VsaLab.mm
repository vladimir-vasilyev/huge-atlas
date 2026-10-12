#import "native/VsaLab.h"

#import "native/NumberRow.h"

#include "core/MeshPrep.h"
#include "core/Operations.h"
#include "core/Partition.h"
#include "core/Proxy.h"
#include "core/Smoothing.h"

#include <algorithm>
#include <cmath>
#include <memory>

// a document view that starts at its top in a scroll view
@interface TopAlignedView : NSView
@end
@implementation TopAlignedView
- (BOOL)isFlipped { return YES; }
@end

// ---- the plot of the energy of every Lloyd step
@interface EnergyPlot : NSView
@property (nonatomic) std::vector<double> values;
@end

@implementation EnergyPlot
- (BOOL)isFlipped { return YES; }
- (void)drawRect:(NSRect)dirty
{
    [NSColor.controlBackgroundColor setFill];
    NSRectFill(self.bounds);
    [[NSColor.separatorColor colorWithAlphaComponent:0.6] setStroke];
    NSFrameRect(self.bounds);
    if (_values.size() < 2) return;
    double lo = _values[0], hi = _values[0];
    for (double v : _values) { lo = std::min(lo, v); hi = std::max(hi, v); }
    const double span = std::max(hi - lo, 1e-12);
    NSBezierPath* path = [NSBezierPath bezierPath];
    const NSRect r = NSInsetRect(self.bounds, 6, 6);
    for (size_t i = 0; i < _values.size(); ++i) {
        const NSPoint p = NSMakePoint(r.origin.x + r.size.width * double(i) / double(_values.size() - 1),
                                      r.origin.y + r.size.height * (1.0 - (_values[i] - lo) / span));
        if (i == 0) [path moveToPoint:p]; else [path lineToPoint:p];
    }
    [NSColor.controlAccentColor setStroke];
    path.lineWidth = 1.5;
    [path stroke];
}
- (NSSize)intrinsicContentSize { return NSMakeSize(NSViewNoIntrinsicMetric, 70); }
@end

static void regionColor(int index, float* rgb)
{
    const float h = std::fmod(float(index) * 0.61803398875f, 1.0f), s = 0.6f, v = 0.92f;
    const float c = v * s, x = c * (1.0f - std::abs(std::fmod(h * 6.0f, 2.0f) - 1.0f)), m = v - c;
    float r = 0, g = 0, b = 0;
    switch (int(h * 6.0f) % 6) {
        case 0: r = c; g = x; break;
        case 1: r = x; g = c; break;
        case 2: g = c; b = x; break;
        case 3: g = x; b = c; break;
        case 4: r = x; b = c; break;
        default: r = c; b = x; break;
    }
    rgb[0] = r + m; rgb[1] = g + m; rgb[2] = b + m;
}

// weight 1 is neutral, heavier goes to red, lighter to blue (log scale)
static void weightColor(float w, float* out)
{
    const float neutral[3] = {0.55f, 0.58f, 0.65f};
    const float lw = std::max(-3.0f, std::min(4.0f, std::log2(std::max(w, 1e-6f))));
    static const float heavy[3] = {1.0f, 0.2f, 0.1f}, light[3] = {0.1f, 0.25f, 0.95f};
    const float* target = lw >= 0 ? heavy : light;
    const float t = std::min(1.0f, std::abs(lw) / (lw >= 0 ? 4.0f : 3.0f));
    for (int k = 0; k < 3; ++k) out[k] = neutral[k] * (1.0f - t) + target[k] * t;
}

@implementation VsaLab {
    NSWindow* _window;
    NSPopUpButton *_metric, *_seeding, *_paint;
    NumberRow *_k, *_seed, *_iterations, *_patience, *_lambda, *_kappa, *_sweeps, *_ratio, *_teleports, *_radius, *_weight;
    NSButton *_runButton, *_brushOn, *_erase;
    NSTextField* _status;
    EnergyPlot* _plot;
    NSTimer* _timer;

    const TriMesh* _mesh;
    FaceData _fd;
    std::vector<Moments> _base, _moments;
    std::vector<float> _weights;
    Moments _whole;
    VSAState _vsa;
    std::unique_ptr<RefineJob> _job;
    BOOL _painted;
    int _steps;
}

static NSTextField* heading(NSString* t)
{
    NSTextField* f = [NSTextField labelWithString:t];
    f.font = [NSFont boldSystemFontOfSize:11];
    f.textColor = NSColor.secondaryLabelColor;
    return f;
}

- (VSAInput)input { return VSAInput{*_mesh, _fd, _moments}; }

- (NSWindow*)window { return _window; }

- (void)attachMesh:(const TriMesh*)mesh
{
    [self stopRun];
    _mesh = mesh;
    _vsa = VSAState();
    _job.reset();
    _steps = 0;
    _painted = NO;
    if (!mesh || mesh->n_faces() == 0) {
        _mesh = nullptr;
        _fd = FaceData();
        _base.clear(); _moments.clear(); _weights.clear();
        [self updateStatus:@"No mesh"];
        return;
    }
    _fd = computeFaceData(*_mesh);
    _base = computeFaceMoments(*_mesh, _fd);
    _weights.assign(_mesh->n_faces(), 1.0f);
    [self rebuildMoments];
    [self updateStatus:@"Seed the proxies to begin"];
}

- (void)rebuildMoments
{
    _moments = _base;
    _whole = Moments();
    for (size_t i = 0; i < _moments.size(); ++i) {
        if (_weights[i] != 1.0f) _moments[i] *= double(_weights[i]);
        _whole += _moments[i];
    }
    if (_vsa.label.size() == _mesh->n_faces()) fitProxies([self input], _vsa);   // also recomputes the region moments
}

// ---- the window
- (void)showWindow
{
    if (!_window) [self build];
    [_window makeKeyAndOrderFront:nil];
    [self refreshView];
}

- (NSButton*)button:(NSString*)title action:(SEL)a tip:(NSString*)tip
{
    NSButton* b = [NSButton buttonWithTitle:title target:self action:a];
    b.controlSize = NSControlSizeSmall;
    b.toolTip = tip;
    return b;
}

- (void)build
{
    _metric = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    [_metric addItemsWithTitles:@[@"L2,1 (normals)", @"L2 (positions)"]];
    _seeding = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    [_seeding addItemsWithTitles:@[@"Random", @"Farthest point"]];
    [_seeding selectItemAtIndex:1];
    _paint = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    [_paint addItemsWithTitles:@[@"Regions", @"Weights"]];
    _paint.target = self;
    _paint.action = @selector(paintModeChanged:);
    _k = [[NumberRow alloc] initWithTitle:@"Proxies (k)" min:1 max:5000 step:10 value:40 decimals:0];
    _seed = [[NumberRow alloc] initWithTitle:@"Random seed" min:0 max:99999 step:1 value:0 decimals:0];
    _iterations = [[NumberRow alloc] initWithTitle:@"Max iterations" min:1 max:1000 step:10 value:30 decimals:0];
    _patience = [[NumberRow alloc] initWithTitle:@"Patience" min:1 max:20 step:1 value:3 decimals:0];
    _lambda = [[NumberRow alloc] initWithTitle:@"Border smoothing (lambda)" min:0 max:1 step:0.05 value:0.1 decimals:2];
    _kappa = [[NumberRow alloc] initWithTitle:@"Crease (kappa)" min:0 max:4 step:0.25 value:0 decimals:2];
    _sweeps = [[NumberRow alloc] initWithTitle:@"Max sweeps" min:1 max:30 step:1 value:10 decimals:0];
    _ratio = [[NumberRow alloc] initWithTitle:@"Teleport ratio" min:0.05 max:1 step:0.05 value:0.5 decimals:2];
    _teleports = [[NumberRow alloc] initWithTitle:@"Max teleports" min:1 max:500 step:5 value:20 decimals:0];
    _radius = [[NumberRow alloc] initWithTitle:@"Brush radius (px)" min:2 max:100 step:2 value:20 decimals:0];
    _weight = [[NumberRow alloc] initWithTitle:@"Brush weight" min:0.1 max:20 step:0.5 value:4 decimals:1];
    __weak VsaLab* weak = self;
    _radius.onChange = ^{ [weak brushChanged:nil]; };
    _runButton = [self button:@"Run" action:@selector(runStop:) tip:@"Lloyd iterations until the energy stops falling"];
    _brushOn = [NSButton checkboxWithTitle:@"Paint on the model (⌥ drag turns)" target:self action:@selector(brushChanged:)];
    _erase = [NSButton checkboxWithTitle:@"Erase (set to 1)" target:nil action:nil];
    _status = [NSTextField wrappingLabelWithString:@""];
    _status.font = [NSFont monospacedSystemFontOfSize:10 weight:NSFontWeightRegular];
    _status.textColor = NSColor.secondaryLabelColor;
    _status.preferredMaxLayoutWidth = 330;
    _plot = [[EnergyPlot alloc] initWithFrame:NSMakeRect(0, 0, 330, 70)];
    _plot.accessibilityLabel = @"Energy of the Lloyd steps";

    auto row = [](NSArray* views) { NSStackView* s = [NSStackView stackViewWithViews:views]; s.spacing = 6; return s; };
    NSStackView* stack = [NSStackView stackViewWithViews:@[
        heading(@"Seeding"),
        row(@[[NSTextField labelWithString:@"Metric:"], _metric]), _k,
        row(@[[NSTextField labelWithString:@"Seeding:"], _seeding]), _seed,
        row(@[[self button:@"Seed" action:@selector(seed:) tip:@"Choose k faces as the first proxies"]]),
        heading(@"Lloyd"),
        row(@[[self button:@"Partition" action:@selector(partition:) tip:@"Flood the faces to the nearest proxy"],
              [self button:@"Fit" action:@selector(fit:) tip:@"Fit every proxy to its region"],
              [self button:@"Step" action:@selector(step:) tip:@"One Lloyd iteration: partition and fit"], _runButton]),
        _iterations, _patience,
        heading(@"Border smoothing"), _lambda, _kappa, _sweeps,
        row(@[[self button:@"Smooth now" action:@selector(smoothNow:) tip:@"Smooth the borders of the regions once"]]),
        heading(@"Proxies"),
        row(@[[self button:@"Insert" action:@selector(insert:) tip:@"Add a proxy where the error is largest"],
              [self button:@"Delete cheapest" action:@selector(removeCheapest:) tip:@"Merge the two regions whose union costs least"],
              [self button:@"Teleport" action:@selector(teleport:) tip:@"Move a proxy from a flat region to a bad one"]]),
        row(@[[self button:@"Grow to k" action:@selector(growToK:) tip:@"Insert proxies, with Lloyd between, until there are k"],
              [self button:@"Run + teleport" action:@selector(runTeleport:) tip:@"Lloyd with teleportation of proxies"]]),
        _ratio, _teleports,
        heading(@"Weights"), _brushOn, _radius, _weight, _erase,
        row(@[[self button:@"Reset weights" action:@selector(resetWeights:) tip:@"Every face has the weight 1 again"],
              [NSTextField labelWithString:@"Show:"], _paint]),
        heading(@"State"), _status, _plot,
        row(@[[self button:@"Use as the Start of the Atlas" action:@selector(useLabels:) tip:@"Make Atlas continues from these regions instead of seeding again"]]),
    ]];
    stack.orientation = NSUserInterfaceLayoutOrientationVertical;
    stack.alignment = NSLayoutAttributeLeading;
    stack.spacing = 6;
    stack.edgeInsets = NSEdgeInsetsMake(14, 14, 14, 14);
    stack.translatesAutoresizingMaskIntoConstraints = NO;
    for (NSView* v in @[(NSView*)_k, (NSView*)_seed, (NSView*)_iterations, (NSView*)_patience, (NSView*)_lambda, (NSView*)_kappa, (NSView*)_sweeps, (NSView*)_ratio,
                        (NSView*)_teleports, (NSView*)_radius, (NSView*)_weight, (NSView*)_plot])
        [v.widthAnchor constraintEqualToAnchor:stack.widthAnchor constant:-28].active = YES;
    NSScrollView* scroll = [[NSScrollView alloc] init];
    scroll.hasVerticalScroller = YES;
    scroll.drawsBackground = NO;
    NSView* doc = [[TopAlignedView alloc] init];
    doc.translatesAutoresizingMaskIntoConstraints = NO;
    [doc addSubview:stack];
    scroll.documentView = doc;
    [NSLayoutConstraint activateConstraints:@[
        [stack.topAnchor constraintEqualToAnchor:doc.topAnchor], [stack.leadingAnchor constraintEqualToAnchor:doc.leadingAnchor],
        [stack.trailingAnchor constraintEqualToAnchor:doc.trailingAnchor], [stack.bottomAnchor constraintEqualToAnchor:doc.bottomAnchor],
        [doc.widthAnchor constraintEqualToConstant:360]]];
    _window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 376, 760)
                                          styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskResizable
                                            backing:NSBackingStoreBuffered defer:NO];
    _window.title = @"Segmentation Lab";
    _window.releasedWhenClosed = NO;
    _window.contentView = scroll;
    [_window setFrameAutosaveName:@"VsaLab"];
    [_window center];
    [self updateStatus:_mesh ? @"Seed the proxies to begin" : @"Open a mesh first"];
}

// ---- state shown
- (BOOL)partitioned { return _mesh && _vsa.label.size() == _mesh->n_faces(); }

- (void)updateStatus:(NSString*)note
{
    NSMutableString* s = [NSMutableString string];
    if (note.length) [s appendFormat:@"%@\n", note];
    if (_mesh) {
        [s appendFormat:@"Proxies %d, partitions run %d", _vsa.k(), _steps];
        if ([self partitioned]) [s appendFormat:@"\nEnergy %.6f", totalEnergy(_vsa)];
    }
    _status.stringValue = s;
    _plot.values = _vsa.energyHistory;
    [_plot setNeedsDisplay:YES];
}

- (void)refreshView
{
    if (!_mesh || !self.onColors) return;
    const size_t n = _mesh->n_faces();
    std::vector<float> rgb(n * 3, 0.72f);
    if (_paint.indexOfSelectedItem == 1 || ![self partitioned]) {
        if (_paint.indexOfSelectedItem == 1) for (size_t f = 0; f < n; ++f) weightColor(_weights[f], &rgb[3 * f]);
    } else {
        for (size_t f = 0; f < n; ++f)
            if (_vsa.label[f] >= 0) regionColor(_vsa.label[f], &rgb[3 * f]);
    }
    self.onColors(rgb);
}

- (void)paintModeChanged:(id)sender { [self refreshView]; }

- (VSAInput)seedInput { return [self input]; }

- (BOOL)needMesh
{
    if (_mesh) return YES;
    [self updateStatus:@"Open a mesh first"];
    return NO;
}

- (void)applySettings
{
    _vsa.metric = _metric.indexOfSelectedItem == 0 ? Metric::L21 : Metric::L2;
    _vsa.smooth = SmoothParams{_lambda.value, _kappa.value, int(_sweeps.value)};
}

- (LloydParams)lloydParams
{
    LloydParams lp;
    lp.maxIterations = int(_iterations.value);
    lp.patience = int(_patience.value);
    return lp;
}

- (TeleportParams)teleportParams
{
    TeleportParams tp;
    tp.ratio = _ratio.value;
    tp.maxTeleports = int(_teleports.value);
    return tp;
}

// ---- actions
- (void)seed:(id)sender
{
    if (![self needMesh]) return;
    [self stopRun];
    _vsa = VSAState();
    [self applySettings];
    const int k = std::max(1, std::min(int(_k.value), int(_mesh->n_faces())));
    if (_seeding.indexOfSelectedItem == 0) initRandomSeeds([self input], _vsa, k, uint64_t(_seed.value));
    else initFarthestSeeds([self input], _vsa, k, uint64_t(_seed.value));
    _steps = 0;
    [self updateStatus:@"Seeded"];
    [self refreshView];
}

- (void)partition:(id)sender
{
    if (![self needMesh] || _vsa.k() == 0) return;
    [self applySettings];
    const int added = partition([self input], _vsa);
    ++_steps;
    [self updateStatus:added > 0 ? [NSString stringWithFormat:@"Partitioned; %d proxies added for unseeded components", added] : @"Partitioned"];
    [self refreshView];
}

- (void)fit:(id)sender
{
    if (![self needMesh] || ![self partitioned]) return;
    fitProxies([self input], _vsa);
    [self updateStatus:@"Fitted"];
    [self refreshView];
}

- (void)step:(id)sender
{
    if (![self needMesh] || _vsa.k() == 0) return;
    [self applySettings];
    lloydStep([self input], _vsa);
    ++_steps;
    [self updateStatus:@"One Lloyd step"];
    [self refreshView];
}

- (void)startJob:(RefineJob::Kind)kind
{
    if (![self needMesh]) return;
    [self applySettings];
    _job = std::make_unique<RefineJob>(kind, [self lloydParams], [self teleportParams], int(_k.value));
    _runButton.title = @"Stop";
    _timer = [NSTimer scheduledTimerWithTimeInterval:0.03 target:self selector:@selector(tick:) userInfo:nil repeats:YES];
}

- (void)stopRun
{
    [_timer invalidate];
    _timer = nil;
    _job.reset();
    _runButton.title = @"Run";
}

- (void)runStop:(id)sender
{
    if (_job) { [self stopRun]; [self updateStatus:@"Stopped"]; return; }
    if (_vsa.k() == 0) return;
    [self startJob:RefineJob::Kind::Lloyd];
}

- (void)growToK:(id)sender
{
    if (![self needMesh] || _job) return;
    if (_vsa.k() == 0) { [self applySettings]; initFarthestSeeds([self input], _vsa, 1, uint64_t(_seed.value)); }
    [self startJob:RefineJob::Kind::Grow];
}

- (void)runTeleport:(id)sender
{
    if (![self needMesh] || _job || _vsa.k() < 3) return;
    [self startJob:RefineJob::Kind::Teleport];
}

// one step of the running job per tick
- (void)tick:(NSTimer*)t
{
    if (!_job || !_mesh) { [self stopRun]; return; }
    if (_vsa.k() == 0) { [self stopRun]; return; }
    const bool more = _job->step([self input], _vsa);
    ++_steps;
    if (more) {
        [self updateStatus:@"Running…"];
        [self refreshView];
        return;
    }
    NSString* done = [NSString stringWithFormat:@"Finished: %d iterations, %d teleports, k = %d, %s", _job->iterations(), _job->teleports(), _vsa.k(),
                      _job->lastRunConverged() ? "converged" : "stopped at max iterations"];
    [self stopRun];
    [self updateStatus:done];
    [self refreshView];
}

- (void)smoothNow:(id)sender
{
    if (![self needMesh] || ![self partitioned] || _job) return;
    [self applySettings];
    const SmoothStats st = smoothLabels([self input], _vsa);
    [self updateStatus:[NSString stringWithFormat:@"Smoothing: %d flips in %d sweeps, perimeter %.3f → %.3f", st.flips, st.sweeps, st.perimeterBefore, st.perimeterAfter]];
    [self refreshView];
}

- (void)insert:(id)sender
{
    if (![self needMesh] || ![self partitioned] || _job) return;
    [self updateStatus:insertProxy([self input], _vsa) ? @"A proxy was inserted" : @"Nothing to split"];
    [self refreshView];
}

- (void)removeCheapest:(id)sender
{
    if (![self needMesh] || ![self partitioned] || _job) return;
    RegionPair m;
    const BOOL ok = findBestMerge([self input], _vsa, m) && deleteProxy([self input], _vsa, m.a, m.b);
    [self updateStatus:ok ? @"Two regions were merged" : @"Nothing to merge"];
    [self refreshView];
}

- (void)teleport:(id)sender
{
    if (![self needMesh] || ![self partitioned] || _job) return;
    [self updateStatus:teleportProxy([self input], _vsa, _ratio.value) ? @"A proxy was teleported" : @"Teleport: no admissible move"];
    [self refreshView];
}

// ---- the brush
- (void)brushChanged:(id)sender
{
    if (self.onBrush) self.onBrush(_brushOn.state == NSControlStateValueOn && _mesh != nullptr, int(_radius.value));
}

- (void)brushFaces:(const std::vector<int>&)faces
{
    if (!_mesh) return;
    const float w = _erase.state == NSControlStateValueOn ? 1.0f : float(_weight.value);
    for (int f : faces) if (f >= 0 && size_t(f) < _weights.size()) _weights[size_t(f)] = w;
    _painted = YES;
    if (_paint.indexOfSelectedItem == 1) [self refreshView];
}

- (void)brushEnded
{
    if (!_painted) return;   // the end of a stroke: the weights go into the moments and the proxies are fitted once
    _painted = NO;
    [self rebuildMoments];
    [self refreshView];
}

- (void)resetWeights:(id)sender
{
    if (![self needMesh]) return;
    std::fill(_weights.begin(), _weights.end(), 1.0f);
    [self rebuildMoments];
    [self updateStatus:@"Weights reset"];
    [self refreshView];
}

- (void)useLabels:(id)sender
{
    if (![self needMesh] || ![self partitioned]) { [self updateStatus:@"Partition first"]; return; }
    if (self.onUseLabels) self.onUseLabels(_vsa.label);
    [self updateStatus:[NSString stringWithFormat:@"%d regions will start the atlas: press Make Atlas", _vsa.k()]];
}


// ---- the checks without a mouse
- (NSString*)selfTestWithK:(int)k
{
    if (!_mesh) return @"no mesh";
    if (!_window) [self build];
    _k.value = k;
    [self seed:nil];
    for (int i = 0; i < 3; ++i) [self step:nil];
    const double e3 = totalEnergy(_vsa);
    [self startJob:RefineJob::Kind::Lloyd];
    for (int i = 0; i < 200 && _job; ++i) [self tick:nil];
    [self stopRun];
    const double eRun = totalEnergy(_vsa);
    [self insert:nil];
    const int afterInsert = _vsa.k();
    [self removeCheapest:nil];
    const int afterDelete = _vsa.k();
    [self teleport:nil];
    // a stroke with the brush: the first faces get the weight 8
    std::vector<int> faces;
    for (int f = 0; f < 200 && size_t(f) < _weights.size(); ++f) faces.push_back(f);
    _weight.value = 8;
    [self brushFaces:faces];
    [self brushEnded];
    const BOOL weighted = _weights[0] == 8.0f && _weights[_weights.size() - 1] == 1.0f;
    [self smoothNow:nil];
    [self useLabels:nil];
    return [NSString stringWithFormat:@"k=%d energy after 3 steps %.5f, after the run %.5f (history %zu), insert -> %d, delete -> %d, brush weights %s, labels %s", k, e3, eRun,
            _vsa.energyHistory.size(), afterInsert, afterDelete, weighted ? "ok" : "WRONG", [self partitioned] ? "ready" : "missing"];
}

@end
