#import "native/DeveloperPanel.h"

#include <algorithm>

// "label  [number] [stepper]"
@interface NumberRow : NSStackView {
    NSTextField* _field;
    NSStepper* _stepper;
    int _decimals;
}
- (instancetype)initWithTitle:(NSString*)title min:(double)lo max:(double)hi step:(double)step value:(double)value decimals:(int)decimals;
@property (nonatomic) double value;
@end

@implementation NumberRow

- (instancetype)initWithTitle:(NSString*)title min:(double)lo max:(double)hi step:(double)step value:(double)value decimals:(int)decimals
{
    NSTextField* name = [NSTextField labelWithString:title];
    name.font = [NSFont systemFontOfSize:12];
    name.lineBreakMode = NSLineBreakByTruncatingTail;
    [name setContentCompressionResistancePriority:NSLayoutPriorityDefaultLow forOrientation:NSLayoutConstraintOrientationHorizontal];
    NSTextField* field = [NSTextField textFieldWithString:@""];
    field.alignment = NSTextAlignmentRight;
    field.font = [NSFont monospacedDigitSystemFontOfSize:12 weight:NSFontWeightRegular];
    field.accessibilityLabel = title;
    NSStepper* stepper = [[NSStepper alloc] init];
    stepper.minValue = lo; stepper.maxValue = hi; stepper.increment = step;
    self = [super initWithFrame:NSZeroRect];
    if (!self) return nil;
    _field = field; _stepper = stepper; _decimals = decimals;
    field.target = self; field.action = @selector(fieldChanged:);
    stepper.target = self; stepper.action = @selector(stepperChanged:);
    [field.widthAnchor constraintEqualToConstant:58].active = YES;
    [name setContentHuggingPriority:NSLayoutPriorityDefaultLow forOrientation:NSLayoutConstraintOrientationHorizontal];
    [self setViews:@[name, field, stepper] inGravity:NSStackViewGravityLeading];
    self.spacing = 4;
    self.value = value;
    return self;
}

- (double)value { return _stepper.doubleValue; }

- (void)setValue:(double)v
{
    v = std::max(_stepper.minValue, std::min(_stepper.maxValue, v));
    _stepper.doubleValue = v;
    _field.stringValue = [NSString stringWithFormat:@"%.*f", _decimals, v];
}

- (void)fieldChanged:(id)sender { self.value = _field.doubleValue; }
- (void)stepperChanged:(id)sender { self.value = _stepper.doubleValue; }

- (void)setEnabled:(BOOL)on { _field.enabled = on; _stepper.enabled = on; }

@end

@implementation DeveloperPanel {
    NSStackView* _view;
    NSButton* _compare;
    NSTextField* _report;
    NumberRow *_smoothing, *_normalIter, *_normalSigma, *_lloyd, *_depth, *_atOnce, *_percent;
    NSButton *_lscm, *_repair;
}

@synthesize view = _view, compareButton = _compare;

static NSTextField* heading(NSString* text)
{
    NSTextField* t = [NSTextField labelWithString:text];
    t.font = [NSFont boldSystemFontOfSize:11];
    t.textColor = NSColor.secondaryLabelColor;
    return t;
}

- (instancetype)initWithTarget:(id)target compareAction:(SEL)action
{
    self = [super init];
    if (!self) return nil;
    const DeveloperOptions d;
    _smoothing = [[NumberRow alloc] initWithTitle:@"Border smoothing" min:0 max:2 step:0.05 value:d.smoothing decimals:2];
    _normalIter = [[NumberRow alloc] initWithTitle:@"Normal passes" min:0 max:30 step:1 value:d.normalIterations decimals:0];
    _normalSigma = [[NumberRow alloc] initWithTitle:@"Normal angle" min:5 max:90 step:5 value:d.normalSigma decimals:0];
    _lloyd = [[NumberRow alloc] initWithTitle:@"Lloyd iterations" min:1 max:300 step:5 value:d.lloydIterations decimals:0];
    _depth = [[NumberRow alloc] initWithTitle:@"Splitting depth" min:1 max:20 step:1 value:d.maxDepth decimals:0];
    _atOnce = [[NumberRow alloc] initWithTitle:@"Split at once above" min:0 max:1000000 step:5000 value:d.splitAtOnce decimals:0];
    _percent = [[NumberRow alloc] initWithTitle:@"Huge copy, %" min:0 max:100 step:5 value:d.simplifyPercent decimals:0];
    _lscm = [NSButton checkboxWithTitle:@"LSCM before Tutte" target:nil action:nil];
    _lscm.state = d.allowLscm ? NSControlStateValueOn : NSControlStateValueOff;
    _repair = [NSButton checkboxWithTitle:@"Repair near misses" target:nil action:nil];
    _repair.state = d.repairNearMisses ? NSControlStateValueOn : NSControlStateValueOff;
    _compare = [NSButton buttonWithTitle:@"Compare with Original Layout…" target:target action:action];
    _compare.toolTip = @"The measures of the layout in the file and of the new atlas side by side";
    _report = [NSTextField wrappingLabelWithString:@""];
    _report.font = [NSFont monospacedSystemFontOfSize:10 weight:NSFontWeightRegular];
    _report.textColor = NSColor.secondaryLabelColor;
    _report.selectable = YES;
    _report.preferredMaxLayoutWidth = 250;

    _view = [NSStackView stackViewWithViews:@[
        heading(@"Developer"), _smoothing, _normalIter, _normalSigma, _lloyd, _depth, _atOnce, _percent, _lscm, _repair, _compare, _report]];
    _view.orientation = NSUserInterfaceLayoutOrientationVertical;
    _view.alignment = NSLayoutAttributeLeading;
    _view.spacing = 6;
    for (NSView* v in @[(NSView*)_smoothing, (NSView*)_normalIter, (NSView*)_normalSigma, (NSView*)_lloyd, (NSView*)_depth, (NSView*)_atOnce, (NSView*)_percent])
        [v.widthAnchor constraintEqualToAnchor:_view.widthAnchor].active = YES;
    _view.hidden = YES;
    return self;
}

- (DeveloperOptions)options
{
    DeveloperOptions o;
    o.smoothing = _smoothing.value;
    o.normalIterations = int(_normalIter.value);
    o.normalSigma = _normalSigma.value;
    o.lloydIterations = int(_lloyd.value);
    o.maxDepth = int(_depth.value);
    o.splitAtOnce = int(_atOnce.value);
    o.simplifyPercent = _percent.value;
    o.allowLscm = _lscm.state == NSControlStateValueOn;
    o.repairNearMisses = _repair.state == NSControlStateValueOn;
    return o;
}

- (void)setReport:(NSString*)text { _report.stringValue = text; }

- (void)setEnabled:(BOOL)on
{
    for (NumberRow* r in @[_smoothing, _normalIter, _normalSigma, _lloyd, _depth, _atOnce, _percent]) [r setEnabled:on];
    _lscm.enabled = on;
    _repair.enabled = on;
}

@end

// ---- a window with monospaced text
static NSMutableArray<NSWindow*>* textWindows;

void showTextWindow(NSString* title, NSString* text)
{
    if (!textWindows) textWindows = [NSMutableArray array];
    NSWindow* w = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 560, 440)
                                              styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskResizable
                                                backing:NSBackingStoreBuffered defer:NO];
    w.title = title;
    w.releasedWhenClosed = NO;
    NSScrollView* scroll = [[NSScrollView alloc] initWithFrame:w.contentView.bounds];
    scroll.hasVerticalScroller = YES;
    scroll.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
    NSTextView* tv = [[NSTextView alloc] initWithFrame:scroll.bounds];
    tv.editable = NO;
    tv.font = [NSFont monospacedSystemFontOfSize:12 weight:NSFontWeightRegular];
    tv.textContainerInset = NSMakeSize(10, 10);
    tv.string = text;
    tv.autoresizingMask = NSViewWidthSizable;
    scroll.documentView = tv;
    w.contentView = scroll;
    [w center];
    [textWindows addObject:w];
    [w makeKeyAndOrderFront:nil];
}
