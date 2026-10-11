#import "native/Settings.h"

#include <algorithm>

namespace prefs {

NSNotificationName const Changed = @"HugeAtlasPrefsChanged";

static NSString* const kQuality = @"quality";
static NSString* const kTexture = @"textureSize";
static NSString* const kMip = @"mipLevels";
static NSString* const kHuge = @"hugeMillions";
static NSString* const kOpenUV = @"openUV";
static NSString* const kDeveloper = @"developerMode";

static void registerDefaults()
{
    static dispatch_once_t once;
    dispatch_once(&once, ^{
        Values d;
        [NSUserDefaults.standardUserDefaults registerDefaults:@{
            kQuality: @(d.quality), kTexture: @(d.textureSize), kMip: @(d.mipLevels), kHuge: @(d.hugeMillions),
            kOpenUV: @(d.openUV), kDeveloper: @(d.developer)}];
    });
}

Values load()
{
    registerDefaults();
    NSUserDefaults* u = NSUserDefaults.standardUserDefaults;
    Values v;
    v.quality = int(std::max<NSInteger>(0, std::min<NSInteger>(2, [u integerForKey:kQuality])));
    v.textureSize = int([u integerForKey:kTexture]);
    v.mipLevels = int(std::max<NSInteger>(0, std::min<NSInteger>(8, [u integerForKey:kMip])));
    v.hugeMillions = int(std::max<NSInteger>(1, std::min<NSInteger>(50, [u integerForKey:kHuge])));
    v.openUV = [u boolForKey:kOpenUV];
    v.developer = [u boolForKey:kDeveloper];
    if (v.textureSize < 256 || v.textureSize > 8192) v.textureSize = 2048;
    return v;
}

void save(const Values& v)
{
    NSUserDefaults* u = NSUserDefaults.standardUserDefaults;
    [u setInteger:v.quality forKey:kQuality];
    [u setInteger:v.textureSize forKey:kTexture];
    [u setInteger:v.mipLevels forKey:kMip];
    [u setInteger:v.hugeMillions forKey:kHuge];
    [u setBool:v.openUV forKey:kOpenUV];
    [u setBool:v.developer forKey:kDeveloper];
    [NSNotificationCenter.defaultCenter postNotificationName:Changed object:nil];
}

void reset()
{
    NSUserDefaults* u = NSUserDefaults.standardUserDefaults;
    for (NSString* k in @[kQuality, kTexture, kMip, kHuge, kOpenUV, kDeveloper]) [u removeObjectForKey:k];
    [NSNotificationCenter.defaultCenter postNotificationName:Changed object:nil];
}

} // namespace prefs

// ---- the window

@interface SettingsController : NSObject <NSToolbarDelegate> {
    NSWindow* _window;
    NSTabView* _tabs;
    NSPopUpButton* _quality;
    NSPopUpButton* _texture;
    NSPopUpButton* _mip;
    NSTextField* _hugeField;
    NSStepper* _hugeStepper;
    NSButton* _openUV;
    NSButton* _developer;
}
@property (nonatomic, readonly) NSWindow* window;
- (void)show:(NSInteger)tab;
@end

@implementation SettingsController

static NSTextField* note(NSString* text)
{
    NSTextField* t = [NSTextField wrappingLabelWithString:text];
    t.font = [NSFont systemFontOfSize:11];
    t.textColor = NSColor.secondaryLabelColor;
    t.preferredMaxLayoutWidth = 330;
    return t;
}

static NSTextField* rowLabel(NSString* text)
{
    NSTextField* t = [NSTextField labelWithString:text];
    t.alignment = NSTextAlignmentRight;
    return t;
}

// a pane: a grid of "label: control" rows in the middle of a fixed size view
static NSView* pane(NSArray<NSArray*>* rows, CGFloat height)
{
    NSGridView* grid = [NSGridView gridViewWithViews:rows];
    grid.translatesAutoresizingMaskIntoConstraints = NO;
    grid.rowSpacing = 10;
    grid.columnSpacing = 10;
    [grid columnAtIndex:0].xPlacement = NSGridCellPlacementTrailing;
    [grid columnAtIndex:1].xPlacement = NSGridCellPlacementLeading;
    NSView* v = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, 520, height)];
    [v addSubview:grid];
    [NSLayoutConstraint activateConstraints:@[
        [grid.centerXAnchor constraintEqualToAnchor:v.centerXAnchor],
        [grid.topAnchor constraintEqualToAnchor:v.topAnchor constant:20],
        [v.widthAnchor constraintEqualToConstant:520],
        [v.heightAnchor constraintEqualToConstant:height]]];
    return v;
}

- (instancetype)init
{
    self = [super init];
    if (!self) return nil;
    _quality = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    [_quality addItemsWithTitles:@[@"Fast", @"Balanced", @"Detailed"]];
    _texture = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    for (int r : {256, 512, 1024, 2048, 4096, 8192}) {
        [_texture addItemWithTitle:[NSString stringWithFormat:@"%d × %d", r, r]];
        _texture.lastItem.tag = r;
    }
    _mip = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    for (int i = 0; i <= 8; ++i) [_mip addItemWithTitle:[NSString stringWithFormat:@"%d", i]];
    for (NSPopUpButton* b in @[_quality, _texture, _mip]) { b.target = self; b.action = @selector(changed:); }
    _quality.accessibilityLabel = @"Default quality";
    _texture.accessibilityLabel = @"Default texture size";
    _mip.accessibilityLabel = @"Default mip levels";

    _hugeField = [NSTextField textFieldWithString:@"3"];
    _hugeField.alignment = NSTextAlignmentRight;
    _hugeField.accessibilityLabel = @"Million faces for the huge mode";
    _hugeField.target = self; _hugeField.action = @selector(hugeFieldChanged:);
    _hugeStepper = [[NSStepper alloc] init];
    _hugeStepper.minValue = 1; _hugeStepper.maxValue = 50; _hugeStepper.increment = 1;
    _hugeStepper.target = self; _hugeStepper.action = @selector(hugeStepperChanged:);
    NSStackView* hugeRow = [NSStackView stackViewWithViews:@[_hugeField, _hugeStepper, [NSTextField labelWithString:@"million faces"]]];
    hugeRow.spacing = 6;
    [_hugeField.widthAnchor constraintEqualToConstant:50].active = YES;

    _openUV = [NSButton checkboxWithTitle:@"Show the UV Layout when the atlas is made" target:self action:@selector(changed:)];
    _developer = [NSButton checkboxWithTitle:@"Developer mode" target:self action:@selector(changed:)];
    NSButton* resetButton = [NSButton buttonWithTitle:@"Reset All Settings" target:self action:@selector(resetAll:)];

    NSView* general = pane(@[
        @[rowLabel(@"Quality:"), _quality],
        @[rowLabel(@"Texture size:"), _texture],
        @[rowLabel(@"Mip levels:"), _mip],
        @[[NSGridCell emptyContentView], note(@"These are the values the window starts with. Changing them here also changes the sidebar.")],
        @[rowLabel(@"Big meshes:"), hugeRow],
        @[[NSGridCell emptyContentView], note(@"A mesh with more faces than this is read straight from the file and unfolded on a simplified copy (the huge mode).")],
        @[rowLabel(@"After the atlas:"), _openUV]], 300);
    NSView* advanced = pane(@[
        @[rowLabel(@"Developer:"), _developer],
        @[[NSGridCell emptyContentView], note(@"Adds the parameters of the pipeline, a report of every run and the comparison with the original layout to the sidebar, and wireframe to the View menu.")],
        @[[NSGridCell emptyContentView], resetButton]], 230);

    _tabs = [[NSTabView alloc] initWithFrame:NSMakeRect(0, 0, 520, 300)];
    _tabs.tabViewType = NSNoTabsNoBorder;
    NSTabViewItem* a = [NSTabViewItem tabViewItemWithViewController:nil];
    a.label = @"General"; a.identifier = @"General"; a.view = general;
    NSTabViewItem* b = [NSTabViewItem tabViewItemWithViewController:nil];
    b.label = @"Advanced"; b.identifier = @"Advanced"; b.view = advanced;
    [_tabs addTabViewItem:a];
    [_tabs addTabViewItem:b];

    _window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 520, 300)
                                          styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable
                                            backing:NSBackingStoreBuffered defer:NO];
    _window.contentView = _tabs;
    _window.releasedWhenClosed = NO;
    NSToolbar* tb = [[NSToolbar alloc] initWithIdentifier:@"SettingsToolbar"];
    tb.delegate = self;
    tb.allowsUserCustomization = NO;
    _window.toolbar = tb;
    _window.toolbarStyle = NSWindowToolbarStylePreference;
    tb.selectedItemIdentifier = @"General";
    [_window setFrameAutosaveName:@"SettingsWindow"];
    [self pull];
    [self select:@"General"];
    [_window makeFirstResponder:nil];
    return self;
}

- (NSWindow*)window { return _window; }

// ---- the toolbar of the panes
- (NSArray<NSToolbarItemIdentifier>*)toolbarAllowedItemIdentifiers:(NSToolbar*)t { return @[@"General", @"Advanced"]; }
- (NSArray<NSToolbarItemIdentifier>*)toolbarDefaultItemIdentifiers:(NSToolbar*)t { return @[@"General", @"Advanced"]; }
- (NSArray<NSToolbarItemIdentifier>*)toolbarSelectableItemIdentifiers:(NSToolbar*)t { return @[@"General", @"Advanced"]; }
- (NSToolbarItem*)toolbar:(NSToolbar*)t itemForItemIdentifier:(NSToolbarItemIdentifier)ident willBeInsertedIntoToolbar:(BOOL)flag
{
    NSToolbarItem* it = [[NSToolbarItem alloc] initWithItemIdentifier:ident];
    it.label = ident;
    it.target = self;
    it.action = @selector(paneChosen:);
    it.image = [NSImage imageWithSystemSymbolName:([ident isEqual:@"General"] ? @"gearshape" : @"gearshape.2")
                         accessibilityDescription:ident];
    return it;
}

- (void)paneChosen:(NSToolbarItem*)item { [self select:item.itemIdentifier]; }

- (void)select:(NSString*)ident
{
    [_tabs selectTabViewItemWithIdentifier:ident];
    _window.toolbar.selectedItemIdentifier = ident;
    _window.title = ident;
    NSView* v = _tabs.selectedTabViewItem.view;
    const NSRect old = _window.frame;
    const NSRect content = [_window contentRectForFrameRect:old];
    const NSSize want = NSMakeSize(520, v.frame.size.height);
    const NSRect nf = [_window frameRectForContentRect:NSMakeRect(content.origin.x, content.origin.y + content.size.height - want.height, want.width, want.height)];
    [_window setFrame:nf display:YES animate:_window.isVisible];
}

// ---- values <-> controls
- (void)pull
{
    const prefs::Values v = prefs::load();
    [_quality selectItemAtIndex:v.quality];
    [_texture selectItemWithTag:v.textureSize];
    [_mip selectItemAtIndex:v.mipLevels];
    _hugeStepper.integerValue = v.hugeMillions;
    _hugeField.integerValue = v.hugeMillions;
    _openUV.state = v.openUV ? NSControlStateValueOn : NSControlStateValueOff;
    _developer.state = v.developer ? NSControlStateValueOn : NSControlStateValueOff;
}

- (void)changed:(id)sender
{
    prefs::Values v;
    v.quality = int(_quality.indexOfSelectedItem);
    v.textureSize = int(_texture.selectedItem.tag);
    v.mipLevels = int(_mip.indexOfSelectedItem);
    v.hugeMillions = int(_hugeStepper.integerValue);
    v.openUV = _openUV.state == NSControlStateValueOn;
    v.developer = _developer.state == NSControlStateValueOn;
    prefs::save(v);
}

- (void)hugeFieldChanged:(id)sender
{
    _hugeStepper.integerValue = std::max(1, std::min(50, _hugeField.intValue));
    _hugeField.integerValue = _hugeStepper.integerValue;
    [self changed:nil];
}

- (void)hugeStepperChanged:(id)sender
{
    _hugeField.integerValue = _hugeStepper.integerValue;
    [self changed:nil];
}

- (void)resetAll:(id)sender
{
    prefs::reset();
    [self pull];
}

- (void)show:(NSInteger)tab
{
    [self pull];
    [self select:tab == 1 ? @"Advanced" : @"General"];
    [_window makeKeyAndOrderFront:nil];
    [_window makeFirstResponder:nil];
}

@end

static SettingsController* theSettings;

void showSettingsWindow(NSInteger tab)
{
    if (!theSettings) theSettings = [[SettingsController alloc] init];
    [theSettings show:tab];
}

NSWindow* settingsWindow() { return theSettings.window; }
