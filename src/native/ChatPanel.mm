#import "native/ChatPanel.h"

#include "chat/Provider.h"

@implementation ChatPanel {
    NSTextView* _text;
    NSTextField* _input;
    NSButton* _send;
    NSButton* _setup;
    BOOL _busy;
}

- (instancetype)initWithFrame:(NSRect)frame
{
    self = [super initWithFrame:frame];
    if (!self) return nil;

    NSScrollView* scroll = [[NSScrollView alloc] init];
    scroll.hasVerticalScroller = YES;
    scroll.drawsBackground = NO;
    scroll.borderType = NSNoBorder;
    scroll.translatesAutoresizingMaskIntoConstraints = NO;
    _text = [[NSTextView alloc] initWithFrame:NSMakeRect(0, 0, 300, 400)];
    _text.editable = NO;
    _text.selectable = YES;
    _text.drawsBackground = NO;
    _text.textContainerInset = NSMakeSize(10, 10);
    _text.autoresizingMask = NSViewWidthSizable;
    _text.verticallyResizable = YES;
    _text.horizontallyResizable = NO;
    _text.textContainer.widthTracksTextView = YES;
    _text.accessibilityLabel = @"Conversation with the assistant";
    scroll.documentView = _text;

    _input = [NSTextField textFieldWithString:@""];
    _input.placeholderString = @"Ask the assistant";
    _input.target = self;
    _input.action = @selector(sendOrStop:);
    _input.accessibilityLabel = @"Message to the assistant";
    _input.translatesAutoresizingMaskIntoConstraints = NO;

    _send = [NSButton buttonWithTitle:@"Send" target:self action:@selector(sendOrStop:)];
    _send.bezelStyle = NSBezelStyleRounded;
    _send.keyEquivalent = @"\r";
    _send.translatesAutoresizingMaskIntoConstraints = NO;

    _setup = [NSButton buttonWithTitle:@"Set Up…" target:self action:@selector(setupClicked:)];
    _setup.bezelStyle = NSBezelStyleRounded;
    _setup.controlSize = NSControlSizeSmall;
    _setup.translatesAutoresizingMaskIntoConstraints = NO;

    NSTextField* title = [NSTextField labelWithString:@"Assistant"];
    title.font = [NSFont systemFontOfSize:13 weight:NSFontWeightSemibold];
    title.translatesAutoresizingMaskIntoConstraints = NO;

    NSBox* line = [[NSBox alloc] init];
    line.boxType = NSBoxSeparator;
    line.translatesAutoresizingMaskIntoConstraints = NO;

    for (NSView* v in @[title, _setup, line, scroll, _input, _send]) [self addSubview:v];
    NSDictionary* views = @{@"title": title, @"setup": _setup, @"line": line, @"scroll": scroll, @"input": _input, @"send": _send};
    NSMutableArray* constraints = [NSMutableArray array];
    for (NSString* f in @[@"H:|-12-[title]-(>=8)-[setup]-12-|", @"H:|[line]|", @"H:|[scroll]|", @"H:|-12-[input]-8-[send]-12-|"]) {
        const NSLayoutFormatOptions o = ([f containsString:@"[title]"] || [f containsString:@"[input]"]) ? NSLayoutFormatAlignAllCenterY : 0;
        [constraints addObjectsFromArray:[NSLayoutConstraint constraintsWithVisualFormat:f options:o metrics:nil views:views]];
    }
    [constraints addObjectsFromArray:[NSLayoutConstraint constraintsWithVisualFormat:@"V:[title]-8-[line(1)][scroll]-8-[input]-12-|" options:0 metrics:nil views:views]];
    [constraints addObject:[title.topAnchor constraintEqualToAnchor:self.safeAreaLayoutGuide.topAnchor constant:12]];   // below the toolbar
    [NSLayoutConstraint activateConstraints:constraints];
    [self appendRole:@"note" text:@"I can open a mesh, make the atlas, read the numbers and tune the settings. Only the numbers go to the model provider, never the mesh. Press Set Up… to choose the provider."];
    return self;
}

- (void)appendRole:(NSString*)role text:(NSString*)text
{
    NSMutableParagraphStyle* para = [[NSMutableParagraphStyle alloc] init];
    para.paragraphSpacing = 8;
    NSDictionary* attrs;
    NSString* prefix = @"";
    if ([role isEqual:@"user"]) {
        attrs = @{NSFontAttributeName: [NSFont systemFontOfSize:13 weight:NSFontWeightSemibold], NSForegroundColorAttributeName: NSColor.controlAccentColor, NSParagraphStyleAttributeName: para};
    } else if ([role isEqual:@"tool"]) {
        attrs = @{NSFontAttributeName: [NSFont monospacedSystemFontOfSize:11 weight:NSFontWeightRegular], NSForegroundColorAttributeName: NSColor.secondaryLabelColor, NSParagraphStyleAttributeName: para};
    } else if ([role isEqual:@"note"]) {
        attrs = @{NSFontAttributeName: [NSFont systemFontOfSize:12], NSForegroundColorAttributeName: NSColor.secondaryLabelColor, NSParagraphStyleAttributeName: para};
    } else {
        attrs = @{NSFontAttributeName: [NSFont systemFontOfSize:13], NSForegroundColorAttributeName: NSColor.labelColor, NSParagraphStyleAttributeName: para};
    }
    NSAttributedString* s = [[NSAttributedString alloc] initWithString:[NSString stringWithFormat:@"%@%@\n", prefix, text] attributes:attrs];
    [_text.textStorage appendAttributedString:s];
    [_text scrollRangeToVisible:NSMakeRange(_text.string.length, 0)];
}

- (void)clear { [_text.textStorage setAttributedString:[[NSAttributedString alloc] initWithString:@""]]; }

- (void)setBusy:(BOOL)busy
{
    _busy = busy;
    _send.title = busy ? @"Stop" : @"Send";
    _input.enabled = !busy;
    _setup.enabled = !busy;
    if (!busy) [self.window makeFirstResponder:_input];
}

- (void)sendOrStop:(id)sender
{
    if (_busy) { if (_onStop) _onStop(); return; }
    NSString* t = [_input.stringValue stringByTrimmingCharactersInSet:NSCharacterSet.whitespaceAndNewlineCharacterSet];
    if (!t.length) return;
    _input.stringValue = @"";
    if (_onSend) _onSend(t);
}

- (void)setupClicked:(id)sender { if (_onSetup) _onSetup(); }

@end

// ------------------------------------------------------------------------------------------------------------ the provider sheet

@interface AssistantSetup : NSObject <NSTextFieldDelegate>
@property (nonatomic, strong) NSPanel* panel;
@end

@implementation AssistantSetup {
    NSPopUpButton* _provider;
    NSTextField* _url;
    NSTextField* _model;
    NSSecureTextField* _key;
    NSTextField* _note;
    ChatConfig _config;
    void (^_done)(void);
    NSWindow* _parent;
}

- (instancetype)initWithParent:(NSWindow*)parent done:(void (^)(void))done
{
    self = [super init];
    _parent = parent;
    _done = done;
    _config = loadChatConfig();

    _provider = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    for (const ChatProviderPreset& p : chatProviderPresets()) [_provider addItemWithTitle:[NSString stringWithUTF8String:p.name]];
    _provider.target = self;
    _provider.action = @selector(providerChanged:);
    _url = [NSTextField textFieldWithString:[NSString stringWithUTF8String:_config.baseUrl.c_str()]];
    _model = [NSTextField textFieldWithString:[NSString stringWithUTF8String:_config.model.c_str()]];
    _key = [[NSSecureTextField alloc] init];
    _key.stringValue = [NSString stringWithUTF8String:_config.apiKey.c_str()];
    _key.placeholderString = @"API key";
    // the preset that matches the stored address
    NSInteger index = (NSInteger)chatProviderPresets().size() - 1;
    for (size_t i = 0; i < chatProviderPresets().size(); ++i)
        if (_config.baseUrl == chatProviderPresets()[i].baseUrl) index = NSInteger(i);
    [_provider selectItemAtIndex:index];

    NSGridView* grid = [NSGridView gridViewWithViews:@[
        @[[NSTextField labelWithString:@"Provider:"], _provider],
        @[[NSTextField labelWithString:@"Address:"], _url],
        @[[NSTextField labelWithString:@"Model:"], _model],
        @[[NSTextField labelWithString:@"API key:"], _key],
    ]];
    grid.rowSpacing = 10;
    grid.columnSpacing = 10;
    [grid columnAtIndex:0].xPlacement = NSGridCellPlacementTrailing;
    for (NSTextField* f in @[_url, _model, _key]) [f.widthAnchor constraintEqualToConstant:300].active = YES;

    _note = [NSTextField wrappingLabelWithString:@"Any service with an OpenAI-compatible API works: DeepSeek, OpenAI, a local Ollama. The key is kept in the macOS keychain. "
                                                  "Only the text of the numbers and reports goes to the provider, never the mesh."];
    _note.font = [NSFont systemFontOfSize:11];
    _note.textColor = NSColor.secondaryLabelColor;
    [_note.widthAnchor constraintEqualToConstant:400].active = YES;

    NSButton* save = [NSButton buttonWithTitle:@"Save" target:self action:@selector(save:)];
    save.keyEquivalent = @"\r";
    NSButton* cancel = [NSButton buttonWithTitle:@"Cancel" target:self action:@selector(cancel:)];
    cancel.keyEquivalent = @"\033";
    NSStackView* buttons = [NSStackView stackViewWithViews:@[cancel, save]];
    buttons.spacing = 10;

    NSStackView* stack = [NSStackView stackViewWithViews:@[grid, _note, buttons]];
    stack.orientation = NSUserInterfaceLayoutOrientationVertical;
    stack.alignment = NSLayoutAttributeTrailing;
    stack.spacing = 16;
    stack.edgeInsets = NSEdgeInsetsMake(20, 20, 20, 20);

    _panel = [[NSPanel alloc] initWithContentRect:NSMakeRect(0, 0, 460, 280) styleMask:NSWindowStyleMaskTitled backing:NSBackingStoreBuffered defer:NO];
    _panel.contentView = stack;
    [_panel setContentSize:stack.fittingSize];
    return self;
}

- (void)providerChanged:(id)sender
{
    const ChatProviderPreset& p = chatProviderPresets()[size_t(_provider.indexOfSelectedItem)];
    if (p.baseUrl[0]) { _url.stringValue = [NSString stringWithUTF8String:p.baseUrl]; _model.stringValue = [NSString stringWithUTF8String:p.model]; }
}

- (void)save:(id)sender
{
    ChatConfig c;
    c.baseUrl = [_url.stringValue stringByTrimmingCharactersInSet:NSCharacterSet.whitespaceCharacterSet].UTF8String;
    c.model = [_model.stringValue stringByTrimmingCharactersInSet:NSCharacterSet.whitespaceCharacterSet].UTF8String;
    c.apiKey = [_key.stringValue stringByTrimmingCharactersInSet:NSCharacterSet.whitespaceAndNewlineCharacterSet].UTF8String;
    saveChatConfig(c);
    [_parent endSheet:_panel];
    if (_done) _done();
}

- (void)cancel:(id)sender { [_parent endSheet:_panel]; }

@end

static AssistantSetup* gSetup;   // alive while the sheet is open

void showAssistantSetup(NSWindow* parent, void (^done)(void))
{
    gSetup = [[AssistantSetup alloc] initWithParent:parent done:done];
    [parent beginSheet:gSetup.panel completionHandler:^(NSModalResponse) { gSetup = nil; }];
}
