#import <Cocoa/Cocoa.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include "app/NativeUI.h"

namespace native {

namespace {

NSArray<UTType*>* typesFor(const std::vector<std::string>& extensions)
{
    NSMutableArray<UTType*>* types = [NSMutableArray array];
    for (const std::string& e : extensions) {
        UTType* t = [UTType typeWithFilenameExtension:[NSString stringWithUTF8String:e.c_str()]];
        if (t) [types addObject:t];
    }
    return types;
}

NSString* ns(const std::string& s) { return [NSString stringWithUTF8String:s.c_str()]; }

MenuHandlers gHandlers;
NSMenuItem* gDebugItem = nil;

} // namespace

std::string openFile(const std::string& title, const std::vector<std::string>& extensions, const std::string& startDir)
{
    @autoreleasepool {
        NSOpenPanel* panel = [NSOpenPanel openPanel];
        panel.title = ns(title);
        panel.canChooseFiles = YES;
        panel.canChooseDirectories = NO;
        panel.allowsMultipleSelection = NO;
        if (!extensions.empty()) panel.allowedContentTypes = typesFor(extensions);
        if (!startDir.empty()) panel.directoryURL = [NSURL fileURLWithPath:ns(startDir) isDirectory:YES];
        if ([panel runModal] == NSModalResponseOK && panel.URL) return std::string(panel.URL.path.UTF8String);
    }
    return {};
}

std::string saveFile(const std::string& title, const std::string& suggestedName, const std::vector<std::string>& extensions,
                     const std::string& startDir)
{
    @autoreleasepool {
        NSSavePanel* panel = [NSSavePanel savePanel];
        panel.title = ns(title);
        panel.canCreateDirectories = YES;
        if (!extensions.empty()) panel.allowedContentTypes = typesFor(extensions);
        if (!suggestedName.empty()) panel.nameFieldStringValue = ns(suggestedName);
        if (!startDir.empty()) panel.directoryURL = [NSURL fileURLWithPath:ns(startDir) isDirectory:YES];
        if ([panel runModal] == NSModalResponseOK && panel.URL) return std::string(panel.URL.path.UTF8String);
    }
    return {};
}

} // namespace native

@interface HugeAtlasMenuTarget : NSObject <NSMenuItemValidation>
- (void)fire:(NSMenuItem*)item;
@end

@implementation HugeAtlasMenuTarget
- (void)fire:(NSMenuItem*)item
{
    if (native::gHandlers.run) native::gHandlers.run(int(item.tag));
}
- (BOOL)validateMenuItem:(NSMenuItem*)item
{
    const int cmd = int(item.tag);
    if (native::gHandlers.checked && (cmd == native::CmdBinaryPly || cmd == native::CmdWireframe || cmd == native::CmdDeveloperMode || cmd == native::CmdShowGrid))
        item.state = native::gHandlers.checked(cmd) ? NSControlStateValueOn : NSControlStateValueOff;
    return native::gHandlers.enabled ? native::gHandlers.enabled(cmd) : YES;
}
@end

namespace native {

namespace {

NSMenuItem* addItem(NSMenu* menu, HugeAtlasMenuTarget* target, NSString* title, int cmd, NSString* key,
                    NSEventModifierFlags mods = NSEventModifierFlagCommand)
{
    NSMenuItem* item = [[NSMenuItem alloc] initWithTitle:title action:@selector(fire:) keyEquivalent:key];
    item.target = target;
    item.tag = cmd;
    if (key.length) item.keyEquivalentModifierMask = mods;
    [menu addItem:item];
    return item;
}

NSMenuItem* addMenu(NSMenu* bar, NSInteger index, NSString* title, NSMenu** out)
{
    NSMenu* menu = [[NSMenu alloc] initWithTitle:title];
    NSMenuItem* top = [[NSMenuItem alloc] initWithTitle:title action:nil keyEquivalent:@""];
    top.submenu = menu;
    [bar insertItem:top atIndex:MIN(index, (NSInteger)bar.numberOfItems)];
    *out = menu;
    return top;
}

} // namespace

void installMenu(const MenuHandlers& handlers)
{
    gHandlers = handlers;
    static HugeAtlasMenuTarget* target = [[HugeAtlasMenuTarget alloc] init];
    NSMenu* bar = [NSApp mainMenu];
    if (!bar) return;

    NSMenu* file = nil;
    addMenu(bar, 1, @"File", &file);
    addItem(file, target, @"Open…", CmdOpen, @"o");
    [file addItem:[NSMenuItem separatorItem]];
    addItem(file, target, @"Save", CmdSave, @"s");
    addItem(file, target, @"Save As…", CmdSaveAs, @"s", NSEventModifierFlagCommand | NSEventModifierFlagShift);
    [file addItem:[NSMenuItem separatorItem]];
    addItem(file, target, @"Binary PLY", CmdBinaryPly, @"");

    NSMenu* view = nil;
    addMenu(bar, 2, @"View", &view);
    addItem(view, target, @"Reset Camera", CmdResetCamera, @"0");
    addItem(view, target, @"Show Grid", CmdShowGrid, @"g", NSEventModifierFlagCommand | NSEventModifierFlagOption);
    addItem(view, target, @"Wireframe", CmdWireframe, @"");
    [view addItem:[NSMenuItem separatorItem]];
    addItem(view, target, @"Developer Mode", CmdDeveloperMode, @"");

    NSMenu* debug = nil;
    gDebugItem = addMenu(bar, 3, @"Debug", &debug);
    addItem(debug, target, @"Save Bad Patch", CmdSaveBadPatch, @"");
    addItem(debug, target, @"Load Bad Patch…", CmdLoadBadPatch, @"");
    gDebugItem.hidden = YES;
}

std::string describeMenu()
{
    std::string out;
    NSMenu* bar = [NSApp mainMenu];
    for (NSMenuItem* top in bar.itemArray) {
        out += std::string(top.title.UTF8String) + (top.hidden ? " (hidden)" : "") + ":";
        for (NSMenuItem* item in top.submenu.itemArray) {
            if (item.isSeparatorItem) { out += " |"; continue; }
            out += " [" + std::string(item.title.UTF8String);
            if (item.keyEquivalent.length) out += " " + std::string(item.keyEquivalent.UTF8String);
            out += "]";
        }
        out += "\n";
    }
    return out;
}

void setDebugMenuVisible(bool visible)
{
    if (gDebugItem) gDebugItem.hidden = visible ? NO : YES;
}

} // namespace native
