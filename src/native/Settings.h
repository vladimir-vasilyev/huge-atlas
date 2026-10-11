#pragma once

#import <Cocoa/Cocoa.h>

// The preferences of the native window (NSUserDefaults) and the Settings window (Cmd+,): a toolbar with two panes, General and Advanced.
namespace prefs {

struct Values {
    int  quality = 1;           // 0 fast, 1 balanced, 2 detailed
    int  textureSize = 2048;
    int  mipLevels = 3;
    int  hugeMillions = 3;      // meshes above this many million faces go through the huge mode
    bool openUV = true;         // show the UV Layout when an atlas is made
    bool developer = false;     // the developer mode: parameters of the pipeline, a report, the comparison with the original layout
};

Values load();
void save(const Values& v);
void reset();

// posted (object: nil) after the Settings window changed something
extern NSNotificationName const Changed;

} // namespace prefs

// The Settings window (made on first use, then shown again).
void showSettingsWindow(NSInteger tab = 0);
NSWindow* settingsWindow();
