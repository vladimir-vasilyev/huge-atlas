#pragma once

#import <Cocoa/Cocoa.h>

#include "native/Session.h"

#include "native/Session.h"

// The developer section of the sidebar: the parameters of the pipeline, the report of the last run and the comparison with the
// original layout. Shown only in the developer mode.
@interface DeveloperPanel : NSObject
@property (nonatomic, readonly) NSStackView* view;
@property (nonatomic, readonly) NSButton* compareButton;
- (instancetype)initWithTarget:(id)target compareAction:(SEL)action matchAction:(SEL)match inspectAction:(SEL)inspect againAction:(SEL)again;
- (DeveloperOptions)options;
- (void)setReport:(NSString*)text;
- (void)setInfo:(NSString*)text;
// How the 3D view is drawn: anti-aliasing (0 off, 2, 4, 8 samples), back faces, the UV checker (squares along the model) and the GPU time.
@property (nonatomic, copy) void (^onViewOptions)(void);
@property (nonatomic, readonly) int msaa;
@property (nonatomic) BOOL cullBackFaces;
@property (nonatomic, readonly) BOOL uvChecker;
@property (nonatomic, readonly) int checkerSquares;
- (void)setGpuMilliseconds:(double)ms;
- (void)setUvChecker:(BOOL)on msaa:(int)samples;   // for the checks without a mouse
- (void)setResultViewsEnabled:(BOOL)on;   // the UV checker needs an atlas   // the diagnostics of the open mesh
// The patch table: a click selects a patch (-1: none), the color mode says how the patches are painted (0 colors, 1 problems, 2 stretch).
- (void)setPatchRows:(std::vector<PatchRow>)rows;
- (void)selectPatchIndex:(NSInteger)patch;
- (void)setPaintMode:(NSInteger)mode;   // 0 patch colors, 1 problems, 2 stretch (as the popup does)
- (void)revealPatchTable;                // scrolls the sidebar so that the patch table is in view   // the row of this patch (-1: none) without calling onPatchSelected
- (PatchRow)rowForPatch:(NSInteger)patch found:(BOOL*)found;
@property (nonatomic, copy) void (^onPatchSelected)(NSInteger patch);
@property (nonatomic, copy) void (^onColorMode)(NSInteger mode);
@property (nonatomic, readonly) NSInteger colorMode;
- (void)setEnabled:(BOOL)enabled;
@end

// A window with monospaced text (the comparison table).
void showTextWindow(NSString* title, NSString* text);
