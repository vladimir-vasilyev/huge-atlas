#pragma once

#import <Cocoa/Cocoa.h>

#include "native/Session.h"

#include "native/Session.h"

// The developer section of the sidebar: the parameters of the pipeline, the report of the last run and the comparison with the
// original layout. Shown only in the developer mode.
@interface DeveloperPanel : NSObject
@property (nonatomic, readonly) NSStackView* view;
@property (nonatomic, readonly) NSButton* compareButton;
- (instancetype)initWithTarget:(id)target compareAction:(SEL)action;
- (DeveloperOptions)options;
- (void)setReport:(NSString*)text;
// The patch table: a click selects a patch (-1: none), the color mode says how the patches are painted (0 colors, 1 problems, 2 stretch).
- (void)setPatchRows:(std::vector<PatchRow>)rows;
@property (nonatomic, copy) void (^onPatchSelected)(NSInteger patch);
@property (nonatomic, copy) void (^onColorMode)(NSInteger mode);
@property (nonatomic, readonly) NSInteger colorMode;
- (void)setEnabled:(BOOL)enabled;
@end

// A window with monospaced text (the comparison table).
void showTextWindow(NSString* title, NSString* text);
