#pragma once

#import <Cocoa/Cocoa.h>

#include "native/Session.h"

// The developer section of the sidebar: the parameters of the pipeline, the report of the last run and the comparison with the
// original layout. Shown only in the developer mode.
@interface DeveloperPanel : NSObject
@property (nonatomic, readonly) NSStackView* view;
@property (nonatomic, readonly) NSButton* compareButton;
- (instancetype)initWithTarget:(id)target compareAction:(SEL)action;
- (DeveloperOptions)options;
- (void)setReport:(NSString*)text;
- (void)setEnabled:(BOOL)enabled;
@end

// A window with monospaced text (the comparison table).
void showTextWindow(NSString* title, NSString* text);
