#pragma once

#import <Cocoa/Cocoa.h>

// The assistant: a chat panel (the transcript, an input line, Send / Stop) and the sheet where the model provider is set up.
@interface ChatPanel : NSView
@property (nonatomic, copy) void (^onSend)(NSString* text);
@property (nonatomic, copy) void (^onStop)(void);
@property (nonatomic, copy) void (^onSetup)(void);
// roles: user, assistant, tool, note (all on the main thread)
- (void)appendRole:(NSString*)role text:(NSString*)text;
- (void)setBusy:(BOOL)busy;
- (void)clear;
@end

// The provider sheet over `parent`; `done` is called after Save.
void showAssistantSetup(NSWindow* parent, void (^done)(void));
