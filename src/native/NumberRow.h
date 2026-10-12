#pragma once

#import <Cocoa/Cocoa.h>

// "label  [number] [stepper]": a number with a field and a stepper in one line.
@interface NumberRow : NSStackView
- (instancetype)initWithTitle:(NSString*)title min:(double)lo max:(double)hi step:(double)step value:(double)value decimals:(int)decimals;
@property (nonatomic) double value;
@property (nonatomic, copy) void (^onChange)(void);
- (void)setEnabled:(BOOL)on;
@end
