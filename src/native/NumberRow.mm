#import "native/NumberRow.h"

#include <algorithm>

@interface NumberRow () {
    NSTextField* _field;
    NSStepper* _stepper;
    int _decimals;
}
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

- (void)fieldChanged:(id)sender { self.value = _field.doubleValue; if (self.onChange) self.onChange(); }
- (void)stepperChanged:(id)sender { self.value = _stepper.doubleValue; if (self.onChange) self.onChange(); }

- (void)setEnabled:(BOOL)on { _field.enabled = on; _stepper.enabled = on; }

@end

