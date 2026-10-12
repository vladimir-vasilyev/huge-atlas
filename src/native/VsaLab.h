#pragma once

#import <Cocoa/Cocoa.h>

#include "core/MeshTypes.h"

#include <vector>

// The segmentation by hand (the VSA of the paper step by step): seed, partition, fit, Lloyd one step or running, border smoothing,
// proxies inserted / deleted / teleported, weights painted on the model with a brush, the energy of every step plotted. The result
// can be taken as the start of the atlas. A window of its own over the open mesh (the regular mode).
@interface VsaLab : NSObject
// the colors of the faces to show in the 3D view (rgb per face)
@property (nonatomic, copy) void (^onColors)(const std::vector<float>& rgb);
// the brush is on or off (the 3D view paints instead of turning), with this radius in pixels
@property (nonatomic, copy) void (^onBrush)(BOOL on, int radius);
// the regions as the start of the atlas (a label per face)
@property (nonatomic, copy) void (^onUseLabels)(const std::vector<int>& labels);
- (void)attachMesh:(const TriMesh*)mesh;   // null: the mesh is gone; the state of the lab is dropped
- (void)showWindow;
- (void)brushFaces:(const std::vector<int>&)faces;   // faces under the brush
- (void)brushEnded;
- (NSWindow*)window;
- (NSString*)selfTestWithK:(int)k;   // seed, run, insert, delete, teleport, use the regions: a line of what happened (checks without a mouse)
@end
