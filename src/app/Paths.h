#pragma once

#include <string>

// Where the program finds and puts files. Started from a developer's shell it works in the current directory, as before; started
// from a .app bundle (the current directory is "/" there) it uses the bundle's Resources and per-user folders.
namespace paths {

bool runningFromBundle();
std::string resourcesDir();     // <bundle>/Contents/Resources, empty outside a bundle
std::string userDataDir();      // bundle: ~/Library/Application Support/huge-atlas (created); otherwise "."
std::string debugOutDir();      // bundle: ~/Documents/huge-atlas/DebugOut; otherwise "DebugOut"
std::string defaultMesh();      // the cube: data/cube.obj in the current directory, else in the bundle
std::string settingsDir();      // ~/Library/Application Support/huge-atlas (created), also outside a bundle: the user's choices of the interface
std::string startDir();         // the folder the file dialogs open in first: bundle: ~/Documents; otherwise "."

} // namespace paths
