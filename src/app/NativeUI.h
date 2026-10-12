#pragma once

#include <functional>
#include <string>
#include <vector>

// The parts of the interface that belong to macOS itself: the open and save panels and the menu bar.
namespace native {

// Both return an empty string when the user cancels. Extensions without the dot ("obj", "ply"); an empty list allows everything.
std::string openFile(const std::string& title, const std::vector<std::string>& extensions, const std::string& startDir);
std::string saveFile(const std::string& title, const std::string& suggestedName, const std::vector<std::string>& extensions,
                     const std::string& startDir);

enum Command {
    CmdOpen = 1, CmdSave, CmdSaveAs, CmdBinaryPly,
    CmdWireframe, CmdResetCamera, CmdDeveloperMode, CmdShowGrid, CmdShowAssistant,
    CmdSaveBadPatch, CmdLoadBadPatch,
};

struct MenuHandlers {
    std::function<void(int)> run;        // a menu item was chosen
    std::function<bool(int)> enabled;    // is the item available now
    std::function<bool(int)> checked;    // does the item have a check mark
};

// File, View and Debug menus next to the application menu that GLFW creates. Shortcuts: Cmd+O, Cmd+S, Cmd+Shift+S.
void installMenu(const MenuHandlers& handlers);
std::string describeMenu();               // the menu bar as text (testing hook VSA_PRINT_MENU)
void setDebugMenuVisible(bool visible);   // the Debug menu belongs to the developer mode

} // namespace native
