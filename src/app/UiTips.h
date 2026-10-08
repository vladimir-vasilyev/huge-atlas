#pragma once

#include <imgui.h>

#include <utility>

// Every control of the interface explains itself in a tooltip whenever the mouse rests on it. The text comes from a table keyed by the label of the control (UiTips.cpp). `ui::X` are drop-in
// replacements for `ImGui::X` that call uiTip() right after the widget.

// Shows the tooltip of `label` if it has one. Call right after a widget.
void uiTip(const char* label);
void uiTipPreview(const char* label, float x, float y);   // testing hook (VSA_TIP_PREVIEW)

namespace ui {

#define UI_WIDGET(NAME)                                                       \
    template <class... A>                                                     \
    inline auto NAME(const char* label, A&&... a)                             \
    {                                                                         \
        auto r = ImGui::NAME(label, std::forward<A>(a)...);                   \
        uiTip(label);                                                         \
        return r;                                                             \
    }

UI_WIDGET(InputInt)
UI_WIDGET(InputFloat)
UI_WIDGET(SliderInt)
UI_WIDGET(SliderFloat)
UI_WIDGET(Checkbox)
UI_WIDGET(RadioButton)
UI_WIDGET(Combo)
UI_WIDGET(BeginCombo)
UI_WIDGET(Button)
UI_WIDGET(SmallButton)
UI_WIDGET(ColorEdit3)
UI_WIDGET(MenuItem)

#undef UI_WIDGET

} // namespace ui
