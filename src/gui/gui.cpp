#include "gui/gui.h"

gui::ScreenGUI &gui::ScreenGUI::access()
{
    static ScreenGUI self;

    return self;
}

void gui::ScreenGUI::setup_ui()
{
}

void gui::ScreenGUI::register_autonomous(auto_routes::Route id, AutonomousConfig config)
{
    this->auto_map[id] = config;
}

auto_routes::Route gui::ScreenGUI::get_selected_auto()
{
    return selected_auto;
}