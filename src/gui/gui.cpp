#include "gui/gui.h"

gui::ScreenGUI &gui::ScreenGUI::access()
{
    static ScreenGUI self;

    return self;
}

void gui::ScreenGUI::setup_ui()
{
}

void gui::ScreenGUI::register_autonomous(Auto id, AutonomousConfig config)
{
    this->auto_map[id] = config;
}

Auto gui::ScreenGUI::get_selected_auto()
{
    return selected_auto;
}