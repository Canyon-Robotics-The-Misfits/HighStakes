#pragma once

#include <memory>
#include <unordered_map>
#include <string>
#include "auto.h"

namespace gui
{

    struct AutonomousConfig
    {
        std::string display_name;
        std::string display_description;
    };

    class ScreenGUI
    {
    private:
        static std::shared_ptr<ScreenGUI> self;
        ScreenGUI();
        ~ScreenGUI();

        std::unordered_map<Auto, AutonomousConfig> auto_map;
        Auto selected_auto = Auto::NONE;

    public:
        static std::shared_ptr<ScreenGUI> &access() { return self; };

        void setup_ui();

        void register_autonomous(Auto id, AutonomousConfig config);

        Auto get_selected_auto();
    };

}