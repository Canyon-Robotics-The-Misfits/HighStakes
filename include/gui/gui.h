#pragma once

#include <memory>
#include <unordered_map>
#include <string>
#include "autonomous.h"

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
        static ScreenGUI self;
        ScreenGUI() {};

        std::unordered_map<auto_routes::Route, AutonomousConfig> auto_map;
        auto_routes::Route selected_auto = auto_routes::Route::NONE;

    public:
        ScreenGUI(ScreenGUI const&) = delete;
        void operator=(ScreenGUI const&) = delete;


        static ScreenGUI& access();

        void setup_ui();

        void register_autonomous(auto_routes::Route id, AutonomousConfig config);

        auto_routes::Route get_selected_auto();
    };

}