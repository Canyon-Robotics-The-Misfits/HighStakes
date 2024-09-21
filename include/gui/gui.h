#pragma once

#include <memory>
#include <unordered_map>
#include <string>
#include "autonomous.h"
#include "liblvgl/lvgl.h"

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

        std::vector<auto_routes::Route> registered_routes;
        int selected_auto = 0;
        std::unordered_map<auto_routes::Route, AutonomousConfig> auto_map;

        lv_obj_t * title_label;
        lv_obj_t * description_label;

        static void left_button_callback(lv_event_t * e);
        static void right_button_callback(lv_event_t * e);

        void update_content();

    public:
        ScreenGUI(ScreenGUI const&) = delete;
        void operator=(ScreenGUI const&) = delete;


        static gui::ScreenGUI &access();

        void setup_ui();

        void register_autonomous(auto_routes::Route id, AutonomousConfig config);

        auto_routes::Route get_selected_auto();
    };

}