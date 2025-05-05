#pragma once

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
// #pragma GCC diagnostic push 
// #pragma GCC diagnostic ignored "-Wdeprecated-enum-enum-conversion"
#include "liblvgl/lvgl.h"
// #pragma GCC diagnostic pop


namespace gui
{
    enum class AllianceColor
    {
        RED,
        BLUE
    };

    enum class Route
    {
        NONE,

        RIGHT_SAFE,
        LEFT_SAFE,

        POSITIVE,
        NEGATIVE,
        POSITIVE_ELIMS,
        NEGATIVE_ELIMS,

        SOLO,
        SKILLS,
    };

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

        std::vector<Route> registered_routes;
        int selected_auto = 0;
        std::unordered_map<Route, AutonomousConfig> auto_map;

        AllianceColor alliance = AllianceColor::RED;

        lv_obj_t * title_label;
        lv_obj_t * description_label;

        lv_obj_t * alliance_button_label;
        lv_obj_t * calibrate_button_label;

        static void left_button_callback(lv_event_t * e);
        static void right_button_callback(lv_event_t * e);
        static void alliance_button_callback(lv_event_t * e);
        static void calibrate_button_callback(lv_event_t * e);

        void update_content();

    public:
        ScreenGUI(ScreenGUI const&) = delete;
        void operator=(ScreenGUI const&) = delete;


        static gui::ScreenGUI &access();

        void setup_ui();

        void register_autonomous(gui::Route id, AutonomousConfig config);

        gui::Route get_selected_auto();
        AllianceColor get_alliance();
    };

}