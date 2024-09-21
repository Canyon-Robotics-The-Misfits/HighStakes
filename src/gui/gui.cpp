#include "gui/gui.h"

gui::ScreenGUI &gui::ScreenGUI::access()
{
    static ScreenGUI self;

    return self;
}

void gui::ScreenGUI::left_button_callback(lv_event_t * e)
{
    gui::ScreenGUI &gui = access();

    gui.selected_auto -= 1;

    if (gui.selected_auto < 0)
    {
        gui.selected_auto = 0;
    }

    gui.update_content();
}

void gui::ScreenGUI::right_button_callback(lv_event_t * e)
{
    gui::ScreenGUI &gui = access();

    gui.selected_auto += 1;

    if (gui.selected_auto > (int)gui.registered_routes.size() - 1)
    {
        gui.selected_auto = (int)gui.registered_routes.size() - 1;
    }

    gui.update_content();
}

void gui::ScreenGUI::update_content()
{
    auto route = registered_routes[selected_auto];

    printf("%d, %d\n", selected_auto, (int)route);

    lv_label_set_text(title_label, auto_map[route].display_name.c_str());
    lv_label_set_text(description_label, auto_map[route].display_description.c_str());
}

void gui::ScreenGUI::setup_ui()
{
    lv_obj_t * left_button = lv_btn_create(lv_scr_act());
    lv_obj_align(left_button, LV_ALIGN_BOTTOM_LEFT, 5, -5);
    lv_obj_set_width(left_button, 100);
    lv_obj_set_height(left_button, 75);
    lv_obj_add_event_cb(left_button, left_button_callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t * right_button = lv_btn_create(lv_scr_act());
    lv_obj_align(right_button, LV_ALIGN_BOTTOM_RIGHT, -5, -5);
    lv_obj_set_width(right_button, 100);
    lv_obj_set_height(right_button, 75);
    lv_obj_add_event_cb(right_button, right_button_callback, LV_EVENT_CLICKED, NULL);

    title_label = lv_label_create(lv_scr_act());
    lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 5, 10);
    
    description_label = lv_label_create(lv_scr_act());
    lv_obj_align(description_label, LV_ALIGN_TOP_LEFT, 5, 30);

    update_content();
}

void gui::ScreenGUI::register_autonomous(auto_routes::Route id, AutonomousConfig config)
{
    registered_routes.push_back(id);
    this->auto_map[id] = config;
}

auto_routes::Route gui::ScreenGUI::get_selected_auto()
{
    return registered_routes[selected_auto];
}