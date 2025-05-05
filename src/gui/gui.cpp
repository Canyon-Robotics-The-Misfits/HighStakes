#include "gui/gui.h"

#include "pros/imu.h"
#include "pros/rtos.h"
#include "config.h"

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

void gui::ScreenGUI::alliance_button_callback(lv_event_t * e)
{
    gui::ScreenGUI &gui = access();

    if (gui.alliance == AllianceColor::RED)
    {
        gui.alliance = AllianceColor::BLUE;
    }
    else
    {
        gui.alliance = AllianceColor::RED;
    }

    gui.update_content();
}

void gui::ScreenGUI::calibrate_button_callback(lv_event_t * e)
{
	pros::c::imu_reset(config::PORT_IMU);
	while (pros::c::imu_get_rotation(config::PORT_IMU) == PROS_ERR_F)
	{
		pros::delay(10);
	}
}

void gui::ScreenGUI::update_content()
{
    auto route = registered_routes[selected_auto];

    // printf("%d, %d\n", selected_auto, (int)route);

    lv_label_set_text(title_label, auto_map[route].display_name.c_str());
    lv_label_set_text(description_label, auto_map[route].display_description.c_str());

    if (alliance == AllianceColor::RED)
    {
        lv_label_set_text(alliance_button_label, "RED");
    }
    else
    {
        lv_label_set_text(alliance_button_label, "BLUE");
    }
}

void gui::ScreenGUI::setup_ui()
{
    lv_obj_t * left_button = lv_btn_create(lv_scr_act());
    lv_obj_align(left_button, LV_ALIGN_BOTTOM_LEFT, 5, -5);
    lv_obj_set_width(left_button, 200);
    lv_obj_set_height(left_button, 75);
    lv_obj_add_event_cb(left_button, left_button_callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t * right_button = lv_btn_create(lv_scr_act());
    lv_obj_align(right_button, LV_ALIGN_BOTTOM_RIGHT, -5, -5);
    lv_obj_set_width(right_button, 200);
    lv_obj_set_height(right_button, 75);
    lv_obj_add_event_cb(right_button, right_button_callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t * alliance_button = lv_btn_create(lv_scr_act());
    lv_obj_align(alliance_button, LV_ALIGN_TOP_RIGHT, -5, -5);
    lv_obj_set_width(alliance_button, 100);
    lv_obj_set_height(alliance_button, 75);
    lv_obj_add_event_cb(alliance_button, alliance_button_callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t * calibrate_button = lv_btn_create(lv_scr_act());
    lv_obj_align(calibrate_button, LV_ALIGN_TOP_RIGHT, -110, -5);
    lv_obj_set_width(calibrate_button, 100);
    lv_obj_set_height(calibrate_button, 75);
    lv_obj_add_event_cb(calibrate_button, calibrate_button_callback, LV_EVENT_CLICKED, NULL);

    alliance_button_label = lv_label_create(alliance_button);
    lv_obj_align(alliance_button_label, LV_ALIGN_CENTER, 0, 0);

    calibrate_button_label = lv_label_create(calibrate_button);
    lv_obj_align(calibrate_button_label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(calibrate_button_label, "Calibrate");

    title_label = lv_label_create(lv_scr_act());
    lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 15, 20);
    
    description_label = lv_label_create(lv_scr_act());
    lv_obj_align(description_label, LV_ALIGN_TOP_LEFT, 15, 40);

    update_content();
}

void gui::ScreenGUI::register_autonomous(gui::Route id, AutonomousConfig config)
{
    registered_routes.push_back(id);
    this->auto_map[id] = config;
}

gui::Route gui::ScreenGUI::get_selected_auto()
{
    return registered_routes[selected_auto];
}

gui::AllianceColor gui::ScreenGUI::get_alliance()
{
    return alliance;
}