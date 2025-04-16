#include <memory>

#include "main.h"
#include "gui/gui.h"
#include "config.h"
#include "autonomous.h"

#define LOGGER "main.cpp"

void initialize() {
	INFO_TEXT("Initializing...");

	gui::ScreenGUI &gui = gui::ScreenGUI::access();

	gui.register_autonomous(gui::Route::NONE, {
		display_name: "NONE",
		display_description: "Do not run any auto"
	});

	gui.register_autonomous(gui::Route::POSITIVE, {
		display_name: "Positive",
		display_description: ""
	});
	gui.register_autonomous(gui::Route::NEGATIVE, {
		display_name: "Negative",
		display_description: ""
	});

	gui.register_autonomous(gui::Route::POSITIVE_ELIMS, {
		display_name: "Positive ELIMS",
		display_description: ""
	});
	gui.register_autonomous(gui::Route::NEGATIVE_ELIMS, {
		display_name: "Negative ELIMS",
		display_description: ""
	});

	gui.register_autonomous(gui::Route::SKILLS, {
		display_name: "Skills",
		display_description: "The skills program"
	});

	gui.setup_ui();

	pros::c::imu_reset(config::PORT_IMU);
	while (pros::c::imu_get_rotation(config::PORT_IMU) == PROS_ERR_F)
	{
		pros::delay(10);
	}
	
	INFO_TEXT("Initialized!");
}

void disabled() {
	INFO_TEXT("Disabled");
}

void competition_initialize() {}