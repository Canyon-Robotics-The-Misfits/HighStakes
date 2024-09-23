#include <memory>

#include "main.h"
#include "gui/gui.h"
#include "config.h"
#include "autonomous.h"

#define LOGGER "main.cpp"

void initialize() {
	INFO_TEXT("Initializing...");

	gui::ScreenGUI &gui = gui::ScreenGUI::access();

	gui.register_autonomous(auto_routes::Route::NONE, {
		display_name: "NONE",
		display_description: ""
	});
	gui.register_autonomous(auto_routes::Route::POSITIVE, {
		display_name: "Positive",
		display_description: ""
	});
	gui.register_autonomous(auto_routes::Route::NEGATIVE, {
		display_name: "Negative",
		display_description: ""
	});
	gui.register_autonomous(auto_routes::Route::POSITIVE_ELIMS, {
		display_name: "Positive Elims",
		display_description: ""
	});
	gui.register_autonomous(auto_routes::Route::NEGATIVE_ELIMS, {
		display_name: "Negative Elims",
		display_description: ""
	});
	gui.register_autonomous(auto_routes::Route::SOLO, {
		display_name: "Solo",
		display_description: ""
	});
	gui.register_autonomous(auto_routes::Route::SKILLS, {
		display_name: "Skills",
		display_description: ""
	});

	gui.setup_ui();

	pros::c::imu_reset(config::PORT_IMU);
	while (isnan(pros::c::imu_get_heading(config::PORT_IMU)))
	{
		pros::delay(10);
	}
	
	INFO_TEXT("Initialized!");
}

void disabled() {
	INFO_TEXT("Disabled");
}

void competition_initialize() {}