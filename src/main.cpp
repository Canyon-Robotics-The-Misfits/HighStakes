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
	
	gui.register_autonomous(gui::Route::RIGHT_SAFE, {
		display_name: "Right Safe",
		display_description: "Score the preload and one other ring on a mogo and touch the ladder"
	});
	gui.register_autonomous(gui::Route::LEFT_SAFE, {
		display_name: "Left Safe",
		display_description: "Score the preload and one other ring on a mogo and touch the ladder"
	});

	gui.register_autonomous(gui::Route::POSITIVE, {
		display_name: "Positive",
		display_description: "Score 5 rings on a mogo, 1 on alliance stake, and touch the ladder"
	});
	gui.register_autonomous(gui::Route::NEGATIVE, {
		display_name: "Negative",
		display_description: ""
	});

	// gui.register_autonomous(gui::Route::SOLO, {
	// 	display_name: "Solo",
	// 	display_description: ""
	// });
	gui.register_autonomous(gui::Route::SKILLS, {
		display_name: "Skills",
		display_description: "The skills program"
	});

	gui.setup_ui();

	pros::c::imu_reset(config::PORT_IMU);
	// while (pros::c::imu_get_rotation(config::PORT_IMU) == PROS_ERR_F && errno == EAGAIN)
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