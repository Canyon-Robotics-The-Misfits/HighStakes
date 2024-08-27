#include <memory>

#include "main.h"
#include "config.h"
#include "auto.h"

#define LOGGER "main.cpp"

void initialize() {
	INFO_TEXT("Initialize");

	gui::ScreenGUI &gui = gui::ScreenGUI::access();

	gui.register_autonomous(Auto::NONE, {
		display_name: "NONE",
		display_description: ""
	});
	gui.register_autonomous(Auto::POSITIVE, {
		display_name: "Positive",
		display_description: ""
	});
	gui.register_autonomous(Auto::NEGATIVE, {
		display_name: "Negative",
		display_description: ""
	});
	gui.register_autonomous(Auto::POSITIVE_ELIMS, {
		display_name: "Positive Elims",
		display_description: ""
	});
	gui.register_autonomous(Auto::NEGATIVE_ELIMS, {
		display_name: "Negative Elims",
		display_description: ""
	});
	gui.register_autonomous(Auto::SOLO, {
		display_name: "Solo",
		display_description: ""
	});
}

void disabled() {
	INFO_TEXT("Disabled");
}

void competition_initialize() {}