#include <memory>

#include "main.h"
#include "config.h"
#include "auto.h"

void initialize() {
	gui::ScreenGUI &gui = gui::ScreenGUI::access();

	gui.register_autonomous(Auto::POSITIVE, {
		display_name: "Positive",
		display_description: ""
	});
}

void disabled() {}

void competition_initialize() {}