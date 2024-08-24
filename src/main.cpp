#include <memory>

#include "main.h"
#include "config.h"
#include "auto.h"

#define LOGGER "main.cpp"

void initialize() {
	std::cout << "HELLO WORLD\n";

	INFO_TEXT("Initialize");


	gui::ScreenGUI &gui = gui::ScreenGUI::access();

	gui.register_autonomous(Auto::POSITIVE, {
		display_name: "Positive",
		display_description: ""
	});
}

void disabled() {
	INFO_TEXT("Disabled");
}

void competition_initialize() {}