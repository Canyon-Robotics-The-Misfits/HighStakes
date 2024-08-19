#include <memory>

#include "main.h"
#include "config.h"
#include "auto.h"

void initialize() {
	std::shared_ptr<gui::GUI> gui = gui::GUI::access();

	gui->register_autonomous(Auto::POSITIVE, {
		display_name: "Positive",
		display_description: ""
	});
}

void disabled() {}

void competition_initialize() {}