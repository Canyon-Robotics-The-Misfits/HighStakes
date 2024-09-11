#include "main.h"
#include "gui/gui.h"
#include "config.h"
#include "autonomous.h"

#define LOGGER "autonomous.cpp"

void autonomous() {
	INFO_TEXT("Autonomous Start");

	double start_time = pros::millis() / 1000.0;

	gui::ScreenGUI &gui = gui::ScreenGUI::access();

	switch (gui.get_selected_auto())
	{
		case auto_routes::Route::POSITIVE: {

		} break;
		case auto_routes::Route::NEGATIVE: {

		} break;
		case auto_routes::Route::POSITIVE_ELIMS: {

		} break;
		case auto_routes::Route::NEGATIVE_ELIMS: {

		} break;
		case auto_routes::Route::SOLO: {

		} break;
		default: break;
	}

	double end_time = pros::millis() / 1000.0;
	
	INFO("Autonomous End. Took %.2fs", end_time - start_time);
}