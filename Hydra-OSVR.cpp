#include "sixense.h"
#include "sixense_math.hpp"

#include <osvr/ClientKit/Context.h>
#include <osvr/ClientKit/Interface.h>
#include <osvr/ClientKit/InterfaceStateC.h>

#include <map>
#include <string>

namespace {

	typedef enum {
		ONE = SIXENSE_BUTTON_1,
		TWO = SIXENSE_BUTTON_2,
		THREE = SIXENSE_BUTTON_3,
		FOUR = SIXENSE_BUTTON_4,
		START = SIXENSE_BUTTON_START,
		BUMPER = SIXENSE_BUTTON_BUMPER,
		JOYSTICK = SIXENSE_BUTTON_JOYSTICK,
		BUTTON_COUNT
	} BUTTONS;

	std::map<BUTTONS, std::string> buttonNames = {
		{ ONE, "1" },
		{ TWO, "2" },
		{ THREE, "3" },
		{ FOUR, "4" },
		{ START, "middle" },
		{ BUMPER, "bumper" },
		{ JOYSTICK, "joystick/button" }
	};

	typedef enum {
		JOYSTICK_X,
		JOYSTICK_Y,
		TRIGGER,
		AXIS_COUNT
	} AXES;

	std::map<AXES, std::string> axisNames = {
		{ JOYSTICK_X, "joystick/x" },
		{ JOYSTICK_Y, "joystick/y" },
		{ TRIGGER, "trigger" }
	};

	struct controller_interface {
		osvr::clientkit::Interface hand;
		osvr::clientkit::Interface buttons[BUTTON_COUNT];
		osvr::clientkit::Interface axes[AXIS_COUNT];
	};

	std::string sides[2] = { "left", "right" };
	unsigned char hands[2] = { 'L', 'R' };
	controller_interface controllers[2];

	osvr::clientkit::ClientContext *context;

}

SIXENSE_EXPORT int sixenseInit(void)
{
	context = new osvr::clientkit::ClientContext("je.nourish.hydra-osvr");
	for (int i = 0; i < 2; i++) {
		controllers[i].hand = context->getInterface("/me/hands/" + sides[i]);
		for (std::pair<BUTTONS, std::string> pair : buttonNames) {
			controllers[i].buttons[pair.first] = context->getInterface("/controller/" + sides[i] + "/" + pair.second);
		}
		for (std::pair<AXES, std::string> pair : axisNames) {
			controllers[i].axes[pair.first] = context->getInterface("/controller/" + sides[i] + "/" + pair.second);
		}
	}

	return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseExit(void)
{
	delete context;
	return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseGetMaxBases()
{
	return 4;
}
SIXENSE_EXPORT int sixenseSetActiveBase(int i)
{
	return SIXENSE_SUCCESS;
}
SIXENSE_EXPORT int sixenseIsBaseConnected(int i)
{
	if (i == 0)
		return 1;
	return 0;
}

SIXENSE_EXPORT int sixenseGetMaxControllers(void)
{
	return SIXENSE_MAX_CONTROLLERS;
}

SIXENSE_EXPORT int sixenseIsControllerEnabled(int which)
{
	if (which <= 1)
		return 1;
	return 0;
}
SIXENSE_EXPORT int sixenseGetNumActiveControllers()
{
	return 2;
}

SIXENSE_EXPORT int sixenseGetHistorySize()
{
	return 0;
}

std::vector<uint8_t> sequence_numbers = { 0, 0 };

SIXENSE_EXPORT int sixenseGetData(int which, int index_back, sixenseControllerData *output)
{
	if (index_back != 0 || which > 2)
		return SIXENSE_FAILURE;

	output->sequence_number = sequence_numbers[which]++;
	output->controller_index = which;
	output->which_hand = hands[which];
	output->enabled = controllers[which].hand.notEmpty();
	output->firmware_revision = 174;
	output->hardware_revision = 0;
	output->is_docked = false;
	output->hemi_tracking_enabled = 1;
	output->magnetic_frequency = 0;
	output->packet_type = 1;

	context->update();

	OSVR_PoseState pose;
	OSVR_TimeValue timestamp;
	OSVR_ReturnCode ret = osvrGetPoseState(controllers[which].hand.get(), &timestamp, &pose);

	if (OSVR_RETURN_SUCCESS == ret) {
		output->pos[0] = osvrVec3GetX(&pose.translation) * 1000;
		output->pos[1] = osvrVec3GetY(&pose.translation) * 1000;
		output->pos[2] = osvrVec3GetZ(&pose.translation) * 1000;

		sixenseMath::Quat(osvrQuatGetX(&pose.rotation), osvrQuatGetY(&pose.rotation),
			osvrQuatGetZ(&pose.rotation), osvrQuatGetW(&pose.rotation)).fill((float*)&output->rot_quat);

		sixenseMath::Matrix3::rotation((sixenseMath::Quat) output->rot_quat).fill((float(*)[3])&output->rot_mat);
	}

	int buttonStates = 0;
	OSVR_ButtonState buttonState;
	for (std::pair<BUTTONS, std::string> pair : buttonNames) {
		ret = osvrGetButtonState(controllers[which].buttons[pair.first].get(), &timestamp, &buttonState);
		if (OSVR_RETURN_SUCCESS == ret && OSVR_BUTTON_PRESSED == buttonState) {
			buttonStates |= pair.first;
		}
	}
	output->buttons = buttonStates;

	OSVR_AnalogState analogState;
	ret = osvrGetAnalogState(controllers[which].axes[TRIGGER].get(), &timestamp, &analogState);
	if (OSVR_RETURN_SUCCESS == ret) {
		output->trigger = (float)analogState;
	}
	ret = osvrGetAnalogState(controllers[which].axes[JOYSTICK_X].get(), &timestamp, &analogState);
	if (OSVR_RETURN_SUCCESS == ret) {
		output->joystick_x = (float)analogState;
	}
	ret = osvrGetAnalogState(controllers[which].axes[JOYSTICK_Y].get(), &timestamp, &analogState);
	if (OSVR_RETURN_SUCCESS == ret) {
		output->joystick_y = (float)analogState;
	}

	return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseGetAllData(int index_back, sixenseAllControllerData *output)
{
	auto success = sixenseGetData(0, index_back, &output->controllers[0]);
	success |= sixenseGetData(1, index_back, &output->controllers[1]);
	return success;
}
SIXENSE_EXPORT int sixenseGetNewestData(int which, sixenseControllerData *output)
{
	return sixenseGetData(which, 0, output);
}

SIXENSE_EXPORT int sixenseGetAllNewestData(sixenseAllControllerData *output)
{
	return sixenseGetAllData(0, output);
}

SIXENSE_EXPORT int sixenseSetHemisphereTrackingMode(int which_controller, int state)
{
	return SIXENSE_SUCCESS;
}
SIXENSE_EXPORT int sixenseGetHemisphereTrackingMode(int which_controller, int *state)
{
	return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseAutoEnableHemisphereTracking(int which_controller)
{
	return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseSetHighPriorityBindingEnabled(int on_or_off)
{
	return SIXENSE_SUCCESS;
}
SIXENSE_EXPORT int sixenseGetHighPriorityBindingEnabled(int *on_or_off)
{
	return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseTriggerVibration(int controller_id, int duration_100ms, int pattern_id)
{
	return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseSetFilterEnabled(int on_or_off)
{
	return SIXENSE_SUCCESS;
}
SIXENSE_EXPORT int sixenseGetFilterEnabled(int *on_or_off)
{
	return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseSetFilterParams(float near_range, float near_val, float far_range, float far_val)
{
	return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseGetFilterParams(float *near_range, float *near_val, float *far_range, float *far_val)
{
	return SIXENSE_SUCCESS;
}

SIXENSE_EXPORT int sixenseSetBaseColor(unsigned char red, unsigned char green, unsigned char blue)
{
	return SIXENSE_SUCCESS;
}
SIXENSE_EXPORT int sixenseGetBaseColor(unsigned char *red, unsigned char *green, unsigned char *blue)
{
	return SIXENSE_SUCCESS;
}