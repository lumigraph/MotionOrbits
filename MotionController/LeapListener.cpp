#include "LeapListener.h"

using namespace Leap;

static const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
static const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
static const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};

void LeapListener::onInit(const Controller& controller) {
  std::cout << "Initialized" << std::endl;
}

void LeapListener::onConnect(const Controller& controller) {
  std::cout << "Connected" << std::endl;
 // controller.enableGesture(Gesture::TYPE_CIRCLE);
  controller.enableGesture(Gesture::TYPE_KEY_TAP);
 // controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
 // controller.enableGesture(Gesture::TYPE_SWIPE);
}

void LeapListener::onDisconnect(const Controller& controller) {
  // Note: not dispatched when running in a debugger.
  std::cout << "Disconnected" << std::endl;
}

void LeapListener::onExit(const Controller& controller) {
  std::cout << "Exited" << std::endl;
}

void LeapListener::onFrame(const Controller& controller) {
}

void LeapListener::onFocusGained(const Controller& controller) {
  std::cout << "Focus Gained" << std::endl;
}

void LeapListener::onFocusLost(const Controller& controller) {
  std::cout << "Focus Lost" << std::endl;
}

void LeapListener::onDeviceChange(const Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
  }
}

void LeapListener::onServiceConnect(const Controller& controller) {
  std::cout << "Service Connected" << std::endl;
}

void LeapListener::onServiceDisconnect(const Controller& controller) {
  std::cout << "Service Disconnected" << std::endl;
}

