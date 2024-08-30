#ifndef COMMANDS_HPP
#define COMMANDS_HPP


#include <functional>
#include <map>
#include <string>
#include <sstream>

// #include <rookie_pico/Definitions.hpp>
#include <pico_interface/PicoInterface.hpp>

using CommandCallbacks = std::map<std::string, std::function<bool(std::string)>>;


#endif // COMMANDS_HPP