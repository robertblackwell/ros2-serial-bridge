#ifndef H_serial_link_serial_settings_H
#define H_serial_link_serial_settings_H

#include <string>
#include <vector>

std::vector<std::string> list_serial_devices(std::string dir = "/dev", std::string prefix = "ttyACM");
int open_serial_blocking(std::string path);
int open_serial_non_blocking(std::string path);
void apply_default_settings(int fd);

#endif