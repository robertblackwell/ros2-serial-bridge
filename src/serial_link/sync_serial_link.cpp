#include <utility>
#include <cerrno>
#include <format>
#include <functional>
#include <memory>
#include <sys/select.h>
#include <chrono>
#include <logger.h>
#include "sync_serial_link.h"
#include "serial_settings.h"

#define CARRIAGE_RETURN '\r'
#define LINE_FEED '\n'

serial_bridge::SyncSerialLink::SyncSerialLink(const std::string& dev )
{
    m_serial_fd = -1;
    m_device = std::string{dev};
    int fd = open_serial_non_blocking(dev);
    if (fd < 0) {
        throw std::runtime_error(std::format("Failed to open serial device {}", dev.c_str()));
    }
    m_serial_fd = fd;
    RBL_LOG_FMT("succeeded in openning %s fd: %d ", dev.c_str(), m_serial_fd)
    apply_default_settings(fd);
}

serial_bridge::SyncSerialLink::~SyncSerialLink()
{
    close(m_serial_fd);
}
void serial_bridge::SyncSerialLink::send(char* buf, std::size_t buflen) const
{
    ssize_t n = write(m_serial_fd, buf, buflen);
    int errno_saved = errno;
    if (n == -1) {
        throw std::runtime_error(std::format("write failed errno: {}", errno_saved));
    } else if ((n > 0) && (static_cast<std::size_t>(n) != buflen)) {
        throw std::runtime_error(std::format("write maybe failed bytes_written less that bytes presented errno: {}", errno_saved));
    } else if (n == 0) {
        throw std::runtime_error(std::format("write maybe failed bytes_written == 0 errno: {}", errno_saved));
    }
}
std::size_t serial_bridge::SyncSerialLink::recv_until(char* buf, size_t buflen, char terminator) const
{
    char* p = buf;
    size_t count = 0;
    while (true) {
        ssize_t n = read(m_serial_fd, &p, 1);
        int errno_saved = errno;
        if (n < 0) {
            throw std::runtime_error(std::format("read failed errno: {}", errno_saved));
        } else if (n == 0) {
            throw std::runtime_error(std::format("read failed return zero bytes errno: {}", errno_saved));
        } else if (*p == terminator) {
            return count;
        }
        count++;
        if (count >= buflen) {
            return count;
        }
        p++;
    }
}
std::size_t serial_bridge::SyncSerialLink::recv(char* buf, size_t buflen) const
{
    char* p = buf;
    size_t count = 0;
    while (true) {
        ssize_t n = read(m_serial_fd, &p, 1);
        int errno_saved = errno;
        if (n < 0) {
            throw std::runtime_error(std::format("read failed errno: {}", errno_saved));
        } else if (n == 0) {
            throw std::runtime_error(std::format("read failed return zero bytes errno: {}", errno_saved));
        }
        count++;
        if (count >= buflen) {
            return count;
        }
        p++;
    }
}
