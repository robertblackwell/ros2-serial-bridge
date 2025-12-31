#ifndef H_sync_serial_link_h
#define H_sync_serial_link_h
#include <utility>
#include <memory>
#include <variant>
#include <sys/select.h>
#include <rbl/queue.h>
#include <rbl/iobuffer.h>

using namespace rbl;
namespace serial_bridge {
    class SyncSerialLink {
    public:
    typedef std::function<void(IoBuffer::UPtr)>  OnRecvCallback;
    typedef std::unique_ptr<SyncSerialLink>          UPtr;
        SyncSerialLink(const std::string& dev);
        ~SyncSerialLink();

        void send(char* buf, std::size_t buflen) const;
        std::size_t recv_until(char* buf, size_t buflen, char terminator) const;
        std::size_t recv(char* buf, size_t buflen) const;
    private:
        std::string         m_device;
        int                 m_serial_fd;
    };
} // namespace
#endif