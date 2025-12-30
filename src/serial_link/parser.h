#ifndef H_serial_link_parser_h
#define H_serial_link_parser_h
#include <utility>
#include <memory>
#include <variant>
#include <sys/select.h>
#include <queue.h>
#include <iobuffer.h>

using namespace rbl;
namespace serial_bridge {
        struct LineParser {
            rbl::IoBuffer::UPtr m_input_message_buffer_uptr;
            LineParser();
            void consume(rbl::IoBuffer& iob, const std::function<void(IoBuffer::UPtr)>& new_msg_cb);
        };
} // namespace
#endif