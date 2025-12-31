#include <utility>
#include <cassert>
#include <cerrno>
#include <format>
#include <functional>
#include <memory>
#include <sys/select.h>
#include <chrono>
#include <rbl/logger.h>
#include <rbl/iobuffer.h>
#include "parser.h"

#define CARRIAGE_RETURN '\r'
#define LINE_FEED '\n'

serial_bridge::LineParser::LineParser()
{
    m_input_message_buffer_uptr = std::make_unique<IoBuffer>();
}
void serial_bridge::LineParser::consume(rbl::IoBuffer& iob, const std::function<void(IoBuffer::UPtr)>& new_msg_cb)
{
    char* p;
    while((! iob.empty()) && (p = iob.get_first_char_ptr()) != nullptr) {
        char ch = *p;
        iob.consume(1);
        if(ch == CARRIAGE_RETURN) {
            RBL_LOG_FMT("got a CR character - these should not be present in the LINE protocol.");
        } else if(ch == LINE_FEED) {
            new_msg_cb(std::move(m_input_message_buffer_uptr));
            m_input_message_buffer_uptr = nullptr;
            m_input_message_buffer_uptr = std::make_unique<IoBuffer>();
        } else {
            m_input_message_buffer_uptr->append(&ch, 1);
        }
    }
}
