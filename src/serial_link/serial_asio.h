#ifndef H_serial_asio_h
#define H_serial_asio_h
#include <utility>
#include <memory>
#include <variant>
#include <sys/select.h>
#include <rbl/queue.h>
#include <boost/asio.hpp>
#include <rbl/iobuffer.h>
#include "parser.h"

using namespace rbl;
namespace asio = boost::asio;
namespace serial_bridge {
    class SerialAsio {
    public:
        // these two typedefs are intended to distinguish between an IoBuffer that holds a protocol frame
        // (which in this case means has a \n on the end) and a message which has been "deframed"
        // which in this simple case means the \n on the end has been removed. This will have more significance
        // with more complicated wire protocols
        typedef IoBuffer MsgBuffer;
        typedef std::shared_ptr<MsgBuffer> MsgBufferPtr;

        typedef std::function<void(rbl::IoBuffer::UPtr, boost::system::error_code& ec)>  OnRecvCallback;
        typedef std::unique_ptr<SerialAsio>          UPtr;
        /**
         * The constructor only initializes a few member variables.
         * all the action takes place in run()
         */
        // explicit SerialLink();
        SerialAsio(const std::string& dev, int instance_id, asio::io_context& io_context);
        ~SerialAsio();

        void send(IoBuffer::UPtr buffer_uptr);
        void recv(std::function<void(IoBuffer::UPtr, boost::system::error_code&)>);
        void runner(std::function<void(IoBuffer::UPtr up, boost::system::error_code& ec)> cb);
        void run(const OnRecvCallback& cb);
    private:
        std::string         m_device;
        int                 m_instance_id;
        int                 m_serial_fd;
        bool                m_read_is_active;
        asio::io_context&   m_io_context;
        asio::serial_port   m_asio_serial_port;
        std::queue<std::unique_ptr<IoBuffer>>  m_write_buffer_queue;
        std::queue<std::unique_ptr<IoBuffer>>  m_read_msg_queue;

        IoBuffer::UPtr      m_write_buffer_uptr;        //holds the ddata bytes that are currenty being written
        OnRecvCallback      m_read_cb;
        IoBuffer::UPtr      m_read_buffer_uptr;         // holds the databytes that have been read but not processed into messages
        IoBuffer::UPtr      m_input_message_buffer_uptr; // holds the bytes that have been processd from m_read_buffer into the
        LineParser          m_parser;
        void try_write();
        void run_start(OnRecvCallback cb);

    };
} // namespace
#endif