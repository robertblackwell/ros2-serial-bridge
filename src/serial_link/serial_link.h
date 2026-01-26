#ifndef H_serial_link_h
#define H_serial_link_h
#include <utility>
#include <memory>
#include <variant>
#include <sys/select.h>
#include <rbl/queue.h>
#include <rbl/iobuffer.h>
#include "parser.h"

using namespace rbl;
namespace serial_bridge {
    class SerialLink {
    public:
        // a protocol frame arrives as an IoBuffer. The deframing process removes the protocol
        // header and trailer info and presents the payload as an IoBuffer. The following typedefs
        // distinguish that process
        typedef IoBuffer MsgBuffer;
        typedef IoBuffer::UPtr MsgBufferUPtr;

        typedef std::function<void(MsgBuffer::UPtr)>  OnRecvCallback;
        typedef std::unique_ptr<SerialLink>          UPtr;
        /**
         * The constructor only initializes a few member variables.
         * all the action takes place in run()
         */
        // explicit SerialLink();
        SerialLink(const std::string& dev, int instance_id);
        ~SerialLink();
        /**
         * Presents a message to the io machinery to be turned into a protocol frame and transmitted
         * Msgs are transmitted in the otder of calls to this function.
         */
        void send_threadsafe(MsgBuffer::UPtr buffer_uptr) const;
        /**
         * SerialLink::run() - calls the "recv_msg_cb()" callback function for each
         * complete message received.
         */
        void run(OnRecvCallback  recv_msg_cb);
    private:

        std::unique_ptr<threadsafe::FdQueue<IoBuffer::UPtr>>      m_output_queue_uptr;
        std::unique_ptr<threadsafe::TriggerQueue<IoBuffer::UPtr>> m_client_queue_uptr;

        OnRecvCallback                     m_recv_callback;
        std::string         m_device;
        int                 m_instance_id;
        int                 m_serial_fd;
        int                 m_output_queue_fd;
        fd_set              m_rfds;
        fd_set              m_wfds;
        fd_set              m_xfds;
        int                 m_nbr_fds;
        struct timeval      tv;
        bool                m_read_eagained_flag;

        IoBuffer::UPtr      m_write_buffer_uptr;        //holds the ddata bytes that are currenty being written
        IoBuffer::UPtr      m_read_buffer_uptr;         // holds the databytes that have been read but not processed into messages
        IoBuffer::UPtr      m_input_message_buffer_uptr; // holds the bytes that have been processd from m_read_buffer into the
        LineParser          m_parser;
        void try_write();
        void try_read();
    };
} // namespace
#endif