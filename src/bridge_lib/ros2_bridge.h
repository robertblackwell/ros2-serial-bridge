#ifndef H_port_h
#define H_port_h
#include <utility>
#include <memory>
#include <variant>
#include <sys/select.h>
#include "queue.h"
#include "iobuffer.h"
#include "SerialPort.h"
#include "msgs.h"


namespace ros2_bridge {
    class Bridge {
    public:
        /**
         * This file descriptor fd, should have been openned and all termios and fctnl settings applied
         * before calling this constructor.
         * @param fd
         * @param client_queue
         */
        Bridge(
            int serial_fd, 
            threadsafe::TriggerQueue<Message>& receive_queue,
            threadsafe::FdQueue<Message>&      send_queue);
        void send_threadsafe(Message msg);
        void run(); 


        threadsafe::FdQueue<Message>      & m_output_queue;
        threadsafe::TriggerQueue<Message> & m_client_queue;
        int m_serial_fd;
        int m_output_queue_fd;

        fd_set m_rfds;
        fd_set m_wfds;
        fd_set m_xfds;
        int    m_nbr_fds;
        struct timeval  tv;
        IoBuffer m_write_buffer;        //holds the ddata bytes that are currenty being written
        IoBuffer m_read_buffer;         // holds the databytes that have been read but not processed into messages
        IoBuffer m_input_message_buffer;// holds the bytes that have been processd from m_read_buffer into the
        void try_write();
        void try_read();
    };
} // namespace
#endif