/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Team-Xmbot-Service-Robot
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Team-Xmbot-Service-Robot nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

// Authors: startar, myyerrol
// Created: 2012.4.8, 2016.4.15

#ifndef XM_ARM_HARDWARE_SERIAL_PORT_H_
#define XM_ARM_HARDWARE_SERIAL_PORT_H_

#include <xm_arm_msgs/xm_ArmSerialData.h>
#include <xm_arm_msgs/xm_ArmSerialDatagram.h>
#include <iomanip>
#include <vector>
#include <queue>
#include <stdio.h>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/make_shared.hpp>

using namespace std;
using namespace boost;
using namespace boost::asio;

namespace xm_serial_node
{
class SerialParams
{
public:
    SerialParams() :
        serial_port_(),
        baud_rate_(115200),
        flow_control_(0),
        parity_bits_(0),
        stop_bits_(0)
    {
    }
    SerialParams(
        string       serial_port,
        unsigned int baud_rate,
        unsigned int flow_control,
        unsigned int parity_bits,
        unsigned int stop_bits
        ) :
        serial_port_(serial_port),
        baud_rate_(baud_rate),
        flow_control_(flow_control),
        parity_bits_(parity_bits),
        stop_bits_(stop_bits)
    {
    }
public:
    string       serial_port_;
    unsigned int baud_rate_;
    unsigned int flow_control_;
    unsigned int parity_bits_;
    unsigned int stop_bits_;
};

typedef vector<u_int8_t> byte_vector;
typedef shared_ptr<byte_vector> ptr_byte_vector;

class SerialPort
{
public:
    SerialPort();
    virtual ~SerialPort();
public:
    void setSerialParams(const SerialParams &params);
    void setTimeOut(int timeout);
    bool startThread();
    bool stopThread();
    void setCallbackFunc(
        const function<void(xm_arm_msgs::xm_ArmSerialDatagramPtr)> &func);
    bool writeDataGram(const xm_arm_msgs::xm_ArmSerialDatagram &datagram);
    bool writeRaw(const byte_vector &raw_data);
private:
    void runMain();
    void startOneRead();
    void startOneWrite();
    void readHandler(const system::error_code &ec, size_t bytes_trans);
    void writeHandler(const system::error_code &ec);
    void timeoutHandler(const system::error_code &ec);
private:
    shared_ptr<deadline_timer> ptr_timer_;
    shared_ptr<io_service>     ptr_io_service_;
    shared_ptr<serial_port>    ptr_serial_;
    mutex                      serial_mutex_;

    enum {HEADER_LEN = 4};
    enum STATE {
        WAITING_FF, WAITING_FF2, READING_HEAD, READING_DATA, READING_CHECKSUM
    } state_;

    SerialParams           serial_params_;
    int                    timeout_;
    byte_vector            temp_buffer_;
    byte_vector            current_header_;
    byte_vector            current_data_;
    size_t                 header_bytes_read_;
    size_t                 data_bytes_read_;
    queue<ptr_byte_vector> write_queue_;
    mutex                  write_queue_mutex_;
    thread                 thread_;
    function<void(xm_arm_msgs::xm_ArmSerialDatagramPtr)> data_callback_func_;
    function<void()>                                     error_callback_func_;
};

} // namespace xm_serial_node

#endif // XM_ARM_HARDWARE_SERIAL_PORT_H_
