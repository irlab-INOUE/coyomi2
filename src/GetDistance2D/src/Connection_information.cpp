/*!
  ¥file
  ¥brief 接続情報の管理
  ¥author Satofumi KAMIMURA

  $Id: Connection_information.cpp,v c5747add6615 2015/05/07 03:18:34 alexandr $
*/

#include "Connection_information.h"
#include "detect_os.h"
#include <string>
#include <cstring>

using namespace std;
using namespace qrk;


struct Connection_information::pImpl
{
    Urg_driver::connection_type_t connection_type_;
    string device_or_ip_name_;
    long baudrate_or_port_number_;


    void set_serial_connection(const char* device = NULL)
    {
        connection_type_ = Urg_driver::Serial;
        if (device != NULL) {
            device_or_ip_name_ = device;
        } else {
#if defined(QRK_WINDOWS_OS)
        //device_or_ip_name_ = "COM3";
#elif defined(QRK_LINUX_OS)
        device_or_ip_name_ = "/dev/ttyACM1";
#else
        //device_or_ip_name_ = "/dev/tty.usbmodemfa131";
#endif
        }
        baudrate_or_port_number_ = 115200;
    }


    void set_ethernet_connection(const char* ip_address = NULL)
    {
        connection_type_ = Urg_driver::Ethernet;
        if (ip_address != NULL) {
            device_or_ip_name_ = ip_address;
        } else {
            //device_or_ip_name_ = "localhost";
            //device_or_ip_name_ = "192.168.0.10";

			//Front Urg
            //device_or_ip_name_ = "172.16.22.253";

			// Rear Urg
			device_or_ip_name_ = "172.16.20.213";
        }
        baudrate_or_port_number_ = 10940;
    }
};


Connection_information::Connection_information(int argc,
                                               const char*const argv[])
    : pimpl(new pImpl)
{
#ifdef DEFAULT
	const char* device = NULL;
  pimpl->set_serial_connection(device);
	//pimpl->set_ethernet_connection(device);
	return;
#endif
//#define SWITCH
#ifdef SWITCH
    for (int i = 1; i < argc; ++i) {
        const char* device = NULL;
        if (!strcmp(argv[i], "-e")) {
            if (argc > i + 1) {
                device = argv[i + 1];
            }
            pimpl->set_ethernet_connection(device);
            return;
        }
        if (!strcmp(argv[i], "-s")) {
            if (argc > i + 1) {
                device = argv[i + 1];
            }
            pimpl->set_serial_connection(device);
            return;
        }
    }
    pimpl->set_ethernet_connection();
    //pimpl->set_serial_connection();
#endif
}

Connection_information::Connection_information() : pimpl(new pImpl) {
  pimpl->set_serial_connection();
  //pimpl->set_ethernet_connection();
}

Connection_information::~Connection_information(void)
{
}


Urg_driver::connection_type_t
Connection_information::connection_type(void) const
{
    return pimpl->connection_type_;
}


const char* Connection_information::device_or_ip_name(void) const
{
    return pimpl->device_or_ip_name_.c_str();
}


long Connection_information::baudrate_or_port_number(void) const
{
    return pimpl->baudrate_or_port_number_;
}
