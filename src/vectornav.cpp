#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <functional>

#include "vn/sensors.h"
#include "vn/thread.h" 

using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class VectorNavPublisher;

struct UserData { 
  VectorNavPublisher* vn;
};

class VectorNavPublisher : public rclcpp::Node
{
  public:
    VectorNavPublisher(std::string vec_port, int vec_baud, int rate)
    : Node("vectornav")
    {
      publisher = this->create_publisher<std_msgs::msg::String>("vn_msg", 10);
      // import and setup vectornav here then do the publishing in the Async handler
      VnSensor vs;
      connectVs(vs, vec_port, vec_baud);
      std::string mn = vs.readModelNumber();
      RCLCPP_INFO(this->get_logger(), "VectorNav connected. Model Number: %s", mn);

      AsciiAsync asciiAsync = (AsciiAsync) 0;
      vs.writeAsyncDataOutputType(asciiAsync); // Turns on Binary Message Type
      SynchronizationControlRegister scr( // synchronizes vecnav off of DAQ clock and triggers at desired rate
        SYNCINMODE_COUNT, // SYNCINMODE_ASYNC, TODO: test sync_in on another vn300, this one is unresponsive
        SYNCINEDGE_RISING,
        0, //(int)(daq_rate/vec_rate-1), // Setting skip factor so that the trigger happens at the desired rate.
        SYNCOUTMODE_IMUREADY,
        SYNCOUTPOLARITY_POSITIVE,
        0,
        1250000);
      vs.writeSynchronizationControl(scr);
      BinaryOutputRegister bor(
        ASYNCMODE_PORT1,
        400/rate,
        COMMONGROUP_TIMEGPS | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE | COMMONGROUP_POSITION | COMMONGROUP_VELOCITY | COMMONGROUP_INSSTATUS, // Note use of binary OR to configure flags.
        TIMEGROUP_NONE,
        IMUGROUP_TEMP | IMUGROUP_PRES,
        GPSGROUP_NONE,
        ATTITUDEGROUP_YPRU,
        INSGROUP_POSU | INSGROUP_VELU,
        GPSGROUP_NONE);
      vs.writeBinaryOutput1(bor);

      // setup vs configuration (TODO: get some of this from YAML)
      vs.writeAsyncDataOutputFrequency(1);
      UserData ud; 
      ud.vn = this; // the Async handler needs to have the current object passed in as a pointer, its not ideal but it is less hacky than a binding
      vs.registerAsyncPacketReceivedHandler(&ud, vecnavBinaryEventHandle);
      for(;;){}
    }

  private:
    static void vecnavBinaryEventHandle(void* userData, Packet &p, size_t index) 
    {
      UserData user_data = *static_cast<UserData*>(userData);
      user_data.vn->_vecnavBinaryEventHandle(p, index);
    }

    void _vecnavBinaryEventHandle(Packet &p, size_t index)
    {
      RCLCPP_INFO(this->get_logger(), "Index: %d", index);
      if(p.type() == Packet::TYPE_BINARY)
      {
        auto message = std_msgs::msg::String();
        message.data = p.datastr();
        if (!p.isCompatible(
          COMMONGROUP_TIMEGPS | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE | COMMONGROUP_POSITION | COMMONGROUP_VELOCITY | COMMONGROUP_INSSTATUS, // Note use of binary OR to configure flags.
          TIMEGROUP_NONE,
          IMUGROUP_TEMP | IMUGROUP_PRES,
          GPSGROUP_NONE,
          ATTITUDEGROUP_YPRU,
          INSGROUP_POSU | INSGROUP_VELU,
          GPSGROUP_NONE)) 
        { 
          // Not the type of binary packet we are expecting.
          return; 
        }
        RCLCPP_INFO(this->get_logger(), "Publishing %s", message.data.c_str());
        publisher->publish(message);
      }
    }

    void connectVs(VnSensor &vs, std::string vec_port, int baudrate) {
      // Default baudrate variable
      int defaultBaudrate;
      // Run through all of the acceptable baud rates until we are connected
      // Looping in case someone has changed the default
      bool baudSet = false;
      // Now let's create a VnSensor object and use it to connect to our sensor.
      while(!baudSet){
        // Make this variable only accessible in the while loop
        static int i = 0;
        defaultBaudrate = vs.supportedBaudrates()[i];
        RCLCPP_INFO(this->get_logger(), "Connecting with default at %d", defaultBaudrate);
        // Default response was too low and retransmit time was too long by default.
        // They would cause errors
        vs.setResponseTimeoutMs(1000); // Wait for up to 1000 ms for response
        vs.setRetransmitDelayMs(50);  // Retransmit every 50 ms

        // Acceptable baud rates 9600, 19200, 38400, 57600, 128000, 115200, 230400, 460800, 921600
        // Data sheet says 128000 is a valid baud rate. It doesn't work with the VN100 so it is excluded.
        // All other values seem to work fine.
        try{
          // Connect to sensor at it's default rate
          if(defaultBaudrate != 128000 && baudrate != 128000)
          {
            vs.connect(vec_port, defaultBaudrate);
            // Issues a change baudrate to the VectorNav sensor and then
            // reconnects the attached serial port at the new baudrate.
            vs.changeBaudRate(baudrate);
            // Only makes it here once we have the default correct
            // cout << "Connected baud rate is " << vs.baudrate();
            baudSet = true;
          }
        }
        // Catch all oddities
        catch(...){
          // Disconnect if we had the wrong default and we were connected
          vs.disconnect();
          sleep(0.2);
        }
        // Increment the default iterator
        i++;
        // There are only 9 available data rates, if no connection
        // made yet possibly a hardware malfunction?
        if(i > 8)
        {
          break;
        }
      }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VectorNavPublisher>("/dev/ttyUSB0", 230400, 20)); // TODO: get these args from yaml
  rclcpp::shutdown();
  return 0;
}
