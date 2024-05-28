#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include "../../inertial-sense-sdk/cltool/src/cltool.h"
#include "InertialSense.h"

using namespace std::chrono_literals;
using namespace std;

class ImuPublisher : public rclcpp::Node
{
public:
  ImuPublisher()
  : Node("imu_publisher"), count_(0)
  {
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data/test", 5);
    cltool_dataStreaming();
  }

  static ImuPublisher* instance_;

private:
  void imu_callback(p_data_t* data)
  {
    sensor_msgs::msg::Imu imu_msg;

    // uDatasets is a union of all datasets that we can receive. See data_sets.h for a full list of all available datasets.
    uDatasets d = {};
    copyDataPToStructP(&d, data, sizeof(uDatasets));

    imu_msg.orientation.w = d.ins2.qn2b[0];
    imu_msg.orientation.x = d.ins2.qn2b[1];
    imu_msg.orientation.y = d.ins2.qn2b[2];
    imu_msg.orientation.z = d.ins2.qn2b[3];

    imu_msg.linear_acceleration.x = 0;
    imu_msg.linear_acceleration.y = 1;
    imu_msg.linear_acceleration.z = 0;

    imu_msg.angular_velocity.x = 0;
    imu_msg.angular_velocity.y = 0;
    imu_msg.angular_velocity.z = 1;

    RCLCPP_INFO(this->get_logger(), "Publishing: imu data");
    imu_publisher_->publish(imu_msg);
  }

  static void cltool_dataCallback(InertialSense* i, p_data_t* data, int pHandle)
  {
    // Ensure this static callback function has access to the ImuPublisher instance
    if (instance_)
    {
      instance_->imu_callback(data);
    }
  }

  static int cltool_dataStreaming()
  {
    InertialSense inertialSenseInterface(cltool_dataCallback);
    cmd_options_t g_commandLineOptions = {};

    if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate, g_commandLineOptions.disableBroadcastsOnClose))
    {
        cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
        return -1;    // Failed to open serial port
    }

    int exitCode = 0;

    if (cltool_setupCommunications(inertialSenseInterface))
    {
        if (g_commandLineOptions.asciiMessages.size() == 0 && !cltool_setupLogger(inertialSenseInterface))
        {
            cout << "Failed to setup logger!" << endl;
            inertialSenseInterface.Close();
            inertialSenseInterface.CloseServerConnection();
            return -1;
        }
        try
        {
            uint32_t startTime = current_timeMs();
            while (!g_inertialSenseDisplay.ExitProgram())
            {
                g_inertialSenseDisplay.GetKeyboardInput();

                if (g_inertialSenseDisplay.UploadNeeded())
                {
                    cInertialSenseDisplay::edit_data_t *edata = g_inertialSenseDisplay.EditData();
                    inertialSenseInterface.SendData(edata->did, edata->data, edata->info.dataSize, edata->info.dataOffset);
                }

                if (!inertialSenseInterface.Update())
                {
                    exitCode = -2;
                    break;
                }

                bool refreshDisplay = g_inertialSenseDisplay.PrintData();

                if (g_commandLineOptions.runDuration && ((current_timeMs() - startTime) > g_commandLineOptions.runDuration))
                {
                    break;
                }
            }
        }
        catch (...)
        {
            cout << "Unknown exception...";
        }
    }

    if ((g_commandLineOptions.updateFirmwareTarget != fwUpdate::TARGET_HOST) && !g_commandLineOptions.updateAppFirmwareFilename.empty())
    {
        for (auto& device : inertialSenseInterface.getDevices())
        {
            if (device.fwUpdate.hasError)
            {
                exitCode = -3;
                break;
            }
        }
    }

    inertialSenseInterface.Close();
    inertialSenseInterface.CloseServerConnection();

    return exitCode;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  size_t count_;
};

ImuPublisher* ImuPublisher::instance_ = nullptr;

int main(int argc, char * argv[])
{ 
  cltool_main(argc, argv);
  
  // rclcpp::init(argc, argv);
  // auto node = std::make_shared<ImuPublisher>();
  // ImuPublisher::instance_ = node.get();
  // rclcpp::spin(node);
  // rclcpp::shutdown();
  // return 0;
}
