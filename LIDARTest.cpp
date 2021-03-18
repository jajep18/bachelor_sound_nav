#include <rplidar.h>

int main(int argc, char* argv)
{
    RPlidarDriver* lidar = RPlidarDriver::CreateDriver();
    u_result res = lidar->connect("/dev/ttyUSB0", 115200);

    if (IS_OK(res))
    {
        while (IS_OK(res)) {
            RplidarScanMode scanMode;
            lidar->startScan(false, true, 0, &scanMode);
            lidar->startMotor();

            rplidar_response_measurement_node_hq_t nodes[8192];
            size_t nodeCount = sizeof(nodes) / sizeof(rplidar_response_measurement_node_hq_t);
            res = lidar->grabScanDataHq(nodes, nodeCount);


            if (IS_FAIL(res))
            {
                std::cout << "failed to get scan data \n";
                break;
            }
        }
    }
    else{
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
    }
    lidar->stopMotor();
    lidar->disconnect();
    RPlidarDriver::DisposeDriver(lidar);

}