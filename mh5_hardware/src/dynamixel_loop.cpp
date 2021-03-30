#include "mh5_hardware/dynamixel_loop.hpp"


using namespace mh5_hardware;


bool GroupSyncRead::prepare(std::vector<Joint>& joints)
{
    bool params_added = false;
    for (auto & joint : joints) {
        if (joint.present()) {
            if(!addParam(joint.id()))
                ROS_WARN("Failed to add servo ID %d to loop %s", joint.id(), getName().c_str());
            else
                params_added = true;
        }
    }

    if (!params_added)
        ROS_WARN("No servos active for loop %s", getName().c_str());

    return params_added;
}


bool GroupSyncRead::Communicate()
{
    int dxl_comm_result = txRxPacket();

    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_DEBUG("[%s] SyncRead communication failed: %s",
                 getName().c_str(),
                 getPacketHandler()->getTxRxResult(dxl_comm_result));
        return false;
    }
    return true;
}


bool PVLReader::afterExecute(std::vector<Joint>& joints)
{
    uint8_t dxl_error = 0;                            // Dynamixel error
    bool dxl_getdata_result = false;                  // GetParam result
    
    // process each servo
    for(auto & joint : joints)
    {
        const char* name = getName().c_str();   // for messsages
        uint8_t id = joint.id();                // to avoid callling it all the time...

        if (!joint.present())                   //only present servos
            continue;
        
        // check no errors
        if (getError(id, &dxl_error)) {
            ROS_DEBUG("[%s] SyncRead error getting ID %d: %s",
                      name, id, getPacketHandler()->getRxPacketError(dxl_error));
            continue;
        }
        //position
        dxl_getdata_result = isAvailable(id, 132, 4);
        if (!dxl_getdata_result)
            ROS_DEBUG("[%s] SyncRead getting position for ID %d failed", name, id);
        else {
            int32_t position = getData(id, 132, 4);
            joint.setPositionFromRaw(position);
        }
        //velocity
        dxl_getdata_result = isAvailable(id, 128, 4);
        if (!dxl_getdata_result)
            ROS_DEBUG("[%s] SyncRead getting velocity for ID %d failed", name, id);
        else {
            int32_t velocity = getData(id, 128, 4);
            joint.setVelocityFromRaw(velocity);
        }
        //load
        dxl_getdata_result = isAvailable(id, 126, 2);
        if (!dxl_getdata_result)
            ROS_DEBUG("[%s] SyncRead getting load for ID %d failed", name, id);
        else {
            int16_t load = getData(id, 126, 2);
            joint.setEffortFromRaw(load);
        }
    }

    // even if there are errors
    return true;
}