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


bool GroupSyncWrite::Communicate()
{
    int dxl_comm_result = txPacket();

    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_DEBUG("[%s] SyncWrite communication failed: %s",
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

bool PVWriter::beforeExecute(std::vector<Joint>& joints)
{
    // buffer for Dynamixel values
    uint8_t command[12];

    bool dxl_addparam_result = false;                 // addParam result
    bool param_added = false;                         // at least one param added

    clearParam();
    
    for (auto & joint : joints)
    {
        if (joint.present())
        {
            int32_t p = joint.getRawPositionFromCommand();
            uint32_t vp = joint.getVelocityProfileFromCommand();
            uint32_t ap = vp / 4;
            // platform-independent handling of byte order
            // acceleration; register 108
            command[0] = DXL_LOBYTE(DXL_LOWORD(ap));
            command[1] = DXL_HIBYTE(DXL_LOWORD(ap));
            command[2] = DXL_LOBYTE(DXL_HIWORD(ap));
            command[3] = DXL_HIBYTE(DXL_HIWORD(ap));
            // velocity profile ; register 112
            command[4] = DXL_LOBYTE(DXL_LOWORD(vp));
            command[5] = DXL_HIBYTE(DXL_LOWORD(vp));
            command[6] = DXL_LOBYTE(DXL_HIWORD(vp));
            command[7] = DXL_HIBYTE(DXL_HIWORD(vp));
            // position; register 116
            command[8] = DXL_LOBYTE(DXL_LOWORD(p));
            command[9] = DXL_HIBYTE(DXL_LOWORD(p));
            command[10] = DXL_LOBYTE(DXL_HIWORD(p));
            command[11] = DXL_HIBYTE(DXL_HIWORD(p));
            // addParam
            dxl_addparam_result = addParam(joint.id(), command);
            if (dxl_addparam_result != true) {
                ROS_ERROR("Failed to add servo ID %d to loop %s", joint.id(), getName().c_str());
                continue;
            }
            else
                param_added = true;
        }
    }
    return param_added;
}