/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *          MASSIVE TODO:
 *          * CATCH ALL EXEPTIONS !!!
 *          * ADD A SERVICE PROCESSING QUEUEINSTEAD OF WAITING
 *          * MOST OF THE CODE IS COMPLETELY INITIAL AND
 *            HAS ONLY SINGLE FUNCTIONALITY, I.E. LAUNCHFILES
 *            ARE NOT SUPPORTED, ETC.
 *          * CHANGE THE "start_sensor_cb" to "start_sensor_cb"
 *          * KEEP TRACK ON WHAT SENSORS ARE CURRENTLY RUNNING
 *            (sensors, registered subscribers. if subs = 0 then
 *             shut the sensor down)
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "package_info/package_info.h"

#include "core/common.h"
#include "temoto_2/nodeSpawnKill.h"
#include "temoto_2/listDevices.h"
#include "temoto_2/startSensorRequest.h"
#include "temoto_2/stopSensorRequest.h"
#include "std_msgs/String.h"
#include <sstream>

class SensorManager
{
public:
    /**
     * @brief List of known sensors
     */
    std::vector <package_info> pkgInfoList_;

    /**
     * @brief SensorManager
     */
    SensorManager()
    {
        // Start the client
        nodeSpawnKillClient_ = n_.serviceClient<temoto_2::nodeSpawnKill>("spawn_kill_process");

        // Start the servers
        startSensorServer_ = n_.advertiseService("start_sensor", &SensorManager::start_sensor_cb, this);
        stopSensorServer_ = n_.advertiseService("stop_sensor", &SensorManager::stop_sensor_cb, this);

        ROS_INFO("[SensorManager::SensorManager] all services initialized, manager is good to go");
    }

private:

    ros::NodeHandle n_;
    ros::ServiceClient nodeSpawnKillClient_;
    ros::ServiceServer startSensorServer_;
    ros::ServiceServer stopSensorServer_;

    /**
     * @brief Callback to a service that lists all available packages that
     * are with a requested "type". For example "list all HANDtracking devices"
     * @param req
     * @param res
     * @return
     */
    bool list_devices_cb (temoto_2::listDevices::Request &req,
                          temoto_2::listDevices::Response &res)
    {
        //std::vector <std::string> devList;

        // Find the devices with the required type
        for (auto& entry : pkgInfoList_)
        {
            if (entry.getType().compare(req.type) == 0)
            {
                res.list.push_back(entry.getName());
            }
        }

        return true;
    }

    /* Callback to a service that executes/runs a requested device
     * and sends back the topic that the device is publishing to
     * THIS IS LIKELY A GENERIC FUNCTION THAT WILL BE USED ALSO BY
     * OTHER MANAGERS
     */

    /**
     * @brief Start node service
     * @param req
     * @param res
     * @return
     */
    bool start_sensor_cb (temoto_2::startSensorRequest::Request &req,
                          temoto_2::startSensorRequest::Response &res)
    {
        std::string reqType = req.type;
        std::string reqName = req.name;
        std::string reqExecutable = req.executable;

        ROS_INFO("[start_sensor_cb] received a request to start a '%s': '%s'", reqType.c_str(), reqName.c_str());

        // Create an empty message that will be filled out by "findSensor" function
        temoto_2::nodeSpawnKill spawnKillMsg;

        // Find the suitable sensor
        if (findSensor(spawnKillMsg.request, res, reqType, reqName, reqExecutable))
        {
            ROS_INFO("[start_sensor_cb] Found a suitable sensor. Trying to call /spawn_kill_process ...");
            while (!nodeSpawnKillClient_.call(spawnKillMsg))
            {
                ROS_ERROR("[start_sensor_cb] Failed to call service /spawn_kill_process, trying again...");
            }

            res.code = spawnKillMsg.response.code;
            res.message = spawnKillMsg.response.message;

            ROS_INFO("[start_sensor_cb] /spawn_kill_process responded: '%s'", res.message.c_str());

            // Check if the /spawn_kill_process service was able to fulfill the request
            return ( (res.code == 0) ? true : false);
        }

        else
        {
            res.name = req.name;
            res.executable = "";
            res.topic = "";
            res.code = 1;
            res.message = "Suitable sensor was not found. Aborting the request";
            ROS_INFO("[start_sensor_cb] %s", res.message.c_str());

            return false;
        }
    }

    /**
     * @brief Stop node service
     * @return
     */
    bool stop_sensor_cb (temoto_2::stopSensorRequest::Request &req,
                       temoto_2::stopSensorRequest::Response &res)
    {
        // TODO: Check if the request makes sense, check the name and type
        temoto_2::nodeSpawnKill spawnKillMsg;
        spawnKillMsg.request.action = "kill";
        spawnKillMsg.request.package = req.name;
        spawnKillMsg.request.name = req.executable;

        ROS_INFO("[stop_sensor_cb] received a request to stop a '%s'", spawnKillMsg.request.name.c_str());

        while (!nodeSpawnKillClient_.call(spawnKillMsg))
        {
            ROS_ERROR("[stop_sensor_cb] Failed to call service /spawn_kill_process, trying again...");
        }

        res.code = spawnKillMsg.response.code;
        res.message = spawnKillMsg.response.message;

        ROS_INFO("[stop_sensor_cb] /spawn_kill_process responded: '%s'", res.message.c_str());

        // Check if the /spawn_kill_process service was able to fulfill the request
        return ( (res.code == 0) ? true : false);
    }


    //      THIS FUNCTION BELOW IS ULTRA CONFUSING AND SHOULD BE
    //      BROKEN INTO MULTIPLE, MORE REASONABLE, SEGMENTS.
    //      PAY ATTENTION AT THE NAMES: "runnable", "launchable",
    //      "executable". THIS HAS TO BE FIXED. these point to different things
    /**
     * @brief Function for finding the right sensor, based on the request parameters
     * type - requested, name - optional, node - optional
     * @param ret
     * @param retstartSensor
     * @param type
     * @param name
     * @param node
     * @return Returns a boolean. If suitable device was found, then the req param
     * is formatted as a nodeSpawnKill::Request (first one in the list, even if there is more)
     */
    bool findSensor (temoto_2::nodeSpawnKill::Request &ret,
                     temoto_2::startSensorRequest::Response &retstartSensor,
                     std::string type,
                     std::string name = "",
                     std::string executable = "")
    {
        // Local list of devices that follow the requirements
        std::vector <package_info> candidates;
        std::vector <package_info> candidatesLocal;

        // Find the devices that follow the "type" criteria
        for (auto& entry : pkgInfoList_)
        {
            if (entry.getType().compare(type) == 0)
            {
                candidates.push_back(entry);
            }
        }

        // If the list is empty, leave the req empty
        if (candidates.empty())
        {
            return false;
        }

        // Check if a name was specified
        if (name.compare("") != 0)
        {
            // Filter out the devices that follow the "name" criteria
            for (auto& entry : candidates)
            {
                if (entry.getName().compare(name) == 0)
                {
                    candidatesLocal.push_back(entry);
                }
            }

            // Copy the contents of "candidatesLocal" into "candidates"
            candidates = candidatesLocal;

            // If the list is empty, return an empty response
            if (candidates.empty())
            {
                return false;
            }
        }

        else
        {
            // Return the first "runnable" of the element in the "candidates" list
            ret.action = "roslaunch";
            ret.package = candidates[0].getName();
            ret.name = candidates[0].getRunnables().begin()->first;

            // The name of the topic that this particular runnable publishes to
            retstartSensor.name = ret.package;
            retstartSensor.executable = ret.name;
            retstartSensor.topic = candidates[0].getRunnables().begin()->second;

            return true;
        }

        // Check if the runnable/launchable/executable was specified
        if (executable.compare("") != 0)
        {
            // Clear out the "candidatesLocal" list
            candidatesLocal.clear();

            // Filter out the devices that follow the "executable" criteria
            for (auto& entry : candidates)
            {
                // Get the local runnables
                std::map<std::string, std::string> runnablesEntry = entry.getRunnables();

                // Check if the required runnable exists in the list of runnables
                if (runnablesEntry.find(executable) != runnablesEntry.end())
                {
                    candidatesLocal.push_back(entry);
                }
            }

            // Copy the contents of "candidatesLocal" into "candidates"
            candidates = candidatesLocal;

            // If the list is empty, return an empty response
            if (candidates.empty())
            {
                return false;
            }
        }

        else
        {
            // Return the first "runnable" of the element in the "candidates" list
            ret.action = "roslaunch";
            ret.package = name;
            ret.name = candidates[0].getRunnables().begin()->first;

            // The name of the topic that this particular runnable publishes to
            retstartSensor.name = name;
            retstartSensor.executable = ret.name;
            retstartSensor.topic = candidates[0].getRunnables().begin()->second;

            return true;
        }

        // If all above constraints were satisfied, then:
        ret.action = "roslaunch";
        ret.package = name;
        ret.name = executable;

        // The name of the topic that this particular runnable publishes to
        retstartSensor.name = name;
        retstartSensor.executable = executable;
        retstartSensor.topic = candidates[0].getRunnables()[executable];

        return true;
    }
};


int main (int argc, char **argv)
{

    ros::init (argc, argv, "sensor_manager");

    // Create a SensorManager object
    SensorManager sensorManager;

    // Add a dummy sensro entry (For testing)
    sensorManager.pkgInfoList_.push_back (package_info("temoto_2", "hand"));
    sensorManager.pkgInfoList_[0].addRunnable({"dummy_sensor", "/dummy_sensor_data"});

    sensorManager.pkgInfoList_.push_back (package_info("temoto_2", "text"));
    sensorManager.pkgInfoList_[1].addRunnable({"test_2.launch", "/dummy_sensor_data"});

    ros::spin();

    return 0;
}
