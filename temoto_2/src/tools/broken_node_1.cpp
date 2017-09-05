#include "ros/ros.h"
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "language_input");
    ros::NodeHandle nh;

    int n = 3;
    std::vector<int> test_vec(n);

    try
    {
        // Access the vector by index and deliberately
        // try to access values out of the size range
        for( int i=0; i<n+6; i++ )
        {
            std::cout << "test_vec[" << i << "] = " << test_vec.at(i) << std::endl;
        }
    }
    catch(...)
    {
        std::cout << "Exception: Oh boy, you're in trouble ...\n";
        return 1;
    }

    while (ros::ok())
    {
        ros::Duration(0.5).sleep();
    }

    return 0;
}
