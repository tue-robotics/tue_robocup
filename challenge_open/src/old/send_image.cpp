#include <ros/ros.h>
#include <tue_serialization/Binary.h>
#include <fstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "send_image");

    std::string filename = argv[1];

    // open the file:
    std::streampos fileSize;
    std::ifstream file(filename.c_str(), std::ios::binary);

    // get its size:
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // read the data:
    tue_serialization::Binary msg;
    msg.data.resize(fileSize);
    file.read((char*) &msg.data[0], fileSize);

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<tue_serialization::Binary>("/amigo_mobile_gui/image", 1);

    ros::Rate r(1);
    while (ros::ok())
    {
        pub.publish(msg);

        r.sleep();
    }

    return 0;
}
