// Ros
#include <ros/ros.h>

// Find ROS pkgs
#include <ros/package.h>

// Messages
#include <text_to_speech/Speak.h>
#include "tue_pocketsphinx/Switch.h"
#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"
#include "amigo_msgs/RGBLightCommand.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

// Follower
#include "challenge_follow_me/Follower.h"

// STL
#include <vector>
#include <map>

enum PhaseChallenge {
    GUIDE,
    ORDER
};

namespace speech_state {
enum SpeechState {
    DRIVE,
    LOC_NAME,
    SIDE,
    NUMBER,
    OBJECT,
    CONFIRM_LOC,
    CONFIRM_SIDE,
    CONFIRM_NUMBER,
    CONFIRM_OBJECT,
    DONE
};
}


// Settings
unsigned int N_ORDERS = 3;                                                        // Number of orders robot needs to take
unsigned int MAX_N_CONFIRMS = 2;

// Speech
ros::ServiceClient srv_speech_;                                                   // Communication: Service that makes AMIGO speak
ros::ServiceClient speech_recognition_client_;                                    // Communication: Client for starting / stopping speech recognition
ros::Publisher pub_speech_;                                                       // Communication: Publisher that makes AMIGO speak

// Visualization
ros::Publisher rgb_pub_;                                                          // Communicatino: Set color AMIGO
ros::Publisher location_marker_pub_;                                              // Communication: Marker publisher

// Follower
Follower* follower_;

// Tf
tf::TransformListener* listener_;												  // Tf listenter to obtain tf information to store locations

// Administration: speech
bool speech_recognition_turned_on_ = false;
speech_state::SpeechState speech_state_ = speech_state::DRIVE;
std::string current_loc_name_ = "";
std::string current_side_ = "";
unsigned int n_tries_ = 1;
int current_order_ = 1;
std::string current_delivery_location_ = "";
std::string current_object_ = "";

// Adminstration: other
std::string current_clr_;
std::map<int, std::pair<std::string, std::string> > order_map_; // object, desired location
std::map<std::string, tf::StampedTransform> location_map_;                // location name, location


std::string getSpeechStateName(speech_state::SpeechState ss)
{
    std::string name = "<unknown mode>";
    if (ss == speech_state::DRIVE) name = "DRIVE";
    else if (ss == speech_state::LOC_NAME) name = "LOC_NAME";
    else if (ss == speech_state::SIDE) name = "SIDE";
    else if (ss == speech_state::NUMBER) name = "NUMBER";
    else if (ss == speech_state::OBJECT) name = "OBJECT";
    else if (ss == speech_state::CONFIRM_LOC) name = "CONFIRM_LOC";
    else if (ss == speech_state::CONFIRM_SIDE) name = "CONFIRM_SIDE";
    else if (ss == speech_state::CONFIRM_NUMBER) name = "CONFIRM_NUMBER";
    else if (ss == speech_state::CONFIRM_OBJECT) name = "CONFIRM_OBJECT";
    else if (ss == speech_state::DONE) name = "DONE";

    return name;
}



void setRGBLights(std::string color)
{

    ROS_DEBUG("AMIGO: %s", color.c_str());

    std_msgs::ColorRGBA clr_msg;

    if (color == "red") clr_msg.r = 255;
    else if (color == "green") clr_msg.g = 255;
    else if (color == "blue") clr_msg.g = 255;
    else if (color == "yellow")
    {
        clr_msg.r = 255;
        clr_msg.g = 255;
    }
    else
    {
        ROS_INFO("Requested color \'%s\' for RGB lights unknown", color.c_str());
        return;
    }

    //! Send color command
    amigo_msgs::RGBLightCommand rgb_cmd;
    rgb_cmd.color = clr_msg;
    rgb_cmd.show_color.data = true;

    rgb_pub_.publish(rgb_cmd);

    //! Update global
    current_clr_ = color;

}



bool startSpeechRecognition()
{
    //! Determine path speech file
    std::string knowledge_path = ros::package::getPath("tue_knowledge");
    if (knowledge_path == "")
    {
        return false;
    }
    std::string restaurant_speech_path = knowledge_path + "/speech_recognition/restaurant/";

    //! Determine file name
    std::string file_name = "";
    if (speech_state_ == speech_state::DRIVE) file_name = "amigostop";
    else if (speech_state_ == speech_state::LOC_NAME) file_name = "location";
    else if (speech_state_ == speech_state::SIDE) file_name = "side";
    else if (speech_state_ == speech_state::NUMBER) file_name = "number";
    else if (speech_state_ == speech_state::OBJECT) file_name = "object";
    else if (speech_state_ == speech_state::DONE)
    {
        ROS_WARN("No need to start recognition when speech state is DONE");
        return false;
    }
    else file_name = "yesno";

    //! Select appropriate speech file
    tue_pocketsphinx::Switch::Request req;
    req.action = tue_pocketsphinx::Switch::Request::START;
    req.hidden_markov_model = "/usr/share/pocketsphinx/model/hmm/wsj1";
    req.dictionary = restaurant_speech_path + file_name + ".dic";
    req.language_model = restaurant_speech_path + file_name + ".lm";

    //! Toggle speech
    tue_pocketsphinx::Switch::Response resp;
    if (speech_recognition_client_.call(req, resp))
    {
        if (resp.error_msg == "")
        {
            ROS_INFO("Switched on speech recognition for %s", file_name.c_str());
            speech_recognition_turned_on_ = true;
        }
        else
        {
            ROS_WARN("Unable to turn on speech recognition: %s", resp.error_msg.c_str());
            return false;
        }
    }
    else
    {
        ROS_WARN("Service call for turning on speech recognition failed");
        return false;
    }

    //setRGBLights("green");

    return true;
}

bool stopSpeechRecognition()
{

    //! Turn off speech recognition
    tue_pocketsphinx::Switch::Request req;
    req.action = tue_pocketsphinx::Switch::Request::STOP;
    tue_pocketsphinx::Switch::Response resp;
    if (speech_recognition_client_.call(req, resp))
    {
        if (resp.error_msg == "")
        {
            ROS_DEBUG("Switched off speech recognition");
            speech_recognition_turned_on_ = false;
        }
        else
        {
            ROS_WARN("Unable to turn off speech recognition: %s", resp.error_msg.c_str());
            return false;
        }
    }
    else
    {
        ROS_WARN("Unable to turn off speech recognition");
        return false;
    }

    //setRGBLights("red");

    return true;
}



/**
 * @brief amigoSpeak let AMIGO say a sentence
 * @param sentence
 */
void amigoSpeak(std::string sentence, bool block = true)
{

    std::string clr_back_up = current_clr_;
    setRGBLights("red");

    ROS_INFO("AMIGO: \'%s\'", sentence.c_str());

    // If needed toggle recognition
    bool toggle_speech = speech_recognition_turned_on_;
    if (toggle_speech) stopSpeechRecognition();

    //! Call speech service (topic if srv fails)
    text_to_speech::Speak speak;
    speak.request.sentence = sentence;
    speak.request.language = "us";
    speak.request.character = "kyle";
    speak.request.voice = "default";
    speak.request.emotion = "normal";
    speak.request.blocking_call = block;
    if (!srv_speech_.call(speak))
    {
        std_msgs::String sentence_msgs;
        sentence_msgs.data = sentence;
        pub_speech_.publish(sentence_msgs);
    }

    if (toggle_speech) startSpeechRecognition();


    setRGBLights(clr_back_up);

}

void updateSpeechState(speech_state::SpeechState new_state)
{
    ROS_DEBUG("Update speech state from %s to %s", getSpeechStateName(speech_state_).c_str(), getSpeechStateName(new_state).c_str());
    speech_state_ = new_state;
    stopSpeechRecognition();
    startSpeechRecognition();
}



bool storeLocation(std::string location_name)
{
    // Get position
    tf::StampedTransform location;
    try
    {
        listener_->lookupTransform("/map", "/amigo/base_link", ros::Time(0), location);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("No tranform /map - /amigo/base_link");
        amigoSpeak("I cannot store this location: %s", ex.what());
        return false;
    }

    // Store location
    if (location_map_.find(location_name) != location_map_.end())
    {
        ROS_WARN("Overwriting location '%s'!", location_name.c_str());
    }
    location_map_[location_name] = location;

    return true;
}


void createMarkerWithLabel(std::string label, tf::StampedTransform& pose, double r, double g, double b, visualization_msgs::MarkerArray& array)
{

    // Geometric marker
    visualization_msgs::Marker marker;
    marker.ns = "restaurant/location_markers";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.header.frame_id = "/map";
    marker.id = location_map_.size();
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1;
    marker.pose.position.x = pose.getOrigin().x();
    marker.pose.position.y = pose.getOrigin().y();
    marker.pose.orientation.w = pose.getRotation().getW();
    marker.pose.orientation.x = pose.getRotation().getX();
    marker.pose.orientation.y = pose.getRotation().getY();
    marker.pose.orientation.z = pose.getRotation().getZ();
    array.markers.push_back(marker);

    // Text label
    visualization_msgs::Marker marker_txt = marker;
    marker_txt.scale.z = 0.1;
    marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_txt.text = label;
    marker_txt.id = location_map_.size()*10;
    marker_txt.pose.position.z *= 1.5;
    array.markers.push_back(marker_txt);

    ROS_INFO("Added marker with color (%f,%f,%f), size (%f,%f,%f) and label %s",
             marker.color.r, marker.color.g, marker.color.b, marker.scale.x, marker.scale.y, marker.scale.z, marker_txt.text.c_str());

}



bool updateLocation(std::string location_name, std::string side)
{
    //! Verify side
    if (side != "left" && side != "right" && side != "front")
    {
        ROS_ERROR("Side must me left, right or front but is %s", side.c_str());
        return false;
    }

    //! See if location is already in the map
    if (location_map_.find(location_name) == location_map_.end())
    {
        ROS_WARN("Location %s not yet in map, please check administration (adding it now)", location_name.c_str());
        if (!storeLocation(location_name)) return false;
    }

    // Get old location
    tf::StampedTransform new_location = location_map_[location_name];

    // If needed add angle
    double theta = 0;
    if (side == "left") theta = 1.57;
    else if (side == "right") theta = -1.57;
    tf::Quaternion q = new_location.getRotation();
    tf::Quaternion offset;
    offset.setRPY(0, 0, theta);
    q += offset;
    new_location.setRotation(q);

    // If front, add 1 m offset
    if (side == "front")
    {
        // Get position
        geometry_msgs::PoseStamped loc_base_link, loc_map;
        try
        {
            loc_base_link.header.frame_id = "/amigo/base_link";
            loc_base_link.pose.position.x = 1.0;
            loc_base_link.pose.orientation.w = 1.0;
            listener_->transformPose("/map", loc_base_link, loc_map);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Cannot tranform position from /map to /amigo/base_link: %s", ex.what());
            amigoSpeak("I cannot store this location: %s", ex.what());
            return false;
        }
        // In appropriate format
        new_location.setOrigin(tf::Vector3(loc_map.pose.position.x, loc_map.pose.position.y, loc_map.pose.position.z));
        new_location.setRotation(tf::Quaternion(loc_map.pose.orientation.x, loc_map.pose.orientation.y, loc_map.pose.orientation.z, loc_map.pose.orientation.w));
    }

    // Store location
    location_map_[location_name] = new_location;

    // Publish marker
    visualization_msgs::MarkerArray marker_array;
    createMarkerWithLabel(location_name, new_location, 0, 0, 1, marker_array);
    location_marker_pub_.publish(marker_array);

    return true;
}



void speechCallbackGuide(std_msgs::String res)
{

    std::string answer = res.data;
    if (answer.empty()) return;

    //! Get first word
    ROS_DEBUG("Received unfiltered speech input: %s", answer.c_str());
    size_t position = answer.find_first_of(" ");
    if (position > 0 && position <= answer.size()) answer = std::string(answer.c_str(), position);
    ROS_INFO("I heard: %s", answer.c_str());

    // ROBOT IS ASKED TO STOP
    if (speech_state_ == speech_state::DRIVE)
    {
        if (answer == "amigostop")
        {
            // stop robot
            follower_->pause();
            ROS_DEBUG("Robot received a stop command");
            updateSpeechState(speech_state::LOC_NAME);
            setRGBLights("yellow");
        }
        else ROS_WARN("Robot expects command 'amigostop' but received an unknown command '%s'!", answer.c_str());
    }

    // RECEIVED A LOCATION NAME
    else if (speech_state_ == speech_state::LOC_NAME)
    {
        if (answer == "continue")
        {
            follower_->resume();
            updateSpeechState(speech_state::DRIVE);
            setRGBLights("green");
        }
        else
        {
            // Received location name
            if (answer == "one") current_loc_name_ = "delivery location one";
            else if (answer == "two") current_loc_name_ = "delivery location two";
            else if (answer == "three") current_loc_name_ = "delivery location three";
            else if (answer == "food") current_loc_name_ = "food shelf";
            else if (answer == "drink") current_loc_name_ = "drink shelf";
            else if (answer == "order") current_loc_name_ = "ordering location";
            else
            {
                // Unknown command!
                ROS_WARN("Robot expects: 1, 2, 3, food or drink (unknown command '%s')", answer.c_str());
                updateSpeechState(speech_state::DRIVE);
                setRGBLights("green");
            }

            // Ask for confirmation
            if (speech_state_ == speech_state::LOC_NAME)
            {
                updateSpeechState(speech_state::CONFIRM_LOC);
                amigoSpeak(current_loc_name_ + "?");
                setRGBLights("green");
            }
        }
    }

    // CONFIRMATION LOCATION NAME
    else if (speech_state_ == speech_state::CONFIRM_LOC)
    {
        if (answer == "yes")
        {
            if (current_loc_name_.empty()) ROS_WARN("Error in location name administration!");
            ROS_INFO("Confirmed location name '%s'!", current_loc_name_.c_str());

            // Store the location
            if (!storeLocation(current_loc_name_)) ROS_WARN("Cannot store location named %s!", current_loc_name_.c_str());

            if (current_loc_name_ == "ordering location")
            {
                // Publish marker
                if (location_map_.find("ordering location") != location_map_.end())
                {
                    visualization_msgs::MarkerArray marker_array;
                    createMarkerWithLabel("Ord. loc.", location_map_["ordering location"], 0, 1, 0, marker_array);
                    location_marker_pub_.publish(marker_array);
                }
                else ROS_WARN("Storing the ordering location failed!");

                // SWITCH TO NEXT MODE
                speech_state_ = speech_state::NUMBER;
            }
            else
            {
                updateSpeechState(speech_state::SIDE);
                amigoSpeak("Which side?");
            }
            setRGBLights("green");
            n_tries_ = 0;
        }
        else if (n_tries_ < MAX_N_CONFIRMS)
        {
            ROS_INFO("Misunderstood the location name");
            updateSpeechState(speech_state::LOC_NAME);
            amigoSpeak("Which location?");
            setRGBLights("green");
            ++n_tries_;
        }
        else
        {
            updateSpeechState(speech_state::DRIVE);
            amigoSpeak("I give up");
            setRGBLights("green");
            n_tries_ = 0;
        }
    }

    // RECEIVED A SIDE
    else if (speech_state_ == speech_state::SIDE)
    {
        if (answer == "left" || answer == "right" || answer == "front")
        {
            current_side_ = answer;
            updateSpeechState(speech_state::CONFIRM_SIDE);
            amigoSpeak(current_side_ + "?");
            setRGBLights("green");
        }
        else
        {
            ROS_WARN("Robot expects: left, front or right (unknown command '%s')", answer.c_str());
            updateSpeechState(speech_state::SIDE);
            setRGBLights("green");
        }
    }

    // CONFIRMATION SIDE
    else if (speech_state_ == speech_state::CONFIRM_SIDE)
    {
        if (answer == "yes")
        {
            if (current_side_.empty()) ROS_WARN("Error in location name administration!");
            ROS_INFO("Confirmed side '%s' for '%s'!", current_side_.c_str(), current_loc_name_.c_str());

            updateLocation(current_loc_name_, current_side_);

            //! Continue following
            setRGBLights("yellow");
            follower_->reset();
            setRGBLights("green");
            updateSpeechState(speech_state::DRIVE);
            n_tries_ = 0;
        }
        else if (n_tries_ < MAX_N_CONFIRMS)
        {
            ROS_DEBUG("Misunderstood the side");
            updateSpeechState(speech_state::SIDE);
            amigoSpeak("Which side?");
            setRGBLights("green");
            ++n_tries_;
        }
        else
        {
            updateSpeechState(speech_state::DRIVE);
            amigoSpeak("I give up");
            setRGBLights("green");
            n_tries_ = 0;

            //! Continue following
            setRGBLights("yellow");
            follower_->reset();
            setRGBLights("green");
        }
    }


}



void speechCallbackOrder(std_msgs::String res)
{

    std::string answer = res.data;
    if (answer.empty()) return;

    //! Get first word
    ROS_INFO("Full answer: %s", answer.c_str());
    if (speech_state_ != speech_state::OBJECT)
    {
        size_t position = answer.find_first_of(" ");
        if (position > 0 && position <= answer.size()) answer = std::string(answer.c_str(), position);
    }
    ROS_INFO("I heard: %s", answer.c_str());

    // ROBOT RECEIVED LOCATION
    if (speech_state_ == speech_state::NUMBER)
    {
        if (answer == "one" || answer == "two" || answer == "three")
        {
            if (answer == "one") current_delivery_location_ = "delivery location one";
            else if (answer == "two") current_delivery_location_ = "delivery location two";
            else if (answer == "three") current_delivery_location_ = "delivery location three";
            updateSpeechState(speech_state::CONFIRM_NUMBER);
            amigoSpeak(current_delivery_location_ + "?");
            setRGBLights("green");
        }
        else ROS_WARN("Robot expects 'one', 'two' or 'three' but received an unknown command '%s'!", answer.c_str());
    }

    // CONFIRMATION DELIVERY LOCATION
    else if (speech_state_ == speech_state::CONFIRM_NUMBER)
    {
        if (answer == "yes")
        {
            updateSpeechState(speech_state::OBJECT);
            amigoSpeak("Which object?");
            n_tries_ = 0;
        }
        else if (n_tries_ < MAX_N_CONFIRMS)
        {
            updateSpeechState(speech_state::NUMBER);
            amigoSpeak("Which location?");
            ++n_tries_;
        }
        else
        {
            updateSpeechState(speech_state::NUMBER);
            amigoSpeak("Let's try again, which delivery location?");
            n_tries_ = 0;
        }
        setRGBLights("green");

    }

    // RECEIVED AN OBJECT
    else if (speech_state_ == speech_state::OBJECT)
    {
        current_object_ = answer;
        updateSpeechState(speech_state::CONFIRM_OBJECT);
        amigoSpeak(current_object_ + "?");
        setRGBLights("green");

    }

    // CONFIRMATION OBJECT
    else if (speech_state_ == speech_state::CONFIRM_OBJECT)
    {
        if (answer == "yes")
        {
            if (current_object_.empty()) ROS_WARN("Error in location name administration!");

            //! Store one or two objects (at most two objects according to rulebook)
            std::string object_1 = current_object_;
            std::string object_2 = "";
            size_t position = current_object_.find_last_of(" ");
            if (position > 0 && position <= current_object_.size())
            {
                object_1 = current_object_.substr(position+1); ROS_INFO("Object 1: %s", object_1.c_str());
                object_2 = current_object_.substr(0,position); ROS_INFO("Object 1: %s", object_2.c_str());
            }

            //! Store first object
            ROS_INFO("Confirmed object '%s' for '%s'!", object_1.c_str(), current_delivery_location_.c_str());
            order_map_[current_order_] = std::make_pair<std::string, std::string>(current_delivery_location_, object_1);
            ROS_INFO("Stored order: %zu/%u", order_map_.size(), N_ORDERS);
            ++current_order_;

            //! Optional second object
            if (!object_2.empty())
            {
                ROS_INFO("Confirmed object '%s' for '%s'!", object_2.c_str(), current_delivery_location_.c_str());
                order_map_[current_order_] = std::make_pair<std::string, std::string>(current_delivery_location_, object_2);
                ROS_INFO("Stored order: %zu/%u", order_map_.size(), N_ORDERS);
                ++current_order_;
            }


            // See if all orders are taken
            if (order_map_.size() < N_ORDERS)
            {
                updateSpeechState(speech_state::NUMBER);
                amigoSpeak("What's the next delivery location?");
            }
            else
            {
                updateSpeechState(speech_state::DONE);
                amigoSpeak("I know all orders.");
            }

            n_tries_ = 0;
        }
        else if (n_tries_ < MAX_N_CONFIRMS)
        {
            ROS_DEBUG("Misunderstood the object");
            updateSpeechState(speech_state::OBJECT);
            amigoSpeak("Which object?");
            setRGBLights("green");
            ++n_tries_;
        }
        else
        {
            updateSpeechState(speech_state::OBJECT);
            amigoSpeak("I don't understand. Which object?");
            setRGBLights("green");
            n_tries_ = 0;
        }
    }


}



int main(int argc, char **argv) {

    ros::init(argc, argv, "test_speech_restaurant");
    ros::NodeHandle nh;

    //! RGB lights
    rgb_pub_ = nh.advertise<amigo_msgs::RGBLightCommand>("/user_set_rgb_lights", 1);
    setRGBLights("blue");

    //! Transforms
    listener_ = new tf::TransformListener();

    //! Location markers
    location_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/restaurant/location_markers", 10);

    //! Topic/srv that make AMIGO speak
    pub_speech_ = nh.advertise<std_msgs::String>("/text_to_speech/input", 10);
    srv_speech_ =  nh.serviceClient<text_to_speech::Speak>("/text_to_speech/speak");
    ROS_INFO("Publisher/service client for text to speech started");

    //! Start speech recognition
    speech_recognition_client_ = nh.serviceClient<tue_pocketsphinx::Switch>("/pocketsphinx/switch");
    speech_recognition_client_.waitForExistence();
    ros::Subscriber sub_speech = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 1, speechCallbackGuide);

    //! Start follower
    follower_ = new Follower(nh, "/amigo/base_link", false);
    if (!follower_->start())
    {
        ROS_ERROR("Could not start the follower!");
        return -1;
    }

    //! Wait for operator
    bool operator_found = false;
    ros::Rate loop_rate(25);
    while (!operator_found)
    {
        operator_found = follower_->update();
        loop_rate.sleep();

    }

    stopSpeechRecognition();
    startSpeechRecognition();
    setRGBLights("green");
    ROS_INFO("Started speech recognition");

    //! Start guiding phase
    while (ros::ok() && speech_state_ != speech_state::NUMBER)
    {
        //! Get information from topics
        ros::spinOnce();

        //! Update the follower
        bool ok = follower_->update();
        if (!ok) ROS_WARN("Could not update Follower");

        //! Wait
        loop_rate.sleep();
    }

    //! Reconfigure speech recognition
    sub_speech.shutdown();
    sub_speech = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 10, speechCallbackOrder);
    amigoSpeak("Read for orders. Which location?");
    speech_state_ = speech_state::NUMBER;
    stopSpeechRecognition();
    startSpeechRecognition();

    //! Done with the following part
    follower_->stop();
    delete follower_;

    //! Take orders
    while (ros::ok() && speech_state_ != speech_state::DONE)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    //! Done
    ROS_INFO("AMIGO took these orders:");
    for (std::map<int, std::pair<std::string, std::string> >::const_iterator it = order_map_.begin(); it != order_map_.end(); ++it)
    {
        ROS_INFO("\tBring %s to %s", it->second.second.c_str(), it->second.first.c_str());
    }


    return 0;
}
