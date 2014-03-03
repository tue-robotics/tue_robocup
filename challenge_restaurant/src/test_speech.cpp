#include <ros/ros.h>
#include <ros/package.h>
#include <text_to_speech/Speak.h>
#include "tue_pocketsphinx/Switch.h"

#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"
#include "amigo_msgs/RGBLightCommand.h"


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

ros::Publisher rgb_pub_;

std::map<std::string, std::string> order_map_; // object, desired location

// Administration
bool speech_recognition_turned_on_ = false;
speech_state::SpeechState speech_state_ = speech_state::DRIVE;
std::string current_loc_name_ = "";
std::string current_side_ = "";
unsigned int n_tries_ = 1;

std::string current_clr_;


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
            // continue driving
            // set original color
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
                amigoSpeak(current_loc_name_ + "?");
                updateSpeechState(speech_state::CONFIRM_LOC);
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
            if (current_loc_name_ == "ordering location")
            {
                // SWITCH TO NEXT MODE
                amigoSpeak("I am ready to take orders. Which delicery location?");
                speech_state_ = speech_state::NUMBER;
            }
            else
            {
                amigoSpeak("Which side?");
                updateSpeechState(speech_state::SIDE);
            }
            setRGBLights("green");
            n_tries_ = 0;
        }
        else if (n_tries_ < MAX_N_CONFIRMS)
        {
            ROS_INFO("Misunderstood the location name");
            amigoSpeak("Which location?");
            updateSpeechState(speech_state::LOC_NAME);
            setRGBLights("green");
            ++n_tries_;
        }
        else
        {
            amigoSpeak("I give up");
            updateSpeechState(speech_state::DRIVE);
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
            amigoSpeak(current_side_ + "?");
            updateSpeechState(speech_state::CONFIRM_SIDE);
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
            // store location + side
            // reset world model
            // start following again
            updateSpeechState(speech_state::DRIVE);
            n_tries_ = 0;
        }
        else if (n_tries_ < MAX_N_CONFIRMS)
        {
            ROS_DEBUG("Misunderstood the side");
            amigoSpeak("Which side?");
            updateSpeechState(speech_state::SIDE);
            setRGBLights("green");
            ++n_tries_;
        }
        else
        {
            amigoSpeak("I give up");
            updateSpeechState(speech_state::DRIVE);
            setRGBLights("green");
            n_tries_ = 0;
        }
    }


}


std::string current_delivery_location_ = "";
std::string current_object_ = "";

void speechCallbackOrder(std_msgs::String res)
{

    std::string answer = res.data;
    if (answer.empty()) return;

    //! Get first word
    ROS_INFO("Full answer: %s", answer.c_str());
    size_t position = answer.find_first_of(" ");
    if (position > 0 && position <= answer.size()) answer = std::string(answer.c_str(), position);
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
            amigoSpeak("Which location?");
            ++n_tries_;
        }
        else
        {
            amigoSpeak("Let's try again, which delivery location?");
            updateSpeechState(speech_state::NUMBER);
            n_tries_ = 0;
        }
        setRGBLights("green");

    }

    // RECEIVED AN OBJECT
    else if (speech_state_ == speech_state::OBJECT)
    {
        current_object_ = answer;
        amigoSpeak(current_object_ + "?");
        updateSpeechState(speech_state::CONFIRM_OBJECT);
        setRGBLights("green");

    }

    // CONFIRMATION OBJECT
    else if (speech_state_ == speech_state::CONFIRM_OBJECT)
    {
        if (answer == "yes")
        {
            if (current_object_.empty()) ROS_WARN("Error in location name administration!");
            ROS_INFO("Confirmed object '%s' for '%s'!", current_object_.c_str(), current_delivery_location_.c_str());

            // Store order
            order_map_[current_delivery_location_] = current_object_;
            if (order_map_.size() < N_ORDERS)
            {
                updateSpeechState(speech_state::NUMBER);
                amigoSpeak("What's the next delivery location?");
            }
            else
            {
                amigoSpeak("I know all orders.");
                updateSpeechState(speech_state::DONE);
            }

            n_tries_ = 0;
        }
        else if (n_tries_ < MAX_N_CONFIRMS)
        {
            ROS_DEBUG("Misunderstood the object");
            amigoSpeak("Which object?");
            updateSpeechState(speech_state::OBJECT);
            setRGBLights("green");
            ++n_tries_;
        }
        else
        {
            amigoSpeak("I don't understand. Which object?");
            updateSpeechState(speech_state::OBJECT);
            setRGBLights("green");
            n_tries_ = 0;
        }
    }


}



int main(int argc, char **argv) {

    ros::init(argc, argv, "test_speech_restaurant");
    ros::NodeHandle nh;

    rgb_pub_ = nh.advertise<amigo_msgs::RGBLightCommand>("/user_set_rgb_lights", 1);
    setRGBLights("blue");

    //! Topic/srv that make AMIGO speak
    pub_speech_ = nh.advertise<std_msgs::String>("/text_to_speech/input", 10);
    srv_speech_ =  nh.serviceClient<text_to_speech::Speak>("/text_to_speech/speak");
    ROS_INFO("Publisher/service client for text to speech started");

    //! Start speech recognition
    speech_recognition_client_ = nh.serviceClient<tue_pocketsphinx::Switch>("/pocketsphinx/switch");
    speech_recognition_client_.waitForExistence();
    ros::Subscriber sub_speech = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 1, speechCallbackGuide);
    stopSpeechRecognition();
    startSpeechRecognition();
    ROS_INFO("Started speech recognition");


    sub_speech.shutdown();
    sub_speech = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 10, speechCallbackOrder);
    amigoSpeak("Which location?");
    speech_state_ = speech_state::NUMBER;

    ros::Rate loop_rate(5);
    while (ros::ok() && speech_state_ != speech_state::DONE)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Done taking orders:");

    for (std::map<std::string, std::string>::const_iterator it = order_map_.begin(); it != order_map_.end(); ++it)
    {
        ROS_INFO("\tBring %s to %s", it->second.c_str(), it->first.c_str());
    }


    return 0;
}

