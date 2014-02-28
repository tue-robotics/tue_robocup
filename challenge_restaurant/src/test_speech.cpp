#include <ros/ros.h>
#include <ros/package.h>
#include <text_to_speech/Speak.h>
#include "tue_pocketsphinx/Switch.h"
#include "std_msgs/String.h"

using namespace std;

#include <iostream>
#include <vector>
#include <cstring>
#include <fstream>

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
    CONFIRM_SIDE
};
}


// Settings
unsigned int N_ORDERS = 3;                                                        // Number of orders robot needs to take
unsigned int MAX_N_CONFIRMS = 2;

// Speech
ros::ServiceClient srv_speech_;                                                   // Communication: Service that makes AMIGO speak
ros::ServiceClient speech_recognition_client_;                                    // Communication: Client for starting / stopping speech recognition
ros::Publisher pub_speech_;                                                       // Communication: Publisher that makes AMIGO speak

map<int, pair<string, string> > order_map_; // index, object, desired location

// Administration
bool speech_recognition_turned_on_ = false;
speech_state::SpeechState speech_state_ = speech_state::DRIVE;
std::string current_loc_name_ = "";
std::string current_side_ = "";
unsigned int n_confirmations_ = 0;


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

    return name;
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
            ROS_INFO("Switched on speech recognition");
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

    return true;
}

bool stopSpeechRecognition() {

    //! Turn off speech recognition
    tue_pocketsphinx::Switch::Request req;
    req.action = tue_pocketsphinx::Switch::Request::STOP;
    tue_pocketsphinx::Switch::Response resp;
    if (speech_recognition_client_.call(req, resp))
    {
        if (resp.error_msg == "")
        {
            ROS_INFO("Switched off speech recognition");
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

    return true;
}



/**
 * @brief amigoSpeak let AMIGO say a sentence
 * @param sentence
 */
void amigoSpeak(string sentence, bool block = true) {

    ROS_INFO("AMIGO: \'%s\'", sentence.c_str());

    //setRGBLights("red");

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


    //setRGBLights("green");

}

void updateSpeechState(speech_state::SpeechState new_state)
{
    ROS_INFO("Update speech state from %s to %s", getSpeechStateName(speech_state_).c_str(), getSpeechStateName(new_state).c_str());
    speech_state_ = new_state;
    startSpeechRecognition();
}


void speechCallbackGuide(std_msgs::String res)
{

    std::string answer = res.data;
    ROS_INFO("Received command: %s", answer.c_str());

    // ROBOT IS ASKED TO STOP
    if (speech_state_ == speech_state::DRIVE)
    {
        if (answer == "amigostop")
        {
            // stop robot
            // set unique color
            updateSpeechState(speech_state::LOC_NAME);
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
        }
        else
        {
            // Received location name
            if (answer == "1") current_loc_name_ = "delivery location one";
            else if (answer == "2") current_loc_name_ = "delivery location two";
            else if (answer == "3") current_loc_name_ = "delivery location three";
            else if (answer == "food") current_loc_name_ = "food shelf";
            else if (answer == "drink") current_loc_name_ = "drink shelf";
            else
            {
                // Unknown command!
                ROS_WARN("Robot expects: 1, 2, 3, food or drink (unknown command '%s')", answer.c_str());
                updateSpeechState(speech_state::DRIVE);
            }

            // Ask for confirmation
            if (speech_state_ == speech_state::LOC_NAME)
            {
                amigoSpeak(current_loc_name_ + "?");
                updateSpeechState(speech_state::CONFIRM_LOC);
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
            amigoSpeak("Which side?");
            updateSpeechState(speech_state::SIDE);
            n_confirmations_ = 0;
        }
        else if (n_confirmations_ <= MAX_N_CONFIRMS)
        {
            ROS_INFO("Misunderstood the location name");
            amigoSpeak("Which location?");
            updateSpeechState(speech_state::LOC_NAME);
            ++n_confirmations_;
        }
        else
        {
            amigoSpeak("I give up");
            updateSpeechState(speech_state::DRIVE);
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
        }
        else
        {
            ROS_WARN("Robot expects: left, front or right (unknown command '%s')", answer.c_str());
            updateSpeechState(speech_state::SIDE);
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
            n_confirmations_ = 0;
        }
        else if (n_confirmations_ <= MAX_N_CONFIRMS)
        {
            ROS_INFO("Misunderstood the side");
            amigoSpeak("Which side?");
            updateSpeechState(speech_state::SIDE);
            ++n_confirmations_;
        }
        else
        {
            amigoSpeak("I give up");
            updateSpeechState(speech_state::DRIVE);
        }
    }


}



int main(int argc, char **argv) {

    ros::init(argc, argv, "test_speech_restaurant");
    ros::NodeHandle nh;

    //! Topic/srv that make AMIGO speak
    pub_speech_ = nh.advertise<std_msgs::String>("/text_to_speech/input", 10);
    srv_speech_ =  nh.serviceClient<text_to_speech::Speak>("/text_to_speech/speak");
    ROS_INFO("Publisher/service client for text to speech started");

    //! Start speech recognition
    speech_recognition_client_ = nh.serviceClient<tue_pocketsphinx::Switch>("/pocketsphinx/switch");
    speech_recognition_client_.waitForExistence();
    ros::Subscriber sub_speech = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 10, speechCallbackGuide);
    startSpeechRecognition();
    ROS_INFO("Started speech recognition");


    //sub_speech.shutdown();
    //sub_speech = nh.subscribe<std_msgs::String>("/pocketsphinx/output", 10, speechCallbackOrder);

    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

