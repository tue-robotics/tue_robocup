#include "challenge_follow_me/Follower.h"

// ROS actions
#include "amigo_head_ref/HeadRefActionGoal.h"

// ROS srvs
#include "std_srvs/Empty.h"
#include "perception_srvs/StartPerception.h"

// ROS msgs
#include "tue_move_base_msgs/MoveBaseGoal.h"
#include "std_msgs/String.h"
#include "text_to_speech/Speak.h"

// Conversions
#include "problib/conversions.h"


Follower::Follower(ros::NodeHandle& nh, std::string frame, bool map, bool demo) :
    nh_(nh), wire_client_(0), nav_frame_(frame), use_map_(map),
    t_last_check_(0), t_no_meas_(0), operator_last_var_(-1.0), demo_(demo), first_time_(true)
{
    //! Set initial mode
    mode_ = Follower::IDLE;

    //! Wire
    wire_client_ = new wire::Client();
    reset_wire_srv_client_ = nh_.serviceClient<std_srvs::Empty>("/wire/reset");

    //! Tf listener
    listener_ = new tf::TransformListener();

    //! Move base
    if (!use_map_)
    {
        carrot_planner_ = new CarrotPlanner("Follower_carrot_planner");
    }
    else
    {
        // Connect to move base
        ac_move_base_ = new actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction>("move_base_3d", true);
        ac_move_base_->waitForServer(ros::Duration(10.0));

        // Connect to path planning service
        srv_client_move_base_ = nh_.serviceClient<tue_move_base_msgs::GetPath>("/move_base_3d/get_plan");
        srv_client_move_base_.waitForExistence(ros::Duration(10.0));
    }

    //! Make the robot speak
    pub_speak_ = nh.advertise<std_msgs::String>("/text_to_speech/input", 10);
    srv_speak_ =  nh.serviceClient<text_to_speech::Speak>("/text_to_speech/speak");

    //! Defaults (@todo: ros parameters)
    time_out_operator_lost_ = 10.0;
    wm_prop_operator_ = "name";
    wm_val_operator_ = "operator";
    follow_distance_ = 1.5;
    max_distance_new_operator_  = 2.5;
    robot_base_frame_ = "/amigo/base_link";

    //! Set pan and tilt head
    ac_head_ = new actionlib::SimpleActionClient<amigo_head_ref::HeadRefAction>("/head_ref_action", true);
    ROS_INFO("Waiting for head ref server...");
    ac_head_->waitForServer(ros::Duration(5.0));
    setHeadPanTilt();
    ROS_INFO("Connected with head ref server");

    //! Connect to pein supervisor
    pein_client_ = nh_.serviceClient<perception_srvs::StartPerception>("/start_perception");

    //! Harcoded settings for finding the operator
    TIME_WAIT_MAX = 10.0;
    DIST_LEFT_RIGHT = 0.35;
    DIST_MIN = 0.5;
    DIST_MAX = 2.0;
    MAX_2D_DISTANCE_TORSO_FACE = 0.5;

}







bool Follower::start()
{
    double t_start = ros::Time::now().toSec();

    //! Set mode
    mode_ = Follower::ACTIVE;

    //! Reset head position
    setHeadPanTilt();

    //! See if the map can be use
    if (use_map_ && !ac_move_base_->isServerConnected() && !!ac_move_base_->waitForServer(ros::Duration(10.0)))
    {
        ROS_ERROR("Could not connect to move base action client, using carrot planner instead.");
        use_map_ = false;
    }

    //! See if paths can be planned
    if (use_map_ && !srv_client_move_base_.exists())
    {
        srv_client_move_base_.waitForExistence(ros::Duration(10.0));
        if (!srv_client_move_base_.exists())
        {
            ROS_ERROR("Could not connect with service client for planning move base paths, using carrot planner instead.");
            use_map_ = false;
            ac_move_base_->cancelAllGoals();
        }
    }

    ROS_INFO("First part of starting the follower took %f [ms]", 1000*(ros::Time::now().toSec()-t_start));

    //! Reset WIRE
    std_srvs::Empty srv;
    if (!reset_wire_srv_client_.call(srv))
    {
        ROS_WARN("Failed to clear world model, start module anyway");
    }

    //! Start
    update();

    return true;
}

void Follower::resume()
{
    mode_ = Follower::ACTIVE;
    update();
}

void Follower::pause()
{
    freezeRobot();
    mode_ = Follower::PAUSE;
}

void Follower::stop()
{

    freezeRobot();
    pein_client_.shutdown();
    ac_head_->cancelAllGoals();
    listener_->clear();
    mode_ = Follower::IDLE;
}

void Follower::freezeRobot()
{
    ROS_INFO("Freeze robot!");

    //! Stop moving
    if (use_map_)
    {
        ac_move_base_->cancelAllGoals();
    }
    else
    {
        carrot_planner_->freeze();
    }
}


bool Follower::reset()
{
    double t_start = ros::Time::now().toSec();

    // Reset WIRE
    std_srvs::Empty srv;
    bool reset = reset_wire_srv_client_.call(srv);
    ROS_INFO("Resetting WIRE took %f [ms]", 1000*(ros::Time::now().toSec()-t_start));

    // Start following
    bool started = start();

    if (!reset)
    {
        ROS_WARN("Failed to clear world model, start module anyway");
    }
    ROS_INFO("Resetting follower took %f [ms]", 1000*(ros::Time::now().toSec()-t_start));

    return (reset && started);
}



bool Follower::update()
{
    //// Checks and inform user if needed
    if (mode_ == Follower::ACTIVE)
    {
        double t = ros::Time::now().toSec();
        if (t-t_last_check_ > 2.0 && !first_time_)
        {
            ROS_WARN("Last update is %f [s] ago, make sure update rate is reasonable!", t-t_last_check_);
        }

    }
    else if (mode_ == Follower::IDLE)
    {
        ROS_WARN("Request for follower update while mode is idle.");
        return true;
    }
    else if (mode_ == Follower::PAUSE)
    {
        ROS_INFO("Follower is paused");
        return true;
    }
    else
    {
        ROS_ERROR("Unknown mode in Follower class");
        return false;
    }

    ROS_DEBUG("In update function");

    //// Query WIRE
    std::vector<wire::PropertySet> objects = wire_client_->queryMAPObjects(nav_frame_);

    //// Get position operator
    pbl::Gaussian pos_operator(3);
    if (getPositionOperator(objects, pos_operator))
    {
        ROS_DEBUG("Found operator");
    }
    else
    {
        ROS_WARN("No operator in the world model: trying to find one");
        if (demo_)
        {
            if (!findOperator(pos_operator))
            {
                ROS_WARN("Operator cannot be found (using face segmentation)!");
                return false;
            }
        }
        else
        {
            if (!findOperatorFast(pos_operator))
            {
                ROS_WARN("Operator cannot be found (without using face segmentation)!");
                return false;
            }
        }
    }

    // NOTE: At this point the position of the operator must be known


    //// Steer robot towards operator with offset (non-blocking)
    bool move_ok = true;
    if (mode_ != Follower::PAUSE)
    {
        move_ok = moveTowardsPosition(pos_operator, follow_distance_, false);
    }

    return move_ok;

}

bool Follower::sendOwnGoalPose(double x, double y, double theta, std::string frame)
{

    //! End point of the path is the given position
    geometry_msgs::PoseStamped end_goal;
    end_goal.header.frame_id = frame;
    end_goal.header.stamp = ros::Time();

    //! Set orientation
    addQuaternion(end_goal, theta);

    //! Set the position
    end_goal.pose.position.x = x;
    end_goal.pose.position.y = y;
    end_goal.pose.position.z = 0;

    //! Transform to the appropriate frame
    transformPoseStamped(end_goal, end_goal, nav_frame_);

    // Move towards position
    ROS_DEBUG("Follower: Move base custom goal: (x,y,theta) = (%f,%f,%f)", end_goal.pose.position.x, end_goal.pose.position.y, theta);

    // See which of the methods must be used
    if (use_map_)
    {
        tue_move_base_msgs::GetPath srv_get_path;
        if (!findPath(end_goal, srv_get_path))
        {
            // No path found
            return false;
        }

        // Send path
        tue_move_base_msgs::MoveBaseGoal base_goal;
        base_goal.path = srv_get_path.response.path;
        ac_move_base_->sendGoal(base_goal);
        if (!ac_move_base_->waitForResult(ros::Duration(60.0)))
        {
            ROS_WARN("Robot could not custom reach map position before time-out!");
        }
    }
    else
    {
        carrot_planner_->MoveToGoal(end_goal);
    }

    return true;

}


void Follower::addQuaternion(geometry_msgs::PoseStamped& ps, double theta)
{
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    ps.pose.orientation.x = q.getX();
    ps.pose.orientation.y = q.getY();
    ps.pose.orientation.z = q.getZ();
    ps.pose.orientation.w = q.getW();

}



bool Follower::getPositionGaussian(pbl::PDF pos, pbl::Gaussian& pos_gauss)
{
    //! Convert position if needed
    if (pos.type() == pbl::PDF::GAUSSIAN)
    {
        pos_gauss = pbl::toGaussian(pos);

    }
    else if (pos.type() == pbl::PDF::MIXTURE)
    {
        // Get most probable Gaussian in case of a mixture
        pbl::Mixture mix = pbl::toMixture(pos);
        double w_best = 0;
        for (unsigned int i = 0; i < mix.components(); ++i)
        {
            pbl::PDF pdf = mix.getComponent(i);
            pbl::Gaussian G = pbl::toGaussian(pdf);
            double w = mix.getWeight(i);
            if (G.isValid() && w > w_best)
            {
                pos_gauss = G;
                w_best = w;
            }
        }
    }
    else
    {
        ROS_WARN("Cannot convert position (not a Gaussian or mixture)");
        return false;
    }

    return true;
}



bool Follower::getPositionOperator(std::vector<wire::PropertySet>& objects, pbl::Gaussian& pos_operator)
{
    //! Iterate over all world model objects
    for(std::vector<wire::PropertySet>::iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj)
    {
        wire::PropertySet& obj = *it_obj;
        const wire::Property& prop_name = obj.getProperty(wm_prop_operator_);

        //! See if there is operator property is present
        if (prop_name.isValid())
        {

            //! Check if the object represents the operator
            if (prop_name.getValue().getExpectedValue().toString() == wm_val_operator_)
            {
                //! See if the operator has a position
                const wire::Property& prop_pos = obj.getProperty("position");
                if (prop_pos.isValid())
                {

                    //! Get position
                    pbl::Gaussian pos_gauss(3);
                    getPositionGaussian(prop_pos.getValue(), pos_gauss);
                    pos_operator = pos_gauss;

                    //! Operator found!
                    pbl::Matrix cov = pos_gauss.getCovariance();
                    ROS_DEBUG("Operator has variance %f, last variance is %f", cov(0,0), operator_last_var_);

                    //! Check if operator position is updated or this is the first detection (initialled negative)
                    if (cov(0,0) < operator_last_var_ || operator_last_var_ < 0)
                    {
                        // Position operator is updated
                        t_no_meas_ = 0;
                        t_last_check_ = ros::Time::now().toSec();

                    }
                    else
                    {
                        // Position operator is not updated
                        t_no_meas_ += (ros::Time::now().toSec() - t_last_check_);

                        // Inform user if needed
                        if (t_no_meas_ > 1) ROS_INFO("%f [s] without position update operator: ", t_no_meas_);

                        //! Position uncertainty increased too long: operator lost
                        if (t_no_meas_ > time_out_operator_lost_)
                        {
                            ROS_INFO("Operator is lost!");
                            say("I lost my operator");
                            return false;
                        }
                    }

                    t_last_check_ = ros::Time::now().toSec();

                    // Update last variance
                    operator_last_var_ = cov(0,0);

                    return true;

                }
                else {
                    ROS_WARN("Found an operator without valid position attribute");
                }
            } // end operator property value check
        } // end operator
    }

    //! If no operator was found, return false
    return false;
}


void Follower::say(std::string sentence, bool block)
{
    ROS_INFO("Robot: \'%s\'", sentence.c_str());

    //! Call speech service (topic if srv fails)
    text_to_speech::Speak speak;
    speak.request.sentence = sentence;
    speak.request.language = "us";
    speak.request.character = "kyle";
    speak.request.voice = "default";
    speak.request.emotion = "normal";
    speak.request.blocking_call = block;
    if (!srv_speak_.call(speak))
    {
        // Use topic if service call fails
        std_msgs::String sentence_msgs;
        sentence_msgs.data = sentence;
        pub_speak_.publish(sentence_msgs);
    }

}



bool Follower::findOperator(pbl::Gaussian& pos_operator)
{


    //! Toggle perception
    perception_srvs::StartPerception pein_srv;
    pein_srv.request.modules.push_back("ppl_detection");
    pein_srv.request.modules.push_back("face_segmentation");
    if (!pein_client_.call(pein_srv))
    {
        pein_srv.request.modules.clear();
        pein_srv.request.modules.push_back("ppl_detection");
        if (!pein_client_.call(pein_srv))
        {
            ROS_ERROR("Cannot switch on perception, hence unable to find an operator!");
            return false;
        }
    }
    else
    {
        ROS_INFO("Switched on both ppl_detection and face_segmentation");
    }

    if (first_time_ || demo_)
    {
        say("I am looking for my operator, can you please stand in front of me", true);
    }

    //! Reset world model
    std_srvs::Empty srv;
    if (!reset_wire_srv_client_.call(srv)) ROS_WARN("Failed to clear world model");

    //! Vector with candidate operators
    std::vector<pbl::Gaussian> vector_possible_operator_torsos;
    pbl::Gaussian pos_closest_face_gauss(3);
    double d_closest_face = -1.0;

    //! See if the a person stands in front of the robot
    double t_start = ros::Time::now().toSec();
    ros::Duration dt(0.5);
    bool found_operator = false;
    while (ros::Time::now().toSec() - t_start < TIME_WAIT_MAX && !found_operator)
    {

        // Get latest world state estimate
        std::vector<wire::PropertySet> objects = wire_client_->queryMAPObjects(robot_base_frame_);
        ROS_INFO("World model contains %zu objects", objects.size());

        //! Iterate over all world model objects and look for a torso or face in front of the robot
        for(std::vector<wire::PropertySet>::iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj)
        {
            //// PERSON
            wire::PropertySet& obj = *it_obj;
            const wire::Property& prop_label = obj.getProperty("class_label");
            if (prop_label.isValid() && prop_label.getValue().getExpectedValue().toString() == "person")
            {
                //ROS_INFO("Found a person!");

                // Check position
                const wire::Property& prop_pos = obj.getProperty("position");
                if (prop_pos.isValid())
                {

                    //! Get position of potential operator
                    pbl::Gaussian pos_gauss(3);
                    getPositionGaussian(prop_pos.getValue(), pos_gauss);

                    //! Check if the person stands in front of the robot
                    if (pos_gauss.getMean()(0) < DIST_MAX &&
                            pos_gauss.getMean()(0) > DIST_MIN &&
                            pos_gauss.getMean()(1) > -1.0*DIST_LEFT_RIGHT &&
                            pos_gauss.getMean()(1) < DIST_LEFT_RIGHT)
                    {
                        vector_possible_operator_torsos.push_back(pos_gauss);
                        ROS_INFO("Found candidate operator torso at (x,y) = (%f,%f)", pos_gauss.getMean()(0), pos_gauss.getMean()(1));

                    }
                    else
                    {
                        ROS_DEBUG("Torso at (x,y) = (%f,%f) is not a candidate operator", pos_gauss.getMean()(0), pos_gauss.getMean()(1));
                    }
                }

            }
            //// FACE
            else if (prop_label.isValid() && prop_label.getValue().getExpectedValue().toString() == "face")
            {
                //ROS_INFO("Found a face!");
                // Check position
                const wire::Property& prop_pos = obj.getProperty("position");
                if (prop_pos.isValid()) {

                    //! Get position of potential operator
                    pbl::Gaussian pos_gauss(3);
                    getPositionGaussian(prop_pos.getValue(), pos_gauss);

                    //! Store position of the closest person (wrt robot frame)
                    double d_face = std::sqrt(pos_gauss.getMean()(0)*pos_gauss.getMean()(0) + pos_gauss.getMean()(1) * pos_gauss.getMean()(1));
                    ROS_INFO("Distance current face is %f", d_face);
                    if ((d_closest_face < 0 || d_face < d_closest_face) &&
                            d_face < DIST_MAX && d_face > DIST_MIN &&
                            pos_gauss.getMean()(1) > -1.0*DIST_LEFT_RIGHT && pos_gauss.getMean()(1) < DIST_LEFT_RIGHT)
                    {
                        pos_closest_face_gauss = pos_gauss;
                        d_closest_face = d_face;
                        ROS_INFO("Updated closest face: (x,y) = (%f,%f)", pos_gauss.getMean()(0), pos_gauss.getMean()(1));

                    }
                    else
                    {
                        ROS_DEBUG("Face at (x,y,z) = (%f,%f,%f) in robot frame is not a candidate face",
                                  pos_gauss.getMean()(0), pos_gauss.getMean()(1), pos_gauss.getMean()(2));
                    }
                }
            }
            //// End person or face
            else
            {
                if (prop_label.isValid()) ROS_INFO("Found a %s", prop_label.getValue().getExpectedValue().toString().c_str());
                else ROS_INFO("Found object without valid class label!");
            }

        } // Done looping over world model objects

        //// SEE IF A TORSO/FACE PAIR WHICH COULD BE THE OPERATOR IS FOUND

        //! Found at least one candidate operator
        if (!vector_possible_operator_torsos.empty() && d_closest_face > 0)
        {

            // Find closest torso closest to the face
            double dist_min = MAX_2D_DISTANCE_TORSO_FACE;
            for (unsigned int i = 0; i < vector_possible_operator_torsos.size(); ++i)
            {
                // Calculate distance torso/face
                double dx = pos_closest_face_gauss.getMean()(0) - vector_possible_operator_torsos[i].getMean()(0);
                double dy = pos_closest_face_gauss.getMean()(1) - vector_possible_operator_torsos[i].getMean()(1);
                double dist = sqrt(dx*dx+dy*dy);
                if (i == 0 || dist < dist_min)
                {
                    dist_min = dist;
                    pos_operator = vector_possible_operator_torsos[i];
                }
            }


            //! See if the face and torso are close enough
            if (dist_min < MAX_2D_DISTANCE_TORSO_FACE)
            {
                //! Operator is found!
                ROS_INFO("Found operator at (x,y) = (%f,%f)", pos_operator.getMean()(0), pos_operator.getMean()(1));

                //! Assert operator property to WIRE
                wire::Evidence ev(ros::Time::now().toSec());
                // Position
                ev.addProperty("position", pos_operator, nav_frame_);
                // Name: operator
                pbl::PMF name_pmf;
                name_pmf.setProbability(wm_val_operator_, 1.0);
                ev.addProperty(wm_prop_operator_, name_pmf);
                // Class label: person
                pbl::PMF class_pmf;
                class_pmf.setProbability("person", 1.0);
                ev.addProperty("class_label", class_pmf);

                // Reset the world model
                std_srvs::Empty srv;
                if (!reset_wire_srv_client_.call(srv)) ROS_WARN("Failed to clear world model");

                // Assert evidence to WIRE
                wire_client_->assertEvidence(ev);

                // Inform user (wait since speech is non-blocking)
                if (first_time_ || demo_)
                {
                    say("I will follow you now", true);
                }
                else
                {
                    say("Let's go");
                }

                //! Reset
                operator_last_var_ = -1;
                t_last_check_ = ros::Time::now().toSec();
                t_no_meas_ = 0;
                first_time_ = false;

                // Break from while loop
                found_operator = true;
            }


        } // end if at least one candidate operator

        dt.sleep();
    }


    //! Switch off face detection
    pein_srv.request.modules.clear();
    pein_srv.request.modules.push_back("ppl_detection");
    if (pein_client_.call(pein_srv))
    {
        ROS_DEBUG("Switched off face detection: only ppl_detection active.");
    }
    else
    {
        // this should not happen
        ROS_ERROR("Follower could not switched off face_detection and turn on ppl_detection");
    }

    if (!found_operator)
    {
        say("I did not find my operator yet", true);
        return false;
    }

    return true;
}


bool Follower::findOperatorFast(pbl::Gaussian& pos_operator)
{
    //! @todo: for now hardcoded settings (must all be positive)
    double TIME_WAIT_MAX = 10.0;
    double DIST_LEFT_RIGHT = 0.35;
    double DIST_MIN = 0.5;
    double DIST_MAX = 2.0;

    //! Toggle perception
    perception_srvs::StartPerception pein_srv;
    pein_srv.request.modules.push_back("ppl_detection");
    if (!pein_client_.call(pein_srv))
    {
        pein_srv.request.modules.clear();
        pein_srv.request.modules.push_back("ppl_detection");
        if (!pein_client_.call(pein_srv))
        {
            ROS_ERROR("Cannot switch on perception, hence unable to find an operator!");
            return false;
        }
    }
    else
    {
        ROS_INFO("Switched on ppl_detection");
    }

    if (first_time_ || demo_)
    {
        say("I am looking for my operator, can you please stand in front of me", true);
    }

    //! Reset world model
    std_srvs::Empty srv;
    if (!reset_wire_srv_client_.call(srv)) ROS_WARN("Failed to clear world model");

    //! Vector with candidate operators
    std::vector<pbl::Gaussian> vector_possible_operator_torsos;

    //! See if the a person stands in front of the robot
    double t_start = ros::Time::now().toSec();
    ros::Duration dt(0.5);
    bool found_operator = false;
    while (ros::Time::now().toSec() - t_start < TIME_WAIT_MAX && !found_operator)
    {

        // Get latest world state estimate
        std::vector<wire::PropertySet> objects = wire_client_->queryMAPObjects(robot_base_frame_);
        ROS_INFO("World model contains %zu objects", objects.size());

        //! Iterate over all world model objects and look for a torso or face in front of the robot
        for(std::vector<wire::PropertySet>::iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj)
        {
            //// PERSON
            wire::PropertySet& obj = *it_obj;
            const wire::Property& prop_label = obj.getProperty("class_label");
            if (prop_label.isValid() && prop_label.getValue().getExpectedValue().toString() == "person")
            {
                //ROS_INFO("Found a person!");

                // Check position
                const wire::Property& prop_pos = obj.getProperty("position");
                if (prop_pos.isValid())
                {

                    //! Get position of potential operator
                    pbl::Gaussian pos_gauss(3);
                    getPositionGaussian(prop_pos.getValue(), pos_gauss);

                    //! Check if the person stands in front of the robot
                    if (pos_gauss.getMean()(0) < DIST_MAX &&
                            pos_gauss.getMean()(0) > DIST_MIN &&
                            pos_gauss.getMean()(1) > -1.0*DIST_LEFT_RIGHT &&
                            pos_gauss.getMean()(1) < DIST_LEFT_RIGHT)
                    {
                        vector_possible_operator_torsos.push_back(pos_gauss);
                        ROS_INFO("\tcandidate operator torso at (x,y) = (%f,%f)", pos_gauss.getMean()(0), pos_gauss.getMean()(1));

                    }
                    else
                    {
                        ROS_DEBUG("\ttorso at (x,y) = (%f,%f) (no candidate operator)", pos_gauss.getMean()(0), pos_gauss.getMean()(1));
                    }
                }

            }

        } // Done looping over world model objects

        //// Select most probable torso

        //! Found at least one candidate operator
        if (!vector_possible_operator_torsos.empty())
        {

            // Find closest torso closest to the face
            double offset_min = 0.0;
            for (unsigned int i = 0; i < vector_possible_operator_torsos.size(); ++i)
            {
                if (vector_possible_operator_torsos[i].getMean()(1) < offset_min || i == 0)
                {
                    offset_min = vector_possible_operator_torsos[i].getMean()(1);
                    pos_operator = vector_possible_operator_torsos[i];
                }
            }

            //! Operator is found!
            ROS_INFO("Found operator at (x,y) = (%f,%f)", pos_operator.getMean()(0), pos_operator.getMean()(1));

            //! Assert operator property to WIRE
            wire::Evidence ev(ros::Time::now().toSec());
            // Position
            ev.addProperty("position", pos_operator, nav_frame_);
            // Name: operator
            pbl::PMF name_pmf;
            name_pmf.setProbability(wm_val_operator_, 1.0);
            ev.addProperty(wm_prop_operator_, name_pmf);
            // Class: person
            pbl::PMF class_pmf;
            class_pmf.setProbability("person", 1.0);
            ev.addProperty("class_label", class_pmf);

            // Reset the world model
            std_srvs::Empty srv;
            if (!reset_wire_srv_client_.call(srv)) ROS_WARN("Failed to clear world model");

            //! Assert to WIRE
            wire_client_->assertEvidence(ev);

            // Inform user (wait since speech is non-blocking)
            if (first_time_ || demo_)
            {
                say("I will follow you now", true);
            }
            else
            {
                say("Let's go", false);
            }

            //! Reset
            operator_last_var_ = -1;
            t_last_check_ = ros::Time::now().toSec();
            t_no_meas_ = 0;
            first_time_ = false;

            // Break from while loop
            found_operator = true;

        } // end if at least one candidate operator

        dt.sleep();
    }

    if (!found_operator && (first_time_ || demo_))
    {
        say("I did not find my operator yet", true);
        return false;
    }
    else if (!found_operator)
    {
        ROS_INFO("Robot did not yet find an operator!");
    }

    return true;
}



bool Follower::setHeadPanTilt(double pan, double tilt, bool block)
{
    if (!ac_head_->isServerConnected() && ac_head_->waitForServer(ros::Duration(1.0)))
    {
        ROS_WARN("Action client head pan/tilt is not connected and cannot connect!");
        return false;
    }

    amigo_head_ref::HeadRefGoal head_goal;
    head_goal.goal_type = amigo_head_ref::HeadRefGoal::PAN_TILT;
    head_goal.pan = pan;
    head_goal.tilt = tilt;

    // Send goal
    ROS_DEBUG("Sending head goal: pan = %f, tilt = %f", pan, tilt);
    ac_head_->sendGoal(head_goal);
    if (block)
    {
        ac_head_->waitForResult(ros::Duration(3.0));
    }

    return true;
}

bool Follower::moveTowardsPosition(pbl::Gaussian& pos, double offset, bool block)
{

    //! Get the position
    pbl::Vector pos_exp = pos.getExpectedValue().getVector();

    //! End point of the path is the given position
    geometry_msgs::PoseStamped end_goal;
    end_goal.header.frame_id = nav_frame_;
    end_goal.header.stamp = ros::Time();


    //! Set orientation
    double theta = atan2(pos_exp(1), pos_exp(0));
    addQuaternion(end_goal, theta);

    //! Incorporate offset in target position (if too close, setpoint is zero)
    double full_distance = sqrt(pos_exp(0)*pos_exp(0)+pos_exp(1)*pos_exp(1));
    double reduced_distance = std::max(0.0, full_distance - offset);
    end_goal.pose.position.x = pos_exp(0) * reduced_distance / full_distance;
    end_goal.pose.position.y = pos_exp(1) * reduced_distance / full_distance;
    end_goal.pose.position.z = 0;

    //! Send goal to planner
    if (t_no_meas_ > 1.5)
    {
        ROS_INFO("No operator position update: robot will not move");
        if (use_map_) {
            ac_move_base_->cancelAllGoals();
        }
        else
        {
            end_goal.pose.position.x = 0;
            end_goal.pose.position.y = 0;
            //carrot_planner_->MoveToGoal(end_goal);
        }
    }
    else
    {
        // Move towards position
        ROS_DEBUG("Follower: Move base goal: (x,y,theta) = (%f,%f,%f), red. and full distances are %f and %f",
                  end_goal.pose.position.x, end_goal.pose.position.y, theta, reduced_distance, full_distance);

        // See which of the methods must be used
        if (use_map_)
        {
            tue_move_base_msgs::GetPath srv_get_path;
            if (!findPath(end_goal, srv_get_path))
            {
                // No path found
                return false;
            }

            // Send path
            tue_move_base_msgs::MoveBaseGoal base_goal;
            base_goal.path = srv_get_path.response.path;
            ac_move_base_->sendGoal(base_goal);
            if (block && !ac_move_base_->waitForResult(ros::Duration(60.0)))
            {
                ROS_WARN("Robot could not reach map position before time-out!");
            }
        }
        else
        {
            carrot_planner_->MoveToGoal(end_goal);
        }

    }

    return true;

}


bool Follower::transformPoseStamped(geometry_msgs::PoseStamped& in, geometry_msgs::PoseStamped& out, std::string frame_out)
{
    //! Transform goal to the map frame
    try
    {
        listener_->transformPose(frame_out, in, out);
    }
    catch (tf::TransformException& e)
    {
        ROS_WARN("Could not transform goal to map frame: %s", e.what());
        return false;
    }

    return true;
}



bool Follower::findPath(geometry_msgs::PoseStamped& end_goal, tue_move_base_msgs::GetPath& srv_get_path)
{
    //! Transform goal to the map frame
    if (!transformPoseStamped(end_goal, srv_get_path.request.target_pose, "/map"))
    {
        return false;
    }

    //! Get the path
    if (srv_client_move_base_.call(srv_get_path))
    {
        // See if a path is found
        if (srv_get_path.response.path.empty())
        {
            ROS_ERROR("No path found.");
            return false;
        }

        // Path found!
        return true;
    }
    else
    {
        ROS_ERROR("Path request failed");
    }

    return false;
}
