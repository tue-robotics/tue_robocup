/**
 * USB ON UBUNTU SERVER EDITION (AMIGO)
 * 
 * Before starting this node, make sure you type:
 * $ pumount --yes-I-really-want-lazy-unmount /media/usb-stick/
 * $ pmount /dev/sdc1 usb-stick
 * 
 * This will mount the device at /media/sdc1 (which is now hardcoded as path) with the correct permissions
 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <setjmp.h>
#include "hpdf.h"
#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "challenge_emergency/Start.h"

//! Dealing with files
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <dirent.h>

using namespace std;

/*
 * PDF Library stuff
 */
#ifndef HPDF_NOPNGLIB
jmp_buf env;

#ifdef HPDF_DLL
void  __stdcall
#else
void
#endif
error_handler  (HPDF_STATUS   error_no,
                HPDF_STATUS   detail_no,
                void         *user_data)
{
    printf ("ERROR: error_no=%04X, detail_no=%u\n", (HPDF_UINT)error_no,
            (HPDF_UINT)detail_no);
    longjmp(env, 1);
}




/// Service for starting
ros::ServiceServer startupSrv_;


/// Scaling parameters for robotics_labA
const double scaledWidth = 1/0.025;
const double scaledHeight = 1/0.025;
const int originOffsetX = 41;
const int originOffsetY = 100;

/* Scaling parameters for rwc2013 -> EINDHOVEN WK
const double scaledWidth = 1/0.025;
const double scaledHeight = 1/0.025;
const int originOffsetX = -6;
const int originOffsetY = 375;
*/


/// Safe string formatting
std::string string_format(const std::string fmt, ...) {
    int size = 100;
    std::string str;
    va_list ap;
    while (1) {
        str.resize(size);
        va_start(ap, fmt);
        int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
        va_end(ap);
        if (n > -1 && n < size) {
            str.resize(n);
            return str;
        }
        if (n > -1)
            size = n + 1;
        else
            size *= 2;
    }
    return str;
}

/// Find the USB directorty
void findUSBDir(std::string& usb_dir, std::string& img_path)
{
    //! Find USB name
    bool found_file_in_media = false;
    DIR *dir;
    struct dirent *ent;
    //string usb_dir, img_path;
    if ((dir = opendir ("/media/")) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            ROS_INFO("file or directory in /media/: %s", string(ent->d_name).c_str());
            if (strlen(ent->d_name)>3) // TODO: hack for ubuntu server strlen(ent->d_name)>3)
            {
                printf("%s\n", ent->d_name);
                usb_dir = "/media/" + string(ent->d_name) + "/emergency_paper.pdf";
                img_path = "/media/" + string(ent->d_name) + "/";
                found_file_in_media = true;

            }
        }
        closedir (dir);
    } else {
        /* could not open directory */
        perror ("");
        exit(EXIT_FAILURE);
    }

    if (!found_file_in_media) {
        ROS_WARN("No files/directories found in /media/ -> no USB stick to write to");
        //usb_dir = "~";
        usb_dir = ros::package::getPath("challenge_emergency")+"/emergency_paper.pdf";
        //img_path = "~";
        img_path = ros::package::getPath("challenge_emergency")+"/";
    }

    // save location
    ROS_INFO("usb_dir = %s", usb_dir.c_str());
    //ROS_INFO("pdf will be stored as %s", fname);
}



/// Initialize the PDF object
void initializePDFObject(HPDF_Doc& pdf, HPDF_Font& font)
{
    pdf = HPDF_New (error_handler, NULL);
    if (!pdf) {
        printf ("error: cannot create PdfDoc object\n");
        exit(1);
    }

    //! Error Handler
    if (setjmp(env)) {
        HPDF_Free (pdf);
        exit(1);
    }

    HPDF_SetCompressionMode (pdf, HPDF_COMP_ALL);

    //! Create default font
    font = HPDF_GetFont (pdf, "Helvetica", NULL);
}


/// Read the status file
void readStatusFile(int** statusArray, float*** coordinatesArray)
{

    //! Define string to pull out each line
    string STRING;
    ifstream myfile;
    int number_lines = 0;

    //! Find number of lines
    myfile.open((ros::package::getPath("challenge_emergency")+"/output/status.txt").c_str());
    while(getline(myfile,STRING))
    {
        number_lines++;
    }
    myfile.close();

    //! Reopen to fill arrays status and coordinates
    *statusArray = new int[number_lines];
    *coordinatesArray = new float*[number_lines];
    for(int i = 0; i < number_lines; i++)
        (*coordinatesArray)[i] = new float[2];

    myfile.open((ros::package::getPath("challenge_emergency")+"/output/status.txt").c_str());

    //! Actual filling
    for (int ii = 0; ii < number_lines; ii++)
    {

        //! Go through every line
        getline(myfile,STRING);
        unsigned found = STRING.find(";");  //find first ';'
        STRING = STRING.substr(found+1);
        //! Prutscode to seperate 'status'/'coordinates'
        string statuss = STRING.substr(0,1);//status = {0,1,2}

        found = STRING.find(";");
        STRING = STRING.substr(found+1);    //move to next ';'
        found = STRING.find(";");
        string x = STRING.substr(0,found);  //length unknown (float)
        string y = STRING.substr(found+1);  //length unknown (float)

        //! Save 'status' information to array
        (*statusArray)[ii] = atoi(statuss.c_str());

        //! Save coordinates of object (fire/person)  in 'x' and 'y'
        (*coordinatesArray)[ii][0] = atof(x.c_str());
        (*coordinatesArray)[ii][1] = atof(y.c_str());
    }

    myfile.close();
}


/// Read parametrs from parameter server
void readParameters(double& x_null, double& y_null, double& length_map, double& height_map)
{
    ros::NodeHandle nh("~");
    //get namespace
    string ns = ros::this_node::getName();
    ROS_INFO("ns = %s",ns.c_str());
    //! Starting location of the map
    if (nh.getParam(ns+"/x_null", x_null))
    {
        ROS_INFO("Got param x_null: %f", x_null);
    }
    else
    {
        ROS_ERROR("Failed to get param 'x_null'.");
    }
    if (nh.getParam(ns+"/y_null", y_null))
    {
        ROS_INFO("Got param y_null: %f", y_null);
    }
    else
    {
        ROS_ERROR("Failed to get param 'x_null'.");
    }

    //! Scaling between image and actual  map
    if (nh.getParam(ns+"/length_map", length_map))
    {
        ROS_INFO("Got param length_map: %f", length_map);
    }
    else
    {
        ROS_ERROR("Failed to get param 'length_map'.");
    }
    if (nh.getParam(ns+"/height_map", height_map))
    {
        ROS_INFO("Got param height_map: %f", height_map);
    }
    else
    {
        ROS_ERROR("Failed to get param 'height_mapl'.");
    }

}


int createPDF()
{
    ros::NodeHandle nh("~");

    HPDF_Doc  pdf;
    HPDF_Font font;
    HPDF_Page page[5];
    const char* fname;
    HPDF_Destination dst;
    HPDF_Image image_map;
    HPDF_Image image_person;

    //! Position of map
    double x_map;
    double y_map;
    //! Number of page
    int n_page = 0;
    //! Image width and height
    double iw;
    double ih;

    std::string usb_dir, img_path;

    //Status arrays
    int* status = NULL;
    float** coordinates = NULL;

    // Map [0,0]
    double x_null;// = (450/2)-x_map;
    double y_null;// = y_map;

    // Map size
    double length_map;
    double height_map;

    /*
     * Begin intializing
     */

    //Read the status file
    readStatusFile(&status, &coordinates);

    //Find correct usb dir
    findUSBDir(usb_dir, img_path);

    fname = usb_dir.c_str();

    //Initialize the PDF object
    initializePDFObject(pdf, font);

    //Read parameters from server
    readParameters(x_null, y_null, length_map, height_map);

    /*
     * End initialization
     */

    //! Add a new page object
    page[n_page] = HPDF_AddPage (pdf);

    HPDF_Page_SetWidth (page[n_page], 550);
    HPDF_Page_SetHeight (page[n_page], 650);

    //! Position on page
    double x = 50;
    double y = HPDF_Page_GetHeight (page[n_page]) - 50; // 0;

    dst = HPDF_Page_CreateDestination (page[n_page]);
    HPDF_Destination_SetXYZ (dst, 0, HPDF_Page_GetHeight (page[n_page]), 1);
    HPDF_SetOpenAction(pdf, dst);

    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 12);
    HPDF_Page_MoveTextPos (page[n_page], x, y);
    HPDF_Page_ShowText (page[n_page], "Emergency Report, Tech United Eindhoven");
    HPDF_Page_EndText (page[n_page]);

    //! Create header with overview of information
    y = y - 40;
    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 16);
    HPDF_Page_MoveTextPos (page[n_page], x, y); // HEADER
    HPDF_Page_ShowText (page[n_page], "Header");
    HPDF_Page_EndText (page[n_page]);

    y = y-18;
    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 12);
    HPDF_Page_MoveTextPos (page[n_page], x, y);
    HPDF_Page_ShowText (page[n_page], "Location: Kitchen room");
    HPDF_Page_EndText (page[n_page]);

    y = y - 14;

    //! Load image of 'map' and 'fire' and 'symbolic fire'
    image_map = HPDF_LoadPngImageFromFile (pdf, (ros::package::getPath("challenge_emergency")+"/output/map.png").c_str());

    //! Image width and height of map needed for transformations of placing fire and people in map
    iw = HPDF_Image_GetWidth (image_map);
    if (iw > 500)
    {
        iw = 500;//may not exceed page width
    }
    ih = HPDF_Image_GetHeight (image_map);
    HPDF_Page_SetLineWidth (page[n_page], 0.5);

    //! Coordinates for map (needed for placing persons/fire in map)
    //y = y - (ih+50);
    y_map = y;


    //! Read map into OpenCV
    cv::Mat image_map_cv = cv::imread(ros::package::getPath("challenge_emergency") + "/output/map.png");
    ROS_INFO("Loaded map image");

    //! Calculate tranformation ratio between pixels/meters
    double ratios [2] = { iw/length_map, ih/height_map };
    ROS_INFO("Ratio: (iw / length_map) %f, (ih / height_map) %f", ratios[0], ratios[1]);

    //! Initialize strings to display person (number, status, coordinates and image)
    std::string person_num;
    std::string person_stat;
    std::string person_coords;
    std::string person_image;
    std::string new_str ;
    float r = 0; //print red or black depends on status


    //! Find the number of persons (= status-'fire')
    //int n_person = sizeof(status)/sizeof(int);

    //! Loop over persons and fire!
    int person_index = 1;
    ROS_INFO("Start looping over persons");

    for (int i = 0; i < 1; i++){

        //! Position person/fire
        int pos_x = coordinates[i][0] * scaledWidth+originOffsetX;
        int pos_y = image_map_cv.size().height - coordinates[i][1] * scaledHeight-originOffsetY;

        //! Give number to person

        person_num = string_format("Person,");

        //! Give status to person
        if (status[i]==0)
        {
            ROS_INFO("Status HELP");
            person_stat = "\n Status: Need Assistance!";
            r = 1;

            //! Draw a picture of 'fire' with subscript
            stringstream ss;
            ss << "pers" << person_index;
            cv::putText(image_map_cv, ss.str(), cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0), 1, 8);
            cv::putText(image_map_cv, "help!", cv::Point(pos_x, pos_y+20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0), 1, 8);

            ROS_INFO("Added text to image (help) at coords: %f, %f",pos_x, pos_y);
        }
        else if (status[i]==1)
        {
            ROS_INFO("Status OK");
            person_stat = "\n Status: Ok!";
            r = 0;


            //! Draw a picture of 'fire' with subscript
            stringstream ss;
            ss << "pers" << person_index;
            cv::putText(image_map_cv, ss.str(), cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0), 1, 8);
            cv::putText(image_map_cv, "=ok", cv::Point(pos_x, pos_y+20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0), 1, 8);

            ROS_INFO("Added text to image (ok)");

        }

        //! New page if y is below treshold
        if (y < 300)
        {
            ROS_INFO("y = %f, new page needed", y);
            n_page = n_page + 1;
            //! Add a new page object
            page[n_page] = HPDF_AddPage (pdf);

            HPDF_Page_SetWidth (page[n_page], 550);
            HPDF_Page_SetHeight (page[n_page], 650);

            dst = HPDF_Page_CreateDestination (page[n_page]);
            HPDF_Destination_SetXYZ (dst, 0, HPDF_Page_GetHeight (page[n_page]), 1);
            HPDF_SetOpenAction(pdf, dst);

            HPDF_Page_BeginText (page[n_page]);
            HPDF_Page_SetFontAndSize (page[n_page], font, 12);
            HPDF_Page_MoveTextPos (page[n_page], x, HPDF_Page_GetHeight (page[n_page]) - 50);
            HPDF_Page_ShowText (page[n_page], "Emergency Report, Tech United Eindhoven");
            HPDF_Page_EndText (page[n_page]);
            y = HPDF_Page_GetHeight (page[n_page]) - 100;
            ROS_INFO("y updated to %f", y);
        }
        y -= 170;

        ROS_INFO("Print person and status");


        //! Give location to person
        person_coords = string_format("\t Location(x,y): x = %f, y = %f",  coordinates[i][0], coordinates[i][1]);



        new_str = person_num + person_stat + person_coords;
        //! Merge person and status and location
        /*if((new_str = (char*) malloc(strlen(person_num)+strlen(person_stat)+strlen(person_coords))+1) != NULL){
            new_str[0] = '\0';   // ensures the memory is an empty string
            strcat(new_str,person_num);
            strcat(new_str,person_coords);
            strcat(new_str,person_stat);
        }
        else {
            printf("malloc failed!\n");
        }*/

        //! Load image of person
        person_image = string_format("/output/person.png");
        string string_person_image = ros::package::getPath("challenge_emergency")+person_image;

        ROS_INFO("Trying to load %s", string_person_image.c_str());
        image_person = HPDF_LoadPngImageFromFile (pdf, string_person_image.c_str());
        ROS_INFO("Loaded image person");


        //! Print picture of person and give status in PDF format
        HPDF_Page_DrawImage (page[n_page], image_person, x, y, 100, 100);
        HPDF_Page_BeginText (page[n_page]);
        HPDF_Page_SetRGBFill (page[n_page], r, 0, 0);
        HPDF_Page_SetFontAndSize (page[n_page], font, 10);
        HPDF_Page_MoveTextPos (page[n_page], x, y-10);
        HPDF_Page_ShowText (page[n_page], new_str.c_str());
        HPDF_Page_EndText (page[n_page]);

        ROS_INFO("Added image person");

        y = y - 120;
    }


    //! Store generated image on disk
    ROS_INFO("Storing image on disk...");
    string file_name_map = img_path + "img_map_cv.png";
    ROS_INFO("Image name: %s", file_name_map.c_str());
    cv::imwrite(file_name_map.c_str(), image_map_cv);
    ROS_INFO("Image stored");

    //! Load this image and draw it in pdf
    HPDF_Image image_map_generated = HPDF_LoadPngImageFromFile (pdf, file_name_map.c_str());
    //HPDF_Page_DrawImage (page[n_page], image_map, x, y, image_map_cv.size().width, image_map_cv.size().height);
    int width_map_in_pdf = image_map_cv.size().width;
    int height_map_in_pdf = image_map_cv.size().height;
    if (width_map_in_pdf > 500) {
        height_map_in_pdf = 500.0/(double)width_map_in_pdf * height_map_in_pdf;
        width_map_in_pdf = 500;
    }

    //! New page
    n_page = n_page + 1;

    //! Add a new page object
    page[n_page] = HPDF_AddPage (pdf);

    HPDF_Page_SetWidth (page[n_page], 550);
    HPDF_Page_SetHeight (page[n_page], 650);

    dst = HPDF_Page_CreateDestination (page[n_page]);
    HPDF_Destination_SetXYZ (dst, 0, HPDF_Page_GetHeight (page[n_page]), 1);
    HPDF_SetOpenAction(pdf, dst);

    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 12);
    HPDF_Page_MoveTextPos (page[n_page], x, HPDF_Page_GetHeight (page[n_page]) - 50);
    HPDF_Page_ShowText (page[n_page], "Emergency Report, Tech United Eindhoven");
    HPDF_Page_EndText (page[n_page]);
    //y = HPDF_Page_GetHeight (page[n_page]) - 170;

    //! Map in pdf
    ROS_INFO("Adding map to pdf...");
    y -= 180;
    HPDF_Page_DrawImage (page[n_page], image_map_generated, x, y, width_map_in_pdf, height_map_in_pdf);
    HPDF_Page_BeginText (page[n_page]);
    HPDF_Page_SetFontAndSize (page[n_page], font, 12);
    HPDF_Page_MoveTextPos (page[n_page], x, y-25.5);
    //HPDF_Page_ShowText (page[n_page], "Apartment: red coordinates present person(s) in need of assistance");
    HPDF_Page_EndText (page[n_page]);
    ROS_INFO("added");

    //! Save the document to a file
    ROS_INFO("Saving pdf to file...");
    HPDF_SaveToFile (pdf, fname);
    ROS_INFO("Saved pdf");

    //! Clean up
    HPDF_Free (pdf);

    return 0;
}


bool startUp(challenge_emergency::Start::Request& req, challenge_emergency::Start::Response& resp)
{
    // Create the pdf brother G
    createPDF();
    return true;
}

int main (int argc, char **argv)
{
    //! Start up node
    ros::init(argc, argv, "emergency_paper");
    ros::NodeHandle nh("~");
    ROS_INFO("Starting pdf creation service..");
    startupSrv_ = nh.advertiseService("startup", &startUp);

    createPDF();

    while(ros::ok())
        ros::spinOnce();
    return 0;
}
#else

int main()
{
    printf("WARNING: if you want to run this demo, \n"
           "make libhpdf with HPDF_USE_PNGLIB option.\n");
    return 0;
}

#endif /* HPDF_NOPNGLIB */
