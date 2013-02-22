#include <ros/ros.h>

#include <rdf/RDFTriples.h>
#include <rdf/conversions.h>

#include <graphviz/gvc.h>

#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <list>

using namespace std;

GVC_t *gvc;


void msgToGraph(const rdf::RDFTriples& msg, Agraph_t* G, vector<Agnode_t*>& nodes, vector<Agedge_t*>& edges) {
    if (msg.subjects.size() != msg.predicates.size() || msg.predicates.size() != msg.objects.size()) {
        ROS_WARN("Ill formed RDF message");
    }

    for(unsigned int i = 0; i < msg.concept_table.size(); ++i) {
        Agnode_t* node = agnode(G,  (char*)msg.concept_table[i].c_str());
        agsafeset(node, (char*)"label", (char*)msg.concept_table[i].c_str(), (char*)"");
        agsafeset(node, (char*)"style", (char*)"filled", (char*)"");
        agsafeset(node, (char*)"color", (char*)"skyblue", (char*)"");
        nodes.push_back(node);
    }

    for(unsigned int i = 0; i < msg.subjects.size(); ++i) {

        Agnode_t* sub = nodes[msg.subjects[i]];

        Agnode_t* obj;

        if (msg.object_types[i] == rdf::RDFTriples::CONCEPT) {
            obj = nodes[msg.objects[i]];
        } else {
            stringstream obj_uri;

            if (msg.object_types[i] == rdf::RDFTriples::INTEGER) {
                obj_uri << msg.integer_table[msg.objects[i]];
            } else if (msg.object_types[i] == rdf::RDFTriples::FLOAT) {
                obj_uri << msg.float_table[msg.objects[i]];
            } else if (msg.object_types[i] == rdf::RDFTriples::STRING) {
                obj_uri << msg.string_table[msg.objects[i]];
            } else if (msg.object_types[i] == rdf::RDFTriples::FLOAT_ARRAY) {
                const vector<double>& vec = msg.float_array_table[msg.objects[i]].array;
                obj_uri << "[";
                for(unsigned int i = 0; i < vec.size(); ++i) {
                    obj_uri << " " << vec[i];
                }
                obj_uri << "]";
            }

            stringstream obj_id;
            obj_id << i;

            obj = agnode(G, (char*)obj_id.str().c_str());
            agsafeset(obj, (char*)"label", (char*)obj_uri.str().c_str(), (char*)"");
            agsafeset(obj, (char*)"style", (char*)"filled", (char*)"");
            agsafeset(obj, (char*)"color", (char*)"green", (char*)"");
            nodes.push_back(obj);
        }

        string pred_uri = msg.predicate_table[msg.predicates[i]];

        Agedge_t* edge = agedge(G, sub, obj);
        agsafeset(edge, (char*)"label", (char*)pred_uri.c_str(), (char*)"");
        agsafeset(edge, (char*)"penwidth",  (char*)"2", (char*)"");
        agsafeset(edge, (char*)"color",  (char*)"#0000ff", (char*)"");
        edges.push_back(edge);
    }

    return;
}

#include <stdio.h>

void rdfCallback(const rdf::RDFTriples::ConstPtr& msg) {
    Agraph_t* G = agopen("g", AGDIGRAPH);

    agsafeset(G, (char*)"dpi",  (char*)"60", (char*)"");

    vector<Agnode_t*> nodes;
    vector<Agedge_t*> edges;

    msgToGraph(*msg, G, nodes, edges);

    // compute the layout
    gvLayout(gvc, G, "dot");  // "neato"

    // render to file
    gvRenderFilename (gvc, G, "png", "/tmp/out.png");

    // load and show image using OpenCV
    cv::Mat image;
    image = cv::imread("/tmp/out.png", CV_LOAD_IMAGE_COLOR);
    cv::imshow("RDF Graph", image);
    cv::waitKey(3);

    /* Free layout data */
    gvFreeLayout(gvc, G);

    /* Free graph structures */
    agclose(G);
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "rdf_viewer");
    ros::NodeHandle nh;

    string topic = argv[1];

    /* set up a graphviz context */
    gvc = gvContext();

    // subscribing to a topic to listen to data
    ros::Subscriber sub = nh.subscribe(topic, 1, &rdfCallback);

    // spin
    ros::Rate r(10);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    sub.shutdown();

    /* close output file, free context, and return number of errors */
    return (gvFreeContext(gvc));
}
