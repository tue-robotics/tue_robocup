#include <ros/ros.h>

#include <graphviz/gvc.h>

#include <cv.h>
#include <highgui.h>

#include <psi/Client.h>

#include <iostream>
#include <list>
#include <set>

using namespace std;

GVC_t *gvc;

string headToLabel(psi::Term t) {
    stringstream s;
    if (t.getSize() == 0) {
        s << t;
    } else {
        s << t.getFunctor();
        s << "/" << t.getSize();
    }
    return s.str();
}

Agnode_t* findOrAddState(Agraph_t* G, map<string, Agnode_t*>& states, vector<Agnode_t*>& nodes, const string& state_name, const set<string>& current_states) {
    Agnode_t* node = 0;
    map<string, Agnode_t*>::iterator it_state = states.find(state_name);
    if (it_state == states.end()) {
        node = agnode(G,  (char*)state_name.c_str());
        agsafeset(node, (char*)"label", (char*)state_name.c_str(), (char*)"");
        agsafeset(node, (char*)"style", (char*)"filled", (char*)"");

        if (current_states.find(state_name) != current_states.end()) {
            agsafeset(node, (char*)"color", (char*)"red", (char*)"");
        } else {
            agsafeset(node, (char*)"color", (char*)"skyblue", (char*)"");
        }


        nodes.push_back(node);
        states[state_name] = node;
    } else {
        node = it_state->second;
    }
    return node;
}

void transitionsToGraph(const psi::Term& transitions, Agraph_t* G, vector<Agnode_t*>& nodes, vector<Agedge_t*>& edges, const set<string>& current_states) {

    map<string, Agnode_t*> states;

    for(unsigned int i = 0 ; i < transitions.getSize(); ++i) {
        psi::Term trans = transitions.get(i);
        if (!trans.get(0).isVariable()) {

            string machine = headToLabel(trans.get(0));
            string state1 = machine + ":" + headToLabel(trans.get(1));
            string state2 = machine + ":" + headToLabel(trans.get(2));

            Agnode_t* node1 = findOrAddState(G, states, nodes, state1, current_states);
            Agnode_t* node2 = findOrAddState(G, states, nodes, state2, current_states);

            Agedge_t* edge = agedge(G, node1, node2);
            //agsafeset(edge, (char*)"label", (char*)pred_uri.c_str(), (char*)"");
            agsafeset(edge, (char*)"penwidth",  (char*)"2", (char*)"");
            agsafeset(edge, (char*)"color",  (char*)"#0000ff", (char*)"");
            edges.push_back(edge);
        }
    }

}

#include <stdio.h>

void showTransitions(const psi::Term& transitions, const set<string>& current_states) {
    Agraph_t* G = agopen("g", AGDIGRAPH);

    agsafeset(G, (char*)"dpi",  (char*)"60", (char*)"");

    vector<Agnode_t*> nodes;
    vector<Agedge_t*> edges;

    transitionsToGraph(transitions, G, nodes, edges, current_states);

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

    psi::Client client("/reasoner/query", "/reasoner/assert");

    /* set up a graphviz context */
    gvc = gvContext();

    // spin
    ros::Rate r(1);
    while(ros::ok()) {
        vector<psi::BindingSet> state_res = client.query(psi::Compound("current_state",
                                                                        psi::Variable("Machine"),
                                                                        psi::Variable("State"),
                                                                        psi::Variable("Priority")));

        set<string> current_states;

        for(unsigned int i = 0; i < state_res.size(); ++i) {
            string machine = headToLabel(state_res[i].get(psi::Variable("Machine")));
            string state = machine + ":" + headToLabel(state_res[i].get(psi::Variable("State")));
            current_states.insert(state);
        }

        vector<psi::BindingSet> results = client.query(psi::Compound("list_transitions", psi::Variable("T")));
        if (!results.empty()) {
            psi::Term transitions = results[0].get(psi::Variable("T"));
            showTransitions(transitions, current_states);
        }

        r.sleep();
    }

    return 0;
}
