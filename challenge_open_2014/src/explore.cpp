#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <tf/transform_datatypes.h>

double robot_inscribed_radius = 0.3;

struct FrontierPoint{
  int idx;     //position
  tf::Vector3 d; //direction

  FrontierPoint(int idx_, const tf::Vector3& d_) : idx(idx_), d(d_) {}
};

struct Frontier {
  geometry_msgs::Pose pose;
  int size;
  Frontier():pose(),size(0) {}
  Frontier(const Frontier& copy) { pose = copy.pose; size = copy.size; }
};

void findFrontiers(const nav_msgs::OccupancyGrid& occ_grid, std::vector<Frontier>& frontiers) {
    frontiers.clear();

    int idx;
    int w = occ_grid.info.width;
    int h = occ_grid.info.height;
    int size = (w * h);

    std::vector<bool> frontier_map(size, false);

    // Find all frontier cells (free cells next to unknown cells).
    const std::vector<signed char>& map = occ_grid.data;
    for (idx = 0; idx < size; idx++) {
        bool free_cell = (map[idx] < 50);

        if (free_cell &&
                (   (idx + 1 < size && map[idx + 1] < 0)
                 || (idx - 1 >= 0 && map[idx - 1] < 0)
                 || (idx + w < size && map[idx + w] < 0)
                 || (idx - w >= 0 && map[idx - w] < 0))) {
            frontier_map[idx] = true;
        }
    }

    // Clean up frontiers detected on separate rows of the map
    idx = h - 1;
    for (int y = 0; y < w; y++) {
        frontier_map[idx] = false;
        idx += h;
    }

    // Group adjoining map_ pixels
    int segment_id = 127;
    std::vector< std::vector<FrontierPoint> > segments;
    for (int i = 0; i < size; i++) {
        if (frontier_map[i]) {
            std::vector<int> neighbors;
            std::vector<FrontierPoint> segment;
            neighbors.push_back(i);

            // claim all neighbors
            while (neighbors.size() > 0) {
                int idx = neighbors.back();
                neighbors.pop_back();

                // make sure this frontier cell is not visited again
                frontier_map[idx] = false;

                tf::Vector3 dir(0, 0, 0);
                int c = 0;
                if (idx + 1 < size && map[idx+1] < 0) {
                    dir += tf::Vector3(1, 0, 0);
                    c++;
                }
                if (idx-1 >= 0 && map[idx-1] < 0) {
                    dir += tf::Vector3(-1, 0, 0);
                    c++;
                }
                if (idx+w < size && map[idx+w] < 0) {
                    dir += tf::Vector3(0, 1, 0);
                    c++;
                }
                if (idx-w >= 0 && map[idx-w] < 0) {
                    dir += tf::Vector3(0, -1, 0);
                    c++;
                }

                segment.push_back(FrontierPoint(idx, dir / c));

                // consider 8 neighborhood
                if (idx - 1 > 0 && frontier_map[idx - 1])
                    neighbors.push_back(idx - 1);

                if (idx + 1 < size && frontier_map[idx + 1])
                    neighbors.push_back(idx + 1);

                if (idx - w > 0 && frontier_map[idx - w])
                    neighbors.push_back(idx - w);

                if (idx - w + 1 > 0 && frontier_map[idx-w+1])
                    neighbors.push_back(idx - w + 1);

                if (idx - w - 1 > 0 && frontier_map[idx - w - 1])
                    neighbors.push_back(idx-w-1);

                if (idx + w < size && frontier_map[idx + w])
                    neighbors.push_back(idx+w);

                if (idx + w + 1 < size && frontier_map[idx + w + 1])
                    neighbors.push_back(idx + w + 1);

                if (idx + w - 1 < size && frontier_map[idx + w - 1])
                    neighbors.push_back(idx + w - 1);
            }

            segments.push_back(segment);
            segment_id--;
            if (segment_id < -127)
                break;
        }
    }

    int num_segments = 127 - segment_id;
    if (num_segments <= 0)
        return;

    for (unsigned int i=0; i < segments.size(); i++) {
        Frontier frontier;
        std::vector<FrontierPoint>& segment = segments[i];
        uint size = segment.size();
        //we want to make sure that the frontier is big enough for the robot to fit through
        if (size * occ_grid.info.resolution < robot_inscribed_radius)
            continue;

        float x = 0, y = 0;
        tf::Vector3 d(0,0,0);

        for (uint j=0; j<size; j++) {
            d += segment[j].d;
            int idx = segment[j].idx;
            x += (idx % w);
            y += (idx / w);
        }
        d = d / size;
        frontier.pose.position.x = occ_grid.info.origin.position.x + occ_grid.info.resolution * (x / size);
        frontier.pose.position.y = occ_grid.info.origin.position.y + occ_grid.info.resolution * (y / size);
        frontier.pose.position.z = 0.0;

        frontier.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(d.y(), d.x()));
        frontier.size = size;

        frontiers.push_back(frontier);
    }
}

void visualizeFrontiers(const std::vector<Frontier>& frontiers) {
    for(std::vector<Frontier>::const_iterator it = frontiers.begin(); it != frontiers.end(); ++it) {
        const Frontier& f = *it;
        std::cout << f.pose << std::endl;
    }
}

void foo(const nav_msgs::OccupancyGrid& msg) {
    std::vector<Frontier> frontiers;
    findFrontiers(msg, frontiers);
    visualizeFrontiers(frontiers);
}

void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    foo(*msg);
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/map", 10, &callback);

    // subscribing to gmapping service (get map)
    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("/dynamic_map");

    // create service request
    nav_msgs::GetMap srv;

    // populate service request
    // srv.request.field1 = ...
    // ...

    // call the service
    if (client.call(srv)) {
        foo(srv.response.map);
    } else {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    // spin
    ros::spin();

    return 0;
}

