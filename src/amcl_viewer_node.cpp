#define WITH_WAYPOINTS 1
#define WITH_NCURSES 1
#define WITH_NAV_GOAL 0

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <sys/ioctl.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_datatypes.h>
//#include <algorithm>
#include <cctype>
#include <clocale>

#if WITH_WAYPOINTS
#include <strands_navigation_msgs/TopologicalMap.h>
#include <strands_navigation_msgs/TopologicalRoute.h>
#include <strands_executive_msgs/TaskEvent.h>
#endif

#if WITH_NCURSES
#include <ncurses.h>
#endif

/* FOREGROUND */
#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST


/* BACKGROUND */
#define KKRED  "\x1B[41m"
#define KKGRN  "\x1B[42m"
#define KKYEL  "\x1B[43m"
#define KKBLU  "\x1B[44m"
#define KKMAG  "\x1B[45m"
#define KKCYN  "\x1B[46m"
#define KKWHT  "\x1B[47m"

#define BRED(x) KKRED x RST
#define BGRN(x) KKGRN x RST
#define BYEL(x) KKYEL x RST
#define BBLU(x) KKBLU x RST
#define BMAG(x) KKMAG x RST
#define BCYN(x) KKCYN x RST
#define BWHT(x) KKWHT x RST

/* BOLD, UNDERSCORE */
#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST

using namespace std;

class MapViewerNode {

public:

    ros::Subscriber pose_sub;
#if WITH_WAYPOINTS
    ros::Subscriber route_sub;
    ros::Subscriber exec_sub;
#endif

#if WITH_NAV_GOAL
    geometry_msgs::PoseStampedConstPtr pose;
#else
    geometry_msgs::PoseConstPtr pose;
#endif
    cv::Mat full_map;
    cv::Mat subsampled_map;
    costmap_2d::Costmap2D map_conv;
    int full_height;
    int full_width;
    int subsampled_height;
    int subsampled_width;
    int subsampled_direction;
    int subsampled_pose_x;
    int subsampled_pose_y;
    int old_subsampled_pose_x;
    int old_subsampled_pose_y;
    struct winsize last_w;

    double char_scale;

    vector<pair<string, geometry_msgs::Pose> > waypoints;
    vector<pair<string, geometry_msgs::Pose> > waypoints_short;
    string last_goal;
    int goal_x;
    int goal_y;
    int old_goal_x;
    int old_goal_y;
    string goal_action;
    int old_goal_string_size;

    string pointer_signs_str;

    MapViewerNode()
    {
        ros::NodeHandle n;

        ros::NodeHandle pn("~");
        string pose_input, map_input;
#if WITH_NAV_GOAL
        pn.param<std::string>("pose", pose_input, std::string("/move_base_simple/goal"));
#else
        pn.param<std::string>("pose", pose_input, std::string("/robot_pose"));
#endif
        pn.param<std::string>("map", map_input, std::string("/map"));
        char_scale = 2.0;
        pointer_signs_str = string("<v>^");
        goal_x = goal_y = -1;
        subsampled_pose_x = subsampled_pose_y = -1;
        old_goal_string_size = 0;

        pose_sub = n.subscribe(pose_input, 1, &MapViewerNode::pose_callback, this);

        nav_msgs::OccupancyGrid::ConstPtr global_costmap_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_input, n, ros::Duration(5));
        if (!global_costmap_msg) {
            ROS_ERROR("Could not get global costmap, exiting...");
            exit(0);
        }

#if WITH_WAYPOINTS
        //route_sub = n.subscribe("/topological_navigation/Route", 1, &MapViewerNode::route_callback, this);
        exec_sub = n.subscribe("/task_executor/events", 1, &MapViewerNode::exec_callback, this);
        strands_navigation_msgs::TopologicalMapConstPtr topo_map_msg = ros::topic::waitForMessage<strands_navigation_msgs::TopologicalMap>("/topological_map", n, ros::Duration(5));
        if (!topo_map_msg) {
            ROS_ERROR("Could not get topological map, skipping...");
        }
        else {
            save_waypoints(topo_map_msg);
        }
#endif

        save_map(global_costmap_msg);

#if WITH_NCURSES
        initscr();
        start_color();
        init_pair(1, COLOR_RED, COLOR_WHITE); // pointer foreground / background
        init_pair(2, COLOR_RED, COLOR_WHITE); // goal foreground / background
        init_pair(3, COLOR_CYAN, COLOR_CYAN); // occupied foreground / background
        init_pair(4, COLOR_WHITE, COLOR_WHITE); // free foreground / background
        init_pair(5, COLOR_BLUE, COLOR_WHITE); // waypoint foreground / background
        init_pair(6, COLOR_WHITE, COLOR_CYAN); // header foreground / background
#endif
    }

    virtual ~MapViewerNode()
    {
#if WITH_NCURSES
        endwin();
#endif
    }

#if WITH_NAV_GOAL
    void pose_callback(const geometry_msgs::PoseStampedConstPtr& pose_msg)
#else
    void pose_callback(const geometry_msgs::PoseConstPtr& pose_msg)
#endif
    {
        pose = pose_msg;
    }

#if WITH_WAYPOINTS
    void route_callback(const strands_navigation_msgs::TopologicalRouteConstPtr& route_msg)
    {
        last_goal = route_msg->nodes.back();
    }

    void exec_callback(const strands_executive_msgs::TaskEvent::ConstPtr& exec_msg)
    {
        if (exec_msg->event == strands_executive_msgs::TaskEvent::NAVIGATION_STARTED) {
            last_goal = exec_msg->task.start_node_id;
            goal_action = exec_msg->task.action;
        }
        //else if (exec_msg->event == strands_executive_msgs::TaskEvent::EXECUTION_FAILED)
    }

    void save_waypoints(strands_navigation_msgs::TopologicalMapConstPtr& topo_map_msg)
    {
        for (const strands_navigation_msgs::TopologicalNode& node : topo_map_msg->nodes) {
            string name = node.name;
            name.erase(std::remove_if(name.begin(), name.end(), [](char c){
                return std::islower(c);
            }), name.end());
            waypoints_short.push_back(make_pair(name, node.pose));
            waypoints.push_back(make_pair(node.name, node.pose));
        }
    }
#endif

    void save_map(nav_msgs::OccupancyGrid::ConstPtr& map)
    {
        int map_height = map->info.height;
        int map_width = map->info.width;
        double map_origin_x = map->info.origin.position.x;
        double map_origin_y = map->info.origin.position.y;
        double map_res = map->info.resolution;
        ROS_INFO_STREAM("Map width x height: " << map_width << " x " << map_height);
        map_conv = costmap_2d::Costmap2D(map_width, map_height, map_res, map_origin_x, map_origin_y);

        cv::Mat map_image = cv::Mat::zeros(map_height, map_width, CV_8UC1);

        // incoming dynamic map to Costmap2D
        for(int i = 0; i < map_width; i++) {
            for(int j = 0; j < map_height; j++) {
                map_image.at<uchar>(j, i) = map->data[map_conv.getIndex(i, j)];
            }
        }

        full_height = map_height;
        full_width = int(char_scale*double(map_width));
        cv::Size size(full_width, full_height);
        cv::resize(map_image, full_map, size);//resize image
    }

    tuple<int, int, int> pose_to_discrete_pose(const geometry_msgs::Pose& pose)
    {
        double x = pose.position.x;
        double y = pose.position.y;
        int map_x, map_y;

        map_conv.worldToMapEnforceBounds(x, y, map_x, map_y);

        int pose_x = int(double(subsampled_width)/double(full_width)*char_scale*double(map_x));
        int pose_y = int(double(subsampled_height)/double(full_height)*double(map_y));

        double roll, pitch, yaw;
        tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        yaw = fmod(yaw + 2.0*M_PI, 2.0*M_PI);
        int direction = int(round(4.0 * yaw / (2.0*M_PI))) % 4;

        return make_tuple(pose_x, pose_y, direction);
    }

    bool maybe_subsample_map()
    {
        struct winsize w;
#if WITH_NCURSES
        //initscr();
        refresh();
        getmaxyx(stdscr, w.ws_row, w.ws_col);
        //ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
        //endwin();
#else
        ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
#endif
        if (w.ws_col == last_w.ws_col && w.ws_row == last_w.ws_row) {
            return false;
        }
        last_w = w;

        int rows = w.ws_row;
        int cols = w.ws_col;

        if (double(cols)/double(full_width) < double(rows)/double(full_height)) {
            subsampled_width = cols;
            subsampled_height = int(double(full_height)*double(subsampled_width)/double(full_width));
        }
        else {
#if WITH_NCURSES
            subsampled_height = rows;
#else
            subsampled_height = rows - 1;
#endif
            subsampled_width = int(double(full_width)*double(subsampled_height)/double(full_height));
        }

        cv::Size size(subsampled_width, subsampled_height);//the dst image size,e.g.100x100
        cv::resize(full_map, subsampled_map, size);//resize image

        for (int r = 0; r < subsampled_height; ++r) {
            for (int c = 0; c < subsampled_width; ++c) {
                if (subsampled_map.at<uchar>(r, c) > 254) {
                    subsampled_map.at<uchar>(r, c) = 1; // occupied
                }
                else {
                    subsampled_map.at<uchar>(r, c) = 0; // not occupied
                }
            }
        }

        for (const pair<string, geometry_msgs::Pose>& wp : waypoints_short) {
            int pose_x, pose_y, direction;
            tie(pose_x, pose_y, direction) = pose_to_discrete_pose(wp.second);
            subsampled_map.at<char>(pose_y, pose_x) = '*';
            for (int x = pose_x - 1; x >= 0 && pose_x - x - 1 < wp.first.size(); --x) {
                subsampled_map.at<char>(pose_y, x) = wp.first[pose_x - x - 1];
            }
        }

        return true;
    }

    bool subsample_pose()
    {

        if (!pose) {
            return false;
        }

        old_subsampled_pose_x = subsampled_pose_x;
        old_subsampled_pose_y = subsampled_pose_y;
        int old_subsampled_direction = subsampled_direction;

#if WITH_NAV_GOAL
        tie(subsampled_pose_x, subsampled_pose_y, subsampled_direction) = pose_to_discrete_pose(pose->pose);
#else
        tie(subsampled_pose_x, subsampled_pose_y, subsampled_direction) = pose_to_discrete_pose(*pose);
#endif

        return (old_subsampled_direction != subsampled_direction ||
                old_subsampled_pose_x != subsampled_pose_x ||
                old_subsampled_pose_y != subsampled_pose_y);
    }

    bool subsample_goal()
    {
        if (last_goal.empty() || waypoints.empty()) {
            return false;
        }

        auto iter = std::find_if(waypoints.begin(), waypoints.end(), [&](const pair<string, geometry_msgs::Pose>& wp) {
            return wp.first == last_goal;
        });

        if (iter == waypoints.end()) {
            return false;
        }

        int direction;
        old_goal_x = goal_x;
        old_goal_y = goal_y;
        tie(goal_x, goal_y, direction) = pose_to_discrete_pose(iter->second);
        return (old_goal_x != goal_x ||
                old_goal_y != goal_y);
    }

    void draw()
    {
        bool did_update = maybe_subsample_map();
        bool did_move = subsample_pose();
        bool did_update_goal = subsample_goal();

        if (!did_update && !did_move && !did_update_goal) {
            return;
        }

        vector<string> pointer_signs = { BOLD(FRED(BWHT("<"))), BOLD(FRED(BWHT("v"))), BOLD(FRED(BWHT(">"))), BOLD(FRED(BWHT("^")))};

        cout << endl;
        int start_pos = 0;
        if (!last_goal.empty() && !goal_action.empty() && last_goal.size() + goal_action.size() + 2 < subsampled_width) {
            cout << KBLU << KKWHT << last_goal << ": " << goal_action << RST << RST;
            start_pos += last_goal.size() + goal_action.size() + 2;
        }
        for (int r = 0; r < subsampled_height; ++r) {
            for (int c = r == 0 ? start_pos : 0; c < subsampled_width; ++c) {
                int c_flip = subsampled_width-c-1;
                if (pose && c_flip == subsampled_pose_x && r == subsampled_pose_y) {
                    cout << pointer_signs[subsampled_direction];
                }
                else if (!last_goal.empty() && c_flip == goal_x && r == goal_y) {
                    cout << BOLD(FRED(BWHT("*")));
                }
                else if (subsampled_map.at<uchar>(r, c_flip) == 1) {
                    cout << BOLD(FBLU(BWHT("F")));
                }
                else if (subsampled_map.at<uchar>(r, c_flip) == 0) {
                    cout << BWHT(" ");
                }
                else {
                    cout << KBLU << KKWHT << subsampled_map.at<char>(r, c_flip) << RST << RST;
                }
            }
            cout << '\n';
        }

    }

#if WITH_NCURSES
    void draw_block(int r, int c)
    {
        int c_flip = subsampled_width-c-1;
        move(r, c);
        if (pose && c_flip == subsampled_pose_x && r == subsampled_pose_y) {
            attron(COLOR_PAIR(1));
            addch(pointer_signs_str[subsampled_direction]);
            attroff(COLOR_PAIR(1));
        }
        else if (!last_goal.empty() && c_flip == goal_x && r == goal_y) {
            attron(COLOR_PAIR(2));
            printw("*");
            attroff(COLOR_PAIR(2));
        }
        else if (subsampled_map.at<uchar>(r, c_flip) == 1) {
            attron(COLOR_PAIR(3));
            printw("O");
            attroff(COLOR_PAIR(3));
        }
        else if (subsampled_map.at<uchar>(r, c_flip) == 0) {
            attron(COLOR_PAIR(4));
            printw(" ");
            attroff(COLOR_PAIR(4));
        }
        else {
            attron(COLOR_PAIR(5));
            addch(subsampled_map.at<char>(r, c_flip));
            attroff(COLOR_PAIR(5));
        }
    }

    void draw_header()
    {
        string goal_header = last_goal + ": " + goal_action;
        if (!last_goal.empty() && !goal_action.empty() && goal_header.size() < subsampled_width) {
            move(0, 0);
            attron(COLOR_PAIR(6));
            printw(goal_header.c_str());
            attroff(COLOR_PAIR(6));
            for (int c = goal_header.size(); c < min(old_goal_string_size, subsampled_width); ++c) {
                draw_block(0, c);
            }
            old_goal_string_size = goal_header.size();
        }
    }

    void draw_ncurses()
    {
        bool did_update = maybe_subsample_map();
        bool did_move = subsample_pose();
        bool did_update_goal = subsample_goal();

        if (!did_update) {
            if (did_move) {
                draw_block(subsampled_pose_y, subsampled_width-subsampled_pose_x-1);
                if (old_subsampled_pose_x != -1 && old_subsampled_pose_y != -1) {
                    draw_block(old_subsampled_pose_y, subsampled_width-old_subsampled_pose_x-1);
                }
                refresh();
            }
            if (did_update_goal) {
                draw_block(goal_y, subsampled_width-goal_x-1);
                if (old_goal_x != -1 && old_goal_y != -1) {
                    draw_block(old_goal_y, subsampled_width-old_goal_x-1);
                }
                draw_header();
                refresh();
            }

            return;
        }

        clear();

        for (int r = 0; r < subsampled_height; ++r) {
            for (int c = 0; c < subsampled_width; ++c) {
                draw_block(r, c);
            }
        }
        old_goal_string_size = 0;
        draw_header();

        refresh();

    }
#endif

    void run()
    {
        ros::Rate rate(1.0);
        while (ros::ok()) {
            rate.sleep();
            ros::spinOnce();
            //draw();
            draw_ncurses();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "amcl_viewer_node");

    MapViewerNode mv;

    mv.run();

    return 0;

}
