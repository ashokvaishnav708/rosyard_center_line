#include "ros/ros.h"
#include "ros/rate.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "rosyard_common/Map.h"
#include <boost/bind.hpp>

bool map_calculated = false;
/*
//transform slam result pointcloud to Map msg
void mapCallback(rosyard_common::Map* map, const sensor_msgs::PointCloud2::ConstPtr& slam_map){

    if(map != nullptr && ! map_calculated){
    ///TODO calculate not only once
        map_calculated = true;

        sensor_msgs::PointCloud2 cone_cloud = *slam_map;

        //check colors of pcl points and save them to the corresponding lists
        sensor_msgs::PointCloud2Modifier pcl_modifier(cone_cloud);
        pcl_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

        sensor_msgs::PointCloud2Iterator<float> iter_x(cone_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cone_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cone_cloud, "z");

        sensor_msgs::PointCloud2Iterator<u_int8_t> iter_rgb(cone_cloud, "rgb");

        int num_of_cones = slam_map->width;

        for(int i = 0; i < num_of_cones; i++){
            geometry_msgs::Point p;
            p.x = *iter_x;
            p.y = *iter_y;
            p.z = *iter_z;

            rosyard_common::Cone cone;
            cone.position = p;

            if(iter_rgb[1] == 0){//blue
                cone.color = rosyard_common::Cone::BLUE;
                map->cone_blue.push_back(cone);
            }
            else if(iter_rgb[1] == 255){
                cone.color = rosyard_common::Cone::YELLOW;
                map->cone_yellow.push_back(cone);
            }
            else{
                cone.color = rosyard_common::Cone::ORANGE;
                map->cone_orange.push_back(cone);
            }
            //TODO handle none

            ++iter_rgb;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
        std::vector<geometry_msgs::Point> center_line;
        if(map->cone_blue.size() > 0 && map->cone_yellow.size() > 0){
            //determine if more yellow or blue cones are available
            bool more_yellow = (map->cone_blue.size() < map->cone_yellow.size());
            if(more_yellow){
                for(const auto &blue: map->cone_blue){
                    const auto it_yellow = std::min_element(map->cone_yellow.begin(), map->cone_yellow.end(),
                                                            [&](const rosyard_common::Cone &a, const rosyard_common::Cone &b) {
                        const double da = std::hypot(blue.position.x - a.position.x,
                                                     blue.position.y - a.position.y);
                        const double db = std::hypot(blue.position.x - b.position.x,
                                                     blue.position.y - b.position.y);

                        return da < db;
                    });
                    geometry_msgs::Point p;
                    p.x = static_cast<float>((blue.position.x + it_yellow->position.x) / 2.0);
                    p.y = static_cast<float>((blue.position.y + it_yellow->position.y) / 2.0);
                    p.z = 0.0;
                    center_line.push_back(p);
                }
            }
            else{
                for(const auto &yellow: map->cone_yellow){
                    const auto it_blue = std::min_element(map->cone_blue.begin(), map->cone_blue.end(),
                                                          [&](const rosyard_common::Cone &a, const rosyard_common::Cone &b) {
                        const double da = std::hypot(yellow.position.x - a.position.x,
                                                     yellow.position.y - a.position.y);
                        const double db = std::hypot(yellow.position.x - b.position.x,
                                                     yellow.position.y - b.position.y);

                        return da < db;
                    });
                    geometry_msgs::Point p;
                    p.x = static_cast<float>((yellow.position.x + it_blue->position.x) / 2.0);
                    p.y = static_cast<float>((yellow.position.y + it_blue->position.y) / 2.0);
                    p.z = 0.0;
                    center_line.push_back(p);
                }
            }
        }

        // Densify the center line
        const double precision = 0.2;
        for (unsigned int i = 1; i < center_line.size(); i++) {
            const double dx = center_line[i].x - center_line[i-1].x;
            const double dy = center_line[i].y - center_line[i-1].y;
            const double d  = std::hypot(dx, dy);

            const int nm_add_points = d/precision;
            for (unsigned int j = 0; j < nm_add_points; ++j) {
                geometry_msgs::Point new_p = center_line[i - 1];
                new_p.x += precision * j * dx / d;
                new_p.y += precision * j * dy / d;
                map->centerline.push_back(new_p);
            }
        }
    }
}
*/
int main(int argc, char **argv){
/*
    ros::init(argc, argv, "center_line_estimator");
    ros::NodeHandle n("~");

    // load configs parameter
    bool cheat_driver_active;
    n.getParam("/node_parameters/cheating_driver/activate", cheat_driver_active);
    std::string map_topic;
    if(cheat_driver_active){
        n.getParam("/nodes/gt_cone_topic_name",map_topic); // get ground truth
    }else{
        n.getParam("/nodes/slam_map_topic_name",map_topic); // get real data
    }
    std::string center_line_topic;
    n.getParam("/nodes/map_topic_name",center_line_topic);
    int updateRate;
    n.getParam("/nodes/map_rate",updateRate);

    //init subscriber
    rosyard_common::Map map;
    ros::Subscriber map_listener = n.subscribe<sensor_msgs::PointCloud2>(map_topic, 10, boost::bind(mapCallback, &map ,_1));

    //init publisher
    ros::Publisher map_publisher = n.advertise<rosyard_common::Map>(center_line_topic, 10);

    ros::Rate loop_rate(updateRate);

    bool published = false;
    while (ros::ok()){
        // if(!published){
            ros::spinOnce();
            map_publisher.publish(map);
            loop_rate.sleep();
        //     break;
        // }
    }
*/
}
