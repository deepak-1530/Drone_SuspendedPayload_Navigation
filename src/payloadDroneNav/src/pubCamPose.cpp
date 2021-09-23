
/**
 * Broadcast the transformation from base_link to the map frame
 * Then get the point cloud in the map frame
 * Use this TF for octomap and costMap generation
**/
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_broadcaster.h> // tf data is published using transform broadcaster


void getPose(const geometry_msgs::PoseStamped msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
    tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    transform.setRotation(q); // set rotation from the current frame to the base frame
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map","base_link")); // transform
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "drone_tf_publisher");
    ros::NodeHandle n;
    ros::Subscriber tf_subscriber = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, getPose);
    ros::spin();

    return 0;
}