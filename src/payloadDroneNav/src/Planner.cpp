                                                                                /** 
                                                                                 * Fast Planner with Octomap and EDT3D library 
                                                                                **/
/**
 * Octomap generation using point cloud from depth camera and pose from PX4-EKF
 * Fixed Window (8,6,6) EDT calculation using EDT3D library
 * Planning in the window using kino-dynamic A* algorithm
 * Horizon limitation due to limitation of range sensor
 * Drone controlled using PX4-Offboard position control
**/

#include"payloadDroneNav/utils.h"

/** Planning and mapping headers **/
#include"payloadDroneNav/kinodynamic_astar.h"
#include"payloadDroneNav/bspline.h"
#include"payloadDroneNav/Map.h"

#include"std_msgs/Float64.h"

#include <math.h>

/** Drone states for planner **/
Eigen::Vector3d goalPose, currPose, startPose, startVel, startAcc, goalVel;

/** Overall trajectory **/
std::vector<Eigen::Vector3d> trajectory;
nav_msgs::Path generatedPath;
nav_msgs::Path splinePath;

/** time step to generate the trajectory **/
float deltaT = 0.50;

/** decision variables **/
bool goalReceived = false;
bool DESTINATION_REACHED = false;
bool PLAN = false;
int startOver = 0;

/** bspline parameters **/
float ctrlPtDist = 0.5;
float maxVel = 3.0;
float splineInterval = ctrlPtDist/maxVel;
int order = 3;

int count;     // count for planning iteration

#define INF 1000 // inifinity

/** Initialize the planner and mapping objects **/
fast_planner::KinodynamicAstar kAstar;
Map3D::OctoMapEDT costMap3D;
BSpline::BSpline bsplineDrone(splineInterval);
BSpline::BSpline bsplinePayload(splineInterval);

/** Cost Map visualization **/
visualization_msgs::MarkerArray costMap_vis;

/**********************************************************************************************************************************************************
 * -------------------------------------------------------------------Callbacks---------------------------------------------------------------------------*
***********************************************************************************************************************************************************/

/** octomap callback **/
void octomap_cb(const octomap_msgs::Octomap octo)
{
    costMap3D.new_tree = octomap_msgs::binaryMsgToMap(octo);  // this is the abstract tree for an octomap
    costMap3D.tree = dynamic_cast<octomap::OcTree*>(costMap3D.new_tree);

    //std::cout<<"final tree has "<<costMap3D.tree->getNumLeafNodes()<<" leaves"<<std::endl;

    // get the min and max of the map
    costMap3D.setMinMax();
    costMap3D.isOctomapUpdated = true;
}

/** current drone position callback **/
void local_pose_cb(const geometry_msgs::PoseStamped pose)
{
    currPose(0) = pose.pose.position.x;
    currPose(1) = pose.pose.position.y;
    currPose(2) = pose.pose.position.z;
}

/** get goal location **/
void goal_pose_cb(const geometry_msgs::PoseStamped pose)
{
    goalReceived = true;

    goalPose(0) = pose.pose.position.x;
    goalPose(1) = pose.pose.position.y;
    
    std::cout<<"Enter the height at the goal point ";
    std::cin>>goalPose(2);

    std::cout<<"Goal Pose is ... "<<goalPose.transpose()<<std::endl;
    std::cout<<"\n";
}

///////////////////////////////////////////////////////////////////
/**  Plan the path until goal is reached **/
void plan(ros::Publisher path,  ros::Publisher splinePub, ros::Publisher splinePubPayload, ros::Publisher map)
{
    while(!DESTINATION_REACHED || ros::ok()) /** until the goal is reached or the node is killed, keep running the process **/
    {
        if(!ros::ok())
        {
            break;
        }
        if(count == 0) // 1st iteration
        {   
            std::cout<<"Starting pose is "<<currPose<<std::endl;
            costMap3D.setStartPosition(currPose); // set this point as the point where map was initialized
            startPose = currPose;
            startPose(0) += 0.5;  // start planning from a small distance away from the starting location  
            PLAN = true;
            //count++;
        }
        else
        {
            if(costMap3D.ifUpdateMap(currPose))
            {
                costMap3D.setStartPosition(currPose);
                PLAN = true;
                //count++;
            }
            else
            {
                if(!ros::ok)
                {
                    break;
                }
               // std::cout<<"Waiting for map to update ..."<<std::endl;
                PLAN = false;
                ros::spinOnce();
                continue;
            }
        }


        if(PLAN)
        {    
            PLAN = false;

            // set the map range with respect to the current point
            costMap3D.setMapRange(currPose);

            std::cout<<"Updating the map from ... "<<costMap3D.start<<" to"<<costMap3D.end<<std::endl;

            // calculate EDT now
            DynamicEDTOctomap DistMap(5.0, costMap3D.tree, costMap3D.start, costMap3D.end, false); // take unknwon region as unoccupied
            DistMap.update();

            // set planning range and pass cost map to planner
            kAstar.init(costMap3D.start, costMap3D.end, currPose);
            kAstar.setEnvironment(&DistMap);

            // visualize the EDT Map
            costMap3D.getCostMapMarker(costMap_vis, &DistMap, map);

            // run the planner now (x is the status of the planner)
            int x;
            
            if(startOver == 1)
                {
                    x = kAstar.search(startPose, startVel, startAcc, goalPose, goalVel, true, false, 0.0);
                    startOver = 0;
                }                
            else
                {
                    x = kAstar.search(startPose, startVel, startAcc, goalPose, goalVel, false, false, 0.0);
                }
            
            
            std::cout<<"Planner output status is >>>>> "<<x<<std::endl;
            std::cout<<"\n";

            if(x==3)
                {
                    std::cout<<"No trajectory found ..."<<std::endl;
                    std::cout<<"Trying again ..."<<std::endl;
                    std::cout<<"\n";
                    ros::spinOnce();
                    return;
                }

            if(x==2)
                {
                    std::cout<<"+++++++++++__________________Goal reached__________________++++++++++++ ...."<<std::endl;
                    DESTINATION_REACHED = true;
                    //break;
                }
            
            /** get the planned path **/
            std::vector<Eigen::Vector3d> currTraj = kAstar.getKinoTraj(deltaT);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /*
               we got the trajectory generated for the drone
               now for the ideal trajectory of the payload -> just decrease the z coordinate by 1
            */
            std::vector<Eigen::Vector3d> payloadTraj;

            for(int i = 0; i<currTraj.size(); i++)
            {
                Eigen::Vector3d pt = currTraj.at(i);

                Eigen::Vector3d payloadPt;

                payloadPt(0) = pt(0);
                payloadPt(1) = pt(1);
                payloadPt(2) = pt(2) - 1;

                payloadTraj.push_back(payloadPt);
            }
            
            count++;

            std::cout<<"Trajectory length for payload and drone "<<payloadTraj.size()<<" "<<currTraj.size()<<std::endl;

            std::vector<Eigen::Vector3d> spTraj;
            std::vector<Eigen::Vector3d> spTrajPayload;

            if(currTraj.size() > 0)
            {
                /** generate bspline trajectory **/
                for(int i = 0; i<2; i++)
                {
                    currTraj.pop_back();
                    payloadTraj.pop_back();
                }
                
                bsplineDrone.setControlPoints(currTraj);
                bsplinePayload.setControlPoints(payloadTraj);

                // generate the bspline trajectory
                spTraj = bsplineDrone.getBSplineTrajectory();
                spTrajPayload = bsplinePayload.getBSplineTrajectory();
                
                std::cout<<" Returned bspline size (drone)   is " << spTraj.size()        << std::endl;   
                std::cout<<" Returned bspline size (payload) is " << spTrajPayload.size() << std::endl;         

                bsplineDrone.publishTrajectory(spTraj, splinePub);
                bsplinePayload.publishTrajectory(spTrajPayload, splinePubPayload);

            }
            
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            for(auto i = currTraj.begin(); i != currTraj.end(); i++)
            {
                geometry_msgs::PoseStamped p;
                Eigen::Vector3d pos = *i;
                Eigen::Vector3d pos_next;

                //std::cout<<"Waypoint in current trajectory ..."<<pos<<std::endl;

                if(-INF<pos(0)<INF && -INF<pos(1)<INF && -INF<pos(2)<INF)
                { 
                    p.pose.position.x = pos(0);
                    p.pose.position.y = pos(1);
                    p.pose.position.z = pos(2);

                if(i!=currTraj.end()-1)
                    {
                        pos_next = *(i+1);

                        float currYaw = atan2((pos_next(1) - pos(1)),(pos_next(0) - pos(0)));
                        float qz = sin(currYaw/2.0);
                        float qw = cos(currYaw/2.0);

                        p.pose.orientation.x = 0.0;
                        p.pose.orientation.y = 0.0;
                        p.pose.orientation.z = qz;
                        p.pose.orientation.w = qw;

                        generatedPath.header.stamp = ros::Time::now();
                        generatedPath.header.frame_id = "map";
                        generatedPath.poses.push_back(p); 
                    }
                }
                    ros::spinOnce();
            }
            
            path.publish(generatedPath);
            
            // insert this in the global trajectory
            trajectory.insert(trajectory.end(), spTraj.begin(), spTraj.end());
            
            std::cout<<"Global trajectory size ... "<<trajectory.size()<<std::endl;
            std::cout<<"\n";

            kAstar.reset();
            ros::spinOnce();

            if(!ros::ok() || x==2)
            {
                if(x==2)
                {
                    std::cout<<"***************** Reached goal ********************"<<std::endl;
                    return;
                }
                break;
            }

            // set the starting point for the next planning iteration
            auto it = trajectory.end() - 1;
            startPose = *it;
            std::cout<<">>>>>>>>>>>>>>__________Next starting point for the planner is "<<startPose.transpose()<<std::endl;
            std::cout<<"\n";

        }        
            ros::spinOnce();

            if(!ros::ok())
            {
                break;
            }
    }
}

int main(int argc, char **argv)
{
    /** set all the subscribers and publishers **/
    ros::init(argc, argv, "fast_planner_octomap");
    ros::NodeHandle n;

    /** Subscribers **/
    ros::Subscriber oct             = n.subscribe<octomap_msgs::Octomap>("/octomap_binary",1,octomap_cb);
    ros::Subscriber pos             = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,local_pose_cb);
    ros::Subscriber goal            = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,goal_pose_cb);
 
    /** Publishers **/
    ros::Publisher path             = n.advertise<nav_msgs::Path>("/fastPlanner_path",1); 
    ros::Publisher splinePub        = n.advertise<nav_msgs::Path>("/fastPlanner_spline",1);
    ros::Publisher splinePubPayload = n.advertise<nav_msgs::Path>("/fastPlanner_payload_spline",1);
    ros::Publisher map              = n.advertise<visualization_msgs::MarkerArray>("/costMap_marker_array",1); // one at a time

    ros::Rate rate(20);

    std::cout<<"Start over ...? ";
    std::cin>>startOver;

    while(!goalReceived)
    {
        std::cout<<"Waiting for goal ..."<<std::endl;
        ros::spinOnce();
        rate.sleep();

        if(!ros::ok())
        {
            break;
        }
    }

    if(goalReceived)
    {
        std::cout<<"Goal received "<<goalPose.transpose()<<std::endl;
        std::cout<<"\n";
        std::cout<<"Setting initial velocity and acceleration .."<<std::endl;

        goalVel  = Eigen::Vector3d::Zero();  // velocity at goal location set to 0
        startVel = Eigen::Vector3d::Zero();  // starting with 0 initial velocity i.e. static 
        startAcc = Eigen::Vector3d::Ones();  // set the starting acceleration as (1,1,1)

        bsplineDrone.setOrder(order);
        bsplinePayload.setOrder(order);
        
        std::cout<<"Starting planning now ..."<<std::endl;

        kAstar.setParam(n); // set the fast planner parameters

        plan(path, splinePub, splinePubPayload, map);
    }

    return 0;
}
