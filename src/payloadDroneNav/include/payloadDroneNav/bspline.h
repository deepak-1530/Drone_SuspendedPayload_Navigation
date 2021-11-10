// bspline header file

#include<iostream>
#include<math.h>
#include<cmath>
#include<vector>
#include<Eigen/Dense>

#include<fstream>
#include<cstdlib>

#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>

namespace BSpline
{
class BSpline
{
    public:
        int order=3;
        int numCtrlPoints, knotSize, numSegments;
        nav_msgs::Path splinePath;
        std::vector<float> knotVector;
        std::vector<Eigen::Vector3d> ctrlPoints;
        std::vector<Eigen::Vector3d> splineTrajectory;             // this contains all the points in the spline for all the control points
        std::vector<std::vector<Eigen::Vector3d> > splineSegments; // spline trajectory of each segment
        BSpline(float interval_);                                                 // constructor
        void setOrder(int _order_);
        void setKnotVector();
        void setNumSegments();
        std::vector<Eigen::Vector3d> getBSplineTrajectory();
        void setControlPoints(std::vector<Eigen::Vector3d> _ctrlPoints_);
        float interval;
        void publishTrajectory(std::vector<Eigen::Vector3d>, ros::Publisher);

    private:
        double coxDeBoorBasis(int i, int k, float u); // this is the spline basis function --> using recursive cox-deboor equation to get bspline basis function

};  
} // namesapce

/////////////////////////////////////////////////////////////////////////
/** constructor **/
BSpline::BSpline::BSpline(float interval_)
{
    order = 5;
    std::cout<<"Spline initialized \n";
    interval = 0.2;
}

//////////////////////////////////////////////////////////////////////////
/** set control points for which spline is to be calculated **/
void BSpline::BSpline::setControlPoints(std::vector<Eigen::Vector3d> _ctrlPoints_)
{

    // clear the vectors first to ensure previous points are not in the vector
    knotVector.clear();
    splineTrajectory.clear();
    splineSegments.clear();
    ctrlPoints.clear();

    ctrlPoints = _ctrlPoints_;
    numCtrlPoints = _ctrlPoints_.size()-1;

    std::cout<<"No. of control points are "<<numCtrlPoints<<std::endl;

    this->setKnotVector();
    this->setNumSegments();
}

//////////////////////////////////////////////////////////////////////////
/** set order of spline **/
void BSpline::BSpline::setOrder(int _order_)
{
    order = _order_;
}

//////////////////////////////////////////////////////////////////////////
/** get the knot vectors **/
void BSpline::BSpline::setKnotVector()
{
    knotSize = order + numCtrlPoints + 1;  // m = n + p + 1

    std::cout<<"knot vector size is "<<knotSize<<std::endl;

    /**
     *  use the bspline theory
     *  t[i] = 0 if i<k
     *  t[i] = i-k+1 if k<=i<n
     *  t[i] = n-k+2 if i >= n
    **/
    float count  = 0;
    for(int i = 0; i<knotSize; i++)
    {
        int m = i;

        if(i<order)
        {
            knotVector.push_back(0);
        }

        else if(i>=order && i<=numCtrlPoints)
        {
            count += interval;
            knotVector.push_back(count);
        }

        else  if(i > numCtrlPoints)
        {
            //count += 0.25;
            knotVector.push_back(count+interval);//numCtrlPoints - order + 2.0);
        }

        std::cout<<knotVector[i]<<std::endl;
        
    }

}

///////////////////////////////////////////////////////////////////////////
/** set the number of segments in the spline **/
void BSpline::BSpline::setNumSegments()
{
    numSegments = numCtrlPoints - order + 2;

    std::cout<<"Number of segments are "<<numSegments<<std::endl;
}

///////////////////////////////////////////////////////////////////////////
/** calculate the bsplines given the control points and the knot vectors **/
std::vector<Eigen::Vector3d> BSpline::BSpline::getBSplineTrajectory()
{
    /** spline for each segment **/
    std::vector<Eigen::Vector3d> splineSegment;

    /** create segment-wise splines **/
    for(int i = 0; i<numSegments; i++)
    {    
        float t_s = knotVector[int(order) - 1 + i];
        
        /** now for each segment take the control points (each segment is affected by 
         * "order" no. of points i.e. here each segment is affected by 4 points) **/
        for(float t=t_s; t<t_s + interval; t=t+0.01) 
        {
            Eigen::Vector3d pt(0,0,0);

            for(int j = i; j<i+order; j++)
                {
                    pt += coxDeBoorBasis(j,order,t)*ctrlPoints[j];
                }

                splineSegment.push_back(pt);
                splineTrajectory.push_back(pt);
        }   

        splineSegments.push_back(splineSegment);
    }

    return splineSegment;

}

//////////////////////////////////////////////////////////////////////////////
/** calculate the spline coefficients using cox-deBoor recursive equations **/
double BSpline::BSpline::coxDeBoorBasis(int i, int k, float t)
{   
    if(k==1)
    {   
        if (t==knotVector[i] && t==knotVector[i + 1] && i>=order-1 && i<=knotSize-order)
            {
                return 1;
            } 
        else if (t >= knotVector[i] && t < knotVector[i + 1] && i>=order-1 && i<=knotSize-order)
            {
                return 1;
            }
        else
            {
                return 0;
            }
    }

    float delT1 = knotVector[i+k-1] - knotVector[i];
    float delT2 = knotVector[i+k] - knotVector[i+1];

    float C1, C2;

    if(delT1==0)
    {
        C1 = 0;
    }
    else
    {
        C1 = ((t - knotVector[i])*coxDeBoorBasis(i,k-1,t))/delT1;
    }

    if (delT2 == 0)
    {
        C2 = 0;
    }
    else
    {
        C2 = ((knotVector[i+k] - t)*coxDeBoorBasis(i+1, k-1, t))/delT2;
    }
    
    return C1 + C2;
}   

///////////////////////////////////////////////////////////////////////////
/* Publish the bspline trajectory generated */
void BSpline::BSpline::publishTrajectory(std::vector<Eigen::Vector3d> splineTraj, ros::Publisher splinePub)
{
        float INF = 1000;

        for(auto i = splineTraj.begin(); i!=splineTraj.end(); i++)
            {
               geometry_msgs::PoseStamped p;
               Eigen::Vector3d pos = *i; 
               Eigen::Vector3d pos_next;

               //std::cout<<"Waypoint in spline "<<pos.transpose()<<std::endl;

                if(-INF<pos(0)<INF && -INF<pos(1)<INF && -INF<pos(2)<INF)
                { 
                    p.pose.position.x = pos(0);
                    p.pose.position.y = pos(1);
                    p.pose.position.z = pos(2);

                if(i!=splineTraj.end()-1)
                    {
                        pos_next = *(i+1);

                        float currYaw = atan2((pos_next(1) - pos(1)),(pos_next(0) - pos(0)));
                        float qz = sin(currYaw/2.0);
                        float qw = cos(currYaw/2.0);

                        p.pose.orientation.x = 0.0;
                        p.pose.orientation.y = 0.0;
                        p.pose.orientation.z = qz;
                        p.pose.orientation.w = qw;

                        splinePath.header.stamp = ros::Time::now();
                        splinePath.header.frame_id = "map";
                        splinePath.poses.push_back(p); 
                    }
                }
                    ros::spinOnce();

            }

            splinePub.publish(splinePath);
            std::cout<<"Published spline for the current path "<<std::endl;


}