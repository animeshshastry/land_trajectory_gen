#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

geometry_msgs::PoseStamped pub_msg;

double quad_position[3];
double q[4];
double position_des[3];
double loiter_alt;
double R0 = 1.0;
bool first_call = true;
double yaw_des,pitch_des=0.0,roll_des=0.0;
double quad_yaw;

void yaw_callback(const geometry_msgs::Vector3 & msg)
{
    yaw_des = quad_yaw + msg.x;
}

void current_pose_callback(const geometry_msgs::PoseStamped & msg)
{
        quad_position[0]=msg.pose.position.x;
        quad_position[1]=msg.pose.position.y;
        quad_position[2]=msg.pose.position.z;

        q[0]=msg.pose.orientation.w;
        q[1]=msg.pose.orientation.x;
        q[2]=msg.pose.orientation.y;
        q[3]=msg.pose.orientation.z;

        quad_yaw = atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));

        if (first_call) {
            loiter_alt = quad_position[2];
            first_call = false;
        }
}

void Generate_Trajectory()
{
    double x=quad_position[0];
    double y=quad_position[1];
    double z=quad_position[2];
    double psi = atan2(y,x);
    if (( x*x + y*y ) > ( R0 * R0 * exp( 2*z ) )){
        //const. vel. home-in at const. altitude
        if (x>0) position_des[0] = x - 0.5*cos(psi);
        else position_des[0] = x + 0.5*cos(psi);

        if (y>0) position_des[1] = y - 0.5*sin(psi);
        else position_des[1] = y + 0.5*sin(psi);

        position_des[2] = loiter_alt;
    }
    else if (( x*x + y*y ) > 1.0*1.0){
        //const. vel. home-in and descend
        if (x>0) position_des[0] = x - 0.3*cos(psi);
        else position_des[0] = x + 0.3*cos(psi);

        if (y>0) position_des[1] = y - 0.3*sin(psi);
        else position_des[1] = y + 0.3*sin(psi);

        position_des[2] = z - 0.1;
        if (position_des[2] < 1.0) position_des[2] = 1.0;
        loiter_alt = z;
    }
    else if (( x*x + y*y ) > 0.15*0.15){
        //const. vel. home-in and descend
        if (x>0) position_des[0] = x - 0.2*cos(psi);
        else position_des[0] = x + 0.2*cos(psi);

        if (y>0) position_des[1] = y - 0.2*sin(psi);
        else position_des[1] = y + 0.2*sin(psi);

        position_des[2] = z - 0.1;
        if (position_des[2] < 0.6) position_des[2] = 0.6;
        loiter_alt = z;
    }
    else if (z > 0.5){
        //strong home-in and const. vel. descend
        position_des[0] = 0.0;
        position_des[1] = 0.0;
        position_des[2] = z - 0.1;
        if (position_des[2] < 0.49) position_des[2] = 0.49;
        loiter_alt = z;
    }
    else {
        //strong descend and forget home-in
        position_des[0] = x;
        position_des[1] = y;
        position_des[2] = 0.0;
    }

}

int main(int argc, char **argv)
{
        ros::init(argc,argv,"land_trajectory_gen");
	ros::NodeHandle nh;

        ros::Publisher pub_set_pos = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 100);
        ros::Subscriber sub_pose = nh.subscribe("mavros/local_position/pose", 1000, current_pose_callback);
        ros::Subscriber sub_yaw = nh.subscribe("gimbal_asetp", 1000, yaw_callback);

	ros::Rate loop_rate(100);
   	ros::spinOnce();
	
	int count = 1;

 	while (ros::ok()){

            Generate_Trajectory();

            pub_msg.header.stamp = ros::Time::now();
            pub_msg.header.seq=count;
            pub_msg.header.frame_id = 1;
            pub_msg.pose.position.x = position_des[0];
            pub_msg.pose.position.y = position_des[1];
            pub_msg.pose.position.z = position_des[2];

            double cy = cos(yaw_des * 0.5);
            double sy = sin(yaw_des * 0.5);
            double cr = cos(roll_des * 0.5);
            double sr = sin(roll_des * 0.5);
            double cp = cos(pitch_des * 0.5);
            double sp = sin(pitch_des * 0.5);

            pub_msg.pose.orientation.x = cy * sr * cp - sy * cr * sp;
            pub_msg.pose.orientation.y = cy * cr * sp + sy * sr * cp;
            pub_msg.pose.orientation.z = sy * cr * cp - cy * sr * sp;
            pub_msg.pose.orientation.w = cy * cr * cp + sy * sr * sp;

            pub_set_pos.publish(pub_msg);
            ros::spinOnce();
            count++;
            loop_rate.sleep();
   	}

	return 0;
}

