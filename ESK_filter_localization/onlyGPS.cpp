#include "adas/canbus/mobileyeq4/eyeq4_obstacles_preprocessing.h"
#include "adas/canbus/mobileyeq4/mobileyeq4_driver.h"
#include "adas/canbus/proto/mobileye_eyeQ4_config.pb.h"
#include "adas/common/defines/defines.h"
#include "adas/common/util/coord.h"
#include "adas/common/util/utils.h"
#include <adas_dds/common/double_buffer.h>

#include "adas_dds/include/adas_dds.h"
#include "adas_dds/include/adas_dds_topics.h"
#include "adas_dds/include/adas_dds_frame.h"
#include "adas_dds/convertor/adas_dds_convertor_EyeQ4.h"
#include "adas_dds/convertor/adas_dds_convertor_model.h"
#include "adas_dds/convertor/adas_dds_convertor_lane.h"
#include "adas_dds/convertor/adas_dds_convertor_obstacle.h"
#include "adas_dds/convertor/adas_dds_convertor_basic.h"
#include "adas_dds/convertor/adas_dds_convertor_model.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <json/json.h>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <vector> 




#define MAXLINE 1500 
using namespace adas;
using namespace std;
using namespace Eigen;

int count_GPS = 0, count_IMU = 0, count_gt_pos_vel = 0;

/*************************************************************************/
class NodePublisher {
   public:
    explicit NodePublisher(const adas_dds::NodeHandle& ns) : n_(ns) {
        Pose_pub = n_.advertise<nav_msgs::Odometry>("/adas/localization/pose", 10);
        IMU_odom_predict = n_.advertise<nav_msgs::Odometry>("/adas/localization/IMU_odom", 10);

        IMU_data_check = n_.advertise<sensor_msgs::Imu>("/adas/sensor_check/IMU", 10);
        GPS_Odom_check = n_.advertise<nav_msgs::Odometry>("/adas/sennsor_check/GPS_odom", 10);
    }

    void publish_odom(nav_msgs::Odometry& Odom){
        Pose_pub.publish(Odom);
    }

    void publish_IMU_odom(nav_msgs::Odometry& IMU_odom){
        IMU_odom_predict.publish(IMU_odom);
    }

    void publish_IMU_data_check(sensor_msgs::Imu& IMU_data){
        IMU_data_check.publish(IMU_data);
    }

    void publish_GPS_odom_check(nav_msgs::Odometry& GPS_odom){
        GPS_Odom_check.publish(GPS_odom);
    }
   private:
    adas_dds::NodeHandle n_;
    adas_dds::Publisher Pose_pub;
    adas_dds::Publisher IMU_odom_predict;
    adas_dds::Publisher IMU_data_check;
    adas_dds::Publisher GPS_Odom_check;
};

class NodeSubscriber {
   public:
    NodeSubscriber(const adas_dds::NodeHandle& ns) : n_(ns) {
        IMU_data =
            n_.subscribe<sensor_msgs::Imu>("/vehicle/imu/data_raw", 5,
                         std::bind(&NodeSubscriber::IMU_subscriber_callback, this, std::placeholders::_1));

        GPS_odom_data =
            n_.subscribe<nav_msgs::Odometry>("/vehicle/gps/odometry", 5,
                         std::bind(&NodeSubscriber::GPS_odom_subscriber_callback, this, std::placeholders::_1));
        Gound_truth_pos_vel =
            n_.subscribe<nav_msgs::Odometry>("/vehicle/groundtruth/gt_pose_vel", 5,
                         std::bind(&NodeSubscriber::Ground_truth_posvel_subscriber_callback, this, std::placeholders::_1));
        Gound_truth_acc =
            n_.subscribe<geometry_msgs::Vector3>("/vehicle/groundtruth/gt_linear_acc", 5,
                         std::bind(&NodeSubscriber::Ground_truth_acc_subscriber_callback, this, std::placeholders::_1));                 
    }
    void IMU_subscriber_callback(const sensor_msgs::Imu& msg) { count_IMU++; IMU_raw_data = msg; }
    void GPS_odom_subscriber_callback(const nav_msgs::Odometry& msg) { count_GPS++; GPS_odom = msg; }
    void Ground_truth_posvel_subscriber_callback(const nav_msgs::Odometry& msg) { count_gt_pos_vel++; gt_pos_vel = msg; }
    void Ground_truth_acc_subscriber_callback( geometry_msgs::Vector3& msg) { gt_acc = msg; }

    inline sensor_msgs::Imu get_IMU_data() const {
        return IMU_raw_data;
    }
    inline nav_msgs::Odometry get_GPS_Odometry() const {
        return GPS_odom;
    }
    inline nav_msgs::Odometry get_gt_pos_vel() const {
        return gt_pos_vel;
    }
    inline geometry_msgs::Vector3 get_gt_acc() const {
        return gt_acc;
    }

    private:
    adas_dds::NodeHandle n_;
    // subscriber
    adas_dds::Subscriber IMU_data;
    adas_dds::Subscriber GPS_odom_data;
    adas_dds::Subscriber Gound_truth_pos_vel;
    adas_dds::Subscriber Gound_truth_acc;
    
    sensor_msgs::Imu IMU_raw_data;
    geometry_msgs::Vector3 gt_acc;
    nav_msgs::Odometry GPS_odom;
    nav_msgs::Odometry gt_pos_vel;
};

/****************************************************/
//Function
Eigen::Matrix3d skew_symetric_matrix(Eigen::Vector3d v){
	Eigen::Matrix3d A;              A.setZero();
    double x = v(0);
    double y = v(1);
    double z = v(2);	
    A(0,0) = 0;     A(0,1) = -z;    A(0,2) = y;
    A(1,0) = z;     A(1,1) = 0;     A(1,2) = -x;
    A(2,0) = -y;    A(2,1) = x;     A(2,2) = 0;
	return A;
}

Eigen::Matrix3d rotational_matrix(Eigen::Quaterniond Quaternion_input){
    double q0 = Quaternion_input.w();
    double q1 = Quaternion_input.x();
    double q2 = Quaternion_input.y();
    double q3 = Quaternion_input.z();

    Matrix3d A;     A.setZero();

    A(0,0) = q0*q0 + q1*q1 - q2*q2 - q3*q3,    A(0,1) = 2*(q1*q2 - q0*q3),               A(0,2) = 2*(q1*q3 + q0*q2);
    A(1,0) = 2*(q1*q2 + q0*q3),                A(1,1) = q0*q0 - q1*q1 + q2*q2 - q3*q3,   A(1,2) = 2*(q2*q3 - q0*q1);
    A(2,0) = 2*(q1*q3 - q0*q2),                A(2,1) = 2*(q2*q3 + q0*q1),               A(2,2) = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
    return A;
}

Eigen::Matrix3d rotational_matrix_inverse(Eigen::Quaterniond Quaternion_input){
    double q0 = Quaternion_input.w();
    double q1 = Quaternion_input.x();
    double q2 = Quaternion_input.y();
    double q3 = Quaternion_input.z();

    Matrix3d A;     A.setZero();

    A(0,0) = q0*q0 + q1*q1 - q2*q2 - q3*q3,    A(0,1) = 2*(q1*q2 - q0*q3),               A(0,2) = 2*(q1*q3 + q0*q2);
    A(1,0) = 2*(q1*q2 + q0*q3),                A(1,1) = q0*q0 - q1*q1 + q2*q2 - q3*q3,   A(1,2) = 2*(q2*q3 - q0*q1);
    A(2,0) = 2*(q1*q3 - q0*q2),                A(2,1) = 2*(q2*q3 + q0*q1),               A(2,2) = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
    return A.inverse();
}

Eigen::Quaterniond Euler_to_Quat(Eigen::Vector3d Euler_ang){
    Eigen::Quaterniond Quatern;             Quatern.setIdentity();
    double roll, pitch, yaw;
    double cy, sy, cr, sr, cp, sp;

    roll = Euler_ang[0];   pitch = Euler_ang[1];    yaw = Euler_ang[2];

    cy = cos(yaw * 0.5);      sy = sin(yaw * 0.5);  
    cr = cos(roll * 0.5);     sr = sin(roll * 0.5);
    cp = cos(pitch * 0.5);    sp = sin(pitch * 0.5);

    Quatern.w() = cr * cp * cy + sr * sp * sy;
    Quatern.x() = sr * cp * cy - cr * sp * sy;
    Quatern.y() = cr * sp * cy + sr * cp * sy;
    Quatern.z() = cr * cp * sy - sr * sp * cy;

    return Quatern;
    }

Eigen::Vector3d Quat_to_Euler(Eigen::Quaterniond Quat){
    Eigen::Vector3d Euler_ang;          Euler_ang.setZero();
    double roll, pitch, yaw;
    // double x, y, z, w;

    roll = atan2(2 * (Quat.w() * Quat.x() + Quat.y() * Quat.z()), 1 - 2 * (Quat.x()*Quat.x() + Quat.y()*Quat.y()));
    pitch = asin(2 * (Quat.w() * Quat.y() - Quat.z() * Quat.x()));
    yaw = atan2(2 * (Quat.w() * Quat.z() + Quat.x() * Quat.y()), 1 - 2 * (Quat.y()*Quat.y() + Quat.z()*Quat.z()));
    
    Euler_ang(0) = roll; Euler_ang(1) = pitch; Euler_ang(2) = yaw;
    
    return Euler_ang;
}

Eigen::Quaterniond quatMult(Eigen::Quaterniond q1, Eigen::Quaterniond q2) {
    Eigen::Quaterniond resultQ;
    resultQ.setIdentity();

    resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

    return resultQ;
}

/*************************************************************************/
///@brief main

int main(int argc, char** argv) {
    adas::uint16_t hz = 100;
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    // ros init
    ros::init(argc, argv, "simulator");
    adas_dds::NodeHandle n;
    ros::NodeHandle nh("~");
    int loglevel = google::WARNING;
    std::string verbosity = "SILENT";
    nh.getParam("loglevel", loglevel);
    nh.getParam("verbosity", verbosity);
    glogInit2("simulator", false, loglevel, (int)adas::VERBOSITY_TYPE(verbosity));
    ros::Rate loop_rate(hz);
    // current vehicle name
    std::string vehicle_name = adas::GetPlatformName();
    // set up publisher
    ::NodePublisher pub(n);
    ::NodeSubscriber sub(n);
    //setup udp server
    std::string host;
    int port;
    int sockfd; 
    nh.getParam("host", host);
    nh.getParam("port", port);
    // uint8_t buffer[MAXLINE]; 
    struct sockaddr_in servaddr, cliaddr;    
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    }     
    memset(&servaddr, 0, sizeof(servaddr)); 
    memset(&cliaddr, 0, sizeof(cliaddr));      
    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = inet_addr(host.c_str());
    servaddr.sin_port = htons(port);     
    // Bind the socket with the server address 
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
            sizeof(servaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    
    /**ES EKF sensor fusion GPS with IMU**/
    /***********************************************************/
    double pre_time;      //init_time
    double time, delta_time;

    // IMU_data
    sensor_msgs::Imu pre_Imu_raw_data;
    sensor_msgs::Imu Imu_raw_data;
    // Odometry_data
    nav_msgs::Odometry pre_GPS_odom;
    nav_msgs::Odometry GPS_odom;
    nav_msgs::Odometry gt_pos_vel;
    geometry_msgs::Vector3 gt_acc;

    Eigen::Vector3d C_IMU_acc;                  C_IMU_acc.setZero();
    Eigen::Vector3d C_pre_IMU_acc;              C_pre_IMU_acc.setZero();
    Eigen::Vector3d C_Nav_frame_acc;            C_Nav_frame_acc.setZero();
    Eigen::Vector3d C_IMU_angular_rate;         C_IMU_angular_rate.setZero();
    Eigen::Vector3d C_pre_IMU_angular_rate;     C_pre_IMU_angular_rate.setZero();

    //Initial Value for ES EKF
    Eigen::Vector3d C_Position_predict;         C_Position_predict.setZero();
    Eigen::Vector3d C_Position_estimate;        C_Position_estimate.setZero();
    Eigen::Vector3d C_pre_Position_estimate;    C_pre_Position_estimate.setZero();

    Eigen::Vector3d C_Velocity_predict;         C_Velocity_predict.setZero();   
    Eigen::Vector3d C_Velocity_estimate;        C_Velocity_estimate.setZero();
    Eigen::Vector3d C_pre_Velocity_estimate;    C_pre_Velocity_estimate.setZero();

    Eigen::Quaterniond C_Quaternion_predict;       C_Quaternion_predict.setIdentity();
    Eigen::Quaterniond C_Quaternion_estimate;      C_Quaternion_estimate.setIdentity();
    Eigen::Quaterniond C_pre_Quaternion_estimate;  C_pre_Quaternion_estimate.setIdentity();

    Eigen::MatrixXd skew_matrix(3,3);         skew_matrix.setZero();
    Eigen::MatrixXd rotation_matrix(3,3);     rotation_matrix.setZero();

    //define nominal covariance
    float var_imu_acc = 0.1, var_imu_w = 0.01, var_gnss_x = 0.01, var_gnss_y = 0.01, var_gnss_z = 0.01;
    // float gravity = 9.81;
    float gravity = 0;

    Eigen::Vector3d gravity_vect;               gravity_vect.setZero();
    gravity_vect(2) = -gravity;            // [0, 0 , -g]

    /*Define Jacobian matrix*/
    //Motion noise Jacobian
    Eigen::MatrixXd L_jacobian(9,6);
    L_jacobian.setZero();
    L_jacobian.block<6,6> (3,0).setIdentity();

    //Measurement noise Jacobian
    /*in case GPS generate only position*/
    // Eigen::MatrixXd H_jacobian(3,9);
    // H_jacobian.setZero();
    // H_jacobian.block<3,3> (0,0).setIdentity();
    /*in case GPS generate info about velocity*/
    Eigen::MatrixXd H_jacobian(6,9);
    H_jacobian.setZero();
    H_jacobian.block<6,6> (0,0).setIdentity();
              
             
    
    /*Define Covariance Matrix*/
    Eigen::MatrixXd P_covariance(9,9);          P_covariance.setIdentity();
    Eigen::MatrixXd pre_P_covariance(9,9);      pre_P_covariance.setIdentity();

    pre_time = 0; 
    int count_update = 0;

    Eigen::Vector3d Euler_angle;                Euler_angle.setIdentity();        
    Eigen::Vector3d gt_Euler;                    gt_Euler.setIdentity();
    Eigen::Vector3d Euler_estimate;         Euler_estimate.setZero();

    // Qvector<double> Position_estimate_x_plot(3000);
    // Qvector<double> time_plot(3000);

    Eigen::Vector3d Initial_position;       Initial_position.setZero();
    Eigen::VectorXd GNSS_save(6);       GNSS_save.setZero();

    ofstream myfile;
    myfile.open ("../../../../ES\ EKF/Error-State-Extended-Kalman-Filter-master/100hz.txt"); 

    while (ros::ok()) {    
         
        Imu_raw_data = sub.get_IMU_data();
        GPS_odom = sub.get_GPS_Odometry();
        gt_pos_vel = sub.get_gt_pos_vel();
        gt_acc = sub.get_gt_acc();
        
        Eigen::Quaterniond gt_Quat;                 gt_Quat.setIdentity();
        gt_Quat.w() = gt_pos_vel.pose().pose().orientation().w();
        gt_Quat.x() = gt_pos_vel.pose().pose().orientation().x();
        gt_Quat.y() = gt_pos_vel.pose().pose().orientation().y();
        gt_Quat.z() = gt_pos_vel.pose().pose().orientation().z();
        /* Initial value of Quaternion */
        
        if (count_update == 0){
            C_Position_estimate(0) = gt_pos_vel.pose().pose().position().x();
            C_Position_estimate(1) = gt_pos_vel.pose().pose().position().y();
            C_Position_estimate(2) = gt_pos_vel.pose().pose().position().z();

            C_Quaternion_estimate.w() = gt_pos_vel.pose().pose().orientation().w();
            C_Quaternion_estimate.x() = gt_pos_vel.pose().pose().orientation().x();
            C_Quaternion_estimate.y() = gt_pos_vel.pose().pose().orientation().y();
            C_Quaternion_estimate.z() = gt_pos_vel.pose().pose().orientation().z();
        }

        count_update++;
        if((count_update % 10 == 0) && ((count_update <= 2000) || (count_update >= 2200))){
            C_Position_estimate(0) = gt_pos_vel.pose().pose().position().x();
            C_Position_estimate(1) = gt_pos_vel.pose().pose().position().y();
            C_Position_estimate(2) = gt_pos_vel.pose().pose().position().z();

            C_Quaternion_estimate.w() = gt_pos_vel.pose().pose().orientation().w();
            C_Quaternion_estimate.x() = gt_pos_vel.pose().pose().orientation().x();
            C_Quaternion_estimate.y() = gt_pos_vel.pose().pose().orientation().y();
            C_Quaternion_estimate.z() = gt_pos_vel.pose().pose().orientation().z();

        }

        nav_msgs::Odometry Odometry_pose;
        Odometry_pose.header().stamp().sec() = ros::Time::now().toSec();
        Odometry_pose.header().stamp().nsec() = ros::Time::now().toNSec();

        Odometry_pose.pose().pose().position().x() = C_Position_estimate(0);
        Odometry_pose.pose().pose().position().y() = C_Position_estimate(1);
        Odometry_pose.pose().pose().position().z() = C_Position_estimate(2);
        Odometry_pose.pose().pose().orientation().x() = C_Quaternion_estimate.x();
        Odometry_pose.pose().pose().orientation().y() = C_Quaternion_estimate.y();
        Odometry_pose.pose().pose().orientation().z() = C_Quaternion_estimate.z();
        Odometry_pose.pose().pose().orientation().w() = C_Quaternion_estimate.w();
        Odometry_pose.twist().twist().linear().x() = C_Velocity_estimate(0);
        Odometry_pose.twist().twist().linear().y() = C_Velocity_estimate(0);
        Odometry_pose.twist().twist().linear().z() = C_Velocity_estimate(0);
        Odometry_pose.twist().twist().angular().x() = Imu_raw_data.angular_velocity().x();
        Odometry_pose.twist().twist().angular().y() = Imu_raw_data.angular_velocity().y();
        Odometry_pose.twist().twist().angular().z() = Imu_raw_data.angular_velocity().z();

        Euler_angle = Quat_to_Euler(C_Quaternion_estimate);
        gt_Euler = Quat_to_Euler(gt_Quat);

        cout << "count update:                           " << count_update <<endl;

        /*print*/
        printf("estimate_x     = %f \r\n", C_Position_estimate(0));
        printf("gt_x           = %f \r\n", gt_pos_vel.pose().pose().position().x());
        printf("estimate_y     = %f \r\n", C_Position_estimate(1));
        printf("gt_y           = %f \r\n", gt_pos_vel.pose().pose().position().y());
        printf("estimate_z     = %f \r\n", C_Position_estimate(2));
        printf("gt_z           = %f \r\n", gt_pos_vel.pose().pose().position().z());
        printf("estimate_vel_x = %f \r\n", C_Velocity_estimate(0));
        printf("gt_vel_x       = %f \r\n", gt_pos_vel.twist().twist().linear().x());
        printf("estimate_vel_y = %f \r\n", C_Velocity_estimate(1));
        printf("gt_vel_y       = %f \r\n", gt_pos_vel.twist().twist().linear().y());
        printf("estimate_yaw   = %f \r\n", Euler_angle(2) * 180/ M_PI); 
        printf("gt_yaw         = %f \r\n", gt_Euler(2) * 180/ M_PI);
        printf("yaw_error_rad  = %f \r\n", abs(gt_Euler(2) - Euler_angle(2))* 180/ M_PI);


        double ES_covariance[9];

        for (int i = 0; i <= 8; i++){
            ES_covariance[i] = P_covariance(i,i);
        }
        for (unsigned i = 0; i <= 8; i++) {
            if(i <= 2){
                Odometry_pose.pose().covariance()[i*7] = ES_covariance[i];
            }
            else if ((i > 2) && (i <= 5)){
                Odometry_pose.twist().covariance()[(i - 3)*7] = ES_covariance[i];
                Odometry_pose.twist().covariance()[i*7] = var_imu_w;
            }
            else{
                Odometry_pose.pose().covariance()[(i - 3)*7] = ES_covariance[i];
            }
            
        }
        cout << "uncertain x y z vx vy vz r p y: " ;
        for (int i = 0; i <= 8; i++){
            cout << ES_covariance [i] << "   " ;
        }
        cout << endl << endl;

        pub.publish_odom(Odometry_pose);

        if ((count_update >= 500) && (count_update <= 5500))
        {   //   0     1    2   3      4    5     6     7        8      9       10        11       12         13     14        15
            // time x_est x_gt y_est y_gt x_err y_err yaw_est yaw_gt yaw_err vel_x_est vel_x_gt vel_y_est vel_y_gt vel_x_err vel_y_err 
            myfile << double(count_update)/100;
            myfile << " ";
            myfile << C_Position_estimate(0);
            myfile << " ";
            myfile << gt_pos_vel.pose().pose().position().x();
            myfile << " ";
            myfile << C_Position_estimate(1);
            myfile << " ";
            myfile << gt_pos_vel.pose().pose().position().y();
            myfile << " ";
            myfile << gt_pos_vel.pose().pose().position().x() - C_Position_estimate(0);
            myfile << " ";
            myfile << gt_pos_vel.pose().pose().position().y() - C_Position_estimate(1);
            myfile << " ";

            myfile << Euler_angle(2) * 180/ M_PI;
            myfile << " ";
            myfile << gt_Euler(2) * 180/ M_PI;
            myfile << " ";
            myfile << (gt_Euler(2) - Euler_angle(2)) * 180/ M_PI;
            myfile << " ";

            myfile << C_Velocity_estimate(0);
            myfile << " ";
            myfile << gt_pos_vel.twist().twist().linear().x();
            myfile << " ";
            myfile << C_Velocity_estimate(1);
            myfile << " ";
            myfile << gt_pos_vel.twist().twist().linear().y();
            myfile << " ";
            myfile << gt_pos_vel.twist().twist().linear().x() - C_Velocity_estimate(0);
            myfile << " ";
            myfile << gt_pos_vel.twist().twist().linear().y() - C_Velocity_estimate(1);

            myfile << "\n";
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    myfile.close();
    
    return 0;
}