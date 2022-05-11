#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <math.h>
#include <bits/stdc++.h>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "ros/time.h"
#include "sensor_msgs/JointState.h"
#include <termios.h>
#include <Eigen/Dense>
#include <chrono>
#include <sys/stat.h>
#include "trajectory_msgs/JointTrajectory.h"
#include <pwd.h>

#define ELBOW 0 
#define GRIPPER 1
#define LEFT_FINGER 2 
#define RIGHT_FINGER 3
#define SHOULDER 4 
#define WAIST 5
#define WRIST_ANGLE 6
#define WRIST_ROTATE 7
#define PI 3.1428

using namespace std;
using namespace Eigen;
using namespace std::chrono;




double omega_link1 = 0; 
double omega_link2 = 0;
double omega_link3 = 0;
double omega_link4 = 0;
double omega_link5 = 0; 
double zeta_link1 = 0;
double zeta_link2 = 0;
double zeta_link3 = 0;
double zeta_link4 = 0;
double zeta_link5 = 0;

Vector2d K_LQR1;
Vector2d K_LQR2;
Vector2d K_LQR3;
Vector2d K_LQR4;
Vector2d K_LQR5;

double alpha = 0;
double p_x = 0;
double p_y = 0;
double p_z = 0;
double z_5x = 0;
double z_5y = 0;
double z_5z = 0;

unsigned int trajectory_planning = 0;

double tf = 0;

VectorXd qf = VectorXd::Zero(5);

void cal_torque(void);
void update_parameters_from_file(void);
void calculate_final_reference_position(void);

ofstream myfile;
ofstream Data_file;
string file_path;

ros::Publisher wx200_effort_pub;
ros::Publisher wx200_wrist_rot_var;
ros::Subscriber sub;
std_msgs::Float64MultiArray msg;
trajectory_msgs::JointTrajectory traj;

char ch;

VectorXd q_ref = VectorXd::Zero(5);
VectorXd q_ref_dot = VectorXd::Zero(5);
VectorXd q_ref_ddot = VectorXd::Zero(5);

VectorXd q = VectorXd::Zero(5);
VectorXd q_dot = VectorXd::Zero(5);
VectorXd torque = VectorXd::Zero(5);

unsigned int start_flag = 0;

/***********************for LSPB ********************/
void LSPB(double Tf, double t, unsigned int count, const Ref<const VectorXd>& q0, const Ref<const VectorXd>& qf);
MatrixXd q_ref_t = MatrixXd::Zero(20000, 5);
MatrixXd q_ref_dot_t = MatrixXd::Zero(20000, 5);
MatrixXd q_ref_ddot_t = MatrixXd::Zero(20000, 5);
unsigned int start_flag_LSPB = 0;
unsigned int count_1 = 0;
/***********************for LSPB ********************/
duration<double, std::milli> time_span;
high_resolution_clock::time_point t0;
high_resolution_clock::time_point t1;
high_resolution_clock::time_point t2;
ofstream Data_file_q_dot;
double t = 0;
double prev_t = 0;
double dt = 0;
//ofstream Data_file_q_ddot;

/****************************quintic_polynomial*****************/
unsigned int start_flag_quintic_pol = 0;
MatrixXd coefficient = MatrixXd::Zero(6, 5);
void quintic_polynomial(double T_max, const Ref<const VectorXd>& r0, const Ref<const VectorXd>& rf, const Ref<const VectorXd>& v0, const Ref<const VectorXd>& vf, const Ref<const VectorXd>& a0, const Ref<const VectorXd>& af);
/****************************quintic_polynomial*****************/

unsigned long count_t = 0;

ros::Time time_val;

unsigned int testing_flag = 0;

bool IsPathExist(const std::string &s)
{
  struct stat buffer;
  return (stat (s.c_str(), &buffer) == 0);
}


/***** this function is used to get the current datetime*******/
std::string datetime()
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d-%m-%Y %H-%M-%S",timeinfo);
    return std::string(buffer);
}

/************* this is the callback function  for Joint_state_publisher*********/
void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{

  static unsigned int temp_count=0;
  double t_main =0.0 ;

/* for(int i = 0; i< 8 ; i++)
  { 
	myfile << _js->name[i].c_str() << ": Position = " << _js->position[i] << " Effort = " << _js->effort[i] << " Velocity = " << _js->velocity[i] << "\n";
  }*/
//  myfile << "\n\n";
  if(!start_flag)
  	t0 = high_resolution_clock::now();  /* the time is read at the start of the controller*/

  t2 = high_resolution_clock::now();
  time_span = t2 - t1;
  t = (time_span.count()/1000);
  if(start_flag_LSPB)
 	 count_1 = count_1 + 1;
  time_span = t2 - t0;
  t_main = (time_span.count()/1000);
  dt = t_main - prev_t;
  prev_t = t_main;

  /*********current position and velocity are updated from sensor messages*********/
  q(0) = _js->position[WAIST];
  q(1) = _js->position[SHOULDER];
  q(2) = _js->position[ELBOW];
  q(3) = _js->position[WRIST_ANGLE];
  q(4) = _js->position[WRIST_ROTATE];
			 
  q_dot(0) = _js->velocity[WAIST];
  q_dot(1) = _js->velocity[SHOULDER];
  q_dot(2) = _js->velocity[ELBOW];
  q_dot(3) = _js->velocity[WRIST_ANGLE];
  q_dot(4) = _js->velocity[WRIST_ROTATE];

 cal_torque(); /* the required torque which needs to be provided to the joint is calculated using this function*/
  start_flag = 1;
  for(int i = 0; i< 5 ; i++) 
  {
	msg.data[i] = torque(i); /*the calculated torque values are updated in the message which needs to be publised*/	
	//myfile << "torque = " << torque(i) << "\n";
	//myfile << "commanded position = " << q_ref(i) << "\n";
  }
  wx200_effort_pub.publish(msg); /* the variable msg is publsihed in the effort_publisher*/	
  Data_file << t_main << ","<< q_ref(0) << "," << q(0) << ","<< q_ref(1) << "," << q(1) << ","<< q_ref(2) << "," << q(2) << ","<< q_ref(3) << "," << q(3) << ","<< q_ref(4) << "," << q(4)<< "\n"; /* the reference and current positions of all the joints are written in a .dat file for plotting*/

}



int main(int argc, char **argv)
{
	
	static double prev_alpha = 0;
	static double prev_p_x = 0;
	static double prev_p_y = 0;
	static double prev_p_z = 0;
	static double prev_z_5x = 0;
	static double prev_z_5y = 0;
	static double prev_z_5z = 0;
	int myuid;
	passwd *mypasswd;
	static VectorXd q0 = VectorXd::Zero(5);
	VectorXd v0 = VectorXd::Zero(5);
	VectorXd vf = VectorXd::Zero(5);
	VectorXd alpha0 = VectorXd::Zero(5);
	VectorXd alphaf = VectorXd::Zero(5);
	VectorXd temp1 = VectorXd::Zero(6);
	VectorXd temp2 = VectorXd::Zero(6);
	VectorXd temp3 = VectorXd::Zero(6);
	string fname;


	ros::init(argc, argv, "wx200_effort"); /* Initialize the ros */
	ros::NodeHandle n; /* create a node handler*/


	myuid = getuid(); /* get the current user ID*/
	mypasswd = getpwuid(myuid);
	file_path = mypasswd->pw_dir; /* get the user home path*/
	file_path+="/interbotix_ws/src/Interbotix_src/Gazebo/Data_log"; /* add the home path with the path where Data is stored for the graph and data is read to update the paramenters*/

	std::string path = file_path+"/data.csv"; /* path for data.dat file used to store current and refernce positions of the joints*/

	//fname = file_path + "/"+ datetime() + ".txt";
	//myfile.open (fname);

	Data_file.open(path);
	sub = n.subscribe("/wx200/joint_states", 1, SetJointStates); /* create a subscriber, /wx200/joint_states is the node which is subscribed and SetJointStates is the callback function*/
	wx200_effort_pub = n.advertise<std_msgs::Float64MultiArray>("/wx200/arm_controller/command",1);	/*create a publisher which publishes on /wx200/arm_controller/command node*/
	
	msg.layout.dim.resize(5); /* initialize the dimension of the layout to 5 i.e. number of joints*/
	msg.data.resize(5); /* initialize the dimension of the data to 5 i.e. number of joints*/
	/* updating the layout variables for each joint and initializing the data variables with zero*/ 
	msg.layout.dim[0].label = "waist";
	msg.layout.dim[0].size = 0;
	msg.layout.dim[0].stride = 0;
	msg.data[0] = 0;
	msg.layout.dim[1].label = "shoulder";
	msg.layout.dim[1].size = 0;
	msg.layout.dim[1].stride = 0;
	msg.data[1] = 0;
	msg.layout.dim[2].label = "elbow";
	msg.layout.dim[2].size = 0;
	msg.layout.dim[2].stride = 0;
	msg.data[2] = 0;
	msg.layout.dim[3].label = "wrist_angle";
	msg.layout.dim[3].size = 0;
	msg.layout.dim[3].stride = 0;
	msg.data[3] = 0;
	msg.layout.dim[4].label = "wrist_rotate";
	msg.layout.dim[4].size = 0;
	msg.layout.dim[4].stride = 0;
	msg.data[4] = 0;

	while(ros::ok()) /* check for the rosmaster is on or off*/
	{
		update_parameters_from_file();	/* update the configuration parameters and end affector position from the text file*/ 	
		if(((alpha != prev_alpha) || (p_x != prev_p_x) ||(p_y != prev_p_y) ||(p_z != prev_p_z) ||(z_5x != prev_z_5x) ||(z_5y != prev_z_5y) ||(z_5z != prev_z_5z)) && (start_flag))/* check if the current end affector position is changed or not*/ 
		{
			/*if the end affector position is changed*/ 	
			t = 0;
			calculate_final_reference_position(); /* calculate the new reference position for each joint using inverse kinematics*/
			t1 = high_resolution_clock::now(); /* initialize the clock for trajectory*/
			prev_alpha = alpha;
			prev_p_x = p_x;
			prev_p_y = p_y;
			prev_p_z = p_z;
			prev_z_5x = z_5x;
			prev_z_5y = z_5y;
			prev_z_5z = z_5z;
			q0 = q;/*update variable q0 with current position of the joints*/

			/*depending on the user input for the trajectory calculation, trajectory is calculated using LSPB or quintic polynomial */ 
			if(trajectory_planning == 0)
			{	
				count_1 = 0;
				start_flag_LSPB = 1;
				start_flag_quintic_pol = 0;
			}
			else
			{
				quintic_polynomial(tf,q0,qf,v0,vf,alpha0,alphaf);/* calculate the coeffcient for quintic polynomial*/
				start_flag_quintic_pol = 1;
				start_flag_LSPB = 0;
			}
		}	

		if((start_flag_LSPB == 1) && (t < tf))
		{
			/* calculate the reference position of all joints using LSPB*/
			LSPB(tf, t, count_1, q0, qf); 
			q_ref  = q_ref_t.row(count_1);
			q_ref_dot =  q_ref_dot_t.row(count_1);
			q_ref_ddot = q_ref_ddot_t.row(count_1);			
		}
		else if((start_flag_quintic_pol == 1) && (t < tf))
		{
			/* calculate the reference position of all joints using quintic polynomial*/
			temp1 << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);
			temp2 << 0, 1, 2*t, 3*pow(t, 2), 4*pow(t, 3), 5*pow(t, 4);
			temp3 << 0, 0, 2, 6*t, 12*pow(t, 2), 20*pow(t, 3);

			q_ref = coefficient.transpose()*temp1;
			q_ref_dot = coefficient.transpose()*temp2;
			q_ref_ddot = coefficient.transpose()*temp3;
			
		}
		else
		{
			start_flag_LSPB = 0;
			start_flag_quintic_pol = 0;
		}
		ros::spinOnce(); /* spin the loop once */
	}


//myfile.close();
Data_file.close();
Data_file_q_dot.close();
return 0;	
}


void cal_torque()
{
    VectorXd Kq = VectorXd::Zero(5);
    MatrixXd M = MatrixXd::Zero(5, 5);
    MatrixXd C = MatrixXd::Zero(5, 5);
    VectorXd q_ddot = VectorXd::Zero(5);
    VectorXd q_error = VectorXd::Zero(5);
    VectorXd q_dot_error = VectorXd::Zero(5);
    static VectorXd q_int_error = VectorXd::Zero(5);
    VectorXd out_vector_1 = VectorXd::Zero(5);
    VectorXd out_vector_2 = VectorXd::Zero(5);
    MatrixXd KP = MatrixXd::Zero(5, 5);
    MatrixXd KD = MatrixXd::Zero(5, 5);
    MatrixXd KI = MatrixXd::Zero(5, 5);
    VectorXd U_LQR = VectorXd::Zero(5);
    VectorXd U_baseline = VectorXd::Zero(5);


    double m1 = 1.607196;
    double m2 = 0.715452;
    double m3 = 0.669306;
    double m4 = 0.416434;
    double m5 = 0.730841;

    double g = 9.81;

    double l1 = 0.11325;
    double l2 = 0.20616;
    double l3 = 0.2;
    double l4 = 0.065;
    double l5 = 0.10915;

    double lc1 = 0.047;
    double lc2 = 0.1353;
    double lc3 = 0.118;
    double lc4 = 0.0444;
    double lc5 = 0.0955;

    double I1 = 0.002175;
    double I2 = 0.00401991;
    double I3 = 0.00258893;
    double I4 = 0.00013472;
    double I5 = 0.00029738;
    
    unsigned int i = 0;
    unsigned int j = 0;

   KP.diagonal() << pow(omega_link1,2), pow(omega_link2,2),pow(omega_link3,2), pow(omega_link4,2), pow(omega_link5,2);
   
   KD.diagonal() << 2*omega_link1*zeta_link1, 2*omega_link2*zeta_link2,2*omega_link3*zeta_link3, 2*omega_link4*zeta_link4, 2*omega_link5*zeta_link5;
   
   KI.diagonal() << 0,0,0,0,0;
   
   /* equation for M */
    M(0,0) = I1 + I2 + I3 + I4 + I5 + (m4*pow(cos(q(0)),2)*pow((l3*cos(q(1) + q(2)) + l2*cos(q(1)) + lc4*cos(q(1) + q(2) + q(3))),2)) + (m5*pow(cos(q(0)),2)* pow((l3*cos(q(1) + q(2)) + l2*cos(q(1)) + lc5*cos(q(1) + q(2) + q(3))),2)) + (m4*pow(sin(q(0)),2)*pow((l3*cos(q(1) + q(2)) + l2*cos(q(1)) + lc4*cos(q(1) + q(2) + q(3))),2)) + (m5*pow(sin(q(0)),2)*pow((l3*cos(q(1) + q(2)) + l2*cos(q(1)) + lc5*cos(q(1) + q(2) + q(3))),2)) + (m3*pow(cos(q(0)),2)*pow((lc3*cos(q(1) + q(2)) + l2*cos(q(1))),2)) + (m3*pow(sin(q(0)),2)*pow((lc3*cos(q(1) + q(2)) + l2*cos(q(1))),2)) + (pow(lc2,2)*m2*pow(cos(q(0)),2)*pow(cos(q(1)),2)) + (pow(lc2,2)*m2*pow(cos(q(1)),2)*pow(sin(q(0)),2));
    M(0,4) = -I5*cos(q(1) + q(2) + q(3));
    M(1,1) = I2 + I3 + I4 + I5 + pow(l2,2)*m3 + pow(l2,2)*m4 + pow(l2,2)*m5 + pow(l3,2)*m4 + pow(l3,2)*m5 + pow(lc2,2)*m2 + pow(lc3,2)*m3 + pow(lc4,2)*m4 + pow(lc5,2)*m5 + 2*l2*lc4*m4*cos(q(2) + q(3)) + 2*l2*lc5*m5*cos(q(2) + q(3)) + 2*l2*l3*m4*cos(q(2)) + 2*l2*l3*m5*cos(q(2)) + 2*l2*lc3*m3*cos(q(2)) + 2*l3*lc4*m4*cos(q(3)) + 2*l3*lc5*m5*cos(q(3));
    M(1,2) = I3 + I4 + I5 + (pow(l3,2)*(m4 + m5)) + pow(lc3,2)*m3 + pow(lc4,2)*m4 + pow(lc5,2)*m5 + l2*lc4*m4*cos(q(2) + q(3)) + l2*lc5*m5*cos(q(2) + q(3)) + l2*l3*m4*cos(q(2)) + l2*l3*m5*cos(q(2)) + l2*lc3*m3*cos(q(2)) + 2*l3*lc4*m4*cos(q(3)) + 2*l3*lc5*m5*cos(q(3));
    M(1,3) = I4 + I5 + pow(lc4,2)*m4 + pow(lc5,2)*m5 + (l2*cos(q(2) + q(3)) + l3 * cos(q(3))) * (m4*lc4 + m5 * lc5);
    M(2,1) = M(1,2);
    M(2,2) = I3 + I4 + I5 + (m4 + m5)*pow(l3,2) + m3*pow(lc3,2) + m4*pow(lc4,2) + m5*pow(lc5,2) + 2*l3*cos(q(3))*(m4*lc4 + m5*lc5);
    M(2,3) = I4 + I5 + m4*pow(lc4,2) + m5*pow(lc5,2) + l3*cos(q(3))*(m4*lc4 + m5*lc5);
    M(3,1) = M(1,3);
    M(3,2) = M(2,3);
    M(3,3) = I4 + I5 + m4*pow(lc4,2) + m5*pow(lc5,2);
    M(4,0) = M(0,4);
    M(4,4) = I5;

 /* equation for C */
    C(0,0) = (- (pow(l3,2)*m4*sin(2*q(1) + 2*q(2)))/2 - (pow(l3,2)*m5*sin(2*q(1) + 2*q(2)))/2 - (pow(lc3,2)*m3*sin(2*q(1) + 2*q(2)))/2 - (pow(l2,2)*m3*sin(2*q(1)))/2 - (pow(l2,2)*m4*sin(2*q(1)))/2 - (pow(l2,2)*m5*sin(2*q(1)))/2 - (pow(lc2,2)*m2*sin(2*q(1)))/2 - (pow(lc4,2)*m4*sin(2*q(1) + 2*q(2) + 2*q(3)))/2 - (pow(lc5,2)*m5*sin(2*q(1) + 2*q(2) + 2*q(3)))/2 - l2*lc4*m4*sin(2*q(1) + q(2) + q(3)) - l2*lc5*m5*sin(2*q(1) + q(2) + q(3)) - l3*lc4*m4*sin(2*q(1) + 2*q(2) + q(3)) - l3*lc5*m5*sin(2*q(1) + 2*q(2) + q(3)) - l2*l3*m4*sin(2*q(1) + q(2)) - l2*l3*m5*sin(2*q(1) + q(2)) - l2*lc3*m3*sin(2*q(1) + q(2)))*q_dot(1) + (- (pow(l3,2)*m4*sin(2*q(1) + 2*q(2)))/2 - (pow(l3,2)*m5*sin(2*q(1) + 2*q(2)))/2 - (pow(lc3,2)*m3*sin(2*q(1) + 2*q(2)))/2 - (pow(lc4,2)*m4*sin(2*q(1) + 2*q(2) + 2*q(3)))/2 - (pow(lc5,2)*m5*sin(2*q(1) + 2*q(2) + 2*q(3)))/2 - (l2*lc4*m4*sin(2*q(1) + q(2) + q(3)))/2 - (l2*lc5*m5*sin(2*q(1) + q(2) + q(3)))/2 - (l2*lc4*m4*sin(q(2) + q(3)))/2 - (l2*lc5*m5*sin(q(2) + q(3)))/2 - (l2*l3*m4*sin(q(2)))/2 - (l2*l3*m5*sin(q(2)))/2 - (l2*lc3*m3*sin(q(2)))/2 - l3*lc4*m4*sin(2*q(1) + 2*q(2) + q(3)) - l3*lc5*m5*sin(2*q(1) + 2*q(2) + q(3)) - (l2*l3*m4*sin(2*q(1) + q(2)))/2 - (l2*l3*m5*sin(2*q(1) + q(2)))/2 - (l2*lc3*m3*sin(2*q(1) + q(2)))/2)*q_dot(2) + (- (pow(lc4,2)*m4*sin(2*q(1) + 2*q(2) + 2*q(3)))/2 - (pow(lc5,2)*m5*sin(2*q(1) + 2*q(2) + 2*q(3)))/2 - (l2*lc4*m4*sin(2*q(1) + q(2) + q(3)))/2 - (l2*lc5*m5*sin(2*q(1) + q(2) + q(3)))/2 - (l2*lc4*m4*sin(q(2) + q(3)))/2 - (l2*lc5*m5*sin(q(2) + q(3)))/2 - (l3*lc4*m4*sin(q(3)))/2 - (l3*lc5*m5*sin(q(3)))/2 - (l3*lc4*m4*sin(2*q(1) + 2*q(2) + q(3)))/2 - (l3*lc5*m5*sin(2*q(1) + 2*q(2) + q(3)))/2)*q_dot(3);
    C(0,1) = (- (pow(l3,2)*m4*sin(2*q(1) + 2*q(2)))/2 - (pow(l3,2)*m5*sin(2*q(1) + 2*q(2)))/2 - (pow(lc3,2)*m3*sin(2*q(1) + 2*q(2)))/2 - (pow(l2,2)*m3*sin(2*q(1)))/2 - (pow(l2,2)*m4*sin(2*q(1)))/2 - (pow(l2,2)*m5*sin(2*q(1)))/2 - (pow(lc2,2)*m2*sin(2*q(1)))/2 - (pow(lc4,2)*m4*sin(2*q(1) + 2*q(2) + 2*q(3)))/2 - (pow(lc5,2)*m5*sin(2*q(1) + 2*q(2) + 2*q(3)))/2 - l2*lc4*m4*sin(2*q(1) + q(2) + q(3)) - l2*lc5*m5*sin(2*q(1) + q(2) + q(3)) - l3*lc4*m4*sin(2*q(1) + 2*q(2) + q(3)) - l3*lc5*m5*sin(2*q(1) + 2*q(2) + q(3)) - l2*l3*m4*sin(2*q(1) + q(2)) - l2*l3*m5*sin(2*q(1) + q(2)) - l2*lc3*m3*sin(2*q(1) + q(2)))*q_dot(0) + ((I5*sin(q(1) + q(2) + q(3)))/2)*q_dot(4);
    C(0,2) = (- (pow(l3,2)*m4*sin(2*q(1) + 2*q(2)))/2 - (pow(l3,2)*m5*sin(2*q(1) + 2*q(2)))/2 - (pow(lc3,2)*m3*sin(2*q(1) + 2*q(2)))/2 - (pow(lc4,2)*m4*sin(2*q(1) + 2*q(2) + 2*q(3)))/2 - (pow(lc5,2)*m5*sin(2*q(1) + 2*q(2) + 2*q(3)))/2 - (l2*lc4*m4*sin(2*q(1) + q(2) + q(3)))/2 - (l2*lc5*m5*sin(2*q(1) + q(2) + q(3)))/2 - (l2*lc4*m4*sin(q(2) + q(3)))/2 - (l2*lc5*m5*sin(q(2) + q(3)))/2 - (l2*l3*m4*sin(q(2)))/2 - (l2*l3*m5*sin(q(2)))/2 - (l2*lc3*m3*sin(q(2)))/2 - l3*lc4*m4*sin(2*q(1) + 2*q(2) + q(3)) - l3*lc5*m5*sin(2*q(1) + 2*q(2) + q(3)) - (l2*l3*m4*sin(2*q(1) + q(2)))/2 - (l2*l3*m5*sin(2*q(1) + q(2)))/2 - (l2*lc3*m3*sin(2*q(1) + q(2)))/2)*q_dot(0) + ((I5*sin(q(1) + q(2) + q(3)))/2)*q_dot(4);
    C(0,3) = (- (pow(lc4,2)*m4*sin(2*q(1) + 2*q(2) + 2*q(3)))/2 - (pow(lc5,2)*m5*sin(2*q(1) + 2*q(2) + 2*q(3)))/2 - (l2*lc4*m4*sin(2*q(1) + q(2) + q(3)))/2 - (l2*lc5*m5*sin(2*q(1) + q(2) + q(3)))/2 - (l2*lc4*m4*sin(q(2) + q(3)))/2 - (l2*lc5*m5*sin(q(2) + q(3)))/2 - (l3*lc4*m4*sin(q(3)))/2 - (l3*lc5*m5*sin(q(3)))/2 - (l3*lc4*m4*sin(2*q(1) + 2*q(2) + q(3)))/2 - (l3*lc5*m5*sin(2*q(1) + 2*q(2) + q(3)))/2)*q_dot(0) + ((I5*sin(q(1) + q(2) + q(3)))/2)*q_dot(4);
    C(0,4) = ((I5*sin(q(1) + q(2) + q(3)))/2)*q_dot(1) + ((I5*sin(q(1) + q(2) + q(3)))/2)*q_dot(2) + ((I5*sin(q(1) + q(2) + q(3)))/2)*q_dot(3);
    C(1,0) = -C(0,1);
    C(1,1) = (-l2*(l3*m4*sin(q(2)) + l3*m5*sin(q(2)) + lc3*m3*sin(q(2)) + lc4*m4*sin(q(2) + q(3)) + lc5*m5*sin(q(2) + q(3))))*q_dot(2) + (-(l2*sin(q(2) + q(3)) + l3*sin(q(3)))*(lc4*m4 + lc5*m5))*q_dot(3);
    C(1,2) = (-l2*(l3*m4*sin(q(2)) + l3*m5*sin(q(2)) + lc3*m3*sin(q(2)) + lc4*m4*sin(q(2) + q(3)) + lc5*m5*sin(q(2) + q(3))))*q_dot(1) + (-l2*(l3*m4*sin(q(2)) + l3*m5*sin(q(2)) + lc3*m3*sin(q(2)) + lc4*m4*sin(q(2) + q(3)) + lc5*m5*sin(q(2) + q(3))))*q_dot(2) + (-(l2*sin(q(2) + q(3)) + l3*sin(q(3)))*(lc4*m4 + lc5*m5))*q_dot(3);
    C(1,3) = (-(l2*sin(q(2) + q(3)) + l3*sin(q(3)))*(lc4*m4 + lc5*m5))*q_dot(1) + (-(l2*sin(q(2) + q(3)) + l3*sin(q(3)))*(lc4*m4 + lc5*m5))*q_dot(2) + (-(l2*sin(q(2) + q(3)) + l3*sin(q(3)))*(lc4*m4 + lc5*m5))*q_dot(3);
    C(1,4) = (-(I5*sin(q(1) + q(2) + q(3)))/2)*q_dot(0);
    C(2,0) = -C(0,2);
    C(2,1) = l2*(l3*m4*sin(q(2)) + l3*m5*sin(q(2)) + lc3*m3*sin(q(2)) + lc4*m4*sin(q(2) + q(3)) + lc5*m5*sin(q(2) + q(3)))*q_dot(1) + (-l3*sin(q(3))*(lc4*m4 + lc5*m5))*q_dot(3);
    C(2,2) = (-l3*sin(q(3))*(lc4*m4 + lc5*m5))*q_dot(3);
    C(2,3) = (-l3*sin(q(3))*(lc4*m4 + lc5*m5))*q_dot(1) + (-l3*sin(q(3))*(lc4*m4 + lc5*m5))*q_dot(2) + (-l3*sin(q(3))*(lc4*m4 + lc5*m5))*q_dot(3);
    C(2,4) = (-(I5*sin(q(1) + q(2) + q(3)))/2)*q_dot(0);
    C(3,0) = -C(0,3);
    C(3,1) = (l2*sin(q(2) + q(3)) + l3*sin(q(3)))*(lc4*m4 + lc5*m5)*q_dot(1) + l3*sin(q(3))*(lc4*m4 + lc5*m5)*q_dot(2);
    C(3,2) = l3*sin(q(3))*(lc4*m4 + lc5*m5)*q_dot(1) + l3*sin(q(3))*(lc4*m4 + lc5*m5)*q_dot(2);
    C(3,4) = (-(I5*sin(q(1) + q(2) + q(3)))/2)*q_dot(0);
    C(4,0) = C(0,4);
    C(4,1) = -C(1,4);
    C(4,2) = -C(2,4);
    C(4,3) = -C(3,4);

 /* equation for Kq */
    Kq(1) = g*(l2*m3*cos(q(1)) + l2*m4*cos(q(1)) + l2*m5*cos(q(1)) + lc2*m2*cos(q(1)) + lc4*m4*cos(q(1) + q(2) + q(3)) + lc5*m5*cos(q(1) + q(2) + q(3)) + l3*m4*cos(q(1) + q(2)) + l3*m5*cos(q(1) + q(2)) + lc3*m3*cos(q(1) + q(2)));
    Kq(2) = g*(lc4*m4*cos(q(1) + q(2) + q(3)) + lc5*m5*cos(q(1) + q(2) + q(3)) + l3*m4*cos(q(1) + q(2)) + l3*m5*cos(q(1) + q(2)) + lc3*m3*cos(q(1) + q(2)));
    Kq(3) = g*(cos(q(1)+q(2)+q(3)) *(lc4 * m4 + lc5 * m5) );

	q_error = q - q_ref;
	q_dot_error = q_dot - q_ref_dot;
	q_ddot = q_ref_ddot - (KP * q_error) - (KD * q_dot_error) ;

	/* calculate the LQR torque*/
	U_LQR(0) = -K_LQR1(0)*q_error(0) - K_LQR1(1)*q_dot_error(0);
	U_LQR(1) = -K_LQR2(0)*q_error(1) - K_LQR2(1)*q_dot_error(1);
	U_LQR(2) = -K_LQR3(0)*q_error(2) - K_LQR3(1)*q_dot_error(2);
	U_LQR(3) = -K_LQR4(0)*q_error(3) - K_LQR4(1)*q_dot_error(3);
	U_LQR(4) = -K_LQR5(0)*q_error(4) - K_LQR5(1)*q_dot_error(4);
	
	out_vector_1 = M * q_ddot;
	out_vector_2 = C * q_dot;
	
	U_baseline = out_vector_1 + out_vector_2 + Kq; /*calculation of baseline torque*/
	torque = U_baseline + U_LQR; /* baseline torque plus LQR torque */
	

	/* limit the torque to 8.2 Nm for shoulder joint and 4.1 Nm for all other joint*/
	for(i = 0; i <5; i++)
	{
		if(i == 1)
		{
			if(torque(i) > 8.1)
				torque(i) = 8.1;
			if(torque(i) < (-8.1))
				torque(i) = -8.1;
		}
		else
		{
			if(torque(i) > 4.1)
				torque(i) = 4.1;
			if(torque(i) < (-4.1))
				torque(i) = -4.1;
		}
	}
}


/* this function is used for calculating the reference position using LSPB*/
void LSPB(double Tf,double t, unsigned int count, const Ref<const VectorXd>& q0, const Ref<const VectorXd>& qf)
{

	VectorXd V = VectorXd::Zero(5);
	VectorXd tb = VectorXd::Zero(5);
	VectorXd alpha = VectorXd::Zero(5);
	unsigned int loop_count = 0;

	V = 1.5*(qf - q0)/Tf;
	tb = ((q0 - qf) + (V*Tf)).cwiseProduct(V.cwiseInverse());
	alpha = V.cwiseProduct(tb.cwiseInverse());

	for (loop_count = 0; loop_count < 5; loop_count++)
	{
		if ((t >= 0) && (t <= tb(loop_count)))
		{
			q_ref_t(count,loop_count) = q0(loop_count) + ((alpha(loop_count)/ 2) *pow(t,2));
			q_ref_dot_t(count, loop_count) = alpha(loop_count)*t;
			q_ref_ddot_t(count, loop_count) = alpha(loop_count);
		}
		else if ((t > tb(loop_count)) && (t <= (Tf-tb(loop_count))))
		{
			q_ref_t(count, loop_count) = ((qf(loop_count) + q0(loop_count) - (V(loop_count)*Tf))/2) + (V(loop_count)*t);
			q_ref_dot_t(count, loop_count) = V(loop_count);
			q_ref_ddot_t(count, loop_count) = 0;
		}
		else if ((t > (Tf - tb(loop_count))) && (t <= Tf))
		{
			q_ref_t(count, loop_count) = qf(loop_count) - ((alpha(loop_count)*pow(Tf, 2))/ 2) + (alpha(loop_count)*Tf*t) - ((alpha(loop_count)/2) * pow(t, 2));
			q_ref_dot_t(count, loop_count) = (alpha(loop_count)*Tf) - alpha(loop_count)*t;
			q_ref_ddot_t(count, loop_count) = -alpha(loop_count);
		}
	}

}

/* this function is used for calculating the coefficent values used in quintic polynomial*/
void quintic_polynomial(double T_max, const Ref<const VectorXd>& r0, const Ref<const VectorXd>& rf, const Ref<const VectorXd>& v0, const Ref<const VectorXd>& vf, const Ref<const VectorXd>& a0, const Ref<const VectorXd>& af)
{
	MatrixXd M = MatrixXd::Zero(6, 6);
	M << 1, 0, 0, 0, 0, 0,
		1, T_max, pow(T_max, 2), pow(T_max, 3), pow(T_max, 4), pow(T_max, 5),
		0, 1, 0, 0, 0, 0,
		0, 1, 2 * T_max, 3 * pow(T_max, 2), 4 * pow(T_max, 3), 5 * pow(T_max, 4),
		0, 0, 2, 0, 0, 0,
		0, 0, 2, 6 * T_max, 12 * pow(T_max, 2), 20 * pow(T_max, 3);

	MatrixXd div = MatrixXd::Zero(6,5);

	div << r0(0), r0(1), r0(2), r0(3), r0(4),
		rf(0), rf(1), rf(2), rf(3), rf(4),
		v0(0), v0(1), v0(2), v0(3), v0(4),
		vf(0), vf(1), vf(2), vf(3), vf(4),
		a0(0), a0(1), a0(2), a0(3), a0(4),
		af(0), af(1), af(2), af(3), af(4);
	coefficient = M.partialPivLu().solve(div);
}


void update_parameters_from_file(void)
{
	ifstream read_data_file;
	string data_123;
	string delimeter = "=";/*The value of all the parameters read from the text file is read after =*/
	string data_test;
	size_t pos = 0;
	double conv_data = 0;
	string f_path = file_path + "/data_to_read.txt"; /*file path for text file to read the parameters*/
	read_data_file.open(f_path,ios::in);
	
	/* read the line from the text file and convert the string in to float value and store in appropriate varialbles*/ 
	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		omega_link1 = stod(data_123,&pos);
	}
	
	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		omega_link2 = stod(data_123,&pos);
	}
	
	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		omega_link3 = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		omega_link4 = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		omega_link5 = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		zeta_link1 = stod(data_123,&pos);
	}
	
	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		zeta_link2 = stod(data_123,&pos);
	}
	
	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		zeta_link3 = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		zeta_link4 = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		zeta_link5 = stod(data_123,&pos);
	}


	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		K_LQR1(0) = stod(data_123,&pos);
	}
	
	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		K_LQR1(1) = stod(data_123,&pos);
	}
	
	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		K_LQR2(0) = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		K_LQR2(1) = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		K_LQR3(0) = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		K_LQR3(1) = stod(data_123,&pos);
	}
	
	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		K_LQR4(0) = stod(data_123,&pos);
	}
	
	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		K_LQR4(1) = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		K_LQR5(0) = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		K_LQR5(1) = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		alpha = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		p_x = stod(data_123,&pos);
	}
		
	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		p_y = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		p_z = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		z_5x = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		z_5y = stod(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		z_5z = stod(data_123,&pos);
	}
	
	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		trajectory_planning = stoi(data_123,&pos);
	}

	getline(read_data_file,data_123);
	while ((pos = data_123.find(delimeter)) != std::string::npos)
	{
		data_test = data_123.substr(0, pos);
		data_123.erase(0, pos + delimeter.length());
		tf = stoi(data_123,&pos);
	}
	read_data_file.close();
}


/* calculating the final reference using inverse kinematics*/
void calculate_final_reference_position(void)
{
	
	double Z5X = 0;
	double P_X = 0;
	double rho_x = 0;
	double rho_z = 0;
	double C = 0;
	double phi = 0;

	double d1 = 0.11325;
	double a2 = 0.20616;
	double a3 = 0.2;
	double d5 = 0.10915+0.065;


	qf(4) = alpha;
	qf(0) = atan2(p_y, p_x);
	Z5X = double(sqrt(pow(z_5x,2) + pow(z_5y,2)));
	P_X = double(sqrt(pow(p_x,2) + pow(p_y,2)));
	phi = atan2(z_5z,Z5X);

	rho_x = P_X - (d5*cos(phi));
	rho_z = p_z - d1 - (d5*sin(phi));

	C = double((pow(rho_x, 2) + pow(rho_z, 2) + pow(a2, 2) - pow(a3, 2)) / (2 * a2));

	qf(1) = (2 * atan2((rho_z + double(sqrt(pow(rho_z, 2) + pow(rho_x , 2) - pow(C , 2)))) , (rho_x + C)));
	qf(2) = (atan2((rho_z - (a2*sin(qf(1)))) , (rho_x - (a2*cos(qf(1))))) - qf(1));
	qf(3) = (phi - qf(2) - qf(1));
	
	qf(1) = -qf(1) + PI/2; /* reverse the sign of the calculated angle and add 1.57 to adjust the angle required according to the interbotix arm angle of shoulder*/
	qf(2) = qf(2) + PI/2; /* add 1.57 to adjust the angle required according to the interbotix arm angle of elbow*/
	qf(3) = qf(3);

	if (qf(1)<-1.97||qf(1)>1.88||qf(2)<-1.62||qf(2)>1.88||qf(3)<-2.15||qf(3)>1.75) /*limit the angle for all the joint, if the angle is out of limit then set the final position equal to current position*/
	{
		qf = q;
	}

	
}
