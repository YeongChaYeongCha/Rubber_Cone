 #include "ros/ros.h"
#include "string.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

//양쪽으로 물체 2개 인식했을 때 위치 평균값으로 각도 publish

#define RAD2DEG(x) (x*(180./M_PI))
#define DEG2RAD(x) (x*(M_PI/180.))

//Lidar_Obstacle_Range | Object_Minimum_Size | Object_Maximum_Size | Lidar_Angle_Range 값은 5.0(m), 0.04(m), 6.0(m), 80(degree)로 정하고 추후 실험을 통해 수정필요.
#define Lidar_Obstacle_Range 1.0 //5.0m
#define Object_Minimum_Size 0.04
#define Object_Maximum_Size 1.0 //6.0
#define Lidar_Angle_Range 45 //80

int count_object=0;
int detect_index=0;
int count_object_Count=0;
int fir_detect_index=0;
float fir_inclination=0.0;
float sec_inclination=0.0;
float fir_inclination_angle=0.0;
float sec_inclination_angle=0.0;
float average_inclination_angle=0.0;
float stack_pos_x=0.0;
float stack_pos_y=0.0;
float object_size=0.0;
float start_coordinate[1][2]={};
float end_coordinate[1][2]={};
float object_coordinate[][2]={};
float waypoint[2]={};

void cal_waypoint(){
	waypoint[0]=(object_coordinate[count_object-1][0]+object_coordinate[0][0])/2.0;
	waypoint[1]=(object_coordinate[count_object-1][1]+object_coordinate[0][1])/2.0;
	
	ROS_INFO("waypoint : %f	|	%f", waypoint[0], waypoint[1]); //x | y
	
	if(count_object>=2){
		/*if(waypoint[0]<0.0){
			average_inclination_angle=RAD2DEG(atan(waypoint[0]/waypoint[1]));
		}
		else{
			average_inclination_angle=-1*RAD2DEG(atan(waypoint[0]/waypoint[1]));
		}
		*/
		average_inclination_angle=RAD2DEG(atan(waypoint[0]/waypoint[1]));
	}
	else{
		ROS_INFO("!-----ERROR-----!");
		average_inclination_angle=0.0;
	}
	
	//average_inclination_angle=RAD2DEG(atan(waypoint[0]/waypoint[1]));
	waypoint[0]=0.0;
	waypoint[1]=0.0;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
	int count=(int)(360./RAD2DEG(scan->angle_increment));
	count_object=0;
	fir_inclination=0.0;
	sec_inclination=0.0;
	fir_inclination_angle=0.0;
	sec_inclination_angle=0.0;
	average_inclination_angle=0.0;
	
	for(int i=0; i<count; i++){
		float degree=RAD2DEG(scan->angle_min+scan->angle_increment*i); //degree unit : degree
		float next_degree=RAD2DEG(scan->angle_min+scan->angle_increment*(i+1));
		//scan->increment = 0.184995deg, count 1946, range_min=0.150000m
		//if(((degree>=180-Lidar_Angle_Range)&&(degree<=180-30))||((degree<=180+Lidar_Angle_Range)&&(degree>=180+30))&&(scan->ranges[i]<=Lidar_Obstacle_Range)){
		if((degree>=180-Lidar_Angle_Range)&&(degree<=180+Lidar_Angle_Range)&&(scan->ranges[i]<=Lidar_Obstacle_Range)){
			count_object_Count++;
			float point_x=-1*scan->ranges[i]*sin((degree-180.0)*M_PI/180.0);
			float point_y=scan->ranges[i]*cos((degree-180.0)*M_PI/180.0);
			stack_pos_x+=point_x;
			stack_pos_y+=point_y;
			
			if(count_object_Count==1){
				start_coordinate[0][0]=point_x;
				start_coordinate[0][1]=point_y;
			}
			
			//if(((next_degree>180-Lidar_Angle_Range)&&(next_degree<180-30))||((next_degree>180+Lidar_Angle_Range)&&(next_degree<180+30))&&(scan->ranges[i]<=Lidar_Obstacle_Range)){
			if((next_degree>=180-Lidar_Angle_Range)&&(next_degree<=180+Lidar_Angle_Range)&&(scan->ranges[i+1]>Lidar_Obstacle_Range)){
				end_coordinate[0][0]=point_x;
				end_coordinate[0][1]=point_y;
				object_size=sqrt(pow(end_coordinate[0][0]-start_coordinate[0][0], 2)+pow(end_coordinate[0][1]-start_coordinate[0][1], 2));
				
				if(object_size<Object_Maximum_Size && object_size>Object_Minimum_Size){
					ROS_INFO("--------------------------------");
					ROS_INFO("Object Size : %f", object_size);
					ROS_INFO("-->p1 : (%f, %f)  |  p2 : (%f, %f)", start_coordinate[0][0], start_coordinate[0][1], end_coordinate[0][0], end_coordinate[0][1]);
					count_object++;
					object_coordinate[fir_detect_index][0]=round(stack_pos_x/(float)(count_object_Count)*100)/100; //x
					object_coordinate[fir_detect_index][1]=round(stack_pos_y/(float)(count_object_Count)*100)/100; //y
					//ROS_INFO("First_detect_index : %d", fir_detect_index);
					fir_detect_index++;
					memset(start_coordinate, 0, sizeof(start_coordinate));
					memset(end_coordinate, 0, sizeof(end_coordinate));
					stack_pos_x=0.0;
					stack_pos_y=0.0;
					count_object_Count=0;
					object_size=0.0;
				}
			}
		}	
	}
	ROS_INFO("Object Number : %d", count_object);
	for(int i=0; i<count_object; i++){
		ROS_INFO("%d. x = %f | y = %f", i+1, object_coordinate[i][0], object_coordinate[i][1]);
	}
	
	cal_waypoint();
	
	//ROS_INFO("Inclination : %f	%f", fir_inclination, sec_inclination);
	//ROS_INFO("Inclination Angle : %f %f", fir_inclination_angle, sec_inclination_angle);
	
	fir_detect_index=0;
	memset(object_coordinate, 0, sizeof(object_coordinate));
}

int main(int argc, char **argv){
	ros::init(argc, argv, "Lidar_Detect");
	ros::NodeHandle nh;

	std_msgs::Float32 cones_inclination_angle;

	ros::Subscriber lidar_info_sub=nh.subscribe("/scan", 100, &scanCallback);
	ros::Publisher pub_inclination_angle=nh.advertise<std_msgs::Float32>("/inclination_angle", 10);
	
	ros::Rate loop_rate(5);
	
	while(ros::ok()){
		ROS_INFO("Average Inclination Angle : %f", average_inclination_angle);
		ROS_INFO("--------------------------------");
		cones_inclination_angle.data=average_inclination_angle*0.500+0.1; //0.477
		pub_inclination_angle.publish(cones_inclination_angle);
		
		average_inclination_angle=0.0;
		
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}

