#include "PathPlaner.h"
//#include "helpers.h"


PATHPLANER::PATHPLANER(SENSOR_FUSION_LIST perception_data,double time_step){
    m_vehicle_localization = perception_data;
	m_time_step = time_step;
    cout<<"Hello Path Planer!"<<endl;

}

/*
PATHPLANER::PATHPLANER(int ID){
    m_ID = ID;
    cout<<"Hello Path Planer!"<<endl;
    cout<<"The ID is"<<m_ID<<endl;
}
*/
PATHPLANER::~PATHPLANER(){

}

vector<float>PATHPLANER::predication(vector<vector<double>> sensor_fusion,int path_size, const double &lane, double future_car_s){
	float too_close = -1;
	float targete_vel = -1;
	double too_close_min = 1000;
	for(int i=0; i<sensor_fusion.size(); i++){
		double d = sensor_fusion[i][6];
		// Main vehicle is in the middle lane at the beginning of running
		if(d>4*lane && d<4*lane+4){
			double front_car_vx = sensor_fusion[i][3];
			double front_car_vy = sensor_fusion[i][4];
			double front_car_v = sqrt(front_car_vx*front_car_vx + front_car_vy*front_car_vy);
			double front_car_s = sensor_fusion[i][5];

			double future_front_car_s = front_car_s+front_car_v*0.02*path_size;

			if(future_front_car_s > future_car_s){
				if((future_front_car_s - future_car_s) < too_close_min){
					too_close = future_front_car_s - future_car_s;
					too_close_min = too_close;
					targete_vel = front_car_v;
				}
			}
		}
	}
	//cout<<too_close<<"  "<<targete_vel<<endl;

	return {too_close, targete_vel};
}

vector<float>PATHPLANER::predication_within_lane(int path_size, const double &lane, double future_car_s){
	float too_close = -1;
	float targete_vel = -1;
	double too_close_min = 100;//m
	for(int i=0; i<m_vehicle_localization.perception.size(); i++){
		double car_position_d = m_vehicle_localization.perception.at(i).car_position_d;
		if(car_position_d>4*lane && car_position_d<4*lane+4){

			double car_velocity_x = m_vehicle_localization.perception.at(i).car_velocity_x;
			double car_velocity_y = m_vehicle_localization.perception.at(i).car_velocity_y;
			double car_velocity = sqrt(car_velocity_x*car_velocity_x + car_velocity_y*car_velocity_y);
			double front_car_s = m_vehicle_localization.perception.at(i).car_position_s;

			double future_front_car_s = front_car_s+car_velocity*m_time_step*path_size;

			if(future_front_car_s > future_car_s){
				if((future_front_car_s - future_car_s) < too_close_min){
					too_close = future_front_car_s - future_car_s;
					too_close_min = too_close;
					targete_vel = car_velocity;
				}
			}
		}
	}

	return {too_close, targete_vel};
}

vector<float> PATHPLANER::predication_neighbour_lane(int path_size, const double &lane, double future_car_s){

vector<double> future_s_left_car, future_s_right_car;
	vector<double> future_v_left_car, future_v_right_car;
	for(int i=0; i<m_vehicle_localization.perception.size(); i++){
		if(lane > 0){
			//double d_lane_l = sensor_fusion[i][6];
			double d_lane_l = m_vehicle_localization.perception.at(i).car_position_d;
			if(d_lane_l >4*(lane-1) && d_lane_l<4*(lane-1)+4){

				double car_velocity_x = m_vehicle_localization.perception.at(i).car_velocity_x;
				double car_velocity_y = m_vehicle_localization.perception.at(i).car_velocity_y;
				double car_velocity = sqrt(car_velocity_x*car_velocity_x + car_velocity_y*car_velocity_y);
				double front_car_s = m_vehicle_localization.perception.at(i).car_position_s;

				double future_left_car_s = front_car_s+car_velocity*m_time_step*path_size;


				future_s_left_car.push_back(future_left_car_s);
				future_v_left_car.push_back(car_velocity);
			}
		}


		if(lane <2){
			double d_lane_r = m_vehicle_localization.perception.at(i).car_position_d;
			if(d_lane_r >4*(lane+1) && d_lane_r<4*(lane+1)+4){


				double car_velocity_x = m_vehicle_localization.perception.at(i).car_velocity_x;
				double car_velocity_y = m_vehicle_localization.perception.at(i).car_velocity_y;
				double car_velocity = sqrt(car_velocity_x*car_velocity_x + car_velocity_y*car_velocity_y);
				double right_car_s = m_vehicle_localization.perception.at(i).car_position_s;

				double future_right_car_s = right_car_s+car_velocity*m_time_step*path_size;

				future_s_right_car.push_back(future_right_car_s);
				future_v_right_car.push_back(car_velocity);
			}
		}
	}


	//choice the car front and behind our car in the future time(the curent path end point).
	float left_front_close = -1;
	float left_front_vel = -1;
	float left_after_close = -1;
	float left_after_vel = -1;
	float left_front_min = 1000;
	float left_after_min = 1000;
	for( int i = 0; i<future_s_left_car.size(); i++){

		if(future_s_left_car[i] > future_car_s){
			if((future_s_left_car[i] - future_car_s)< left_front_min){

				left_front_close = future_s_left_car[i] - future_car_s;
				left_front_min = left_front_close;
				left_front_vel = future_v_left_car[i];
			}
		}
		
		if(future_s_left_car[i] < future_car_s){
		
			if((future_car_s - future_s_left_car[i])< left_after_min){
				left_after_close = future_car_s- future_s_left_car[i];
				left_after_min = left_after_close;
				left_after_vel = future_v_left_car[i];
			}
		}
		
	}

	float right_front_close = -1;
	float right_front_vel = -1;
	float right_after_close = -1;
	float right_after_vel = -1;
	float right_front_min = 1000;
	float right_after_min = 1000;
	for( int i = 0; i<future_s_right_car.size(); i++){

		if(future_s_right_car[i] > future_car_s){
			if((future_s_right_car[i] - future_car_s)< right_front_min){
				right_front_close = future_s_right_car[i] - future_car_s;
				right_front_min = right_front_close;
				right_front_vel = future_v_right_car[i];
			}
		}

		if(future_s_right_car[i] < future_car_s){
			if((future_car_s - future_s_right_car[i])< right_after_min){
				right_after_close = future_car_s- future_s_right_car[i];
				right_after_min = right_after_close;
				right_after_vel = future_v_right_car[i];
			}
		}
	}
	//cout<<left_front_close<<"  "<<right_front_close<<endl;

	return {left_front_close, right_front_close, left_after_close, right_after_close, 
       		left_front_vel, right_front_vel, left_after_vel,right_after_vel};
}


bool PATHPLANER::veicle_state_machine(vector<float> prediction_front, vector<float> prediction_left_right,double &ref_vel, double &lane, double max_v){
	double weight_crash = 20;
	double weight_save_time = 50;
	double weight_buffer = 1000;

	double lane_before = lane;
	double cost_keep_lane,cost_change_left, cost_change_right;

	//if have no vehicle or vehicle is vary far away in front of our car, just drive as fast as we can,
	//no need to calculate the three states cost.
	if(prediction_front[0] == -1 || prediction_front[0]>30){   
		if(ref_vel<max_v){
			ref_vel +=0.23;
		}
	}

	// if finde vehicle in front of us, calculate the cost of the three states, choose the min cost state.
	
	// *****************the three states:keep_lane, change_left, change_right.************
	
	else{
	       //when close to front vehicle,and it fast than our car, no need to slow down..	
		if(ref_vel<max_v + 0.5){
			if(prediction_front[1] > ref_vel){
				ref_vel +=0.23;
			}
			//other conditions, when close to front vehicle slow down the acc according to the diff of v.
			else{
				ref_vel -=exp((ref_vel - prediction_front[1])/(max_v+0.5))/8.0;
			}
		}


		cost_keep_lane = cost_of_all(weight_crash, weight_buffer, weight_save_time,
				prediction_front[0], -1, ref_vel, prediction_front[1], -1, max_v);


		if(lane > 0){
			//plus 12 to the cost of change lang, avoid frequent lane changes when cost_change_left
			// and cost_change_right wave aroud cost_keep_lane.
			cost_change_left =10 + cost_of_all(weight_crash, weight_buffer, weight_save_time,
					prediction_left_right[0], prediction_left_right[2], ref_vel,
					prediction_left_right[4],prediction_left_right[6], max_v);
		}
		else{
			cost_change_left = 1000;
		}



		if(lane < 2){
			cost_change_right =12 +  cost_of_all(weight_crash,weight_buffer, weight_save_time,
					prediction_left_right[1],prediction_left_right[3], ref_vel,
					prediction_left_right[5], prediction_left_right[7], max_v);
		}
		else{
			cost_change_right = 1000;
		}
	

 		bool change_left_state = false;
		bool change_right_state = false;

		//set default state keep_lane.
		double min_cost;
		min_cost = cost_keep_lane;

		if(cost_change_left<min_cost){
			min_cost = cost_change_left;
			change_left_state = true;
		}
		if(cost_change_right < min_cost){
			change_right_state = true;
		}

		//when cost_change_right is min, change to right.
		if(change_right_state){
			if(lane < 2){
				lane += 1;
			}
		} 

		//when cost_change_left is min, change to left.
		if(change_left_state && !change_right_state){
			if(lane > 0){
				lane -= 1;
			}
		}
	}
	
	double lane_after = lane;
	bool change_lane = false;
	if(abs(lane_before - lane_after) > 0.1){
		change_lane = true;
	}

	return change_lane;
}

vector<vector<double>> PATHPLANER::trajectories_sample(
								  const vector<double> &pre_path_x, 
								  const vector<double> &pre_path_y, 
								  double ref_x,
								  double ref_y, 
								  double ref_yaw, 
								  bool change_lane, 
								  int PATH_SIZE,
								  double car_s, 
								  const vector<double> &map_waypoints_x,
								  const vector<double> &map_waypoints_y, 
								  const vector<double> &map_waypoints_s, 
								  double lane, double ref_vel){
    vector<double> spine_x,spine_y;
	vector<double> next_x_vals, next_y_vals;
	vector<vector<double>> trjectoryXY;
	int path_size = pre_path_x.size();

	if(path_size < 5){
		double pos_x = ref_x - cos(ref_yaw);
		double pos_y = ref_y - sin(ref_yaw);
		spine_x.push_back(pos_x);
		spine_y.push_back(pos_y);

		spine_x.push_back(ref_x);
		spine_y.push_back(ref_y);
	}
	else{
		ref_x = pre_path_x[path_size-1];
		ref_y = pre_path_y[path_size-1];
		double pre_ref_x = pre_path_x[path_size-5];
		double pre_ref_y = pre_path_y[path_size-5];

		spine_x.push_back(pre_ref_x);
		spine_y.push_back(pre_ref_y);

		spine_x.push_back(ref_x);
		spine_y.push_back(ref_y);

		ref_yaw = atan2(ref_y - pre_ref_y, ref_x - pre_ref_x);
	}

	vector<double> getxy1,getxy2,getxy3;
	if(change_lane = true){  //set further points to get more smooth trajectory, avoid large lateral jerk.
		getxy1= getXY(car_s + 50, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		getxy2= getXY(car_s + 70, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		getxy3= getXY(car_s + 90, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	}
	else{
		getxy1= getXY(car_s + 30, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		getxy2= getXY(car_s + 60, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		getxy3= getXY(car_s + 90, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	}
	spine_x.push_back(getxy1[0]);
	spine_x.push_back(getxy2[0]);
	spine_x.push_back(getxy3[0]);

	spine_y.push_back(getxy1[1]);
	spine_y.push_back(getxy2[1]);
	spine_y.push_back(getxy3[1]);

	//Convert the global coordinate system to the vichle coordinate for math easy.
	for(int i =0; i<spine_x.size(); i++){
		double shifto_car_x = spine_x[i] - ref_x;
		double shifto_car_y = spine_y[i] - ref_y;

		spine_x[i] = shifto_car_x*cos(0-ref_yaw) - shifto_car_y*sin(0-ref_yaw);
		spine_y[i] = shifto_car_x*sin(0-ref_yaw) + shifto_car_y*cos(0-ref_yaw);
	}

	tk::spline s;
	s.set_points(spine_x, spine_y);

	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);
	double x_add = 0;

	for(int i = 0; i<pre_path_x.size(); i++) {
		next_x_vals.push_back(pre_path_x[i]);
		next_y_vals.push_back(pre_path_y[i]);
	}

	for(int i = 1; i<= PATH_SIZE - pre_path_x.size(); i++){
		double num = target_dist/(0.02*ref_vel/2.24);
		double x_point = x_add + target_x/num;
		double y_point = s(x_point);

		x_add = x_point;
		double x_ref = x_point;
		double y_ref = y_point;

		x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
		y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}

	trjectoryXY.push_back(next_x_vals);
	trjectoryXY.push_back(next_y_vals);

	return trjectoryXY;
//cout<<"Hello TS!"<<endl;
}
