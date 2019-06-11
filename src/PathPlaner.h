#ifndef PATHPLANER_H
#define PATHPLANER_H

#include <vector>
#include <math.h>
#include <iostream>

#include "spline.h"
//#include "helpers.h"

using namespace std;

struct SENSOR_FUSION{
    int car_ID;
    double car_position_x;
    double car_position_y;
    double car_velocity_x;
    double car_velocity_y;
    double car_position_s;
    double car_position_d;
    // for Main vehicle
    double car_yaw;
    double car_velocity;
};

class SENSOR_FUSION_LIST{
    public:
        vector<SENSOR_FUSION> perception;
        SENSOR_FUSION main_car_localization;

    private:
        int m_size;

};

class PATHPLANER{
    public:
        PATHPLANER(SENSOR_FUSION_LIST perception_data,double time_step);
        ~PATHPLANER();


        vector<float> predication(vector<vector<double>> sensor_fusion,int path_size, const double &lane, double future_car_s);
        vector<float> predication_within_lane(int path_size, const double &lane, double future_car_s);
        vector<float> predication_neighbour_lane(int path_size, const double &lane, double future_car_s);
        bool veicle_state_machine(vector<float> prediction_front, vector<float> prediction_left_right,double &ref_vel, double &lane, double max_v);
        vector<vector<double>> trajectories_sample(const vector<double> &pre_path_x, 
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
								  double lane, double ref_vel);

    private:
        inline double cost_buffer(double weight_buffer, double diff_s)
        {
	        double cost_buf = -1;
            if(diff_s >0){
            cost_buf =(1.0/diff_s)*weight_buffer;
            }
            return cost_buf;
        }

        inline double cost_crash(double weight_crash, double diff_front_s,double diff_after_s)
        {
            double cost_crash_front = -1;
            double cost_crash_after = -1;

            if(diff_front_s > 0){
                if(diff_front_s<15){
                    cost_crash_front = (20 - diff_front_s)*weight_crash;
                }
            }

            if(diff_after_s > 0){
                if(diff_after_s<15){
                    cost_crash_after = (20 - diff_after_s)*weight_crash;
                }
            }
        
            double cost_crash_max = max(cost_crash_front,cost_crash_after);

            return cost_crash_max;
        }

        inline double cost_save_time(double weight_save_time, double front_vehicle_v, double max_v)
        {
	        double cost_savetime = -1;
	        if(front_vehicle_v > 0){
		        cost_savetime = weight_save_time*(max_v - front_vehicle_v)/max_v;
		    }

	        return cost_savetime;
        }
        inline double cost_of_all(double weight_crash, double weight_buffer, double weight_save_time, 
		                          double diff_front_s, double diff_after_s, double our_vehicle_v,
	                              double front_vehicle_v, double after_vehicle_v, double max_v)
        {

            double cost_cra = cost_crash(weight_crash, diff_front_s, diff_after_s);
            double cost_buf = cost_buffer(weight_buffer, diff_front_s);
            double cost_sav = cost_save_time(weight_save_time, front_vehicle_v, max_v);
            double cost_all = cost_buf + cost_sav + cost_cra;

            return cost_all;
        }
        // Transform from Frenet s,d coordinates to Cartesian x,y
        inline vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                            const vector<double> &maps_x, 
                            const vector<double> &maps_y) {
        int prev_wp = -1;

        while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
            ++prev_wp;
        }

        int wp2 = (prev_wp+1)%maps_x.size();

        double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                                (maps_x[wp2]-maps_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s-maps_s[prev_wp]);

        double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
        double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

        double perp_heading = heading-M_PI/2;

        double x = seg_x + d*cos(perp_heading);
        double y = seg_y + d*sin(perp_heading);

        return {x,y};
        }



    private:
        SENSOR_FUSION_LIST m_vehicle_localization;
        double m_time_step;
};


/*
class SENSOR_TEST{
    public:
        SENSOR_TEST(int size){
            m_size = size;
        }

        vector<double> vector_test(m_size);
    private:
        int m_size;
};
*/


#endif  // PATHPLANER_H