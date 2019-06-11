#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
//#include "spline.h"
#include "helpers.h"

#include "PathPlaner.h"

#include "prediction.h"
#include "vehicle.h"
#include "trajectory.h"

using namespace std;
using json = nlohmann::json;
int main() {
  uWS::Hub h;

	cout<<"Hello Udacity PPP"<<endl;
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
	
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  double lane = 1;
  double ref_vel = 0;
  h.onMessage([&ref_vel,&lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      SENSOR_FUSION_LIST sensor_data;
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {

          //sensor_data.perception.capacity;
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

            sensor_data.main_car_localization.car_position_x = j[1]["x"];
            sensor_data.main_car_localization.car_position_y = j[1]["y"];
            sensor_data.main_car_localization.car_position_s = j[1]["s"];
            sensor_data.main_car_localization.car_position_d = j[1]["d"];
            sensor_data.main_car_localization.car_yaw = j[1]["yaw"];
            sensor_data.main_car_localization.car_velocity = j[1]["speed"];

            cout<<"Main car's location"<<","<<car_x<<","<<car_y<<","<<car_s<<","<<car_d<<","<<car_yaw<<","<<car_speed<<endl;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

            cout<<"Previous path"<<","<<previous_path_x<<","<<previous_path_y<<","<<end_path_s<<","<<end_path_d<<endl;

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            vector<SENSOR_FUSION> perception_test(20);
            
            //sensor_data.perception.at(0).car_ID = 1;//out of range
            //SENSOR_TEST testValue(10);
            //testValue.vector_test
            //SENSOR_FUSION_LIST * test_data_set = new SENSOR_FUSION_LIST;   
            //test_data_set->perception.at(0).car_ID = 1;          

            cout<<"Sensor Fusion"<<",";
            for(int i = 0;i<sensor_fusion.size();i++){
              cout<<sensor_fusion.at(i)<<",";
              
              perception_test.at(i).car_ID = sensor_fusion.at(i)[0]; 
              perception_test.at(i).car_position_x = sensor_fusion.at(i)[1];
              perception_test.at(i).car_position_y = sensor_fusion.at(i)[2];
              perception_test.at(i).car_velocity_x = sensor_fusion.at(i)[3];
              perception_test.at(i).car_velocity_y = sensor_fusion.at(i)[4];
              perception_test.at(i).car_position_s = sensor_fusion.at(i)[5];
              perception_test.at(i).car_position_d = sensor_fusion.at(i)[6];

              sensor_data.perception.push_back(perception_test.at(i));
          
            }
            cout<<endl;

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	//define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
		//*************************************************//

		int PATH_SIZE = 30;
		double max_v = 49.5;

		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = car_yaw;

		int path_size = previous_path_x.size();
		double future_car_s = end_path_s;

    
    //vehicle_localization.car_ID = sensor_fusion.at()

    PATHPLANER lattice(sensor_data,0.02);
    //lattice.trajectories_sample();
		
		//get the diff of s and v betwen the front vehicle
    vector<float> predict_front = lattice.predication_within_lane(path_size, lane, future_car_s);

		//get the diff of s and v betwen the front and after vehicle, when our vehicle change lane to left or right.
    vector<float> predict_left_right = lattice.predication_neighbour_lane(path_size, lane, future_car_s);

		//Using finite state machines and cost function to choose the best state.
		//the ref_vel and lane will be changed in this function.
		//return bool change_lane, representative the lane change or not.
		bool change_lane = lattice.veicle_state_machine(predict_front, predict_left_right, ref_vel, lane, max_v);
		//generate the trajectory points.
    vector<vector<double>> traject = lattice.trajectories_sample(previous_path_x, previous_path_y, ref_x, ref_y, ref_yaw, change_lane,PATH_SIZE, car_s, map_waypoints_x, map_waypoints_y, map_waypoints_s, lane, ref_vel);

    next_x_vals = traject[0];
    next_y_vals = traject[1];
		//****************************************************//
            

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
