#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


// our car's lane
int lane = 1;
// reference velocity
double ref_vel = 0;
// change lane counter
double change_count = 0;


int main() {

  uWS::Hub h;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // the size of previous path points
          int pre_size = previous_path_x.size();

          // preventing collitions
          if (pre_size > 0){

          	car_s = end_path_s;
          
          }

          // too close flag
          bool too_close = false ;
          // change lane flag
          bool is_change = false;
          // detect if there are vehicles on the right lane
          bool is_right = false;
          // detect if there are vehicles on the left lane
          bool is_left = false;

          // ahead car's speed
          float ahead_speed = 0.0;

          // analyze the vehicles detected by the sensor
          for (int i =0;i< sensor_fusion.size();i++){

          	// get the information of the vehicles
          	float vd = sensor_fusion[i][6];
          	double vx = sensor_fusion[i][3];
          	double vy = sensor_fusion[i][4];
          	double check_car_s = sensor_fusion[i][5];


          	double check_speed = sqrt(vx*vx + vy*vy);
          	check_car_s += ((double)pre_size * 0.02 * check_speed);

          	double dis_car_s = check_car_s - car_s;
          	double dis_car_d = vd - car_d;

          	// determine if it's too close to our car
          	if(dis_car_s > - 18 &&  dis_car_s < 30){


          		// the same lane of our car
          		if (dis_car_s > 0 && vd < (2 + 4*lane +2 ) && vd > (2+4*lane -2)){

          			too_close = true;
          			// get the ahead vehicle's speed
          			ahead_speed = check_speed ;

          			//std::cout << "too close!!!" << std::endl;

          		}else if (vd > 0 && dis_car_d > 0 && dis_car_d <= 6 ) {

					is_right = true ;

					//std::cout << "There are vehicles on the right lane !!" << std::endl;

				}else if (vd > 0 && dis_car_d <= 0 && dis_car_d >= -6 ){

					is_left = true;
					
					//std::cout << "There are vehicles on the left lane !!" << std::endl;

				}

          	}


          }


          // behavior planning
          if(too_close){

          	//slowdown
          	ref_vel -= 0.224;

          	// keep the ahead vehicle's speed
          	if (ref_vel < ahead_speed){

          		ref_vel = ahead_speed;
          	}

          	// change lane
          	is_change = true;
          	change_count += 2;

          }else if (ref_vel < 49.5){

          	// accelerated
          	ref_vel += 0.224;
          	change_count--;

          }else if (is_right == false && is_left == false && too_close == false && (car_d > 8 || car_d < 4)){

          	is_change = true;
          	change_count++;
          }else {

          	change_count--;
          }

          // set the boundary of change_count
          if (change_count < 0){

          	change_count = 0;
          }

          // judgment change lane
          if (is_change && change_count > 160){

          	change_count = 0;

          	// if our car's lane is 0 and no vehicle on the left lane
          	if (lane == 2 && is_left == false){
          		lane = 1;

          	// if our car's lane is 1 
          	}else if (lane == 1 ) {

          		// no vehicle on the left lane and there's vehicle on the right lane
          		if (is_left == false && is_right){

          			lane = 0;

          		// no vehicle on the right lane and there's vehicle on the left lane
          		}else if (is_right == false && is_left){

          			lane = 2;

          		// no vehicle on the left and right lane,but there's vehicle ahead
          		}else if (is_left == false && is_right == false && too_close) {

          			lane = 0;

          		}else{

          			lane = 1;
          		}

            // no vehicle on the right lane
          	}else if (lane == 0 && is_right == false ){

          		lane = 1;
          	}


          }

         
          //Trajectory generation
          vector <double> ptsx;
          vector <double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // using the previous points
          if (pre_size < 2){

          	double pre_car_x = car_x - cos(car_yaw);
          	double pre_car_y = car_y - sin(car_yaw);

          	ptsx.push_back(pre_car_x);
          	ptsx.push_back(car_x);

          	ptsy.push_back(pre_car_y);
          	ptsy.push_back(car_y);

          }else{

          	ref_x = previous_path_x[pre_size -1];
          	ref_y = previous_path_y[pre_size -1];

          	double ref_x_prev = previous_path_x[pre_size-2];
          	double ref_y_prev = previous_path_y[pre_size-2];
          	ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);

          	ptsx.push_back(ref_x_prev);
          	ptsx.push_back(ref_x);

          	ptsy.push_back(ref_y_prev);
          	ptsy.push_back(ref_y);

          } 

          //get the future points's XY values 
          vector <double> newpoint1 = getXY(car_s + 30,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector <double> newpoint2 = getXY(car_s + 60,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector <double> newpoint3 = getXY(car_s + 90,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(newpoint1[0]);
          ptsx.push_back(newpoint2[0]);
          ptsx.push_back(newpoint3[0]);

          ptsy.push_back(newpoint1[1]);
          ptsy.push_back(newpoint2[1]); 
          ptsy.push_back(newpoint3[1]);  

          //convert to vehicle's coordinate
          for (int i=0;i< ptsx.size();i++){

          	double x_diff = ptsx[i] - ref_x;
          	double y_diff = ptsy[i] - ref_y;

          	ptsx[i] = x_diff * cos(ref_yaw) + y_diff * sin(ref_yaw);
          	ptsy[i] = - x_diff * sin(ref_yaw) + y_diff * cos(ref_yaw);

          }

          //using the previous and future points to create spline interpolation
          tk::spline s;
          s.set_points(ptsx,ptsy); 

          // using the previous points as the next points of our car
          for(int i=0;i< previous_path_x.size();i++){

          	next_x_vals.push_back(previous_path_x[i]);
          	next_y_vals.push_back(previous_path_y[i]);
          }

          //calculate the our car's speed
	      double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x * target_x) + target_y * target_y);

          double x_add_on = 0;

          for (int i=0;i<= 50 - previous_path_x.size();i++){

          	double N = (target_dist / (0.02 * ref_vel /2.24));
          	double x_point = x_add_on + (target_x / N);
          	double y_point = s(x_point);

          	x_add_on = x_point;

          	// convert to global coordinate
          	double v_x = x_point;
          	double v_y = y_point;

          	x_point = v_x * cos(ref_yaw) - v_y * sin(ref_yaw);
          	y_point = v_y * cos(ref_yaw) + v_x * sin(ref_yaw);

          	x_point += ref_x;
          	y_point += ref_y;

          	// add to next points vector
          	next_x_vals.push_back(x_point);
          	next_y_vals.push_back(y_point);

          }


		  //end todo
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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