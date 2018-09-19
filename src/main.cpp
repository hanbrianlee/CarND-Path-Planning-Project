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
#include "spline.h"


#define INITIAL_DESIRED_VELOCITY 49.5 //mph
//define finite state machine predefinitions
#define KL 0 //keep lane
#define PLCL 1 //plan lane change left
#define PLCR 2 //plan lane change right
#define LCL 3 //lane change left
#define LCR 4 //lane change right
#define LEFT_LANE 0
#define RIGHT_LANE 1
#define MY_LANE 2
#define NONE 99
#define CARS_OF_CONCERN_DISTANCE 50.0
#define ALLOWED_DISTANCE 30.0
#define SAFE_GAP_TO_CL 20.0
#define VEL_INC 0.4
#define IN_LANE_TOLERANCE_D 0.5
//testing git commit with pycharm

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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

  //declare and define lane
  int lane = 2; //by default lane 2 is the leftmost lane (2 out of 3 lanes)

  //declare and efine reference velocity which will be the requested/expected velocity
  double ref_vel = 0; //mph - start at 0 and later accelerate slowly

  int fsm = KL;

  double desired_vel = INITIAL_DESIRED_VELOCITY;

  bool transition = false;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &fsm, &desired_vel, &transition](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	int prev_size = previous_path_x.size();

          	if(prev_size > 0)
            {
          	    car_s = end_path_s;
            }

            //extracting actors in my scene from here
            struct car {
          	    double vel;
          	    double s;
          	};

          	vector<car> cars_left;
            vector<car> cars_mylane;
            vector<car> cars_right;

          	//cars_left = {{0.0,0.0,NONE}}; //initialize to no cars

            bool too_close = false;

          	//for each car detected, determine if the car is of concern or not, as well as assigning lanes to them for each frame.
          	for (int i = 0; i < sensor_fusion.size(); i++)
            {
          	    float d = sensor_fusion[i][6];
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_vel = sqrt(vx*vx + vy*vy);
                double check_s = sensor_fusion[i][5];

                check_s += ((double)prev_size*0.02*check_vel); //predict where the car will be, as the previous points are being used still.
                if((fabs(check_s - car_s) < CARS_OF_CONCERN_DISTANCE)) //only concern the cars that are close to me
                {
                    if (d < (4 * lane) && d > (4 * lane - 4)) //if the car is within my lane.. lane = 1~3
                    {
                        cars_mylane.push_back({check_vel, check_s});
                    } else if ((d < (4 * lane) - 4) && (d > (4 * lane - 4 - 4)) &&
                               (lane > 1)) //if car is on my left lane
                    {
                        cars_left.push_back({check_vel, check_s});
                    } else if ((d < (4 * lane) + 4) && (d > (4 * lane - 4 + 4)) &&
                               (lane < 3)) //if car is on my right lane
                    {
                        cars_right.push_back({check_vel, check_s});
                    }
                }
            }

            //try printing all the cars identified
//            for(int i = 0; i < cars.size(); i++)
//            {
//                //only print if the car right lane and is close to me
//                if((cars[i].lane == RIGHT_LANE))
//                {
//                    std::cout << cars[i].vel*2.24 << endl;
//                    //std::cout << "my speed: " << car_speed << endl;
//                }
//
//            }



          	/********PERFORM GAP CHECKS *************/
            //for my lane
            double gap_mylane = 0.0; //initialize the gap as 0.0
            double front_car_speed = INITIAL_DESIRED_VELOCITY; //initialize to desired velocity 49.5
            if(cars_mylane.size() >= 2) //if more than or equal to 2 car are there
            {
                //std:: cout << "more than two cars! ";
                vector<double> s;
                //get the closest max 2 vehicles on the right side
                for(int i = 0; i < cars_mylane.size(); i++)
                {
                    s.push_back(fabs(cars_mylane[i].s - car_s));
                }
                sort(s.begin(), s.end());
                gap_mylane = s[0];

                /********FIND the distance to the first car in front of me********/
                sort( cars_mylane.begin(), cars_mylane.end(),
                      [](const car& p1, const car& p2){ return p1.s < p2.s; }
                );
                std::cout << "car1 distance: " << cars_mylane[0].s << ", car1 speed: " << cars_mylane[0].vel << ", car2 distance" << cars_mylane[1].s << ", car2 speed: " << cars_mylane[1].vel << endl;
                front_car_speed = cars_mylane[0].vel;
                //std::cout << "end";

            }
            else if(cars_mylane.size() == 1)
            {
                gap_mylane = fabs(car_s - cars_mylane[0].s);
                front_car_speed = cars_mylane[0].vel;
            }
            else //if there's no car
            {
                gap_mylane = CARS_OF_CONCERN_DISTANCE; //set the gap to large number to indicate it's good to move over
                //std::cout << gap_right << endl;
            }

            //for the right side
            double gap_right = 0.0; //initialize the gap as 0.0
            if(cars_right.size() >= 2) //if more than or equal to 2 cars are there
            {
                //std:: cout << "more than two cars! ";
                vector<double> s;
                //get the closest max 2 vehicles on the right side
                for(int i = 0; i < cars_right.size(); i++)
                {
                    s.push_back(fabs(cars_right[i].s - car_s));
                }
                sort(s.begin(), s.end());
                gap_right = s[0];
            }
            else if(cars_right.size() == 1)
            {
                gap_right = fabs(car_s - cars_right[0].s);
            }
            else //if there's no car
            {
                gap_right = CARS_OF_CONCERN_DISTANCE; //set the gap to large number to indicate it's good to move over
                //std::cout << gap_right << endl;
            }

            //for the left side
            double gap_left = 0.0; //initialize the gap as 0.0
            if(cars_left.size() >= 2) //if more than or equal to 2 cars are there
            {
                //std:: cout << "more than two cars! ";
                vector<double> s;
                //get the closest max 2 vehicles on the right side
                for(int i = 0; i < cars_left.size(); i++)
                {
                    s.push_back(fabs(cars_left[i].s - car_s));
                }
                sort(s.begin(), s.end());
                gap_left = s[0];
            }
            else if(cars_left.size() == 1)
            {
                gap_left = fabs(car_s - cars_left[0].s);
            }
            else //if there's no car
            {
                gap_left = CARS_OF_CONCERN_DISTANCE; //set the gap to large number to indicate it's good to move over
                //std::cout << gap_left << endl;
            }

            //std::cout << "gap_left: " << gap_left << ", gap_right: " << gap_right << endl;
            /**************** PERFORM GAP CHECKS END **********************/

            //path planning and state machine starts here
            if (fsm == KL)
            {
                std::cout << "let's do Keep Lane!!" << endl;
                //for all the cars of concern in front of us, if there is a car that is less than ALLOWED_DISTANCE from me and the speed is less than or equal to mine, then consider lane change
                for(int i = 0; i < cars_mylane.size(); i++)
                {
                    if((cars_mylane[i].s > car_s) && ((cars_mylane[i].s - car_s) < ALLOWED_DISTANCE) && (cars_mylane[i].vel <= (desired_vel - 5))) //-5 as a buffer
                    {
                        //consider lane change, so either PLCL or PLCR
                        //for PLCL: if the predicted gap on the left lane is okay AND if the predicted distance to car closest to me on the left lane is less than that to the right
                        //for PLCR: if the predicted gap on the right lane is okay AND if the predicted distance to car closest to me on the right lane is less than that to the left
                        //first check PLCR condition, if not default to PLCL as left lane should be usually more favorable
                        too_close = true;

                        if(((gap_right > gap_left) && (lane != 3)) || (lane == 1)) //if the gap on the right is better than the left AND i'm not on the rightmost lane, then do PLCR, or if I'm at the leftmost lane
                        {
                            fsm = PLCR;
                        }
                        else if((lane != 1) || (lane==3)) //else, do PLCL if i'm not in the leftmost lane OR if i'm at the rightmost lane
                        {
                            fsm = PLCL;
                        }
                        else
                        {
                            //else, do nothing, keep current state
                        }
                        //fsm = PLCL;
                    }
                }
                //just try to approach the desired velocity if no slow car is in front of me
                if(too_close)
                {
                    ref_vel -= VEL_INC; //approximately 5m/s^2
                }
                else if(ref_vel < desired_vel) //desired_vel is dynamic based on FSM behaviour
                {
                    ref_vel += VEL_INC; //approximately 5m/s^2
                }
            }
            else if (fsm == PLCL)
            {
                std::cout << "let's do PLCL!! <--- left" << endl;


                if(gap_left > SAFE_GAP_TO_CL) //condition to transition to LCL state
                {
                    fsm = LCL; //if there's enough gap on the right lane, perform LCR!
                }

                //go back to KL state if the following conditions are met:
                //if the gap on the other side seems favorable OR
                //if too much time has elapsed in this state OR (maybe later)
                //if the car in front of us went to our desired speed OR (maybe later)
                //if the car in front of us has given us enough space to accelerate: greater than a certain amount
                if(((gap_right > gap_left) && (lane != 3)) || (gap_mylane > ALLOWED_DISTANCE)) //condition to transition back to KL state, must invalidate the gap_right > gap_left check when i'm in the rightmost lane, otherwise gap_right is big number
                {
                    fsm = KL;
                }

                //if not the above condition, stay within this state, where you would keep looking for chances to lane change
                //just keep slowing down until the gap_left gets big enough so it's safe to go into that lane, making sure your speed is at least the speed of the car in front
                if(car_speed > front_car_speed*0.9) //10% speed buffer
                {
                    ref_vel -= VEL_INC; //approximately 5m/s^2
                    std::cout << "ref_vel: " << ref_vel*2.24 << ", car_speed: " << car_speed << ", front_car_speed: " << front_car_speed << endl;
                }
            }
            else if (fsm == PLCR)
            {
                std::cout << "let's do PLCR ---> right" << endl;
                //ref_vel -= VEL_INC; //approximately 5m/s^2

                if(gap_right > SAFE_GAP_TO_CL) //condition to transition to LCR state, try.. only change lane if there's no car of concern!
                {
                    fsm = LCR; //if there's enough gap on the right lane, perform LCR!
                }

                //go back to KL state if the following conditions are met:
                //if the gap on the other side seems favorable OR
                //if too much time has elapsed in this state OR (maybe later)
                //if the car in front of us went to our desired speed OR (maybe later)
                //if the car in front of us has given us enough space to accelerate: greater than a certain amount
                if(((gap_left > gap_right) && (lane != 1)) || (gap_mylane > ALLOWED_DISTANCE)) //condition to transition back to KL state, must invalidate the gap_right < gap_left check when i'm in the leftmost lane, otherwise gap_left is big number
                {
                    fsm = KL;
                }

                //if not the above condition, stay within this state, where you would keep looking for chances to lane change
                //just keep slowing down until the gap_right gets big enough so it's safe to go into that lane, making sure your speed is at least the speed of the car in front
                if(car_speed > front_car_speed*0.9) //10% speed buffer
                {
                    ref_vel -= VEL_INC; //approximately 5m/s^2
                    std::cout << "ref_vel: " << ref_vel*2.24 << ", car_speed: " << car_speed << ", front_car_speed: " << front_car_speed << endl;
                }
            }
            //#TODO: Make in-transition flags so that in the midst of lane change, another Lane change cannot happen, if this ahppens, then the car can oscillate between two lanes and stay somewhere in the middle, not desired
            else if (fsm == LCR)
            {
                std::cout << "let's do LCR ---> right NOW!!" << endl;
                //change lane to right
                if(transition == false)
                {
                    transition = true;
                    lane = lane + 1;
                }

                //during lane change, press on the gas as usual, not exceeding the disired_vel of 49.5
                /*TODO: must account for situations where there's a car going slow close to me in the lane i'm changing into, same for LCL case.**/
                if(ref_vel < desired_vel) //desired_vel is dynamic based on FSM behaviour
                {
                    ref_vel += VEL_INC; //approximately 5m/s^2
                }

                if(fabs(car_d - (-2 + 4*lane)) < IN_LANE_TOLERANCE_D)
                {
                    //after completing lane change, go back to keep lane state
                    fsm = KL;
                    //reset transition flag, meaning completion of change lane
                    transition = false;
                }

            }
            else if (fsm == LCL)
            {
                std::cout << "let's do LCL <--- left NOW!!" << endl;
                //change lane to right
                if(transition == false)
                {
                    transition = true;
                    lane = lane - 1;
                }

                //during lane change, press on the gas as usual, not exceeding the disired_vel of 49.5
                if(ref_vel < desired_vel) //desired_vel is dynamic based on FSM behaviour
                {
                    ref_vel += VEL_INC; //approximately 5m/s^2
                }

                if(fabs(car_d - (-2 + 4*lane)) < IN_LANE_TOLERANCE_D)
                {
                    //after completing lane change, go back to keep lane state
                    fsm = KL;
                    //reset transition flag, meaning completion of change lane
                    transition = false;
                }
            }




//
//            for(int i = 0; i < cars_right.size(); i++)
//            {
//                //only print if the car right lane and is close to me
//                if((cars_right[i].lane == RIGHT_LANE))
//                {
//                    std::cout << cars_right[i].vel*2.24 << endl;
//                    //std::cout << "my speed: " << car_speed << endl;
//                }
//            }
//            for(int i = 0; i < cars_right.size(); i++)
//            {
//                //only print if the car right lane and is close to me
//                if((cars_right[i].lane == RIGHT_LANE))
//                {
//                    std::cout << cars_right[i].vel*2.24 << endl;
//                    //std::cout << "my speed: " << car_speed << endl;
//                }
//            }
//            for(int i = 0; i < cars_mylane.size(); i++)
//            {
//                std::cout << cars_mylane[i].vel*2.24 << endl;
//            }


            //from here, the following code is for drawing trajectories

          	json msgJson;

          	vector<double> ptsx;
          	vector<double> ptsy;

          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);

            //std::cout << ref_y << endl;

          	if(prev_size < 2)
            {
          	    std::cout << "i'm here" <<endl;
          	    double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw); //make sure this is sin not cosine, otherwise the car will instantly try to go left after starting!!!

                ptsx.push_back(prev_car_x); //put in the previous point x
                ptsx.push_back(car_x);  //put in the current car's x position

                ptsy.push_back(prev_car_y); //put in the previous point y
                ptsy.push_back(car_y); //put in the current car's y position
            }
            else
            {
                ref_x = previous_path_x[prev_size-1]; //access the last x (remaining..)
                ref_y = previous_path_y[prev_size-1]; //access the last y (remaining..)

                double ref_x_prev = previous_path_x[prev_size-2];
                double ref_y_prev = previous_path_y[prev_size-2];

                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }

            vector<double> next_wp0 = getXY(car_s + 30, (-2 + 4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, (-2 + 4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, (-2 + 4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            //convert all the waypoints with respect to the reference x,y, which would be either the last points from previous path or the current car path depending on the previous path size
            for (int i = 0; i < ptsx.size(); i++)
            {
                //shift car reference angle to 0 degrees
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;

                ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
                ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
            }

            tk::spline s;

            s.set_points(ptsx,ptsy);

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            for(int i = 0; i < previous_path_x.size(); i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x) + (target_y) * (target_y));

            double x_add_on = 0;

            for (int i = 1; i < 50-previous_path_x.size(); i++)
            {
                double N = (target_dist / (0.02*ref_vel/2.24));
                double x_point = x_add_on + (target_x)/N;
                double y_point = s(x_point);

                x_add_on = x_point; //update the x_add_on so that the next spline point can be selected

                //perform conversion back from ego vehicle frame to global frame
                double x_ref = x_point;
                double y_ref = y_point;

                x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;
                //std::cout << y_point << endl;

                //push the results into next_x_vals and next_y_vals so they can be sent to the simulator
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	//trying to follow the lane taking advantage of frenet coordinate
//          	double dist_inc = 0.3;
//          	for(int i=0; i<50; i++)
//            {
//          	    double next_s = car_s + dist_inc*(i); //adding +1 to i otherwise, we will be doing car_s + 0 for the very first waypoint which will serve no purpose.. but still works without this +1
//                double next_d = 6;
//                vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//          	    next_x_vals.push_back(xy[0]);
//                next_y_vals.push_back(xy[1]);
//            }
//
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
