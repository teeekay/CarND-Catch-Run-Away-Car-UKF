#include "json.hpp"
#include "ukf.h"
#include <iostream>
#include <math.h>
#include <uWS/uWS.h>

using namespace std;

double target_vel;
double target_yaw;
double target_yawd;
///* x,y co-ords for nxt target
VectorXd nxt_target;
VectorXd hunter_loc;

inline double wrapAngletoPI( double in_angle );

inline double wrapAngletoPI( double in_angle )
{

	double out_angle = in_angle - 2. * M_PI * floor((in_angle + M_PI) / (2. * M_PI));
	if (out_angle != in_angle){
		cout << "wrapAngletoPI: Adjusted in_angle " << in_angle << " to out_angle " << out_angle << "." << endl;
	}
	return out_angle;
}

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("]");
	if (found_null != std::string::npos)
		return "";
	else if (b1 != std::string::npos && b2 != std::string::npos)
		return s.substr(b1, b2 - b1 + 1);
	return "";
}

int main()
{
  hunter_loc = VectorXd(2);
	uWS::Hub h;

	// Create a UKF instance
	UKF ukf;

	double target_x = 0.0;
	double target_y = 0.0;


	h.onMessage([&ukf,&target_x,&target_y](uWS::WebSocket<uWS::SERVER> ws,
	                                       char* data, size_t length,
	                                       uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event

		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{
		  auto s = hasData(std::string(data));
		  if (s != "")
		  {
		    auto j = json::parse(s);
		    std::string event = j[0].get<std::string>();

		    if (event == "telemetry")
		    {
		      // j[1] is the data JSON object
          double hunter_x = std::stod(j[1]["hunter_x"].get<std::string>());
					double hunter_y = std::stod(j[1]["hunter_y"].get<std::string>());
          double hunter_heading =	std::stod(j[1]["hunter_heading"].get<std::string>());
          string lidar_measurment = j[1]["lidar_measurement"];
          MeasurementPackage meas_package_L;
		      istringstream iss_L(lidar_measurment);
		      long long timestamp_L;

		      // reads first element from the current line
		      string sensor_type_L;
		      iss_L >> sensor_type_L;

		      // read measurements at this timestamp
		      meas_package_L.sensor_type_ = MeasurementPackage::LASER;
		      meas_package_L.raw_measurements_ =  VectorXd(2);
		      float px;
		      float py;
		      iss_L >> px;
          iss_L >> py;
          meas_package_L.raw_measurements_ << px, py;
          iss_L >> timestamp_L;
          meas_package_L.timestamp_ = timestamp_L;

          ukf.ProcessMeasurement(meas_package_L);

          string radar_measurment = j[1]["radar_measurement"];

		      MeasurementPackage meas_package_R;
		      istringstream iss_R(radar_measurment);
		      long long timestamp_R;

          // reads first element from the current line
          string sensor_type_R;
          iss_R >> sensor_type_R;

          // read measurements at this timestamp
          meas_package_R.sensor_type_ = MeasurementPackage::RADAR;
          meas_package_R.raw_measurements_ = VectorXd(3);
          float ro;
          float theta;
          float ro_dot;
          iss_R >> ro;
          iss_R >> theta;
          iss_R >> ro_dot;
          meas_package_R.raw_measurements_ << ro,theta, ro_dot;
          iss_R >> timestamp_R;
          meas_package_R.timestamp_ = timestamp_R;

          ukf.ProcessMeasurement(meas_package_R);

          target_x = ukf.x_[0];
          target_y = ukf.x_[1];

          //target the current location of the runaway car to start
          cout << "current target is = ( " << target_x << ", " << target_y
               << " )."<< endl;

          bool reachable = false;  //can we reach the cars trajectory in timeframe
          double speed_factor = 0.8; //estimate amount that hunter is slower than target
          //had to add absolute value, because car may be going in negative direction!
          double max_velocity = speed_factor * fabs(ukf.x_[2]);

          //set up structure to put target location
          VectorXd nxt_target = VectorXd(ukf.n_x_);

          if(ukf.step_ <= 5) // before 5 steps, the model may not be that great
          {
            nxt_target = ukf.x_;
          }
          else //now we can try using the UKF model to predict the path of the runaway car
          {
            nxt_target = ukf.x_;

            double interval = 0.05;
            if(ukf.dt_ > 0.05)
            {
              cout << "setting interval to "<< ukf.dt_<<" secs." << endl;
              interval = ukf.dt_;
            }

            //run projections for the runaway car for up to the next 5 seconds
            for(double del_t = interval; del_t < 5.0 and reachable == false;del_t += interval)
            {
              // run the UKF predictor to determine projected location of car at del_t
              // and see if the hunter can reach that location in del_t seconds
              ukf.Predictor(del_t);
              double distance_difference = sqrt(
                (ukf.predictor_x_[1] - hunter_y) * (ukf.predictor_x_[1] - hunter_y)
                + (ukf.predictor_x_[0] - hunter_x) * (ukf.predictor_x_[0] - hunter_x));

              //cout << "del_t = "<<del_t << ", target x,y = ( "<< ukf.predictor_x_[0] <<
              //       ", "  << ukf.predictor_x_[1] <<" ), distance = "
              //       << distance_difference <<", reachable distance = " <<
              //       max_velocity * del_t << "." <<endl;

              if(distance_difference < max_velocity * del_t)
              {
                reachable = true;
                nxt_target = ukf.predictor_x_;
                cout << "found reachable location: "<< nxt_target << ", distance = "
                     << distance_difference << " , reachable distance = "
                     << max_velocity * del_t << "." <<endl;
              }
            }
          }


          if (reachable == false)
               cout << "next target is " << nxt_target << "." <<endl;

          double heading_to_target = wrapAngletoPI(atan2(nxt_target[1] - hunter_y, nxt_target[0] - hunter_x));
		      //turn towards the target
		      double heading_difference = wrapAngletoPI(heading_to_target - hunter_heading);

          double distance_difference = sqrt((nxt_target[1] - hunter_y) * (nxt_target[1] - hunter_y) + (nxt_target[0] - hunter_x) * (nxt_target[0] - hunter_x));

          json msgJson;
          msgJson["turn"] = heading_difference;
          msgJson["dist"] = distance_difference;
          auto msg = "42[\"move_hunter\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
		  else
		  {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req,
	                   char* data, size_t, size_t)
  {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
			res->end(s.data(), s.length());
		else
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws,
	                    uWS::HttpRequest req)
  {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
	                       char* message, size_t length)
  {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port))
		std::cout << "Listening to port " << port << std::endl;
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
/*
VectorXd get_nxt_target_dest(double hunter_x, double hunter_y);

VectorXd get_nxt_target_dest(double hunter_x, double hunter_y)
{
  //find next target that can be reached.
  vectorXd target_dst = ukf.x_;

  bool reachable = false;
  double speed_factor = 0.95; //estimate amount that hunter is slower than target
  double max_velocity = speed_factor * ukf.x_[2]; //we're slower than the target
  for(double del_t = 0.0; del_t < 10.0 and reachable == false;del_t += 0.5)
  {
    VectorXd test_dst = ukf.Predictor(del_t);
    double distance_difference = sqrt((test_dst[1] - hunter_y) * (test_dst[1] - hunter_y) + (test_dst[1] - hunter_x) * (test_dst[1] - hunter_x));
    if(distance_difference < max_velocity * del_t)
    {
      reachable = true;
      target_dst = test_dst;
    }
  }
  return(target_dst);
}
*/
