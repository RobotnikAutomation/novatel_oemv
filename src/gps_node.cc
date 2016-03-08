/*
 * gps_node
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik 
 */

#include "novatel_oemv/novatel_oemv1gl1.h"
#include <iostream>
#include <stdio.h>

#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

// Msgs
#include "std_msgs/Bool.h"
#include "novatel_oemv/bestxyza.h"
#include "novatel_oemv/bestposa.h"
#include "novatel_oemv/pdpxyza.h"
#include "novatel_oemv/ionutca.h"
#include "novatel_oemv/pdpposa.h"

#include <sensor_msgs/NavSatFix.h>              // message gps data

using namespace std;

class gps_node
{
public:

  novatel_oemv1gl1 *novatel;			   // gps object  
  novatel_oemv::bestxyza bestxyza_reading;
  novatel_oemv::bestposa bestposa_reading;
  novatel_oemv::pdpxyza pdpxyza_reading; 
  novatel_oemv::ionutca ionutca_reading;
  novatel_oemv::pdpposa pdpposa_reading;
  self_test::TestRunner self_test_;
  diagnostic_updater::Updater diagnostic_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Publisher gps_data_bestxyza_pub_;
  ros::Publisher gps_data_bestposa_pub_;
  ros::Publisher gps_data_pdpxyza_pub_;
  ros::Publisher gps_data_ionutca_pub_;
  ros::Publisher gps_data_pdpposa_pub_;  
  ros::Publisher gps_navsatfix_pub_;
  string port_;
  bool running;

  // error counters and flags
  int error_count_;
  int slow_count_;
  std::string was_slow_;
  std::string error_status_;

  double desired_freq_;
  diagnostic_updater::FrequencyStatus freq_diag_;

  string gps_mode_;
  
  // gps_node member variables
  gps_node(ros::NodeHandle h) : 
	self_test_(), diagnostic_(), 
	node_handle_(h), private_node_handle_("~"),    //calibrate_requested_(false),
	error_count_(0), 
	slow_count_(0), 
	desired_freq_(2), 
	freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.1))
  {
    running = false;
    ros::NodeHandle gps_node_handle(node_handle_, "gps_node");

    private_node_handle_.param("port", port_, string("/dev/ttyUSB0"));
        
    private_node_handle_.param<string>("mode", gps_mode_, "GL1DE");  // NORMAL, EGNOS, PDPFILTER or GL1DE

    
    if ((gps_mode_=="NORMAL") || (gps_mode_=="EGNOS")) {
		gps_data_bestxyza_pub_ = gps_node_handle.advertise<novatel_oemv::bestxyza>("bestxyza", 100);
	    gps_data_bestposa_pub_ = gps_node_handle.advertise<novatel_oemv::bestposa>("bestposa", 100);		
		}
	
    // gps_data_pdpxyza_pub_ = gps_node_handle.advertise<novatel_oemv::pdpxyza>("pdpxyza", 100);
    
    if ( gps_mode_=="GL1DE" ) {
		gps_data_pdpposa_pub_ = gps_node_handle.advertise<novatel_oemv::pdpposa>("pdpposa", 100);
		}
    
    gps_data_ionutca_pub_ = gps_node_handle.advertise<novatel_oemv::ionutca>("ionutca", 100);
    

    // Publishing NavSatFix msgs
    gps_navsatfix_pub_ = gps_node_handle.advertise<sensor_msgs::NavSatFix>("nav_sat_fix", 100);


    self_test_.add("Connect Test", this, &gps_node::ConnectTest);
    diagnostic_.add( freq_diag_ );
    diagnostic_.add( "Device Status", this, &gps_node::deviceStatus );

    // Create gps device
    novatel = new novatel_oemv1gl1( port_.c_str(), NOVATEL_OEMV1GL1_THREAD_DESIRED_HZ, gps_mode_ );
  }

  ~gps_node()
  {
    stop();
  }

  int start()
  {
    stop();

    ROS_INFO("GPS using port = %s", port_.c_str());

    // Open serial port RTK DGPS
    if(novatel->Setup()!=OK){
        ROS_ERROR("gps_node start : Error opening the RTK DGPS");
        return ERROR;
    }else {
        //novatel->FReset();
        ROS_INFO("gps_node start : RTK DGPS - Port opened successfully");
        if(novatel->Start()!=OK) {
            ROS_ERROR("gps_node start : Error running the RTK DGPS device");
            return ERROR;
            }
	}

    ROS_INFO("Connected to GPS device %s", port_.c_str() );
    freq_diag_.clear();

    running = true;

    return(0);
  }

  int stop()
  {
    if(running)
    {
      novatel->Stop();
      running = false;
    }
    return(0);
  }

  int read_and_publish()
  {
      static double prevtime = 0;

      double starttime = ros::Time::now().toSec();
      if (prevtime && prevtime - starttime > 1.0)  // 2Hz by default check 1s
      {
        ROS_WARN("Full gps loop took %f ms. Nominal is 500ms.", 1000 * (prevtime - starttime));
        was_slow_ = "Full gps loop was slow.";
        slow_count_++;
      }

      // Fill the structures 
      if ((gps_mode_=="NORMAL") || (gps_mode_=="EGNOS")) {
		getData(bestxyza_reading);
		getData(bestposa_reading);
		gps_data_bestxyza_pub_.publish(bestxyza_reading);
        gps_data_bestposa_pub_.publish(bestposa_reading);
		}
      if (gps_mode_=="PDPFILTER") {
		getData(bestxyza_reading);
		getData(bestposa_reading);
		getData(pdpxyza_reading);
		gps_data_bestxyza_pub_.publish(bestxyza_reading);
        gps_data_bestposa_pub_.publish(bestposa_reading);		
		gps_data_pdpxyza_pub_.publish(pdpxyza_reading);
	    }
	  if (gps_mode_=="GL1DE") {
		getData(pdpposa_reading);
		gps_data_pdpposa_pub_.publish(pdpposa_reading);
	    }
      getData(ionutca_reading);
      gps_data_ionutca_pub_.publish(ionutca_reading);

      double endtime = ros::Time::now().toSec();
      if (endtime - starttime > 0.05)
      {
        ROS_WARN("Gathering data took %f ms. Nominal is 10ms.", 1000 * (endtime - starttime));
        was_slow_ = "Full gps loop was slow.";
        slow_count_++;
      }
      prevtime = starttime;
      starttime = ros::Time::now().toSec();
                  
            
      // Publish GPS DATA
      sensor_msgs::NavSatFix NavSatFixMsg;
      NavSatFixMsg.header.stamp = ros::Time::now();
	  // In normal or egnos mode get data from bestposa cmd
      if ( (gps_mode_ =="NORMAL") || (gps_mode_=="EGNOS") || (gps_mode_=="PDPFILTER") ) {
		  NavSatFixMsg.status.status = bestposa_reading.p_sol_status;  // -1 NO_FIX 0 FIX 1 SBAS_FIX 2 GBAS_FIX
		  NavSatFixMsg.status.service = 1;                        // 1 GPS 2 GLONASS 4 COMPASS 8 GALILEO
		  NavSatFixMsg.latitude = bestposa_reading.lat;
		  NavSatFixMsg.longitude = bestposa_reading.lon;
		  NavSatFixMsg.altitude = bestposa_reading.hgt;
		  for (int i=0; i<9; i++) NavSatFixMsg.position_covariance[i]= 0.0;      
		  if (bestposa_reading.lat * bestposa_reading.lon != 0.0) {
			  NavSatFixMsg.position_covariance[0] = bestposa_reading.lat_std * bestposa_reading.lat_std;
			  NavSatFixMsg.position_covariance[4] = bestposa_reading.lon_std * bestposa_reading.lon_std;
			  NavSatFixMsg.position_covariance[8] = bestposa_reading.hgt_std * bestposa_reading.hgt_std;
			  NavSatFixMsg.position_covariance_type = 2; //COVARIANCE_TYPE_DIAGONAL_KNOWN;
			  }
		  else {
			  NavSatFixMsg.position_covariance[0] = 99999.0;
			  NavSatFixMsg.position_covariance[4] = 99999.0;
			  NavSatFixMsg.position_covariance[8] = 99999.0;
			  NavSatFixMsg.position_covariance_type = 0; //2 COVARIANCE_TYPE_UNKNOWN;    
			 }
		  }

	  // In pdp modes get the data from pdpposa
      if ( gps_mode_=="GL1DE" ) {
		  NavSatFixMsg.status.status = pdpposa_reading.p_sol_status;  // -1 NO_FIX 0 FIX 1 SBAS_FIX 2 GBAS_FIX
		  NavSatFixMsg.status.service = 1;                        // 1 GPS 2 GLONASS 4 COMPASS 8 GALILEO
		  NavSatFixMsg.latitude = pdpposa_reading.lat;
		  NavSatFixMsg.longitude = pdpposa_reading.lon;
		  NavSatFixMsg.altitude = pdpposa_reading.hgt;
		  for (int i=0; i<9; i++) NavSatFixMsg.position_covariance[i]= 0.0;      
		  if (bestposa_reading.lat * pdpposa_reading.lon != 0.0) {
			  NavSatFixMsg.position_covariance[0] = pdpposa_reading.lat_std * pdpposa_reading.lat_std;
			  NavSatFixMsg.position_covariance[4] = pdpposa_reading.lon_std * pdpposa_reading.lon_std;
			  NavSatFixMsg.position_covariance[8] = pdpposa_reading.hgt_std * pdpposa_reading.hgt_std;
			  NavSatFixMsg.position_covariance_type = 2; //COVARIANCE_TYPE_DIAGONAL_KNOWN;
			  }
		  else {
			  NavSatFixMsg.position_covariance[0] = 99999.0;
			  NavSatFixMsg.position_covariance[4] = 99999.0;
			  NavSatFixMsg.position_covariance[8] = 99999.0;
			  NavSatFixMsg.position_covariance_type = 0; //2 COVARIANCE_TYPE_UNKNOWN;    
			 }		 
		}
		 
      //publish the gps message
      gps_navsatfix_pub_.publish(NavSatFixMsg);

      endtime = ros::Time::now().toSec();
      if (endtime - starttime > 0.05)
      {
        ROS_WARN("Publishing took %f ms. Nominal is 10 ms.", 1000 * (endtime - starttime));
        was_slow_ = "Full gps loop was slow.";
        slow_count_++;
      }

      freq_diag_.tick();
      return(0);
  }

  bool spin()
  {
    ros::Rate r(10.0);
  
    while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {

      if (start() == 0)
      {
        while(node_handle_.ok()) {
          if(read_and_publish() < 0)
            break;
          self_test_.checkTest();
          diagnostic_.update();
          ros::spinOnce();
          r.sleep();
        }
       } else {
        // No need for diagnostic here since a broadcast occurs in start
        // when there is an error.
        usleep(1000000);
        self_test_.checkTest();
        ros::spinOnce();
      }
    }

    ROS_INFO("modbus_io::spin - calling stop !");
    stop();
    return true;
  }

  void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    // connection test
    // TBC
    status.summary(0, "Connected successfully.");
  }

  //  void getData(novatel_oemv::bestxyza& data)
  void getData(novatel_oemv::bestxyza& data)
  {
	// Read Bestxyza 
	bestxyza_t bestxyza = novatel->GetBestxyza();	
	// only a subset is published as message
	data.seconds = bestxyza.amh.seconds;
	data.sequence = bestxyza.amh.sequence;
	data.p_sol_status = bestxyza.p_sol_status;
	data.pos_type = bestxyza.pos_type;
	data.p[0] = bestxyza.p.x; data.p[1] = bestxyza.p.y; data.p[2] = bestxyza.p.z;
	data.p_std[0] = bestxyza.p_std.x; data.p_std[1] = bestxyza.p_std.y; data.p_std[2] = bestxyza.p_std.z;
	data.v_sol_status = bestxyza.v_sol_status;
	data.vel_type = bestxyza.vel_type;			
	data.v[0] = bestxyza.v.x; data.v[1] = bestxyza.v.y; data.v[2] = bestxyza.v.z;
	data.v_std[0] = bestxyza.v_std.x; data.v_std[1] = bestxyza.v_std.y; data.v_std[2] = bestxyza.v_std.z;  
	data.stnID = bestxyza.stnID;	// string <- char[4]
	data.v_latency = bestxyza.v_latency;
	data.diff_age = bestxyza.diff_age;
	data.sol_age = bestxyza.sol_age;
	data.SVs = bestxyza.SVs;
	data.solnSVs = bestxyza.solnSVs;
  }


  //  void getData(novatel_oemv::bestposa& data)
  void getData(novatel_oemv::bestposa& data)
  {
	// Read Bestposa 
	bestposa_t bestposa = novatel->GetBestposa();
	data.seconds = bestposa.amh.seconds;
	data.sequence = bestposa.amh.sequence;
	data.p_sol_status = bestposa.p_sol_status;
	data.pos_type = bestposa.pos_type;
	data.lat = bestposa.lat;
	data.lon = bestposa.lon;
	data.hgt = bestposa.hgt;
	data.lat_std = bestposa.lat_std;
	data.lon_std = bestposa.lon_std;
	data.hgt_std = bestposa.hgt_std;
	data.stnID = bestposa.stnID;	// string <- char[4]
	data.diff_age = bestposa.diff_age;
	data.sol_age = bestposa.sol_age;
	data.SVs = bestposa.SVs;
	data.solnSVs = bestposa.solnSVs;
  }

  //  void getData(novatel_oemv::pdpxyza& data)
  void getData(novatel_oemv::pdpxyza& data)
  {
	// Read pdpxyza 
	pdpxyza_t pdpxyza = novatel->GetPdpxyza();	
	// only a subset is published as message
	data.seconds = pdpxyza.amh.seconds;
	data.sequence = pdpxyza.amh.sequence;
	data.p_sol_status = pdpxyza.p_sol_status;
	data.pos_type = pdpxyza.pos_type;
	data.p[0] = pdpxyza.p.x; data.p[1] = pdpxyza.p.y; data.p[2] = pdpxyza.p.z;
	data.p_std[0] = pdpxyza.p_std.x; data.p_std[1] = pdpxyza.p_std.y; data.p_std[2] = pdpxyza.p_std.z;
	data.v_sol_status = pdpxyza.v_sol_status;
	data.vel_type = pdpxyza.vel_type;			
	data.v[0] = pdpxyza.v.x; data.v[1] = pdpxyza.v.y; data.v[2] = pdpxyza.v.z;
	data.v_std[0] = pdpxyza.v_std.x; data.v_std[1] = pdpxyza.v_std.y; data.v_std[2] = pdpxyza.v_std.z;  
	data.stnID = pdpxyza.stnID;	
	data.v_latency = pdpxyza.v_latency;
	data.diff_age = pdpxyza.diff_age;
	data.sol_age = pdpxyza.sol_age;
	data.SVs = pdpxyza.SVs;
	data.solnSVs = pdpxyza.solnSVs;
  }

  //  void getData(novatel_oemv::ionutca& data)
  void getData(novatel_oemv::ionutca& data)
  {
	ionutca_t ionutca = novatel->GetIonutca();
	data.a0 = ionutca.a0; data.a1 = ionutca.a1; data.a2 = ionutca.a2; data.a3 = ionutca.a3;
	data.b0 = ionutca.b0; data.b1 = ionutca.b1; data.b2 = ionutca.b2; data.b3 = ionutca.b3;
	data.utc_wn = ionutca.utc_wn;
	data.tot = ionutca.tot;
	data.A0 = ionutca.A0; data.A1 = ionutca.A1;
	data.wn_lsf = ionutca.wn_lsf;
	data.dn = ionutca.dn;
	data.deltat_ls = ionutca.deltat_ls;
    	data.deltat_lsf = ionutca.deltat_lsf;
    	data.deltat_utc = ionutca.deltat_utc;
  }
	
  //  void getData(novatel_oemv::pdpposa& data)
  void getData(novatel_oemv::pdpposa& data)
  {
	// Read Bestposa 
	pdpposa_t pdpposa = novatel->GetPdpposa();	
	data.seconds = pdpposa.amh.seconds;
	data.sequence = pdpposa.amh.sequence;
	data.p_sol_status = pdpposa.p_sol_status;
	data.pos_type = pdpposa.pos_type;
	data.lat = pdpposa.lat;
	data.lon = pdpposa.lon;
	data.hgt = pdpposa.hgt;
	data.lat_std = pdpposa.lat_std;
	data.lon_std = pdpposa.lon_std;
	data.hgt_std = pdpposa.hgt_std;
	data.stnID = pdpposa.stnID;	// string <- char[4]
	data.diff_age = pdpposa.diff_age;
	data.sol_age = pdpposa.sol_age;
	data.SVs = pdpposa.SVs;
	data.solnSVs = pdpposa.solnSVs;
  }


  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    if (!running)
      status.summary(2, "gps_node is stopped");
    else if (!was_slow_.empty())
    {
      status.summary(1, "gps_node - Excessive delay");
      was_slow_.clear();
    }
    else
      status.summary(0, "gps_node is running");

    status.add("Error count", error_count_);
    status.add("Excessive delay", slow_count_);
  }

};

int 
main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_node");

  ros::NodeHandle nh;

  gps_node gn(nh);

  gn.spin();

  return(0);
}




