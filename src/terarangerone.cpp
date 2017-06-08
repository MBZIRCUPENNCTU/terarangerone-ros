/****************************************************************************
 *
 * Copyright (C) 2014 Flavio Fontana & Luis Rodrigues. All rights reserved.
 * Author: Flavio Fontana <fly.fontana@gmail.com>
 * Author: Luis Rodrigues <luis.rodrigues@terabee.com>

 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name TerarangerOne nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <string>

#include "terarangerone/terarangerone.h"

#define MAXIMAL_TIME_INTERVAL 1

namespace terarangerone
{

TerarangerOne::TerarangerOne()
{
  // Get paramters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_, std::string("/dev/ttyUSB0"));

  // Publishers
  range_publisher_ = nh_.advertise<sensor_msgs::Range>("terarangerone", 1);

  // Output loaded parameters to console for double checking
  ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());

  // Dynamic reconfigure
  dyn_param_server_callback_function_ = boost::bind(&TerarangerOne::dynParamCallback, this, _1, _2);
  // dyn_param_server_.setCallback(dyn_param_server_callback_function_);

	connectToSensor();
}

TerarangerOne::~TerarangerOne()
{
}

uint8_t TerarangerOne::connectToSensor(void) {

  // Create serial port
  serial_port_ = new SerialPort();

  // Set callback function for the serial ports
  serial_data_callback_function_ = boost::bind(&TerarangerOne::serialDataCallback, this, _1);
  serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);

  // Connect serial port
  if (!serial_port_->connect(portname_))
  {
		ROS_ERROR("Could not connect to Teraranger.");
    return 0;
  }

  // Set operation Mode
  setMode(BINARY_MODE);

  // Set outdoor mode
  setMode(OUTDOOR_MODE);

	ROS_INFO("Connected to Teraranger.");	

  lastReceived = ros::Time::now();

	return 1;
}

void TerarangerOne::releaseSerialLine(void) {

	delete serial_port_;
}

uint8_t TerarangerOne::crc8(uint8_t *p, uint8_t len)
{
  uint16_t i;
  uint16_t crc = 0x0;

  while (len--)
  {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

void TerarangerOne::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;

  sensor_msgs::Range range_msg;
  range_msg.field_of_view = 0.0593;
  range_msg.max_range = 14.0;
  range_msg.min_range = 0.2;
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;

  if (single_character != 'T' && buffer_ctr < 4)
  {
    // not begin of serial feed so add char to buffer
    input_buffer[buffer_ctr++] = single_character;
    return;
  }
  else if (single_character == 'T')
  {
    if (buffer_ctr == 4)
    {
      // end of feed, calculate
      int16_t crc = crc8(input_buffer, 3);

      if (crc == input_buffer[3])
      {
        int16_t range = input_buffer[1] << 8;
        range |= input_buffer[2];

        if (range < 14000 && range >= 0)
        {
          range_msg.header.stamp = ros::Time::now();
          range_msg.header.seq = seq_ctr++;
          range_msg.range = range * 0.001; // convert to m
          range_publisher_.publish(range_msg);
        	lastReceived = ros::Time::now();
				}
        ROS_DEBUG("[%s] all good %.3f m", ros::this_node::getName().c_str(), range_msg.range);
      }
      else
      {
        ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
      }
    }
    else
    {
      ROS_DEBUG("[%s] reveived T but did not expect it, reset buffer without evaluating data",
               ros::this_node::getName().c_str());
    }
  }
  else
  {
    ROS_DEBUG("[%s] buffer_overflowed without receiving T, reset input_buffer", ros::this_node::getName().c_str());
  }
  // reset
  buffer_ctr = 0;

  // clear struct
  bzero(&input_buffer, BUFFER_SIZE);

  // store T
  input_buffer[buffer_ctr++] = 'T';
}

void TerarangerOne::setMode(char c)
{
  serial_port_->sendChar(c);
}

void TerarangerOne::dynParamCallback(const terarangerone::TerarangerOneConfig &config, uint32_t level)
{
  if (config.Mode == terarangerone::TerarangerOne_Fast)
  {
		ROS_INFO("Teraranger set to fast mode.");
		setMode(FAST_MODE);
  }

  if (config.Mode == terarangerone::TerarangerOne_Precise)
  {
		ROS_INFO("Teraranger set to precise mode.");
		setMode(PRECISE_MODE);
  }
 
  if (config.Mode == terarangerone::TerarangerOne_Outdoor)
  {
		ROS_INFO("Teraranger set to outdoor mode.");
		setMode(OUTDOOR_MODE);
  }
}

} // namespace terarangerone

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terarangerone");
  terarangerone::TerarangerOne tera_bee;
  ros::Rate loop_rate(1);

	while (ros::ok()) {

		// check whether the teraranger stopped sending data
    ros::Duration interval = ros::Time::now() - tera_bee.lastReceived;
		if (interval.toSec() > MAXIMAL_TIME_INTERVAL) {
						
			tera_bee.releaseSerialLine();
			
			ROS_WARN("Teraranger not responding, resetting connection...");
			
			// if establishing the new connection was successfull	
			if (tera_bee.connectToSensor() == 1) {
			
				ROS_WARN("New connection to Teraranger was established.");
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

  return 0;
}
