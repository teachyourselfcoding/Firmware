/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */
#define DEBUG
#define SIZE 100
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <math.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_attitude.h>
#include <px4_posix.h>

extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[]);
uint64_t timestamp_before_publishing;
uint64_t timestamp_after_publishing;
uint64_t timestamp_feedback;
uint64_t timestamp1;10
uint64_t timestamp2;
int array[SIZE]={};

uint64_t _orb_stat;

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");
	/* trigger subscription updated */

	int _trigger_sub =  orb_subscribe(ORB_ID(camera_trigger));

	// "px4_pollfd_struct_t fds[1] = {};
	// fds[0].fd = _trigger_sub;
	// fds[0].events = POLLIN;"
	/*try to trigger camera*/

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _trigger_sub;
	fds[0].events = POLLIN;


	struct vehicle_command_s cmd{};
	orb_advert_t veh_trig = orb_advertise(ORB_ID(vehicle_command), &cmd);
	 //parameters to be changed for triggering camera
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL;
	cmd.param5 = 1;                                                          

	// orb_stat(_trigger_sub, &_orb_stat);
	// PX4_INFO("orb_stat: %llu", _orb_stat);
	//irqstate_t s = enter_critical_section();
	
	//leave_critical_section(s);

	#ifdef DEBUG

	int k = 0;
	int i = 0;
	struct camera_trigger_s trig;
	while(i<SIZE){ //running camera_trigger SIZE times within the simple_app

	#endif

		timestamp_before_publishing= hrt_absolute_time();
		cmd.timestamp= timestamp_before_publishing;
		orb_publish(ORB_ID(vehicle_command), veh_trig, &cmd);
		timestamp_after_publishing= hrt_absolute_time();
		
		while(k<100) { //if camera_trigger does not send a reply in 1second, exit loop
			
			/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
			int poll_ret = px4_poll(fds, 1, 10);

			/* handle the poll result */
			if (poll_ret == 0) {
				/* this means none of our providers is giving us data */
			}
			else {
				if (fds[0].revents & POLLIN) {
					/* obtained data for the first file descriptor */
					/* copy sensors raw data into local buffer */
					bool updated = true;    //updated set to true to ensure we orb_check
					while(updated){
						orb_check(_trigger_sub, &updated);     //check for an update in topic
						orb_copy(ORB_ID(camera_trigger), _trigger_sub, &trig);
						if(trig.timestamp<timestamp_before_publishing){  //if we obtain a previous timestamp (new one didnt come in),
							continue; //continue waiting for interrupt
						}	
					}
					break;//we received the timestamp! Exit while(k<100) loop


				}else{
					continue
				}
			}
				k++; 
			}
	#ifdef DEBUG
	
			//timestamp received, prepare to trigger camera once more
			array[i]=(trig.timestamp-timestamp_before_publishing); //enter the delay into an array
			PX4_INFO("%d =  %d ",i,array[i]);
			PX4_INFO("trig.timestamp =  %llu ",trig.timestamp);
			PX4_INFO("timestamp before publishing =  %llu ",timestamp_before_publishing);
			i++;
		// timestamp_feedback = trig.timestamp;
	
	  }
	#endif
	orb_unadvertise(veh_trig);

	#ifdef DEBUG

	int64_t sum = 0;   //Adding up sum of all the members in the array, then finding the average
	for(int j=0;j<SIZE;j++){
		 sum =sum + array[j];
	}
	
	double avg = static_cast<double>(sum)/SIZE;
	PX4_INFO("sum = %lld ",sum);
	PX4_INFO(" avg delay =: %.2lf ",avg);

	#endif




	// PX4_INFO("delay =: %llu ",timestamp_feedback-timestamp_before_publishing);
	// #ifdef DEBUG

	// PX4_INFO("Publishing delay: %llu ",timestamp_after_publishing-timestamp_before_publishing);
	// PX4_INFO("delay after publishing =: %llu ",timestamp_feedback-timestamp_after_publishing);
	// PX4_INFO("feedback timestamp =: %llu ",timestamp_feedback);
	// PX4_INFO("exiting");

	// #endif
	orb_unsubscribe(_trigger_sub);
	return 0;
}