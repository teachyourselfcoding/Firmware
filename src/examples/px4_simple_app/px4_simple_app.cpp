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

extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[]);
uint64_t timestamp_before_publishing;
uint64_t timestamp_after_publishing;
uint64_t timestamp_feedback;
uint64_t timestamp1;
uint64_t timestamp2;


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
	px4_pollfd_struct_t fds[] = {
   	 { .fd = _trigger_sub,   .events = POLLIN },
	};
	vehicle_command_s cmd{};

	// memset(&cam, 0, sizeof(cam));
	orb_advert_t veh_trig = orb_advertise(ORB_ID(vehicle_command), &cmd);
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL;
	cmd.param5 = 1;



	// while(true){
	// 	 if (fds[0].revents & POLLIN) {
	// 		 struct camera_trigger_s trig;
	// 		 orb_copy(ORB_ID(camera_trigger), _trigger_sub, &trig);
	// 		 PX4_INFO("timestamp2: %lld", trig.timestamp);

	// 	 }
	// }

	while(true) {

		orb_stat(_trigger_sub, &_orb_stat);
		PX4_INFO("orb_stat: %llu", _orb_stat);
		timestamp_before_publishing= hrt_absolute_time();


		orb_publish(ORB_ID(vehicle_command), veh_trig, &cmd);

		timestamp_after_publishing= hrt_absolute_time();
		PX4_INFO("timestamp_before_publishing: %llu ",timestamp_before_publishing);
		PX4_INFO("timestamp_after_publishing: %llu ",timestamp_after_publishing);

		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */

		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			// PX4_ERR("Got no data within a second");

		}
		else {
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct camera_trigger_s trig;
				/* copy sensors raw data into local buffer */
				bool updated = false;
				orb_check(_trigger_sub, &updated);
				if(updated){
					orb_copy(ORB_ID(camera_trigger), _trigger_sub, &trig);
				}
				timestamp_feedback = trig.timestamp;

				/*To resolve occasional error in timestamp collection*/
				if(timestamp_feedback>timestamp_before_publishing){
					break;
				}


			}
		}
	}
	orb_unadvertise(veh_trig);

	timestamp1= hrt_absolute_time();
	timestamp2= hrt_absolute_time();
	PX4_INFO("delay =: %llu ",timestamp_feedback-timestamp_before_publishing);
	PX4_INFO("delay after publishing =: %llu ",timestamp_feedback-timestamp_after_publishing);
	PX4_INFO("hrt_absolute_time1 =: %llu ",timestamp1);
	PX4_INFO("hrt_absolute_time2 =: %llu ",timestamp2);
	PX4_INFO("hrt_absolute_time_delay =: %llu ",timestamp2-timestamp1);
	PX4_INFO("exiting");

	orb_unsubscribe(_trigger_sub);
	return 0;
}
