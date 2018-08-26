/**
 * @file custom.c
 * Minimal application example for PX4 autopilot
 *
 * @author mrinmoy sarkar <mrinmoy.pol@gmail.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>


#include <px4_getopt.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_module.h>


#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#ifdef __PX4_NUTTX
#include <nuttx/fs/ioctl.h>
#endif

#include <arch/board/board.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "systemlib/param/param.h"
#include "drivers/drv_pwm_output.h"

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int custom_main(int argc, char *argv[]);

int custom_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");
/*
	// subscribe to sensor_combined topic 
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	// limit the update rate to 5 Hz 
	orb_set_interval(sensor_sub_fd, 200);

	// advertise attitude topic 
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	// one could wait for multiple topics with this technique, just using one here 
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		// there could be more file descriptors here, in the form like:
		// { .fd = other_sub_fd,   .events = POLLIN },
		 
	};

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		// wait for sensor update of 1 file descriptor for 1000 ms (1 second) 
		int poll_ret = px4_poll(fds, 1, 1000);

		// handle the poll result 
		if (poll_ret == 0) {
			// this means none of our providers is giving us data 
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			// this is seriously bad - should be an emergency 
			if (error_counter < 10 || error_counter % 50 == 0) {
				// use a counter to prevent flooding (and slowing us down) 
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				// obtained data for the first file descriptor 
				struct sensor_combined_s raw;
				// copy sensors raw data into local buffer 
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.accelerometer_m_s2[0],
					 (double)raw.accelerometer_m_s2[1],
					 (double)raw.accelerometer_m_s2[2]);

				// set att and publish this information for other apps
				 //the following does not have any meaning, it's just an example
				
				att.q[0] = raw.accelerometer_m_s2[0];
				att.q[1] = raw.accelerometer_m_s2[1];
				att.q[2] = raw.accelerometer_m_s2[2];

				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			}

			// there could be more file descriptors here, in the form like:
			 // if (fds[1..n].revents & POLLIN) {}
			 
		}
	}

	PX4_INFO("exiting");
*/
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int ret;
	// open for ioctl only 
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_ERR("can't open %s", dev);
		return 1;
	}

	// get the number of servo channels 
	unsigned servo_count;
	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);

	if (ret != OK) {
		PX4_ERR("PWM_SERVO_GET_COUNT");
		return 0;
	}
	PX4_INFO("%d\n", servo_count);

	ret = px4_ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);

	if (ret != OK)
	{
		err(1, "PWM_SERVO_SET_ARM_OK");
	}

	// tell IO that the system is armed (it will output values if safety is off) 
	ret = px4_ioctl(fd, PWM_SERVO_ARM, 0);

	if (ret != OK) 
	{
		err(1, "PWM_SERVO_ARM");
	}

		
	PX4_INFO("Outputs armed");

/*

		// disarm, but do not revoke the SET_ARM_OK flag 
		ret = px4_ioctl(fd, PWM_SERVO_DISARM, 0);

		if (ret != OK) {
			err(1, "PWM_SERVO_DISARM");
		}

		if (print_verbose) {
			PX4_INFO("Outputs disarmed");
		}

*/

/*
else if (oneshot || !strcmp(command, "rate")) {

		// Change alternate PWM rate or set oneshot
		 // Either the "oneshot" command was used
		 // and/OR -r was provided on command line and has changed the alt_rate
		 // to the non default of -1, so we will issue the PWM_SERVO_SET_UPDATE_RATE
		 // ioctl
		 //

		if (oneshot || alt_rate >= 0) {
			ret = px4_ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, oneshot ? 0 : alt_rate);

			if (ret != OK) {
				PX4_ERR("PWM_SERVO_SET_UPDATE_RATE (check rate for sanity)");
				return error_on_warn;
			}
		}

		// directly supplied channel mask 
		if (set_mask > 0) {
			ret = px4_ioctl(fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, set_mask);

			if (ret != OK) {
				PX4_ERR("PWM_SERVO_SET_SELECT_UPDATE_RATE");
				return error_on_warn;
			}
		}

		// assign alternate rate to channel groups 
		if (alt_channels_set) {
			uint32_t mask = 0;

			for (group = 0; group < 32; group++) {
				if ((1 << group) & alt_channel_groups) {
					uint32_t group_mask;

					ret = px4_ioctl(fd, PWM_SERVO_GET_RATEGROUP(group), (unsigned long)&group_mask);

					if (ret != OK) {
						PX4_ERR("PWM_SERVO_GET_RATEGROUP(%u)", group);
						return error_on_warn;
					}

					mask |= group_mask;
				}
			}

			ret = px4_ioctl(fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, mask);

			if (ret != OK) {
				PX4_ERR("PWM_SERVO_SET_SELECT_UPDATE_RATE");
				return error_on_warn;
			}
		}

		return 0;

	}
*/


/*
else if (!strcmp(command, "disarmed")) {

		if (set_mask == 0) {
			usage("no channels set");
			return 1;
		}

		if (pwm_value < 0) {
			return 0;
		}

		if (pwm_value == 0) {
			PX4_WARN("reading disarmed value of zero, disabling disarmed PWM");
		}

		struct pwm_output_values pwm_values;

		memset(&pwm_values, 0, sizeof(pwm_values));

		pwm_values.channel_count = servo_count;

		// first get current state before modifying it 
		ret = px4_ioctl(fd, PWM_SERVO_GET_DISARMED_PWM, (long unsigned int)&pwm_values);

		if (ret != OK) {
			PX4_ERR("failed get disarmed values");
			return ret;
		}

		for (unsigned i = 0; i < servo_count; i++) {
			if (set_mask & 1 << i) {
				pwm_values.values[i] = pwm_value;

				if (print_verbose) {
					PX4_INFO("chan %d: disarmed PWM: %d", i + 1, pwm_value);
				}
			}
		}

		if (pwm_values.channel_count == 0) {
			usage("disarmed: no PWM channels");
			return 1;

		} else {

			ret = px4_ioctl(fd, PWM_SERVO_SET_DISARMED_PWM, (long unsigned int)&pwm_values);

			if (ret != OK) {
				PX4_ERR("failed setting disarmed values (%d)", ret);
				return error_on_warn;
			}
		}

		return 0;

	}
	*/



/*
else if (!strcmp(command, "test")) {

		if (set_mask == 0) {
			usage("no channels set");
			return 1;
		}

		if (pwm_value == 0) {
			usage("no PWM provided");
			return 1;
		}

		// get current servo values 
		struct pwm_output_values last_spos;

		for (unsigned i = 0; i < servo_count; i++) {


			ret = px4_ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&last_spos.values[i]);

			if (ret != OK) {
				PX4_ERR("PWM_SERVO_GET(%d)", i);
				return 1;
			}
		}

		// perform PWM output 

		// Open console directly to grab CTRL-C signal 
		struct pollfd fds;
		fds.fd = 0; // stdin 
		fds.events = POLLIN;

		PX4_INFO("Press CTRL-C or 'c' to abort.");

		while (1) {
			for (unsigned i = 0; i < servo_count; i++) {
				if (set_mask & 1 << i) {
					ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);

					if (ret != OK) {
						PX4_ERR("PWM_SERVO_SET(%d)", i);
						return 1;
					}
				}
			}

			// abort on user request 
			char c;
			ret = poll(&fds, 1, 0);

			if (ret > 0) {

				ret = read(0, &c, 1);

				if (c == 0x03 || c == 0x63 || c == 'q') {
					// reset output to the last value 
					for (unsigned i = 0; i < servo_count; i++) {
						if (set_mask & 1 << i) {
							ret = px4_ioctl(fd, PWM_SERVO_SET(i), last_spos.values[i]);

							if (ret != OK) {
								PX4_ERR("PWM_SERVO_SET(%d)", i);
								return 1;
							}
						}
					}

					PX4_INFO("User abort\n");
					return 0;
				}
			}

			// Delay longer than the max Oneshot duration 

			usleep(2542);

#ifdef __PX4_NUTTX
			//Trigger all timer's channels in Oneshot mode to fire
			 // the oneshots with updated values.
			 

			up_pwm_update();
#endif
		}

		return 0;


	}
	*/
 

		printf("device: %s\n", dev);

		uint32_t info_default_rate;
		uint32_t info_alt_rate;
		uint32_t info_alt_rate_mask;

		ret = px4_ioctl(fd, PWM_SERVO_GET_DEFAULT_UPDATE_RATE, (unsigned long)&info_default_rate);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_DEFAULT_UPDATE_RATE");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_UPDATE_RATE, (unsigned long)&info_alt_rate);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_UPDATE_RATE");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_SELECT_UPDATE_RATE, (unsigned long)&info_alt_rate_mask);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_SELECT_UPDATE_RATE");
			return 1;
		}

		struct pwm_output_values failsafe_pwm;

		struct pwm_output_values disarmed_pwm;

		struct pwm_output_values min_pwm;

		struct pwm_output_values max_pwm;

		struct pwm_output_values trim_pwm;

		ret = px4_ioctl(fd, PWM_SERVO_GET_FAILSAFE_PWM, (unsigned long)&failsafe_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_FAILSAFE_PWM");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_DISARMED_PWM, (unsigned long)&disarmed_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_DISARMED_PWM");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_MIN_PWM, (unsigned long)&min_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_MIN_PWM");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_MAX_PWM, (unsigned long)&max_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_MAX_PWM");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_TRIM_PWM, (unsigned long)&trim_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_TRIM_PWM");
			return 1;
		}

		// print current servo values 
		for (unsigned i = 0; i < servo_count; i++) {
			servo_position_t spos;

			ret = px4_ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&spos);

			if (ret == OK) {
				printf("channel %u: %u us", i + 1, spos);

				if (info_alt_rate_mask & (1 << i)) {
					printf(" (alternative rate: %d Hz", info_alt_rate);

				} else {
					printf(" (default rate: %d Hz", info_default_rate);
				}


				printf(" failsafe: %d, disarmed: %d us, min: %d us, max: %d us, trim: %5.2f)",
				       failsafe_pwm.values[i], disarmed_pwm.values[i], min_pwm.values[i], max_pwm.values[i],
				       (double)((int16_t)(trim_pwm.values[i]) / 10000.0f));
				printf("\n");

			} else {
				printf("%u: ERROR\n", i);
			}
		}

		// print rate groups 
		for (unsigned i = 0; i < servo_count; i++) {
			uint32_t group_mask;

			ret = px4_ioctl(fd, PWM_SERVO_GET_RATEGROUP(i), (unsigned long)&group_mask);

			if (ret != OK) {
				break;
			}

			if (group_mask != 0) {
				printf("channel group %u: channels", i);

				for (unsigned j = 0; j < 32; j++) {
					if (group_mask & (1 << j)) {
						printf(" %u", j + 1);
					}
				}

				printf("\n");
			}
		}



	return 0;

}
