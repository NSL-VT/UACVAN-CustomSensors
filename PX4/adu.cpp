/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @author Binay Rijal <binaypr@vt.edu>
 * modified from "airspeed.cpp"
*/

#include <drivers/drv_hrt.h>
#include "adu.hpp"
#include <math.h>
#include <lib/geo/geo.h> // For CONSTANTS_*

const char *const UavcanADUBridge::NAME = "air_data_unit";

UavcanADUBridge::UavcanADUBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_airspeed", ORB_ID(sensor_flow_angle)),
	_sub_aoa_data(node),
	_sub_ss_data(node)
	{}

int UavcanADUBridge::init()
{
	int res = _sub_aoa_data.start(AOACbBinder(this, &UavcanADUBridge::aoa_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	int res2 = _sub_ss_data.start(SSCbBinder(this, &UavcanADUBridge::ss_sub_cb));

	if (res2 < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res2);
		return res2;
	}
	return 0;
}


void
UavcanADUBridge::ss_sub_cb(const
				 uavcan::ReceivedDataStructure<uavcan::equipment::air_data::Sideslip> &msg)
{
	_last_ss_val = msg.sideslip_angle;
	_last_ss_variance_val = msg.sideslip_angle_variance;

}

void
UavcanADUBridge::aoa_sub_cb(const
				 uavcan::ReceivedDataStructure<uavcan::equipment::air_data::AngleOfAttack> &msg)
{
	sensor_flow_angle_s report{};



	/*
	 * FIXME HACK
	 * This code used to rely on msg.getMonotonicTimestamp().toUSec() instead of HRT.
	 * It stopped working when the time sync feature has been introduced, because it caused libuavcan
	 * to use an independent time source (based on hardware TIM5) instead of HRT.
	 * The proper solution is to be developed.
	 */
	report.timestamp   		= hrt_absolute_time();
	report.device_id		= msg.sensor_id;
	report.beta_vane		= _last_ss_val;
	report.alpha_angle		= msg.aoa;

	publish(msg.getSrcNodeID().get(), &report);
}
