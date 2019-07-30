#include "stdafx.h"
#include "RealsenseSensor.h"
#include <utility>

#ifdef REAL_SENSE

RealsenseSensor::RealsenseSensor() : alignToDepth(RS2_STREAM_DEPTH)
{
	auto devices = rsCtx.query_devices();
	if (devices.size() == 0) {
		throw std::runtime_error("No realsense device detected!");
	}
	rsDevice = devices.front();
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_stream(RS2_STREAM_COLOR);
	rsPipe.start(cfg);
	auto profile = rsPipe.get_active_profile();

	auto depthStream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto colorStream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

	//auto depthExtr = depthStream.get_extrinsics_to(colorStream);
	auto depthIntr = depthStream.get_intrinsics();
	//auto colorExtr = colorStream.get_extrinsics_to(depthStream);
	auto colorIntr = colorStream.get_intrinsics();
	resDepth = std::make_pair(depthStream.width(), depthStream.height());
	//resColor = std::make_pair(colorStream.width(), colorStream.height());
	// aligned color has same res as depth!
	resColor = resDepth;

	RGBDSensor::init((uint32_t)resDepth.first, (uint32_t)resDepth.second, (uint32_t)resColor.first, (uint32_t)resColor.second);

	initializeDepthExtrinsics(mat4f::identity());
	initializeColorExtrinsics(mat4f::identity());

	initializeDepthIntrinsics(depthIntr.fx, depthIntr.fy, depthIntr.ppx, depthIntr.ppy);
	initializeColorIntrinsics(colorIntr.fx, colorIntr.fy, colorIntr.ppx, colorIntr.ppy);
}


RealsenseSensor::~RealsenseSensor()
{
	rsPipe.stop();
}

void RealsenseSensor::createFirstConnected()
{
	// todo? anything here?
}

bool RealsenseSensor::processDepth()
{
	//rs2::frameset frames;
	//if (rsPipe.poll_for_frames(&frames)) {
	auto frames = rsPipe.wait_for_frames();
	auto depth = frames.get_depth_frame();
		float* depthBuff = getDepthFloat();
		// get depth data as 16-bit values
		const uint16_t* dataPtr = (uint16_t*)depth.get_data();

		for (auto y = 0u; y < (uint32_t)resDepth.second; ++y) {
			for (auto x = 0u; x < (uint32_t)resDepth.first; ++x) {
				auto idx = y * (uint32_t)resDepth.first + x;
				depthBuff[idx] = (float)dataPtr[idx] * 0.001f; // default depth scale in realsense is 1 mm
			}
		}
		return true;
	//}
	return false;
}

bool RealsenseSensor::processColor()
{
	//rs2::frameset frames;
	//if (rsPipe.poll_for_frames(&frames)) {
	auto frames = rsPipe.wait_for_frames();
		frames = alignToDepth.process(frames);
		auto aligned = frames.get_color_frame();
		uchar* colorBuff = (uchar*)getColorRGBX();

		const auto* dataPtr = (uint8_t*)aligned.get_data();
		for (auto idx = 0u; idx < (uint32_t)(resColor.first * resColor.second); ++idx) {
				colorBuff[0] = dataPtr[0];
				colorBuff[1] = dataPtr[1];
				colorBuff[2] = dataPtr[2];
				colorBuff[3] = 0u;
				colorBuff += 4u;
				dataPtr += 3u;
		}
		return true;
	//}
	return false;
}

std::string RealsenseSensor::getSensorName() const
{
	return "RealSense";
}


#endif