#pragma once
#include "GlobalAppState.h"

#ifdef REAL_SENSE
#include "RGBDSensor.h"
#include "librealsense2/rs.hpp"

class RealsenseSensor :
	public RGBDSensor
{
public:
	RealsenseSensor();
	~RealsenseSensor();

	void createFirstConnected();

	// Inherited via RGBDSensor 
	bool processDepth();

	// Inherited via RGBDSensor
	bool processColor();

	// Inherited via RGBDSensor
	std::string getSensorName() const override;

private:
	rs2::context rsCtx;
	rs2::device rsDevice;
	rs2::pipeline rsPipe;

	rs2::align alignToDepth;

	std::pair<int, int> resDepth;
	std::pair<int, int> resColor;


};

#endif