/*
 * AUTHOR: Edgar Buchanan <edgar.buchanan@york.ac.uk>
 *
 * Controller for the food items.
 * Food items are robots that follow the foraging robots when they receive
 * the signal
 *
 * This controller is meant to be used with the configuration file:
 *    experiments/taskPartitioning.argos
 */

#ifndef FOOTBOT_SYNCHRONIZATION_H
#define FOOTBOT_SYNCHRONIZATION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/*Definition of the generic positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

using namespace argos;

class CFootBotFood : public CCI_Controller {

public:

	CFootBotFood();
	virtual ~CFootBotFood() {}
	virtual void Init(TConfigurationNode& t_node);
	virtual void ControlStep();
	virtual void Reset();
	virtual void Destroy() {}

	UInt32 m_UnFoodId;
	UInt32 m_UnForagerId;
	UInt32 m_UnTransferId;
	UInt32 m_UnHoopCounter;
	UInt32 m_UnMovePositionCounter;
	UInt32 m_UnHoopPositionCounter;
	Real rHoopPosition[20000][2];
	bool bObjectMoved;
	bool m_UnFollow;
	CVector3 cFoodInitialPosition;

private:

	/* Pointer to the LEDs actuator */
	CCI_LEDsActuator* m_pcLEDs;
	/* Pointer to the differential steering actuator */
	CCI_DifferentialSteeringActuator* m_pcWheels;
	/* Pointer to the range and bearing actuator */
	CCI_RangeAndBearingActuator*  m_pcRABA;
	/* Pointer to the range and bearing sensor */
	CCI_RangeAndBearingSensor* m_pcRABS;
	/* Pointer to the omnidirectional camera sensor */
	CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
	/* Pointer to the foot-bot proximity sensor */
	CCI_FootBotProximitySensor* m_pcProximity;
	/* Pointer to the positioning sensors */
	CCI_PositioningSensor*  m_pcPositioning;

};

#endif
