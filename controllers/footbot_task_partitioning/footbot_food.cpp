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

/* Include the controller definition */
#include "footbot_food.h"

/****************************************/
/****************************************/

CFootBotFood::CFootBotFood() :
	m_pcLEDs(NULL),
	m_pcWheels(NULL),
	m_pcRABA(NULL),
	m_pcRABS(NULL),
	m_pcCamera(NULL),	
	m_pcProximity(NULL),
	m_pcPositioning(NULL){}

/****************************************/
/****************************************/

void CFootBotFood::Init(TConfigurationNode& t_node) {

	/* Get sensor/actuator handles */
	m_pcLEDs	= GetActuator<CCI_LEDsActuator	>("leds");
	m_pcWheels	= GetActuator<CCI_DifferentialSteeringActuator	>("differential_steering");
	m_pcRABA	= GetActuator<CCI_RangeAndBearingActuator	>("range_and_bearing"    );
	m_pcRABS	= GetSensor  <CCI_RangeAndBearingSensor	>("range_and_bearing"    );
	m_pcCamera	= GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor	>("colored_blob_omnidirectional_camera");
	m_pcProximity	= GetSensor <CCI_FootBotProximitySensor	>("footbot_proximity"    );
	m_pcPositioning	= GetSensor	<CCI_PositioningSensor	>("positioning"			 );

	/* Switch the camera on */
	m_pcCamera->Enable();
	const CCI_PositioningSensor::SReading& tPosition = m_pcPositioning->GetReading();
	cFoodInitialPosition = tPosition.Position;
	Reset();

	/* ID first digit takes the fifth position after the word  'food' (food00)*/
	if(GetId().length() <= 5) m_UnFoodId = GetId().at(4) - '0' + 1;
	else  m_UnFoodId = (GetId().at(4) - '0') * 10 + GetId().at(5) - '0' + 1;
}

/****************************************/
/****************************************/

void CFootBotFood::ControlStep() {

	/* Get readings from sensors */
	const CCI_PositioningSensor::SReading& tPosition = m_pcPositioning->GetReading();
	const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs = m_pcCamera->GetReadings();

	for(size_t i = 0; i < tPackets.size(); ++i) {

		/*Communicate with forager*/
		if(tPackets[i].Range <= 23 && tPackets[i].Data[1] == 1 && !m_UnFollow){
			m_UnForagerId = tPackets[i].Data[0];
			m_UnFollow = true;
			m_pcLEDs->SetAllColors(CColor::BLUE);
			m_pcRABA->SetData(2, m_UnTransferId);
			m_UnHoopCounter++;
			if(m_UnHoopCounter > 1){
				rHoopPosition[m_UnHoopPositionCounter][0] = tPosition.Position.GetX();
				rHoopPosition[m_UnHoopPositionCounter][1] = tPosition.Position.GetY();
				m_UnHoopPositionCounter++;
			}
			m_pcRABA->SetData(3, m_UnHoopCounter);
		}	
		if(m_UnFollow){

			/*Follow forager*/
			if(tPackets[i].Range <= 45 && tPackets[i].Data[0] == m_UnForagerId){
				/* Keep a predefined distance between forager and itself */
				if(tPackets[i].Range >= 20){
					if(tPackets[i].HorizontalBearing.GetValue() < 0.1 &&
							tPackets[i].HorizontalBearing.GetValue() > -0.1) m_pcWheels->SetLinearVelocity(12.0f, 12.0f);
					else{
						if(tPackets[i].HorizontalBearing.GetValue() > 0)
							m_pcWheels->SetLinearVelocity(-10.0f, 10.0f);
						else 
							m_pcWheels->SetLinearVelocity(10.0f, -10.0f);

					}
				}
				else m_pcWheels->SetLinearVelocity(0.0f, 0.0f);	
			}

			/* Stop following prey */
			if(tPackets[i].Range <= 24 && tPackets[i].Data[1] == 2){
				m_UnTransferId = m_UnForagerId; //Save last forager
				m_UnForagerId = 0;
				m_pcLEDs->SetAllColors(CColor::GREEN);
				m_UnFollow = false;
				bObjectMoved = false;
				m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
				m_UnMovePositionCounter = 0;
			}
		}
		else m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
	}				
}

/****************************************/
/****************************************/

void CFootBotFood::Reset() {
	m_UnHoopPositionCounter = 0;
	m_UnTransferId = 0;
	m_UnHoopCounter = 0;
	m_UnForagerId = 0;
	m_UnMovePositionCounter = 0;
	m_pcLEDs->SetAllColors(CColor::GREEN);
	m_UnFollow = false;
	bObjectMoved = true;
	m_pcWheels->SetLinearVelocity(0.0f, 0.0f);	
	m_pcRABA->SetData(3, m_UnHoopCounter);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotFood, "footbot_food_controller")
