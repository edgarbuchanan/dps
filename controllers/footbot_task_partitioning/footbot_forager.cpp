/*
 * AUTHOR: Edgar Buchanan <edgar.buchanan@york.ac.uk>
 *
 * Task partitioning controller for the foot-bot robot platform.
 *
 * This controller used the foraging controller in the
 * ARGoS examples as a template.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/taskPartitioning.argos
 */

/****************************************/
/****************************************/

/*
 * Libraries
 */
#include "footbot_forager.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <cmath>
#include <cstdlib>
#include <unistd.h>

/****************************************/
/****************************************/

/*
 * Constants
 */
#define lowerLimitConstant 1.1 //Defines at what point robots should stop doing task partitioning.
#define minimumTravellingDistance 0.25

/****************************************/
/****************************************/

CFootBotForager::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

/*
 * Retrieve values for the diffusion parameters from xml file
 */
void CFootBotForager::SDiffusionParams::Init(TConfigurationNode& t_node) {
   try {
      CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
      GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
      GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                               ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
      GetNodeAttribute(t_node, "delta", Delta);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
   }
}

/*
 * Retrieve values for the wheel turning parameters from xml file
 */
void CFootBotForager::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/*
 * Retrieve values for the DPS parameters from xml file
 */
void CFootBotForager::SErrorParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "accumulative_error", AccumulativeError);
      GetNodeAttribute(t_node, "partition_type", m_unPartitionType);
      GetNodeAttribute(t_node, "initial_travelling_distance", rInitialTravellingDistance);
      GetNodeAttribute(t_node, "travelling_gain", rTravellingGain);
      GetNodeAttribute(t_node, "travelling_differential", rTravellingDifferential);
      GetNodeAttribute(t_node, "correction_rate", rCorrectionRate);
      GetNodeAttribute(t_node, "item_transfer_time", m_unItemTransfTime);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

CFootBotForager::SStateData::SStateData() :
	ProbRangeDegresTurn(-115.0f, 115.0f),
	ProbRange(0.0f, 100.0f) {}

// Reset to the initial state of the robots
void CFootBotForager::SStateData::Reset() {
   State = STATE_EXPLORING;
   InNest = true;	
}


CFootBotForager::CFootBotForager() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcPositioning(NULL),
   m_pcProximity(NULL),
   m_pcGround(NULL),
   m_pcCamera(NULL),
   m_pcLight(NULL),
   m_pcRNG(NULL) {}

/*
 * Initialise controller
 */
void CFootBotForager::Init(TConfigurationNode& t_node) {
	try {

		/*
		 * Initialise sensors/actuators
		 */
		m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
		m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
		m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
		m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
		m_pcPositioning = GetSensor <CCI_PositioningSensor				>("positioning"		 );
		m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
		m_pcCamera = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>
			("colored_blob_omnidirectional_camera");
		m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
		m_pcGround    = GetSensor  <CCI_FootBotMotorGroundSensor    >("footbot_motor_ground" );

		/*
		 * Parse XML parameters
		 */
		m_sDiffusionParams.Init(GetNode(t_node, "diffusion")); 			// Diffusion parameters
		m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));	// Wheel turning
		m_sErrorParams.Init(GetNode(t_node, "dead_reckoning_error"));	// Get error
	}
	catch(CARGoSException& ex) {
		THROW_ARGOSEXCEPTION_NESTED("Error initializing the foot-bot foraging controller for robot \""
				<< GetId() << "\"", ex);
	}

	/*
	 * Initialise other stuff
	 */
	m_pcCamera->Enable();	// Enable camera */
	if(GetId().length() <= 8) m_UnRobotId = GetId().at(7) - '0' + 1;	/* Get ID */
	else  m_UnRobotId = (GetId().at(7) - '0') * 10 + GetId().at(8) - '0' + 1;	/* ID first digit takes the
		seventh position after the word  'forager' (forager00) */
	m_pcRNG = CRandom::CreateRNG("argos");	/* Create a random number generator. We use the 'argos' category so
		that creation, reset, seeding and cleanup are managed by ARGoS. */
	Reset();
}

/*
 * Execute the appropiate method according to the actual state
 */
void CFootBotForager::ControlStep() {
   switch(m_sStateData.State) {
      case SStateData::STATE_EXPLORING: {
         Explore();
         break;
      }
      case SStateData::STATE_GO_TO_NEST: {
         GoToNest();
         break;
      }
		case SStateData::STATE_GO_TO_SOURCE: {
         GoToSource();
         break;
      }
      case SStateData::STATE_NEIGHBORHOOD_EXPLORATION: {
         NeighborhoodExploration();
         break;
      }
		case SStateData::STATE_WAIT_FOR_TRANSFER: {
         WaitForTransfer();
         break;
      }
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
         break;
      }
   }
}

/*
 * Reset variables
 */
void CFootBotForager::Reset() {
	m_sStateData.Reset();	// Reset state
	m_pcLEDs->SetAllColors(CColor::BLUE);	// Blue indicate exploring
	m_pcRABA->ClearData();	// Delete any data in RAB memory
	bDirectionFlag = false;
	bChangeDirectionFlag = false;
	m_unMissedPreyCounter = 0;
	m_unNighbourhoodExplorationTimer = 0;
	m_unNeighbourhoodExplorationConstant = 1;

	/* Timers to change state */
	m_unWaitingTransferTimer = 0;
	m_unGoToSourceTimer = 0;

	m_unHandoverCounter = 0;
	bOutOfNeighbourhoodRegion = false; // This flag indicates whether the robot is out of the neighbourhood
	rLowerRegionLimit = 0.1;

	/* If initial travelling distance equals to 0 it will pick a random value between 0.5 and 3.5 m */
	if(m_sErrorParams.rInitialTravellingDistance != 0) rTravellingDistance =
			m_sErrorParams.rInitialTravellingDistance / 10.0;
	else rTravellingDistance = ((m_pcRNG->Uniform(m_sStateData.ProbRange) / 100.0) * 30 + 5) / 10;

	/* Variables used when a robot find an item */
	rActualDistanceFromPrey = 100;
	rActualAngleFromPrey = 100; 
	cTargetAngle.SetValue(0);

	bFoodFound = false;
	bFoodAttached = false;
	bObstacleAvoidance = true;
	bHandoverFlag = false;
	m_pcRABA->SetData(0, m_UnRobotId);
	m_pcRABA->SetData(1, 0);
	m_pcRABA->SetData(3, 99);
	rNoiseError = m_pcRNG->Rayleigh(m_sErrorParams.AccumulativeError / 10000);
	cSourcePosition[0] = 0.0;
	cSourcePosition[1] = -1.5;

	/* Variables used to calculate the drift of the estimated food position */
	cEstimateIdealPosition[0] = 0.0;
	cEstimateIdealPosition[1] = -1.5;
	cEstimateActualPosition[0] = 0.0;
	cEstimateActualPosition[1] = 2.025;
	rReferenceAngle = ARGOS_PI * 1.5;
	rAccumulativeAngle = ARGOS_PI * 1.5;

	rGoToNestTimer = 0;

	/* Variables to measure amount of time spent in each task */
	m_UnSearchTimer = 0;
	m_UnHandleObjectsTimer = 0;
	m_UnNavigateTimer = 0;

	/* Variables used to retrieve data to calculate item not found rate */
	m_UnGotLost = 0;	// Counter used whenever an item is not found after neighbourhood exploration
	m_UnTargetFound = 0;	// Counter used whenever an item is found after neighbourhood exploration
	m_UnObjectFoundExploring = 0; // Counter used whenever an item is not found after exploring state

	rSuccessRate = 0;
	bSucccesFlag = false;
	cNestPosition.Set(0,2.205,0);
}

/*
 * Detect if there are any food items and if so approach to them
 */
void CFootBotForager::Camera() {
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs = m_pcCamera->GetReadings();	
	bObstacleAvoidance = true;
	/*
	 * Search objects using camera
	 */
	for(size_t i = 0; i < sBlobs.BlobList.size(); ++i) {
		if(sBlobs.BlobList[i]->Color == CColor::GREEN){
			
			/* Calculate distance from the closest prey */
			if(rActualDistanceFromPrey >= sBlobs.BlobList[i]->Distance) 
				rActualDistanceFromPrey = sBlobs.BlobList[i]->Distance;
			
			if(sBlobs.BlobList[i]->Distance <= 80) 
				rActualAngleFromPrey = sBlobs.BlobList[i]->Angle.GetValue();

			/* Approach prey if it closer than 75 cm */
			if(sBlobs.BlobList[i]->Distance <= 75) {
				if(bFoodFound == false) {
					bChangeDirectionFlag = true;
				}
				bFoodFound = true;
				m_pcLEDs->SetAllColors(CColor::RED);
				m_pcRABA->SetData(0, m_UnRobotId);
			}

			/* Travel straight if close enough */
			if((sBlobs.BlobList[i]->Distance <= 18) &&
					(cActualAngle.GetValue() > cTargetAngle.GetValue() - 2.5 &&
							cActualAngle.GetValue() < cTargetAngle.GetValue() + 2.5))
				bObstacleAvoidance = false;
			
			/* Attached to object */
			if(rActualDistanceFromPrey <= 14){
				m_pcRABA->SetData(1, 1);	// Activate Prey
				bFoodFound = false;
				if(bFoodAttached == false){
					bChangeDirectionFlag = true;
				}
				bFoodAttached = true;
				bHandoverFlag = true;
				bObstacleAvoidance = true;
			}	
		}
	}
	
}

/*
 * Calculate vector from light source
 */
CVector2 CFootBotForager::CalculateVectorToLight() {
   
	/* Get readings from light sensor */
	const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
	/* Sum them together */
	CVector2 cAccumulator;
	for(size_t i = 0; i < tLightReads.size(); ++i) {
		cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
	}
	/* If the light was perceived, return the vector */
	if(cAccumulator.Length() > 0.0f) 
		return CVector2(1.0f, cAccumulator.Angle());
	   
	/* Otherwise, return zero */
	else
		return CVector2();
	
}

/*
 * Change partition length where:
 * Case 1: No partitions
 * Case 2: Static partition
 * Case 3: Dynamic partition with step mechanism
 * Case 4: Dynamic partition with exponential mechanism
 */
void CFootBotForager::PartitionLength(bool bSearchResult) {
	const CCI_PositioningSensor::SReading& tPosition = m_pcPositioning->GetReading();
	if(rSuccessRate < -10) 
		rSuccessRate = -10;

	if(rSuccessRate > 10) 
		rSuccessRate = 10;

	cSourcePosition = tPosition.Position;

	/*
	 * Select partitioning method
	 */
	switch(m_sErrorParams.m_unPartitionType){

		/* Nonpartitioning */
		case 1:
			rLowerRegionLimit = 0.01;
			break;

		/* Static partitioning */
		case 2:
			rTotalDistance = Distance(cSourcePosition, cNestPosition);
			rLowerRegionLimit = rTotalDistance - rTravellingDistance;
			if(rLowerRegionLimit < lowerLimitConstant) 
				rLowerRegionLimit = 0.01;

			break;

		/* Dynamic partitioning with step mechanism using alpha */
		case 3:

			/* Different compensations according to rate */
			if(bSearchResult && !IsExploring()){

			/* Any angle partitioning */
				rTotalDistance = Distance(cSourcePosition, cNestPosition);
				rTravellingDistance += m_sErrorParams.rTravellingGain / 10000 *
						(100 - m_sErrorParams.rCorrectionRate);
				rLowerRegionLimit = rTotalDistance - rTravellingDistance;
				if(rLowerRegionLimit < lowerLimitConstant) rLowerRegionLimit = 0.01;
			}

			if(!bSearchResult){
				rTravellingDistance -= m_sErrorParams.rTravellingGain / 10000 * m_sErrorParams.rCorrectionRate;
				if(rTravellingDistance <= 0.5) 
					rTravellingDistance = 0.5;

				rLowerRegionLimit = rTotalDistance - rTravellingDistance;
				if(rLowerRegionLimit < lowerLimitConstant) 
					rLowerRegionLimit = 0.01;

			}
			break;

		/* Dynamic partitioning with exponential mechanism without using alpha */
		case 4:
			if(bSearchResult && !IsExploring()){
				rTotalDistance = Distance(cSourcePosition, cNestPosition);
				rTravellingDistance += (m_sErrorParams.rTravellingGain / 100) *
						exp(-(m_sErrorParams.rTravellingDifferential / 100) * (rSuccessRate));
				rLowerRegionLimit = rTotalDistance - rTravellingDistance;
				if(rLowerRegionLimit < lowerLimitConstant) rLowerRegionLimit = 0.01;
			}

			if(!bSearchResult){
				rTravellingDistance -= (m_sErrorParams.rTravellingGain / 100) *
						exp(-(m_sErrorParams.rTravellingDifferential / 100)*rSuccessRate);
				if(rTravellingDistance < 0.25)rTravellingDistance = 0.25;
				rLowerRegionLimit = rTotalDistance - rTravellingDistance;
				if(rLowerRegionLimit < lowerLimitConstant) rLowerRegionLimit = 0.01;
			}

			break;
		default: {
			LOGERR << "Wrong partition type selected!" << std::endl;
			break;
		}

	}

	/*
	 * Storage new position
	 */
	if(rTravellingDistance < minimumTravellingDistance) 
		rTravellingDistance = minimumTravellingDistance;

	cSourcePosition = tPosition.Position;
	cEstimateIdealPosition = tPosition.Position;
	rReferenceAngle = atan2(cSourcePosition[1] - cNestPosition[1], cSourcePosition[0] - cNestPosition[0]);
	if(rReferenceAngle < 0) 
		rReferenceAngle += 2 * ARGOS_PI;
}

/*
 * Check if robot is in the nest
 */
void CFootBotForager::UpdateState() {

	/* Reset state flags */
	m_sStateData.InNest = false;
	/* Read ground sensor */
	const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
	if(tGroundReads[2].Value == 0.0f && tGroundReads[3].Value == 0.0f)  // Check if robot is in nest
		m_sStateData.InNest = true;
	
}

/*
 * Detect if there are any obstacles surrounding robot
 * and calculate vector where there are no obstacles
 */
CVector2 CFootBotForager::DiffusionVector(bool& b_collision) {
   
	/* Get readings from proximity sensor */
	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
	/* Sum them together */
	CVector2 cDiffusionVector;
	/* Use front sensors only */
	for(size_t i = 0; i < 24 && i < tProxReads.size(); ++i) {
		cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
	}

	/*
	* If the angle of the vector is small enough and the closest obstacle is far enough,
	* ignore the vector and go straight, otherwise return it
	*/
	if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
			cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
		
		b_collision = false;
		return CVector2::X;
	}

	else {
		b_collision = true;
		cDiffusionVector.Normalize();
		return -cDiffusionVector;
	}
}

/*
 * Robot's motion description
 */
void CFootBotForager::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   
	const CCI_PositioningSensor::SReading& tPosition = m_pcPositioning->GetReading();
	Real rErrorAngle;
	Real rDirectionTurn = m_pcRNG->Uniform(m_sStateData.ProbRange);

	CRadians cHeadingAngle = c_heading.Angle().SignedNormalize(); // Get the heading angle
	Real fHeadingLength = c_heading.Length(); // Get the length of the heading vector
	Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed); // Clamp the speed
	Real rDistanceFromCenter = Distance(tPosition.Position, cSourcePosition);

	/* Calculate actual angle */
	CRadians cEulerAngles[3];
	CRadians (&refToAngles)[3] = cEulerAngles;
	tPosition.Orientation.ToEulerAngles(refToAngles[2], refToAngles[1], refToAngles[0]);
	cActualAngle = ToDegrees(cEulerAngles[2]).UnsignedNormalize();

	/* Source angle */
	if(IsReturningToSource()){
		cTargetAngle.SetValue(atan2((cSourcePosition[1] - tPosition.Position[1]) ,
				(cSourcePosition[0] - tPosition.Position[0])) * 180 / ARGOS_PI);
		if(cTargetAngle.GetValue() < 0) 
			cTargetAngle.SetValue(cTargetAngle.GetValue() + 360);

		if(cTargetAngle.GetValue() > 360) 
			cTargetAngle.SetValue(cTargetAngle.GetValue() - 360);

		cTargetAngle.SetValue(cTargetAngle.GetValue() - cActualAngle.GetValue());
		cHeadingAngle += ToRadians(cTargetAngle).SignedNormalize();
	}

	/* Explore angle */
	if ((IsExploringNeighbourhood() || IsExploring()) && !bDirectionFlag){
		cTargetAngle.SetValue(m_pcRNG->Uniform(m_sStateData.ProbRangeDegresTurn) + cActualAngle.GetValue());
		cTargetAngle = cTargetAngle.UnsignedNormalize();
	}

	/* Approach prey */
	if(bFoodFound && !bFoodAttached){
		if(rPreviousAngleFromPrey == rActualAngleFromPrey){
			bFoodFound = false;
			m_pcLEDs->SetAllColors(CColor::BLUE);
			m_pcRABA->SetData(0, 0);
		}

		cTargetAngle.SetValue(rActualAngleFromPrey * 180 / ARGOS_PI + 180);
		cActualAngle.SetValue(180);
		rPreviousAngleFromPrey = rActualAngleFromPrey;
	}

	/* If out of neighbourhood turn and return */
	if (IsExploringNeighbourhood() && rDistanceFromCenter > 0.45 && !bDirectionFlag){
		cTargetAngle.SetValue(cActualAngle.GetValue() + 180);
		cTargetAngle = cTargetAngle.UnsignedNormalize();
		rReferenceAngle = cTargetAngle.GetValue();
		m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::DIRECTION_TURN;
		bDirectionFlag = true;
		bOutOfNeighbourhoodRegion = true;
	}

	/* Generate a new random angle to turn */
	if((rDirectionTurn < 5 && (IsExploringNeighbourhood() || IsExploring())) && !bDirectionFlag){
		m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::DIRECTION_TURN;
		rReferenceAngle = cTargetAngle.GetValue();
		bDirectionFlag = true;
	}

	/* Obstacle avoidance */
	else{

		if(!bDirectionFlag){
			if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) 
				/* No Turn, heading angle very small */
				m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
			
			else if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) 
				/* Hard Turn, heading angle very large */
				m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
			
			else if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN &&
					  Abs(cHeadingAngle) > m_sWheelTurningParams.SoftTurnOnAngleThreshold) 
				/* Soft Turn, heading angle in between the two cases */
				m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
			
			else 
				m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
		}
	}

	/* Do not turn */
	if(!bObstacleAvoidance){
		m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
	}

	/* Assign speed to motors */
	switch(m_sWheelTurningParams.TurningMechanism) {

		/* Move straight*/
		case SWheelTurningParams::NO_TURN: {
			fSpeed1 = fSpeed2 = fBaseAngularWheelSpeed;
			if(!bHandoverFlag && (IsReturningToNest() || IsReturningToSource())){
				rGoToNestTimer++;
				rGoToSourceTimer = rGoToNestTimer;
			}
			break;
		}

		/* Move straight with a slight curvature in the trajectory */
		case SWheelTurningParams::SOFT_TURN: {

			/* Slow down wheels */
			Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) /
					m_sWheelTurningParams.HardTurnOnAngleThreshold;
			if(cHeadingAngle >= CRadians::ZERO) {
				fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
				fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
			}
			else{
				fSpeed2 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
				fSpeed1 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
			}
			break;
		}

		/* Turn in the same axis */
		case SWheelTurningParams::HARD_TURN: {
			/* Opposite wheel speeds */
			fSpeed1 = fSpeed2 = m_sWheelTurningParams.MaxSpeed;
			if(cHeadingAngle >= CRadians::ZERO) 
				fSpeed1 *= -1;

			else 
				fSpeed2 *= -1;

			break;
		}

		/* Align to a specific direction */
		case SWheelTurningParams::DIRECTION_TURN: {

			/* Wheel speeds based on current turning state */
			if(cActualAngle.GetValue() > cTargetAngle.GetValue() - 2.5 && cActualAngle.GetValue() <
					cTargetAngle.GetValue() + 2.5){

				/* Just go straight */
				fSpeed1 = fSpeed2 = fBaseAngularWheelSpeed;
				
				/* If out of neighbourhood return to center */
				if (bOutOfNeighbourhoodRegion == true && m_unMissedPreyCounter < 50)
					m_unMissedPreyCounter++;
				
				else{
					m_unMissedPreyCounter = 0;
					bDirectionFlag = false;
					bOutOfNeighbourhoodRegion = false;
				}
			}
			else{
				fSpeed1 = fSpeed2 = fBaseAngularWheelSpeed * 0.25;
				if(abs(cActualAngle.GetValue() - cTargetAngle.GetValue()) < 180) {
					if(cActualAngle.GetValue() - cTargetAngle.GetValue() >= 0) 
						fSpeed2 *= -1;

					else 
						fSpeed1 *= -1;

				}
				else{
					if(cActualAngle.GetValue() - cTargetAngle.GetValue() >= 0) 
						fSpeed1 *= -1;

					else 
						fSpeed2 *= -1;

				}
			}
			break;
		}
	}

	/* Handover sequence */
	if(bHandoverFlag && m_unHandoverCounter < m_sErrorParams.m_unItemTransfTime){
		fSpeed1 = fSpeed2 = 0;
		if(!IsReturningToNest() && !IsWaiting()){
			m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::DIRECTION_TURN;
			bDirectionFlag = true;
		}
		m_unHandoverCounter++;
		m_UnHandleObjectsTimer++;
		/* Communicate with food robot */
		if(IsReturningToNest() && m_unHandoverCounter == 5){
			const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
			for(size_t i = 0; i < tPackets.size(); ++i ) {
				if(tPackets[i].Range <= 29){
					if(tPackets[i].Data[3] < 2) 
						m_unNeighbourhoodExplorationConstant = 1;

					else 
						m_unNeighbourhoodExplorationConstant = 3;

				}

			}
		}
	}
	else{
		bHandoverFlag = false;
		m_unHandoverCounter = 0;
	}

	/* Finally, set the wheel speeds */
	m_pcWheels->SetLinearVelocity(fSpeed1, fSpeed2);

	/* Real Implementation of Error */
	if((fSpeed1 != 0 || fSpeed2 != 0) && !IsExploringNeighbourhood()){

		/* Last good implementation */
		fSpeed2 += 0.00001;
		fSpeed1 /= 1000;
		fSpeed2 /= 1000;

		fSpeed1 = 0.01000001;	// Avoid division by zero
		fSpeed2 = 0.010;

		CVector3 rErrorDrift;
		rErrorDrift[0] -= 2 * ((0.14 * (fSpeed2 + fSpeed1)) / (2 * (fSpeed2 - fSpeed1))) *
				(sin((fSpeed2 - fSpeed1) * rGoToNestTimer  / 0.14 + rReferenceAngle) - sin(rReferenceAngle));
		rErrorDrift[1] += 2 * ((0.14 * (fSpeed2 + fSpeed1)) / (2 * (fSpeed2 - fSpeed1))) *
				(cos((fSpeed2 - fSpeed1) * rGoToNestTimer / 0.14 + rReferenceAngle) - cos(rReferenceAngle));

		if(fSpeed1 < 0)
			fSpeed1 += rNoiseError * fSpeed1;
		
		if(fSpeed2 > 0)
			fSpeed2 += rNoiseError * fSpeed2;

		rErrorDrift[0] += 2 * ((0.14 * (fSpeed2 + fSpeed1)) / (2 * (fSpeed2 - fSpeed1))) *
				(sin((fSpeed2 - fSpeed1) * rGoToNestTimer / 0.14 + rReferenceAngle) - sin(rReferenceAngle));
		rErrorDrift[1] -= 2 *((0.14 * (fSpeed2 + fSpeed1)) / (2 * (fSpeed2 - fSpeed1))) *
				(cos((fSpeed2 - fSpeed1) * rGoToNestTimer / 0.14 + rReferenceAngle) - cos(rReferenceAngle));

		cSourcePosition = cEstimateIdealPosition + rErrorDrift;
	}
}

/*
 * Explore state
 */
void CFootBotForager::Explore() {
	/* We switch to 'go to nest' in one situation only, if we have a food item*/
	const CCI_PositioningSensor::SReading& tPosition = m_pcPositioning->GetReading();
	Camera();

	if(bFoodAttached) {

		/* Reset */
		m_unNighbourhoodExplorationTimer = 0;
		m_unWaitingTransferTimer = 0;
		bDirectionFlag = false;
		bChangeDirectionFlag = true;
		bFoodAttached = false;
		bSucccesFlag = true;

		/* Save position and decrease threshold */
		cSourcePosition = tPosition.Position;
		m_UnObjectFoundExploring++;
		rTotalDistance = Distance(cSourcePosition, cNestPosition);
		rLowerRegionLimit = rTotalDistance - rTravellingDistance;
		if(rLowerRegionLimit < lowerLimitConstant) rLowerRegionLimit = 0.01;
		PartitionLength(true);

		/* Change state */
		m_sStateData.State = SStateData::STATE_GO_TO_NEST;
	}
	else {

		/* Get the diffusion vector to perform obstacle avoidance */
		bool bCollision;
		if(!bHandoverFlag) m_UnSearchTimer++;
		CVector2 cDiffusion = DiffusionVector(bCollision);
		SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
	}

}

/*
 * Go to nest
 */
void CFootBotForager::GoToNest() {

	UpdateState();
	m_pcRABA->SetData(1, 0);
	if(!bHandoverFlag) m_UnNavigateTimer++;
	bSucccesFlag = false;

	/* If robot is inside of the nest enter to go to source state */
	if(m_sStateData.InNest) {

		/* Reset */
		m_unWaitingTransferTimer = 0;
		m_unGoToSourceTimer = 0;

		/* Reset camera variables */
		bFoodTransfered = false;
		bFoodAttached = false;
		bFoodFound = false;
		rActualDistanceFromPrey = 100;
		m_pcLEDs->SetAllColors(CColor::BLUE);
		m_pcRABA->SetData(1, 0);
		m_sStateData.State = SStateData::STATE_GO_TO_SOURCE; // Change state
	}

	/*Calculate distance between nest and source and between nest and actual position*/
	const CCI_PositioningSensor::SReading& tPosition = m_pcPositioning->GetReading();
	rTotalDistance = Distance(cSourcePosition, cNestPosition);
	rActualDistance = Distance(tPosition.Position, cNestPosition);
	
	/*
	 * While distance between nest and robot is greater than threshold keep moving,
	 * otherwise enter to 'Wait for Transfer'
	 */
	if(rActualDistance < rLowerRegionLimit && !IsReturningToSource() && m_unWaitingTransferTimer < 1800) {
		m_pcRABA->SetData(1, 2); // Change state for food
		m_sStateData.State = SStateData::STATE_WAIT_FOR_TRANSFER; // Change state
	}

	/* Keep going */
	bool bCollision;
	SetWheelSpeedsFromVector(
		m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) +
		m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());
}

/***************************/
/*	Go to source	   */
/***************************/

void CFootBotForager::GoToSource() {

	const CCI_PositioningSensor::SReading& tPosition = m_pcPositioning->GetReading();
	if(!bHandoverFlag) m_UnNavigateTimer++;

	/* If the estimated distance to the source is not decreasing over 30s enter to exploration state.
	 * This is done to detect whether the estimated position is out of range.
	 */
	Real rDistanceVector = Distance(tPosition.Position,cSourcePosition);
	if(rDistanceVector < rPreviousDistanceFromSource) 
		m_unGoToSourceTimer = 0;

	else 
		m_unGoToSourceTimer++;

	if(m_unGoToSourceTimer > 300){
		m_unGoToSourceTimer = 0;
		m_unNighbourhoodExplorationTimer = 0;
		m_unWaitingTransferTimer = 0;
		m_sStateData.State = SStateData::STATE_EXPLORING;
		rGoToNestTimer = 0;
		rSuccessRate--;
		PartitionLength(false);
	}
	rPreviousDistanceFromSource = rDistanceVector;

	/* If food attached enter to go to nest state */
	if(bFoodAttached) {

		/* Reset */
		m_unGoToSourceTimer = 0;
		m_unNighbourhoodExplorationTimer = 0;
		bDirectionFlag = false;
		bChangeDirectionFlag = true;
		bFoodAttached = false;

		/* Save position and decrease threshold */
		rSuccessRate++;
		bSucccesFlag = true;
		PartitionLength(true);
		m_sStateData.State = SStateData::STATE_GO_TO_NEST; // Change state
	}

	/* Detect objects once in neihbourhood */
	if(tPosition.Position[0] < cSourcePosition[0] + 0.1 && tPosition.Position[0] > cSourcePosition[0] - 0.1
			&& tPosition.Position[1] < cSourcePosition[1] + 0.1 && tPosition.Position[1] > cSourcePosition[1] - 0.1){
		Camera();
	}
	/* Enter to neighbourhood exploration state if estimated position is reached */
	if(tPosition.Position[0] < cSourcePosition[0] + 0.05 && tPosition.Position[0] > cSourcePosition[0] - 0.05
			&& tPosition.Position[1] < cSourcePosition[1] + 0.05 && tPosition.Position[1] > cSourcePosition[1] - 0.05){
		m_sStateData.State = SStateData::STATE_NEIGHBORHOOD_EXPLORATION;
		rGoToNestTimer = 0;
	}

	/* Keep going */
	bool bCollision;
	SetWheelSpeedsFromVector(
			m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision));
}

/*
 * Neighbourhood Exploration
 */
void CFootBotForager::NeighborhoodExploration() {

	const CCI_PositioningSensor::SReading& tPosition = m_pcPositioning->GetReading();
	if(!bHandoverFlag) m_UnSearchTimer++;
	m_unNighbourhoodExplorationTimer++;
	Camera();

	/*
	 * Explore neighbourhood for one minute.
	 * If no object is found enter to exploring state
	 */
	if(m_unNighbourhoodExplorationTimer > 600 * m_unNeighbourhoodExplorationConstant){

		/* Reset */ 
		m_unNighbourhoodExplorationTimer = 0;
		m_unGoToSourceTimer = 0;
		m_UnGotLost++;
		rSuccessRate--;
		PartitionLength(false); // Increase threshold
		m_sStateData.State = SStateData::STATE_EXPLORING; // Change state
	}

	/* If food attached enter to go to nest state */
	if(bFoodAttached) {

		/* Reset */
		m_unNighbourhoodExplorationTimer = 0;
		m_unGoToSourceTimer = 0;
		bDirectionFlag = false;
		bChangeDirectionFlag = true;
		m_UnTargetFound++;
		rSuccessRate++;
		bSucccesFlag = true;
		rNoiseError = m_pcRNG->Rayleigh(m_sErrorParams.AccumulativeError / 10000);
		bFoodAttached = false;
		PartitionLength(true); // Save position and decrease threshold
		m_sStateData.State = SStateData::STATE_GO_TO_NEST; // Change state
	}

	/* Keep going */
	bool bCollision;
	SetWheelSpeedsFromVector(
			m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision));
}

/*
 * Wait for transfer
 */
void CFootBotForager::WaitForTransfer() {

	m_pcWheels->SetLinearVelocity(0, 0);
	m_pcRABA->SetData(1, 0); 
	m_unWaitingTransferTimer++;
	if(!bHandoverFlag) m_UnHandleObjectsTimer++;

	/*
	 * Wait up to 3 minutes for a robot to come and take the object
	 * This is done in order to avoid deadlocks where all robots are waiting forever
	 */
	if(m_unWaitingTransferTimer > 1800){

		/* Attach to food again */
		m_pcLEDs->SetAllColors(CColor::BLUE);
		m_pcRABA->SetData(1, 1); 

		/* Change state */
		m_sStateData.State = SStateData::STATE_GO_TO_NEST;
	}

	const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();

	/* Wait for food to signal that transfer is complete */
	for(size_t i = 0; i < tPackets.size(); ++i ) {
		if(tPackets[i].Data[2] == m_UnRobotId && tPackets[i].Range <= 30){

			/* Reset */ 
			m_unWaitingTransferTimer = 0;
			bChangeDirectionFlag = true;

			/* Deattach food */    
			bFoodAttached = false;
			bDirectionFlag = true;
			m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::DIRECTION_TURN;
			m_pcLEDs->SetAllColors(CColor::BLUE);
			rActualDistanceFromPrey = 100;
			m_pcRABA->SetData(1, 0);
			m_sStateData.State = SStateData::STATE_GO_TO_SOURCE; 
			bHandoverFlag = true;			
		}
	}
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotForager, "footbot_forager_controller")
