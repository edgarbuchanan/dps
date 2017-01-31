/*
 * AUTHOR: Edgar Buchanan <edgar.buchanan@york.ac.uk>
 *
 * This loop function has two main functions:
 * - Regulate the number of the food items by removing and adding food items
 * to the environment.
 * - Create the files and fill them with data for statistical purposes.
 *
 * This loop function is meant to be used with the configuration file:
 *    experiments/taskPartitioning.argos
 */

#include "task_partitioning_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_task_partitioning/footbot_forager.h>
#include <controllers/footbot_task_partitioning/footbot_food.h>

#define ELEM_SWAP(a,b) { register int t=(a);(a)=(b);(b)=t; }

/***************************************************/
/***************************************************/

CForagingLoopFunctions::CForagingLoopFunctions() :	
 m_pcFloor(NULL),
 m_pcRNG(NULL){}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Init(TConfigurationNode& t_node) {
	try {
		TConfigurationNode& tForaging = GetNode(t_node, "foraging");

		/* Get a pointer to the floor entity */
		m_pcFloor = &GetSpace().GetFloorEntity();

		/* Create a new RNG */
		m_pcRNG = CRandom::CreateRNG("argos");

		/* Get distance between nest and source from XML */
		GetNodeAttribute(tForaging, "nest_source_distance", rNestSourceDistance);

		/*
		 * For 'output' file
		 * - Get the output file name from XML
		 * - Open the file and erase its contents
		 * - Write header row
		 */
		GetNodeAttribute(tForaging, "output", m_strOutput);
		m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
		m_cOutput << "# Clock,\tcollected_food,\tsubtasks_average,\tdistance_average,\tsearch,\t"
				"handle_objects,\tnavigate,\tlost_source,\tdistance_robots," << std::endl;

		/* transferpoints file */
		GetNodeAttribute(tForaging, "transfer_points", m_strTransferPoints);
		m_cTransferPoints.open(m_strTransferPoints.c_str(), std::ios_base::trunc | std::ios_base::out);
		m_cTransferPoints << "# Clock, X, Y, Z, ID, P, SuccessRate" << std::endl;

		/* Get parameters for model */
		GetNodeAttribute(tForaging, "params_for_model", m_strParamsForModel);
		m_cParamsForModel.open(m_strParamsForModel.c_str(), std::ios_base::trunc | std::ios_base::out);
		m_cParamsForModel << "ItemsFound_Exploring,\tItemsFound_Navigating,\tItemsLost_Navigating," << std::endl;
	}
	catch(CARGoSException& ex) {
		THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
	}
	Reset();
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Reset() {

	/*
	 * For 'output' file
	 * - Close file
	 * - Open the file and erase its contents
	 * - Write header row
	 */
	m_cOutput.close();
	m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
	m_cOutput << "# Clock,\tcollected_food,\tsubtasks_average,\tdistance_average,\tsearch,\thandle_objects,"
			"\tnavigate,\tlost_source,\tdistance_robots," << std::endl;

	/* transferpoints file */
	m_cTransferPoints.close();
	m_cTransferPoints.open(m_strTransferPoints.c_str(), std::ios_base::trunc | std::ios_base::out);
	m_cTransferPoints << "# Clock, X, Y, Z, ID, P, SuccessRate" << std::endl;

	/* paramsformodel file */
	m_cParamsForModel.close();
	m_cParamsForModel.open(m_strParamsForModel.c_str(), std::ios_base::trunc | std::ios_base::out);
	m_cParamsForModel << "ItemsFound_Exploring,\tItemsFound_Navigating,\tItemsLost_Navigating," << std::endl;

	/* Initialise variables */
	m_UnPreyCounter = 0;
	m_unTimeCounter = 0;
	bSimulationFinished = false;
	for(int i=0; i <= 9; i++){
		m_UnHopCounter[i] = 0;
	}
	m_unCounterTransPos = 0;
	Real rSourcePosition = -rNestSourceDistance / 200 + 0.05;
	cSourcePosition.Set(0, rSourcePosition, 0);
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Destroy() {
   	/* Close files */
	m_cOutput.close();
	m_cTransferPoints.close();
	m_cParamsForModel.close();
}

/****************************************/
/****************************************/

CColor CForagingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) { 
	
	/* Draw home region */  
	Real rNestPosYUp = rNestSourceDistance / 200 + 0.25;
	Real rNestPosYDown = rNestSourceDistance / 200 - 0.2;
	if(c_position_on_plane.GetX() > -0.7f && c_position_on_plane.GetX() < 0.7f && c_position_on_plane.GetY()
			< rNestPosYUp && rNestPosYDown < c_position_on_plane.GetY()) {
		return CColor::BLACK;
	}

	/* Draw positions where objects were transferred */
	Real rSteps = (255*(floor(m_unCounterTransPos/255)+1))/(m_unCounterTransPos+1);
	for(UInt32 i = m_unCounterTransPos; i > 0; i--) {
		if((c_position_on_plane - cTransferPosition[i]).SquareLength() < 0.004) {
			return CColor(255-rSteps*(i), 255-rSteps*(i), rSteps*(i), 255);
		}
	}
	return CColor::WHITE;
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::PreStep() {

	/* Analyse all footbots in arena */
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

	/* Variables used to retrieve data to calculate item not found rate */
	UInt32 m_UnObjectFound_Exploring[99] = {0};
	UInt32 m_UnObjectFound_Navigating[99] = {0};
	UInt32 m_UnObjectLost_Navigating[99] = {0};	

	UInt32 m_UnForagersCounter = 0;
	Real rDistanceCollector = 0;
	Real rGotLostAverage = 0;
	Real rGotTargetFoundAverage = 0;

	/* Variables to measure amount of time spent in each task */
	UInt32 m_UnSearchRobotCollection [99] = {0};
	UInt32 m_UnHandleObjectsCollection [99] = {0};
	UInt32 m_UnNavigateAverage [99] = {0};
	UInt32 m_UnGotLost[99] = {0};
	UInt32 m_UnTargetFound[99] = {0};
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
			it != m_cFootbots.end();
			++it) {

		/* Get handle to foot-bot entity*/
		CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);

		/* Get the position of the foot-bot on the ground as a CVector2 */
		CVector2 cPos;
		cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
			  cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

		/* Get ID */
		char m_UnRobotId = cFootBot.GetId().at(2);

		/* If robot is food */
		if(m_UnRobotId == 'o'){

			/* Get handle to food foot-bot controller */
			CFootBotFood& cController = dynamic_cast<CFootBotFood&>(cFootBot.GetControllableEntity().GetController());

			/* Simulate moving item in front of the robot if partition length is reached */
			if(!cController.bObjectMoved){
				CVector3 cActualPosition;
				cActualPosition = cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position;
				CRadians cEulerAngles[3];
				CRadians (&refToAngles)[3] = cEulerAngles;
				cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(refToAngles[2],
						refToAngles[1], refToAngles[0]);
				CRadians cActualAngle = cEulerAngles[2].UnsignedNormalize();

				/* Change coordings according the angle that the food robot is facing to */
				cActualPosition.SetX(cActualPosition.GetX() + 0.35 * ((ARGOS_PI-cActualAngle.GetValue())/ARGOS_PI));
				cActualPosition.SetY(cActualPosition.GetY() + 0.55 * ((ARGOS_PI-cActualAngle.GetValue())/ARGOS_PI));

				/* Move if there are no obstacles */
				if(cFootBot.GetEmbodiedEntity().MoveTo(cActualPosition,
					cFootBot.GetEmbodiedEntity().GetOriginAnchor().OffsetOrientation,0))
					cController.bObjectMoved = true;
			}

			/* If item is inside of nest return to waiting zone */
			Real rNestPosYUp = rNestSourceDistance / 200 + 0.25;
			Real rNestPosYDown = rNestSourceDistance / 200 - 0.5;

			/* Move food foot-bot to origin whenever it reaches the nest and reset values*/
			if(cPos.GetX() > -1.0f && cPos.GetX() < 1.0f && rNestPosYDown < cPos.GetY() && rNestPosYUp > cPos.GetY()){
				cFootBot.GetEmbodiedEntity().MoveTo(cController.cFoodInitialPosition,
						cFootBot.GetEmbodiedEntity().GetOriginAnchor().OffsetOrientation,0);
				UInt32 m_unZeroCounter = 0;
				m_unHoopCounterAverage = 0;
				for(int i=9; i >= 0; i--){
					m_UnHopCounter[i] = m_UnHopCounter[i - 1];
				}
				m_UnHopCounter[0] = cController.m_UnHoopCounter;
				for(int i=0; i <= 9; i++){
					if(m_UnHopCounter[i] == 0) m_unZeroCounter++;
					m_unHoopCounterAverage += m_UnHopCounter[i];
				}
				if(m_unZeroCounter == 10) m_unHoopCounterAverage = 0;
				else m_unHoopCounterAverage = m_unHoopCounterAverage / (10 - m_unZeroCounter);

				++m_UnPreyCounter;
				cController.m_UnHoopCounter = 0;
				cController.Reset();

				/* PRINT in Terminal */
				printf("%i\t%0.1f\t%i\t%0.3f\t%llu\t%llu\t%llu\t%0.3f\n", GetSpace().GetSimulationClock(),
						m_unHoopCounterAverage, m_UnPreyCounter, rDistanceAverageCounter[m_unTimeCounter],
						m_UnSearchTimer[m_unTimeCounter], m_UnHandleObjectsTimer[m_unTimeCounter],
						m_UnNavigateTimer[m_unTimeCounter], rGetLost[m_unTimeCounter]);
			}

			/* Try to replace items in source */
			if(2.5 < cPos.GetY()){
				CVector3 cMoveOrigin;
				cMoveOrigin.Set(0, 0, 0);
				cFootBot.GetEmbodiedEntity().MoveTo(cSourcePosition,
						cFootBot.GetEmbodiedEntity().GetOriginAnchor().OffsetOrientation, 0);
				cMoveOrigin.SetX(cSourcePosition.GetX() + 0.175);
				cMoveOrigin.SetY(cSourcePosition.GetY() + 0.175);
				cFootBot.GetEmbodiedEntity().MoveTo(cMoveOrigin,
						cFootBot.GetEmbodiedEntity().GetOriginAnchor().OffsetOrientation, 0);
				cMoveOrigin.SetX(cSourcePosition.GetX() - 0.175);
				cFootBot.GetEmbodiedEntity().MoveTo(cMoveOrigin,
						cFootBot.GetEmbodiedEntity().GetOriginAnchor().OffsetOrientation, 0);
				cMoveOrigin.SetY(cSourcePosition.GetY() - 0.175);
				cFootBot.GetEmbodiedEntity().MoveTo(cMoveOrigin,
						cFootBot.GetEmbodiedEntity().GetOriginAnchor().OffsetOrientation, 0);
				cMoveOrigin.SetX(cSourcePosition.GetX() + 0.175);
				cFootBot.GetEmbodiedEntity().MoveTo(cMoveOrigin,
						cFootBot.GetEmbodiedEntity().GetOriginAnchor().OffsetOrientation, 0);
			}

			/*
			 * Avoid ovelapping objects by checking if there any collisions.
			 * If there is any return object to original position.
			 */
			if((cSourcePosition.GetX() + 0.375) > cPos.GetX() &&(cSourcePosition.GetX() - 0.375) < cPos.GetX()
					&& (cSourcePosition.GetY() + 0.375) > cPos.GetY() &&(cSourcePosition.GetY() - 0.375) < cPos.GetY()){
				if(cController.m_UnFollow == false && cFootBot.GetEmbodiedEntity().IsCollidingWithSomething() == true)
					cFootBot.GetEmbodiedEntity().MoveTo(cController.cFoodInitialPosition,
							cFootBot.GetEmbodiedEntity().GetOriginAnchor().OffsetOrientation,0);
			}
		}

		/* If the robot is a forager */
		if(m_UnRobotId == 'r'){
			CFootBotForager& cController = dynamic_cast<CFootBotForager&>(cFootBot.GetControllableEntity().GetController());
			cController.cStartingPosition = cFootBot.GetEmbodiedEntity().GetOriginAnchor().OffsetPosition;

			/* Retrieve data from each robot and gather it in arrays */
			m_UnSearchRobotCollection[m_UnForagersCounter] = cController.m_UnSearchTimer;
			m_UnHandleObjectsCollection[m_UnForagersCounter] = cController.m_UnHandleObjectsTimer;
			m_UnNavigateAverage[m_UnForagersCounter] = cController.m_UnNavigateTimer;
			m_UnGotLost[m_UnForagersCounter] = cController.m_UnGotLost;
			m_UnTargetFound[m_UnForagersCounter] = cController.m_UnTargetFound;

			rGotTargetFoundAverage += cController.m_UnTargetFound;
			rGotLostAverage += cController.m_UnGotLost;
			m_UnForagersCounter++;
			rGetLost[m_unTimeCounter] = rGotLostAverage / (rGotLostAverage + rGotTargetFoundAverage);
			rDistanceCollector += cController.rTravellingDistance;
			rDistanceAverageCounter[m_unTimeCounter] = rDistanceCollector / m_UnForagersCounter;

			/* Get median from the swarm to measure amount of time spent in each task */
			m_UnSearchTimer[m_unTimeCounter] = iGetMedian(m_UnSearchRobotCollection, m_UnForagersCounter);
			m_UnHandleObjectsTimer[m_unTimeCounter] = iGetMedian(m_UnHandleObjectsCollection, m_UnForagersCounter);
			m_UnNavigateTimer[m_unTimeCounter] = iGetMedian(m_UnNavigateAverage, m_UnForagersCounter);

			/* Get median from the swarm to calculate item not found rate */
			m_UnObjectFound_Navigating[m_UnForagersCounter] += cController.m_UnTargetFound;
			m_UnObjectFound_Exploring[m_UnForagersCounter] += cController.m_UnObjectFoundExploring;
			m_UnObjectLost_Navigating[m_UnForagersCounter] += cController.m_UnGotLost;
			rObjectsFound_Navigating[m_unTimeCounter] = iGetMedian(m_UnObjectFound_Navigating,m_UnForagersCounter);
			rObjectsFound_Exploring[m_unTimeCounter] = iGetMedian(m_UnObjectFound_Exploring,m_UnForagersCounter);
			rObjectsLost_Navigating[m_unTimeCounter] = iGetMedian(m_UnObjectLost_Navigating,m_UnForagersCounter);

			if(cController.bSucccesFlag){

				/* Retrieve data to draw positions where objects were transferred */
				if(!((cSourcePosition.GetX() + 0.41) > cController.cSourcePosition.GetX() &&
						(cSourcePosition.GetX() - 0.41) < cController.cSourcePosition.GetX() &&
						(cSourcePosition.GetY() + 0.41) > cController.cSourcePosition.GetY() &&
						(cSourcePosition.GetY() - 0.41) < cController.cSourcePosition.GetY())){
					m_unCounterTransPos++;
					cTransferPosition[m_unCounterTransPos].SetX(cController.cSourcePosition.GetX());
					cTransferPosition[m_unCounterTransPos].SetY(cController.cSourcePosition.GetY());

					/* Uncomment only to show the positions where the object was transferred on the UI
					 * otherwise leave commented to decrease computation time  */
					// m_pcFloor->SetChanged();
				}
				m_cTransferPoints 	<< GetSpace().GetSimulationClock() << ","	
								<< cController.cSourcePosition 	<< ","
								<< cController.m_UnRobotId		<< ","
								<< cController.rTravellingDistance << ","
								<< cController.rSuccessRate << std::endl;
			}
		}

		/* Sample actual conditions of the swarm every 10 minutes */
		if(GetSpace().GetSimulationClock() - 6000 * m_unTimeCounter >= 6000){
			if(m_UnRobotId == 'r'){
				CFootBotForager& cController =
						dynamic_cast<CFootBotForager&>(cFootBot.GetControllableEntity().GetController());
				cController.cStartingPosition = cFootBot.GetEmbodiedEntity().GetOriginAnchor().OffsetPosition;
			}
			m_unFoodTimeCounter[m_unTimeCounter] = m_UnPreyCounter;
			m_unSubtasksAverageCounter[m_unTimeCounter] = m_unHoopCounterAverage;
			m_unTimeIntervals[m_unTimeCounter] = GetSpace().GetSimulationClock();
		
			/* Output stuff to file */
			m_cOutput 	<< m_unTimeIntervals[m_unTimeCounter] << ","
						<< m_unFoodTimeCounter[m_unTimeCounter] << ","
						<< m_unSubtasksAverageCounter[m_unTimeCounter] << ","
						<< rDistanceAverageCounter[m_unTimeCounter] << ","
						<< m_UnSearchTimer[m_unTimeCounter] << ","
						<< m_UnHandleObjectsTimer[m_unTimeCounter] << ","
						<< m_UnNavigateTimer[m_unTimeCounter] << ","
						<< rGetLost[m_unTimeCounter] << ","
						<< std::endl;
			m_cParamsForModel 	<< rObjectsFound_Navigating[m_unTimeCounter] << ","
								<< rObjectsFound_Exploring[m_unTimeCounter] << ","
								<< rObjectsLost_Navigating[m_unTimeCounter] << ","
								<< std::endl;
			m_unTimeCounter++;
		}
	}
}

/****************************************/
/****************************************/

/* Function used to calculate medians */
UInt32 CForagingLoopFunctions::iGetMedian(UInt32 iNumbersGroup[], UInt32 iSize){
	int low, high ;
	int median;
	int middle, ll, hh;

	low = 0 ; high = iSize-1 ; median = (low + high) / 2;
	for (;;) {
		if (high <= low) /* One element only */
			return iNumbersGroup[median] ;

		if (high == low + 1) {  /* Two elements only */
			if (iNumbersGroup[low] > iNumbersGroup[high])
				ELEM_SWAP(iNumbersGroup[low], iNumbersGroup[high]) ;
			return iNumbersGroup[median] ;
		}

		/* Find median of low, middle and high items; swap into position low */
		middle = (low + high) / 2;
		if (iNumbersGroup[middle] > iNumbersGroup[high])    
			ELEM_SWAP(iNumbersGroup[middle], iNumbersGroup[high]);

		if (iNumbersGroup[low] > iNumbersGroup[high])
			ELEM_SWAP(iNumbersGroup[low], iNumbersGroup[high]);

		if (iNumbersGroup[middle] > iNumbersGroup[low])
			ELEM_SWAP(iNumbersGroup[middle], iNumbersGroup[low]) ;

		/* Swap low item (now in position middle) into position (low+1) */
		ELEM_SWAP(iNumbersGroup[middle], iNumbersGroup[low+1]) ;

		/* Nibble from each end towards middle, swapping items when stuck */
		ll = low + 1;
		hh = high;
		for (;;) {
			do ll++; while (iNumbersGroup[low] > iNumbersGroup[ll]) ;
			do hh--; while (iNumbersGroup[hh]  > iNumbersGroup[low]) ;

			if (hh < ll)
				break;

			ELEM_SWAP(iNumbersGroup[ll], iNumbersGroup[hh]) ;
		}

		/* Swap middle item (in position low) back into correct position */
		ELEM_SWAP(iNumbersGroup[low], iNumbersGroup[hh]) ;

		/* Re-set active partition */
		if (hh <= median)
			low = ll;
			if (hh >= median)
			high = hh - 1;
	}
	return 0;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "task_partitioning_loop_functions")
