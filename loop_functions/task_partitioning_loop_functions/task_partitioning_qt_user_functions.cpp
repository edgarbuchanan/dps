/*
 * AUTHOR: Edgar Buchanan <edgar.buchanan@york.ac.uk>
 *
 * Display the position where the robot thinks is the robot source.
 * Also shows the neighbourhood where the robots will be exploring.
 * Lastly, it is possible to see the range of vision of the robot
 *
 * This code is meant to be used with the configuration file:
 *    experiments/taskPartitioning.argos
 */

/* Header files */
#include "task_partitioning_qt_user_functions.h"
#include "task_partitioning_loop_functions.h"
#include <controllers/footbot_task_partitioning/footbot_forager.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

using namespace argos;

/****************************************/
/****************************************/

CForagingQTUserFunctions::CForagingQTUserFunctions() {
   RegisterUserFunction<CForagingQTUserFunctions,CFootBotEntity>(&CForagingQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CForagingQTUserFunctions::Draw(CFootBotEntity& c_entity) {
	/* Get ID */
	char m_UnRobotId = c_entity.GetId().at(2);
	/* For all the robots foraging*/
	if(m_UnRobotId == 'r'){
		/* Retrieve the position of the source and actual position*/
		CFootBotForager& cController = dynamic_cast<CFootBotForager&>(c_entity.GetControllableEntity().GetController());
		SRobots[cController.m_UnRobotId].cSourcePosition = cController.cSourcePosition;
		SRobots[cController.m_UnRobotId].bExploring = cController.IsExploring();
		SRobots[cController.m_UnRobotId].bReturnSource = cController.IsReturningToSource();
		SRobots[cController.m_UnRobotId].bReturnNest = cController.IsReturningToNest();
		SRobots[cController.m_UnRobotId].bNeighbourhoodExploration = cController.IsExploringNeighbourhood();
		SRobots[cController.m_UnRobotId].m_unId = cController.m_UnRobotId;
		SRobots[cController.m_UnRobotId].cEstimateActualPosition = cController.cEstimateActualPosition;
		SRobots[cController.m_UnRobotId].cEstimateIdealPosition = cController.cEstimateIdealPosition;
		SRobots[cController.m_UnRobotId].bSuccessFlag = cController.bSucccesFlag;
		if(m_unRobotsNumber < cController.m_UnRobotId) m_unRobotsNumber = cController.m_UnRobotId;
		/* Display ID on top of the robots */
		std::stringstream ss;
		ss << SRobots[cController.m_UnRobotId].m_unId;
		std::string sId = ss.str();
		DrawText(CVector3(0.0f,0.0f,0.6f),sId, CColor::BLACK);
		/* Uncomment to show range of vision */
//		CQuaternion cCircleOrientation;
//		DrawCircle(CVector3(0.0f,0.0f,0.005f),
//							cCircleOrientation, 0.5f, CColor::GRAY90, false, 20);
	}
	/* Replace food robots with cylinders */
	if(m_UnRobotId == 'o'){
		CQuaternion cCircleOrientation;
		DrawCylinder(CVector3(0.0f,0.0f,0.0f), cCircleOrientation, 0.09, 0.30f, CColor::GRAY50);
	}
}

/****************************************/
/****************************************/

void CForagingQTUserFunctions::DrawInWorld(){
	CQuaternion cCircleOrientation;
	for(int i = 1; i<= m_unRobotsNumber; i++){
		/* Change integer to string */
		std::stringstream ss;
		ss << SRobots[i].m_unId;
		std::string sId = ss.str();
		/* If the robot is returning to the nest or to source draw the position of the target */
		if(SRobots[i].bReturnNest || SRobots[i].bReturnSource){
			DrawCircle(CVector3(SRobots[i].cSourcePosition[0], SRobots[i].cSourcePosition[1], 0.02f),
					cCircleOrientation, 0.05f, CColor::RED);
			DrawText(CVector3(0.0f,0.0f,0.1f),sId, CColor::BLACK);
			DrawCircle(CVector3(-SRobots[i].cSourcePosition[0], -SRobots[i].cSourcePosition[1], 0.0f),
										cCircleOrientation, 0.0001f, CColor::WHITE);	/* Reset coordinates */
		}
		/* If the robot is exploring the neighbourhood draw the neighbourhood*/
		if(SRobots[i].bNeighbourhoodExploration){
			/* Uncomment to show neighbourhood area for each robot  */
//			DrawCircle(CVector3(SRobots[i].cSourcePosition[0], SRobots[i].cSourcePosition[1], 0.02f),
//					cCircleOrientation, 0.5f, CColor::CYAN, false, 20);
			DrawCircle(CVector3(SRobots[i].cSourcePosition[0], SRobots[i].cSourcePosition[1], 0.02f),
								cCircleOrientation, 0.05f, CColor::RED);
			DrawText(CVector3(0.0f,0.0f,0.1f),sId, CColor::BLACK);
			DrawCircle(CVector3(-SRobots[i].cSourcePosition[0], -SRobots[i].cSourcePosition[1], 0.0f),
										cCircleOrientation, 0.0001f, CColor::WHITE);	/* Reset coordinates */
		}
	}
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CForagingQTUserFunctions, "task_partitioning_qt_user_functions")

