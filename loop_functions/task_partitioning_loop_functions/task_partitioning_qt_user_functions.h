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

#ifndef FORAGING_QT_USER_FUNCTIONS_H
#define FORAGING_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CForagingQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CForagingQTUserFunctions();

   virtual ~CForagingQTUserFunctions() {}

   void Draw(CFootBotEntity& c_entity);

	void DrawInWorld();

	struct SRobot{
		/* Draw according to each state */
		bool bExploring;
		bool bNeighbourhoodExploration;
		bool bReturnNest;
		bool bReturnSource;
		bool bSuccessFlag;

		/* Position and ID from the robot */
		UInt32 m_unId;
		CVector3 cSourcePosition;
		CVector3 cEstimateActualPosition;
		CVector3 cEstimateIdealPosition;
	} SRobots[99];
	UInt32 m_unRobotsNumber;
};

#endif
