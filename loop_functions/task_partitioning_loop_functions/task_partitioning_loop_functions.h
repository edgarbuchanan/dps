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

#ifndef FORAGING_LOOP_FUNCTIONS_H
#define FORAGING_LOOP_FUNCTIONS_H

/* Header files */
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CForagingLoopFunctions : public CLoopFunctions {

public:

   CForagingLoopFunctions();
   virtual ~CForagingLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();


private:

   	/* Functions used to calculate medians */
	UInt32 iGetMedian(UInt32 iNumbersGroup[], UInt32 iSize);

	CFloorEntity* m_pcFloor;
	CRandom::CRNG* m_pcRNG;

	UInt32 m_UnPreyCounter;
	bool bSimulationFinished;
	UInt32 m_UnHopCounter[10];
	UInt32 m_unTimeCounter;
	UInt32 m_unFoodTimeCounter[200];
	Real m_unSubtasksAverageCounter[200];
	Real m_unHoopCounterAverage;
	UInt32 m_unTimeIntervals[200];	// Timestamps
	Real rDistanceAverageCounter[200];

	/* Variables to measure amount of time spent in each task */
	UInt64 m_UnSearchTimer[200];
	UInt64 m_UnHandleObjectsTimer[200];
	UInt64 m_UnNavigateTimer[200];

	/* Variables used to retrieve data to calculate item not found rate */
	UInt32 rObjectsFound_Navigating[200];
	UInt32 rObjectsFound_Exploring[200];
	UInt32 rObjectsLost_Navigating[200];
	Real rGetLost[200]; // Item not found rate

	Real rNestSourceDistance;

	//Outputs
	std::string m_strOutput;
	std::ofstream m_cOutput;
	std::string m_strTransferPoints;
	std::ofstream m_cTransferPoints;
	std::string m_strParamsForModel;
	std::ofstream m_cParamsForModel;

	CVector3 cSourcePosition;

	/* Variables used to draw positions where objects were transferred */
	CVector2 cTransferPosition[1000];
	UInt32 m_unCounterTransPos;
};

#endif
