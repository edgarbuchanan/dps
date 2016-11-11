/*
 * AUTHOR: Edgar Buchanan <edgar.buchanan@york.ac.uk>
 *
 * Task partitioning controller for the foot-bot platform.
 *
 * This controller used the foraging controller in the
 * ARGoS examples as a template.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/taskPartitioning.argos
 */

#ifndef FOOTBOT_FORAGING_H
#define FOOTBOT_FORAGING_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot motor ground sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/*Definition of the generic positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>
/* Definition of the camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotForager : public CCI_Controller {

public:


	/*
	 * The following variables are used as parameters for the
	 * diffusion algorithm. You can set their value in the <parameters>
	 * section of the XML configuration file, under the
	 * <controllers><footbot_foraging_controller><parameters><diffusion>
	 * section.
	 */
	struct SDiffusionParams {

		/*
		 * Maximum tolerance for the proximity reading between
		 * the robot and the closest obstacle.
		 * The proximity reading is 0 when nothing is detected
		 * and grows exponentially to 1 when the obstacle is
		 * touching the robot.
		 */
		Real Delta;

		/* Angle tolerance range to go straight. */
		CRange<CRadians> GoStraightAngleRange;

		/* Constructor */
		SDiffusionParams();

		/* Parses the XML section for diffusion */
		void Init(TConfigurationNode& t_tree);
	};

	/*
	 * The following variables are used as parameters for
	 * turning during navigation. You can set their value
	 * in the <parameters> section of the XML configuration
	 * file, under the
	 * <controllers><footbot_foraging_controller><parameters><wheel_turning>
	 * section.
	 */
	struct SWheelTurningParams {

		/*
		 * The turning mechanism.
		 * The robot can be in three different turning states.
		 */
		enum ETurningMechanism {
			NO_TURN = 0,	// go straight
			SOFT_TURN,	// both wheels are turning forwards, but at different speeds
			HARD_TURN,	// wheels are turning with opposite speeds
			DIRECTION_TURN	//change of direction
		} TurningMechanism;

		/*
		 * Angular thresholds to change turning state.
		 */
		CRadians HardTurnOnAngleThreshold;
		CRadians SoftTurnOnAngleThreshold;
		CRadians NoTurnAngleThreshold;
		/* Maximum wheel speed */
		Real MaxSpeed;

		void Init(TConfigurationNode& t_tree);
	};

	/*
	 * The following variables are used as parameters for the
	 * diffusion algorithm. You can set their value in the <parameters>
	 * section of the XML configuration file, under the
	 * <controllers><footbot_forager_controller><parameters><diffusion>
	 * section.
	 */
	struct SErrorParams {

		Real AccumulativeError;
		UInt32 m_unPartitionType;
		Real rInitialTravellingDistance;
		Real rTravellingGain;
		Real rTravellingDifferential;
		Real rCorrectionRate;
		UInt32 m_unItemTransfTime;

		/* Parses the XML section for diffusion */
		void Init(TConfigurationNode& t_tree);
	};

	/*
	 * Contains all the state information about the controller.
	 */
	struct SStateData {

		/* The six possible states in which the controller can be */
		enum EState {
			STATE_EXPLORING,
			STATE_GO_TO_NEST,
			STATE_GO_TO_SOURCE,
			STATE_NEIGHBORHOOD_EXPLORATION,
			STATE_WAIT_FOR_TRANSFER
		} State;

		/* Used as a range for uniform number generation */
		CRange<Real> ProbRange;
		CRange<Real> ProbRangeDegresTurn;

		/* True when the robot is in the nest */
		bool InNest;
		SStateData();
		void Init(TConfigurationNode& t_node);
		void Reset();
	};

public:

	/* Class constructor. */
	CFootBotForager();
	/* Class destructor. */
	virtual ~CFootBotForager() {}

	/*
	 * This function initializes the controller.
	 * The 't_node' variable points to the <parameters> section in the XML
	 * file in the <controllers><footbot_forager_controller> section.
	 */
	virtual void Init(TConfigurationNode& t_node);

	/*
	 * This function is called once every time step.
	 * The length of the time step is set in the XML file.
	 */
	virtual void ControlStep();

	/*
	 * This function resets the controller to its state right after the
	 * Init().
	 * It is called when you press the reset button in the GUI.
	 */
	virtual void Reset();

	/*
	 * Called to cleanup what done by Init() when the experiment finishes.
	 * In this example controller there is no need for clean anything up,
	 * so the function could have been omitted. It's here just for
	 * completeness.
	 */
	virtual void Destroy() {}

	/*
	 * Returns true if the robot is currently exploring.
	 */
	inline bool IsExploring() const {
		return m_sStateData.State == SStateData::STATE_EXPLORING;
	}

	/*
	 * Returns true if the robot is currently returning to the nest.
	 */
	inline bool IsReturningToNest() const {
		return m_sStateData.State == SStateData::STATE_GO_TO_NEST;
	}
	/*
	 * Returns true if the robot is currently returning to the food source.
	 */
	inline bool IsReturningToSource() const {
		return m_sStateData.State == SStateData::STATE_GO_TO_SOURCE;
	}
	/*
	 * Returns true if the robot is currently returning to the nest.
	 */
	inline bool IsExploringNeighbourhood() const {
		return m_sStateData.State == SStateData::STATE_NEIGHBORHOOD_EXPLORATION;
	}

	inline bool IsWaiting() const {
		return m_sStateData.State == SStateData::STATE_WAIT_FOR_TRANSFER;
	  }
	/*
	 * Returns true if the robot is currently returning to the nest.
	 */
	inline CVector3 SourceArea() const {
		return cSourcePosition;
	}

	bool bDirectionFlag;
	bool bChangeDirectionFlag;

	UInt32 m_unMissedPreyCounter;

	/* Timers used to change state */
	UInt32 m_unNighbourhoodExplorationTimer;
	UInt32 m_unWaitingTransferTimer;
	UInt32 m_unGoToSourceTimer;

	UInt32 m_unHandoverCounter;
	Real rTotalDistance, rActualDistance;
	CDegrees cActualAngle;
	CDegrees cTargetAngle;
	Real rLowerRegionLimit;
	Real rTravellingDistance;
	bool bFoodTransfered;	// This flag is used to show if an item was transferred successfully
	bool bOutOfNeighbourhoodRegion;	// This flag indicates whether the robot is out of the neighbourhood
	bool bHandoverFlag;
	bool bObstacleAvoidance;

	/* Variables used when a robot finds an item */
	bool bFoodFound;
	bool bFoodAttached;
	Real rActualDistanceFromPrey;
	Real rActualAngleFromPrey;
	Real rPreviousAngleFromPrey;

	UInt32 m_UnRobotId;
	UInt32 m_unNeighbourhoodExplorationConstant;	
	CVector3 cNestPosition;
	CVector3 cSourcePosition;
	CVector3 cStartingPosition;

	/* Variables used to calculate the drift of the estimated food position */
	Real rNoiseError;
	Real rAccumulativeError[2];
	Real rAccumulativeAngle;
	CVector3 cEstimateActualPosition;
	CVector3 cEstimateIdealPosition;
	Real rReferenceAngle;
	Real rGoToNestTimer;
	Real rGoToSourceTimer;

	/* Variables to measure amount of time spent in each task */
	UInt64 m_UnSearchTimer;
	UInt64 m_UnHandleObjectsTimer;
	UInt64 m_UnNavigateTimer;

	/* Variables used to retrieve data to calculate item not found rate */
	UInt32 m_UnGotLost;
	UInt32 m_UnTargetFound;
	UInt32 m_UnObjectFoundExploring;

	Real fSpeed1, fSpeed2;

	Real rSuccessRate; // Variable used for exponential mechanism

	bool bSucccesFlag;

	Real rPreviousDistanceFromSource;
private:

	/*
	 * Updates the state information.
	 * In practice, it sets the SStateData::InNest flag.
	 * Future, more complex implementations should add their
	 * state update code here.
	 */
	void UpdateState();

	/*
	 * Analyses the neighbourhood
	 * and sets flag if food is close to robot.
	 */
	void Camera();

	/*
	 * Modifies threshold value according to success of failure in search.
	 */
	void PartitionLength(bool bSearchResult);

	/*
	 * Calculates the vector to the light. Used to perform
	 * phototaxis and antiphototaxis.
	 */
	CVector2 CalculateVectorToLight();

	/*
	 * Calculates the diffusion vector. If there is a close obstacle,
	 * it points away from it; it there is none, it points forwards.
	 * The b_collision parameter is used to return true or false whether
	 * a collision avoidance just happened or not. It is necessary for the
	 * collision rule.
	 */
	CVector2 DiffusionVector(bool& b_collision);

	/*
	 * Gets a direction vector as input and transforms it into wheel
	 * actuation.
	 */
	void SetWheelSpeedsFromVector(const CVector2& c_heading);

	/*
	 * Executes the exploring state.
	 */
	void Explore();

	/*
	 * Executes the exploring state.
	 */
	void ExploreRegion();


	/*
	 * Executes the go to nest state.
	 */
	void GoToNest();
	/*
	 * Executes the go to source state.
	 */
	void GoToSource();

	/*
	 * Executes the neighborhood exploration state.
	 */
	void NeighborhoodExploration();
	
	/*
	 * Executes the wat for transfer state
	 */
	void WaitForTransfer();

private:

	/* Pointer to the differential steering actuator */
	CCI_DifferentialSteeringActuator* m_pcWheels;
	/* Pointer to the LEDs actuator */
	CCI_LEDsActuator* m_pcLEDs;
	/* Pointer to the range and bearing actuator */
	CCI_RangeAndBearingActuator*  m_pcRABA;
	/* Pointer to the range and bearing sensor */
	CCI_RangeAndBearingSensor* m_pcRABS;
	/* Pointer to the positioning sensors */
	CCI_PositioningSensor*  m_pcPositioning;
	/* Pointer to the foot-bot proximity sensor */
	CCI_FootBotProximitySensor* m_pcProximity;
	/* Pointer to the omnidirectional camera sensor */
	CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
	/* Pointer to the foot-bot light sensor */
	CCI_FootBotLightSensor* m_pcLight;
	/* Pointer to the foot-bot motor ground sensor */
	CCI_FootBotMotorGroundSensor* m_pcGround;
	/* The random number generator */
	CRandom::CRNG* m_pcRNG;
	/* The controller state information */
	SStateData m_sStateData;
	/* The turning parameters */
	SWheelTurningParams m_sWheelTurningParams;
	/* The diffusion parameters */
	SDiffusionParams m_sDiffusionParams;
	/* The error parameters */
	SErrorParams m_sErrorParams;
};

#endif
