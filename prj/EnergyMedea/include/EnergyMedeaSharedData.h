/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */



#ifndef ENERGYMEDEASHAREDDATA_H
#define ENERGYMEDEASHAREDDATA_H

class EnergyMedeaSharedData {
	
	public: 
	
	// -----
	
	static double gSigmaMin; //! used with gDynamicSigma defined to true
	static double gUpdateSigmaStep; //!step used in the drecrease or increas of the value of sigma
	static double gSigmaRef; //! reference value of sigma
	static double gSigmaMax; //! maximal value of sigma
	static double gProbaMutation; //! probability of transmitting the current genome mutated with sigma ref
	static int gEvaluationTime; //! theoretical duration of a generation (ie. maximum time a controller will be evaluated on a robot)
	static int gIteration; //! used by every class to know what is the current iteration step of roborobo
	static bool gSynchronization; //!If set to false, a robot will restart its controller as soon as it has no more energy. If set to true, the robot without energy will wait and reload its controller at the same time as every other robots.
	static double gDeadTime; //use when synchronization is off. time during which the robot is dead (not listening, not moving, not broadcasting). the time is expressed in ratio of gEvaluationTime (between 0 and 1)

    static bool gEnergyRequestOutput; // does the robot can modulate its energy request (when being given some) ?
    static bool gAltruismEvolved; // do we use a gene under evolution to choose if the coop action is done ? or is it fixed at setup
    static double gSharing; // in case the cooperation is fixed (gAltruismEvolved == false) decides if the agent will cooperate or not: -0.5 no cooperation, 0.5 cooperation
		static int gSetup; // 1 for choice of coop partners based on kinship, 2 for choice of partner based on distance
		static int gCoopPartner; // 1 for choosing close coop partners, 2 for choosing far away coop partners
		static double gSacrifice; // 1 for choosing close coop partners, 2 for choosing far away coop partners
    
  	static double gMonitorPositions; //! used in WorldObserver. Compute and log all necessary information for monitoring position and orientation wrt. center.
    
	static bool gPropertiesLoaded;

    static int gNbHiddenLayers; // default: 1
    static int gNbNeuronsPerHiddenLayer; // default: 5
    static int gNeuronWeightRange; // default: 800.0 (ie. weights are in [-400,+400[

    static bool gSnapshots; // take snapshots
    static int gSnapshotsFrequency; // every N generations
    
    static int gControllerType; // controller type (0: MLP, 1: Perceptron, 2: Elman)
    
    // -----
    

    
};


#endif
