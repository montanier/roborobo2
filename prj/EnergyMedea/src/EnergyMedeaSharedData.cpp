/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */



#include "EnergyMedea/include/EnergyMedeaSharedData.h"

double EnergyMedeaSharedData::gSigmaMin = 0.0;
double EnergyMedeaSharedData::gProbaMutation = 0.0;
double EnergyMedeaSharedData::gUpdateSigmaStep = 0.0;
double EnergyMedeaSharedData::gSigmaRef = 0.0; // reference value of sigma
double EnergyMedeaSharedData::gSigmaMax = 0.0; // maximal value of sigma
int EnergyMedeaSharedData::gEvaluationTime = 0; // how long a controller will be evaluated on a robot
double EnergyMedeaSharedData::gDeadTime = 0.0; // how long a robot will remain dead (ratio of evalluation time)

bool EnergyMedeaSharedData::gSynchronization = true;

bool EnergyMedeaSharedData::gEnergyRequestOutput = 1;
bool EnergyMedeaSharedData::gAltruismEvolved = true;
double EnergyMedeaSharedData::gSharing = 0.0;
int EnergyMedeaSharedData::gSetup = 1;
int EnergyMedeaSharedData::gCoopPartner = 1;

double EnergyMedeaSharedData::gMonitorPositions;

bool EnergyMedeaSharedData::gPropertiesLoaded = false; // global variable local to file -- TODO: move specific properties loader in dedicated WorldObserver

int EnergyMedeaSharedData::gNbHiddenLayers = 1;
int EnergyMedeaSharedData::gNbNeuronsPerHiddenLayer = 5;
int EnergyMedeaSharedData::gNeuronWeightRange = 800;

bool EnergyMedeaSharedData::gSnapshots = true; // take snapshots
int EnergyMedeaSharedData::gSnapshotsFrequency = 50; // every N generations

int EnergyMedeaSharedData::gControllerType = -1; // cf. header for description
