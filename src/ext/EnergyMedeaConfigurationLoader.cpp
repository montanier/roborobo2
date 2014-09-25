#if defined PRJ_ENERGYMEDEA || !defined MODULAR

#include "Config/EnergyMedeaConfigurationLoader.h"

#include "EnergyMedea/include/EnergyMedeaWorldObserver.h"
#include "EnergyMedea/include/EnergyMedeaAgentObserver.h"
#include "EnergyMedea/include/EnergyMedeaController.h"
#include "EnergyMedea/include/EnergyMedeaAgentWorldModel.h"

#include "WorldModels/RobotWorldModel.h"

EnergyMedeaConfigurationLoader::EnergyMedeaConfigurationLoader()
{
}

EnergyMedeaConfigurationLoader::~EnergyMedeaConfigurationLoader()
{
	//nothing to do
}

WorldObserver* EnergyMedeaConfigurationLoader::make_WorldObserver(World* wm)
{
	return new EnergyMedeaWorldObserver(wm);
}

RobotWorldModel* EnergyMedeaConfigurationLoader::make_RobotWorldModel()
{
	return new EnergyMedeaAgentWorldModel();
}

AgentObserver* EnergyMedeaConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new EnergyMedeaAgentObserver(wm);
}

Controller* EnergyMedeaConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new EnergyMedeaController(wm);
}

#endif
