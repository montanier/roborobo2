/*
 * MedeaConfigurationLoader.h
 */

#ifndef ENERGYMEDEACONFIGURATIONLOADER_H
#define ENERGYMEDEACONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"


class EnergyMedeaConfigurationLoader : public ConfigurationLoader
{
	private:

	public:
		EnergyMedeaConfigurationLoader();
		~EnergyMedeaConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};



#endif
