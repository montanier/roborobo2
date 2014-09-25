/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */


#ifndef ENERGYMEDEAAGENTOBSERVER_H
#define ENERGYMEDEAAGENTOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/AgentObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "EnergyMedea/include/EnergyMedeaSharedData.h"
#include "EnergyMedea/include/EnergyMedeaAgentWorldModel.h"

#include <iomanip>

class EnergyMedeaAgentObserver : public AgentObserver
{
	private:
		EnergyMedeaAgentWorldModel *_wm;
		
	public:
		EnergyMedeaAgentObserver(RobotWorldModel *wm);
		~EnergyMedeaAgentObserver();

		void reset();
		void step();

};

#endif

