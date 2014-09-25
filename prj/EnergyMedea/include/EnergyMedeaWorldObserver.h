/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */





#ifndef ENERGYMEDEAWORLDOBSERVER_H
#define ENERGYMEDEAWORLDOBSERVER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Observers/Observer.h"
#include "Observers/WorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "EnergyMedea/include/EnergyMedeaSharedData.h"
#include "EnergyMedea/include/EnergyMedeaSharedData.h"

//class World;

class EnergyMedeaWorldObserver : public WorldObserver
{
	private:
		void updateEnvironment();
		void updateMonitoring();

	protected:
		int _generationCount;
		int _lifeIterationCount;

	public:
		EnergyMedeaWorldObserver(World *world);
		~EnergyMedeaWorldObserver();

		void reset();
		void step();

		int getLifeIterationCount() { return _lifeIterationCount; }

};

#endif
