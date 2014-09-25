/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */


#include "EnergyMedea/include/EnergyMedeaAgentObserver.h"
#include "World/World.h"
#include "Utilities/Misc.h"
#include "RoboroboMain/roborobo.h"
#include "EnergyMedea/include/EnergyMedeaController.h"
#include <cmath>
#include "EnergyMedea/include/EnergyMedeaWorldObserver.h"
#include <string>


EnergyMedeaAgentObserver::EnergyMedeaAgentObserver( RobotWorldModel *wm )
{
	_wm = (EnergyMedeaAgentWorldModel*)wm;

}

EnergyMedeaAgentObserver::~EnergyMedeaAgentObserver()
{
	// nothing to do.
}

void EnergyMedeaAgentObserver::reset()
{
	// nothing to do.
}

void EnergyMedeaAgentObserver::step()
{

	// * send callback messages to objects touched or walked upon.

	// through distance sensors
	for( int i = 0 ; i < _wm->_cameraSensorsNb; i++)
	{
		int targetIndex = _wm->getObjectIdFromCameraSensor(i);

		if ( PhysicalObject::isInstanceOf(targetIndex) )   // sensor ray bumped into a physical object
		{
			targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
			//std::cout << "[DEBUG] Robot #" << _wm->getId() << " touched " << targetIndex << "\n";
			gPhysicalObjects[targetIndex]->isTouched(_wm->getId());
		}
	}

	// through floor sensor
	int targetIndex = _wm->getGroundSensorValue();
	if ( PhysicalObject::isInstanceOf(targetIndex) ) // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
	{
		targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
		//std::cout << "[DEBUG] #" << _wm->getId() << " walked upon " << targetIndex << "\n";
		gPhysicalObjects[targetIndex]->isWalked(_wm->getId());
		_wm->increaseEnergyHarvested();
	}
}
