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

		if ( (*gPhysicalObjects[targetIndex]).getType() == 1 ) //type 1 object: energy points
		{
			//decide to share or not the energy
			if(_wm->getSharing() >= 0.0)
			{
				if (EnergyMedeaSharedData::gSetup == 1)
				{
					if (gWorld->getIterations() > EnergyMedeaSharedData::gEvaluationTime) //wait one generation before starting
					{
						sharingActionKinship();
					}
				}
				else if (EnergyMedeaSharedData::gSetup == 2)
				{
					sharingActionNeighbours();
				}
			}
			else
			{
				//selfish action, keep all, nothing to do
			}
		}
	}
}

void EnergyMedeaAgentObserver::sharingActionKinship()
{
	unsigned long ownParent = _wm->getParent();
	std::vector<int> listSisters;
	std::vector<int> listNonSisters;
	for ( int i = 0 ; i != gNumberOfRobots ; i++ )
	{
		if ( (dynamic_cast<EnergyMedeaAgentWorldModel*>(gWorld->getRobot(i)->getWorldModel()))->isAlive() == true )
		{
			if (i != _wm->getId())
			{
				unsigned long parent = (dynamic_cast<EnergyMedeaAgentWorldModel*>(gWorld->getRobot(i)->getWorldModel()))->getParent();
				if (parent == ownParent)
				{
					listSisters.push_back(i);
				}
				else
				{
					listNonSisters.push_back(i);
				}
			}
		}
	}

	float energyPerReceiver = EnergyMedeaSharedData::gSacrifice / listSisters.size();
	//if EnergyMedeaSharedData::gCoopPartner is equal to 1 give to close
	//otherwise give to far away
	if (EnergyMedeaSharedData::gCoopPartner == 1)
	{
		for (std::vector<int>::iterator it = listSisters.begin(); it != listSisters.end(); it++)
		{
			gWorld->getRobot(*it)->getWorldModel()->addEnergy(energyPerReceiver);
			_wm->substractEnergy(energyPerReceiver);
			if (_wm->getEnergyLevel() == 0) break; // break as soon as there is no more energy
			gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << " ge " << energyPerReceiver << "," << *it << std::endl;
		}
	}
	else
	{
		std::vector<int> receivers;
		if ( listNonSisters.size() > listSisters.size() ) //if there is enough non-sisters pick them randomly
		{
			while(receivers.size() < listSisters.size())
			{
				int candidate = std::rand() % listNonSisters.size();
				bool valid = true;
				for (unsigned int j = 0 ; j < receivers.size() ; j++ )
				{
					if (candidate == receivers[j])
					{
						valid = false;
					}
				}

				if (valid == true)
				{
					receivers.push_back(candidate);
				}
			}
		}
		else //otherwise take all the non-sisters, and change the energy to give
		{
			receivers.swap(listNonSisters);
			energyPerReceiver = EnergyMedeaSharedData::gSacrifice / listNonSisters.size();
		}

		for (std::vector<int>::iterator it = receivers.begin(); it != receivers.end(); it++)
		{
			gWorld->getRobot(*it)->getWorldModel()->addEnergy(energyPerReceiver);
			_wm->substractEnergy(energyPerReceiver);
			if (_wm->getEnergyLevel() == 0) break; // break as soon as there is no more energy
			gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << " ge " << energyPerReceiver << "," << *it << std::endl;
		}
	}

}

void EnergyMedeaAgentObserver::sharingActionNeighbours()
{
	//look at parameter EnergyMedeaSharedData::gCoopPartner to know if we share with close or far
}
