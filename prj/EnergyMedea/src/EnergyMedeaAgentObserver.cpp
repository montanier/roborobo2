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

	indexNeighbours = 0;
	for (int i = 0 ; i < 10 ; i++)
	{
		std::vector<int> tmp;
		neighboursWindow.push_back(tmp);
	}
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
	if (_wm->getLifeStatus() == EnergyMedeaAgentWorldModel::ACTIVE)
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

		if (EnergyMedeaSharedData::gSetup == 2)
		{
			locateNeighbours();
		}

		// through floor sensor
		int targetIndex = _wm->getGroundSensorValue();
		if ( PhysicalObject::isInstanceOf(targetIndex) ) // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
		{
			targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
			//std::cout << "[DEBUG] #" << _wm->getId() << " walked upon " << targetIndex << "\n";
			if ( (*gPhysicalObjects[targetIndex]).getType() == 1 ) //type 1 object: energy points
			{
				gPhysicalObjects[targetIndex]->isWalked(_wm->getId());
				_wm->increaseEnergyHarvested();
				gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << " te " << std::endl;

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
						if (gWorld->getIterations() > 10) //build the neighbour window before starting
						{
							sharingActionNeighbours();
						}
					}
				}
				else
				{
					//selfish action, keep all, nothing to do
				}
			}
		}
	}
}

void EnergyMedeaAgentObserver::sharingActionKinship()
{
	unsigned long ownParent = _wm->getParent();
	std::vector<int> listSisters;
	std::vector<int> listNonSisters;
	std::vector<int> listAll;
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
				listAll.push_back(i);
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

		//receivers are picked among the non sisters
		//functional but decided to not use it for now
		/*
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
		*/

		//receivers are picked among all genomes (which include sisters and non-sisters)
		while(receivers.size() < listSisters.size())
		{
			int candidate = std::rand() % listAll.size();
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
	std::vector<int> listNeighbours;
	for (unsigned int iteration = 0 ; iteration < neighboursWindow.size() ; iteration++ )
	{
		for (unsigned int robot = 0 ; robot < neighboursWindow[iteration].size() ; robot++)
		{
			listNeighbours.push_back(neighboursWindow[iteration][robot]);
		}
	}
	//remove duplicates from vector
	std::sort( listNeighbours.begin(), listNeighbours.end() );
	listNeighbours.erase( std::unique( listNeighbours.begin(), listNeighbours.end() ), listNeighbours.end() );

	std::vector<int> listNonNeighbours;
	std::vector<int> listAll;
	for ( int i = 0 ; i != gNumberOfRobots ; i++ )
	{
		if ( (dynamic_cast<EnergyMedeaAgentWorldModel*>(gWorld->getRobot(i)->getWorldModel()))->isAlive() == true )
		{
			bool aNeighbour = false;
			for (unsigned int robot = 0 ; robot < listNeighbours.size() ; robot++)
			{
				if (i == listNeighbours[robot])
				{
					aNeighbour = true;
					break;
				}
			}
			if (aNeighbour == false)
			{
				listNonNeighbours.push_back(i);
			}
			listAll.push_back(i);
		}
	}

	float energyPerReceiver = EnergyMedeaSharedData::gSacrifice / listNeighbours.size();
	//if EnergyMedeaSharedData::gCoopPartner is equal to 1 give to close
	//otherwise give to far away
	if (EnergyMedeaSharedData::gCoopPartner == 1)
	{
		for (std::vector<int>::iterator it = listNeighbours.begin(); it != listNeighbours.end(); it++)
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
		//receivers are picked among the none neighbours
		//functional but decided to not use it for now
		/*
		if ( listNonNeighbours.size() > listNeighbours.size() ) //if there is enough non-neighbours pick them randomly
		{
			while(receivers.size() < listNeighbours.size())
			{
				int candidate = std::rand() % listNonNeighbours.size();
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
		else //otherwise take all the non-neighbours, and change the energy to give
		{
			receivers.swap(listNonNeighbours);
			energyPerReceiver = EnergyMedeaSharedData::gSacrifice / listNonNeighbours.size();
		}
		*/

		//receivers are picked among all genomes (which include neighbours and non-neighbours)
		while(receivers.size() < listNeighbours.size())
		{
			int candidate = std::rand() % listAll.size();
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

		for (std::vector<int>::iterator it = receivers.begin(); it != receivers.end(); it++)
		{
			gWorld->getRobot(*it)->getWorldModel()->addEnergy(energyPerReceiver);
			_wm->substractEnergy(energyPerReceiver);
			if (_wm->getEnergyLevel() == 0) break; // break as soon as there is no more energy
			gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << " ge " << energyPerReceiver << "," << *it << std::endl;
		}
	}

}

void EnergyMedeaAgentObserver::locateNeighbours()
{
	std::vector<int> tmp;
	for( int i = 0 ; i < _wm->_cameraSensorsNb; i++)
	{
		int targetIndex = _wm->getObjectIdFromCameraSensor(i);

		if ( targetIndex >= gRobotIndexStartOffset )   // sensor ray bumped into a robot : communication is possible
		{
			targetIndex = targetIndex - gRobotIndexStartOffset; // convert image registering index into robot id.

			EnergyMedeaAgentWorldModel* currentAgentWM = dynamic_cast<EnergyMedeaAgentWorldModel*>(gWorld->getRobot(targetIndex)->getWorldModel());
			if (currentAgentWM->getLifeStatus() == EnergyMedeaAgentWorldModel::ACTIVE)
			{
				tmp.push_back(targetIndex);
			}
		}
	}
	neighboursWindow[indexNeighbours] = tmp;
	indexNeighbours = (indexNeighbours+1) % 10;
}
