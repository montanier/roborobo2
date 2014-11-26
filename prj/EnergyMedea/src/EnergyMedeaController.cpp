/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */

#include "EnergyMedea/include/EnergyMedeaController.h"
#include "EnergyMedea/include/EnergyMedeaWorldObserver.h"

#include "World/World.h"
#include "Utilities/Misc.h"
#include <math.h>
#include <string>

#include <neuralnetworks/MLP.h>
#include <neuralnetworks/Perceptron.h>
#include <neuralnetworks/Elman.h>

using namespace Neural;

EnergyMedeaController::EnergyMedeaController( RobotWorldModel *wm )
{
	_wm = (EnergyMedeaAgentWorldModel*)wm;

	nn = NULL;

	// evolutionary engine

	_minValue = -1.0;
	_maxValue = 1.0;

	_currentSigma = EnergyMedeaSharedData::gSigmaRef;

	resetRobot();

	// behaviour

	_iteration = 0;

	_birthdate = 0;

	if ( gEnergyLevel )
		_wm->setEnergyLevel(gEnergyInit);

	_wm->updateLandmarkSensor();

	_wm->setLifeStatus(EnergyMedeaAgentWorldModel::ACTIVE);
	_wm->setRobotLED_colorValues(255, 0, 0);

	//std::cout << "["<< _wm->getId() <<"]BREAKPOINT.0: " << _wm->_desiredTranslationalValue << " , " << _wm->_desiredRotationalVelocity << "\n";


}

EnergyMedeaController::~EnergyMedeaController()
{
	_parameters.clear();
	delete nn;
	nn = NULL;
}

void EnergyMedeaController::reset()
{
	_parameters.clear();
	_parameters = _genome;
}


void EnergyMedeaController::step()
{
	_iteration++;
	stepEvolution();
	if ( _wm->isAlive() )
	{
		stepBehaviour();
	}
	else
	{
		_wm->_desiredTranslationalValue = 0.0;
		_wm->_desiredRotationalVelocity = 0.0;
	}

}


// ################ ######################## ################
// ################ ######################## ################
// ################ BEHAVIOUR METHOD(S)      ################
// ################ ######################## ################
// ################ ######################## ################



void EnergyMedeaController::stepBehaviour()
{
	if (_wm->getLifeStatus() == EnergyMedeaAgentWorldModel::ACTIVE) 
	{
		if( _iteration == 1)
		{
			_startPos = Point2d(_wm->_xReal,_wm->_yReal);
			_cumulatedDistance = 0;
		}

		Point2d currentPos = Point2d (_wm->_xReal , _wm->_yReal);
		_cumulatedDistance += getEuclidianDistance(_startPos, currentPos);
	}

	// ---- update energy if needed
	if ( gEnergyLevel && _wm->getLifeStatus() == EnergyMedeaAgentWorldModel::ACTIVE )
	{
		_wm->substractEnergy(1);
		assert( _wm->getEnergyLevel() >= 0 );
		if ( _wm->getEnergyLevel() == 0 )
		{
			logEndGeneration();
			_wm->setLifeStatus(EnergyMedeaAgentWorldModel::DEAD);
			_iteration = 0;
			_genomesList.clear();
			_genome.clear();

			gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << "::" << _birthdate << " dead" << std::endl ;
			return;
		}
	}

	// pre-compute closest landmark information (if needed)
	if ( gLandmarks.size() > 0 )
	{
		_wm->updateLandmarkSensor();
	}

	// ---- Build inputs ----

	std::vector<double>* inputs = new std::vector<double>(_nbInputs);
	int inputToUse = 0;

	// distance sensors
	for(int i  = 0; i < _wm->_cameraSensorsNb; i++)
	{
		(*inputs)[inputToUse] = _wm->getDistanceValueFromCameraSensor(i) / _wm->getCameraSensorMaximumDistanceValue(i);
		inputToUse++;

		if ( gExtendedSensoryInputs ) 
		{
			int objectId = _wm->getObjectIdFromCameraSensor(i);

			// input: physical object? which type?
			if ( PhysicalObject::isInstanceOf(objectId) )
			{
				if ( (*gPhysicalObjects[objectId - gPhysicalObjectIndexStartOffset]).getType() == 1 ) //type 1 object: energy points
					(*inputs)[inputToUse] = 1; // match
				else
					(*inputs)[inputToUse] = 0;
			}
			else
			{
				// not a physical object. But: should still fill in the inputs (with zeroes)
				(*inputs)[inputToUse] = 0;
			}
			inputToUse++;

			// relative orientation? (ie. angle difference wrt. current agent)
			/*
			double srcOrientation = _wm->_agentAbsoluteOrientation;
			double tgtOrientation = gWorld->getRobot(objectId-gRobotIndexStartOffset)->getWorldModel()->_agentAbsoluteOrientation;
			double delta_orientation = - ( srcOrientation - tgtOrientation );
			if ( delta_orientation >= 180.0 )
				delta_orientation = - ( 360.0 - delta_orientation );
			else
				if ( delta_orientation <= -180.0 )
					delta_orientation = - ( - 360.0 - delta_orientation );
			(*inputs)[inputToUse] = delta_orientation/180.0;
			inputToUse++;
			*/

			/* 
			// ----- DEBUG::SANDBOX
			std::cout << "src.orientation: " <<  srcOrientation
			<< "° ; tgt.orientation: " << tgtOrientation
			<< "° ; delta orientation: " << delta_orientation
			<< " <==> " << delta_orientation/180.0 << "°"
			<< std::endl;
			// ----- DEBUG::SANDBOX. 
			 */
		}
	}

	// floor sensor
	(*inputs)[inputToUse++] = (double)_wm->getGroundSensor_redValue()/255.0;
	(*inputs)[inputToUse++] = (double)_wm->getGroundSensor_greenValue()/255.0;
	(*inputs)[inputToUse++] = (double)_wm->getGroundSensor_blueValue()/255.0;

	// closest landmark (if exists)
	if ( gLandmarks.size() > 0 )
	{
		(*inputs)[inputToUse++] = _wm->getLandmarkDirectionAngleValue();
		(*inputs)[inputToUse++] = _wm->getLandmarkDistanceValue();
	}

	// energy level
	if ( gEnergyLevel )
	{
		(*inputs)[inputToUse++] = _wm->getEnergyLevel() / gEnergyMax;
	}

	// ---- compute and read out ----

	nn->setWeigths(_parameters); // create NN

	nn->setInputs(*inputs);

	nn->step();

	std::vector<double> outputs = nn->readOut();
	delete (inputs);

	_wm->_desiredTranslationalValue = outputs[0];
	_wm->_desiredRotationalVelocity = outputs[1];
	_wm->setEnergyRequestValue(1.0);

	// normalize to motor interval values
	_wm->_desiredTranslationalValue = _wm->_desiredTranslationalValue * gMaxTranslationalSpeed;
	_wm->_desiredRotationalVelocity = _wm->_desiredRotationalVelocity * gMaxRotationalSpeed;
}

void EnergyMedeaController::logEndGeneration()
{
	_endPos = Point2d (_wm->_xReal , _wm->_yReal);
	double distance = getEuclidianDistance(_startPos, _endPos);
	gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << "::" << _birthdate << " dd " << distance << std::endl ;
	gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << "::" << _birthdate << " pe " << _endPos.x << "," << _endPos.y << std::endl ;
	gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << "::" << _birthdate << " dacc " << _cumulatedDistance << std::endl ;
}

void EnergyMedeaController::createNN()
{
	if ( nn != NULL ) // useless: delete will anyway check if nn is NULL or not.
		delete nn;

	switch ( EnergyMedeaSharedData::gControllerType )
	{
		case 0:
			{
				// MLP
				nn = new MLP(_parameters, _nbInputs, _nbOutputs, *(_nbNeuronsPerHiddenLayer));
				break;
			}
		case 1:
			{
				// PERCEPTRON
				nn = new Perceptron(_parameters, _nbInputs, _nbOutputs);
				break;
			}
		case 2:
			{
				// ELMAN
				nn = new Elman(_parameters, _nbInputs, _nbOutputs, *(_nbNeuronsPerHiddenLayer));
				break;
			}
		default: // default: no controller
			std::cerr << "[ERROR] gController type unknown (value: " << EnergyMedeaSharedData::gControllerType << ").\n";
			exit(-1);
	};
}


unsigned int EnergyMedeaController::computeRequiredNumberOfWeights()
{
	unsigned int res = nn->getRequiredNumberOfWeights();
	return res;
}

// ################ ######################## ################
// ################ ######################## ################
// ################ EVOLUTION ENGINE METHODS ################
// ################ ######################## ################
// ################ ######################## ################

void EnergyMedeaController::stepEvolution()
{
	// * broadcasting genome : robot broadcasts its genome to all neighbors (contact-based wrt proximity sensors)
	if  ( gRadioNetwork )
	{
		if ( _wm->getLifeStatus() == EnergyMedeaAgentWorldModel::ACTIVE )  	// only if agent is active (ie. not just revived) and deltaE>0.
		{
			broadcastGenome();
		}
	}

	if (EnergyMedeaSharedData::gSynchronization == true)
	{
		// * lifetime ended: replace genome (if possible)
		if( dynamic_cast<EnergyMedeaWorldObserver*>(gWorld->getWorldObserver())->getLifeIterationCount() >= EnergyMedeaSharedData::gEvaluationTime-1 )
		{
			if ( _wm->isAlive() || gEnergyRefill )
			{
				bool loadDone = loadNewGenome();

				if (loadDone == true )
				{
					_wm->setLifeStatus(EnergyMedeaAgentWorldModel::ACTIVE);
					if ( _wm->getEnergyLevel() == 0 )
						_wm->setEnergyLevel(gEnergyInit);
					_wm->setRobotLED_colorValues(255, 0, 0);

					// Logging
					gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << "::" << _birthdate << " newGenome " ;
					for(unsigned int i=0; i<_genome.size(); i++)
					{
						gLogFile << std::fixed << std::showpoint << _genome[i] << " ";
					}
					gLogFile << std::endl;
				}
				else
				{
					gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << "::" << _birthdate << " noGenome" << std::endl;

					resetRobot(); // destroy then create a new NN
					_wm->setLifeStatus(EnergyMedeaAgentWorldModel::DEAD);
					_wm->setRobotLED_colorValues(0, 0, 255);

				}
			}
		}
	}
	else // synchronization is false
	{
		if( _iteration >= EnergyMedeaSharedData::gEvaluationTime )
		{
			if (_wm->getLifeStatus() == EnergyMedeaAgentWorldModel::ACTIVE) 
			{
				logEndGeneration();
			}

			if ( (_wm->getLifeStatus() == EnergyMedeaAgentWorldModel::ACTIVE) || (_wm->getLifeStatus() == EnergyMedeaAgentWorldModel::LISTEN))
			{
				bool loadDone = loadNewGenome();

				if (loadDone == true )
				{
					_wm->setLifeStatus(EnergyMedeaAgentWorldModel::ACTIVE);

					if ( _wm->getEnergyLevel() <= 0 )
						_wm->setEnergyLevel(gEnergyInit);
					_wm->setRobotLED_colorValues(255, 0, 0);

					gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << "::" << _birthdate << " newGenome " ;
					for(unsigned int i=0; i<_genome.size(); i++)
					{
						gLogFile << std::fixed << std::showpoint << _genome[i] << " ";
					}
					gLogFile << std::endl;
				}
				else
				{
					_wm->setLifeStatus(EnergyMedeaAgentWorldModel::LISTEN);
					gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << "::" << _birthdate << " listen" << std::endl ;
				}
				_iteration = 0;
			}
		}

		if ( _iteration >= (EnergyMedeaSharedData::gEvaluationTime*EnergyMedeaSharedData::gDeadTime) && _wm->getLifeStatus() == EnergyMedeaAgentWorldModel::DEAD )
		{
			_iteration = 0;
			gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << "::" << _birthdate << " listen" << std::endl ;
			_wm->setLifeStatus(EnergyMedeaAgentWorldModel::LISTEN);
			_genomesList.clear();
		}
	}

	if ( getNewGenomeStatus() ) // check for new NN parameters
	{
		reset();
		setNewGenomeStatus(false);
	}

}

bool EnergyMedeaController::loadNewGenome()
{

	// note: at this point, agent got energy, whether because it was revived or because of remaining energy.

	if (_genomesList.size() > 0)
	{
		// case: 1+ genome(s) imported, random pick.
		selectRandomGenome();
		if (EnergyMedeaSharedData::gAltruismEvolved == true)
		{
			_wm->setSharing(_currentGenome[_currentGenome.size()-1]);
		}
		else
		{
			_wm->setSharing(EnergyMedeaSharedData::gSharing);
		}
		return true;
	}
	return false;
}

void EnergyMedeaController::selectRandomGenome()
{
	if(_genomesList.size() != 0)
	{
		gLogFile << gWorld->getIterations() <<  " : " << _wm->getId() << "::" << _birthdate << " r " << computeRelatedness() << std::endl;

		int randomIndex = rand()%_genomesList.size();
		std::map<int, std::vector<double> >::iterator it = _genomesList.begin();
		while (randomIndex !=0 )
		{
			it ++;
			randomIndex --;
		}

		_currentGenome = (*it).second;

		mutate(_sigmaList[(*it).first]);

		setNewGenomeStatus(true);

		_birthdate = gWorld->getIterations();

		_wm->setParent((unsigned long) (_birthdateList[(*it).first]*(gNumberOfRobots+1))+(*it).first);

		// descend from
		gLogFile << gWorld->getIterations() <<  " : " << _wm->getId() << "::" << _birthdate << " df " << (*it).first << "," << _birthdateList[(*it).first] << std::endl;
		gLogFile <<  gWorld->getIterations() << " : " << _wm->getId() << "::" << _birthdate << " e " <<  _wm->getEnergyLevel() << " gList " << _genomesList.size() << std::endl;

		_genomesList.clear();
	}
}


void EnergyMedeaController::storeGenome(std::vector<double> genome, int senderId, int senderBirthdate, float sigma)
{
	_genomesList[senderId] = genome;
	_sigmaList[senderId] = sigma;
	_birthdateList[senderId] = senderBirthdate;
}


void EnergyMedeaController::mutate( float sigma) // mutate within bounds.
{
	_genome.clear();

	_currentSigma = sigma;

	for (unsigned int i = 0 ; i != _currentGenome.size() ; i++ )
	{
		double value = _currentGenome[i] + getGaussianRand(0,_currentSigma);
		// bouncing upper/lower bounds
		if ( value < _minValue )
		{
			double range = _maxValue - _minValue;
			double overflow = - ( (double)value - _minValue );
			overflow = overflow - 2*range * (int)( overflow / (2*range) );
			if ( overflow < range )
				value = _minValue + overflow;
			else // overflow btw range and range*2
				value = _minValue + range - (overflow-range);
		}
		else if ( value > _maxValue )
		{
			double range = _maxValue - _minValue;
			double overflow = (double)value - _maxValue;
			overflow = overflow - 2*range * (int)( overflow / (2*range) );
			if ( overflow < range )
				value = _maxValue - overflow;
			else // overflow btw range and range*2
				value = _maxValue - range + (overflow-range);
		}

		_genome.push_back(value);
	}

	_currentGenome = _genome;

	// Logging
	gLogFile << gWorld->getIterations() <<  " : " << _wm->getId() << "::" << _birthdate << " s " << _currentSigma << std::endl;
}


void EnergyMedeaController::resetRobot()
{
	_nbInputs = 0;

	if ( gExtendedSensoryInputs ) // EXTENDED SENSORY INPUTS: modified for need
	{ 
		_nbInputs =  _wm->_cameraSensorsNb; // presence of energy point
	}

	_nbInputs += _wm->_cameraSensorsNb + 3; // proximity sensors + ground sensor (3 values)
	if ( gEnergyLevel )
		_nbInputs += 1; // incl. energy level
	if ( gLandmarks.size() > 0 )
		_nbInputs += 2; // incl. landmark (angle,dist)

	_nbOutputs = 2;

	_nbHiddenLayers = EnergyMedeaSharedData::gNbHiddenLayers;

	_nbNeuronsPerHiddenLayer = new std::vector<unsigned int>(_nbHiddenLayers);
	for(unsigned int i = 0; i < _nbHiddenLayers; i++)
		(*_nbNeuronsPerHiddenLayer)[i] = EnergyMedeaSharedData::gNbNeuronsPerHiddenLayer;

	createNN();

	unsigned int const nbGene = computeRequiredNumberOfWeights() + 1;//add an altruistic gene. If less than 0 the agent is egoistic. If greter than 0 the agent is altruistic

	if ( gVerbose )
		std::cout << std::flush ;

	_genome.clear();

	for ( unsigned int i = 0 ; i != nbGene ; i++ )
	{
		_genome.push_back((double)(rand()%EnergyMedeaSharedData::gNeuronWeightRange)/(EnergyMedeaSharedData::gNeuronWeightRange/2)-1.0); // weights: random init between -1 and +1
	}
	_currentGenome = _genome;
	setNewGenomeStatus(true);
	_genomesList.clear();

	if (EnergyMedeaSharedData::gAltruismEvolved == true)
	{
		_wm->setSharing(_currentGenome[_currentGenome.size()-1]);
	}
	else
	{
		_wm->setSharing(EnergyMedeaSharedData::gSharing);
	}
}


void EnergyMedeaController::broadcastGenome()
{
	for( int i = 0 ; i < _wm->_cameraSensorsNb; i++)
	{
		int targetIndex = _wm->getObjectIdFromCameraSensor(i);

		if ( targetIndex >= gRobotIndexStartOffset )   // sensor ray bumped into a robot : communication is possible
		{
			targetIndex = targetIndex - gRobotIndexStartOffset; // convert image registering index into robot id.

			EnergyMedeaController* targetRobotController = dynamic_cast<EnergyMedeaController*>(gWorld->getRobot(targetIndex)->getController());

			if ( ! targetRobotController )
			{
				std::cerr << "Error from robot " << _wm->getId() << " : the observer of robot " << targetIndex << " is not compatible" << std::endl;
				exit(-1);
			}

			float dice = float(rand()%100) / 100.0;
			float sigmaSendValue = _currentSigma;

			if ( dice <= EnergyMedeaSharedData::gProbaMutation )
			{
				dice = float(rand() %100) / 100.0;
				if ( dice < 0.5 )
				{
					sigmaSendValue = _currentSigma * ( 1 + EnergyMedeaSharedData::gUpdateSigmaStep ); // increase sigma

					if (sigmaSendValue > EnergyMedeaSharedData::gSigmaMax)
					{
						sigmaSendValue = EnergyMedeaSharedData::gSigmaMax;
					}
				}
				else
				{
					sigmaSendValue = _currentSigma * ( 1 - EnergyMedeaSharedData::gUpdateSigmaStep ); // decrease sigma

					if ( sigmaSendValue < EnergyMedeaSharedData::gSigmaMin )
					{
						sigmaSendValue = EnergyMedeaSharedData::gSigmaMin;
					}
				}
			}

			targetRobotController->storeGenome(_currentGenome, _wm->getId(), _birthdate, sigmaSendValue); // other agent stores my genome.
		}
	}
}

double EnergyMedeaController::computeRelatedness()
{
	// compute relatedness
	// d_local : average of distances between active genome and genome in the list
	// d_global : average of distances between active genome and all genomes present in the lists (except duplications of active genome)
	// relatedness = (d_global - d_local) / d_global

	// record all genomes minus duplications of own genome
	// compute activeCount in the same time
	std::vector< std::vector< double > > allGenomes;
	int activeCount = 0;
	for (int agentId = 0 ; agentId < gNumberOfRobots ; agentId ++)
	{
		EnergyMedeaAgentWorldModel* currentAgentWM = dynamic_cast<EnergyMedeaAgentWorldModel*>(gWorld->getRobot(agentId)->getWorldModel());
		if (currentAgentWM == NULL)
		{
			std::cerr << "Error from EnergyMedeaController : can't get the world model of robot " << agentId << std::endl;
			exit(1);
		}

		EnergyMedeaController* currentAgentC = dynamic_cast<EnergyMedeaController*>(gWorld->getRobot(agentId)->getController());
		if (currentAgentWM == NULL)
		{
			std::cerr << "Error from EnergyMedeaController : can't get the controller of robot " << agentId << std::endl;
			exit(1);
		}

		//TODO: qu'est-ce qu'on fait pour les agents en attente de génome ?
		if (currentAgentWM->getLifeStatus() == EnergyMedeaAgentWorldModel::ACTIVE)
		{
			for (std::map<int, std::vector<double> >::iterator it = currentAgentC->_genomesList.begin(); it != currentAgentC->_genomesList.end() ; it++)
			{
				if (((*it).first != _wm->getId()) || ( currentAgentC->_birthdateList[(*it).first] != getBirthdate()))
				{
					allGenomes.push_back((*it).second);
				}
			}
			activeCount ++;
		}
		
	}

	// compute d_local
	double d_local = 0.0;
	for (std::map<int, std::vector<double> >::iterator it = _genomesList.begin(); it != _genomesList.end() ; it++)
	{
		d_local += computeDistance(_currentGenome,(*it).second);
	}
	d_local = d_local /_genomesList.size();

	// compute d_global
	double d_global = 0.0;
	for (unsigned int i = 0 ; i < allGenomes.size() ; i++)
	{
		d_global += computeDistance(_currentGenome,allGenomes[i]);
	}
	d_global = d_global / allGenomes.size();

	//return (d_local - d_global)/d_global;
	return (d_global - d_local)/d_global;
}


double EnergyMedeaController::computeDistance(std::vector<double> g1 , std::vector<double> g2)
{
	double distance = 0.0;
	if (g1.size() != g2.size())
	{
		std::cerr << "Try to compute the distance between two genomes of different sizes :" << std::endl;
		std::cerr << "G1: " << g1.size() << std::endl;
		std::cerr << "G2: " << g2.size() << std::endl;
		exit(1);
	}

	for (unsigned int i = 0 ; i < g1.size() ; i++)
	{
		distance += pow(g1[i]-g2[i],2);
	}
	return distance;
}
