/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */



#ifndef ENERGYMEDEACONTROLLER_H
#define ENERGYMEDEACONTROLLER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Utilities/Graphics.h"
#include "Controllers/Controller.h"
#include "WorldModels/RobotWorldModel.h"
#include "EnergyMedea/include/EnergyMedeaAgentObserver.h"
#include "EnergyMedea/include/EnergyMedeaAgentWorldModel.h"
#include <neuralnetworks/NeuralNetwork.h>
#include "Utilities/Geometry.h"

#include <iomanip>

using namespace Neural;


class EnergyMedeaController : public Controller
{
	public:
		EnergyMedeaController (RobotWorldModel *wm);

	private:
		int _iteration;
		int _birthdate; // evaluation when this controller was initialized.

		EnergyMedeaAgentWorldModel *_wm;

		std::vector<double> _parameters;
		std::string _nnType;
		std::vector<int> _nbHiddenNeuronsPerLayer;
		std::vector<int> _nbBiaisNeuronsPerLayer;
		NeuralNetwork* nn;

		void createNN();

		//bool _isAlive; // agent stand still if not.
		bool _isNewGenome;

		void selectRandomGenome();
		void mutate(float sigma);

		void stepBehaviour();
		void stepEvolution();
		void logEndGeneration();

		void broadcastGenome();
		bool loadNewGenome();

		double computeRelatedness();
		double computeDistance(std::vector<double> g1 , std::vector<double> g2);

		unsigned int computeRequiredNumberOfWeights();

		//        void setAliveStatus( bool isAlive ) { _isAlive = isAlive; }
		bool getNewGenomeStatus() { return _isNewGenome; }
		void setNewGenomeStatus( bool __status ) { _isNewGenome = __status; }

		// evolutionary engine
		std::vector<double> _genome; // todo: accessing
		std::map<int, std::vector<double> > _genomesList;
		std::map<int, float > _sigmaList;
		std::map<int,int> _birthdateList; // store the birthdate of the received controllers (useful for monitoring).
		std::vector<double> _currentGenome;
		float _currentSigma;

		// ANN
		double _minValue;
		double _maxValue;
		unsigned int _nbInputs;
		unsigned int _nbOutputs;
		unsigned int _nbHiddenLayers;
		std::vector<unsigned int>* _nbNeuronsPerHiddenLayer;
	
		int _totalHarvested;

		Point2d _startPos;
		Point2d _endPos;
		double _cumulatedDistance;

		void storeGenome(std::vector<double> genome, int senderId, int senderBirthdate, float sigma);
		void resetRobot();

	public:

		EnergyMedeaController(EnergyMedeaAgentWorldModel *wm);
		~EnergyMedeaController();

		void reset();
		void step();

		int getBirthdate() { return _birthdate; }

		//bool isAlive() { return _isAlive; }

		RobotWorldModel* getWorldModel() { return _wm; }


};


#endif

