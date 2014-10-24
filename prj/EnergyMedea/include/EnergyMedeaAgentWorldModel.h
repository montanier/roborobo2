/**
 * @author Jean-Marc Montanier <montanier.jeanmarc@gmail.com>
 *
 */

#ifndef ENERGYMEDEAAGENTWORLDMODEL_H
#define ENERGYMEDEAAGENTWORLDMODEL_H 


#include "WorldModels/RobotWorldModel.h"
#include "EnergyMedea/include/EnergyMedeaSharedData.h"

class EnergyMedeaAgentWorldModel : public RobotWorldModel
{
	protected:
		int _lifeStatus ;
		double _totalHarvested;
		unsigned long _parent;
		double _sharing;

	public:
		EnergyMedeaAgentWorldModel();
		~EnergyMedeaAgentWorldModel();

		double maxEnergyLevel;

		//possible life status
		static const int ACTIVE = 0;
		static const int LISTEN = 1;
		static const int DEAD = 2;

		int getLifeStatus();
		void setLifeStatus( int __status );


		double getEnergyHarvested() { return _totalHarvested; }
		void increaseEnergyHarvested() { _totalHarvested += maxEnergyLevel;  } 
		void resetEnergyHarvested() { _totalHarvested = 0 ; }

		unsigned long getParent() {return _parent;}
		void setParent(unsigned long parent) {_parent = parent;}

		double getSharing() {return _sharing;}
		void setSharing(double sharing) {_sharing = sharing;}
};

#endif

