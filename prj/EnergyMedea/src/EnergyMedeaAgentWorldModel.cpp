/**
 * @author Jean-Marc Montanier <montanier.jeanmarc@gmail.com>
 *
 */

#include "EnergyMedea/include/EnergyMedeaAgentWorldModel.h"

EnergyMedeaAgentWorldModel::EnergyMedeaAgentWorldModel()
{
	_lifeStatus = ACTIVE;

    std::string s = "";
	std::stringstream out;
	out << getId();

	s = "physicalObject[";
	s += out.str();
	s += "].energy";
	if ( gProperties.hasProperty( s ) )
		convertFromString<double>(maxEnergyLevel, gProperties.getProperty( s ), std::dec);
	else
    {
        if ( gVerbose )
            std::cerr << "[MISSING] Physical Object #" << _id << " (EnergyItem) missing default energy initial level (integer, >=0). Assume default (" << gEnergyItemDefaultInit << ").\n";
        maxEnergyLevel = gEnergyItemDefaultInit;
    }
	
	_totalHarvested = 0.0;
}

EnergyMedeaAgentWorldModel::~EnergyMedeaAgentWorldModel()
{
}

int EnergyMedeaAgentWorldModel::getLifeStatus()
{
	return _lifeStatus ;
}

void EnergyMedeaAgentWorldModel::setLifeStatus( int __status )
{
	assert (__status == ACTIVE || __status == LISTEN || __status == DEAD );

	if (__status == ACTIVE)
		this->setAlive(true);
	else 
		this->setAlive(false);

	_lifeStatus = __status;
}
