/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * NNlib: Leo Cazenille <leo.cazenille@upmc.fr>
 */

#include "Observers/AgentObserver.h"
#include "Observers/WorldObserver.h"
#include "EnergyMedea/include/EnergyMedeaWorldObserver.h"
#include "EnergyMedea/include/EnergyMedeaController.h"
#include "EnergyMedea/include/EnergyMedeaAgentWorldModel.h"
#include "World/World.h"


EnergyMedeaWorldObserver::EnergyMedeaWorldObserver( World* world ) : WorldObserver( world )
{
    _world = world;

	// ==== loading project-specific properties

	gProperties.checkAndGetPropertyValue("gSigmaRef",&EnergyMedeaSharedData::gSigmaRef,true);
	gProperties.checkAndGetPropertyValue("gSigmaMin",&EnergyMedeaSharedData::gSigmaMin,true);
	gProperties.checkAndGetPropertyValue("gSigmaMax",&EnergyMedeaSharedData::gSigmaMax,true);

	gProperties.checkAndGetPropertyValue("gProbaMutation",&EnergyMedeaSharedData::gProbaMutation,true);
	gProperties.checkAndGetPropertyValue("gUpdateSigmaStep",&EnergyMedeaSharedData::gUpdateSigmaStep,true);
	gProperties.checkAndGetPropertyValue("gEvaluationTime",&EnergyMedeaSharedData::gEvaluationTime,true);
	gProperties.checkAndGetPropertyValue("gSynchronization",&EnergyMedeaSharedData::gSynchronization,true);
	gProperties.checkAndGetPropertyValue("gDeadTime",&EnergyMedeaSharedData::gDeadTime,true);

    gProperties.checkAndGetPropertyValue("gEnergyRequestOutput",&EnergyMedeaSharedData::gEnergyRequestOutput,false);
    gProperties.checkAndGetPropertyValue("gSacrifice",&EnergyMedeaSharedData::gSacrifice,false);
    
	gProperties.checkAndGetPropertyValue("gMonitorPositions",&EnergyMedeaSharedData::gMonitorPositions,true);

    gProperties.checkAndGetPropertyValue("gNbHiddenLayers",&EnergyMedeaSharedData::gNbHiddenLayers,true);
	gProperties.checkAndGetPropertyValue("gNbNeuronsPerHiddenLayer",&EnergyMedeaSharedData::gNbNeuronsPerHiddenLayer,true);
	gProperties.checkAndGetPropertyValue("gNeuronWeightRange",&EnergyMedeaSharedData::gNeuronWeightRange,true);
    
	gProperties.checkAndGetPropertyValue("gSnapshots",&EnergyMedeaSharedData::gSnapshots,false);
	gProperties.checkAndGetPropertyValue("gSnapshotsFrequency",&EnergyMedeaSharedData::gSnapshotsFrequency,false);

    gProperties.checkAndGetPropertyValue("gControllerType",&EnergyMedeaSharedData::gControllerType,false);

    
	// ====

	if ( !gRadioNetwork)
	{
		std::cout << "Error : gRadioNetwork must be true." << std::endl;
		exit(-1);
	}

	// * iteration and generation counters

	_lifeIterationCount = -1;
	_generationCount = -1;

}

EnergyMedeaWorldObserver::~EnergyMedeaWorldObserver()
{
	// nothing to do.
}

void EnergyMedeaWorldObserver::reset()
{
	// nothing to do.
}

void EnergyMedeaWorldObserver::step()
{
    _lifeIterationCount++;
    
    updateMonitoring();

    if( _lifeIterationCount >= EnergyMedeaSharedData::gEvaluationTime ) // switch to next generation.
	{
        // update iterations and generations counters
        _lifeIterationCount = 0;
        _generationCount++;
    }

	updateEnvironment();
    
}


void EnergyMedeaWorldObserver::updateEnvironment()
{
	// ...
}

void EnergyMedeaWorldObserver::updateMonitoring()
{
    // * Log at end of each generation
    
    if( _lifeIterationCount >= EnergyMedeaSharedData::gEvaluationTime ) // switch to next generation.
	{
		// * monitoring: count number of active agents.
        
		int activeCount = 0;
		double total_energy = 0.0;
		for ( int i = 0 ; i != gNumberOfRobots ; i++ )
		{
			if ( (dynamic_cast<EnergyMedeaAgentWorldModel*>(gWorld->getRobot(i)->getWorldModel()))->isAlive() == true )
				activeCount++;
			total_energy += (dynamic_cast<EnergyMedeaAgentWorldModel*>(gWorld->getRobot(i)->getWorldModel()))->getEnergyHarvested();
			(dynamic_cast<EnergyMedeaAgentWorldModel*>(gWorld->getRobot(i)->getWorldModel()))->resetEnergyHarvested();
		}
        
		if ( gVerbose )
		{
			std::cout << "[gen:" << (gWorld->getIterations()/EnergyMedeaSharedData::gEvaluationTime) << ";pop:" << activeCount << "]\n";
		}
        
        // Logging
        gLogFile << gWorld->getIterations() << " : pop_alive " << activeCount << std::endl;
        gLogFile << gWorld->getIterations() << " : tot_energy " << total_energy << std::endl;
	}
    
    // * Every N generations, take a video (one generation)
    
    if ( EnergyMedeaSharedData::gSnapshots )
    {
        if ( ( gWorld->getIterations() ) % ( EnergyMedeaSharedData::gEvaluationTime * EnergyMedeaSharedData::gSnapshotsFrequency ) == 0 )
        {
            if ( gVerbose )
                std::cout << "[START] Video recording: generation #" << (gWorld->getIterations() / EnergyMedeaSharedData::gEvaluationTime ) << ".\n";
            gTrajectoryMonitorMode = 0;
            initTrajectoriesMonitor();
        }
        else
            if ( ( gWorld->getIterations() ) % ( EnergyMedeaSharedData::gEvaluationTime * EnergyMedeaSharedData::gSnapshotsFrequency ) == EnergyMedeaSharedData::gEvaluationTime - 1 )
            {
                std::cout << "[STOP]  Video recording: generation #" << (gWorld->getIterations() / EnergyMedeaSharedData::gEvaluationTime ) << ".\n";
                saveTrajectoryImage();
            }
    }

    // * Snapshots: take screenshots of first and ~ultimate iteration
    /* //todelete
    if ( gWorld->getIterations() == 1 )
    {
        saveScreenshot("firstIteration");
        saveRobotTrackerIndex("firstIteration");
    }
    else
    {
        if ( gWorld->getIterations() == gMaxIt-2 )
        {
            gDisplayMode = 0;
            //gDisplaySensors = 2; // prepare for next it.
        }
        else
        {
            if ( gWorld->getIterations() == gMaxIt-1 )
            {
                saveScreenshot("lastIteration");
                saveRobotTrackerIndex("lastIteration");
            }
        }
    }
    */
    
}

