/*
 * This is header file for tier-1 advisors. 
 *
 * Date Created: Jan. 7, 2014.
 * Last Edited: Jan. 7, 2014.
 * Created by: Slavisa Djukic <sdjukic@hunter.cuny.edu>
 */

#include <string>
#include <vector>
#include "FORRAction.h"
#include "Beliefs.h"
#include "LocalExplore.h"

// A t1 advisor class contains a list of functions each of which returns a single action or vetoes a set of actions
class Tier1Advisor{

public:
        Tier1Advisor(Beliefs *b){
		beliefs = b;
		localExploration = new LocalExplorer();
	}

	void advisorNotOpposite();

	bool advisorAvoidObstacles();

	bool advisorDontGoBack();

	bool advisorSituation();

	bool advisorGetOut(FORRAction *decision);

	bool advisorVictory(FORRAction *decision);

	bool advisorEnforcer(FORRAction *decision);

	bool advisorDoorway(FORRAction *decision);

	bool advisorFindAWay(FORRAction *decision);

	bool advisorBehindYou(FORRAction *decision);

	void resetLocalExploration(){
		localExploration->resetLocalExplorer();
	}
	
private:
	Beliefs *beliefs;
	LocalExplorer *localExploration;
};
