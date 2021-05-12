//
// Copyright (c) 2009-2018 Brandon Haworth, Glen Berseth, Muhammad Usman, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//




#ifndef __PHASE_DECIMATION_GRID_ASTAR_H__
#define __PHASE_DECIMATION_GRID_ASTAR_H__

#include "astar/Environment.h"
#include "SteerLib.h"


#define BRANCHING_FACTOR 7

class SteerSuiteGridEnvironment : public Environment
{
public:

	/// @todo eventually remove the hard-coded "1000" here.
	bool canBeTraversed(unsigned int index) const { return (_spatialDatabase->getTraversalCost(index) < 10); }

	//=================================================================================
	//=================================================================================
	SteerSuiteGridEnvironment(SteerLib::GridDatabase2D * db) { _spatialDatabase = db; }

	//=================================================================================
	//=================================================================================
	float getHeuristic(int start, int target) const 
	{
		unsigned int xstart, zstart, xtarget, ztarget;
		_spatialDatabase->getGridCoordinatesFromIndex(start, xstart, zstart);
		_spatialDatabase->getGridCoordinatesFromIndex(target, xtarget, ztarget);
		// NOTE diffx, and diffz are signed; without the typecasting here, can get overflow.
		int diffx = ((int)xtarget) - ((int)xstart);
		int diffz = ((int)ztarget) - ((int)zstart);
		float score = sqrtf((float)(diffx*diffx + diffz*diffz));
		//float score = sqrtf((float)(xtarget-xstart)*(float)(xtarget-xstart) + (float)(ztarget-zstart)*(float)(ztarget-zstart));
		return score;
	}

	//=================================================================================
	//=================================================================================
	void getSuccessors(int nodeId, int lastNodeId, vector<Successor>& result) const
	{
		if (BRANCHING_FACTOR == 7) {
			result.reserve(7); // there will only be 7 potential neighbors at any given node (the eighth one would be the previous node we came from, doesn't count)
			result.clear();
			unsigned int x, z;
			_spatialDatabase->getGridCoordinatesFromIndex(nodeId, x, z);
			//
			// THREE conditions for each potential node to be a "successor":
			// 1. if x+1, x-1, z+1, or z-1 are still within proper bounds of the grid
			//     - note because these are unsigned types, we have slightly different but equivalent conditional checks.
			// 2. if it's not the lastNodeId (i.e. previous node of our path)
			// 3. if it can be traversed, as told by the database itself
			// note that for diagonals, the adjacent blocks must also be traversable.
			//
			if (x+1 < _spatialDatabase->getNumCellsX()) {
				// possibly add the node located at (x+1, z)
				int n = _spatialDatabase->getCellIndexFromGridCoords(x+1,z);
				if ((n != lastNodeId)&&(canBeTraversed(n))) {
					result.push_back(Successor(n,_spatialDatabase->getTraversalCost(n)));
				}

				// possibly add the node located at (x+1, z+1)
				if (z+1 < _spatialDatabase->getNumCellsZ()) {
					int n = _spatialDatabase->getCellIndexFromGridCoords(x+1,z+1);
					int nAdjacent1 = _spatialDatabase->getCellIndexFromGridCoords(x,z+1);
					int nAdjacent2 = _spatialDatabase->getCellIndexFromGridCoords(x+1,z);
					if ((n != lastNodeId)&&(canBeTraversed(n))&&(canBeTraversed(nAdjacent1))&&(canBeTraversed(nAdjacent2))) {
						result.push_back(Successor(n,_spatialDatabase->getTraversalCost(n) * sqrtf(2)));
					}
				}

				// possibly add the node located at (x+1, z-1)
				if (z >= 1) {
					int n = _spatialDatabase->getCellIndexFromGridCoords(x+1,z-1);
					int nAdjacent1 = _spatialDatabase->getCellIndexFromGridCoords(x,z-1);
					int nAdjacent2 = _spatialDatabase->getCellIndexFromGridCoords(x+1,z);
					if ((n != lastNodeId)&&(canBeTraversed(n))&&(canBeTraversed(nAdjacent1))&&(canBeTraversed(nAdjacent2))) {
						result.push_back(Successor(n,_spatialDatabase->getTraversalCost(n) * sqrtf(2)));
					}
				}
			}

			if (x >= 1) {
				// possibly add the node located at (x-1, z)
				int n = _spatialDatabase->getCellIndexFromGridCoords(x-1,z);
				if ((n != lastNodeId)&&(canBeTraversed(n))) {
					result.push_back(Successor(n,_spatialDatabase->getTraversalCost(n)));
				}

				// possibly add the node located at (x-1, z+1)
				if (z+1 < _spatialDatabase->getNumCellsZ()) {
					int n = _spatialDatabase->getCellIndexFromGridCoords(x-1,z+1);
					int nAdjacent1 = _spatialDatabase->getCellIndexFromGridCoords(x,z+1);
					int nAdjacent2 = _spatialDatabase->getCellIndexFromGridCoords(x-1,z);
					if ((n != lastNodeId)&&(canBeTraversed(n))&&(canBeTraversed(nAdjacent1))&&(canBeTraversed(nAdjacent2))) {
						result.push_back(Successor(n,_spatialDatabase->getTraversalCost(n) * sqrtf(2)));
					}
				}

				// possibly add the node located at (x-1, z-1)
				if (z >= 1) {
					int n = _spatialDatabase->getCellIndexFromGridCoords(x-1,z-1);
					int nAdjacent1 = _spatialDatabase->getCellIndexFromGridCoords(x,z-1);
					int nAdjacent2 = _spatialDatabase->getCellIndexFromGridCoords(x-1,z);
					if ((n != lastNodeId)&&(canBeTraversed(n))&&(canBeTraversed(nAdjacent1))&&(canBeTraversed(nAdjacent2))) {
						result.push_back(Successor(n,_spatialDatabase->getTraversalCost(n) * sqrtf(2)));
					}
				}
			}

			// possibly add the node located at (x, z+1)
			if (z+1 < _spatialDatabase->getNumCellsZ()) {
				int n = _spatialDatabase->getCellIndexFromGridCoords(x,z+1);
				if ((n != lastNodeId)&&(canBeTraversed(n))) {
					result.push_back(Successor(n,_spatialDatabase->getTraversalCost(n)));
				}
			}

			// possibly add the node located at (x, z-1)
			if (z >= 1) {
				int n = _spatialDatabase->getCellIndexFromGridCoords(x,z-1);
				if ((n != lastNodeId)&&(canBeTraversed(n))) {
					result.push_back(Successor(n,_spatialDatabase->getTraversalCost(n)));
				}
			}
		}
		else if (BRANCHING_FACTOR == 3) {
			result.reserve(3); // there will only be 3 potential neighbors at any given node (the fourth one would be the previous node we came from, doesn't count)
			result.clear();
			unsigned int x, z;
			_spatialDatabase->getGridCoordinatesFromIndex(nodeId, x, z);
			// THREE conditions for each potential node to be a "successor":
			// 1. if x+1, x-1, z+1, or z-1 are still within proper bounds of the grid
			//     - note because these are unsigned types, we have slightly different but equivalent conditional checks.
			// 2. if it's not the lastNodeId (i.e. previous node of our path)
			// 3. if its not blocked, as told by the database itself
			if (x+1 < _spatialDatabase->getNumCellsX()) {
				int n = _spatialDatabase->getCellIndexFromGridCoords(x+1,z);
				if ((n != lastNodeId)&&(canBeTraversed(n))) {
					result.push_back(Successor(n,_spatialDatabase->getTraversalCost(n)));
				}
			}
			if (x >= 1) {
				int n = _spatialDatabase->getCellIndexFromGridCoords(x-1,z);
				if ((n != lastNodeId)&&(canBeTraversed(n))) {
					result.push_back(Successor(n,_spatialDatabase->getTraversalCost(n)));
				}
			}
			if (z+1 < _spatialDatabase->getNumCellsZ()) {
				int n = _spatialDatabase->getCellIndexFromGridCoords(x,z+1);
				if ((n != lastNodeId)&&(canBeTraversed(n))) {
					result.push_back(Successor(n,_spatialDatabase->getTraversalCost(n)));
				}
			}
			if (z >= 1) {
				int n = _spatialDatabase->getCellIndexFromGridCoords(x,z-1);
				if ((n != lastNodeId)&&(canBeTraversed(n))) {
					result.push_back(Successor(n,_spatialDatabase->getTraversalCost(n)));
				}
			}
		}
		else {
			throw Util::GenericException("ERROR: invalid branching factor.\n");
		}
	}

	//=================================================================================
	//=================================================================================
	//float getMaxCost() const { return 1.0f; }  // TODO: what is the meaning of "max" in this case?   max possible?  or max of something else?
	//float getMinCost() const { return 1.0f; }
	//int getNumberNodes() const { return _spatialDatabase->getNumCellsX() * _spatialDatabase->getNumCellsZ(); }
	bool isValidNodeId(int nodeId) const  { return (nodeId >= 0) && ((unsigned int)nodeId) < (_spatialDatabase->getNumCellsX() * _spatialDatabase->getNumCellsZ()); }

	SteerLib::GridDatabase2D * _spatialDatabase;
};


#endif
