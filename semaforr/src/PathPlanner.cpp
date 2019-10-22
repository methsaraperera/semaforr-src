/*!
  \file PathPlanner.cpp
  \addtogroup PathPlanner
  @{
 */

#include "PathPlanner.h"
#include <limits.h>
#include <algorithm>

#define PATH_DEBUG true

/*!
  \brief Calculates the shortest path from a start point to a destination point on the navigation graph.

  This function makes necessary calls to populate the PathPlanner::path list, which is a list of node indexes. The list doesn't contain the source (start point) and the target (destination point), only the nodes that the robot needs to get to in sequence, in order to reach its destination.

  First A* algorithm is run on the navigation graph and then the path is smoothed by removing unnecessary waypoints.

  \b Warning a source and a target must be specified prior to this function call.

  \return 0 if path is calculated, 1 if source is not a node or accessible to a valid node, 2 if target is not a node or accessible to a valid node, 3 if no path is found between the source and the target

 */
int PathPlanner::calcPath(bool cautious){
  const string signature = "PathPlanner::calcPath()> ";

  if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){

    if(PATH_DEBUG) {
      cout << signature << "Source:";
      source.printNode();
      cout << endl;
      cout << signature << "Target:";
      target.printNode();
      cout << endl;
    }

    Node s, t;
    if ( navGraph->isNode(source) ) {
      if(PATH_DEBUG)
        cout << signature << "Source is a valid Node in the navigation graph" << endl;
      s = source ;
    }
    else {
      if(PATH_DEBUG)
        cout << signature << "Source is not a valid Node in the navigation graph. Getting closest valid node." << endl;
      s = getClosestNode(source, target);
    }
    //cout << signature << "Checking if source node is invalid" << endl;
    if ( s.getID() == Node::invalid_node_index )
      return 1;

    if ( navGraph->isNode(target) ) {
      if(PATH_DEBUG)
        cout << signature << "Target is a valid Node in the navigation graph" << endl;
      t = target ;
    }
    else {
      if(PATH_DEBUG)
        cout << signature << "Target is not a valid Node in the navigation graph. Getting closest valid node." << endl;
      t = getClosestNode(target, source);
    }
    //cout << signature << "Checking if target node is invalid" << endl;
    if ( t.getID() == Node::invalid_node_index )
      return 2;

    //cout << signature << "Completed finding source and destination nodes" << endl;
    if(PATH_DEBUG) {
      cout << signature << "s:";
      s.printNode();
      cout << endl;
      cout << signature << "t:";
      t.printNode();
      cout << endl;
    }
    if(s.getID() == t.getID())
      return 4;
    //cout << signature << "Updating nav graph" << endl;
    // update the nav graph with the latest crowd model to change the edge weights
    if (name != "distance" and name != "skeleton") {
      cout << "Updating nav graph for non-distance planners" << endl;
      updateNavGraph();
    }
    astar newsearch(*navGraph, s, t, name);
    if ( newsearch.isPathFound() ) {
      path = newsearch.getPathToTarget();
      paths = newsearch.getPathsToTarget();
      objectiveSet = false;
      pathCompleted = false;

      if(!cautious)
        smoothPath(path, s, t);

      pathCost = calcPathCost(path);
      pathCalculated = true;
      for (int i=0; i<paths.size(); i++){
        pathCosts.push_back(calcPathCost(paths[i]));
      }
    }
    else {
      return 3;
    }
  }
  return 0;
}

int PathPlanner::calcOrigPath(bool cautious){
  const string signature = "PathPlanner::calcOrigPath()> ";

  if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){

    if(PATH_DEBUG) {
      cout << signature << "Source:"; 
      source.printNode(); 
      cout << endl;
      cout << signature << "Target:"; 
      target.printNode();
      cout << endl;
    }

    Node s, t;
    if ( originalNavGraph->isNode(source) ) {
      if(PATH_DEBUG)
        cout << signature << "Source is a valid Node in the navigation graph" << endl; 
      s = source ;
    }
    else {
      if(PATH_DEBUG)
        cout << signature << "Source is not a valid Node in the navigation graph. Getting closest valid node." << endl; 
      s = getClosestNode(source, target);
    }
    //cout << signature << "Checking if source node is invalid" << endl;
    if ( s.getID() == Node::invalid_node_index )
      return 1;

    if ( originalNavGraph->isNode(target) ) {
      if(PATH_DEBUG)
        cout << signature << "Target is a valid Node in the navigation graph" << endl; 
      t = target ;
    }
    else {
      if(PATH_DEBUG)
        cout << signature << "Target is not a valid Node in the navigation graph. Getting closest valid node." << endl; 
      t = getClosestNode(target, source);
    }
    //cout << signature << "Checking if target node is invalid" << endl;
    if ( t.getID() == Node::invalid_node_index )
      return 2;

    //cout << signature << "Completed finding source and destination nodes" << endl;
    if(PATH_DEBUG) {
      cout << signature << "s:"; 
      s.printNode(); 
      cout << endl;
      cout << signature << "t:"; 
      t.printNode();
      cout << endl;
    }

    astar newsearch(*originalNavGraph, s, t, "distance");
    if ( newsearch.isPathFound() ) {
      origPath = newsearch.getPathToTarget();
      origObjectiveSet = false;
      origPathCompleted = false;

      if(!cautious)
        smoothPath(origPath, s, t);

      origPathCost = calcOrigPathCost(origPath); 
      origPathCalculated = true;
    }
    else {
      return 3;
    }
  }
  return 0;
}

void PathPlanner::updateNavGraph(){
	cout << "Updating nav graph before with the current crowd model" << endl;
	if(crowdModel.densities.size() == 0 and (name == "density" or name == "risk" or name == "flow")){
		cout << "crowdModel not recieved" << endl;
	}
	else{
		//cout << crowdModel.height << endl;
		/*for(int i = 0 ; i < crowdModel.densities.size(); i++){
			cout << crowdModel.densities[i] << endl;
		}*/
		vector<Edge*> edges = navGraph->getEdges();
		// compute the extra cost imposed by crowd model on each edge in navGraph
		for(int i = 0; i < edges.size(); i++){
			Node toNode = navGraph->getNode(edges[i]->getTo());
			Node fromNode = navGraph->getNode(edges[i]->getFrom());
			double oldcost = edges[i]->getDistCost();
			double newEdgeCostft = computeNewEdgeCost(fromNode, toNode, true, oldcost);
			double newEdgeCosttf = computeNewEdgeCost(fromNode, toNode, false, oldcost); 
			navGraph->updateEdgeCost(i, newEdgeCostft, newEdgeCosttf);
			//cout << "Edge Cost " << oldcost << " -> " << newEdgeCostft << " -> " << newEdgeCosttf << endl;
		}
	}
}


double PathPlanner::computeNewEdgeCost(Node s, Node d, bool direction, double oldcost){
	int b = 30;
  // weights that balance distance, crowd density and crowd flow
  int w1 = 1;
  int w2 = 500;
  int w3 = 500;
  int w4 = 500;
  int w5 = 500;
  int w6 = 1;
  int w7 = 1;
  int w8 = 1;
  if (name == "smooth"){
    //cout << "Updating smooth nav graph" << endl;
    //double smooth_cost = (oldcost * 5);
    double smooth_cost = 1;
    return (w6 * smooth_cost);
  }
  if (name == "novel"){
    double ns_cost = novelCost(s.getX(), s.getY());
    double nd_cost = novelCost(d.getX(), d.getY());
    /*if (ns_cost > 0 or nd_cost > 0) {
      cout << "old cost = " << oldcost << " novelCost = " << (ns_cost+nd_cost)/2 << " combined cost = " << (w1 * oldcost) + (w5 * (ns_cost+nd_cost)/2) << endl;
    }*/
    //return (w1 * oldcost) + (w5 * (ns_cost+nd_cost)/2);
    if (ns_cost > 0 or nd_cost > 0){
      return (w1 * oldcost) + (w5 * (ns_cost+nd_cost)/2);
    }
    else{
      return (w1 * oldcost);
    }
  }
  if (name == "density"){
    //cout << "Updating density nav graph" << endl;
    double s_cost = cellCost(s.getX(), s.getY(), b);
    double d_cost = cellCost(d.getX(), d.getY(), b);
    //return (w1 * oldcost) + (w2 * (s_cost+d_cost)/2);
    if (s_cost > 0 or d_cost > 0){
      return (w1 * oldcost) + (w2 * (s_cost+d_cost)/2);
    }
    else{
      return (w1 * oldcost);
    }
  }
  if (name == "risk"){
    //cout << "Updating risk nav graph" << endl;
    double s_risk_cost = riskCost(s.getX(), s.getY(), b);
    double d_risk_cost = riskCost(d.getX(), d.getY(), b);
    //return (w1 * oldcost) + (w4 * (s_risk_cost+d_risk_cost)/2);
    if (s_risk_cost > 0 or d_risk_cost > 0){
      return (w1 * oldcost) + (w4 * (s_risk_cost+d_risk_cost)/2);
    }
    else{
      return (w1 * oldcost);
    }
  }
  if (name == "flow"){
    //cout << "updating flow nav graph" << endl;
    double flowcost = computeCrowdFlow(s,d);
    if(direction == true){
      flowcost = flowcost * (-1);
    }
    if(flowcost < 0){
      flowcost = 0;
    }
    //return (w1 * oldcost) + (w3 * flowcost);
    if (flowcost > 0){
      return (w1 * oldcost) + (w3 * flowcost);
    }
    else{
      return (w1 * oldcost);
    }
  }
  if (name == "spatial"){
    int sRegion=-1,dRegion=-1;
    for(int i = 0; i < regions.size() ; i++){
      if(regions[i].inRegion(s.getX()/100.0, s.getY()/100.0)){
        sRegion = i;
      }
      if(regions[i].inRegion(d.getX()/100.0, d.getY()/100.0)){
        dRegion = i;
      }
      if(sRegion >= 0 and dRegion >= 0){
        break;
      }
    }

    double s_door_min_distance = std::numeric_limits<double>::infinity();
    double s_exit_min_distance = std::numeric_limits<double>::infinity();
    if(sRegion >= 0){
      CartesianPoint sPoint = CartesianPoint(s.getX()/100.0, s.getY()/100.0);
      for(int i = 0; i < doors[sRegion].size(); i++) {
        double doorDistance = doors[sRegion][i].distanceToDoor(sPoint, regions[sRegion]);
        if (doorDistance < s_door_min_distance){
          s_door_min_distance = doorDistance;
        }
      }
      vector<FORRExit> exits = regions[sRegion].getExits();
      for(int i = 0 ; i < exits.size(); i++){
        double exitDistance = sPoint.get_distance(CartesianPoint(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()));
        if (exitDistance < s_exit_min_distance){
          s_exit_min_distance = exitDistance;
        }
      }
    }

    double d_door_min_distance = std::numeric_limits<double>::infinity();
    double d_exit_min_distance = std::numeric_limits<double>::infinity();
    if(dRegion >= 0){
      CartesianPoint dPoint = CartesianPoint(d.getX()/100.0, d.getY()/100.0);
      for(int i = 0; i < doors[dRegion].size(); i++) {
        double doorDistance = doors[dRegion][i].distanceToDoor(dPoint, regions[dRegion]);
        if (doorDistance < d_door_min_distance){
          d_door_min_distance = doorDistance;
        }
      }
      vector<FORRExit> exits = regions[dRegion].getExits();
      for(int i = 0 ; i < exits.size(); i++){
        double exitDistance = dPoint.get_distance(CartesianPoint(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()));
        if (exitDistance < d_exit_min_distance){
          d_exit_min_distance = exitDistance;
        }
      }
    }

    if (sRegion >= 0 and dRegion >= 0){
      return (w1 * oldcost) * 0.25;
    }
    else if (sRegion >= 0 and dRegion == -1){
      if(s_door_min_distance <= 0.5 and s_exit_min_distance <= 0.5){
        return (w1 * oldcost) * 0.5;
      }
      else if(s_door_min_distance <= 0.5 or s_exit_min_distance <= 0.5){
        return (w1 * oldcost) * 0.75;
      }
      else{
        return (w1 * oldcost) * 1;
      }
    }
    else if (sRegion ==-1 and dRegion >= 0){
      if(d_door_min_distance <= 0.5 and d_exit_min_distance <= 0.5){
        return (w1 * oldcost) * 0.5;
      }
      else if(d_door_min_distance <= 0.5 or d_exit_min_distance <= 0.5){
        return (w1 * oldcost) * 0.75;
      }
      else{
        return (w1 * oldcost) * 1;
      }
    }
    else{
      return (w1 * oldcost) * 10;
    }
  }
  if (name == "hallwayer"){
    double sHallway=0, dHallway=0;
    for(int i = 0; i < hallways.size(); i++){
      if(hallways[i].pointInAggregate(CartesianPoint(s.getX()/100.0, s.getY()/100.0))){
        sHallway++;
      }
      if(hallways[i].pointInAggregate(CartesianPoint(d.getX()/100.0, d.getY()/100.0))){
        dHallway++;
      }
    }
    if (sHallway > 0 and dHallway > 0){
      return (w1 * oldcost) * 1/((sHallway+dHallway)/2);
    }
    else{
      return (w1 * oldcost) * 10;
    }
  }
  if (name == "conveys"){
    double sconveycost = computeConveyorCost(s.getX(), s.getY());
    double dconveycost = computeConveyorCost(d.getX(), d.getY());
    //return (w7 * oldcost*pow(0.25,((sconveycost + dconveycost)/2)));
    if (sconveycost > 0 and dconveycost > 0){
      return (w7 * oldcost * 1/((sconveycost + dconveycost)/2));
    }
    else{
      return (w7 * oldcost * 10);
    }
  }
  if (name == "trailer"){
    //cout << "updating trailer nav graph" << endl;
    double strailcount = 0;
    double dtrailcount = 0;
    //cout << "trails.size() = " << trails.size() << endl;
    for(int i = 0; i < trails.size(); i++){
      //cout << "trails[i].size() = " << trails[i].size() << endl;
      for(int j = 0; j < trails[i].size(); j++){
        if(trails[i][j].get_distance(CartesianPoint(s.getX()/100.0, s.getY()/100.0)) <= 0.5){
          strailcount++;
          //cout << "trails[i][j] = " << trails[i][j].get_x() << ", " << trails[i][j].get_y() << endl;
          //cout << "s = " << s.getX()/100.0 << ", " << s.getY()/100.0 << endl;
        }
        if(trails[i][j].get_distance(CartesianPoint(d.getX()/100.0, d.getY()/100.0)) <= 0.5){
          dtrailcount++;
          //cout << "trails[i][j] = " << trails[i][j].get_x() << ", " << trails[i][j].get_y() << endl;
          //cout << "d = " << d.getX()/100.0 << ", " << d.getY()/100.0 << endl;
        }
      }
    }
    //cout << "strailcount = " << strailcount << " dtrailcount = " << dtrailcount << endl;
    //return (w8 * oldcost*pow(0.25,((strailcount + dtrailcount)/2)));
    if (strailcount > 0 and dtrailcount > 0){
      return (w7 * oldcost * 1/((strailcount + dtrailcount)/2));
    }
    else{
      return (w7 * oldcost * 10);
    }
  }
  if (name == "combined"){
    /*double s_cost = cellCost(s.getX(), s.getY(), b);
    double d_cost = cellCost(d.getX(), d.getY(), b);
    double s_risk_cost = riskCost(s.getX(), s.getY(), b);
    double d_risk_cost = riskCost(d.getX(), d.getY(), b);
    double flowcost = computeCrowdFlow(s,d);
    double ns_cost = novelCost(s.getX(), s.getY());
    double nd_cost = novelCost(d.getX(), d.getY());
    double smooth_cost = (oldcost * 5);
    double sconveycost = computeConveyorCost(s.getX(), s.getY());
    double dconveycost = computeConveyorCost(d.getX(), d.getY());
    if(direction == true){
      flowcost = flowcost * (-1);
    }
    if(flowcost < 0){
      flowcost = 0;
    }
    return (w1 * oldcost) + (w2 * (s_cost+d_cost)/2) + (w3 * flowcost) + (w4 * (s_risk_cost+d_risk_cost)/2) + (w5 * (ns_cost+nd_cost)/2) + (w6 * smooth_cost) + (w7 * oldcost*pow(0.25,((sconveycost + dconveycost)/2)));*/
    int sRegion=-1,dRegion=-1;
    for(int i = 0; i < regions.size() ; i++){
      if(regions[i].inRegion(s.getX()/100.0, s.getY()/100.0)){
        sRegion = i;
      }
      if(regions[i].inRegion(d.getX()/100.0, d.getY()/100.0)){
        dRegion = i;
      }
      if(sRegion >= 0 and dRegion >= 0){
        break;
      }
    }

    double s_door_min_distance = std::numeric_limits<double>::infinity();
    double s_exit_min_distance = std::numeric_limits<double>::infinity();
    if(sRegion >= 0){
      CartesianPoint sPoint = CartesianPoint(s.getX()/100.0, s.getY()/100.0);
      for(int i = 0; i < doors[sRegion].size(); i++) {
        double doorDistance = doors[sRegion][i].distanceToDoor(sPoint, regions[sRegion]);
        if (doorDistance < s_door_min_distance){
          s_door_min_distance = doorDistance;
        }
      }
      vector<FORRExit> exits = regions[sRegion].getExits();
      for(int i = 0 ; i < exits.size(); i++){
        double exitDistance = sPoint.get_distance(CartesianPoint(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()));
        if (exitDistance < s_exit_min_distance){
          s_exit_min_distance = exitDistance;
        }
      }
    }

    double d_door_min_distance = std::numeric_limits<double>::infinity();
    double d_exit_min_distance = std::numeric_limits<double>::infinity();
    if(dRegion >= 0){
      CartesianPoint dPoint = CartesianPoint(d.getX()/100.0, d.getY()/100.0);
      for(int i = 0; i < doors[dRegion].size(); i++) {
        double doorDistance = doors[dRegion][i].distanceToDoor(dPoint, regions[dRegion]);
        if (doorDistance < d_door_min_distance){
          d_door_min_distance = doorDistance;
        }
      }
      vector<FORRExit> exits = regions[dRegion].getExits();
      for(int i = 0 ; i < exits.size(); i++){
        double exitDistance = dPoint.get_distance(CartesianPoint(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()));
        if (exitDistance < d_exit_min_distance){
          d_exit_min_distance = exitDistance;
        }
      }
    }
    double sHallway=0, dHallway=0;
    for(int i = 0; i < hallways.size(); i++){
      if(hallways[i].pointInAggregate(CartesianPoint(s.getX()/100.0, s.getY()/100.0))){
        sHallway++;
      }
      if(hallways[i].pointInAggregate(CartesianPoint(d.getX()/100.0, d.getY()/100.0))){
        dHallway++;
      }
    }
    double sconveycost = computeConveyorCost(s.getX(), s.getY());
    double dconveycost = computeConveyorCost(d.getX(), d.getY());
    //return (w7 * oldcost*pow(0.25,((sconveycost + dconveycost)/2)));
    double strailcount = 0;
    double dtrailcount = 0;
    //cout << "trails.size() = " << trails.size() << endl;
    for(int i = 0; i < trails.size(); i++){
      //cout << "trails[i].size() = " << trails[i].size() << endl;
      for(int j = 0; j < trails[i].size(); j++){
        if(trails[i][j].get_distance(CartesianPoint(s.getX()/100.0, s.getY()/100.0)) <= 0.5){
          strailcount++;
          //cout << "trails[i][j] = " << trails[i][j].get_x() << ", " << trails[i][j].get_y() << endl;
          //cout << "s = " << s.getX()/100.0 << ", " << s.getY()/100.0 << endl;
        }
        if(trails[i][j].get_distance(CartesianPoint(d.getX()/100.0, d.getY()/100.0)) <= 0.5){
          dtrailcount++;
          //cout << "trails[i][j] = " << trails[i][j].get_x() << ", " << trails[i][j].get_y() << endl;
          //cout << "d = " << d.getX()/100.0 << ", " << d.getY()/100.0 << endl;
        }
      }
    }
    //cout << "strailcount = " << strailcount << " dtrailcount = " << dtrailcount << endl;
    //return (w8 * oldcost*pow(0.25,((strailcount + dtrailcount)/2)));
    double finalcost = (w1 * oldcost) * 10;
    if (sRegion >= 0 and dRegion >= 0){
      if ((w1 * oldcost) * 0.25 < finalcost){
        finalcost = (w1 * oldcost) * 0.25;
      }
    }
    else if (sRegion >= 0 and dRegion == -1){
      if(s_door_min_distance <= 0.5 and s_exit_min_distance <= 0.5){
        if ((w1 * oldcost) * 0.5 < finalcost){
          finalcost = (w1 * oldcost) * 0.5;
        }
      }
      else if(s_door_min_distance <= 0.5 or s_exit_min_distance <= 0.5){
        if ((w1 * oldcost) * 0.75 < finalcost){
          finalcost = (w1 * oldcost) * 0.75;
        }
      }
      else if ((w1 * oldcost) * 1 < finalcost){
        finalcost = (w1 * oldcost) * 1;
      }
    }
    else if (sRegion ==-1 and dRegion >= 0){
      if(d_door_min_distance <= 0.5 and d_exit_min_distance <= 0.5){
        if ((w1 * oldcost) * 0.5 < finalcost){
          finalcost = (w1 * oldcost) * 0.5;
        }
      }
      else if(d_door_min_distance <= 0.5 or d_exit_min_distance <= 0.5){
        if ((w1 * oldcost) * 0.75 < finalcost){
          finalcost = (w1 * oldcost) * 0.75;
        }
      }
      else if ((w1 * oldcost) * 1 < finalcost){
        finalcost = (w1 * oldcost) * 1;
      }
    }
    if (sHallway > 0 and dHallway > 0){
      if ((w1 * oldcost) * 1/((sHallway+dHallway)/2) < finalcost){
        finalcost = (w1 * oldcost) * 1/((sHallway+dHallway)/2);
      }
    }
    if (sconveycost > 0 and dconveycost > 0){
      if ((w7 * oldcost * 1/((sconveycost + dconveycost)/2)) < finalcost){
        finalcost = (w7 * oldcost * 1/((sconveycost + dconveycost)/2));
      }
    }
    if (strailcount > 0 and dtrailcount > 0){
      if ((w7 * oldcost * 1/((strailcount + dtrailcount)/2)) < finalcost){
        finalcost = (w7 * oldcost * 1/((strailcount + dtrailcount)/2));
      }
    }
    return finalcost;
  }

  //double newEdgeCost = (oldcost * flowcost);
  //double newEdgeCost = (w1 * oldcost) + (w2 * (s_cost+d_cost)/2) + (w3 * flowcost) + (w4 * (s_risk_cost+d_risk_cost)/2);

	//cout << "Flow cost --------- " << endl;
  /*if (s_cost > 0 or d_cost > 0 or flowcost > 0 or s_risk_cost > 0 or d_risk_cost > 0){
    cout << "Dist cost    :" << oldcost << endl;
  	cout << "Node penalty : " << s_cost << " + " << d_cost << endl;
    cout << "Flow cost    :" << flowcost << endl;
    cout << "Risk penalty : " << s_risk_cost << " + " << d_risk_cost << endl;
  	//cout << "Old cost : " << oldcost << " new cost : " << newEdgeCost << std::endl;
    cout << "New cost    :" << newEdgeCost << endl;
  }*/
	//return newEdgeCost;
}


double PathPlanner::cellCost(int nodex, int nodey, int buffer){
	int x = (int)((nodex/100.0)/crowdModel.resolution);
	int x1 = (int)(((nodex+buffer)/100.0)/crowdModel.resolution);
	int x2 = (int)(((nodex-buffer)/100.0)/crowdModel.resolution);
	int y = (int)((nodey/100.0)/crowdModel.resolution);
	int y1 = (int)(((nodey+buffer)/100.0)/crowdModel.resolution);
	int y2 = (int)(((nodey-buffer)/100.0)/crowdModel.resolution);

	//std::cout << "x " << x << " y " << y;
	double d = crowdModel.densities[(y * crowdModel.width) + x];
	double d1 = crowdModel.densities[(y1 * crowdModel.width) + x];
	double d2 = crowdModel.densities[(y2 * crowdModel.width) + x];
	double d3 = crowdModel.densities[(y * crowdModel.width) + x1];
	double d4 = crowdModel.densities[(y * crowdModel.width) + x2];
	//std::cout << " Cell cost " << d << std::endl;
	//return (d + d1 + d2 + d3 + d4)/5;
	double da = std::max(std::max(d, d1),d2);
	double db = std::max(d3, d4);
	return std::max(da,db);
	//return d;
}


double PathPlanner::riskCost(int nodex, int nodey, int buffer){
  int x = (int)((nodex/100.0)/crowdModel.resolution);
  int x1 = (int)(((nodex+buffer)/100.0)/crowdModel.resolution);
  int x2 = (int)(((nodex-buffer)/100.0)/crowdModel.resolution);
  int y = (int)((nodey/100.0)/crowdModel.resolution);
  int y1 = (int)(((nodey+buffer)/100.0)/crowdModel.resolution);
  int y2 = (int)(((nodey-buffer)/100.0)/crowdModel.resolution);

  //std::cout << "x " << x << " y " << y;
  double d = crowdModel.risk[(y * crowdModel.width) + x];
  double d1 = crowdModel.risk[(y1 * crowdModel.width) + x];
  double d2 = crowdModel.risk[(y2 * crowdModel.width) + x];
  double d3 = crowdModel.risk[(y * crowdModel.width) + x1];
  double d4 = crowdModel.risk[(y * crowdModel.width) + x2];
  //std::cout << " Cell cost " << d << std::endl;
  //return (d + d1 + d2 + d3 + d4)/5;
  double da = std::max(std::max(d, d1),d2);
  double db = std::max(d3, d4);
  return std::max(da,db);
  //return d;
}

// Projection of crowd flow vectors on vector at s and d and then take the average
double PathPlanner::computeCrowdFlow(Node s, Node d){
	int s_x_index = (int)((s.getX()/100.0)/crowdModel.resolution);
	int s_y_index = (int)((s.getY()/100.0)/crowdModel.resolution);
	int d_x_index = (int)((d.getX()/100.0)/crowdModel.resolution);
	int d_y_index = (int)((d.getY()/100.0)/crowdModel.resolution);
	//Assuming crowd densities are normalized between 0 and 1
	double s_l = crowdModel.left[(s_y_index * crowdModel.width) + s_x_index];
	double d_l = crowdModel.left[(d_y_index * crowdModel.width) + d_x_index];
	double s_r = crowdModel.right[(s_y_index * crowdModel.width) + s_x_index];
	double d_r = crowdModel.right[(d_y_index * crowdModel.width) + d_x_index];
	double s_u = crowdModel.up[(s_y_index * crowdModel.width) + s_x_index];
	double d_u = crowdModel.up[(d_y_index * crowdModel.width) + d_x_index];
	double s_d = crowdModel.down[(s_y_index * crowdModel.width) + s_x_index];
	double d_d = crowdModel.down[(d_y_index * crowdModel.width) + d_x_index];

	double s_ul = crowdModel.up_left[(s_y_index * crowdModel.width) + s_x_index];
	double d_ul = crowdModel.up_left[(d_y_index * crowdModel.width) + d_x_index];
	double s_ur = crowdModel.up_right[(s_y_index * crowdModel.width) + s_x_index];
	double d_ur = crowdModel.up_right[(d_y_index * crowdModel.width) + d_x_index];
	double s_dl = crowdModel.down_left[(s_y_index * crowdModel.width) + s_x_index];
	double d_dl = crowdModel.down_left[(d_y_index * crowdModel.width) + d_x_index];
	double s_dr = crowdModel.down_right[(s_y_index * crowdModel.width) + s_x_index];
	double d_dr = crowdModel.down_right[(d_y_index * crowdModel.width) + d_x_index];

	//cout << "Left : " << d_l << " * " << s_l << endl;
	//cout << "Right : " << d_r << " * " << s_r << endl;
	//cout << "Up : " << d_u << " * " << s_u << endl;
	//cout << "Down : " << d_d << " * " << s_d << endl;
	//cout << "Up-right : " << d_ur << " * " << s_ur << endl;
	//cout << "Up-left : " << d_ul << " * " << s_ul << endl;
	//cout << "Down-right : " << d_dr << " * " << s_dr << endl;
	//cout << "Down-left : " << d_dl << " * " << s_dl << endl;


	double l_avg = (s_l + d_l) / 2;
	double r_avg = (s_r + d_r) / 2;
	double u_avg = (s_u + d_u) / 2;
	double d_avg = (s_d + d_d) / 2;
	double ul_avg = (s_ul + d_ul) / 2;
	double dl_avg = (s_dl + d_dl) / 2;
	double ur_avg = (s_ur + d_ur) / 2;
	double dr_avg = (s_dr + d_dr) / 2;

	double pi = 3.145;
	double cost_u = projection(pi/2, u_avg, s.getX(), s.getY(), d.getX(), d.getY());
	double cost_d = projection(3*pi/2, d_avg, s.getX(), s.getY(), d.getX(), d.getY());
	double cost_r = projection(0, r_avg, s.getX(), s.getY(), d.getX(), d.getY());
	double cost_l = projection(pi, l_avg, s.getX(), s.getY(), d.getX(), d.getY());

	double cost_ur = projection(pi/4, ur_avg, s.getX(), s.getY(), d.getX(), d.getY());
	double cost_ul = projection(3*pi/4, ul_avg, s.getX(), s.getY(), d.getX(), d.getY());
	double cost_dr = projection(7*pi/4, dr_avg, s.getX(), s.getY(), d.getX(), d.getY());
	double cost_dl = projection(5*pi/4, dl_avg, s.getX(), s.getY(), d.getX(), d.getY());

	double cost = cost_u + cost_d + cost_r + cost_l + cost_ur + cost_ul + cost_dr + cost_dl;
	return cost;
}

double PathPlanner::novelCost(int nodex, int nodey){
  //cout << "Inside novelCost : Node x = " << (nodex/100.0) << " Node y = " << (nodey/100.0) << endl;
  //cout << "Modified Node x = " << (int)(((nodex/100.0)/(map_width*1.0)) * boxes_width) << " Modified Node y = " << (int)(((nodey/100.0)/(map_height*1.0)) * boxes_height) << endl;
  //cout << "novelCost = " << posHistMapNorm[(int)(((nodex/100.0)/(map_width*1.0)) * boxes_width)][(int)(((nodey/100.0)/(map_height*1.0)) * boxes_height)] << endl;
  return posHistMapNorm[(int)(((nodex/100.0)/(map_width*1.0)) * boxes_width)][(int)(((nodey/100.0)/(map_height*1.0)) * boxes_height)];
}

double PathPlanner::computeConveyorCost(int nodex, int nodey){
  //cout << "Inside computeConveyorCost : Node x = " << (nodex/100.0) << " Node y = " << (nodey/100.0) << endl;
  //cout << "ConveyorCost = " << conveyors->getGridValue((nodex/100.0),(nodey/100.0)) << endl;
  return conveyors->getGridValue((nodex/100.0),(nodey/100.0));
}


double PathPlanner::projection(double flow_angle, double flow_length, double xs, double ys, double xd, double yd){
	//double pi = 3.145;
	double edge_angle = atan2((ys - yd),(xs - xd))+M_PI;
	//edge_angle = (edge_angle > 0 ? edge_angle : (2*pi + edge_angle));
  edge_angle = (edge_angle < (2*M_PI) ? edge_angle : (0.0));
	double theta = flow_angle - edge_angle;
  if(theta>M_PI){
    theta = theta-2*M_PI;
  }
  else if(theta<-M_PI){
    theta = theta+2*M_PI;
  }
	//cout << "Flow angle: " << flow_angle << " Edge angle: " << edge_angle << " Diff: " << theta << endl;
	return flow_length * cos(theta);
}




bool PathPlanner::isAccessible(Node s, Node t) {
  if ( !navGraph->isNode(s) )
    s = getClosestNode(s, t);

  if ( !navGraph->isNode(t) )
    t = getClosestNode(t, s);

  if ( s.getID() == Node::invalid_node_index || t.getID() == Node::invalid_node_index )
    return false;

  astar newsearch(*navGraph, s, t, name);
  if ( newsearch.isPathFound() )
    return true;

  return false;
}


list<pair<int,int> > PathPlanner::getPathXYBetween(int x1, int y1, int x2, int y2){

  // flags show if the (x1, y1) and (x2, y2) are not valid nodes in the navgraph
  bool s_invalid = false;
  bool t_invalid = false;

  // flags represent what happens after attempting to get the closest valid node to (x1, y1) and (x2, y2)
  bool no_source = false;
  bool no_target = false;

  // create temp nodes for pairs
  Node temp_s(1, x1, y1);
  Node temp_t(1, x2, y2);

  /* check if the (x1, y1) is a valid node in the graph, if so get a copy of the node and assign it to s
   * else attempt to get the closest valid node from the graph, if it fails set the no_source flag to true
   * if it succeeds assign the closest valid node to s and set the s_invalid flag to true
   */
  Node s;
  int s_id = navGraph->getNodeID(x1, y1);
  if(s_id == Node::invalid_node_index) {
    s = getClosestNode(temp_s, temp_t);

    if(!navGraph->isNode(s))
      no_source = true;

    s_invalid = true;
  }
  else {
    s = navGraph->getNode(s_id);
  }

  /* check if the (x2, y2) is a valid node in the graph, if so get a copy of the node and assign it to t
   * else attempt to get the closest valid node from the graph, if it fails set the no_target flag to true
   * if it succeeds assign the closest valid node to t and set the t_invalid flag to true
   */
  Node t;
  int t_id = navGraph->getNodeID(x2, y2);
  if(t_id == Node::invalid_node_index) {
    t = getClosestNode(temp_t, temp_s);

    if(!navGraph->isNode(t))
      no_target = true;

    t_invalid = true;
  }

  // computed path list containing node ids
  list<int> path_c;

  // computed path list containing (x,y) values of the nodes
  list< pair<int,int> > path_c_points;

  /* if either source or the target is invalid, astar can't find a path.
   * to inform the caller of this function if this situation arises, a special pair <source, target>
   * will be added to the path and the function will return. if any of the values in the pair is -1
   * it will mean the path wasn't found due to invalid source, target or both
   */
  if(no_source || no_target) {
    int sval = (no_source) ? -1 : 0;
    int tval = (no_target) ? -1 : 0;
    pair<int,int> p(sval, tval);
    path_c_points.push_back(p);

    return path_c_points;
  }

  // calculate the path. if no path is found return the path_points list empty
  astar newsearch(*navGraph, s, t, name);
  if(newsearch.isPathFound()) {
    path_c = newsearch.getPathToTarget();
    smoothPath(path_c, s, t);
  }
  else {
    return path_c_points;
  }

  // compute the distance
  if(s_invalid) {
    pair<int, int> st(x1,y1);
    path_c_points.push_back(st);
  }

  list<int>::iterator iter;
  for ( iter = path_c.begin(); iter != path_c.end(); iter++ ){
    Node d = navGraph->getNode(*iter);
    pair<int, int> p(d.getX(), d.getY());
    path_c_points.push_back(p);
  }

  if(t_invalid) {
    pair<int, int> tg(x2,y2);
    path_c_points.push_back(tg);
  }

  return path_c_points;
}


int PathPlanner::getPathLength(list<pair<int,int> > path){
  double length = 0;
  list<pair<int,int> >::iterator iter, iter_next;
  for ( iter = path.begin(); iter != path.end(); iter++ ){
    iter_next = iter;
    iter_next++;
    if ( iter_next != path.end() ) {
      length += Map::distance(iter->first, iter->second, iter_next->first, iter_next->second);
    }
  }
  return static_cast<int>(length);
}


double PathPlanner::calcPathCost(list<int> p){
  double pcost = 0;
  list<int>::iterator iter;
  int first;
  Edge * e;

  for( iter = p.begin(); iter != p.end() ; iter++ ){
    first = *iter++;
    if (iter != p.end()){
      e = navGraph->getEdge(first, *iter);
      pcost += e->getCost(true);
    }
    iter--;
  }
  // add reaching from source and to target costs
  // Note: This will double count when called from estimateCost() and calcPath()

  /*if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){
    if ( !p.empty() ){
      pcost += Map::distance(source.getX(), source.getY(),
					     navGraph->getNode(p.front()).getX(),
					     navGraph->getNode(p.front()).getY());
      pcost += Map::distance(navGraph->getNode(p.back()).getX(),
					     navGraph->getNode(p.back()).getY(),
					     target.getX(), target.getY());
    }
    else
      pcost += Map::distance(source.getX(), source.getY(),
					     target.getX(), target.getY());
  }*/

  return pcost;
}

double PathPlanner::calcOrigPathCost(list<int> p){
  double pcost = 0;
  list<int>::iterator iter;
  int first;
  Edge * e;

  for( iter = p.begin(); iter != p.end() ; iter++ ){
    first = *iter++;
    if (iter != p.end()){
      e = originalNavGraph->getEdge(first, *iter);
      pcost += e->getCost(true);
    }
    iter--;
  }
  // add reaching from source and to target costs
  // Note: This will double count when called from estimateCost() and calcPath()

  /*if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){
    if ( !p.empty() ){
      pcost += Map::distance(source.getX(), source.getY(),
               navGraph->getNode(p.front()).getX(),
               navGraph->getNode(p.front()).getY());
      pcost += Map::distance(navGraph->getNode(p.back()).getX(),
               navGraph->getNode(p.back()).getY(),
               target.getX(), target.getY());
    }
    else
      pcost += Map::distance(source.getX(), source.getY(),
               target.getX(), target.getY());
  }*/

  return pcost;
}

double PathPlanner::calcPathCost(vector<CartesianPoint> waypoints, Position source, Position target){
  double pcost = 0;
  list<int> p;

  vector<CartesianPoint>::iterator it;
  for (it = waypoints.begin(); it != waypoints.end(); it++ ){
    Node s;
    int s_id = navGraph->getNodeID((*it).get_x()*100.0, (*it).get_y()*100.0);
    if(s_id == Node::invalid_node_index) {
      Node temp_s(1, (*it).get_x()*100.0, (*it).get_y()*100.0); 
      Node temp_t(1, (*it).get_x()*100.0, (*it).get_y()*100.0);
      s = getClosestNode(temp_s, temp_t);
      s_id = navGraph->getNodeID(s.getX(), s.getY());
    }
    p.push_back(s_id);
  }

  list<int>::iterator iter;
  int first;
  Edge * e;

  for( iter = p.begin(); iter != p.end() ; iter++ ){
    first = *iter++;
    if (iter != p.end()){
      e = navGraph->getEdge(first, *iter);
      pcost += e->getCost(true);
    }
    iter--;
  }
  
  pcost += (estimateCost(source.getX(), source.getY(), waypoints[0].get_x(), waypoints[0].get_y()) + estimateCost(target.getX(), target.getY(), waypoints[waypoints.size()-1].get_x(), waypoints[waypoints.size()-1].get_y()));

  return pcost;
}

double PathPlanner::estimateCost(int x1, int y1, int x2, int y2) {
  Node source(1, x1, y1);
  Node target(1, x2, y2);

  return estimateCost(source, target, 1);
}

double PathPlanner::estimateCost(Node s, Node t, int l){
  Node sn = getClosestNode(s, t);
  Node tn = getClosestNode(t, s);

  if(tn.getID() < 0 || sn.getID() < 0)
    return INT_MAX;

  double pathCost = Map::distance(s.getX(), s.getY(), sn.getX(), sn.getY());
  pathCost += Map::distance(t.getX(), t.getY(), tn.getX(), tn.getY());

  astar nsearch(*navGraph, sn, tn, name);
  if ( nsearch.isPathFound() ){
    list<int> p = nsearch.getPathToTarget();
    pathCost += calcPathCost(p);
  }
  else {
    if(PATH_DEBUG)
      cout << "PathPlanner::estimateCost> no path found to target! "
	   << "Source accessible: " << sn.isAccessible()
	   << ", target accessible: " << tn.isAccessible() << endl;
    pathCost = INT_MAX;
  }

  return pathCost;
}

double PathPlanner::getRemainingPathLength(double x, double y) {
  list<pair<int,int> > path_xy;

  pair<int,int> s(x, y);
  path_xy.push_back(s);

  list<int>::iterator iter;
  for(iter = path.begin(); iter != path.end(); iter++) {
    Node d = navGraph->getNode(*iter);
    pair<int, int> p(d.getX(), d.getY());
    path_xy.push_back(p);
  }

  pair<int,int> t(target.getX(), target.getY());
  path_xy.push_back(t);

  return getPathLength(path_xy);
}


/*!
  \brief Returns the closest Node in the navigation graph to an arbirary \f$(x, y)\f$ position.

  \param Node \c n, any \f$(x, y)\f$ on the map which we are searching for the closest node
  \param Node \c ref, any \f$(x,y)\f$ on the map, which we are using as a guiding point
  \return Node, this is a member node of the navigation graph, or an invalid node if not found

  This function is used to determine the closest member nodes of the navigation graph to target and the source,
  since the A* runs only over the member nodes.

  It first asks for the nodes within a region from the navigation graph and returns an accessible node in
  that area, that has the minimum total distances from the itself to \c n and \c ref nodes.

  The radius of the search region is initially set to the \c proximity defined by the \c Graph class.
  If no accessible nodes are found, the radius of the search area is increased and the search is repeated.
  The maximum search radius is set to 1.5 * proximity. If there are no accessible nodes within that distance
  the robot is probably surrounded by obstacles, therefore this function returns an invalid node.

 */
Node PathPlanner::getClosestNode(Node n, Node ref){
  const string signature = "PathPlanner::getClosestNode()> ";

  Node temp;
  double s_radius = navGraph->getProximity();
  double max_radius = navGraph->getProximity() * 1.5;

  do {

    if(PATH_DEBUG)
      cout << signature << "Searching for the closest node within " << s_radius << endl;

    vector<Node*> nodes = navGraph->getNodesInRegion(n.getX(), n.getY(), s_radius);

    double dist = INT_MAX;

    vector<Node*>::iterator iter;
    for( iter = nodes.begin(); iter != nodes.end(); iter++ ){
      double d = Map::distance( (*iter)->getX(), (*iter)->getY(), n.getX(), n.getY() );

      if(PATH_DEBUG){
        cout << "\tChecking ";
        (*iter)->printNode();
        cout << endl;
        cout << "\tDistance between the n and this node: " << d << endl;
      }

      double d_t = 0.0;
      if(ref.getID() != Node::invalid_node_index)
        d_t = Map::distance((*iter)->getX(), (*iter)->getY(), ref.getX(), ref.getY());

      if(PATH_DEBUG)
        cout << "\tDistance between this node to ref: " << d_t << endl;

      if(name != "skeleton"){
        if (( d + d_t < dist ) && !map.isPathObstructed( (*iter)->getX(), (*iter)->getY(), n.getX(), n.getY()) && (*iter)->isAccessible()) {
          //cout << "Checking if node is accesible : " << (*iter)->getX() << " " << (*iter)->getY()  << endl;
          dist = d + d_t;
          temp = (*(*iter));
          if(PATH_DEBUG) {
            cout << "\tFound a new candidate!: ";
            temp.printNode();
            cout << endl << endl;
          }
        }
      }
      else{
        if (( d + d_t < dist ) && (*iter)->isAccessible()) {
          //cout << "Checking if node is accesible : " << (*iter)->getX() << " " << (*iter)->getY()  << endl;
          dist = d + d_t;
          temp = (*(*iter));
          if(PATH_DEBUG) {
            cout << "\tFound a new candidate!: ";
            temp.printNode();
            cout << endl << endl;
          }
        }
      }
    }

    if(temp.getID() == Node::invalid_node_index) {
      s_radius += 0.1 * s_radius;
      if(PATH_DEBUG)
        cout << signature << "Didn't find a suitable candidate. Increasing search radius to: " << s_radius << endl;
    }

  } while(temp.getID() == Node::invalid_node_index && s_radius <= max_radius);

  return temp;
}

/*!
  \brief Used to remove extra points off of the path
 */
void PathPlanner::smoothPath(list<int>& pathCalc, Node s, Node t){
  int proximity = navGraph->getProximity();

  if ( pathCalc.size() > 1 ) {
    // smooth the end points
    // if getting to second node from source is shorter and not obstructed remove first node.
    list<int>::iterator iter = pathCalc.begin();
    Node first = navGraph->getNode(*iter++);
    Node second = navGraph->getNode(*iter);
    iter--; // point back to the first element

    if ( PATH_DEBUG ) {
      cout << "source: ";
      s.printNode();
      cout << " - first: " ;
      first.printNode();
      cout << " - second: " ;
      second.printNode();
      cout << endl ;
    }

    if ( !map.isPathObstructed(s.getX(), s.getY(), second.getX(), second.getY()) &&
	 Map::distance( s.getX(), s.getY(), second.getX(), second.getY() ) + proximity * 0.5 <
	 ( Map::distance( s.getX(), s.getY(), first.getX(), first.getY() ) +
	   Map::distance( first.getX(), first.getY(), second.getX(), second.getY() ) )){
      if ( PATH_DEBUG ) cout << "Erasing first" << endl;
      pathCalc.erase(iter);
    }
  }

  if ( pathCalc.size() > 1 ) {
    // if getting to second node from source is shorter and not obstructed remove first node.
    list<int>::iterator iter = pathCalc.end();
    Node last = navGraph->getNode(*(--iter));
    Node onebeforelast = navGraph->getNode(*(--iter));
    iter++;

    if ( PATH_DEBUG ){
      cout << "onebeforelast: ";
      onebeforelast.printNode();
      cout << " - last: " ;
      last.printNode();
      cout << " - target: " ;
      t.printNode();
      cout << endl;
    }

    if ( !map.isPathObstructed(onebeforelast.getX(), onebeforelast.getY(), t.getX(), t.getY()) &&
	 Map::distance( onebeforelast.getX(), onebeforelast.getY(), t.getX(), t.getY() ) + proximity * 0.5 <
	 ( Map::distance( onebeforelast.getX(), onebeforelast.getY(), last.getX(), last.getY() ) +
	   Map::distance( last.getX(), last.getY(), t.getX(), t.getY() ) )){
      if ( PATH_DEBUG ) cout << "Erasing last" << endl ;
      pathCalc.erase(iter);
    }
  }

  if ( PATH_DEBUG ) {
    cout << "after smoothing: " << endl;
    printPath(pathCalc);
  }
}

/*!
  \brief Prints the path node by node
 */
void PathPlanner::printPath(){
  list<int>::iterator it;
  for ( it = path.begin(); it != path.end(); it++ ){
    navGraph->getNode(*it).printNode() ;
    cout << endl;
  }
}

void PathPlanner::printPath(list<int> p){
  list<int>::iterator it ;
  for ( it = p.begin(); it != p.end(); it++ ){
    navGraph->getNode(*it).printNode() ;
    cout << endl;
  }
}


void PathPlanner::printPath(list<pair<int,int> > p) {
  list<pair<int,int> >::iterator it ;
  for ( it = p.begin(); it != p.end(); it++ ){
    int nodeId = navGraph->getNodeID(it->first, it->second);
    if ( nodeId == -1 ) {
      cout << "Not a graph node - (" << it->first << ", " << it->second << ")" << endl;
    }
    else {
      navGraph->getNode(nodeId).printNode() ;
      cout << endl;
    }
  }
}


bool PathPlanner::allWaypointsValid() {
  list<int>::iterator it;
  for(it = path.begin(); it != path.end(); ++it) {
    if(!navGraph->getNode(*it).isAccessible())
      return false;
  }
  return true;
}

/*! @} */
