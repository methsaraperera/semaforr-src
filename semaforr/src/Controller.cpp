/*
 * Controller.cpp
 *
 */
          
#include "Controller.h"
#include "FORRGeometry.h"
#include <unistd.h>

#include <deque>
#include <iostream> 
#include <fstream>
#include <math.h>
#include <time.h>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

#define CTRL_DEBUG true

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize advisors and weights and spatial learning modules based on the advisors
//
//
void Controller::initialize_advisors(string filename){
 
  string fileLine;
  string advisor_name, advisor_description;
  bool advisor_active;
  double advisor_weight = 1;
  double parameters[4];
  std::ifstream file(filename.c_str());
  ROS_DEBUG_STREAM("Reading read_advisor_file:" << filename);
  if(!file.is_open()){
    ROS_DEBUG("Unable to locate or read advisor config file!");
  }
  //read advisor names and parameters from the config file and create new advisor objects
  while(getline(file, fileLine)){
    if(fileLine[0] == '#')  // skip comment lines
      continue;
    else{
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      advisor_name = vstrings[0];
      advisor_description = vstrings[1];
      if(vstrings[2] == "t")
        advisor_active = true;
      else
        advisor_active = false;
        advisor_weight = atof(vstrings[3].c_str());
        parameters[0]= atof(vstrings[4].c_str());
        parameters[1] = atof(vstrings[5].c_str());
        parameters[2] = atof(vstrings[6].c_str());
        parameters[3] = atof(vstrings[7].c_str());
        tier3Advisors.push_back(Tier3Advisor::makeAdvisor(getBeliefs(), advisor_name, advisor_description, advisor_weight, parameters, advisor_active));
    }
  }
     
  ROS_DEBUG_STREAM("" << tier3Advisors.size() << " advisors registered.");
  for(unsigned i = 0; i < tier3Advisors.size(); ++i)
    ROS_DEBUG_STREAM("Created advisor " << tier3Advisors[i]->get_name() << " with weight: " << tier3Advisors[i]->get_weight());

  //CONVEYORS = isAdvisorActive("ConveyLinear");
  //REGIONS = isAdvisorActive("ExitLinear");
  //TRAILS = isAdvisorActive("TrailerLinear");
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize robot parameters
//
//
void Controller::initialize_params(string filename){
// robot intial position
// robot laser sensor range, span and increment
// robot action <-> semaFORR decision

  string fileLine;
  std::ifstream file(filename.c_str());
  ROS_DEBUG_STREAM("Reading params_file:" << filename);
  //cout << "Inside file in tasks " << endl;
  if(!file.is_open()){
    ROS_DEBUG("Unable to locate or read params config file!");
  }
  while(getline(file, fileLine)){
  //cout << "Inside while in tasks" << endl;
    if(fileLine[0] == '#')  // skip comment lines
      continue;
    else if (fileLine.find("decisionlimit") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      taskDecisionLimit = atoi(vstrings[1].c_str());
      ROS_DEBUG_STREAM("decisionlimit " << taskDecisionLimit);
    }
    else if (fileLine.find("canSeePointEpsilon") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      canSeePointEpsilon = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("canSeePointEpsilon " << canSeePointEpsilon);
    }
    else if (fileLine.find("laserScanRadianIncrement") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      laserScanRadianIncrement = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("laserScanRadianIncrement " << laserScanRadianIncrement);
    }
    else if (fileLine.find("robotFootPrint") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      robotFootPrint = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("robotFootPrint " << robotFootPrint);
    }
    else if (fileLine.find("bufferForRobot") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      robotFootPrintBuffer = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("bufferForRobot " << robotFootPrintBuffer);
    }
    else if (fileLine.find("maxLaserRange") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      maxLaserRange = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("maxLaserRange " << maxLaserRange);
    }
    else if (fileLine.find("maxForwardActionBuffer") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      maxForwardActionBuffer = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("maxForwardActionBuffer " << maxForwardActionBuffer);
    }
    else if (fileLine.find("maxForwardActionSweepAngle") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      maxForwardActionSweepAngle = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("maxForwardActionSweepAngle " << maxForwardActionSweepAngle);
    }
    else if (fileLine.find("planLimit") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      planLimit = atoi(vstrings[1].c_str());
      ROS_DEBUG_STREAM("planLimit " << planLimit);
    }
    else if (fileLine.find("trailsOn") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      trailsOn = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("trailsOn " << trailsOn);
    }
    else if (fileLine.find("conveyorsOn") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      conveyorsOn = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("conveyorsOn " << conveyorsOn);
    }
    else if (fileLine.find("regionsOn") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      regionsOn = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("regionsOn " << regionsOn);
    }
    else if (fileLine.find("doorsOn") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      doorsOn = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("doorsOn " << doorsOn);
    }
    else if (fileLine.find("hallwaysOn") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      hallwaysOn = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("hallwaysOn " << hallwaysOn);
    }
    else if (fileLine.find("barrsOn") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      barrsOn = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("barrsOn " << barrsOn);
    }
    else if (fileLine.find("situationsOn") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      situationsOn = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("situationsOn " << situationsOn);
    }
    else if (fileLine.find("aStarOn") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      aStarOn = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("aStarOn " << aStarOn);
    }
    else if (fileLine.find("move") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      moveArrMax = vstrings.size()-1;
      int arr_length = 0;
      for (int i=1; i<vstrings.size(); i++){
        if(arr_length < moveArrMax) {
          arrMove[arr_length++] = (atof(vstrings[i].c_str()));
        }
      }
      for (int i=0; i<moveArrMax; i++) {
        ROS_DEBUG_STREAM("arrMove " << arrMove[i]);
      }
    }
    else if (fileLine.find("rotate") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      rotateArrMax = vstrings.size()-1;
      int arr_length = 0;
      for (int i=1; i<vstrings.size(); i++){
        if(arr_length < rotateArrMax) {
          arrRotate[arr_length++] = (atof(vstrings[i].c_str()));
        }
      }
      for (int i=0; i<rotateArrMax; i++) {
        ROS_DEBUG_STREAM("arrRotate " << arrRotate[i]);
      }
    }
    else if (fileLine.find("distance") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      distance = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("distance " << distance);
    }
    else if (fileLine.find("smooth") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      smooth = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("smooth " << smooth);
    }
    else if (fileLine.find("novel") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      novel = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("novel " << novel);
    }
    else if (fileLine.find("density") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      density = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("density " << density);
    }
    else if (fileLine.find("risk") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      risk = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("risk " << risk);
    }
    else if (fileLine.find("flow") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      flow = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("flow " << flow);
    }
    else if (fileLine.find("combined") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      combined = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("combined " << combined);
    }
    else if (fileLine.find("CUSUM") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      CUSUM = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("CUSUM " << CUSUM);
    }
    else if (fileLine.find("discount") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      discount = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("discount " << discount);
    }
    else if (fileLine.find("explore") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      explore = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("explore " << explore);
    }
    else if (fileLine.find("spatial") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      spatial = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("spatial " << spatial);
    }
    else if (fileLine.find("hallwayer") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      hallwayer = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("hallwayer " << hallwayer);
    }
    else if (fileLine.find("trailer") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      trailer = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("trailer " << trailer);
    }
    else if (fileLine.find("barrier") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      barrier = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("barrier " << barrier);
    }
    else if (fileLine.find("conveys") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      conveys = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("conveys " << conveys);
    }
    else if (fileLine.find("turn") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      turn = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("turn " << turn);
    }
    else if (fileLine.find("skeleton") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      skeleton = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("skeleton " << skeleton);
    }
    else if (fileLine.find("hallwayskel") != std::string::npos) {
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      hallwayskel = atof(vstrings[1].c_str());
      ROS_DEBUG_STREAM("hallwayskel " << hallwayskel);
    }
  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the map file and intialize planner
//
//
void Controller::initialize_planner(string map_config, string map_dimensions, int &l, int &h){
  string fileLine;
  double p;
  std::ifstream file(map_dimensions.c_str());
  ROS_DEBUG_STREAM("Reading map dimension file:" << map_dimensions);
  if(!file.is_open()){
    ROS_DEBUG("Unable to locate or read map dimensions file!");
  }
  while(getline(file, fileLine)){
    //cout << "Inside while in tasks" << endl;
    if(fileLine[0] == '#')  // skip comment lines
      continue;
    else{
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      ROS_DEBUG("Unable to locate or read map dimensions file!");
      l = atoi(vstrings[0].c_str());
      h = atoi(vstrings[1].c_str());
      p = atof(vstrings[2].c_str());
      ROS_DEBUG_STREAM("Map dim:" << l << " " << h << " " << p << endl);
    }
  }	
  Map *map = new Map(l*100, h*100);
  map->readMapFromXML(map_config);
  cout << "Finished reading map"<< endl;
  Graph *origNavGraph = new Graph(map,(int)(p*100.0));
  //Graph *navGraph = new Graph(map,(int)(p*100.0));
  //cout << "initialized nav graph" << endl;
  //navGraph->printGraph();
  //navGraph->outputGraph();
  Node n;
  if(distance == 1){
    Graph *navGraphDistance = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphDistance, *map, n,n, "distance");
    //tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: distance");
  }
  if(smooth == 1){
    Graph *navGraphSmooth = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphSmooth, *map, n,n, "smooth");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: smooth");
  }
  if(novel == 1){
    Graph *navGraphNovel = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphNovel, *map, n,n, "novel");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: novel");
  }
  if(density == 1){
    Graph *navGraphDensity = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphDensity, *map, n,n, "density");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: density");
  }
  if(risk == 1){
    Graph *navGraphRisk = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphRisk, *map, n,n, "risk");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: risk");
  }
  if(flow == 1){
    Graph *navGraphFlow = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphFlow, *map, n,n, "flow");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: flow");
  }
  if(combined == 1){
    Graph *navGraphCombined = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphCombined, *map, n,n, "combined");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: combined");
  }
  if(CUSUM == 1){
    Graph *navGraphCUSUM = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphCUSUM, *map, n,n, "CUSUM");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: CUSUM");
  }
  if(discount == 1){
    Graph *navGraphDiscount = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphDiscount, *map, n,n, "discount");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: discount");
  }
  if(explore == 1){
    Graph *navGraphExplore = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphExplore, *map, n,n, "explore");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: explore");
  }
  if(spatial == 1){
    Graph *navGraphSpatial = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphSpatial, *map, n,n, "spatial");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: spatial");
  }
  if(hallwayer == 1){
    Graph *navGraphHallwayer = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphHallwayer, *map, n,n, "hallwayer");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: hallwayer");
  }
  if(trailer == 1){
    Graph *navGraphTrailer = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphTrailer, *map, n,n, "trailer");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: trailer");
  }
  if(barrier == 1){
    Graph *navGraphBarrier = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphBarrier, *map, n,n, "barrier");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: barrier");
  }
  if(conveys == 1){
    Graph *navGraphConveys = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphConveys, *map, n,n, "conveys");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: conveys");
  }
  if(turn == 1){
    Graph *navGraphTurn = new Graph(map,(int)(p*100.0));
    cout << "initialized nav graph" << endl;
    planner = new PathPlanner(navGraphTurn, *map, n,n, "turn");
    tier2Planners.push_back(planner);
    planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: turn");
  }
  if(skeleton == 1){
    Graph *navGraphSkeleton = new Graph((int)(p*1000.0*2), l*100, h*100);
    cout << "initialized nav graph" << endl;
    PathPlanner *sk_planner = new PathPlanner(navGraphSkeleton, n,n, "skeleton");
    tier2Planners.push_back(sk_planner);
    sk_planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: skeleton");
  }
  if(hallwayskel == 1){
    Graph *navGraphHallwaySkeleton = new Graph((int)(p*1000.0), l*100, h*100);
    cout << "initialized nav graph" << endl;
    PathPlanner *hwsk_planner = new PathPlanner(navGraphHallwaySkeleton, n,n, "hallwayskel");
    tier2Planners.push_back(hwsk_planner);
    hwsk_planner->setOriginalNavGraph(origNavGraph);
    ROS_DEBUG_STREAM("Created planner: hallwayskel");
  }
  cout << "initialized planners" << endl;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize tasks
//
//
void Controller::initialize_tasks(string filename){
  string fileLine;
  std::ifstream file(filename.c_str());
  ROS_DEBUG_STREAM("Reading read_task_file:" << filename);
  //cout << "Inside file in tasks " << endl;
  if(!file.is_open()){
    ROS_DEBUG("Unable to locate or read task config file!");
  }
  while(getline(file, fileLine)){
    //cout << "Inside while in tasks" << endl;
    if(fileLine[0] == '#')  // skip comment lines
      continue;
    else{
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      double x = atof(vstrings[0].c_str());
      double y = atof(vstrings[1].c_str());
      beliefs->getAgentState()->addTask(x,y);
      ROS_DEBUG_STREAM("Task: " << x << " " << y << endl);
    }
  }
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Read from the config file and intialize tasks
//
//
void Controller::initialize_situations(string filename){
  string fileLine;
  std::ifstream file(filename.c_str());
  ROS_DEBUG_STREAM("Reading read_situation_file:" << filename);
  //cout << "Inside file in tasks " << endl;
  if(!file.is_open()){
    ROS_DEBUG("Unable to locate or read situation config file!");
  }
  while(getline(file, fileLine)){
    //cout << "Inside while in tasks" << endl;
    if(fileLine[0] == '#')  // skip comment lines
      continue;
    else{
      std::stringstream ss(fileLine);
      std::istream_iterator<std::string> begin(ss);
      std::istream_iterator<std::string> end;
      std::vector<std::string> vstrings(begin, end);
      int count = atoi(vstrings[0].c_str());
      vector<float> values;
      for (int i=1; i<vstrings.size(); i++){
        values.push_back(atof(vstrings[i].c_str()));
      }
      beliefs->getSpatialModel()->getSituations()->createSituations(count, values);
      ROS_DEBUG_STREAM("Situation: " << count << endl);
    }
  }
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Initialize the controller and setup messaging to ROS
//
//
Controller::Controller(string advisor_config, string params_config, string map_config, string target_set, string map_dimensions, string situation_config){

  // Initialize robot parameters from a config file
  initialize_params(params_config);
  
  // Initialize planner and map dimensions
  int l,h;
  initialize_planner(map_config,map_dimensions,l,h);
  
  // Initialize the agent's 'beliefs' of the world state with the map and nav
  // graph and spatial models
  beliefs = new Beliefs(l, h, 2, arrMove, arrRotate, moveArrMax, rotateArrMax); // Hunter Fourth
  
  // Initialize advisors and weights from config file
  initialize_advisors(advisor_config);

  // Initialize the tasks from a config file
  initialize_tasks(target_set);

  // Initialize parameters
  beliefs->getAgentState()->setAgentStateParameters(canSeePointEpsilon, laserScanRadianIncrement, robotFootPrint, robotFootPrintBuffer, maxLaserRange, maxForwardActionBuffer, maxForwardActionSweepAngle);
  tier1 = new Tier1Advisor(beliefs);
  firstTaskAssigned = false;
  decisionStats = new FORRActionStats();

  // Initialize situations
  initialize_situations(situation_config);
  highwayFinished = 0;
  highwayExploration = new HighwayExplorer(l, h, 7, arrMove, arrRotate, moveArrMax, rotateArrMax);
}


// Function which takes sensor inputs and updates it for semaforr to use for decision making, and updates task status
void Controller::updateState(Position current, sensor_msgs::LaserScan laser_scan, geometry_msgs::PoseArray crowdpose, geometry_msgs::PoseArray crowdposeall){
  cout << "In update state" << endl;
  beliefs->getAgentState()->setCurrentSensor(current, laser_scan);
  // beliefs->getSpatialModel()->getSituations()->addObservationToSituations(laser_scan, current, true);
  beliefs->getAgentState()->setCrowdPose(crowdpose);
  beliefs->getAgentState()->setCrowdPoseAll(crowdposeall);
  if(firstTaskAssigned == false){
      cout << "Set first task" << endl;
      if(aStarOn and highwayExploration->getHighwaysComplete()){
        tierTwoDecision(current);
      }
      else{
        beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
      }
      firstTaskAssigned = true;
  }
  if(highwayExploration->getHighwaysComplete()){
    //bool waypointReached = beliefs->getAgentState()->getCurrentTask()->isWaypointComplete(current);
    bool waypointReached = beliefs->getAgentState()->getCurrentTask()->isAnyWaypointComplete(current);
    bool taskCompleted = beliefs->getAgentState()->getCurrentTask()->isTaskComplete(current);
    bool isPlanActive = beliefs->getAgentState()->getCurrentTask()->getIsPlanActive();
    if(highwayFinished == 1){
      learnSpatialModel(beliefs->getAgentState(), true);
      ROS_DEBUG("Finished Learning Spatial Model!!");
      beliefs->getAgentState()->finishTask();
      ROS_DEBUG("Selecting Next Task");
      if(aStarOn){
        tierTwoDecision(current);
        ROS_DEBUG("Next Plan Generated!!");
      }
      else{
        beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
        ROS_DEBUG("Next Task Selected!!");
      }
    }
    //if task is complete
    if(taskCompleted == true){
      ROS_DEBUG("Target Achieved, moving on to next target!!");
      //Learn spatial model only on tasks completed successfully
      if(beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size() <= 2000){
        learnSpatialModel(beliefs->getAgentState(), true);
        ROS_DEBUG("Finished Learning Spatial Model!!");
        beliefs->getSpatialModel()->getSituations()->learnSituationActions(beliefs->getAgentState(), beliefs->getSpatialModel()->getTrails()->getTrail(beliefs->getSpatialModel()->getTrails()->getSize()-1));
        ROS_DEBUG("Finished Learning Situations!!");
      }
      //Clear existing task and associated plans
      beliefs->getAgentState()->finishTask();
      //ROS_DEBUG("Task Cleared!!");
      //cout << "Agenda Size = " << beliefs->getAgentState()->getAgenda().size() << endl;
      if(beliefs->getAgentState()->getAgenda().size() > 0){
        //Tasks the next task , current position and a planner and generates a sequence of waypoints if astaron is true
        ROS_DEBUG_STREAM("Controller.cpp taskCount > " << (beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size()) << " planLimit " << (planLimit - 1));
        if((beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size()) > (planLimit - 1)){
          aStarOn = false;
        }
        ROS_DEBUG("Selecting Next Task");
        if(aStarOn){
          tierTwoDecision(current);
          ROS_DEBUG("Next Plan Generated!!");
        }
        else{
          beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
          ROS_DEBUG("Next Task Selected!!");
        }
      }
    }
    // else if subtask is complete
    else if(waypointReached == true and aStarOn){
      ROS_DEBUG("Waypoint reached, but task still incomplete, switching to nearest visible waypoint towards target!!");
      //beliefs->getAgentState()->getCurrentTask()->setupNextWaypoint(current);
      beliefs->getAgentState()->getCurrentTask()->setupNearestWaypoint(current);
      //beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getCurrentTask(),current,planner,aStarOn);
    }
    // else if(isPlanActive == false and aStarOn){
    //   ROS_DEBUG("No active plan, setting up new plan!!");
    //   tierTwoDecision(current);
    // }
    else if(waypointReached == true){
      ROS_DEBUG("Temporary Waypoint reached!!");
      beliefs->getAgentState()->getCurrentTask()->setIsPlanActive(false);
      beliefs->getAgentState()->getCurrentTask()->clearWaypoints();
    }
    // otherwise if task Decision limit reached, skip task 
    if(beliefs->getAgentState()->getCurrentTask() != NULL){
      if(beliefs->getAgentState()->getCurrentTask()->getDecisionCount() > taskDecisionLimit){
        ROS_DEBUG_STREAM("Controller.cpp decisionCount > " << taskDecisionLimit << " , skipping task");
        learnSpatialModel(beliefs->getAgentState(), false);
        ROS_DEBUG("Finished Learning Spatial Model!!");
        beliefs->getSpatialModel()->getSituations()->learnSituationActions(beliefs->getAgentState(), beliefs->getSpatialModel()->getTrails()->getTrail(beliefs->getSpatialModel()->getTrails()->getSize()-1));
        ROS_DEBUG("Finished Learning Situations!!");
        //beliefs->getAgentState()->skipTask();
        beliefs->getAgentState()->finishTask();
        if(beliefs->getAgentState()->getAgenda().size() > 0){
          ROS_DEBUG_STREAM("Controller.cpp taskCount > " << (beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size()) << " planLimit " << (planLimit - 1));
          if((beliefs->getAgentState()->getAllAgenda().size() - beliefs->getAgentState()->getAgenda().size()) > (planLimit - 1)){
            aStarOn = false;
          }
          if(aStarOn){
            tierTwoDecision(current);
          }
          else{
            beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());
          }
        }
      }
    }
  }
  
  //ROS_DEBUG("End Of UpdateState");
}


// Function which returns the mission status
bool Controller::isMissionComplete(){
  return beliefs->getAgentState()->isMissionComplete();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Main robot decision making engine, return decisions that would lead the robot to complete its mission
// Manages switching tasks and stops if the robot is taking too long
//
FORRAction Controller::decide() {
  ROS_DEBUG("Entering decision loop");
  if(!highwayExploration->getHighwaysComplete()){
    return highwayExploration->exploreDecision(beliefs->getAgentState()->getCurrentPosition(), beliefs->getAgentState()->getCurrentLaserScan());
  }
  else{
    highwayFinished++;
    return FORRDecision();
  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Update spatial model after every task 
//
//

void Controller::learnSpatialModel(AgentState* agentState, bool taskStatus){
  double computationTimeSec=0.0;
  timeval cv;
  double start_timecv;
  double end_timecv;
  gettimeofday(&cv,NULL);
  start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);

  Task* completedTask = agentState->getCurrentTask();
  vector<Position> *pos_hist = completedTask->getPositionHistory();
  vector< vector<CartesianPoint> > *laser_hist = completedTask->getLaserHistory();
  vector< vector<CartesianPoint> > all_trace = beliefs->getAgentState()->getAllTrace();
  //vector< vector<CartesianPoint> > all_laser_hist = beliefs->getAgentState()->getAllLaserHistory();
  vector<CartesianPoint> trace;
  for(int i = 0 ; i < pos_hist->size() ; i++){
    trace.push_back(CartesianPoint((*pos_hist)[i].getX(),(*pos_hist)[i].getY()));
  }
  all_trace.push_back(trace);
  //for(int i = 0 ; i < laser_hist->size() ; i++){
  //  all_laser_hist.push_back((*laser_hist)[i]);
  //}

  if(trailsOn){
    beliefs->getSpatialModel()->getTrails()->updateTrails(agentState);
    beliefs->getSpatialModel()->getTrails()->resetChosenTrail();
    ROS_DEBUG("Trails Learned");
  }
  vector< vector<CartesianPoint> > trails_trace = beliefs->getSpatialModel()->getTrails()->getTrailsPoints();
  if(conveyorsOn and taskStatus){
    //beliefs->getSpatialModel()->getConveyors()->populateGridFromPositionHistory(pos_hist);
    beliefs->getSpatialModel()->getConveyors()->populateGridFromTrailTrace(trails_trace.back());
    ROS_DEBUG("Conveyors Learned");
  }
  if(regionsOn){
    beliefs->getSpatialModel()->getRegionList()->learnRegions(pos_hist, laser_hist);
    ROS_DEBUG("Regions Learned");
    beliefs->getSpatialModel()->getRegionList()->clearAllExits();
    beliefs->getSpatialModel()->getRegionList()->learnExits(all_trace);
    beliefs->getSpatialModel()->getRegionList()->learnExits(trails_trace);
    ROS_DEBUG("Exits Learned");
  }
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  if(doorsOn){
    beliefs->getSpatialModel()->getDoors()->clearAllDoors();
    beliefs->getSpatialModel()->getDoors()->learnDoors(regions);
    ROS_DEBUG("Doors Learned");
  }
  if(hallwaysOn){
    //beliefs->getSpatialModel()->getHallways()->clearAllHallways();
    //beliefs->getSpatialModel()->getHallways()->learnHallways(agentState, all_trace, all_laser_hist);
    beliefs->getSpatialModel()->getHallways()->learnHallways(agentState, trace, laser_hist);
    //beliefs->getSpatialModel()->getHallways()->learnHallways(trails_trace);
    ROS_DEBUG("Hallways Learned");
  }
  if(barrsOn){
    beliefs->getSpatialModel()->getBarriers()->updateBarriers(laser_hist, all_trace.back());
    ROS_DEBUG("Barriers Learned");
  }

  gettimeofday(&cv,NULL);
  end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
  computationTimeSec = (end_timecv-start_timecv);
  decisionStats->learningComputationTime = computationTimeSec;

  if(skeleton){
    PathPlanner *skeleton_planner;
    for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
      if((*it)->getName() == "skeleton"){
        skeleton_planner = *it;
      }
    }
    skeleton_planner->resetGraph();
    int index_val = 0;
    // vector<bool> added_nodes;
    for(int i = 0 ; i < regions.size(); i++){
      int x = (int)(regions[i].getCenter().get_x()*100);
      int y = (int)(regions[i].getCenter().get_y()*100);
      cout << "Region " << regions[i].getCenter().get_x() << " " << regions[i].getCenter().get_y() << " " << x << " " << y << endl;
      bool success = skeleton_planner->getGraph()->addNode(x, y, index_val);
      // added_nodes.push_back(success);
      if(success)
        index_val++;
      vector<FORRExit> exits = regions[i].getExits();
      for(int j = 0; j < exits.size() ; j++){
        int ex = (int)(exits[j].getExitPoint().get_x()*100);
        int ey = (int)(exits[j].getExitPoint().get_y()*100);
        cout << "Exit " << exits[j].getExitPoint().get_x() << " " << exits[j].getExitPoint().get_y() << " " << ex << " " << ey << endl;
        success = skeleton_planner->getGraph()->addNode(ex, ey, index_val);
        // added_nodes.push_back(success);
        if(success)
          index_val++;
      }
      // if(regions[i].getRadius() >= 0.5){
      //   double angle = 0;
      //   for(int k = 0; k < 24; k++){
      //     int ax = (int)((regions[i].getCenter().get_x() + regions[i].getRadius()*0.9 * cos(angle))*100);
      //     int ay = (int)((regions[i].getCenter().get_y() + regions[i].getRadius()*0.9 * sin(angle))*100);
      //     cout << "Points on Region " << ax << " " << ay << endl;
      //     success = skeleton_planner->getGraph()->addNode(ax, ay, index_val);
      //     // added_nodes.push_back(success);
      //     if(success)
      //       index_val++;
      //     // ax = (int)((regions[i].getCenter().get_x() + regions[i].getRadius()/2 * cos(angle))*100);
      //     // ay = (int)((regions[i].getCenter().get_y() + regions[i].getRadius()/2 * sin(angle))*100);
      //     // cout << "Half Points on Region " << ax << " " << ay << endl;
      //     // success = skeleton_planner->getGraph()->addNode(ax, ay, index_val);
      //     // added_nodes.push_back(success);
      //     // index_val++;
      //     angle = angle + 0.2617993878;
      //   }
      // }
    }
    // index_val = 0;
    for(int i = 0 ; i < regions.size(); i++){
      int region_id = skeleton_planner->getGraph()->getNodeID((int)(regions[i].getCenter().get_x()*100), (int)(regions[i].getCenter().get_y()*100));
      // int region_id = index_val;
      // index_val++;
      vector<FORRExit> exits = regions[i].getExits();
      for(int j = 0; j < exits.size() ; j++){
        int index_val = skeleton_planner->getGraph()->getNodeID((int)(exits[j].getExitPoint().get_x()*100), (int)(exits[j].getExitPoint().get_y()*100));
        // if(added_nodes[index_val]){
        cout << "Edge from " << region_id << " to " << index_val << " Distance " << regions[i].getRadius()*100 << endl;
        skeleton_planner->getGraph()->addEdge(region_id, index_val, regions[i].getRadius()*100);
        int tx = (int)(exits[j].getExitRegionPoint().get_x()*100);
        int ty = (int)(exits[j].getExitRegionPoint().get_y()*100);
        cout << "Edge from " << index_val<< " to " << skeleton_planner->getGraph()->getNodeID(tx, ty) << " Distance " << exits[j].getExitDistance()*100 << endl;
        skeleton_planner->getGraph()->addEdge(index_val, skeleton_planner->getGraph()->getNodeID(tx, ty), exits[j].getExitDistance()*100);
        // }
        // index_val++;
        // cout << "Edge from " << i << " to " << exits[j].getExitRegion() << " Distance " << exits[j].getExitDistance()*100 << endl;
        // skeleton_planner->getGraph()->addEdge(i, exits[j].getExitRegion(), exits[j].getExitDistance()*100);
      }
      // if(regions[i].getRadius() >= 0.5){
      //   double angle = 0;
      //   for(int k = 0; k < 24; k++){
      //     int index_val = skeleton_planner->getGraph()->getNodeID((int)((regions[i].getCenter().get_x() + regions[i].getRadius()*0.9 * cos(angle))*100), (int)((regions[i].getCenter().get_y() + regions[i].getRadius()*0.9 * sin(angle))*100));
      //     // if(added_nodes[index_val] and added_nodes[index_val+1]){
      //     //   cout << "Edge from " << index_val << " to " << index_val+1 << " Distance " << regions[i].getRadius()*100 << endl;
      //     //   skeleton_planner->getGraph()->addEdge(index_val, index_val+1, regions[i].getRadius()*100);
      //     //   cout << "Edge from " << region_id << " to " << index_val+1 << " Distance " << regions[i].getRadius()/2*100 << endl;
      //     //   skeleton_planner->getGraph()->addEdge(region_id, index_val, regions[i].getRadius()/2*100);
      //     // }
      //     // if(added_nodes[index_val]){
      //     cout << "Edge from " << region_id << " to " << index_val << " Distance " << regions[i].getRadius()*0.9*100 << endl;
      //     skeleton_planner->getGraph()->addEdge(region_id, index_val, regions[i].getRadius()*0.9*100);
      //     // }
      //     // index_val++;
      //     // if(added_nodes[index_val]){
      //     //   cout << "Edge from " << region_id << " to " << index_val << " Distance " << regions[i].getRadius()/2*100 << endl;
      //     //   skeleton_planner->getGraph()->addEdge(region_id, index_val, regions[i].getRadius()/2*100);
      //     // }
      //     // index_val++;
      //     angle = angle + 0.2617993878;
      //   }
      // }
    }
    cout << "Connected Graph: " << skeleton_planner->getGraph()->isConnected() << endl;
  }
  if(hallwayskel){
    PathPlanner *hwskeleton_planner;
    for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
      if((*it)->getName() == "hallwayskel"){
        hwskeleton_planner = *it;
      }
    }
    hwskeleton_planner->resetGraph();
    int index_val = 0;
    vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
    for(int i = 0; i < hallways.size(); i++){
      vector<CartesianPoint> points_in_hallway = hallways[i].getPoints();
      cout << "Hallway " << i << " contains " << points_in_hallway.size() << " points" << endl;
      for(int j = 0; j < points_in_hallway.size(); j++){
        int x = (int)(points_in_hallway[j].get_x()*100);
        int y = (int)(points_in_hallway[j].get_y()*100);
        cout << "Hallway point " << points_in_hallway[j].get_x() << " " << points_in_hallway[j].get_y() << " " << x << " " << y << endl;
        bool success = hwskeleton_planner->getGraph()->addNode(x, y, index_val);
        if(success)
          index_val++;
      }
    }
    for(int i = 0; i < hallways.size(); i++){
      vector<CartesianPoint> points_in_hallway = hallways[i].getPoints();
      for(int j = 0; j < points_in_hallway.size()-1; j++){
        for(int k = j+1; k < points_in_hallway.size(); k++){
          double distance_jk = points_in_hallway[j].get_distance(points_in_hallway[k]);
          if(distance_jk <= 3){
            int j_ind = hwskeleton_planner->getGraph()->getNodeID((int)(points_in_hallway[j].get_x()*100), (int)(points_in_hallway[j].get_y()*100));
            int k_ind = hwskeleton_planner->getGraph()->getNodeID((int)(points_in_hallway[k].get_x()*100), (int)(points_in_hallway[k].get_y()*100));
            cout << "Edge from " << j_ind << " to " << k_ind << " Distance " << distance_jk*100 << endl;
            if(j_ind != k_ind)
              hwskeleton_planner->getGraph()->addEdge(j_ind, k_ind, distance_jk*100);
          }
        }
      }
    }
    // hwskeleton_planner->getGraph()->populateNodeNeighbors(false);
    // hwskeleton_planner->getGraph()->populateEdges();
    // for(int i = 0; i < hallways.size()-1; i++){
    //   for(int j = i+1; j < hallways.size(); j++){
    //     if(!hallways[i].isHallwayConnected(j)){
    //       vector<double> points_between = hallways[i].closestPointsBetweenAggregates(hallways[j]);
    //       cout << "Closest Points between hallway " << i << " and " << j << endl;
    //       int sx = (int)(points_between[1]*100);
    //       int sy = (int)(points_between[2]*100);
    //       int ex = (int)(points_between[3]*100);
    //       int ey = (int)(points_between[4]*100);
    //       int sid = hwskeleton_planner->getGraph()->getNodeID(sx, sy);
    //       int eid = hwskeleton_planner->getGraph()->getNodeID(ex, ey);
    //       cout << "Start " << points_between[1] << " " << points_between[2] << " End " << points_between[3] << " " << points_between[4] << " Distance " << points_between[0] << endl;
    //       cout << "sx " << sx << " sy " << sy << " sid " << sid << " ex " << ex << " ey " << ey << " eid " << eid << endl;
    //       hwskeleton_planner->getGraph()->addEdge(sid, eid, points_between[0]*100);
    //     }
    //   }
    // }
    for(int i = 0; i < trails_trace.size(); i++){
      cout << "Trail " << i << " contains " << trails_trace[i].size() << " points" << endl;
      for(int j = 0; j < trails_trace[i].size(); j++){
        int x = (int)round(trails_trace[i][j].get_x())*100;
        int y = (int)round(trails_trace[i][j].get_y())*100;
        cout << "Trail point " << j << " " << trails_trace[i][j].get_x() << " " << trails_trace[i][j].get_y() << " " << x << " " << y << endl;
        bool success = hwskeleton_planner->getGraph()->addNode(x, y, index_val);
        // cout << success << endl;
        if(success)
          index_val++;
        // cout << "Finished adding node" << endl;
      }
      // cout << "Finished trail " << i << endl;
    }
    for(int i = 0; i < trails_trace.size(); i++){
      for(int j = 0; j < trails_trace[i].size()-1; j++){
        double distance_jk = trails_trace[i][j].get_distance(trails_trace[i][j+1]);
        int j_ind = hwskeleton_planner->getGraph()->getNodeID((int)round(trails_trace[i][j].get_x())*100, (int)round(trails_trace[i][j].get_y())*100);
        int k_ind = hwskeleton_planner->getGraph()->getNodeID((int)round(trails_trace[i][j+1].get_x())*100, (int)round(trails_trace[i][j+1].get_y())*100);
        cout << "Edge from " << j_ind << " to " << k_ind << " Distance " << distance_jk*100 << endl;
        if(j_ind != k_ind)
          hwskeleton_planner->getGraph()->addEdge(j_ind, k_ind, distance_jk*100);
      }
    }
    for(int i = 0; i < trails_trace.size()-1; i++){
      double distance_jk = trails_trace[i][trails_trace[i].size()-1].get_distance(trails_trace[i+1][0]);
      int j_ind = hwskeleton_planner->getGraph()->getNodeID((int)round(trails_trace[i][trails_trace[i].size()-1].get_x())*100, (int)round(trails_trace[i][trails_trace[i].size()-1].get_y())*100);
      int k_ind = hwskeleton_planner->getGraph()->getNodeID((int)round(trails_trace[i+1][0].get_x())*100, (int)round(trails_trace[i+1][0].get_y())*100);
      cout << "Edge from " << j_ind << " to " << k_ind << " Distance " << distance_jk*100 << endl;
      if(j_ind != k_ind)
        hwskeleton_planner->getGraph()->addEdge(j_ind, k_ind, distance_jk*100);
    }

    for(int i = 0 ; i < regions.size(); i++){
      int x = (int)round(regions[i].getCenter().get_x())*100;
      int y = (int)round(regions[i].getCenter().get_y())*100;
      cout << "Region " << regions[i].getCenter().get_x() << " " << regions[i].getCenter().get_y() << " " << x << " " << y << endl;
      bool success = hwskeleton_planner->getGraph()->addNode(x, y, index_val);
      if(success)
        index_val++;
      vector<FORRExit> exits = regions[i].getExits();
      for(int j = 0; j < exits.size() ; j++){
        int ex = (int)round(exits[j].getExitPoint().get_x())*100;
        int ey = (int)round(exits[j].getExitPoint().get_y())*100;
        cout << "Exit " << exits[j].getExitPoint().get_x() << " " << exits[j].getExitPoint().get_y() << " " << ex << " " << ey << endl;
        success = hwskeleton_planner->getGraph()->addNode(ex, ey, index_val);
        if(success)
          index_val++;
      }
    }
    for(int i = 0 ; i < regions.size(); i++){
      int region_id = hwskeleton_planner->getGraph()->getNodeID((int)round(regions[i].getCenter().get_x())*100, (int)round(regions[i].getCenter().get_y())*100);
      vector<FORRExit> exits = regions[i].getExits();
      for(int j = 0; j < exits.size() ; j++){
        int index_val = hwskeleton_planner->getGraph()->getNodeID((int)round(exits[j].getExitPoint().get_x())*100, (int)round(exits[j].getExitPoint().get_y())*100);
        cout << "Edge from " << region_id << " to " << index_val << " Distance " << regions[i].getRadius()*100 << endl;
        if(region_id != index_val)
          hwskeleton_planner->getGraph()->addEdge(region_id, index_val, regions[i].getRadius()*100);
        int tx = (int)round(exits[j].getExitRegionPoint().get_x())*100;
        int ty = (int)round(exits[j].getExitRegionPoint().get_y())*100;
        cout << "Edge from " << index_val<< " to " << hwskeleton_planner->getGraph()->getNodeID(tx, ty) << " Distance " << exits[j].getExitDistance()*100 << endl;
        if(hwskeleton_planner->getGraph()->getNodeID(tx, ty) != index_val)
          hwskeleton_planner->getGraph()->addEdge(index_val, hwskeleton_planner->getGraph()->getNodeID(tx, ty), exits[j].getExitDistance()*100);
      }
    }

    cout << "Connected Graph: " << hwskeleton_planner->getGraph()->isConnected() << endl;
  }
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// SemaFORR decision workflow
//
//
FORRAction Controller::FORRDecision()
{  
  ROS_DEBUG("In FORR decision");
  FORRAction *decision = new FORRAction();
  // Basic semaFORR three tier decision making architecture 
  if(!tierOneDecision(decision)){
  	ROS_DEBUG("Decision to be made by t3!!");
  	//decision->type = FORWARD;
  	//decision->parameter = 5;
  	tierThreeDecision(decision);
  	tierThreeAdvisorInfluence();
  	decisionStats->decisionTier = 3;
  }
  //cout << "decisionTier = " << decisionStats->decisionTier << endl;
  //ROS_DEBUG("After decision made");
  beliefs->getAgentState()->getCurrentTask()->incrementDecisionCount();
  //ROS_DEBUG("After incrementDecisionCount");
  beliefs->getAgentState()->getCurrentTask()->saveDecision(*decision);
  //ROS_DEBUG("After saveDecision");
  beliefs->getAgentState()->clearVetoedActions();
  //ROS_DEBUG("After clearVetoedActions");
  if(decision->type == FORWARD or decision->type == PAUSE){
    beliefs->getAgentState()->setRotateMode(true);
  }
  else{
    beliefs->getAgentState()->setRotateMode(false);
  }
  Position current = beliefs->getAgentState()->getCurrentPosition();
  sensor_msgs::LaserScan laser_scan = beliefs->getAgentState()->getCurrentLaserScan();
  beliefs->getSpatialModel()->getSituations()->addObservationToSituations(laser_scan, current, true, *decision);
  return *decision;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Generate tier 1 decision
//
//
bool Controller::tierOneDecision(FORRAction *decision){
  //decision making tier1 advisor
  bool decisionMade = false;
  if(tier1->advisorVictory(decision)){ 
    ROS_INFO_STREAM("Advisor victory has made a decision " << decision->type << " " << decision->parameter);
    decisionStats->decisionTier = 1;
    decisionMade = true;
  }
  else{
    if(tier1->advisorGetOut(decision)){
      ROS_INFO_STREAM("Advisor get out has made a decision " << decision->type << " " << decision->parameter);
      decisionStats->decisionTier = 1;
      decisionMade = true;
    }
    else{
      // group of vetoing tier1 advisors which adds to the list of vetoed actions
      ROS_INFO("Advisor don't go back will veto actions");
      tier1->advisorDontGoBack();
      ROS_INFO("Advisor avoid wall will veto actions");
      tier1->advisorAvoidWalls();
      ROS_INFO("Advisor not opposite will veto actions");
      tier1->advisorNotOpposite();
    }
    // if(situationsOn){
    //   ROS_INFO("Advisor situation will veto actions");
    //   tier1->advisorSituation();
    // }
  }
  set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
  std::stringstream vetoList;
  set<FORRAction>::iterator it;
  for(it = vetoedActions->begin(); it != vetoedActions->end(); it++){
    vetoList << it->type << " " << it->parameter << ";";
  }
  decisionStats->vetoedActions = vetoList.str();
  //cout << "vetoedActions = " << vetoList.str() << endl;
  
  return decisionMade;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Generate tier 2 decision
//
//
void Controller::tierTwoDecision(Position current){
  ROS_DEBUG_STREAM("Tier 2 Decision");
  vector< list<int> > plans;
  vector<string> plannerNames;
  typedef vector< list<int> >::iterator vecIT;

  beliefs->getAgentState()->setCurrentTask(beliefs->getAgentState()->getNextTask());

  double computationTimeSec=0.0;
  timeval cv;
  double start_timecv;
  double end_timecv;
  gettimeofday(&cv,NULL);
  start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);

  for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
    PathPlanner *planner = *it;
    planner->setPosHistory(beliefs->getAgentState()->getAllTrace());
    vector< vector<CartesianPoint> > trails_trace = beliefs->getSpatialModel()->getTrails()->getTrailsPoints();
    planner->setSpatialModel(beliefs->getSpatialModel()->getConveyors(),beliefs->getSpatialModel()->getRegionList()->getRegions(),beliefs->getSpatialModel()->getDoors()->getDoors(),trails_trace,beliefs->getSpatialModel()->getHallways()->getHallways());
    //ROS_DEBUG_STREAM("Creating plans " << planner->getName());
    //gettimeofday(&cv,NULL);
    //start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
    vector< list<int> > multPlans = beliefs->getAgentState()->getPlansWaypoints(current,planner,aStarOn);
    for (int i = 0; i < multPlans.size(); i++){
      plans.push_back(multPlans[i]);
      plannerNames.push_back(planner->getName());
    }
    //gettimeofday(&cv,NULL);
    //end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
    //computationTimeSec = (end_timecv-start_timecv);
    //ROS_DEBUG_STREAM("Planning time = " << computationTimeSec);
  }
  if(beliefs->getAgentState()->getCurrentTask()->getIsPlanActive() == true){
    vector< vector<double> > planCosts;
    typedef vector< vector<double> >::iterator costIT;

    for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
      PathPlanner *planner = *it;
      vector<double> planCost;
      // ROS_DEBUG_STREAM("Computing plan cost " << planner->getName());
      for (vecIT vt = plans.begin(); vt != plans.end(); vt++){
        double costOfPlan = planner->calcPathCost(*vt);
        planCost.push_back(costOfPlan);
        // ROS_DEBUG_STREAM("Cost = " << costOfPlan);
      }
      planCosts.push_back(planCost);
    }

    typedef vector<double>::iterator doubIT;
    vector< vector<double> > planCostsNormalized;
    for (costIT it = planCosts.begin(); it != planCosts.end(); it++){
      double max = *max_element(it->begin(), it->end());
      double min = *min_element(it->begin(), it->end());
      double norm_factor = (max - min)/10;
      vector<double> planCostNormalized;
      // ROS_DEBUG_STREAM("Computing normalized plan cost: Max = " << max << " Min = " << min << " Norm Factor = " << norm_factor);
      for (doubIT vt = it->begin(); vt != it->end(); vt++){
        if (max != min){
          planCostNormalized.push_back((*vt - min)/norm_factor);
          // ROS_DEBUG_STREAM("Original value = " << *vt << " Normalized = " << ((*vt - min)/norm_factor));
        }
        else{
          planCostNormalized.push_back(0);
          // ROS_DEBUG_STREAM("Original value = " << *vt << " Normalized = 0");
        }
      }
      planCostsNormalized.push_back(planCostNormalized);
    }
    //planCostsNormalized.pop_back();
    vector<double> totalCosts;
    for (int i = 0; i < plans.size(); i++){
      double cost=0;
      // ROS_DEBUG_STREAM("Computing total cost = " << cost);
      for (costIT it = planCostsNormalized.begin(); it != planCostsNormalized.end(); it++){
        cost += it->at(i);
        // ROS_DEBUG_STREAM("cost = " << cost);
      }
      // ROS_DEBUG_STREAM("Final cost = " << cost);
      totalCosts.push_back(cost);
    }
    double minCost=100000;
    // ROS_DEBUG_STREAM("Computing min cost");
    for (int i=0; i < totalCosts.size(); i++){
      // ROS_DEBUG_STREAM("Total cost = " << totalCosts[i]);
      if (totalCosts[i] < minCost){
        minCost = totalCosts[i];
      }
    }

    /*double minCombinedCost=1000;
    for (int i=18; i < totalCosts.size(); i++){
      ROS_DEBUG_STREAM("Total cost = " << totalCosts[i]);
      if (totalCosts[i] < minCombinedCost){
        minCombinedCost = totalCosts[i];
      }
    }*/
    // ROS_DEBUG_STREAM("Min cost = " << minCost);
    //ROS_DEBUG_STREAM("Min Combined cost = " << minCombinedCost);

    vector<string> bestPlanNames;
    vector<int> bestPlanInds;
    for (int i=0; i < totalCosts.size(); i++){
      if(totalCosts[i] == minCost){
        bestPlanNames.push_back(plannerNames[i]);
        bestPlanInds.push_back(i);
        // ROS_DEBUG_STREAM("Best plan " << plannerNames[i]);
      }
    }

    srand(time(NULL));
    int random_number = rand() % (bestPlanInds.size());
    ROS_DEBUG_STREAM("Number of best plans = " << bestPlanInds.size() << " random_number = " << random_number);
    ROS_DEBUG_STREAM("Selected Best plan " << bestPlanNames.at(random_number));
    decisionStats->chosenPlanner = bestPlanNames.at(random_number);
    beliefs->getAgentState()->setCurrentWaypoints(current,tier2Planners[0],aStarOn, plans.at(bestPlanInds.at(random_number)));
  }
  for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
    PathPlanner *planner = *it;
    planner->resetPath();
  }
  gettimeofday(&cv,NULL);
  end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
  computationTimeSec = (end_timecv-start_timecv);
  decisionStats->planningComputationTime = computationTimeSec;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Generate tier 3 decision
//
//
void Controller::tierThreeDecision(FORRAction *decision){
  std::map<FORRAction, double> comments;
  // This map will aggregate value of all advisers
  std::map<FORRAction, double> allComments;

  // typedef to make for declaration that iterates over map shorter
  typedef map<FORRAction, double>::iterator mapIt;

  // vector of all the actions that got max comment strength in iteration
  vector<FORRAction> best_decisions;
  
  double rotationBaseline, linearBaseline;
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    Tier3Advisor *advisor = *it;
    //if(advisor->is_active() == true)
      //cout << advisor->get_name() << " : " << advisor->get_weight() << endl;
    if(advisor->get_name() == "RotationBaseLine") rotationBaseline = advisor->get_weight();
    if(advisor->get_name() == "BaseLine")         linearBaseline   = advisor->get_weight();
  }
       
  std::stringstream advisorsList;
  std::stringstream advisorCommentsList;
  //cout << "processing advisors::"<< endl;
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    Tier3Advisor *advisor = *it; 
    //cout << advisor->get_name() << endl;
    // check if advisor should make a decision
    advisor->set_commenting();
    if(advisor->is_active() == false){
      //cout << advisor->get_name() << " is inactive " << endl;
      advisorsList << advisor->get_name() << " " << advisor->get_weight() << " " << advisor->is_active() << " " << advisor->is_commenting() << ";";
      continue;
    }
    if(advisor->is_commenting() == false){
      //cout << advisor->get_name() << " is not commenting " << endl;
      advisorsList << advisor->get_name() << " " << advisor->get_weight() << " " << advisor->is_active() << " " << advisor->is_commenting() << ";";
      continue;
    }

    advisorsList << advisor->get_name() << " " << advisor->get_weight() << " " << advisor->is_active() << " " << advisor->is_commenting() << ";";

    //cout << "Before commenting " << endl;
    comments = advisor->allAdvice();
    //cout << "after commenting " << endl;
    // aggregate all comments

    for(mapIt iterator = comments.begin(); iterator != comments.end(); iterator++){
      //cout << "comment : " << (iterator->first.type) << (iterator->first.parameter) << " " << (iterator->second) << endl;
      // If this is first advisor we need to initialize our final map
      float weight;
      //cout << "Agenda size :::::::::::::::::::::::::::::::::: " << beliefs->getAgenda().size() << endl;
      //cout << "<" << advisor->get_name() << "," << iterator->first.type << "," << iterator->first.parameter << "> : " << iterator->second << endl; 
      weight = advisor->get_weight();
      //cout << "Weight for this advisor : " << weight << endl;
      // if(advisor->get_name() == "Explorer" or advisor->get_name() == "ExplorerRotation" or advisor->get_name() == "LearnSpatialModel" or advisor->get_name() == "LearnSpatialModelRotation" or advisor->get_name() == "Curiosity" or advisor->get_name() == "CuriosityRotation"){
      //   weight = beliefs->getAgentState()->getAgenda().size()/5;
      // }

      advisorCommentsList << advisor->get_name() << " " << iterator->first.type << " " << iterator->first.parameter << " " << iterator->second << ";";

      if( allComments.find(iterator->first) == allComments.end()){
	    allComments[iterator->first] =  iterator->second * weight;
      }
      else{
	    allComments[iterator->first] += iterator->second * weight;
      }
    }
  } 
  
  // Loop through map advisor created and find command with the highest vote
  double maxAdviceStrength = -1000;
  double maxWeight;
  for(mapIt iterator = allComments.begin(); iterator != allComments.end(); iterator++){
    double action_weight = 1.0;
    if(situationsOn){
      action_weight = beliefs->getSpatialModel()->getSituations()->getWeightForAction(beliefs->getAgentState(), iterator->first);
    }
    cout << "Values are : " << iterator->first.type << " " << iterator->first.parameter << " with value: " << iterator->second << " and weight: " << action_weight << endl;
    if(action_weight * iterator->second > maxAdviceStrength){
      maxAdviceStrength = action_weight * iterator->second;
      maxWeight = action_weight;
    }
  }
  cout << "Max vote strength " << maxAdviceStrength << endl;
  
  for(mapIt iterator = allComments.begin(); iterator!=allComments.end(); iterator++){
    if(maxWeight * iterator->second == maxAdviceStrength)
      best_decisions.push_back(iterator->first);
  }
  
  cout << "There are " << best_decisions.size() << " decisions that got the highest grade " << endl;
  if(best_decisions.size() == 0){
      (*decision) = FORRAction(PAUSE,0);
  }
  //for(unsigned i = 0; i < best_decisions.size(); ++i)
      //cout << "Action type: " << best_decisions.at(i).type << " parameter: " << best_decisions.at(i).parameter << endl;
    
  //generate random number using system clock as seed
  srand(time(NULL));
  int random_number = rand() % (best_decisions.size());
    
  (*decision) = best_decisions.at(random_number);
  decisionStats->advisors = advisorsList.str();
  decisionStats->advisorComments = advisorCommentsList.str();
  //cout << " advisors = " << decisionStats->advisors << "\nadvisorComments = " << decisionStats->advisorComments << endl;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Checks if an T3 advisor is active
//
//
bool Controller::
isAdvisorActive(string advisorName){
  bool isActive = false;
  
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    Tier3Advisor *advisor = *it;
    if(advisor->is_active() == true && advisor->get_name() == advisorName)
      isActive = true;
  }
  
  return isActive;
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Check influence of tier 3 Advisors
//
//
void Controller::tierThreeAdvisorInfluence(){
  /*std::map<FORRAction, double> comments;
  // This map will aggregate value of all advisers
  std::map<FORRAction, double> allComments;

  // typedef to make for declaration that iterates over map shorter
  typedef map<FORRAction, double>::iterator mapIt;

  // vector of all the actions that got max comment strength in iteration
  vector<FORRAction> best_decisions;
  
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    Tier3Advisor *advisor = *it; 

    // check if advisor should make a decision
    advisor->set_commenting();
    if(advisor->is_active() == false){
      continue;
    }
    if(advisor->is_commenting() == false){
      continue;
    }

    comments = advisor->allAdvice();
    // aggregate all comments

    for(mapIt iterator = comments.begin(); iterator != comments.end(); iterator++){
      // If this is first advisor we need to initialize our final map
      float weight;
      weight = advisor->get_weight();

      if( allComments.find(iterator->first) == allComments.end()){
      allComments[iterator->first] =  iterator->second * weight;
      }
      else{
      allComments[iterator->first] += iterator->second * weight;
      }
    }
  } 
  
  // Loop through map advisor created and find command with the highest vote
  double maxAdviceStrength = -1000;
  for(mapIt iterator = allComments.begin(); iterator != allComments.end(); iterator++){
    cout << "Values are : " << iterator->first.type << " " << iterator->first.parameter << " with value: " << iterator->second << endl;
    if(iterator->second > maxAdviceStrength){
      maxAdviceStrength = iterator->second;
    }
  }
  //cout << "Max vote strength " << maxAdviceStrength << endl;
  
  for(mapIt iterator = allComments.begin(); iterator!=allComments.end(); iterator++){
    if(iterator->second == maxAdviceStrength)
      best_decisions.push_back(iterator->first);
  }
  
  std::stringstream advisorsInfluence;
  std::map<FORRAction, double> takeOneOutComments;
  for (advisor3It it = tier3Advisors.begin(); it != tier3Advisors.end(); ++it){
    takeOneOutComments = allComments;
    Tier3Advisor *advisor = *it; 

    // check if advisor should make a decision
    advisor->set_commenting();
    if(advisor->is_active() == false){
      continue;
    }
    if(advisor->is_commenting() == false){
      advisorsInfluence << advisor->get_name() << " -1;";
      continue;
    }

    comments = advisor->allAdvice();
    // aggregate all comments
    if (comments.size() > 1) {
      for(mapIt iterator = comments.begin(); iterator != comments.end(); iterator++){
        // If this is first advisor we need to initialize our final map
        float weight;
        weight = advisor->get_weight();
        takeOneOutComments[iterator->first]-= iterator->second * weight;
      }
      double maxAdviceStrength = -1000;
      for(mapIt iterator = takeOneOutComments.begin(); iterator != takeOneOutComments.end(); iterator++){
        if(iterator->second > maxAdviceStrength){
          maxAdviceStrength = iterator->second;
        }
      }
      bool same=0;
      for(mapIt iterator = takeOneOutComments.begin(); iterator!=takeOneOutComments.end(); iterator++){
        if(iterator->second == maxAdviceStrength){
          for(unsigned i = 0; i < best_decisions.size(); ++i){
            if(iterator->first.type == best_decisions.at(i).type and iterator->first.parameter == best_decisions.at(i).parameter) {
              same = 0;
            }
            else{
              same = 1;
            }
          }
        }
      }
      if (same == 0){
        advisorsInfluence << advisor->get_name() << " 0;";
      }
      else{
        advisorsInfluence << advisor->get_name() << " 1;";
      }
    }
    else {
      advisorsInfluence << advisor->get_name() << " -1;";
    }
  }
  decisionStats->advisorInfluence = advisorsInfluence.str();
  cout << "advisorInfluence = " << decisionStats->advisorInfluence << endl;*/
  decisionStats->advisorInfluence = "";
}
