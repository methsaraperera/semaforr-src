/************************************************
FORRRegionList.h 
This file contains the class which contains information about the regions and exits that FORR learns and uses

Written by Anoop Aroor, 2014
**********************************************/

#ifndef FORRREGIONLIST_H
#define FORRREGIONLIST_H

#include <iostream>
#include <fstream>
#include "FORRGeometry.h"
#include "FORRRegion.h"
#include "FORRExit.h"

class FORRRegionList{
 public:
  FORRRegionList(){};
  vector<FORRRegion> getRegions(){ return regions; }
  FORRRegion getRegion(int index){ return regions[index]; }
  void setRegions(vector<FORRRegion> regions_para) { regions = regions_para; }
  //void addRegion(vector<FORRRegion> region) { regions.push_back(region);}

  //bool isExitToLeaf(FORRExit exit){
  //  return isLeaf(regions[exit.getExitRegion()]);
  //}
    
  bool isLeaf(FORRRegion region, int numDoors){
    //cout << "In isleaf" << endl;
    vector<FORRExit> exits = region.getExits();
    bool isLeaf = false;
    if(exits.size() <= 1 or numDoors <= 1){
      isLeaf = true;
      //cout << "exits.size = " << exits.size() << " numDoors = " << numDoors << endl;
      //region.setIsLeaf(isLeaf);
      return isLeaf;
    }
    double min_angle = 10;
    double max_angle = -10;
    for(int i = 0 ; i < exits.size(); i++){
      double angle = atan2(exits[i].getExitPoint().get_y() - region.getCenter().get_y(), exits[i].getExitPoint().get_x() - region.getCenter().get_x());
      if(angle < 0){
	angle += 2*M_PI;
      }
      //cout << "Exit no : " << i << " Angle : " << angle << endl; 
      if(angle < min_angle) min_angle = angle;
      if(angle >= max_angle) max_angle = angle;
    }
    //cout << "min_angle :" << min_angle << endl;
    //cout << "max_angle :" << max_angle << endl;
    if(max_angle - min_angle < (M_PI)/2){
      isLeaf = true;
      //region.setIsLeaf(isLeaf);
      //cout << "In isLeaf setting is leaf to true " << endl; 
    }
    return isLeaf;
  } 

  
			   
 //  void learnRegions(vector<Position> *pos_hist, vector< vector<CartesianPoint> > *laser_hist){
 //    vector<Position> positionHis = *pos_hist;
 //    vector < vector <CartesianPoint> > laserHis = *laser_hist;
 //    cout << "In learning regions" << endl;
 //    CartesianPoint current_point;
 //    FORRRegion current_region;
 //    for(int k = 0 ; k < laserHis.size(); k++){
 //      vector <CartesianPoint> laserEndpoints = laserHis[k];
 //      Position current_position = positionHis[k];

 //      double radius = 10000;
 //      int direction = -1;
 //      for(int i = 0; i< laserEndpoints.size(); i++){
	// //cout << "wall distance " << wallDistanceVector[i] << endl;
	// double range = laserEndpoints[i].get_distance(CartesianPoint(current_position.getX(),current_position.getY()));
	// if (range < radius and range >= 0.1){
	//   radius = range;
	//   direction = i;
	// }
 //      }
     
 //      current_point = CartesianPoint(current_position.getX(), current_position.getY());
 //      current_region = FORRRegion(current_point, laserEndpoints, radius);
 //      for(int j = k+1 ; j < laserHis.size(); j++){
	// vector <CartesianPoint> nextLaserEndpoints = laserHis[j];
	// Position next_position = positionHis[j];
	// // if next position in still inside the current_region update current_region radius
	// if(current_region.inRegion(next_position.getX(), next_position.getY()) && j != k){
	//   double next_radius = 10000;
	//   int next_direction = -1;
	//   for(int i = 0; i < nextLaserEndpoints.size(); i++){
	//     //cout << "wall distance " << wallDistanceVector[i] << endl;
	//     double next_range = nextLaserEndpoints[i].get_distance(CartesianPoint(next_position.getX(),next_position.getY()));
	//     if (next_range < next_radius){
	//       next_radius = next_range;
	//       next_direction = i;
	//     }
	//   }
 //          // current_region.addLaser(nextLaserEndpoints);
 //          current_region.adjustVisibility(CartesianPoint(next_position.getX(), next_position.getY()), nextLaserEndpoints);
	//   double x = nextLaserEndpoints[next_direction].get_x();
	//   double y = nextLaserEndpoints[next_direction].get_y();
	//   double dist = current_region.getCenter().get_distance(CartesianPoint(x , y));
	//   if(dist < current_region.getRadius() and dist >= 0.1)
	//     current_region.setRadius(dist);
	// } 
 //      }
      
 // //      // check if the robot is in a previously created region
 // //      int robotRegion = -1;
 // //      for(int i = 0; i < regions.size(); i++){
	// // if(regions[i].inRegion(current_point.get_x(), current_point.get_y()))
	// //   robotRegion = i;
 // //      }
 // //      // correct previously create region 
 // //      if(robotRegion != -1){
 // //        // regions[robotRegion].addLaser(laserEndpoints);
 // //        regions[robotRegion].adjustVisibility(current_point, laserEndpoints);
	// // double x = laserEndpoints[direction].get_x();
	// // double y = laserEndpoints[direction].get_y();
	// // double dist = regions[robotRegion].getCenter().get_distance(CartesianPoint(x,y));
	// // if(dist < regions[robotRegion].getRadius())
	// //   regions[robotRegion].setRadius(dist);
 // //      }
      
 //      bool new_region = true;
 // //      // if there is atleast one intersecting region which is bigger than the current region , dont add the current region
 // //      for(int i = 0; i < regions.size(); i++){
	// // if(current_region.doIntersect(regions[i]) == true)
	// //   if(current_region.getRadius() < regions[i].getRadius()){
	// //     new_region = false;
	// //   }
 // //      }
      
 //      //cout << "Current region " << current_region.getCenter().get_x() << "," << current_region.getCenter().get_y() << ":" << current_region.getRadius() << endl;
 //      //cout << "New region = " << new_region << endl; 
      
 //      if(new_region == true){
	// // save the region
	// //cout << "current_region is a new region , adding it into the list" << endl;
	
	// // // delete all intersecting regions
	// // for(int i = 0; i < regions.size() ; i++){
	// //   //cout << "for region " << i << endl;
	// //   if(current_region.doIntersect(regions[i])){
	// //     //cout << "Deleting region:"  << regions[i].getCenter().get_x() << " " << regions[i].getCenter().get_y() << " " << regions[i].getRadius() << endl;
	// //     // current_region.addLasers(regions[i].getLasers());
 // //            current_region.mergeVisibility(regions[i].getVisibility());
 // //            regions.erase(regions.begin() + i);
	// //     i--;
	// //   }
	// // }
	// regions.push_back(current_region);
 //      }
 //    }
 //    cout << "Removing overlapping regions" << endl;
 //    int num_intersections = 0;
 //    int i = 0;
 //    while(i < regions.size()){
 //      cout << "Region " << i << endl;
 //      vector<int> removals;
 //      for(int j = i+1; j < regions.size(); j++){
 //        if(regions[i].doIntersect(regions[j])){
 //          cout << "Intersection with region " << j << endl;
 //          num_intersections++;
 //          if(regions[i].inRegion(regions[j].getCenter())){
 //            cout << "Center inside" << endl;
 //            removals.push_back(j);
 //          }
 //          else{
 //            cout << "Center outside, adjust radius" << endl;
 //            double dist_between_centers = regions[i].getCenter().get_distance(regions[j].getCenter());
 //            cout << "dist_between_centers " << dist_between_centers << " first radius " << regions[i].getRadius() << " second new radius " << dist_between_centers - regions[i].getRadius() - 0.1 << endl;
 //            if(dist_between_centers - regions[i].getRadius() - 0.1 < 0.1){
 //              cout << "new radius too small, remove" << endl;
 //              removals.push_back(j);
 //            }
 //            else{
 //              regions[j].setRadius(dist_between_centers - regions[i].getRadius() - 0.01);
 //            }
 //          }
 //        }
 //      }
 //      cout << "num_intersections " << num_intersections << endl;
 //      for(int j = removals.size()-1; j >= 0; j--){
 //        cout << "Removing region " << removals[j] << endl;
 //        regions.erase(regions.begin()+removals[j]);
 //      }
 //      i++;
 //      num_intersections = 0;
 //    }
 //    cout << "Exit learning regions" << endl;
 //  }


  void learnRegions(vector<Position> *pos_hist, vector< vector<CartesianPoint> > *laser_hist){
    vector<Position> positionHis = *pos_hist;
    vector < vector <CartesianPoint> > laserHis = *laser_hist;
    cout << "In learning regions" << endl;
    CartesianPoint current_point;
    FORRRegion current_region;
    for(int k = 0 ; k < laserHis.size(); k++){
      vector <CartesianPoint> laserEndpoints = laserHis[k];
      Position current_position = positionHis[k];

      double radius = 10000;
      int direction = -1;
      for(int i = 0; i< laserEndpoints.size(); i++){
        //cout << "wall distance " << wallDistanceVector[i] << endl;
        double range = laserEndpoints[i].get_distance(CartesianPoint(current_position.getX(),current_position.getY()));
        if (range < radius and range >= 0.1){
          radius = range;
          direction = i;
        }
      }
      current_point = CartesianPoint(current_position.getX(), current_position.getY());
      current_region = FORRRegion(current_point, laserEndpoints, radius);
      for(int j = k+1 ; j < laserHis.size(); j++){
        vector <CartesianPoint> nextLaserEndpoints = laserHis[j];
        Position next_position = positionHis[j];
        // if next position in still inside the current_region update current_region radius
        if(current_region.inRegion(next_position.getX(), next_position.getY()) && j != k){
          double next_radius = 10000;
          int next_direction = -1;
          for(int i = 0; i < nextLaserEndpoints.size(); i++){
            //cout << "wall distance " << wallDistanceVector[i] << endl;
            double next_range = nextLaserEndpoints[i].get_distance(CartesianPoint(next_position.getX(),next_position.getY()));
            if (next_range < next_radius){
              next_radius = next_range;
              next_direction = i;
            }
          }
          // current_region.addLaser(nextLaserEndpoints);
          current_region.adjustVisibility(CartesianPoint(next_position.getX(), next_position.getY()), nextLaserEndpoints);
          double x = nextLaserEndpoints[next_direction].get_x();
          double y = nextLaserEndpoints[next_direction].get_y();
          double dist = current_region.getCenter().get_distance(CartesianPoint(x , y));
          if(dist < current_region.getRadius() and dist >= 0.1)
            current_region.setRadius(dist);
        }
      }
      
      // check if the robot is in a previously created region
      int robotRegion = -1;
      for(int i = 0; i < regions.size(); i++){
        if(regions[i].inRegion(current_point.get_x(), current_point.get_y()))
          robotRegion = i;
      }
      // correct previously create region 
      if(robotRegion != -1){
        // regions[robotRegion].addLaser(laserEndpoints);
        regions[robotRegion].adjustVisibility(current_point, laserEndpoints);
        double x = laserEndpoints[direction].get_x();
        double y = laserEndpoints[direction].get_y();
        double dist = regions[robotRegion].getCenter().get_distance(CartesianPoint(x,y));
        if(dist < regions[robotRegion].getRadius())
          regions[robotRegion].setRadius(dist);
      }
      
      bool new_region = true;
      // if there is atleast one intersecting region which is bigger than the current region , dont add the current region
      for(int i = 0; i < regions.size(); i++){
        if(current_region.doIntersect(regions[i]) == true)
          if(current_region.getRadius() < regions[i].getRadius()){
            new_region = false;
          }
      }
      
      //cout << "Current region " << current_region.getCenter().get_x() << "," << current_region.getCenter().get_y() << ":" << current_region.getRadius() << endl;
      //cout << "New region = " << new_region << endl; 
      
      if(new_region == true){
        // save the region
        //cout << "current_region is a new region , adding it into the list" << endl;
        
        // delete all intersecting regions
        for(int i = 0; i < regions.size() ; i++){
          //cout << "for region " << i << endl;
          if(current_region.doIntersect(regions[i])){
            //cout << "Deleting region:"  << regions[i].getCenter().get_x() << " " << regions[i].getCenter().get_y() << " " << regions[i].getRadius() << endl;
            // current_region.addLasers(regions[i].getLasers());
            current_region.mergeVisibility(regions[i].getVisibility());
            regions.erase(regions.begin() + i);
            i--;
          }
        }
        regions.push_back(current_region);
      }
    }
    cout << "Exit learning regions"<<endl;
  }

  
  
  
  void learnExits(vector< vector<CartesianPoint> > run_trace, vector <int> trace_inds, vector< vector < vector<CartesianPoint> > > laser_trace){
    // learning gates between different regions
    //clearAllExits();
    // for every position in the position history vector .. check if a move is from one region to another and save it as gate
    //cout << "In learning exits: size of trace is " << run_trace.size() << endl;
    for(int k = 0; k < run_trace.size() ; k++){
      vector<CartesianPoint> history = run_trace[k];
      vector < vector<CartesianPoint> > laser_history = laser_trace[k];
      vector<CartesianPoint> stepped_history;
      vector<int> step_to_trace;
      //cout << "Learning exits between regions" << endl;
      int region_id=-1, previous_position_region_id=-1,  begin_region_id, end_region_id, begin_position , end_position;
      bool beginFound = false;
      double step_size = 0.1;
      for(int j = 0; j < history.size()-1; j++){
        double tx,ty;
        for(double step = 0; step <= 1; step += step_size){
          tx = (history[j].get_x() * (1-step)) + (history[j+1].get_x() * (step));
          ty = (history[j].get_y() * (1-step)) + (history[j+1].get_y() * (step));
          stepped_history.push_back(CartesianPoint(tx,ty));
          step_to_trace.push_back(j);
        }
      }
      for (int j = 0; j < stepped_history.size(); j++){
        //cout << stepped_history[j].get_x() << " " << stepped_history[j].get_y() << endl;
        previous_position_region_id = region_id;
        region_id = pointInRegions(stepped_history[j].get_x(), stepped_history[j].get_y());
        //cout << "previous region id : " << previous_position_region_id << endl;
        //cout << "current region id : " << region_id << endl;
        if(region_id == -1){
          //cout << "skipping position" << endl;
          continue;
        }
        // either we have not found a starting region or 
        if(region_id != -1 && (previous_position_region_id == region_id || beginFound == false)){
          beginFound = true;
          begin_region_id = region_id;
          begin_position = j;
          // cout << "Starting position : " << begin_position << endl;
          continue;
        }
        if(region_id != -1 && region_id != begin_region_id && beginFound == true){
          beginFound = false;
          end_region_id = region_id;
          end_position = j;
          CartesianPoint midpoint = stepped_history[(int)((begin_position+end_position)/2)];
          cout << "Starting position : " << begin_position << " Middle Position : " << (int)((begin_position+end_position)/2) << " Ending Position : " << end_position << endl;
          vector<CartesianPoint> pathBetweenRegions;
          vector< vector<CartesianPoint> > laserBetweenRegions;
          double connectionBetweenRegions = 0;
          for(int m = begin_position; m < end_position; m++){
            // connectionBetweenRegions += stepped_history[m].get_distance(stepped_history[m+1]);
            cout << "step " << m << " step_to_trace " << step_to_trace[m] << endl;
            pathBetweenRegions.push_back(history[step_to_trace[m]]);
            laserBetweenRegions.push_back(laser_history[step_to_trace[m]]);
          }

          // Initialize trail vectors
          std::vector<CartesianPoint> trailPositions;
          // std::vector< vector<CartesianPoint> > trailLaserEndpoints;
          // Push first point in path to trail
          trailPositions.push_back(pathBetweenRegions[0]);
          // trailLaserEndpoints.push_back(laserBetweenRegions[0]);
          // Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
          for(int i = 0; i < pathBetweenRegions.size(); i++){
            for(int n = pathBetweenRegions.size()-1; n > i; n--){
              if(canAccessPoint(laserBetweenRegions[i], pathBetweenRegions[i], pathBetweenRegions[n], 5)) {
                trailPositions.push_back(pathBetweenRegions[n]);
                // trailLaserEndpoints.push_back(laserBetweenRegions[n]);
                i = n-1;
              }
            }
          }
          if (pathBetweenRegions[pathBetweenRegions.size()-1].get_x() != trailPositions[trailPositions.size()-1].get_x() or pathBetweenRegions[pathBetweenRegions.size()-1].get_y() != trailPositions[trailPositions.size()-1].get_y()) {
            trailPositions.push_back(pathBetweenRegions.back());
            // trailLaserEndpoints.push_back(laserBetweenRegions.back());
          }
          cout << "Length of trail " << trailPositions.size() << endl;
          for(int i = 0; i < trailPositions.size()-1; i++){
            cout << trailPositions[i].get_x() << " " << trailPositions[i].get_y() << endl;
            connectionBetweenRegions += trailPositions[i].get_distance(trailPositions[i+1]);
          }
          cout << trailPositions[trailPositions.size()-1].get_x() << " " << trailPositions[trailPositions.size()-1].get_y() << endl;
          cout << "Distance of connection : " << connectionBetweenRegions << endl;
          saveExit(stepped_history[begin_position], stepped_history[begin_position+1], begin_region_id, midpoint, stepped_history[end_position-1] , stepped_history[end_position] , end_region_id, connectionBetweenRegions, trace_inds[k], trailPositions);
        }
      }
    }
    for(int i = 0; i< regions.size(); i++){
      regions[i].print();
    }
  }

  void learnRegionsAndExits(vector<Position> *pos_hist, vector< vector<CartesianPoint> > *laser_hist, vector< vector<CartesianPoint> > run_trace, vector< vector < vector<CartesianPoint> > > laser_trace){
    vector<Position> positionHis = *pos_hist;
    vector < vector <CartesianPoint> > laserHis = *laser_hist;
    cout << "In learning regions and exits" << endl;
    // vector <FORRRegion> new_regions;
    // vector <FORRRegion> regions_to_remove;
    vector <int> regions_to_remove_inds;
    CartesianPoint current_point;
    FORRRegion current_region;
    for(int k = 0 ; k < laserHis.size(); k++){
      vector <CartesianPoint> laserEndpoints = laserHis[k];
      Position current_position = positionHis[k];

      double radius = 10000;
      int direction = -1;
      for(int i = 0; i< laserEndpoints.size(); i++){
        //cout << "wall distance " << wallDistanceVector[i] << endl;
        double range = laserEndpoints[i].get_distance(CartesianPoint(current_position.getX(),current_position.getY()));
        if (range < radius and range >= 0.1){
          radius = range;
          direction = i;
        }
      }
      current_point = CartesianPoint(current_position.getX(), current_position.getY());
      current_region = FORRRegion(current_point, laserEndpoints, radius);
      for(int j = k+1 ; j < laserHis.size(); j++){
        vector <CartesianPoint> nextLaserEndpoints = laserHis[j];
        Position next_position = positionHis[j];
        // if next position in still inside the current_region update current_region radius
        if(current_region.inRegion(next_position.getX(), next_position.getY()) && j != k){
          double next_radius = 10000;
          int next_direction = -1;
          for(int i = 0; i < nextLaserEndpoints.size(); i++){
            //cout << "wall distance " << wallDistanceVector[i] << endl;
            double next_range = nextLaserEndpoints[i].get_distance(CartesianPoint(next_position.getX(),next_position.getY()));
            if (next_range < next_radius){
              next_radius = next_range;
              next_direction = i;
            }
          }
          // current_region.addLaser(nextLaserEndpoints);
          current_region.adjustVisibility(CartesianPoint(next_position.getX(), next_position.getY()), nextLaserEndpoints);
          double x = nextLaserEndpoints[next_direction].get_x();
          double y = nextLaserEndpoints[next_direction].get_y();
          double dist = current_region.getCenter().get_distance(CartesianPoint(x , y));
          if(dist < current_region.getRadius() and dist >= 0.1)
            current_region.setRadius(dist);
        }
      }
      
      // check if the robot is in a previously created region
      int robotRegion = -1;
      for(int i = 0; i < regions.size(); i++){
        if(regions[i].inRegion(current_point.get_x(), current_point.get_y()))
          robotRegion = i;
      }
      // correct previously create region 
      if(robotRegion != -1){
        // regions[robotRegion].addLaser(laserEndpoints);
        regions[robotRegion].adjustVisibility(current_point, laserEndpoints);
        double x = laserEndpoints[direction].get_x();
        double y = laserEndpoints[direction].get_y();
        double dist = regions[robotRegion].getCenter().get_distance(CartesianPoint(x,y));
        if(dist < regions[robotRegion].getRadius())
          regions[robotRegion].setRadius(dist);
      }
      
      bool new_region = true;
      // if there is atleast one intersecting region which is bigger than the current region , dont add the current region
      for(int i = 0; i < regions.size(); i++){
        if(current_region.doIntersect(regions[i]) == true)
          if(current_region.getRadius() < regions[i].getRadius()){
            new_region = false;
          }
      }
      
      //cout << "Current region " << current_region.getCenter().get_x() << "," << current_region.getCenter().get_y() << ":" << current_region.getRadius() << endl;
      //cout << "New region = " << new_region << endl; 
      
      if(new_region == true){
        // save the region
        cout << "current_region is a new region , adding it into the list" << endl;
        
        // delete all intersecting regions
        for(int i = 0; i < regions.size() ; i++){
          cout << "for region " << i << endl;
          if(current_region.doIntersect(regions[i])){
            cout << "Deleting region:"  << regions[i].getCenter().get_x() << " " << regions[i].getCenter().get_y() << " " << regions[i].getRadius() << endl;
            // current_region.addLasers(regions[i].getLasers());
            current_region.mergeVisibility(regions[i].getVisibility());
            // regions_to_remove.push_back(regions[i]);
            if(find(regions_to_remove_inds.begin(), regions_to_remove_inds.end(), i) == regions_to_remove_inds.end()) {
              regions_to_remove_inds.push_back(i);
              cout << "Region i " << i << endl;
            }
            // regions.erase(regions.begin() + i);
            // i--;
          }
        }
        regions.push_back(current_region);
      }
    }
    cout << "regions " << regions.size() << " regions_to_remove_inds " << regions_to_remove_inds.size() << endl;
    sort (regions_to_remove_inds.begin(), regions_to_remove_inds.end());
    for(int i = 0; i < regions_to_remove_inds.size(); i++){
      cout << regions_to_remove_inds[i] << endl;
    }
    //remove exits from regions connected to removed regions
    for(int i = 0; i < regions.size(); i++){
      for(int j = 0; j < regions_to_remove_inds.size(); j++){
        regions[i].removeExitsToRegion(regions_to_remove_inds[j]);
      }
      for(int j = 0; j < regions_to_remove_inds.size(); j++){
        regions[i].fixExitReferences(regions_to_remove_inds[j]);
      }
    }
    set <int> paths_for_removed;
    //save paths for removed exits
    for(int i = 0; i < regions_to_remove_inds.size(); i++){
      vector<FORRExit> region_exits = regions[regions_to_remove_inds[i]].getExits();
      for(int j = 0; j < region_exits.size(); j++){
        paths_for_removed.insert(region_exits[j].getConnectionPath());
        cout << "region " << regions_to_remove_inds[i] << " path " << region_exits[j].getConnectionPath() << endl;
      }
    }
    cout << "paths_for_removed " << paths_for_removed.size() << endl;
    //remove regions
    for(int i=regions_to_remove_inds.size() - 1; i >= 0; i--){
      regions.erase(regions.begin() + regions_to_remove_inds[i]);
    }
    cout << "regions " << regions.size() << endl;
    //add new regions
    // for(int i = 0; i < new_regions.size(); i++){
    //   regions.push_back(new_regions[i]);
    // }
    //run paths from removed exits and new path
    paths_for_removed.insert(run_trace.size()-1);
    cout << "paths_for_removed " << paths_for_removed.size() << endl;
    vector< vector<CartesianPoint> > selected_traces;
    vector< vector < vector<CartesianPoint> > > selected_laser_traces;
    vector <int> selected_inds;
    set<int>::iterator it = paths_for_removed.begin();
    while (it != paths_for_removed.end())
    {
      selected_traces.push_back(run_trace[(*it)]);
      selected_laser_traces.push_back(laser_trace[(*it)]);
      selected_inds.push_back((*it));
      cout << "path " << (*it) << endl;
      it++;
    }
    learnExits(selected_traces, selected_inds, selected_laser_traces);
    cout << "Exit learning regions and exits" << endl;
  }

  void clearAllExits(){
    for(int i = 0; i < regions.size() ; i++){
      regions[i].clearExits();
    }
  }

  void saveExit(CartesianPoint begin1, CartesianPoint begin2, int begin_region, CartesianPoint midpoint, CartesianPoint end1, CartesianPoint end2, int end_region, double connection_length, int connection_path, vector<CartesianPoint> connection_points){
    //CartesianPoint exit_begin_p = getPointOnRegion(begin2, begin_region);
    //CartesianPoint exit_end_p = getPointOnRegion(end1, end_region);
    
    CartesianPoint exit_begin_p = getCrossingPointOnRegion(begin1, begin2, begin_region);
    CartesianPoint exit_end_p = getCrossingPointOnRegion(end1, end2, end_region);
    cout << begin1.get_x() << " " << begin1.get_y() << " " << begin2.get_x() << " " << begin2.get_y() << " " << regions[begin_region].getCenter().get_x() << " " << regions[begin_region].getCenter().get_y() << " " << regions[begin_region].getRadius() << " " << exit_begin_p.get_x() << " " << exit_begin_p.get_y() << endl;
    cout << end1.get_x() << " " << end1.get_y() << " " << end2.get_x() << " " << end2.get_y() << " " << regions[end_region].getCenter().get_x() << " " << regions[end_region].getCenter().get_y() << " " << regions[end_region].getRadius() << " " << exit_end_p.get_x() << " " << exit_end_p.get_y() << endl;
    cout << midpoint.get_x() << " " << midpoint.get_y() << endl;
    //cout << "End exit begin/end" << endl;

    //cout << "In save exit  " << "Start region : " << begin_region << "End region : " << end_region << endl;
    FORRExit exit_begin (exit_begin_p, midpoint, exit_end_p, end_region, connection_length, connection_path, connection_points);
    FORRExit exit_end (exit_end_p, midpoint, exit_begin_p, begin_region, connection_length, connection_path, connection_points);
    if(!regions[begin_region].isExitAlreadyPresent(exit_begin))
       regions[begin_region].addExit(exit_begin);
    if(!regions[end_region].isExitAlreadyPresent(exit_end))
      regions[end_region].addExit(exit_end);
    //cout << exit_begin_p.get_x() << " " << exit_begin_p.get_y() << " " ;
    //cout << exit_end_p.get_x() << " " << exit_end_p.get_y() << endl;
  }

  CartesianPoint getPointOnRegion(CartesianPoint pos, int region_id){
    CartesianPoint center = regions[region_id].getCenter();
    double radius = regions[region_id].getRadius();
    double angle = atan2(pos.get_y() - center.get_y(), pos.get_x() - center.get_x()); 
    return CartesianPoint (center.get_x() + (radius * cos(angle)), center.get_y() + (radius * sin(angle))); 
  }

  CartesianPoint getCrossingPointOnRegion(CartesianPoint pos1, CartesianPoint pos2, int region_id){
    Circle region = Circle(regions[region_id].getCenter(), regions[region_id].getRadius());
    return intersection_point(region, LineSegment(pos1, pos2));
  }

  // returns the position of the region in which the point is located
  int pointInRegions(double x, double y){
    for(int i = 0; i < regions.size(); i++){
      if(regions[i].inRegion(x, y))
        return i;
    }
    return -1;
  }

  
  
 private:
  vector<FORRRegion> regions;
};


#endif
