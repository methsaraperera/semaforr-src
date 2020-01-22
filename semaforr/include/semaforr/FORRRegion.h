/************************************************
FORRRegion.h 
This file contains the class FORRRegion which is intended to be a data structure which abstracts the notion of 
open spaces and doors in the maps as regions

Written by Anoop Aroor, 2014
**********************************************/

#ifndef FORRREGION_H
#define FORRREGION_H

#include <iostream>
#include "FORRGeometry.h"
#include "FORRExit.h"
#include <algorithm>
#include <vector>

class FORRRegion{
 public:
  FORRRegion(){};
  FORRRegion(CartesianPoint point, vector <CartesianPoint> lep, double r){
    center = point;
    lasers.push_back(lep);
    this->setRadius(r);
  }

  bool doIntersect(FORRRegion test){
    int buffer = 0;
    if((test.getRadius() + this->getRadius() - distance(test.getCenter(), center)) > buffer)
      return true;
    else 
      return false;
  }

  bool operator==(FORRRegion b){
    bool equal = false;
    if(radius == b.getRadius() && center.get_x() == b.getCenter().get_x() && center.get_y() == b.getCenter().get_y()){
      equal = true;
    }
    return equal;
  }
  
  void clearExits(){
    exits.clear();
  }

  void print(){
    cout << center.get_x() << " " << center.get_y() << " " << this->getRadius();
    for(int i = 0; i < exits.size() ; i++){
      cout << " " << exits[i].getExitPoint().get_x() << " "  << exits[i].getExitPoint().get_y() << " "  << exits[i].getExitRegion();
    }
    cout << endl;
  }

  bool equals(FORRRegion test) { 
    if(test.getCenter().get_x() == center.get_x() && test.getCenter().get_y() == center.get_y() && test.getRadius() == this->getRadius())
      return true;
    else 
      return false;
  }

  bool isExitAlreadyPresent(FORRExit exit){
    for(int i = 0; i < exits.size() ; i++){
      if(exit.getExitPoint().get_x() == exits[i].getExitPoint().get_x() && exit.getExitPoint().get_y() == exits[i].getExitPoint().get_y() && exit.getExitRegion() == exits[i].getExitRegion() && exit.getExitDistance() == exits[i].getExitDistance())
        return true;
    }
    return false;
  }

  bool inRegion(double x, double y){ return (distance(CartesianPoint(x,y),center) < this->getRadius());}

  bool inRegion(CartesianPoint p){ return (distance(p,center) < this->getRadius());}
    
  double distance(CartesianPoint point1, CartesianPoint point2){
    double dy = point1.get_y() - point2.get_y();
    double dx = point1.get_x() - point2.get_x();
    return sqrt((dx*dx) + (dy*dy));
  }

  CartesianPoint getCenter(){ return center;}
  void setCenter(CartesianPoint point) { center = point;}
  
  double getRadius(){ 
    return radius;
  }
  void setRadius(double r){ 
    radius = r;
  }

  void addLaser(vector <CartesianPoint> lep){
    lasers.push_back(lep);
  }

  void addLasers(vector < vector <CartesianPoint> > leps){
    for(int i = 0; i < leps.size(); i++){
      lasers.push_back(leps[i]);
    }
  }

  vector < vector <CartesianPoint> > getLasers(){
    return lasers;
  }

  bool visibleFromRegion(CartesianPoint point, double distanceLimit){
    CartesianPoint laserPos = center;
    double distLaserPosToPoint = laserPos.get_distance(point);
    if(distLaserPosToPoint > distanceLimit){
      return false;
    }
    double point_direction = atan2((point.get_y() - laserPos.get_y()), (point.get_x() - laserPos.get_x()));
    for(int j = 0; j < lasers.size(); j++){
      vector<CartesianPoint> givenLaserEndpoints = lasers[j];
      bool canAccessPoint = false;
      int index = 0;
      double min_angle = 100000;
      for(int i = 0; i < givenLaserEndpoints.size(); i++){
        //ROS_DEBUG_STREAM("Laser endpoint : " << givenLaserEndpoints[i].get_x() << "," << givenLaserEndpoints[i].get_y());
        double laser_direction = atan2((givenLaserEndpoints[i].get_y() - laserPos.get_y()), (givenLaserEndpoints[i].get_x() - laserPos.get_x()));
        if(abs(laser_direction - point_direction) < min_angle){
          //ROS_DEBUG_STREAM("Laser Direction : " << laser_direction << ", Point Direction : " << point_direction);
          min_angle = abs(laser_direction - point_direction);
          index = i;
        }
      }
      while (index-2 < 0){
        index = index + 1;
      }
      while (index+2 > givenLaserEndpoints.size()-1){
        index = index - 1;
      }
      //ROS_DEBUG_STREAM("Min angle : " << min_angle << ", " << index);
      int numFree = 0;
      for(int i = -2; i < 3; i++) {
        double distLaserEndPointToLaserPos = givenLaserEndpoints[index+i].get_distance(laserPos);
        //ROS_DEBUG_STREAM("Distance Laser EndPoint to Laser Pos : " << distLaserEndPointToLaserPos << ", Distance Laser Pos to Point : " << distLaserPosToPoint);
        if (distLaserEndPointToLaserPos > distLaserPosToPoint) {
          numFree++;
        }
      }
      //ROS_DEBUG_STREAM("Number farther than point : " << numFree);
      if (numFree > 3) {
        canAccessPoint = true;
      }
      else{
        continue;
      }
      //else, not visible
      //return canAccessPoint;
      double epsilon = 0.005;
      bool canSeePoint = false;
      double ab = laserPos.get_distance(point);
      for(int i = 0; i < givenLaserEndpoints.size(); i++){
        //ROS_DEBUG_STREAM("Laser endpoint : " << givenLaserEndpoints[i].get_x() << "," << givenLaserEndpoints[i].get_y());
        double ac = laserPos.get_distance(givenLaserEndpoints[i]);
        double bc = givenLaserEndpoints[i].get_distance(point);
        if(((ab + bc) - ac) < epsilon){
          //cout << "Distance vector endpoint visible: ("<<laserEndpoints[i].get_x()<<","<< laserEndpoints[i].get_y()<<")"<<endl; 
          //cout << "Distance: "<<distance_to_point<<endl;
          canSeePoint = true;
          return true;
        }
      }
    }
    return false;
  }

  vector<FORRExit> getExtExits() { return ext_exits; }

  vector<FORRExit> getExits() { return exits;}
  void setExits(vector<FORRExit> exit_points) { exits = exit_points;}
  void addExit(FORRExit exit) {
    exits.push_back(exit);
    ext_exits.push_back(FORRExit(CartesianPoint(2*(exit.getExitPoint().get_x()) - center.get_x(), 2*(exit.getExitPoint().get_y()) - center.get_y()), exit.getExitRegionPoint(), exit.getExitRegion(), exit.getExitDistance()));
  }

  void setIsLeaf(bool leaf){isLeaf = leaf;}
  bool getIsLeaf() {return isLeaf;}

 private:
  CartesianPoint center;
  vector < vector <CartesianPoint> > lasers;
  double radius;
  bool isLeaf;
  // each value in the list denotes a possible exit from the region
  vector<FORRExit> exits;
  vector<FORRExit> ext_exits;
};


#endif
