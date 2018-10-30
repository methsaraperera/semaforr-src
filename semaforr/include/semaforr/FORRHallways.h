/************************************************
FORRHallways.h 
This file contains the class which contains information about the hallways that FORR learns and uses

Written by Raj Korpan, adapted from Sarah Mathew and Gil Dekel, 2018
**********************************************/

#ifndef FORRHALLWAYS_H
#define FORRHALLWAYS_H

#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <cmath>        //for atan2 and M_PI
#include <algorithm>
#include <map>
#include <FORRGeometry.h>

using namespace std;

//=========================================================//=========================================================//

struct Aggregate {
public:
  Aggregate(vector<CartesianPoint> coordinates, int id): points_(coordinates), hallway_type_(id) {}

  vector<CartesianPoint> getPoints() const {return points_;}

  int getHallwayType() const {return hallway_type_;}

  bool pointInAggregate(CartesianPoint point){
    cout << "Inside pointInAggregate" << endl;
    std::vector<CartesianPoint>::iterator it;
    CartesianPoint roundedPoint = CartesianPoint((int)(point.get_x()),(int)(point.get_y()));
    cout << "point x = " << point.get_x() << ", y = " << point.get_y() << "; Rounded point x = " << roundedPoint.get_x() << ", y = " << roundedPoint.get_y() << endl;
    it = find(points_.begin(), points_.end(), roundedPoint);
    if(it != points_.end()){
      cout << "point found in aggregate" << endl;
      return true;
    }
    else{
      cout << "point NOT found in aggregate" << endl;
      return false;
    }
  }

  double distanceToAggregate(CartesianPoint point){
    cout << "Inside distanceToAggregate" << endl;
    std::vector<CartesianPoint>::iterator it;
    CartesianPoint roundedPoint = CartesianPoint((int)(point.get_x()),(int)(point.get_y()));
    cout << "point x = " << point.get_x() << ", y = " << point.get_y() << "; Rounded point x = " << roundedPoint.get_x() << ", y = " << roundedPoint.get_y() << endl;
    it = find(points_.begin(), points_.end(), roundedPoint);
    double dist = 1000000.0;
    if(it != points_.end()){
      cout << "point found in aggregate so distance = 0.0" << endl;
      dist = 0.0;
    }
    else{
      cout << "point NOT found in aggregate" << endl;
      for(int i = 0; i < points_.size(); i++){
        double tempDist = point.get_distance(points_[i]);
        if(tempDist < dist){
          dist = tempDist;
        }
      }
    }
    cout << "distance = " << dist << endl;
    return dist;
  }

  void findConnection(Aggregate &hlwy, int id1, int id2){
    cout << "Inside findConnection" << endl;
    for(int i = 0; i < points_.size(); i++){
      if(hlwy.pointInAggregate(points_[i]) == true){
        connectedHallways.push_back(id2);
        hlwy.addConnection(id1);
        cout << "Connection found between " << id1 << " and " << id2 << endl;
        break;
      }
    }
  }

  void addConnection(int id){
    connectedHallways.push_back(id);
  }

  int numConnections(){
    cout << "Inside numConnections = " << connectedHallways.size() << endl;
    return connectedHallways.size();
  }

  bool isHallwayConnected(int id){
    cout << "Inside isHallwayConnected" << endl;
    for(int i = 0; i < connectedHallways.size(); i++){
      if(connectedHallways[i] == id){
        cout << "Hallways are connected" << endl;
        return true;
      }
    }
    cout << "Hallways are NOT connected" << endl;
    return false;
  }

  double distanceBetweenAggregates(Aggregate hlwy){
    cout << "Inside distanceBetweenAggregates" << endl;
    double dist = 1000000.0;
    for(int i = 0; i < points_.size(); i++){
      double tempDist = hlwy.distanceToAggregate(points_[i]);
      if(tempDist < dist){
        dist = tempDist;
      }
    }
    cout << "Distance between = " << dist << endl;
    return dist;
  }

private:
  vector<CartesianPoint> points_;
  int hallway_type_;
  vector<int> connectedHallways;
};


//=========================================================//=========================================================//

struct Segment {
public:
  Segment(CartesianPoint left, CartesianPoint right){
    left_point_ = left;
    right_point_ = right;
    angle_ = FindAngle();
  }


  void PrintSegment() {
    cout << left_point_.get_x() << " " << left_point_.get_y() << endl;
    cout << right_point_.get_x() << " " << right_point_.get_y() << endl;
    cout << angle_ << endl;
  }

  CartesianPoint GetLeftPoint() const {return left_point_;}
  CartesianPoint GetRightPoint() const {return right_point_;}
  double GetAngle() const {return angle_;}

private:
  CartesianPoint left_point_;
  CartesianPoint right_point_;
  double angle_;

  double FindAngle() {
    CartesianPoint coordinate = TranslateSegment();
    double angle = atan2(coordinate.get_y(), coordinate.get_x());
    angle = MakeAnglePositive(angle);
    return angle;
  }
  CartesianPoint TranslateSegment(const CartesianPoint reference_point = CartesianPoint(0,0)){
    double new_x = right_point_.get_x() - (left_point_.get_x()- reference_point.get_x());
    double new_y = right_point_.get_y() - (left_point_.get_y()- reference_point.get_y());
    CartesianPoint new_coordinate = CartesianPoint(new_x, new_y);
    return new_coordinate;
  }
  double MakeAnglePositive(double angle) {
    if(angle < 0)
      angle += M_PI;
    return angle;
  }

};


//=========================================================//=========================================================//

/* FORRHallways class
 *
 * FORRHallways represent the hallways that are learned by SemaFORR.
 *
 */
class FORRHallways{
public:
    FORRHallways(double wid, double hgt){
        hallways = vector<Aggregate>();
        map_width_ = wid;
        map_height_ = hgt;
    };
    vector<Aggregate> getHallways(){return hallways;}
    int getWidth(){return map_width_;}
    int getHeight(){return map_height_;}
    ~FORRHallways(){};

    void clearAllHallways(){
        hallways.clear();
    }

    void learnHallways(vector< vector<CartesianPoint> > trails_trace) {
        vector< vector<CartesianPoint> > trails_coordinates = trails_trace;

        vector<Segment> trails_segments;
        CreateSegments(trails_segments, trails_coordinates);

        vector<vector<double> > segments_normalized(trails_segments.size(), vector<double>(5,0));
        NormalizeVector(segments_normalized, trails_segments);
        cout << "num of segments normalized " << segments_normalized.size() << endl;

        vector<vector<double> > segments_similarities;
        ListSimilarities(segments_similarities, segments_normalized);
        cout << "num of segments similarities " << segments_similarities.size() << endl;

        vector<vector<double> > most_similar_segments;
        FindMostSimilarSegments(most_similar_segments, segments_similarities);
        cout << "num of most similar segments " << most_similar_segments.size() << endl;

        vector<Segment> mean_segments;
        CreateMeanSegments(mean_segments, most_similar_segments, trails_segments);
        cout << "num of mean_segments " << mean_segments.size() << endl;

        vector<vector<Segment> > hallway_types(4);
        // horizontal_segments //id 1
        // minor_diagonal_segments; // id 2
        // vertical_segments; // id  3
        // major_diagonal_segments; // id 4


        //int id = 0;
        double step = 180/8;
        for(int i = 0; i < mean_segments.size(); i++) {
            double angle = mean_segments[i].GetAngle() * 180/M_PI;
            if(angle < step)
                hallway_types[0].push_back(mean_segments[i]);
            else if(angle < 3*step)
                hallway_types[1].push_back(mean_segments[i]);
            else if(angle < 5*step)
                hallway_types[2].push_back(mean_segments[i]);
            else if(angle < 7*step)
                hallway_types[3].push_back(mean_segments[i]);
            else
                hallway_types[0].push_back(mean_segments[i]);
        }

        int filter_size = 9; // magic number
        cout << "start map" << endl;

        vector<string> hallway_names;
        hallway_names.push_back("horizontal");
        hallway_names.push_back("minor_diagonal");
        hallway_names.push_back("vertical");
        hallway_names.push_back("major_diagonal");
        vector<Aggregate> all_aggregates;
        for(int i = 0; i < hallway_names.size(); i++) {
            vector<vector<CartesianPoint> > hallway_groups = ProcessHallwayData( hallway_types[i], filter_size, map_width_, map_height_);
            cout << "process agg" << endl;
            for(int j = 0; j< hallway_groups.size(); j++) {
                Aggregate group = Aggregate(hallway_groups.at(j), i);
                all_aggregates.push_back(group);
            }
            cout << hallway_groups.size() << endl;
            cout << "done proccessing " << hallway_names[i] << endl;
        }
        cout << "finished map" << endl;
        cout << "finding connections between hallways" << endl;
        for (int i = 0; i < all_aggregates.size()-1; i++){
          for (int j = i + 1; j < all_aggregates.size(); j++){
            all_aggregates[i].findConnection(all_aggregates[j],i,j);
          }
        }
        for (int i = 0; i < all_aggregates.size(); i++){
          all_aggregates[i].numConnections();
        }

        hallways = all_aggregates;
    }

private:
    vector<Aggregate> hallways;
    vector<vector<int> > interpolate;

    int map_height_;
    int map_width_;

    void CreateSegments(vector<Segment> &segments, const vector<vector<CartesianPoint> > &trails);

    void NormalizeVector(vector<vector<double> > &normalized_segments, const vector<Segment> &segments);
    void ConvertSegmentsToDouble(vector<vector<double> > &data, const vector<Segment> &segments);
    double Normalize(const double original_val, const double min_val, const double max_val);
    double FindMin(const vector<vector<double> > &segments, const int pos);
    double FindMax(const vector<vector<double> > &segments, const int pos);

    void ListSimilarities(vector<vector<double> > &similarities,const vector<vector<double> > &normalized_segments);
    double ComputeDistance(const vector<double> first_segment, const vector<double> second_segment);

    void FindMostSimilarSegments(vector<vector<double> > &most_similar,const vector<vector<double> > &similarities);

    void CreateMeanSegments(vector<Segment> &averaged_segments,const vector<vector<double> > &most_similar,const vector<Segment> &segments);

    vector<vector<CartesianPoint> > ProcessHallwayData(const vector<Segment> &hallway_group,int filter_size, int width, int height);
    void UpdateMap(vector<vector<int> > &frequency_map, const vector<Segment> &segments);
    void Interpolate(vector<vector<int> > &frequency_map,double left_x, double left_y, double right_x, double right_y);
    vector<vector<int> > CreateCircularAveragingFilter(int radius);
    vector<vector<double> > ReturnMatlabFilter();
    void FilterImage(vector<vector<double> > &filtered, const vector<vector<int> > &original, int radius);
    void BinarizeImage(vector<vector<int> > &binarized,const vector<vector<int> > &original,  int threshold);
    //void ConvertMatrixToImage(const vector<vector<int> > &binary_map, string image_name);
    void doUnion(int x, int y, int x2, int y2, vector<vector<int> > &labeled_image);
    void unionCoords(int x, int y, int x2, int y2, const vector<vector<int> > &binary_map, vector<vector<int> > &labeled_image);
    void LabelImage(const vector<vector<int> > &binary_map, vector<vector<int> > &labeled_image);
    void ListGroups(vector<vector< pair<int,int> > > &aggregates_and_points, const vector<vector<int> > &labeled_image);
    void ConvertPairToCartesianPoint(vector<vector<CartesianPoint> > &trails, const vector<vector< pair<int,int> > > &input);
};

#endif
