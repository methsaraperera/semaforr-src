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
#include <FORRGeometry.h>

using namespace std;

//=========================================================//=========================================================//

struct Aggregate {
public:
  Aggregate(vector<CartesianPoint> coordinates, int id): points_(coordinates), hallway_type_(id) {}
  vector<CartesianPoint> getPoints() const {return points_;}
  int getHallwayType() const {return hallway_type_;}
private:
  vector<CartesianPoint> points_;
  int hallway_type_;
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

void ConvertDoubleToCartesianPoint(vector<vector<CartesianPoint> > &trails, const vector<vector<double> > &input) {
  for(int i = 0; i < input.size(); i++) {
    vector<CartesianPoint> trail_coordinates;
    for(int j = 0; j < input[i].size(); j+=2) {
      CartesianPoint new_point = CartesianPoint(input[i][j], input[i][j+1]);
      trail_coordinates.push_back(new_point);
    }
    trails.push_back(trail_coordinates);
  }
}

void ConvertPairToCartesianPoint(vector<vector<CartesianPoint> > &trails, const vector<vector< pair<int,int> > > &input) {
  for(int i = 0; i < input.size(); i++){
    vector<CartesianPoint> trail_coordinates;
    for(int j = 0; j < input[i].size(); j++) {
      CartesianPoint new_point = CartesianPoint(input[i][j].first,input[i][j].second);
      trail_coordinates.push_back(new_point);
    }
    trails.push_back(trail_coordinates);
  }
}


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

        vector<vector<double> > segments_similarities;
        ListSimilarities(segments_similarities, segments_normalized);

        vector<vector<double> > most_similar_segments;
        FindMostSimilarSegments(most_similar_segments, segments_similarities);

        vector<Segment> mean_segments;
        CreateMeanSegments(mean_segments, most_similar_segments, trails_segments);


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
        string map_type = "binary_map_name";
        vector<string> output_files;
        for(int i = 0; i< hallway_names.size(); i++)
            output_files.push_back(map_type + "_" + hallway_names.at(i) + ".pgm");
        vector<Aggregate> all_aggregates;
        for(int i = 0; i < hallway_names.size(); i++) {
            vector<vector<CartesianPoint> > hallway_groups = ProcessHallwayData( hallway_types[i], filter_size, map_width_, map_height_, output_files[i]);
            cout << "process agg" << endl;
            for(int j = 0; j< hallway_groups.size(); j++) {
                Aggregate group = Aggregate(hallway_groups.at(j), i);
                all_aggregates.push_back(group);
            }
            cout << hallway_groups.size() << endl;
            cout << "done proccessing" << endl;
        }
        cout << "finished map" << endl;
        //for(auto x: all_aggregates)
        //  x.PrintAggregate();
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


    vector<vector<CartesianPoint> > ProcessHallwayData(const vector<Segment> &hallway_group,int filter_size, int width, int height, string file_name);
    void UpdateMap(vector<vector<int> > &frequency_map, const vector<Segment> &segments);
    void Interpolate(vector<vector<int> > &frequency_map,double left_x, double left_y, double right_x, double right_y);
    vector<vector<int> > CreateCircularAveragingFilter(int radius);
    vector<vector<double> > ReturnMatlabFilter();
    void FilterImage(vector<vector<double> > &filtered, const vector<vector<int> > &original, int radius);
    void BinarizeImage(vector<vector<int> > &binarized,const vector<vector<double> > &original,  int threshold);
    void ConvertMatrixToImage(const vector<vector<int> > &binary_map, string image_name);
};



//----------------------//------------------------//



void FORRHallways::CreateSegments(vector<Segment> &segments, const vector<vector<CartesianPoint> > &trails) {
  for(int i = 0; i < trails.size(); i++) {
    for(int j = 0; j < trails[i].size()-1; j++) {
      Segment current_segment = Segment(trails[i][j], trails[i][j+1]);
      if(trails[i][j].get_x() > trails[i][j+1].get_x())
        current_segment = Segment(trails[i][j+1], trails[i][j]);
      if((trails[i][j].get_x() != trails[i][j+1].get_x()) || (trails[i][j].get_y() != trails[i][j+1].get_y()))
        segments.push_back(current_segment);
    }
  }
}



//----------------------//------------------------//



// find min and max for each row (x0, y0, x1, y1, angle)
void FORRHallways::NormalizeVector(vector<vector<double> > &normalized_segments, const vector<Segment> &segments) {
  vector<vector<double> > segments_data;
  ConvertSegmentsToDouble(segments_data, segments);
  for(int i = 0; i < segments_data[0].size(); i++) {
    double min, max;

    min = FindMin(segments_data, i);
    max = FindMax(segments_data, i);

    //cout << "min: " << min << " " << "max: " << max << endl;

    for(int j = 0; j < segments_data.size(); j++) {
      normalized_segments[j][i] = Normalize(segments_data[j][i], min, max); // be careful with matrix numbering here
      //Normalize(segments_data[j][i], min, max);
    }
  }
}

void FORRHallways::ConvertSegmentsToDouble(vector<vector<double> > &data, const vector<Segment> &segments) {
  vector<double> single_segment_data;
  for(int i = 0; i< segments.size(); i++) {
    single_segment_data.push_back(segments.at(i).GetLeftPoint().get_x());
    single_segment_data.push_back(segments.at(i).GetLeftPoint().get_y());
    single_segment_data.push_back(segments.at(i).GetRightPoint().get_x());
    single_segment_data.push_back(segments.at(i).GetRightPoint().get_y());
    single_segment_data.push_back(segments.at(i).GetAngle());
    data.push_back(single_segment_data);
    single_segment_data.clear();
  }
}

// assumes vector has at least 1 vector w/ 1 value
//TODO:: very similar to FindMax
// how to set functions equal to each other
// to set up 2 boolean conditions for min/max
// reduce this so only a 1-D vector is passed
double FORRHallways::FindMin(const vector<vector<double> > &segments, const int pos) {
  double min = segments[0][pos];
  for(int i = 0; i < segments.size(); i++) {
    if(segments[i][pos] < min)
      min = segments[i][pos];
  }
  return min;
}

// assumes vector has at least 1 vector w/ 1 value
double FORRHallways::FindMax(const vector<vector<double> > &segments, const int pos) {
  double max = segments[0][pos];
  for(int i = 0; i < segments.size(); i++) {
    if(segments[i][pos] > max)
      max = segments[i][pos];
  }
  return max;
}

//input:
//output:
//TODO: what if min = max
double FORRHallways::Normalize(const double original_val, const double min_val, const double max_val) {
  return (original_val - min_val)/(max_val - min_val);
}



//----------------------//------------------------//



void FORRHallways::ListSimilarities(vector<vector<double> > &similarities, const vector<vector<double> > &normalized_segments) {
  vector<double> segments_and_similarities;

  for(int i = 0; i < normalized_segments.size() - 1; i++) {
    for(int j = i + 1; j < normalized_segments.size(); j++) {
      segments_and_similarities.push_back(double(i));
      segments_and_similarities.push_back(double(j));
      segments_and_similarities.push_back(ComputeDistance(normalized_segments[i], normalized_segments[j]));
      similarities.push_back(segments_and_similarities);
      segments_and_similarities.clear();
    }
  }
}

//input: two vectors of 2 coordinates each, type double
//output: sum of squared differences, type double
//TODO:: should name be changed to reflect squared nature?
//TODO:: parameterize nth root
double FORRHallways::ComputeDistance(const vector<double> first_segment, const vector<double> second_segment) {
  double sum = 0;
  for(int i = 0; i < first_segment.size(); i++) {
    sum += pow((first_segment[i] - second_segment[i]), 2);
  }
  sum = pow(sum, .5); //remember it's rooted in matlab function
  return sum;
}



//----------------------//------------------------//



void FORRHallways::FindMostSimilarSegments(vector<vector<double> > &most_similar, const vector<vector<double> > &similarities) {
  double average_of_distances, sum_of_distances, std, sum_of_squared_differences, threshold = 0;

  for(int i = 0; i < similarities.size(); i++) {
    sum_of_distances += similarities[i][2]; // similarity score
  }
  average_of_distances = sum_of_distances / similarities.size();

  for(int i = 0; i < similarities.size(); i++) {
    sum_of_squared_differences += pow((similarities[i][2]-average_of_distances), 2);
  }

  double normalized_sum_of_squared_differences = sum_of_squared_differences/(similarities.size());
  std = pow(normalized_sum_of_squared_differences, .5); // square root of squared difference sum
  threshold = average_of_distances - (2*std);

  for(int i = 0; i < similarities.size(); i++) {
    if(similarities[i][2] <= threshold) {
      vector<double> similar_pairing;
      similar_pairing.push_back(similarities[i][0]);
      similar_pairing.push_back(similarities[i][1]);
      most_similar.push_back(similar_pairing);
      similar_pairing.clear();
    }
  }
}



//----------------------//------------------------//



void FORRHallways::CreateMeanSegments(vector<Segment> &averaged_segments, const vector<vector<double> > &most_similar, const vector<Segment> &segments) {
  double average_left_x, average_left_y, average_right_x, average_right_y = 0;
  for(int i = 0; i < most_similar.size(); i++) {
    Segment left = segments[most_similar.at(i)[0]];
    Segment right = segments[most_similar.at(i)[1]];
    average_left_x = (left.GetLeftPoint().get_x() + right.GetLeftPoint().get_x())/2;
    average_left_y = (left.GetLeftPoint().get_y() + right.GetLeftPoint().get_y())/2;
    CartesianPoint left_coord = CartesianPoint(average_left_x, average_left_y);

    average_right_x = (left.GetRightPoint().get_x() + right.GetRightPoint().get_x())/2;
    average_right_y = (left.GetRightPoint().get_y() + right.GetRightPoint().get_y())/2;
    CartesianPoint right_coord = CartesianPoint(average_right_x, average_right_y);

    Segment average = Segment(left_coord, right_coord);
    averaged_segments.push_back(average);
  }
}

//----------------------//------------------------//

// filtered line is one below than expected pos
// just pass 1 vector of hallway
vector<vector<CartesianPoint> > FORRHallways::ProcessHallwayData(const vector<Segment> &hallway_group,int filter_size, int width, int height, string image_name) {
    vector<vector<int> > heat_map(width,vector<int>(height, 0));
    vector<vector<double> > filtered_heat_map(width,vector<double>(height, 0));
    vector<vector<int> > binarized_heat_map(width,vector<int>(height, 0));


    //cout << hallway_groups[i].size() << endl;
    UpdateMap(heat_map, hallway_group);
    cout << "1a" << endl;

    FilterImage(filtered_heat_map,heat_map,9);

    BinarizeImage(binarized_heat_map, filtered_heat_map, 0);

    //PrintIntVector(binarized_heat_map);

    cout << "2a" << endl;
    ConvertMatrixToImage(binarized_heat_map, image_name);

    string labeled_image = "labeled_" + image_name;
    cout << "3a" << endl;
    LabelImage(image_name, labeled_image);

    cout << "3c" << endl; //error1
    vector<vector< pair<int,int> > > points_in_aggregates;
    ListGroups(points_in_aggregates, labeled_image);
    cout << "3d" << endl;

    vector<vector<CartesianPoint> > points_in_groups;
    ConvertPairToCartesianPoint(points_in_groups, points_in_aggregates);
    cout << "3e" << endl; //error1

    return points_in_groups;
}

// isnt accurate to line
// also slow
void FORRHallways::UpdateMap(vector<vector<int> > &frequency_map, const vector<Segment> &segments) {
  //cout << "map size input: " << frequency_map.size() << " " << frequency_map[0].size() << endl;
  //cout << segments.size() << endl;
  for(int i = 0; i < segments.size(); i++) {
    //cout << i << endl;
    CartesianPoint left = segments[i].GetLeftPoint();
    //cout << i << "left" << endl;
    CartesianPoint right = segments[i].GetRightPoint();
    //cout << i << "right" << endl;
    Interpolate(frequency_map, round(left.get_x()), round(left.get_y()), round(right.get_x()), round(right.get_y()));
    //cout << i <<"Done"<< endl;
  }
}

//********$^$^^%#^#

// does this produce a similar map?
void FORRHallways::Interpolate(vector<vector<int> > &frequency_map, double left_x, double left_y, double right_x, double right_y) {

  double step = abs(left_x - right_x) + abs(left_y - right_y);
  //cout << "step: " << step << " " << 1/step << endl;

  for(double j = 0; j <= 1; j += 1/step) {
    //cout << j << endl;
    int xcoord = round(j*(right_x - left_x) + left_x);
    int ycoord = round(j*(right_y - left_y) + left_y);
    //cout << xcoord << " " << ycoord << endl;
    //cout << frequency_map[0].size() << " " << frequency_map.size() << endl;
    frequency_map[xcoord][ycoord]++;
  }
}


//TODO: MATLAB filter specifically for radius of 9, and all values
vector<vector<int> > FORRHallways::CreateCircularAveragingFilter(int radius) {
  vector<vector<int> > filter((2*radius) + 1,vector<int>((2*radius)+1, 0));
  for(int i = 0; i < filter.size(); i++) {
    for(int j = 0; j < filter[i].size(); j++) {
      if( pow((i-radius), 2) + pow((j-radius), 2)  <= pow(radius,2))
    filter[i][j] = 1;
    }
  }
  return filter;
}

vector<vector<double> > FORRHallways::ReturnMatlabFilter() {
  vector<vector<double> > filter;
  double arr1[19] = {0,0,0,0,0,0,0.00013527,0.0010609,0.0017273,0.0019467,0.0017273,0.0010609,0.00013527,0,0,0,0,0,0};
  filter.push_back(vector<double> v1(arr1, arr1+19));
  double arr2[19] = {0,0,0,0,0.00028092,0.0021842,0.0037149,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0037149,0.0021842,0.00028092,0,0,0,0};
  filter.push_back(vector<double> v1(arr2, arr2+19));
  double arr3[19] = {0,0,0,0.00092058,0.0035515,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0035515,0.00092058,0,0,0};
  filter.push_back(vector<double> v1(arr3, arr3+19));
  double arr4[19] = {0,0,0.00092058,0.0037832,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0037832,0.00092058,0,0};
  filter.push_back(vector<double> v1(arr4, arr4+19));
  double arr5[19] = {0,0.00028092,0.0035515,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0035515,0.00028092,0};
  filter.push_back(vector<double> v1(arr5, arr5+19));
  double arr6[19] = {0,0.0021842,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0021842,0};
  filter.push_back(vector<double> v1(arr6, arr6+19));
  double arr7[19] = {0.00013527,0.0037149,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0037149,0.00013527};
  filter.push_back(vector<double> v1(arr7, arr7+19));
  double arr8[19] = {0.0010609,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0010609};
  filter.push_back(vector<double> v1(arr8, arr8+19));
  double arr9[19] = {0.0017273,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0017273};
  filter.push_back(vector<double> v1(arr9, arr9+19));
  double arr10[19] = {0.0019467,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0019467};
  filter.push_back(vector<double> v1(arr10, arr10+19));
  double arr11[19] = {0.0017273,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0017273};
  filter.push_back(vector<double> v1(arr11, arr11+19));
  double arr12[19] = {0.0010609,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0010609};
  filter.push_back(vector<double> v1(arr12, arr12+19));
  double arr13[19] = {0.00013527,0.0037149,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0037149,0.00013527};
  filter.push_back(vector<double> v1(arr13, arr13+19));
  double arr14[19] = {0,0.0021842,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0021842,0};
  filter.push_back(vector<double> v1(arr14, arr14+19));
  double arr15[19] = {0,0.00028092,0.0035515,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0035515,0.00028092,0};
  filter.push_back(vector<double> v1(arr15, arr15+19));
  double arr16[19] = {0,0,0.00092058,0.0037832,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0037832,0.00092058,0,0};
  filter.push_back(vector<double> v1(arr16, arr16+19));
  double arr17[19] = {0,0,0,0.00092058,0.0035515,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0035515,0.00092058,0,0,0};
  filter.push_back(vector<double> v1(arr17, arr17+19));
  double arr18[19] = {0,0,0,0,0.00028092,0.0021842,0.0037149,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0037149,0.0021842,0.00028092,0,0,0,0};
  filter.push_back(vector<double> v1(arr18, arr18+19));
  double arr19[19] = {0,0,0,0,0,0,0.00013527,0.0010609,0.0017273,0.0019467,0.0017273,0.0010609,0.00013527,0,0,0,0,0,0};
  filter.push_back(vector<double> v1(arr19, arr19+19));
  return filter;
}

// must produce double value matrix


// assumes matrix is odd sized square
// first inner loop includes 0 now
// why is everything one line below?
// and one line over...
void FORRHallways::FilterImage(vector<vector<double> > &filtered, const vector<vector<int> > &original, int radius) {

  //vector<vector<int> > averaging_filter = createCircularAveragingFilter(radius);
  vector<vector<double> > averaging_filter = ReturnMatlabFilter();

  int matrix_radius  = (averaging_filter.size() - 1)/2;
  //create 19x19 vector all 0s

  vector<vector<double> > matrix_piece(averaging_filter.size(), vector<double>(averaging_filter.size(), 0));
  int num_rows = original[0].size();
  int num_columns = original.size();

  for(int i = 0; i < original[0].size(); i++) {
    for(int j = 0; j < original.size(); j++) {

      for(int n = -matrix_radius; n <= matrix_radius; n++) {
        for(int m = -matrix_radius; m <= matrix_radius; m++) {
          if((i+m >= 0 && (i+m <= num_rows-1)) && (j+n >= 0 && (j+n <= num_columns-1))) {
            matrix_piece[n+matrix_radius][m+matrix_radius] = static_cast<double>(original[j+n][i+m]);
          }
          else {
            matrix_piece[n+matrix_radius][m+matrix_radius] = 0;
          }
        }
      }


      double hold = 0;
      for(int k = 0; k < averaging_filter.size(); k++) {
        for(int u = 0; u < averaging_filter.size(); u++) {
          hold += matrix_piece[k][u]*averaging_filter[k][u];
        }
      }
      filtered[j][i] = hold;
    }
  }
}
  // assumes binarized has the same dimensions as original
void FORRHallways::BinarizeImage(vector<vector<int> > &binarized, const vector<vector<double> > &original, int threshold) {
  for(int i = 0; i < original.size(); i++) {
    for(int j = 0; j < original[0].size(); j++) {
      if(original[i][j] > threshold)
        binarized[i][j] = 255;
    }
  }
}

// mapping is still weird...
void FORRHallways::ConvertMatrixToImage(const vector<vector<int> > &binary_map, string file_name) {

  Image output;

  output.AllocateSpaceAndSetSize(binary_map[0].size(), binary_map.size());

  output.SetNumberGrayLevels(1);

  //cout << binary_map[0].size() << ", " << binary_map.size() << endl;
  for (int i = 0; i < binary_map[0].size(); ++i) {
    for (int j = 0; j < binary_map.size(); ++j) {
      output.SetPixel(i,j, binary_map[j][i]);
    }
  }

  WriteImage(file_name, output);

}

#endif
