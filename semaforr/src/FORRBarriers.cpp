#include<FORRBarriers.h>

using namespace std;



//----------------------//------------------------//



void FORRBarriers::CreateSegments(vector<LineSegment> &segments, vector<CartesianPoint> position_history, vector < vector <CartesianPoint> > laser_history) {
  cout << "num of pos history " << position_history.size() << " num of laser history " << laser_history.size() << endl;
  for (int i = 0; i < laser_history.size(); i+=2){
    cout << "num of laser sensors " << laser_history[i].size() << endl;
    for (int j = 0; j < laser_history[i].size()-1; j++){
      LineSegment current_segment = LineSegment(laser_history[i][j], laser_history[i][j+1]);
      if(laser_history[i][j].get_distance(position_history[i]) <= 5 and laser_history[i][j+1].get_distance(position_history[i]) <= 5 and laser_history[i][j].get_distance(laser_history[i][j+1]) <= 1){
        segments.push_back(current_segment);
      }
    }
  }
  cout << "num of segments " << segments.size() << endl;
}



//----------------------//------------------------//

void FORRBarriers::ListSimilarities(vector<vector<double> > &similarities, const vector<LineSegment> &laser_segments) {
  vector<double> segments_and_similarities;

  for(int i = 0; i < laser_segments.size() - 1; i++) {
    for(int j = i + 1; j < laser_segments.size(); j++) {
      segments_and_similarities.push_back(double(i));
      segments_and_similarities.push_back(double(j));
      //cout << i << " " << j << endl;
      segments_and_similarities.push_back(ComputeDistance(laser_segments[i], laser_segments[j]));
      if(segments_and_similarities[2] <= 1){
        similarities.push_back(segments_and_similarities);
      }
      segments_and_similarities.clear();
    }
  }
}

double FORRBarriers::ComputeDistance(LineSegment first_segment, LineSegment second_segment) {
  CartesianPoint intersection_point;
  //cout << "First = " << first_segment.get_endpoints().first << " " << first_segment.get_endpoints().second << " Second = " << second_segment.get_endpoints().first << " " << second_segment.get_endpoints().second << endl;
  double dist;
  if(do_intersect(first_segment, second_segment, intersection_point)){
    dist = 0;
    //cout << "Segments intersect dist = 0" << endl;
  }
  else{
    double first_left_dist = distance(first_segment.get_endpoints().first, second_segment);
    double first_right_dist = distance(first_segment.get_endpoints().second, second_segment);
    double second_left_dist = distance(second_segment.get_endpoints().first, first_segment);
    double second_right_dist = distance(second_segment.get_endpoints().second, first_segment);
    //cout << first_left_dist << " " << first_right_dist << " " << second_left_dist << " " << second_right_dist << endl;
    dist = min(min(first_left_dist,first_right_dist),min(second_left_dist,second_right_dist));
    //cout << "Segments don't intersect dist = " << dist << endl;
  }

  double angledist = first_segment.get_slope() - second_segment.get_slope();
  //cout << "Initial angledist = " << angledist << endl;
  if(angledist > M_PI/2){
    angledist = angledist - M_PI;
  }
  else if(angledist < -M_PI/2){
    angledist = angledist + M_PI;
  }
  angledist = abs(angledist);
  //cout << "Final angledist = " << angledist << endl;
  double sum = (dist + angledist)/2;
  //cout << "Sum = " << sum << endl;
  return sum;
}



//----------------------//------------------------//



void FORRBarriers::FindMostSimilarSegments(vector<vector<double> > &most_similar, const vector<vector<double> > &similarities) {
  double average_of_distances, sum_of_distances, std, sum_of_squared_differences, threshold = 0;

  for(int i = 0; i < similarities.size(); i++) {
    sum_of_distances += similarities[i][2]; // similarity score
  }
  average_of_distances = sum_of_distances / similarities.size();
  cout << "average_of_distances = " << average_of_distances << endl;
  for(int i = 0; i < similarities.size(); i++) {
    sum_of_squared_differences += pow((similarities[i][2]-average_of_distances), 2);
  }

  double normalized_sum_of_squared_differences = sum_of_squared_differences/(similarities.size());
  std = pow(normalized_sum_of_squared_differences, .5); // square root of squared difference sum
  cout << "std = " << std << endl;
  double deviations = 3.5;
  if(isinf(std) == false){
    while(most_similar.size() == 0 and deviations > 0){
      deviations = deviations-0.25;
      threshold = average_of_distances - (deviations*std);
      cout << "threshold " << threshold << endl;
      for(int i = 0; i < similarities.size(); i++) {
        //cout << similarities[i][2] << endl;
        if(similarities[i][2] <= threshold) {
          vector<double> similar_pairing;
          similar_pairing.push_back(similarities[i][0]);
          similar_pairing.push_back(similarities[i][1]);
          most_similar.push_back(similar_pairing);
          //cout << similarities[i][0] << " " << similarities[i][1] << " " << similarities[i][2] << endl;
          similar_pairing.clear();
        }
      }
    }
  }
}



//----------------------//------------------------//



void FORRBarriers::CreateInitialSegments(vector<LineSegment> &initial_barriers,const vector<vector<double> > &most_similar,const vector<LineSegment> &segments) {
  vector<set<double> > all_similar;
  set<int> used_inds;
  for(int i = 0; i < most_similar.size() - 1; i++) {
    if(used_inds.count(i) == 0){
      //cout << most_similar[i][0] << " " << most_similar[i][1] << endl;
      set<double> associated_values;
      associated_values.insert(most_similar[i][0]);
      associated_values.insert(most_similar[i][1]);
      vector<double> additions;
      additions.push_back(most_similar[i][0]);
      additions.push_back(most_similar[i][1]);
      used_inds.insert(i);
      //cout << "remaining additions " << additions.size() << endl;
      while(additions.size()>0){
        for(int j = i + 1; j < most_similar.size(); j++) {
          if(used_inds.count(j) == 0){
            if(most_similar[j][0] == additions[0]){
              //cout << most_similar[j][1] << endl;
              if(associated_values.count(most_similar[j][1]) == 0){
                additions.push_back(most_similar[j][1]);
                associated_values.insert(most_similar[j][1]);
                used_inds.insert(j);
              }
            }
            if(most_similar[j][1] == additions[0]){
              //cout << most_similar[j][0] << endl;
              if(associated_values.count(most_similar[j][1]) == 0){
                additions.push_back(most_similar[j][0]);
                associated_values.insert(most_similar[j][0]);
                used_inds.insert(j);
              }
            }
          }
        }
        additions.erase(additions.begin());
        //cout << "remaining additions " << additions.size() << endl;
      }
      all_similar.push_back(associated_values);
      associated_values.clear();
      additions.clear();
    }
  }
  cout << "all_similar size " << all_similar.size() << " used_inds size " << used_inds.size() << endl;

}


//----------------------//------------------------//


LineSegment FORRBarriers::MergeSegments(LineSegment first_segment, LineSegment second_segment) {
  
}


//----------------------//------------------------//

void FORRBarriers::MergeNearbyBarriers(vector<LineSegment> &merged_barriers, const vector<LineSegment> &initial_barriers) {
  merged_barriers = initial_barriers;
}