#include<FORRBarriers.h>

using namespace std;



//----------------------//------------------------//



void FORRBarriers::CreateSegments(vector<LineSegment> &segments, CartesianPoint current_position, vector < vector <CartesianPoint> > laser_history) {
  cout << "num of laser history " << laser_history.size() << endl;
  for (int i = 0; i < laser_history.size(); i++){
    cout << "num of laser sensors " << laser_history[i].size() << endl;
    for (int j = 0; j < laser_history[i].size()-1; j++){
      LineSegment current_segment = LineSegment(laser_history[i][j], laser_history[i][j+1]);
      if(laser_history[i][j].get_distance(current_position) <= 10 and laser_history[i][j+1].get_distance(current_position) <= 10 and laser_history[i][j].get_distance(laser_history[i][j+1]) <= 10){
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
  vector<vector<double> > all_similar;

  for(int i = 0; i < most_similar.size() - 1; i++) {
    vector<double> combined_pairs;
    combined_pairs.push_back(most_similar[i][0]);
    combined_pairs.push_back(most_similar[i][1]);
    cout << combined_pairs[0] << " " << combined_pairs[1] << endl;
    vector<double> potential_additions;
    for(int j = i + 1; j < most_similar.size(); j++) {
      if(most_similar[j][0] == combined_pairs[0] or most_similar[j][0] == combined_pairs[1]){
        potential_additions.push_back(most_similar[j][1]);
        //cout << most_similar[j][1] << endl;
      }
      if(most_similar[j][1] == combined_pairs[0] or most_similar[j][1] == combined_pairs[1]){
        potential_additions.push_back(most_similar[j][0]);
        //cout << most_similar[j][0] << endl;
      }
    }
    double num_changes = potential_additions.size();
    cout << "num_changes = " << num_changes << endl;
    while(num_changes > 0){
      vector<double> new_additions;
      double initial_size = potential_additions.size();
      cout << "initial_size = " << initial_size << endl;
      for(int k = 0; k < potential_additions.size(); k++){
        for(int j = i + 1; j < most_similar.size(); j++) {
          if(most_similar[j][0] == potential_additions[k]){
            ///////NEED TO CHECK IF ALREADY THERE
            new_additions.push_back(most_similar[j][1]);
            //cout << most_similar[j][1] << endl;
          }
          if(most_similar[j][1] == potential_additions[k]){
            ///////NEED TO CHECK IF ALREADY THERE
            new_additions.push_back(most_similar[j][0]);
            //cout << most_similar[j][0] << endl;
          }
        }
        ///////NEED TO CHECK IF ALREADY THERE
        combined_pairs.push_back(potential_additions[k]);
        //cout << potential_additions[k] << endl;
      }
      potential_additions = new_additions;
      num_changes = potential_additions.size();
      cout << "num_changes = " << num_changes << endl;
      new_additions.clear();
    }
    cout << combined_pairs.size() << endl;
    all_similar.push_back(combined_pairs);
  }
  cout << "Finished initial all_similar " << all_similar.size() << endl;
  /*map<double, vector<double> > combinations;
  for(int i = 0; i < most_similar.size(); i++) {
    combinations.insert(pair<double,vector<double> >(most_similar[i][0], vector<double>()));
  }
  for(int i = 0; i < most_similar.size(); i++) {
    combinations[most_similar[i][0]].push_back(most_similar[i][1]);
  }
  for(map<double, vector<double> >::iterator it=combinations.begin(); it!=combinations.end(); ++it){
    cout << it->first << endl;
    for(int i = 0; i < it->second.size(); i++){
      cout << it->second[i] << endl;
      map<double, vector<double> >::iterator fit = combinations.find(it->second[i]);
      if(fit != combinations.end()){
        cout << fit->first << endl;
        for(int j = 0; j < fit->second.size(); j++){
          cout << fit->second[j] << endl;
          it->second.push_back(fit->second[j]);
        }
      }
    }
  }
  cout << "Finished map version of all_similar" << endl;*/
}

//----------------------//------------------------//

void FORRBarriers::MergeNearbyBarriers(vector<LineSegment> &merged_barriers, const vector<LineSegment> &initial_barriers) {
  merged_barriers = initial_barriers;
}