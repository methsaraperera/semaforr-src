#include<FORRHallways.h>

using namespace std;



//----------------------//------------------------//



void FORRHallways::CreateSegments(vector<Segment> &segments, const vector<vector<CartesianPoint> > &trails) {
  cout << "num of trails " << trails.size() << endl;
  for(int i = 0; i < trails.size(); i++) {
  	cout << "num of trail markers " << trails[i].size() << endl;
    for(int j = 0; j < trails[i].size()-1; j++) {
      Segment current_segment = Segment(trails[i][j], trails[i][j+1]);
      if(trails[i][j].get_x() > trails[i][j+1].get_x())
        current_segment = Segment(trails[i][j+1], trails[i][j]);
      if((trails[i][j].get_x() != trails[i][j+1].get_x()) and (trails[i][j].get_y() != trails[i][j+1].get_y()) and (pow(pow((trails[i][j].get_x() - trails[i][j+1].get_x()), 2)+pow((trails[i][j].get_y() - trails[i][j+1].get_y()), 2),0.5)>=0.2))
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
  if(max_val != min_val){
  	return (original_val - min_val)/(max_val - min_val);
  }
  else{
  	return 0.0;
  }
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
  cout << "average_of_distances = " << average_of_distances << endl;
  for(int i = 0; i < similarities.size(); i++) {
    sum_of_squared_differences += pow((similarities[i][2]-average_of_distances), 2);
  }

  double normalized_sum_of_squared_differences = sum_of_squared_differences/(similarities.size());
  std = pow(normalized_sum_of_squared_differences, .5); // square root of squared difference sum
  cout << "std = " << std << endl;
  double deviations = 2;
  threshold = average_of_distances - (deviations*std);
  cout << "threshold " << threshold << endl;
  while(threshold <= 0){
  	deviations = deviations-0.5;
  	threshold = average_of_distances - (deviations*std);
  	cout << "threshold " << threshold << endl;
  }
  
  for(int i = 0; i < similarities.size(); i++) {
  	//cout << similarities[i][2] << endl;
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
    Segment left = segments[most_similar[i][0]];
    Segment right = segments[most_similar[i][1]];
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
vector<vector<CartesianPoint> > FORRHallways::ProcessHallwayData(const vector<Segment> &hallway_group,int filter_size, int width, int height) {
    vector<vector<double> > heat_map(width,vector<double>(height, 0));
    vector<vector<double> > filtered_heat_map(width,vector<double>(height, 0));
    vector<vector<int> > binarized_heat_map(width,vector<int>(height, 0));
    vector<vector<int> > labeled_image(width,vector<int>(height, 0));

    //cout << heat_map.size() << " " << heat_map[0].size() << endl;
    //cout << hallway_groups[i].size() << endl;
    UpdateMap(heat_map, hallway_group);
    cout << "1a" << endl;

    /*FilterImage(filtered_heat_map,heat_map,9);

    cout << "2a" << endl;
    BinarizeImage(binarized_heat_map, filtered_heat_map, 0);*/
    BinarizeImage(binarized_heat_map, heat_map, 1);

    cout << "3a" << endl;
    LabelImage(binarized_heat_map,labeled_image);

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
void FORRHallways::UpdateMap(vector<vector<double> > &frequency_map, const vector<Segment> &segments) {
  cout << "map size input: " << frequency_map.size() << " " << frequency_map[0].size() << endl;
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
  /*for(int i = 0; i < frequency_map.size(); i++) {
    for(int j = 0; j < frequency_map[0].size(); j++) {
      cout << frequency_map[i][j] << " ";
    }
    cout << endl;
  }*/
}

//********$^$^^%#^#

// does this produce a similar map?
void FORRHallways::Interpolate(vector<vector<double> > &frequency_map, double left_x, double left_y, double right_x, double right_y) {

  double step = abs(left_x - right_x) + abs(left_y - right_y);
  //cout << "step: " << step << " " << 1/step << endl;

  for(double j = 0; j <= 1; j += 1/step) {
    //cout << j << endl;
    int xcoord = round(j*(right_x - left_x) + left_x);
    int ycoord = round(j*(right_y - left_y) + left_y);
    //cout << xcoord << " " << ycoord << endl;
    //cout << frequency_map[0].size() << " " << frequency_map.size() << endl;
    frequency_map[xcoord][ycoord] += 1.0;
    //cout << xcoord << " " << ycoord << " " << frequency_map[xcoord][ycoord] << endl;
    if(xcoord>0 and ycoord>0){
      frequency_map[xcoord-1][ycoord-1] += 0.17;
    }
    if(xcoord>0){
      frequency_map[xcoord-1][ycoord] += 0.17;
    }
    if(ycoord>0){
      frequency_map[xcoord][ycoord-1] += 0.17;
    }
    if(xcoord < frequency_map.size() and ycoord < frequency_map[0].size()){
      frequency_map[xcoord+1][ycoord+1] += 0.17;
    }
    if(xcoord < frequency_map.size()){
      frequency_map[xcoord+1][ycoord] += 0.17;
    }
    if(ycoord < frequency_map[0].size()){
      frequency_map[xcoord][ycoord+1] += 0.17;
    }
    if(xcoord>0 and ycoord < frequency_map[0].size()){
      frequency_map[xcoord-1][ycoord+1] += 0.17;
    }
    if(xcoord < frequency_map.size() and ycoord>0){
      frequency_map[xcoord+1][ycoord-1] += 0.17;
    }
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
  double arr1[] = {0,0,0,0,0,0,0.00013527,0.0010609,0.0017273,0.0019467,0.0017273,0.0010609,0.00013527,0,0,0,0,0,0};
  std::vector<double> v1(arr1, arr1+19);
  filter.push_back(v1);
  double arr2[19] = {0,0,0,0,0.00028092,0.0021842,0.0037149,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0037149,0.0021842,0.00028092,0,0,0,0};
  std::vector<double> v2(arr2, arr2+19);
  filter.push_back(v2);
  double arr3[19] = {0,0,0,0.00092058,0.0035515,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0035515,0.00092058,0,0,0};
  std::vector<double> v3(arr3, arr3+19);
  filter.push_back(v3);
  double arr4[19] = {0,0,0.00092058,0.0037832,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0037832,0.00092058,0,0};
  std::vector<double> v4(arr4, arr4+19);
  filter.push_back(v4);
  double arr5[19] = {0,0.00028092,0.0035515,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0035515,0.00028092,0};
  std::vector<double> v5(arr5, arr5+19);
  filter.push_back(v5);
  double arr6[19] = {0,0.0021842,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0021842,0};
  std::vector<double> v6(arr6, arr6+19);
  filter.push_back(v6);
  double arr7[19] = {0.00013527,0.0037149,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0037149,0.00013527};
  std::vector<double> v7(arr7, arr7+19);
  filter.push_back(v7);
  double arr8[19] = {0.0010609,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0010609};
  std::vector<double> v8(arr8, arr8+19);
  filter.push_back(v8);
  double arr9[19] = {0.0017273,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0017273};
  std::vector<double> v9(arr9, arr9+19);
  filter.push_back(v9);
  double arr10[19] = {0.0019467,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0019467};
  std::vector<double> v10(arr10, arr10+19);
  filter.push_back(v10);
  double arr11[19] = {0.0017273,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0017273};
  std::vector<double> v11(arr11, arr11+19);
  filter.push_back(v11);
  double arr12[19] = {0.0010609,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0010609};
  std::vector<double> v12(arr12, arr12+19);
  filter.push_back(v12);
  double arr13[19] = {0.00013527,0.0037149,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0037149,0.00013527};
  std::vector<double> v13(arr13, arr13+19);
  filter.push_back(v13);
  double arr14[19] = {0,0.0021842,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0021842,0};
  std::vector<double> v14(arr14, arr14+19);
  filter.push_back(v14);
  double arr15[19] = {0,0.00028092,0.0035515,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0035515,0.00028092,0};
  std::vector<double> v15(arr15, arr15+19);
  filter.push_back(v15);
  double arr16[19] = {0,0,0.00092058,0.0037832,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0037832,0.00092058,0,0};
  std::vector<double> v16(arr16, arr16+19);
  filter.push_back(v16);
  double arr17[19] = {0,0,0,0.00092058,0.0035515,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0035515,0.00092058,0,0,0};
  std::vector<double> v17(arr17, arr17+19);
  filter.push_back(v17);
  double arr18[19] = {0,0,0,0,0.00028092,0.0021842,0.0037149,0.0039298,0.0039298,0.0039298,0.0039298,0.0039298,0.0037149,0.0021842,0.00028092,0,0,0,0};
  std::vector<double> v18(arr18, arr18+19);
  filter.push_back(v18);
  double arr19[19] = {0,0,0,0,0,0,0.00013527,0.0010609,0.0017273,0.0019467,0.0017273,0.0010609,0.00013527,0,0,0,0,0,0};
  std::vector<double> v19(arr19, arr19+19);
  filter.push_back(v19);
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
      if(original[i][j] >= threshold){
        binarized[i][j] = 255;
        /*if(i>0 and j>0){
          binarized[i-1][j-1] = 255;
        }
        if(i>0){
          binarized[i-1][j] = 255;
        }
        if(j>0){
          binarized[i][j-1] = 255;
        }
        if(i < original.size() and j < original[0].size()){
          binarized[i+1][j+1] = 255;
        }
        if(i < original.size()){
          binarized[i+1][j] = 255;
        }
        if(j < original[0].size()){
          binarized[i][j+1] = 255;
        }
        if(i>0 and j < original[0].size()){
          binarized[i-1][j+1] = 255;
        }
        if(i < original.size() and j>0){
          binarized[i+1][j-1] = 255;
        }*/
      }
      //cout << binarized[i][j] << " ";
    }
    //cout << endl;
  }
}


void FORRHallways::doUnion(int x, int y, int x2, int y2, vector<vector<int> > &labeled_image){
  int a = map_width_*y+x;
  int ya = y;
  int xa = x;
  //cout << "a " << a << " " << xa << " " << ya << endl;
  while (labeled_image[xa][ya] != a){
    a = labeled_image[xa][ya];
    ya = floor(a/map_width_);
    xa = a - map_width_*ya;
    //cout << a << " " << xa << " " << ya << endl;
  }
  int b = map_width_*y2+x2;
  int yb = y2;
  int xb = x2;
  //cout << "b " << b << " " << xb << " " << yb << endl;
  while (labeled_image[xb][yb] != b){
    b = labeled_image[xb][yb];
    yb = floor(b/map_width_);
    xb = b - map_width_*yb;
    //cout << b << " " << xb << " " << yb << endl;
  }
  labeled_image[xb][yb] = a;
}
 
void FORRHallways::unionCoords(int x, int y, int x2, int y2, const vector<vector<int> > &binary_map, vector<vector<int> > &labeled_image){
  if (y2 < map_height_ && x2 < map_width_ && binary_map[x][y] && binary_map[x2][y2]){
    //cout << y2 << " " << x2 << " " << binary_map[x][y] << " " << binary_map[x2][y2] << endl;
    doUnion(x, y, x2, y2, labeled_image);
  }
}

void FORRHallways::LabelImage(const vector<vector<int> > &binary_map, vector<vector<int> > &labeled_image){
  //cout << "map_width_ " << map_width_ << " map_height_ " << map_height_ << endl;
  for (int x = 0; x < map_width_; x++){
    for (int y = 0; y < map_height_; y++){
      labeled_image[x][y] = map_width_*y+x;
      //cout << x << " " << y << " " << map_width_*y+x << endl;
    }
  }

  for (int x = 0; x < map_width_; x++){
    for (int y = 0; y < map_height_; y++){
      unionCoords(x, y, x+1, y, binary_map, labeled_image);
      unionCoords(x, y, x, y+1, binary_map, labeled_image);
    }
  }
  for (int x = 0; x < map_width_; x++){
    for (int y = 0; y < map_height_; y++){
      if (binary_map[x][y] == 0){
        //cout << ' ';
        labeled_image[x][y] = -1;
        continue;
      }
      int c = map_width_*y+x;
      int xnew = x;
      int ynew = y;
      while (labeled_image[xnew][ynew] != c){
        c = labeled_image[xnew][ynew];
        ynew = floor(c/map_width_);
        xnew = c - map_width_*ynew;
      }
      //cout << c << " ";
      labeled_image[x][y] = c;
    }
    //cout << "\n";
  }
}

void FORRHallways::ListGroups(vector<vector< pair<int,int> > > &aggregates_and_points, const vector<vector<int> > &labeled_image){
  int num_rows = labeled_image.size();
  int num_cols = labeled_image[0].size();
  //cout << "num_rows " << num_rows << " num_cols " << num_cols << endl;
  map<int, int> aggregates_ids;

  int id_given = 0;

  for(int i = 0; i < num_rows; i++){
    for(int j = 0; j < num_cols; j++){
      int pixel = labeled_image[i][j];
      //cout << pixel << endl;
      if(aggregates_ids.count(pixel) > 0) {
        int id = aggregates_ids[pixel];
        aggregates_and_points[id].push_back(make_pair(i,j));
      }
      else if(pixel >= 0){
        aggregates_ids[pixel] = id_given;
        vector<pair<int,int> > first_element;
        first_element.push_back(make_pair(i,j));
        aggregates_and_points.push_back(first_element);
        id_given++;
      }
    }
    //cout << aggregates_and_points.size() << endl;
  }
}

void FORRHallways::ConvertPairToCartesianPoint(vector<vector<CartesianPoint> > &trails, const vector<vector< pair<int,int> > > &input) {
  for(int i = 0; i < input.size(); i++){
    vector<CartesianPoint> trail_coordinates;
    for(int j = 0; j < input[i].size(); j++) {
      CartesianPoint new_point0 = CartesianPoint(input[i][j].first,input[i][j].second);
      trail_coordinates.push_back(new_point0);
      /*CartesianPoint new_point1 = CartesianPoint(input[i][j].first+1.0,input[i][j].second+1.0);
      trail_coordinates.push_back(new_point1);
      CartesianPoint new_point2 = CartesianPoint(input[i][j].first+1.0,input[i][j].second-1.0);
      trail_coordinates.push_back(new_point2);
      CartesianPoint new_point3 = CartesianPoint(input[i][j].first-1.0,input[i][j].second+1.0);
      trail_coordinates.push_back(new_point3);
      CartesianPoint new_point4 = CartesianPoint(input[i][j].first-1.0,input[i][j].second-1.0);
      trail_coordinates.push_back(new_point4);
      CartesianPoint new_point5 = CartesianPoint(input[i][j].first+1.0,input[i][j].second);
      trail_coordinates.push_back(new_point5);
      CartesianPoint new_point6 = CartesianPoint(input[i][j].first-1.0,input[i][j].second);
      trail_coordinates.push_back(new_point6);
      CartesianPoint new_point7 = CartesianPoint(input[i][j].first,input[i][j].second+1.0);
      trail_coordinates.push_back(new_point7);
      CartesianPoint new_point8 = CartesianPoint(input[i][j].first,input[i][j].second-1.0);
      trail_coordinates.push_back(new_point8);*/
    }
    trails.push_back(trail_coordinates);
  }
}