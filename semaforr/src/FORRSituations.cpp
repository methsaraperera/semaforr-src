#include<FORRSituations.h>

using namespace std;



//----------------------//------------------------//


void FORRSituations::createSituations(int count, vector<int> values) {
  situations.push_back(values);
  situation_counts.push_back(count);
}


//----------------------//------------------------//


void FORRSituations::addObservationToSituations() {

}


//----------------------//------------------------//


void FORRSituations::updateSituations() {
  clearAllSituations();
}


//----------------------//------------------------//


vector<int> FORRSituations::identifySituation(sensor_msgs::LaserScan ls) {
  double angle = ls.angle_min;
  double increment = ls.angle_increment;
  vector<double> laser_ranges = ls.ranges;
  vector< vector<int> > grid;
  for(int i = 0; i < 51; i++){
    vector<int> col;
    for(int j = 0; j < 51; j++){
      col.push_back(0);
    }
    grid.push_back(col);
  }
  for(int i = 0; i < laser_ranges.size(); i++){
    for(double j = 0.0; j <= laser_ranges[i]; j+=0.9){
      int x = (int)(round(j * cos(angle)))+25;
      int y = (int)(round(j * sin(angle)))+25;
      grid[x][y] = 1;
      x = (int)(j * cos(angle))+25;
      y = (int)(j * sin(angle))+25;
      grid[x][y] = 1;
      x = (int)(floor(j * cos(angle)))+25;
      y = (int)(floor(j * sin(angle)))+25;
      grid[x][y] = 1;
      x = (int)(ceil(j * cos(angle)))+25;
      y = (int)(ceil(j * sin(angle)))+25;
      grid[x][y] = 1;
    }
    angle = angle + increment;
  }
  vector<int> new_situation;
  for(int i = 16; i < grid.size(); i++){
    for(int j = 0; j < grid[i].size(); j++){
      new_situation.push_back(grid[i][j]);
    }
  }
  vector<int> distances;
  for(int i = 0; i < situations.size(); i++){
    int dist = 0;
    for(int j = 0; j < new_situation.size(); j++){
      dist += abs(situations[i][j] - new_situation[j]);
    }
    distances.push_back(dist);
  }
  
}