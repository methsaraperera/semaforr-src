#include<FORRSituations.h>

using namespace std;



//----------------------//------------------------//


void FORRSituations::addObservationToSituations(sensor_msgs::LaserScan ls, Position pose, bool add_to_existing, FORRAction actual_action) {
  double angle = ls.angle_min;
  double increment = ls.angle_increment;
  vector<float> laser_ranges = ls.ranges;
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
  vector<float> new_situation_float;
  for(int i = 16; i < grid.size(); i++){
    for(int j = 0; j < grid[i].size(); j++){
      new_situation.push_back(grid[i][j]);
      new_situation_float.push_back((float)(grid[i][j]));
      cout << grid[i][j] << " ";
    }
    cout << endl;
  }
  vector<float> distances;
  for(int i = 0; i < situations.size(); i++){
    float dist = 0;
    for(int j = 0; j < new_situation.size(); j++){
      dist += abs(situations[i][j] - (float)(new_situation[j]));
    }
    // cout << "Situation " << i << " " << situation_counts[i] << " dist : " << dist << endl;
    distances.push_back(dist);
  }
  int min_pos = distance(distances.begin(),min_element(distances.begin(),distances.end()));
  if(distances[min_pos] <= dist_cutoff){
    situation_observations.push_back(SituationMarker(ls, pose, min_pos, new_situation_float, actual_action));
    // situation_assignments.push_back(min_pos);
    // situation_laserscans.push_back(ls);
    if(add_to_existing){
      // cout << min_pos << " " << situation_counts[min_pos] << endl;
      // printSituation(min_pos);
      vector<float> min_sit = situations[min_pos];
      vector<float> min_sit_counts;
      for(int i = 0; i < min_sit.size(); i++){
        // cout << (min_sit[i]*((float)(situation_counts[min_pos]))) << " " << ((float)(new_situation[i])) << " " << ((float)(situation_counts[min_pos]+1)) << " " << (((min_sit[i]*((float)(situation_counts[min_pos])))+((float)(new_situation[i])))/((float)(situation_counts[min_pos]+1))) << endl;
        min_sit_counts.push_back(((min_sit[i]*((float)(situation_counts[min_pos])))+((float)(new_situation[i])))/((float)(situation_counts[min_pos]+1)));
      }
      situations[min_pos] = min_sit_counts;
      situation_counts[min_pos] = situation_counts[min_pos]+1;
      // for(int i = 0; i < situation_counts.size(); i++){
      //   cout << "Situation " << i << " : " << situation_counts[i] << endl;
      // }
    }
  }
  // else if(add_to_existing){
  //   situation_assignments.push_back(situations.size());
  //   situation_laserscans.push_back(ls);
  //   situation_counts.push_back(1);
  //   situations.push_back(new_situation_float);
  // }
  else{
    situation_observations.push_back(SituationMarker(ls, pose, -1, new_situation_float, actual_action));
    // situation_assignments.push_back(-1);
    // situation_laserscans.push_back(ls);
  }
}


//----------------------//------------------------//


void FORRSituations::clusterOutlierObservations(){
  vector< vector<float> > action_grids_for_clustering;
  // vector<SituationMarker> observations_for_clustering;
  vector<int> observation_inds;
  for(int i = 0; i < situation_observations.size(); i++){
    if(situation_observations[i].assignment == -1){
      int matched_situation = identifySituationAssignment(situation_observations[i].action_grid);
      // cout << "Before " << i << " " << situation_observations[i].assignment << " " << matched_situation << endl;
      if(matched_situation != -1){
        situation_observations[i].assignment = matched_situation;
        // cout << "After " << i << " " << situation_observations[i].assignment << " " << matched_situation << endl;
      }
      else{
        // observations_for_clustering.push_back(situation_observations[i]);
        observation_inds.push_back(i);
        action_grids_for_clustering.push_back(situation_observations[i].action_grid);
      }
    }
  }
  cout << "Unclustered obs size: " << observation_inds.size() << " " << action_grids_for_clustering.size() << endl;
  if(action_grids_for_clustering.size() > 200){
    SpectralCluster clustering = SpectralCluster(15, 10, 50, 0.95);
    int situation_ind = situations.size();
    vector< vector<float> > results = clustering.cluster(action_grids_for_clustering);
    if(results.size() > 0){
      for(int i = 0; i < results.size(); i++){
        int count = (int)(results[i][0]);
        vector<float> values = results[i];
        values.erase(values.begin());
        createSituations(count, values);
      }
      vector<int> new_assignments = clustering.clusterAssignments();
      cout << "New assignments size: " << observation_inds.size() << " " << new_assignments.size() << " " << situation_ind << endl;
      for(int i = 0; i < observation_inds.size(); i++){
        int matched_situation = new_assignments[i];
        // cout << "Before " << i << " " << situation_observations[observation_inds[i]].assignment << " " << matched_situation << endl;
        if(matched_situation != -1){
          situation_observations[observation_inds[i]].assignment = matched_situation + situation_ind;
          // cout << "After " << i << " " << situation_observations[observation_inds[i]].assignment << " " << matched_situation + situation_ind << endl;
        }
      }
    }
    clustering.eraseFiles();
  }
}


//----------------------//------------------------//


vector<int> FORRSituations::identifySituation(sensor_msgs::LaserScan ls) {
  double angle = ls.angle_min;
  double increment = ls.angle_increment;
  vector<float> laser_ranges = ls.ranges;
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
      // cout << grid[i][j] << " ";
    }
    // cout << endl;
  }
  // vector<float> distances;
  // for(int i = 0; i < situations.size(); i++){
  //   float dist = 0;
  //   for(int j = 0; j < new_situation.size(); j++){
  //     dist += abs(situations[i][j] - (float)(new_situation[j]));
  //   }
  //   // cout << dist << endl;
  //   distances.push_back(dist);
  // }
  // int min_pos = distance(distances.begin(),min_element(distances.begin(),distances.end()));
  // // cout << min_pos << endl;
  // vector<int> situation_median;
  // for(int i = 0; i < situations[min_pos].size(); i++){
  //   if(situations[min_pos][i] >= 0.25){
  //     situation_median.push_back(1);
  //   }
  //   else{
  //     situation_median.push_back(0);
  //   }
  // }
  // return situation_median;
  return new_situation;
}


//----------------------//------------------------//


vector< vector<int> > FORRSituations::overlaySituations(vector< vector<CartesianPoint> > laserendpoints, vector<Position> poses){
  cout << "overlaySituations laserendpoints " << laserendpoints.size() << " poses " << poses.size() << endl;
  int dimension = 200;
  vector< vector <vector<int> > > total_coverages;
  for(int k = 0; k < laserendpoints.size(); k++){
    vector< vector<int> > grid;
    for(int i = 0; i < dimension; i++){
      vector<int> col;
      for(int j = 0; j < dimension; j++){
        col.push_back(0);
      }
      grid.push_back(col);
    }
    for(int l = 0; l < laserendpoints[k].size(); l++){
      double x1 = poses[k].getX();
      double y1 = poses[k].getY();
      double x2 = laserendpoints[k][l].get_x();
      double y2 = laserendpoints[k][l].get_y();
      double step_size = 0.01;
      double tx,ty;
      for(double step = 0; step <= 1; step += step_size){
        tx = (int)((x2 * step) + (x1 * (1-step)));
        ty = (int)((y2 * step) + (y1 * (1-step)));
        if(tx >= 0 and tx < dimension and ty >= 0 and ty < dimension){
          grid[tx][ty] += 1;
        }
      }
    }
    for(int i = 0; i < dimension; i++){
      for(int j = 0; j < dimension; j++){
        cout << grid[i][j] << " ";
      }
      cout << endl;
    }
    cout << endl;
    total_coverages.push_back(grid);
  }
  cout << "total_coverages " << total_coverages.size() << endl;
  vector < vector<int> > total_coverage_added;
  for(int i = 0; i < dimension; i++){
    vector<int> col;
    for(int j = 0; j < dimension; j++){
      col.push_back(0);
    }
    total_coverage_added.push_back(col);
  }
  for(int i = 0; i < total_coverages.size(); i++){
    for(int j = 0; j < total_coverages[i].size(); j++){
      for(int k = 0; k < total_coverages[i][j].size(); k++){
        total_coverage_added[j][k] += total_coverages[i][j][k];
      }
    }
  }
  for(int i = 0; i < total_coverage_added.size(); i++){
    for(int j = 0; j < total_coverage_added[i].size(); j++){
      cout << total_coverage_added[i][j] << " ";
    }
    cout << endl;
  }
  cout << endl;
  for(int i = 0; i < total_coverage_added.size(); i++){
    for(int j = 0; j < total_coverage_added[i].size(); j++){
      if(total_coverage_added[i][j] <= 5){
        total_coverage_added[i][j] = 0;
      }
      cout << total_coverage_added[i][j] << " ";
    }
    cout << endl;
  }
  cout << endl;
  return total_coverage_added;
}
  // sensor_msgs::LaserScan ls1 = laserscans[0];
  // Position pose1 = poses[0];

  // double angle1 = ls1.angle_min;
  // double increment1 = ls1.angle_increment;
  // vector<float> laser_ranges1 = ls1.ranges;
  // // cout << "angle1 " << angle1 << " increment1 " << increment1 << " laser_ranges1 " << laser_ranges1.size() << endl;
  // vector< vector<int> > grid;
  // for(int i = 0; i < 51; i++){
  //   vector<int> col;
  //   for(int j = 0; j < 51; j++){
  //     col.push_back(0);
  //   }
  //   grid.push_back(col);
  // }
  // for(int i = 0; i < laser_ranges1.size(); i++){
  //   // cout << angle1 << " " << laser_ranges1[i] << endl;
  //   for(double j = 0.0; j <= laser_ranges1[i]; j+=0.01){
  //     // int x = (int)(round(j * cos(angle1)))+25;
  //     // int y = (int)(round(j * sin(angle1)))+25;
  //     // if(x >= 0 and x <= 51 and y >= 0 and y <= 51){
  //     //   grid[x][y] += 1;
  //     // }
  //     int x = (int)(j * cos(angle1))+25;
  //     int y = (int)(j * sin(angle1))+25;
  //     if(x >= 0 and x <= grid[0].size()-1 and y >= 0 and y <= grid[0].size()-1){
  //       grid[x][y] = 1;
  //       // cout << x << " " << y << endl;
  //     }
  //     // x = (int)(floor(j * cos(angle1)))+25;
  //     // y = (int)(floor(j * sin(angle1)))+25;
  //     // if(x >= 0 and x <= 51 and y >= 0 and y <= 51){
  //     //   grid[x][y] += 1;
  //     // }
  //     // x = (int)(ceil(j * cos(angle1)))+25;
  //     // y = (int)(ceil(j * sin(angle1)))+25;
  //     // if(x >= 0 and x <= 51 and y >= 0 and y <= 51){
  //     //   grid[x][y] += 1;
  //     // }
  //   }
  //   if(laser_ranges1[i] < 25){
  //     int x1 = (int)(laser_ranges1[i] * cos(angle1))+25;
  //     int y1 = (int)(laser_ranges1[i] * sin(angle1))+25;
  //     int x2 = (int)(laser_ranges1[i]+1 * cos(angle1))+25;
  //     int y2 = (int)(laser_ranges1[i]+1 * sin(angle1))+25;
  //     // cout << x1 << " " << y1 << " " << x2 << " " << y2 << endl;
  //     if(x1 >= 0 and x1 <= grid[0].size()-1 and y1 >= 0 and y1 <= grid[0].size()-1){
  //       if(grid[x1][y1] == 0){
  //         grid[x1][y1] = -1;
  //       }
  //       else if(x2 >= 0 and x2 <= grid[0].size()-1 and y2 >= 0 and y2 <= grid[0].size()-1){
  //         if(grid[x2][y2] == 0){
  //           grid[x2][y2] = -1;
  //         }
  //       }
  //     }
  //   }
  //   angle1 = angle1 + increment1;
  // }
  // for(int i = 16; i < grid.size(); i++){
  //   for(int j = 0; j < grid[i].size(); j++){
  //     cout << grid[i][j] << " ";
  //   }
  //   cout << endl;
  // }
  // cout << endl;
  // vector < vector< vector<int> > > new_grids;
  // double required_rotation = 0 - pose1.getTheta();
  // for(int k = 1; k < laserscans.size(); k++){
  //   vector< vector<int> > n_grid;
  //   for(int i = 0; i < 51; i++){
  //     vector<int> col;
  //     for(int j = 0; j < 51; j++){
  //       col.push_back(0);
  //     }
  //     n_grid.push_back(col);
  //   }
  //   sensor_msgs::LaserScan ls2 = laserscans[k];
  //   Position pose2 = poses[k];
  //   // cout << "pose1 theta " << pose1.getTheta() << " pose2 theta " << pose2.getTheta() << " required_rotation " << required_rotation << endl;
  //   double angle2 = ls2.angle_min + pose2.getTheta() + required_rotation;
  //   double increment2 = ls2.angle_increment;
  //   vector<float> laser_ranges2 = ls2.ranges;
  //   // cout << "angle2 " << angle2 << " increment2 " << increment2 << " laser_ranges2 " << laser_ranges2.size() << endl;
  //   double required_x_shift = pose2.getX() - pose1.getX();
  //   double required_y_shift = pose2.getY() - pose1.getY();
  //   // cout << "pose1 " << pose1.getX() << " " << pose1.getY() << " pose2 " << pose2.getX() << " " << pose2.getY() << " required_x_shift " << required_x_shift << " required_y_shift " << required_y_shift << endl;
  //   for(int i = 0; i < laser_ranges2.size(); i++){
  //     // cout << angle2 << " " << laser_ranges2[i] << endl;
  //     for(double j = 0.0; j <= laser_ranges2[i]; j+=0.01){
  //       // int x = (int)(round(j * cos(angle2) + required_x_shift))+25;
  //       // int y = (int)(round(j * sin(angle2) + required_y_shift))+25;
  //       // if(x >= 0 and x < 51 and y >= 0 and y < 51){
  //       //   grid[x][y] += 1;
  //       // }
  //       int x = (int)(j * cos(angle2) + required_x_shift)+25;
  //       int y = (int)(j * sin(angle2) + required_y_shift)+25;
  //       if(x >= 0 and x <= grid[0].size()-1 and y >= 0 and y <= grid[0].size()-1){
  //         n_grid[x][y] = 1;
  //         // cout << x << " " << y << endl;
  //       }
  //       // x = (int)(floor(j * cos(angle2) + required_x_shift))+25;
  //       // y = (int)(floor(j * sin(angle2) + required_y_shift))+25;
  //       // if(x >= 0 and x <= 51 and y >= 0 and y <= 51){
  //       //   grid[x][y] += 1;
  //       // }
  //       // x = (int)(ceil(j * cos(angle2) + required_x_shift))+25;
  //       // y = (int)(ceil(j * sin(angle2) + required_y_shift))+25;
  //       // if(x >= 0 and x <= 51 and y >= 0 and y <= 51){
  //       //   grid[x][y] += 1;
  //       // }
  //     }
  //     if(laser_ranges2[i] < 25){
  //       int x1 = (int)(laser_ranges2[i] * cos(angle2) + required_x_shift)+25;
  //       int y1 = (int)(laser_ranges2[i] * sin(angle2) + required_y_shift)+25;
  //       int x2 = (int)(laser_ranges2[i]+1 * cos(angle2) + required_x_shift)+25;
  //       int y2 = (int)(laser_ranges2[i]+1 * sin(angle2) + required_y_shift)+25;
  //       // cout << x1 << " " << y1 << " " << x2 << " " << y2 << endl;
  //       if(x1 >= 0 and x1 <= grid[0].size()-1 and y1 >= 0 and y1 <= grid[0].size()-1){
  //         if(n_grid[x1][y1] == 0){
  //           n_grid[x1][y1] = -1;
  //         }
  //         else if(x2 >= 0 and x2 <= grid[0].size()-1 and y2 >= 0 and y2 <= grid[0].size()-1){
  //           if(n_grid[x2][y2] == 0){
  //             n_grid[x2][y2] = -1;
  //           }
  //         }
  //       }
  //     }
  //     angle2 = angle2 + increment2;
  //   }
  //   for(int i = 16; i < n_grid.size(); i++){
  //     for(int j = 0; j < n_grid[i].size(); j++){
  //       cout << n_grid[i][j] << " ";
  //     }
  //     cout << endl;
  //   }
  //   cout << endl;
  //   new_grids.push_back(n_grid);
  // }
  // for(int i = 0; i < new_grids.size(); i++){
  //   for(int j = 0; j < new_grids[i].size(); j++){
  //     for(int k = 0; k < new_grids[i][k].size(); k++){
  //       if(new_grids[i][j][k] == 1 and grid[j][k] >= 0){
  //         grid[j][k] = grid[j][k] + 1;
  //       }
  //       else if(new_grids[i][j][k] == 1 and grid[j][k] == -1){
  //         grid[j][k] = 1;
  //       }
  //       else if(new_grids[i][j][k] == -1 and grid[j][k] == 0){
  //         grid[j][k] = -1;
  //       }
  //     }
  //   }
  // }
  // for(int i = 16; i < grid.size(); i++){
  //   for(int j = 0; j < grid[i].size(); j++){
  //     cout << grid[i][j] << " ";
  //   }
  //   cout << endl;
  // }
  // cout << endl;
  // for(int i = 0; i < grid.size(); i++){
  //   for(int j = 0; j < grid[i].size(); j++){
  //     if(grid[i][j] > 1){
  //       grid[i][j] = 1;
  //     }
  //   }
  // }
  // for(int i = 16; i < grid.size(); i++){
  //   for(int j = 0; j < grid[i].size(); j++){
  //     cout << grid[i][j] << " ";
  //   }
  //   cout << endl;
  // }
  // return grid;
// }


//----------------------//------------------------//


int FORRSituations::identifySituationAssignment(sensor_msgs::LaserScan ls) {
  double angle = ls.angle_min;
  double increment = ls.angle_increment;
  vector<float> laser_ranges = ls.ranges;
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
      // cout << grid[i][j] << " ";
    }
    // cout << endl;
  }
  vector<float> distances;
  for(int i = 0; i < situations.size(); i++){
    float dist = 0;
    for(int j = 0; j < new_situation.size(); j++){
      dist += abs(situations[i][j] - (float)(new_situation[j]));
    }
    // cout << "Situation dist " << i << " : " << dist << endl;
    distances.push_back(dist);
  }
  int min_pos = distance(distances.begin(),min_element(distances.begin(),distances.end()));
  if(distances[min_pos] <= dist_cutoff){
    return min_pos;
  }
  else{
    return -1;
  }
}


//----------------------//------------------------//


int FORRSituations::identifySituationAssignment(vector<float> action_grid) {
  vector<float> distances;
  for(int i = 0; i < situations.size(); i++){
    float dist = 0;
    for(int j = 0; j < action_grid.size(); j++){
      dist += abs(situations[i][j] - action_grid[j]);
    }
    // cout << "Situation dist " << i << " : " << dist << endl;
    distances.push_back(dist);
  }
  int min_pos = distance(distances.begin(),min_element(distances.begin(),distances.end()));
  if(distances[min_pos] <= dist_cutoff){
    return min_pos;
  }
  else{
    return -1;
  }
}


//----------------------//------------------------//


void FORRSituations::learnSituationActions(AgentState *agentState, vector<TrailMarker> trail) {
  cout << "Inside learnSituationActions" << endl;
  // Position target(agentState->getCurrentTask()->getTaskX(),agentState->getCurrentTask()->getTaskY(),0);
  Position target = agentState->getCurrentTask()->getPositionHistory()->back();
  cout << "Target " << target.getX() << " " << target.getY() << endl;
  vector<double> target_distances;
  vector<double> target_angles;
  vector<FORRAction> trail_actions;
  vector<FORRAction> actual_actions;
  clusterOutlierObservations();
  vector<SituationMarker> current_situation_observations(situation_observations.end() - agentState->getCurrentTask()->getPositionHistory()->size(), situation_observations.end());
  cout << "Trail size " << trail.size() << " Situation Observations size " << current_situation_observations.size() << endl;
  cout << "Trail start " << trail[0].coordinates.get_x() << " " << trail[0].coordinates.get_y() << " Situation Observations Start " << current_situation_observations[0].pose.getX() << " " << current_situation_observations[0].pose.getY() << endl;
  cout << "Trail end " << trail[trail.size()-1].coordinates.get_x() << " " << trail[trail.size()-1].coordinates.get_y() << " Situation Observations end " << current_situation_observations[current_situation_observations.size()-1].pose.getX() << " " << current_situation_observations[current_situation_observations.size()-1].pose.getY() << endl;
  vector<int> trail_situation_assignments;
  set<FORRAction> *action_set = agentState->getActionSet();
  set<FORRAction>::iterator actionIter;
  // for(int i = 0 ; i < pos_hist->size() ; i++){
  for(int i = 0 ; i < current_situation_observations.size() ; i++){
    cout << "pose " << current_situation_observations[i].pose.getX() << " " << current_situation_observations[i].pose.getY() << " " << current_situation_observations[i].pose.getTheta() << endl;
    if(current_situation_observations[i].assignment == -1){
      continue;
    }
    vector<CartesianPoint> current_laser = agentState->transformToEndpoints(current_situation_observations[i].pose, current_situation_observations[i].ls);
    FORRAction max_forward = agentState->maxForwardAction(current_situation_observations[i].pose, current_laser);
    ROS_DEBUG_STREAM("Max allowed forward action : " << max_forward.type << " " << max_forward.parameter);
    int intensity = max_forward.parameter;
    set<FORRAction> vetoedActions;
    set<FORRAction> *forward_set = agentState->getForwardActionSet();
    cout << forward_set->size() << endl;
    for(int f = forward_set->size()-1 ; f > 0; f--){
      cout << f << endl;
      FORRAction a(FORWARD,f);
      if(f > intensity){
        ROS_DEBUG_STREAM("Vetoed action : " << a.type << " " << a.parameter);
        vetoedActions.insert(a);
      }
      cout << f << endl;
    }
    vector<int> visible_trailmarkers;
    vector<double> dist_to_trailmarkers;
    int target_trailmarker = -1;
    int max_trailmarker = -1;
    int close_trailmarker = -1;
    cout << "visible_trailmarkers ";
    for(int j = 0; j < trail.size(); j++){
      dist_to_trailmarkers.push_back(current_situation_observations[i].pose.getDistance(trail[j].coordinates.get_x(), trail[j].coordinates.get_y()));
      if(agentState->canAccessPoint(current_laser, CartesianPoint(current_situation_observations[i].pose.getX(), current_situation_observations[i].pose.getY()), trail[j].coordinates, 10)) {
        visible_trailmarkers.push_back(j);
        cout << j << " ";
      }
    }
    cout << endl;
    if(visible_trailmarkers.size() > 0){
      max_trailmarker = visible_trailmarkers[visible_trailmarkers.size()-1];
      cout << "max_trailmarker " << max_trailmarker << endl;
    }
    // if(visible_trailmarkers.size() == 0){
    //   continue;
    // }
    int min_dist_trailmarker;
    double min_dist = 1000000.0;
    for(int j = 0; j < dist_to_trailmarkers.size()-1; j++){
      double combined_dist = dist_to_trailmarkers[j] + dist_to_trailmarkers[j+1];
      if(combined_dist < min_dist){
        min_dist = combined_dist;
        min_dist_trailmarker = j;
      }
    }
    cout << "min_dist_trailmarker " << min_dist_trailmarker << " min_dist combined " << min_dist << " dist_to_trailmarker 1 " << dist_to_trailmarkers[min_dist_trailmarker] << " dist_to_trailmarker 2 " << dist_to_trailmarkers[min_dist_trailmarker+1] << endl;
    close_trailmarker = min_dist_trailmarker+1;
    // if(max_trailmarker <= min_dist_trailmarker){
    //   continue;
    // }
    if(max_trailmarker >= close_trailmarker){
      target_trailmarker = max_trailmarker;
    }
    else{
      target_trailmarker = close_trailmarker;
    }
    if(target_trailmarker == -1){
      continue;
    }
    double dist_to_target_trailmarker = dist_to_trailmarkers[target_trailmarker];
    cout << "target_trailmarker " << target_trailmarker << " dist_to_target_trailmarker " << dist_to_target_trailmarker << endl;
    cout << "found furthest trail marker " << trail[target_trailmarker].coordinates.get_x() << " " << trail[target_trailmarker].coordinates.get_y() << endl;
    double angle_to_target = atan2((target.getY() - current_situation_observations[i].pose.getY()), (target.getX() - current_situation_observations[i].pose.getX()));
    double required_rotation = angle_to_target - current_situation_observations[i].pose.getTheta();
    if(required_rotation > M_PI){
      required_rotation = required_rotation - (2 * M_PI);
    }
    if(required_rotation < -M_PI){
      required_rotation = required_rotation + (2 * M_PI);
    }
    cout << "Distance to target " << current_situation_observations[i].pose.getDistance(target) << " Rotation to target " << required_rotation << " Situation " << current_situation_observations[i].assignment << endl;
    printSituation(current_situation_observations[i].assignment);
    std::map <FORRAction, double> result;
    typedef map<FORRAction, double>::iterator mapIt;
    for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
      FORRAction forrAction = *actionIter;
      // cout << forrAction.type << " " << forrAction.parameter << endl;
      if(forrAction.type == PAUSE){
      // if(forrAction.type == PAUSE or forrAction.type == FORWARD){
        continue;
      }
      else if(vetoedActions.size() > 0){
        if(vetoedActions.find(forrAction) != vetoedActions.end()){
          continue;
        }
        else{
          // cout << forrAction.type << " " << forrAction.parameter << endl;
          Position expectedPosition = agentState->getExpectedPositionAfterAction(forrAction, current_laser, current_situation_observations[i].pose);
          cout << expectedPosition.getX() << " " << expectedPosition.getY() << endl;
          double dist_to_expected = expectedPosition.getDistance(trail[target_trailmarker].coordinates.get_x(), trail[target_trailmarker].coordinates.get_y());
          if(dist_to_expected <= dist_to_target_trailmarker){
            result[forrAction] = dist_to_expected;
            cout << forrAction.type << " " << forrAction.parameter << " " << dist_to_expected << endl;
          }
        }
      }
      else{
        // cout << forrAction.type << " " << forrAction.parameter << endl;
        Position expectedPosition = agentState->getExpectedPositionAfterAction(forrAction, current_laser, current_situation_observations[i].pose);
        cout << expectedPosition.getX() << " " << expectedPosition.getY() << endl;
        double dist_to_expected = expectedPosition.getDistance(trail[target_trailmarker].coordinates.get_x(), trail[target_trailmarker].coordinates.get_y());
        if(dist_to_expected <= dist_to_target_trailmarker){
          result[forrAction] = dist_to_expected;
          cout << forrAction.type << " " << forrAction.parameter << " " << dist_to_expected << endl;
        }
      }
    }
    double minDistanceToTrailMarker = 1000000;
    for(mapIt iterator = result.begin(); iterator != result.end(); iterator++){
      if(iterator->second < minDistanceToTrailMarker){
        minDistanceToTrailMarker = iterator->second;
      }
    }
    for(mapIt iterator = result.begin(); iterator != result.end(); iterator++){
       if(iterator->second == minDistanceToTrailMarker){
        trail_actions.push_back(iterator->first);
        target_distances.push_back(current_situation_observations[i].pose.getDistance(target));
        target_angles.push_back(required_rotation);
        trail_situation_assignments.push_back(current_situation_observations[i].assignment);
        actual_actions.push_back(current_situation_observations[i].actual_action);
      }
    }
    // for(int j = trail.size()-1; j >= 1; j--){
    //   if(agentState->canAccessPoint(current_laser, CartesianPoint(current_situation_observations[i].pose.getX(), current_situation_observations[i].pose.getY()), trail[j].coordinates, 5)) {
    //     cout << "found furthest trail marker" << endl;
    //     cout << "pose " << current_situation_observations[i].pose.getX() << " " << current_situation_observations[i].pose.getY() << " trail marker " << trail[j].coordinates.get_x() << " " << trail[j].coordinates.get_y() << endl;
    //     double angle_to_target = atan2((target.getY() - current_situation_observations[i].pose.getY()), (target.getX() - current_situation_observations[i].pose.getX()));
    //     double required_rotation = angle_to_target - current_situation_observations[i].pose.getTheta();
    //     if(required_rotation > M_PI){
    //       required_rotation = required_rotation - (2 * M_PI);
    //     }
    //     if(required_rotation < -M_PI){
    //       required_rotation = required_rotation + (2 * M_PI);
    //     }
    //     cout << current_situation_observations[i].pose.getDistance(target) << " " << required_rotation << " " << current_situation_observations[i].assignment << endl;
    //     std::map <FORRAction, double> result;
    //     typedef map<FORRAction, double>::iterator mapIt;
    //     for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
    //       FORRAction forrAction = *actionIter;
    //       if(forrAction.type == PAUSE or vetoedActions->find(forrAction) != vetoedActions->end()){
    //       // if(forrAction.type == PAUSE or forrAction.type == FORWARD){
    //         continue;
    //       }
    //       else{
    //         Position expectedPosition = agentState->getExpectedPositionAfterAction(forrAction, current_laser, current_situation_observations[i].pose);
    //         result[forrAction] = expectedPosition.getDistance(trail[j].coordinates.get_x(), trail[j].coordinates.get_y());
    //         cout << forrAction.type << " " << forrAction.parameter << " " << expectedPosition.getDistance(trail[j].coordinates.get_x(), trail[j].coordinates.get_y()) << endl;
    //       }
    //     }
    //     for(mapIt iterator = result.begin(); iterator != result.end(); iterator++){
    //       if(iterator->second <= 1){
    //         trail_actions.push_back(iterator->first);
    //         target_distances.push_back(current_situation_observations[i].pose.getDistance(target));
    //         target_angles.push_back(required_rotation);
    //         trail_situation_assignments.push_back(current_situation_observations[i].assignment);
    //         actual_actions.push_back(current_situation_observations[i].actual_action);
    //       }
    //     }
    //     // double minDistanceToTrailMarker = 1000000;
    //     // for(mapIt iterator = result.begin(); iterator != result.end(); iterator++){
    //     //   if(iterator->second < minDistanceToTrailMarker){
    //     //     minDistanceToTrailMarker = iterator->second;
    //     //   }
    //     // }
    //     // if(minDistanceToTrailMarker > 1){
    //     //   continue;
    //     // }
    //     // // cout << minDistanceToTrailMarker << endl;
    //     // for(mapIt iterator = result.begin(); iterator != result.end(); iterator++){
    //     //   if(iterator->second == minDistanceToTrailMarker){
    //     //     cout << iterator->first.type << " " << iterator->first.parameter << endl;
    //     //     trail_actions.push_back(iterator->first);
    //     //     break;
    //     //   }
    //     // }
    //     // target_distances.push_back(current_situation_observations[i].pose.getDistance(target));
    //     // double angle_to_target = atan2((target.getY() - current_situation_observations[i].pose.getY()), (target.getX() - current_situation_observations[i].pose.getX()));
    //     // double required_rotation = angle_to_target - current_situation_observations[i].pose.getTheta();
    //     // if(required_rotation > M_PI){
    //     //   required_rotation = required_rotation - (2 * M_PI);
    //     // }
    //     // if(required_rotation < -M_PI){
    //     //   required_rotation = required_rotation + (2 * M_PI);
    //     // }
    //     // target_angles.push_back(required_rotation);
    //     // trail_situation_assignments.push_back(current_situation_observations[i].assignment);
    //     // actual_actions.push_back(current_situation_observations[i].actual_action);
    //     // cout << current_situation_observations[i].pose.getDistance(target) << " " << required_rotation << " " << current_situation_observations[i].assignment << endl;
    //     break;
    //   }
    // }
    // cout << target_distances.size() << " " << target_angles.size() << " " << trail_actions.size() << " " << trail_situation_assignments.size() << endl;
  }
  cout << target_distances.size() << " " << target_angles.size() << " " << trail_actions.size() << " " << trail_situation_assignments.size() << endl;
  for(int i = 0; i < target_distances.size(); i++){
    // cout << "Action Assignment " << i << " : " << target_distances[i] << " " << target_angles[i] << " " << trail_actions[i].type << " " << trail_actions[i].parameter << " " << trail_situation_assignments[i] << endl;
    vector<int> assignment_values;
    assignment_values.push_back(trail_situation_assignments[i]);
    if(target_distances[i] <= 2){
      assignment_values.push_back(2);
    }
    else if(target_distances[i] <= 4){
      assignment_values.push_back(4);
    }
    else if(target_distances[i] <= 8){
      assignment_values.push_back(8);
    }
    else if(target_distances[i] <= 16){
      assignment_values.push_back(16);
    }
    else if(target_distances[i] <= 32){
      assignment_values.push_back(32);
    }
    else if(target_distances[i] <= 64){
      assignment_values.push_back(64);
    }
    else if(target_distances[i] <= 128){
      assignment_values.push_back(128);
    }
    else if(target_distances[i] <= 256){
      assignment_values.push_back(256);
    }

    if(target_angles[i] >= -M_PI/8.0 and target_angles[i] < M_PI/8.0){
      assignment_values.push_back(1);
    }
    else if(target_angles[i] >= M_PI/8.0 and target_angles[i] < 3.0*M_PI/8.0){
      assignment_values.push_back(2);
    }
    else if(target_angles[i] >= 3.0*M_PI/8.0 and target_angles[i] < 5.0*M_PI/8.0){
      assignment_values.push_back(3);
    }
    else if(target_angles[i] >= 5.0*M_PI/8.0 and target_angles[i] < 7.0*M_PI/8.0){
      assignment_values.push_back(4);
    }
    else if(target_angles[i] >= 7.0*M_PI/8.0 or target_angles[i] < -7.0*M_PI/8.0){
      assignment_values.push_back(5);
    }
    else if(target_angles[i] >= -7.0*M_PI/8.0 and target_angles[i] < -5.0*M_PI/8.0){
      assignment_values.push_back(6);
    }
    else if(target_angles[i] >= -5.0*M_PI/8.0 and target_angles[i] < -3.0*M_PI/8.0){
      assignment_values.push_back(7);
    }
    else if(target_angles[i] >= -3.0*M_PI/8.0 and target_angles[i] < -M_PI/8.0){
      assignment_values.push_back(8);
    }
    cout << "Action Assignment " << i << " : " << assignment_values[0] << " " << assignment_values[1] << " " << assignment_values[2] << " " << trail_actions[i].type << " " << trail_actions[i].parameter << endl;
    if(assignment_values.size() == 3){
      if(action_assignments.count(assignment_values) == 0){
        action_assignments.insert(pair< vector<int>, vector<FORRAction> >(assignment_values, vector<FORRAction>()));
        accuracy_action_assignments.insert(pair< vector<int>, vector<double> >(assignment_values, vector<double>()));
      }
      // else{
      //   for(int j = 0; j < action_assignments[assignment_values].size(); j++){
      //     // cout << "Action Prediction " << i << " : " << action_assignments[assignment_values][j].type << " " << action_assignments[assignment_values][j].parameter << endl;
      //   }
      // }
      action_assignments[assignment_values].push_back(trail_actions[i]);
      if(trail_actions[i] == actual_actions[i]){
        accuracy_action_assignments[assignment_values].push_back(1.0);
      }
      else if(trail_actions[i].type == actual_actions[i].type){
        accuracy_action_assignments[assignment_values].push_back(0.5);
      }
      else{
        accuracy_action_assignments[assignment_values].push_back(0.0);
      }
    }
  }
  action_assignment_combinations.clear();
  map< vector<int>, vector<FORRAction> >::iterator aait;
  for(aait = action_assignments.begin(); aait != action_assignments.end(); aait++){
    map< FORRAction, double > action_weights;
    map< FORRAction, double >::iterator awit;
    set<FORRAction> *action_set = agentState->getActionSet();
    set<FORRAction>::iterator actionIter;
    for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
      FORRAction forrAction = *actionIter;
      action_weights[forrAction] = 1.0;
    }
    for(int j = 0; j < aait->second.size(); j++){
      action_weights[aait->second[j]] += 1.0;
    }
    double max_value = 0.0;
    for(awit = action_weights.begin(); awit != action_weights.end(); awit++){
      // cout << awit->first.type << " " << awit->first.parameter << " " << awit->second << endl;
      if(awit->second > max_value){
        max_value = awit->second;
      }
    }

    for(awit = action_weights.begin(); awit != action_weights.end(); awit++){
      // cout << aait->first[0] << " " << aait->first[1] << " " << aait->first[2] << " " << awit->first.type << " " << awit->first.parameter << " " << awit->second-1 << endl;
      vector<double> assignment_counts;
      assignment_counts.push_back(aait->first[0]);
      assignment_counts.push_back(aait->first[1]);
      assignment_counts.push_back(aait->first[2]);
      assignment_counts.push_back(awit->first.type);
      assignment_counts.push_back(awit->first.parameter);
      assignment_counts.push_back(awit->second-1);
      action_assignment_combinations.push_back(assignment_counts);
      awit->second = awit->second / max_value;
    }
    action_assignment_weights[aait->first] = action_weights;
  }
  // cout << "Finished action assignment" << endl;
  // printSituations();
}


//----------------------//------------------------//


double FORRSituations::getWeightForAction(AgentState *agentState, FORRAction action){
  Position target(agentState->getCurrentTask()->getX(),agentState->getCurrentTask()->getY(),0);
  Position curr = agentState->getCurrentPosition();
  double target_dist = curr.getDistance(target);

  double angle_to_target = atan2((target.getY() - curr.getY()), (target.getX() - curr.getX()));
  double required_rotation = angle_to_target - curr.getTheta();
  if(required_rotation > M_PI){
    required_rotation = required_rotation - (2 * M_PI);
  }
  if(required_rotation < -M_PI){
    required_rotation = required_rotation + (2 * M_PI);
  }
  double target_angle = required_rotation;
  int sit_assignment = identifySituationAssignment(agentState->getCurrentLaserScan());
  vector<int> assignment_values;
  assignment_values.push_back(sit_assignment);
  if(target_dist <= 2){
    assignment_values.push_back(2);
  }
  else if(target_dist <= 4){
    assignment_values.push_back(4);
  }
  else if(target_dist <= 8){
    assignment_values.push_back(8);
  }
  else if(target_dist <= 16){
    assignment_values.push_back(16);
  }
  else if(target_dist <= 32){
    assignment_values.push_back(32);
  }
  else if(target_dist <= 64){
    assignment_values.push_back(64);
  }
  else if(target_dist <= 128){
    assignment_values.push_back(128);
  }
  else if(target_dist <= 256){
    assignment_values.push_back(256);
  }

  if(target_angle >= -M_PI/8.0 and target_angle < M_PI/8.0){
    assignment_values.push_back(1);
  }
  else if(target_angle >= M_PI/8.0 and target_angle < 3.0*M_PI/8.0){
    assignment_values.push_back(2);
  }
  else if(target_angle >= 3.0*M_PI/8.0 and target_angle < 5.0*M_PI/8.0){
    assignment_values.push_back(3);
  }
  else if(target_angle >= 5.0*M_PI/8.0 and target_angle < 7.0*M_PI/8.0){
    assignment_values.push_back(4);
  }
  else if(target_angle >= 7.0*M_PI/8.0 or target_angle < -7.0*M_PI/8.0){
    assignment_values.push_back(5);
  }
  else if(target_angle >= -7.0*M_PI/8.0 and target_angle < -5.0*M_PI/8.0){
    assignment_values.push_back(6);
  }
  else if(target_angle >= -5.0*M_PI/8.0 and target_angle < -3.0*M_PI/8.0){
    assignment_values.push_back(7);
  }
  else if(target_angle >= -3.0*M_PI/8.0 and target_angle < -M_PI/8.0){
    assignment_values.push_back(8);
  }
  if(assignment_values.size() == 3){
    if(action_assignments.count(assignment_values) == 0){
      return 1.0;
    }
    else{
      return action_assignment_weights[assignment_values][action];
    }
  }
  return 1.0;
}


//----------------------//------------------------//


double FORRSituations::getAccuracyForSituation(AgentState *agentState){
  Position target(agentState->getCurrentTask()->getX(),agentState->getCurrentTask()->getY(),0);
  Position curr = agentState->getCurrentPosition();
  double target_dist = curr.getDistance(target);

  double angle_to_target = atan2((target.getY() - curr.getY()), (target.getX() - curr.getX()));
  double required_rotation = angle_to_target - curr.getTheta();
  if(required_rotation > M_PI){
    required_rotation = required_rotation - (2 * M_PI);
  }
  if(required_rotation < -M_PI){
    required_rotation = required_rotation + (2 * M_PI);
  }
  double target_angle = required_rotation;
  int sit_assignment = identifySituationAssignment(agentState->getCurrentLaserScan());
  vector<int> assignment_values;
  assignment_values.push_back(sit_assignment);
  if(target_dist <= 2){
    assignment_values.push_back(2);
  }
  else if(target_dist <= 4){
    assignment_values.push_back(4);
  }
  else if(target_dist <= 8){
    assignment_values.push_back(8);
  }
  else if(target_dist <= 16){
    assignment_values.push_back(16);
  }
  else if(target_dist <= 32){
    assignment_values.push_back(32);
  }
  else if(target_dist <= 64){
    assignment_values.push_back(64);
  }
  else if(target_dist <= 128){
    assignment_values.push_back(128);
  }
  else if(target_dist <= 256){
    assignment_values.push_back(256);
  }

  if(target_angle >= -M_PI/8.0 and target_angle < M_PI/8.0){
    assignment_values.push_back(1);
  }
  else if(target_angle >= M_PI/8.0 and target_angle < 3.0*M_PI/8.0){
    assignment_values.push_back(2);
  }
  else if(target_angle >= 3.0*M_PI/8.0 and target_angle < 5.0*M_PI/8.0){
    assignment_values.push_back(3);
  }
  else if(target_angle >= 5.0*M_PI/8.0 and target_angle < 7.0*M_PI/8.0){
    assignment_values.push_back(4);
  }
  else if(target_angle >= 7.0*M_PI/8.0 or target_angle < -7.0*M_PI/8.0){
    assignment_values.push_back(5);
  }
  else if(target_angle >= -7.0*M_PI/8.0 and target_angle < -5.0*M_PI/8.0){
    assignment_values.push_back(6);
  }
  else if(target_angle >= -5.0*M_PI/8.0 and target_angle < -3.0*M_PI/8.0){
    assignment_values.push_back(7);
  }
  else if(target_angle >= -3.0*M_PI/8.0 and target_angle < -M_PI/8.0){
    assignment_values.push_back(8);
  }
  if(assignment_values.size() == 3){
    if(action_assignments.count(assignment_values) == 0){
      return 0.0;
    }
    else{
      double num_instances = accuracy_action_assignments[assignment_values].size();
      double sum_of_instances = accumulate(accuracy_action_assignments[assignment_values].begin(), accuracy_action_assignments[assignment_values].end(), 0.0);
      cout << num_instances << " " << sum_of_instances << " " << sum_of_instances/num_instances << endl;
      return sum_of_instances/num_instances;
    }
  }
  return 0.0;
}


//----------------------//------------------------//


double FORRSituations::overlapBetweenSituations(vector<CartesianPoint> laser, Position pose, vector< vector<int> > comp_grid){
  cout << "overlapBetweenSituations" << endl;
  int dimension = 200;
  vector< vector<int> > grid;
  for(int i = 0; i < dimension; i++){
    vector<int> col;
    for(int j = 0; j < dimension; j++){
      col.push_back(0);
    }
    grid.push_back(col);
  }
  for(int l = 0; l < laser.size(); l++){
    double x1 = pose.getX();
    double y1 = pose.getY();
    double x2 = laser[l].get_x();
    double y2 = laser[l].get_y();
    double step_size = 0.01;
    double tx,ty;
    for(double step = 0; step <= 1; step += step_size){
      tx = (int)((x2 * step) + (x1 * (1-step)));
      ty = (int)((y2 * step) + (y1 * (1-step)));
      if(tx >= 0 and tx < dimension and ty >= 0 and ty < dimension){
        grid[tx][ty] += 1;
      }
    }
  }
  for(int i = 0; i < dimension; i++){
    for(int j = 0; j < dimension; j++){
      cout << grid[i][j] << " ";
    }
    cout << endl;
  }
  cout << endl;
  double comp_grid_count = 0;
  double overlap_count = 0;
  for(int i = 0; i < comp_grid.size(); i++){
    for(int j = 0; j < comp_grid[i].size(); j++){
      cout << comp_grid[i][j] << " ";
      if(comp_grid[i][j] > 0){
        comp_grid_count += 1;
        if(grid[i][j] > 0){
          overlap_count += 1;
        }
      }
    }
    cout << endl;
  }
  cout << endl;
  cout << "comp_grid_count " << comp_grid_count << " overlap_count " << overlap_count << " overlap_percent " << overlap_count / comp_grid_count << endl;
  return overlap_count / comp_grid_count;
}