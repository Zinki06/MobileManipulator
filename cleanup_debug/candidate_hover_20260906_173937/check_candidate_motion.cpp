#include <iostream>
#include <iomanip>
#include "pick_and_place/candidate_grasp.hpp"
int main(){
 double floor;int n;std::vector<double> start(4);
 if(!(std::cin>>floor>>n)||n<0||n>30)return 1;
 for(auto &q:start)if(!(std::cin>>q))return 1;
 std::vector<std::vector<double>> path;
 for(int i=0;i<n;++i){std::vector<double> q(4);for(auto &v:q)if(!(std::cin>>v))return 1;path.push_back(q);}
 auto valid=[](const std::vector<double>&q){
  const double lo[]={-2.8,-1.75,-.92,-1.75},hi[]={2.8,1.55,1.35,2.0};
  for(int j=0;j<4;++j)if(!std::isfinite(q[j])||q[j]<lo[j]||q[j]>hi[j])return false;
  return true;
 };
 if(!valid(start))return 2;for(auto&q:path)if(!valid(q))return 2;
 double clearance=pick_and_place::fingerFloorClearance(start,floor);
 bool safe=clearance>=.006&&pick_and_place::fingerPathClear(start,path,floor);
 std::cout<<std::setprecision(12)<<"{\"safe\":"<<(safe?"true":"false")<<",\"actual_floor_clearance\":"<<clearance<<"}";
}
