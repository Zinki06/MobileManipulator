#include <iostream>
#include "pick_and_place/grasp_kinematics.hpp"
int main(){
 double x,y,z,floor;
 while(std::cin>>x>>y>>z>>floor){
  int count=0;double best=0;
  for(double pitch=-80;pitch<=0;pitch+=2.5){
   std::vector<double> up,down;
   double top=z+.05-pick_and_place::fingerOffsetZ(pitch*M_PI/180);
   double body=std::max(z-std::min(.020,.4*(z-floor)),floor+.008-pick_and_place::fingerOffsetZ(pitch*M_PI/180));
   if(z-body<.004)continue;
   if(pick_and_place::solve4DofIK(x,y,top,up,pitch,false)&&
      pick_and_place::solve4DofIK(x,y,body,down,pitch,false)&&
      pick_and_place::fingerPathClear(up,{down},floor)){
    if(!count)best=pitch;
    count++;
   }
  }
  std::cout<<count<<" "<<best<<"\n";
 }
}
