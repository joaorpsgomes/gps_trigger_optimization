#include <rtabmap/core/Link.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/optimizer/OptimizerGTSAM.h>
#include <rtabmap/core/optimizer/OptimizerCeres.h>
#include <iostream>
#include <bits/stdc++.h>
#include <map>
#include "CImg.h"

#include "../include/gps_handler.h"

#include <iomanip>

using namespace std;
using namespace cv;

#define GTSAM 1

void display_poses(std::map<int, rtabmap::Transform> poses,
                   std::map<int, rtabmap::Transform> poses_optimized);

int main(){
    
    



    /*
        OptimizerCeres.cpp 
            std::map<int, Transform> OptimizerCeres::optimize(
                                                            [ ] int rootId,
                                                            [ ] const std::map<int, Transform> & poses,
                                                            [ ] const std::multimap<int, Link> & edgeConstraints,
                                                            [ ] cv::Mat & outputCovariance,
                                                            [ ] std::list<std::map<int, Transform> > * intermediateGraphes, // contains poses after tree init to last one before the end
                                                            [ ] double * finalError,
                                                            [ ] int * iterationsDone
                                                              )
                                                    




    */


    ///////////////// argumentos da função de otimização///////////////// 
   int root_id=1;
    

    std::map<int, rtabmap::Transform> poses;
    std::map<int, rtabmap::Transform> poses_optimized;
    std::vector<rtabmap::Transform> poses_vector;






    ////// Criar poses //////
    rtabmap::Transform poses_aux = rtabmap::Transform(0,0,0,0,0,0,1);
    poses_vector.push_back(poses_aux);

    poses_aux = rtabmap::Transform(1,0.2,0,0,0,0,1);
    poses_vector.push_back(poses_aux);

    poses_aux = rtabmap::Transform(2,0.6,0,0,0,0,1);
    poses_vector.push_back(poses_aux);

    poses_aux = rtabmap::Transform(3,0.2,0,0,0,0,1);
    poses_vector.push_back(poses_aux);

    //poses_aux = rtabmap::Transform(-0.6,0.4,0,0,0,0,1);
    //poses_vector.push_back(poses_aux);

    ////////////////////////





    
    std::multimap<int, rtabmap::Link>  edgeConstraints;
    rtabmap::Link::Type type=rtabmap::Link::Type::kNeighbor;  
    std::vector<rtabmap::Link> edges_vector;






    ////// Criar Edges //////
    rtabmap::Transform transformation=rtabmap::Transform(1,0.2,0,0,0,0,1);
    rtabmap::Link link = rtabmap::Link(1,2,type,transformation);
    edges_vector.push_back(link);

    transformation=rtabmap::Transform(1,0.1,0,0,0,0,1);
    link = rtabmap::Link(2,3,type,transformation);
    edges_vector.push_back(link);
    
    transformation=rtabmap::Transform(1,-0.1,0,0,0,0,1);
    link = rtabmap::Link(3,4,type,transformation);
    edges_vector.push_back(link);

    type=rtabmap::Link::Type::kPosePrior;
    transformation=rtabmap::Transform(3,0,0,0,0,0,1);
    link = rtabmap::Link(4,4,type,transformation);
    edges_vector.push_back(link);

    transformation=rtabmap::Transform(1,0,0,0,0,0,1);
    link = rtabmap::Link(2,2,type,transformation);
    edges_vector.push_back(link);

    transformation=rtabmap::Transform(0,0,0,0,0,0,1);
    link = rtabmap::Link(1,1,type,transformation);
    edges_vector.push_back(link);


    
    /*type=rtabmap::Link::Type::kPosePrior; //Fazer loop closure do 5 -> 1
    link = rtabmap::Link(5,1,type,transformation);
    transformation=rtabmap::Transform(1,0,0,0,0,0,1);
    edges_vector.push_back(link);*/
    ////////////////////////















    
    cv::Mat outputCovariance;


    std::list<std::map<int, rtabmap::Transform> > * intermediateGraphes;//não é usado na função do ceres


    double finalError=1;


    int iterationsDone=1;

    /////////////////////////////////////////////////////////


    
    
    const rtabmap::ParametersMap params = rtabmap::Parameters::getDefaultParameters();
    pair<int, rtabmap::Link> link_aux;
    pair<int, rtabmap::Transform> pose_aux;      
    //assert(poses_vector.size()==edges_vector.size()); // in case of loop
    //assert(poses_vector.size()==edges_vector.size()); // with no loop
    

    for(int i=0; i<poses_vector.size();i++){

        
        link_aux.first=i+1;
        link_aux.second=edges_vector.at(i);

        printf("link %d: %d->%d transform:(%1.3f,%1.3f,%1.3f)\n",link_aux.first
                                                       ,edges_vector.at(i).from()
                                                       ,edges_vector.at(i).to()
                                                       ,edges_vector.at(i).transform().x()
                                                       ,edges_vector.at(i).transform().y()
                                                       ,edges_vector.at(i).transform().z()
                                                       );

        edgeConstraints.insert(link_aux);

        pose_aux.first=i+1;
        pose_aux.second=poses_vector.at(i);
        printf("Pose %d: (%1.3f,%1.3f,%1.3f)\n\n",pose_aux.first
                                      ,poses_vector.at(i).x()
                                      ,poses_vector.at(i).y()
                                      ,poses_vector.at(i).z());
        poses.insert(pose_aux);
        

    }   
    
    #if GTSAM
    
        rtabmap::OptimizerGTSAM gtsam(params);
        gtsam.setPriorsIgnored(false);
        poses_optimized=gtsam.rtabmap::OptimizerGTSAM::optimize(root_id, poses, edgeConstraints,outputCovariance,intermediateGraphes,&finalError,&iterationsDone);
        cout << "Using GTSAM\n";

        
    
    #else
        rtabmap::OptimizerCeres ceres(params);
        poses_optimized=ceres.rtabmap::OptimizerCeres::optimize(root_id, poses, edgeConstraints,outputCovariance,intermediateGraphes,&finalError,&iterationsDone);

    #endif
    
    std::cout << "Error: " << finalError << std::endl;  

    /////////////// Print result ///////////////////

    

    for (auto& t : poses_optimized){

        printf("Pose %d: (%1.3f,%1.3f,%1.3f)\n\n",t.first
                                                 ,t.second.x()
                                                 ,t.second.y()
                                                 ,t.second.z()
                                                 );
                 
    }
    





    ////////////////////////////////////////////////    
    
    //cout<<distance(40.634494,-8.659920,40.634148,-8.659719)*1000<<"m"<<endl;

    display_poses(poses, poses_optimized);
    

    return 0;
}


void display_poses(std::map<int, rtabmap::Transform> poses,
                   std::map<int, rtabmap::Transform> poses_optimized){


    unsigned int w = 800;
    unsigned int h = 800;
    
    const unsigned char blue[] = {0,0,255};
    const unsigned char red[] = {255,0,0};
    const int scale = 100;
    cimg_library::CImg<unsigned char> bg(w,h,1,3,255);
    for (auto& t : poses){
        bg.draw_circle((t.second.x()*scale)+w/2,h/2-(t.second.y()*scale),w/30,blue);
    }

    for (auto& t : poses_optimized){
        bg.draw_circle((t.second.x()*scale)+w/2,h/2-(t.second.y()*scale),w/30,red);
    }
    
    
    
    //bg.draw_circle(w/2, h/2, 100, blue);
    //bg.draw_line(w/2+70.7,h/2+70.7,w, 0, blue);
    cimg_library::CImgDisplay dsp(w,h,"Graph",0);
    dsp.display(bg);
    //bg.display();

    std::getchar();


}



