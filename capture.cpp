/*
 *
 *  Author : Surjith Bhagavath Singh
 *
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dirent.h>

using namespace cv;
using namespace std;

float ZOOM = 0.6;
float DX = 62.0, DY = 60.0;

int HRES=320,VRES=240;

Mat map_x,map_y,IR,VI,crop;
int num_of_frames = 240;
int frame_number =0;
int fps;

void update_map()
{
    for( int j = 0; j < IR.rows; j++ )
    { 
        for( int i = 0; i < IR.cols; i++ )
       {
                 map_x.at<float>(j,i) = (1/ZOOM)*(i - IR.cols*(DX/HRES)) ;
                 map_y.at<float>(j,i) = (1/ZOOM)*(j - IR.rows*(DY/VRES)) ;
       }
    }
}

int main( int argc, char** argv )
{
   
    char filename_VI[150],filename_IR[150];
    printf("Usage :\n\t ./capture fps NumberOfFrames");

    if(argv[1])
        sscanf(argv[1],"%i",&fps);

    else 
        fps = 30;

    if(argv[2])
        sscanf(argv[2],"%i",&num_of_frames);
    else
        num_of_frames = 240;

    printf("\n\tFPS = %d \n\tNumberOfFrames = %d\n\tHRES = %d\n\tVRES = %d\n\t ",fps,num_of_frames,HRES,VRES);

    float wait = (1000.0/fps);

    
    VideoWriter output_fused;                                        // Open the output
    
    output_fused.open("capture_fused.avi", CV_FOURCC('P','I','M','1'), fps, Size(HRES, VRES), true);

    if (!output_fused.isOpened())
    {
        cout  << "Could not open the output video for write: " <<  endl;
        return -1;
    }



    while(frame_number <= num_of_frames)
    {   

        frame_number++;
        snprintf(filename_VI,150, "/home/surjith/CV_Research/fusion/litiv2012_dataset/SEQUENCE1/VISIBLE/input/in%06d.jpg",frame_number);
        snprintf(filename_IR,150, "/home/surjith/CV_Research/fusion/litiv2012_dataset/SEQUENCE1/THERMAL/input/in%06d.jpg",frame_number);

        printf("\n\t First File  = %s  \n\t Second File = %s",filename_IR,filename_VI);
    
    
        IR = imread(filename_IR,CV_LOAD_IMAGE_COLOR);
        VI = imread(filename_VI,CV_LOAD_IMAGE_COLOR);
    
        map_x.create(IR.size(),CV_32FC1);

        map_y.create(IR.size(),CV_32FC1);

    

        update_map();
        remap(IR,crop,map_x,map_y,INTER_LINEAR,BORDER_CONSTANT,Scalar(0.6,0.6,0)); 

        Mat fused_image = (VI*0.5 + crop*0.8); 
        putText(fused_image,"TIme",Point2f(280,190),FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255));

        output_fused << fused_image;

    
        char c = cvWaitKey(wait);
        if( c == 27 ) break;
        
        
    }

}