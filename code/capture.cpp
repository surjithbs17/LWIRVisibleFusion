/*
Author: Surjith Bhagvath Singh
Functionality: This code performs real time multispectral fusion and analyses the deadlines

References: Dr.Sam Siewert 's code examples
*/

#include <pthread.h>
#include <stdio.h>
#include <sched.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <errno.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace cv;
using namespace std;

#define HRES 320
#define VRES 480
#define NUM_THREADS		4
#define START_SERVICE 		0
#define HIGH_PRIO_SERVICE 	1
#define MID_PRIO_SERVICE 	2
#define LOW_PRIO_SERVICE 	3
#define NUM_MSGS 		3

//Camera Constants
float ZOOM = 0.6;
float DX = 62.0, DY = 125.0;

//Initializations of global variables
int global_id = 0;
int fps = 30;
char filename_VI[150],filename_IR[150],filename_fused[150];
Mat map_x,map_y,IR,VI,remapped_IR;
struct timespec timenow,timesoon,neededsleep;
int num_frames = 300;
VideoWriter output_fused;
double total_visible=0,total_fusion=0,total_lwir=0;
double sleep_lwir=0,sleep_visible=0,sleep_fusion=0;
int deadlinemiss;


pthread_t threads[NUM_THREADS];
pthread_attr_t rt_sched_attr[NUM_THREADS];
int rt_max_prio, rt_min_prio;
struct sched_param rt_param[NUM_THREADS];
struct sched_param nrt_param;

pthread_mutex_t msgSem;
pthread_mutexattr_t rt_safe;

int rt_protocol;


//Write video functionality
 void write_vid(Mat image)
 {
  output_fused << image;
 }

//timespec difference function
void timespec_diff(struct timespec *start, struct timespec *stop,
struct timespec *result, bool check_neg)
{
  result->tv_sec = stop->tv_sec - start->tv_sec;
  result->tv_nsec = stop->tv_nsec - start->tv_nsec;

  if ( check_neg && result->tv_nsec < 0) {
    result->tv_sec = result->tv_sec - 1;
    result->tv_nsec = result->tv_nsec + 1000000000;
  }
  
  //printf("\ndiff time %lu sec, %lu nsec\n",result->tv_sec,result->tv_nsec);

}

//Timed Unlock 
int timed_unlock(int argument)
{
    struct timespec absolute_time,sleep_time;
    int error;
    double microsleep;
    
    clock_gettime(CLOCK_MONOTONIC, &absolute_time);

    long long int converted_time = ((timenow.tv_sec*1000000000) + timenow.tv_nsec );
    if(argument == 1)
    {
      
      timesoon.tv_sec  = timenow.tv_sec ;
      timesoon.tv_nsec = timenow.tv_nsec+75000000;
      neededsleep.tv_sec = timenow.tv_sec+1;
      neededsleep.tv_nsec= timenow.tv_nsec;

      timespec_diff(&absolute_time,&timesoon,&sleep_time,true);


    }
    else if(argument == 2)
    {
      timesoon.tv_sec  = timenow.tv_sec ;
      timesoon.tv_nsec = timenow.tv_nsec+10000000;

      timespec_diff(&absolute_time,&timesoon,&sleep_time,true);

    }
    else if(argument == 3)
    {
      timesoon.tv_sec  = timenow.tv_sec ;
      timesoon.tv_nsec = timenow.tv_nsec+25000000;

      timespec_diff(&absolute_time,&neededsleep,&sleep_time,true);
    }
    
    

    microsleep = ((sleep_time.tv_sec * 1000000000.0f) + sleep_time.tv_nsec)/1000.0f; 
    
    if(microsleep>=0)
    {
    printf("sleep_time  = %f milliseconds\n",microsleep/1000.0 );
    usleep(microsleep);
      if(argument == 1)
    {
      sleep_lwir = sleep_lwir+microsleep;
    }
    else if(argument == 2)
    {
      sleep_visible = sleep_visible+microsleep;
    }
    else if(argument == 3)
    {
     sleep_fusion = sleep_fusion+microsleep;
    }

    pthread_mutex_unlock(&msgSem);
    
    }
    else if (microsleep<0)
    {
      pthread_mutex_unlock(&msgSem);
      printf("Deadline Missed = %llf\n",microsleep);
      deadlinemiss++;
    }
    
}
 

// Map updation for remap functionality
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


//Time stamp function
Mat time_stamp(Mat image)
{

        time_t rawTime;
        struct tm * timeinfo;
        time (&rawTime);
        timeinfo = localtime(&rawTime);
        printf("timeinfo  %s",asctime(timeinfo));
        putText(image,asctime(timeinfo),Point2f(10,190),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255,255));
        return image;
}

void LWIRRead(int, void*)
{
     snprintf(filename_IR,150, "/home/ubuntu/project/lab6/litiv2012_dataset/SEQUENCE2/THERMAL/input/in%06d.jpg",global_id);
     
     IR = imread(filename_IR,CV_LOAD_IMAGE_COLOR);

     map_x.create(IR.size(),CV_32FC1);

     map_y.create(IR.size(),CV_32FC1);
     
     update_map();

     
    remap(IR,remapped_IR,map_x,map_y,INTER_LINEAR,BORDER_CONSTANT,Scalar(0.6,0.6,0)); 

         

}


//Visible read function
void VISIBLERead(int,void*)
{
   snprintf(filename_VI,150, "/home/ubuntu/project/lab6/litiv2012_dataset/SEQUENCE2/VISIBLE/input/in%06d.jpg",global_id);
   VI = imread(filename_VI,CV_LOAD_IMAGE_COLOR);
 
}

//Fusion
void Fusion(int,void*)
{

  
  
	snprintf(filename_fused,150, "/home/ubuntu/project/lab6/fusion/in%06d.jpg",global_id);
  
  Mat fused_image = (VI*0.5 + remapped_IR*0.6); 

  fused_image = time_stamp(fused_image);
  
  imwrite(filename_fused,fused_image);
   
  
  
}



//Visible thread
void *VISIBLEThread(void *threadid)
{
  struct timespec start,end,diff,lock_start,lock_end,lock_diff;
  
  clock_gettime(CLOCK_MONOTONIC,&lock_start);
  pthread_mutex_lock(&msgSem);
  clock_gettime(CLOCK_MONOTONIC,&timenow);
  clock_gettime(CLOCK_MONOTONIC,&start);
  
  VISIBLERead(0,0);
  clock_gettime(CLOCK_MONOTONIC,&end);
  

  timespec_diff(&start,&end,&diff,true);

  double time_elapsed = (diff.tv_sec * 1000000000.0f) + diff.tv_nsec ;

  printf("Elapsed time for VisibleThread inside lock is %lu Seconds %llf milliseconds\n",diff.tv_sec,diff.tv_nsec/1000000.0);
  clock_gettime(CLOCK_MONOTONIC,&timenow);
  timed_unlock(2);
  clock_gettime(CLOCK_MONOTONIC,&lock_end);

  timespec_diff(&lock_start,&lock_end,&lock_diff,true);
  
  printf("Entire Lock time for VisibleThread is %lu Seconds %llf milliseconds\n\n",lock_diff.tv_sec,lock_diff.tv_nsec/1000000.0);


  double visible_time = (lock_diff.tv_sec * 1000000000.0f) + lock_diff.tv_nsec ;
  total_visible = total_visible + visible_time;

}

//FusionThread
void *FusionThread(void *threadid)
{
  struct timespec start,end,diff,lock_start,lock_end,lock_diff;
  
  clock_gettime(CLOCK_MONOTONIC,&lock_start);
  pthread_mutex_lock(&msgSem);
  clock_gettime(CLOCK_MONOTONIC,&timenow);
  clock_gettime(CLOCK_MONOTONIC,&start);
  
  Fusion(0,0);
  clock_gettime(CLOCK_MONOTONIC,&end);
  

  timespec_diff(&start,&end,&diff,true);

  double time_elapsed = (diff.tv_sec * 1000000000.0f) + diff.tv_nsec ;

  printf("Elapsed time for FusionThread inside lock is %lu Seconds %llf milliseconds\n",diff.tv_sec,diff.tv_nsec/1000000.0);
  clock_gettime(CLOCK_MONOTONIC,&timenow);
  timed_unlock(3);
  clock_gettime(CLOCK_MONOTONIC,&lock_end);

  timespec_diff(&lock_start,&lock_end,&lock_diff,true);
  
  printf("Entire Lock time for FusionThread is %lu Seconds %llf milliseconds\n\n",lock_diff.tv_sec,lock_diff.tv_nsec/1000000.0);


  double fusion_time = (lock_diff.tv_sec * 1000000000.0f) + lock_diff.tv_nsec ;
  total_fusion = total_fusion + fusion_time;

}


//LWIR Thread
void *LWIRThread(void *threadid)
{
  struct timespec start,end,diff,lock_start,lock_end,lock_diff;
  
  clock_gettime(CLOCK_MONOTONIC,&lock_start);
  pthread_mutex_lock(&msgSem);
  clock_gettime(CLOCK_MONOTONIC,&timenow);
  clock_gettime(CLOCK_MONOTONIC,&start);
  
  LWIRRead(0,0);
  clock_gettime(CLOCK_MONOTONIC,&end);
  

  timespec_diff(&start,&end,&diff,true);

  double time_elapsed = (diff.tv_sec * 1000000000.0f) + diff.tv_nsec ;

  printf("Elapsed time for LWIRThread inside lock is %lu Seconds %llf milliseconds\n",diff.tv_sec,diff.tv_nsec/1000000.0);

  timed_unlock(1);
  clock_gettime(CLOCK_MONOTONIC,&lock_end);

  timespec_diff(&lock_start,&lock_end,&lock_diff,true);
  
  printf("Entire Lock time for LWIRThread is %lu Seconds %llf milliseconds\n\n",lock_diff.tv_sec,lock_diff.tv_nsec/1000000.0);

  double lwir_time = (lock_diff.tv_sec * 1000000000.0f) + lock_diff.tv_nsec ;
  total_lwir = total_lwir + lwir_time;

}


//Main transform
void *mytransform(void *threadid)
{
  printf ("\nInside transform function ");
	pthread_mutex_lock(&msgSem);
  pthread_mutex_unlock(&msgSem);
  cpu_set_t cpus;
   
    
for(global_id = 1;global_id <= num_frames; global_id++)
{
for (int i = 0; i < 3; i++) 
{
    CPU_ZERO(&cpus);
    CPU_SET(i, &cpus);

    int rc;

    if(i==0)
    {
      pthread_attr_setaffinity_np(&rt_sched_attr[HIGH_PRIO_SERVICE], sizeof(cpu_set_t), &cpus);
      rc = pthread_create(&threads[HIGH_PRIO_SERVICE], &rt_sched_attr[HIGH_PRIO_SERVICE], LWIRThread, (void *)HIGH_PRIO_SERVICE);

      if (rc)
      {
       printf("ERROR; pthread_create() rc is %d\n", rc);
       perror(NULL);
       exit(-1);
      }
    }

    if(i==1)
    {
      pthread_attr_setaffinity_np(&rt_sched_attr[MID_PRIO_SERVICE], sizeof(cpu_set_t), &cpus);

      rc = pthread_create(&threads[MID_PRIO_SERVICE], &rt_sched_attr[HIGH_PRIO_SERVICE], VISIBLEThread, (void *)MID_PRIO_SERVICE);

      if (rc)
      {
       printf("ERROR; pthread_create() rc is %d\n", rc);
       perror(NULL);
       exit(-1);
      }
    }

     if(i==2)
      {
        pthread_attr_setaffinity_np(&rt_sched_attr[LOW_PRIO_SERVICE], sizeof(cpu_set_t), &cpus);

        rc = pthread_create(&threads[LOW_PRIO_SERVICE], &rt_sched_attr[HIGH_PRIO_SERVICE], FusionThread, (void *)LOW_PRIO_SERVICE);

        if (rc)
        {
          printf("ERROR; pthread_create() rc is %d\n", rc);
          perror(NULL);
          exit(-1);
        }
      }

	
  pthread_join(threads[HIGH_PRIO_SERVICE], NULL);
  pthread_join(threads[MID_PRIO_SERVICE], NULL);
  pthread_join(threads[LOW_PRIO_SERVICE], NULL);
  
}


}

double avg_lwir = total_lwir/num_frames;
double avg_visible = total_visible/num_frames;
double avg_fusion = total_fusion/num_frames;

double avg_sleep_lwir = sleep_lwir/num_frames;
double avg_sleep_visible = sleep_visible/num_frames;
double avg_sleep_fusion = sleep_fusion/num_frames;

printf("Average time for LWIR \t= %llf \nAverage time for Visible \t= %llf \nAverage time for Fusion \t= %llf \nDeadlines Missed \t=%d",avg_lwir,avg_visible,avg_fusion,deadlinemiss);

printf("\n\nAverage Sleep time for LWIR \t= %llf \nAverage Sleep time for Visible \t= %llf \nAverage Sleep time for Fusion \t= %llf \n",avg_sleep_lwir,avg_sleep_visible,avg_sleep_fusion);


}




//Scheduler print function 
void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
     case SCHED_FIFO:
	   printf("Pthread Policy is SCHED_FIFO\n");
	   break;
     case SCHED_OTHER:
	   printf("Pthread Policy is SCHED_OTHER\n");
       break;
     case SCHED_RR:
	   printf("Pthread Policy is SCHED_OTHER\n");
	   break;
     default:
       printf("Pthread Policy is UNKNOWN\n");
   }
}

//Main function
int main ()
{
   int rc, invSafe=0, i, scope;
   struct timespec sleepTime, dTime;

   print_scheduler();

   pthread_attr_init(&rt_sched_attr[START_SERVICE]);
   pthread_attr_setinheritsched(&rt_sched_attr[START_SERVICE], PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy(&rt_sched_attr[START_SERVICE], SCHED_FIFO);

   pthread_attr_init(&rt_sched_attr[HIGH_PRIO_SERVICE]);
   pthread_attr_setinheritsched(&rt_sched_attr[HIGH_PRIO_SERVICE], PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy(&rt_sched_attr[HIGH_PRIO_SERVICE], SCHED_FIFO);

   pthread_attr_init(&rt_sched_attr[LOW_PRIO_SERVICE]);
   pthread_attr_setinheritsched(&rt_sched_attr[LOW_PRIO_SERVICE], PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy(&rt_sched_attr[LOW_PRIO_SERVICE], SCHED_FIFO);

   rt_max_prio = sched_get_priority_max(SCHED_FIFO);
   rt_min_prio = sched_get_priority_min(SCHED_FIFO);

   rc=sched_getparam(getpid(), &nrt_param);

   if (rc) 
   {
       printf("ERROR; sched_setscheduler rc is %d\n", rc);
       perror(NULL);
       exit(-1);
   }

   print_scheduler();

   printf("min prio = %d, max prio = %d\n", rt_min_prio, rt_max_prio);
   pthread_attr_getscope(&rt_sched_attr[START_SERVICE], &scope);

   if(scope == PTHREAD_SCOPE_SYSTEM)
     printf("PTHREAD SCOPE SYSTEM\n");
   else if (scope == PTHREAD_SCOPE_PROCESS)
     printf("PTHREAD SCOPE PROCESS\n");
   else
     printf("PTHREAD SCOPE UNKNOWN\n");

   pthread_mutex_init(&msgSem, NULL);

   rt_param[START_SERVICE].sched_priority = rt_max_prio;
   pthread_attr_setschedparam(&rt_sched_attr[START_SERVICE], &rt_param[START_SERVICE]);

   printf("Creating thread %d\n", START_SERVICE);
   rc = pthread_create(&threads[START_SERVICE], &rt_sched_attr[START_SERVICE], mytransform, (void *)START_SERVICE);

   if (rc)
   {
       printf("ERROR; pthread_create() rc is %d\n", rc);
       perror(NULL);
       exit(-1);
   }
   printf("Start services thread spawned\n");


   printf("will join service threads\n");

   if(pthread_join(threads[START_SERVICE], NULL) == 0)
     printf("START SERVICE done\n");
   else
     perror("START SERVICE");


   rc=sched_setscheduler(getpid(), SCHED_OTHER, &nrt_param);

   if(pthread_mutex_destroy(&msgSem) != 0)
     perror("mutex destroy");
    
   printf("All done\n");

   exit(0);
}



