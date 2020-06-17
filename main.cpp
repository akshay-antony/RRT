#include "rrt.h"
#include<iostream>
#include<cmath>
#include <vector>
#include<cfloat>
using namespace std;
int main()
{
    float length=10, breadth=10; //length and breadth of navigating space preset
    std::vector<float> start;
    float xstart,ystart,xdest,ydest;
    std::vector<float> destination;
    float destination_radius=0.05;
    int numberOfIterations=20000 ;
    float maxDistance=.3;
    cout<<"\n THE AREA FOR PATH PLANNING IS ASSUMED TO BE A SQUARE OF DIMENSIONS 10*10,PLEASE ENTER START POINT ,END POINT,OBSTACLES SO THAT THEY FIT IN THE SQUARE\n\n";
    cout<<"\n Enter Start Point (0<X,Y,10)  :  ";
    cin>>xstart>>ystart;
    cout<<"\n Enter Destination Point (0<X,Y<10) : ";
    cin>>xdest>>ydest;
    cout<<"\n Enter radius of the destination(destination is assumed a circle with center X,Y and radius) (0<X,Y<10)(for this problem anything greater than 0.1 gives result almost always) :";
    cin>>destination_radius;
    //cout<<"\n enter number of iterations";
    //cin>>numberOfIterations;
    //cout<<"\n enter max step size distance";
    //cin>>maxDistance;
    //std::vector<std::vector<float> > obstacleInput;// assume circular obstacles , each vector is an obstacle defined by center and radius which is a total of 3 elements
    //obstacleInput[0].push_back(1);
    //obstacleInput[0].push_back(1);
    //obstacleInput[0].push_back(1);
    vector<vector<float> > obstacleInput;
    int numberofObstacles;
    cout<<"\n Enter the number of obstacles  :  ";
    cin>>numberofObstacles;
    float X,Y,r;
    cout<<"\n Enter centre and radius of the obstacles (ie enter three paramters X,Y,radius for each obstacle eg (2,2,1) where centre 2,2 radius 1)(enter small radius obstacles so that it does not fill the entire area and no path is found  : \n";
    for(int u=0;u<numberofObstacles;++u)
    {   vector<float> row;
        cin>>X>>Y>>r;
        row.push_back(X);
        row.push_back(Y);
        row.push_back(r);
        obstacleInput.push_back(row);
    }
    //obstacleInput.push_back({1,1,1});
    //obstacleInput.push_back(5,6,.5);
   // obstacleInput.push_back(7,8,.25);
    start.push_back(xstart);
    start.push_back(ystart);
    destination.push_back(xdest);
    destination.push_back(ydest);

    for(int u=0;u<numberofObstacles;++u)
    {  float m,n;
       n=((xdest-obstacleInput[u][0])*(xdest-obstacleInput[u][0])+(ydest-obstacleInput[u][1])*(ydest-obstacleInput[u][1])-obstacleInput[u][2]*obstacleInput[u][2]);
       m=((xstart-obstacleInput[u][0])*(xstart-obstacleInput[u][0])+(ystart-obstacleInput[u][1])*(ystart-obstacleInput[u][1])-obstacleInput[u][2]*obstacleInput[u][2]);
       if( m<=0)
       {
           cout<< "\n start Point and Obstacle Coincides";
           return 0;
       }
       if(n <=0)
       {
           cout<< "\n End Point and Obstacle Coincides";
           return 0;
       }

    }

    rrt RRT;
    RRT.initialize(start,destination,maxDistance,destination_radius,obstacleInput);
    //int u=RRT.check();
    Node* temp= new Node;
    temp->previous= NULL;
    temp->xy.push_back(start[0]);
    temp->xy.push_back(start[1]);
    RRT.tree.push_back(temp);
    int condition=0;
    int cCollision;
    int cCollisionDestination;
    int cStraightLine;
    int h;
    std::vector<float> newCordinates;// randomly generated stores x coordinate in newCordinates[0] and y coordinate in newCordinates[1]
    std::vector<float> actualNewCordinate;
    for( h=0;h<numberOfIterations;++h)
    {  //cout<<"\n loop no :"<<h;
        newCordinates=RRT.generateRandom();
        Node * nearestNode=RRT.Nearest(newCordinates);
        cCollisionDestination=RRT.checkCollisionDestination(nearestNode,newCordinates);
        if(cCollisionDestination==1)

         {   //cout<<"\n loop n "<<h<<" "<<newCordinates[0]<<"  "<<newCordinates[1];
            RRT.AddNode(nearestNode,destination);
            condition=1;
            break;
         }

        actualNewCordinate=RRT.newNodePosition(nearestNode,newCordinates);
       // std::cout<<"Actual New "<<actualNewCordinate[0]<<"  "<<actualNewCordinate[1]<<"\n\n";
        cCollision=RRT.checkCollision(nearestNode,actualNewCordinate);
        if(cCollision==0)
        {   //cout<<"\n Collision occured in point "<<actualNewCordinate[0]<<" "<<actualNewCordinate[1];
            continue;
        }
        condition=RRT.checkReached(nearestNode,actualNewCordinate);
        if(condition==1)
        {
            RRT.AddNode(nearestNode,destination);
            break;
        }

        if(condition==0)
        {
            RRT.AddNode(nearestNode,actualNewCordinate);
        }
    }
    if(condition==1)
    {   std::vector<Node *> finalPath;
        cout<<"\n Path Reached at Iteration number  :"<<" "<<h;
       finalPath= RRT.generatepath();
       int sizen=finalPath.size();
       ofstream f("/home/akshay/results.dat");
       for(int i=sizen-1;i>=0;--i)
       {
           float xtemp=finalPath[i]->xy[0];
           float ytemp=finalPath[i]->xy[1];
           f<<xtemp<<"  "<<ytemp<<"\n";
           std::cout<<"\n"<<xtemp<<" "<<ytemp<<"\n";
       }
    }
    else
        cout<<"Path Not Found"<<" "<<h;
}
