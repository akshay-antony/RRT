#include "rrt.h"

rrt::rrt()
{
    //ctor
}
void rrt::initialize(std::vector<float> &a,std::vector<float> &b,float c,float d,std::vector<std::vector<float> > &e)
{
    startPoint=a;
    endPoint=b;
    maxNodeDistance=c;
    endPointRadius=d;
    obstacle=e;

}
std::vector<float> rrt ::generateRandom()
{
    std::vector<float> newCordinates1;
    newCordinates1.push_back(10*((float) rand()/RAND_MAX));// generate random float btw 0 and 1 and multiply it with 10 ie area breadth
    newCordinates1.push_back(10*((float) rand()/RAND_MAX));
    //std::cout<<"\n Random Nodes  "<<newCordinates1[0]<<" "<<newCordinates1[1];
    return newCordinates1;

}
//int rrt::check()

Node* rrt::Nearest(std::vector<float> & a)
{
    float minimum=FLT_MAX;
    int minimumNodeNumber=0;
    for(int i=0;i<tree.size();++i)
    {
      float tempDistance=distance(a,tree[i]->xy);
      if(tempDistance<minimum)
      {
          minimum=tempDistance;
          minimumNodeNumber=i;
      }

    }
    minCurrentNodeDistance =minimum;
     //std::cout<<"  \n "<<minimum<<" \n Neartest Node  "<<tree[minimumNodeNumber]->xy[0]<<"   "<<tree[minimumNodeNumber]->xy[1]<<"\n";
    return tree[minimumNodeNumber];

}
float rrt:: distance(std::vector<float> &a,std::vector<float> &b)
{
    float p;
    p=sqrt(pow((a[0]-b[0]),2)+pow((a[1]-b[1]),2));
    return p;
}
std::vector<float> rrt::newNodePosition(Node* a,std::vector<float> &b)
{
    std::vector<float> temp;
    if(minCurrentNodeDistance<=maxNodeDistance)
        return b;
    else
    {
       float x1,x2,y1,y2;
       float slope=(b[1]-a->xy[1])/(b[0]-a->xy[0]);
       x1=a->xy[0]+maxNodeDistance*(1/sqrt(1+pow(slope,2)));
       x2=a->xy[0]-maxNodeDistance*(1/sqrt(1+pow(slope,2)));
       y1=a->xy[1]+slope*maxNodeDistance*(1/sqrt(1+pow(slope,2)));
       y2=a->xy[1]-slope*maxNodeDistance*(1/sqrt(1+pow(slope,2)));
       if((x1>a->xy[0] && x1>b[0]) || (x1<a->xy[0] && x1<b[0]))
         {
             temp.push_back(x2);
             temp.push_back(y2);
         }
       else
       {
           temp.push_back(x1);
           temp.push_back(y1);
       }
    }
   //std::cout<<"\n  "<<temp[0]<<"  "<<temp[1];
    return temp;
}
int rrt::checkCollision(Node * q,std::vector<float> &w)
{
  for(int i=0;i<obstacle.size();++i)
  {

    float a=(w[0]-q->xy[0])*(w[0]-q->xy[0])+(w[1]-q->xy[1])*(w[1]-q->xy[1]);
    float b=2*((q->xy[0]-obstacle[i][0])*(w[0]-q->xy[0])+(w[1]-q->xy[1])*(q->xy[1]-obstacle[i][1]));
    float c=(q->xy[0]-obstacle[i][0])*(q->xy[0]-obstacle[i][0])+(q->xy[1]-obstacle[i][1])*(q->xy[1]-obstacle[i][1])-obstacle[i][2];
    float discriminant=b*b-4*a*c;
    if (discriminant<0)
        continue;
    if(discriminant>=0)
    {
        discriminant=sqrt(discriminant);
        float t1=(-b-discriminant)/(2*a);
        float t2=(-b+discriminant)/(2*a);
          if( t1 >= 0 && t1 <= 1 )
            return 0;
          if(t2>=0 && t2<=1)
            return 0;
      continue;
    }

  }
  return 1;
}
int rrt::checkCollisionDestination(Node *a,std::vector<float> &b)
{   float lineparameter1,lineparameter2,lineparameter3,checkPointLiesInsideCircle;//ax+by+c=0
    float slope=(b[1]-a->xy[1])/(b[0]-a->xy[0]);
    lineparameter2=1;//y
    lineparameter1=-slope;
    lineparameter3=slope*(a->xy[0])-a->xy[1];
    //std::cout<<" a"<<lineparameter1<<"  b"<<lineparameter2<<"  c"<<lineparameter3;
    float perpendicularDistance=abs(lineparameter1*endPoint[0]+lineparameter2*endPoint[1]+lineparameter3)/( sqrt( pow(lineparameter1,2)+pow(lineparameter2,2) ));
    checkPointLiesInsideCircle=pow(b[0]-endPoint[0],2)+pow(b[1]-endPoint[1],2)-endPointRadius*endPointRadius;
    //std::cout<<"\nPerpendicular"<<perpendicularDistance;
    if(perpendicularDistance<=endPointRadius && checkPointLiesInsideCircle<=0)
        return(checkStraightLine(a,endPoint));
    //else if(perpendicularDistance<=endPointRadius && checkPointLiesInsideCircle>0)
       // return 2;
    else
        return 0;

}
int rrt::checkStraightLine(Node* q,std::vector<float> & w)
{ for(int i=0;i<obstacle.size();++i)
  {

    float a=(w[0]-q->xy[0])*(w[0]-q->xy[0])+(w[1]-q->xy[1])*(w[1]-q->xy[1]);
    float b=2*((q->xy[0]-obstacle[i][0])*(w[0]-q->xy[0])+(w[1]-q->xy[1])*(q->xy[1]-obstacle[i][1]));
    float c=(q->xy[0]-obstacle[i][0])*(q->xy[0]-obstacle[i][0])+(q->xy[1]-obstacle[i][1])*(q->xy[1]-obstacle[i][1])-obstacle[i][2];
    float discriminant=b*b-4*a*c;
    if (discriminant<0)
        continue;
    if(discriminant>=0)
    {
        discriminant=sqrt(discriminant);
        float t1=(-b-discriminant)/(2*a);
        float t2=(-b+discriminant)/(2*a);
          if( t1 >= 0 && t1 <= 1 )
            return 0;
          if(t2>=0 && t2<=1)
            return 0;
      continue;
    }

}
return 1;
}
int rrt::checkReached(Node* a,std::vector<float> &b)
{
    float lineparameter1,lineparameter2,lineparameter3,checkPointLiesInsideCircle;//ax+by+c=0
    float slope=(b[1]-a->xy[1])/(b[0]-a->xy[0]);
    lineparameter2=1;//y
    lineparameter1=-slope;
    lineparameter3=slope*(a->xy[0])-a->xy[1];
    //std::cout<<" a"<<lineparameter1<<"  b"<<lineparameter2<<"  c"<<lineparameter3;
    float perpendicularDistance=abs(lineparameter1*endPoint[0]+lineparameter2*endPoint[1]+lineparameter3)/( sqrt( pow(lineparameter1,2)+pow(lineparameter2,2) ));
    checkPointLiesInsideCircle=pow(b[0]-endPoint[0],2)+pow(b[1]-endPoint[1],2)-endPointRadius*endPointRadius;
    //std::cout<<"\nPerpendicular"<<perpendicularDistance;
    if(perpendicularDistance<=endPointRadius && checkPointLiesInsideCircle<=0)
        return 1;
    //else if(perpendicularDistance<=endPointRadius && checkPointLiesInsideCircle>0)
       // return 2;
    else
        return 0;
}
void rrt:: AddNode(Node* a,std::vector<float> &b)
{
    Node *temp= new Node;
    temp->previous=a;
    temp->xy=b;
    tree.push_back(temp);
    finalNode=temp;
    //std::cout<<temp->xy[0]<<" "<<temp->xy[1];
}
std::vector<Node*> rrt::generatepath()
{
    std::vector<Node*> tempPath;//Stores the path
    Node* temp=new Node;
    temp=finalNode;
    while(temp !=NULL)
    {
        tempPath.push_back(temp);
        temp=temp->previous;
    }
    return tempPath;
}
