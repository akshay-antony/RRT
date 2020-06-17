#ifndef RRT_H
#define RRT_H
#include<iostream>
#include<cmath>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
//#include< bits/stdc++.h>
#include<cfloat>

struct Node
{
    Node * previous;
    std::vector<float> xy; // stores x coordinate in xy[0] and y coordinate in xy[1]
};
class rrt
{
    public:
        rrt();
        std::vector<Node *> tree;
        std::vector<Node *> path;
        Node* finalNode;
        std::vector<float> startPoint;
        std::vector<float> endPoint;
        float endPointRadius;//consider round goal
        float maxNodeDistance;
        float minCurrentNodeDistance;
        //int check();
        std::vector<std::vector<float> > obstacle;
        std::vector<float> generateRandom();
        int checkCollisionDestination(Node *a,std::vector<float>&b);
        int checkStraightLine(Node *a,std::vector<float>& b);
        void initialize(std::vector<float> &a,std::vector<float> &b,float m,float d,std::vector<std::vector<float> > &e);
        int checkCollision(Node * q,std::vector<float> &w);
        Node* Nearest(std::vector<float> & a);
        float distance(std::vector<float> &a,std::vector<float> &b);
        std::vector<float> newNodePosition(Node* a,std::vector<float> &b);
        int checkReached(Node *a, std::vector<float> & b);
        void AddNode(Node * a,std::vector<float> &b);
        std::vector<Node *> generatepath();



    protected:

    private:
};

#endif // RRT_H
