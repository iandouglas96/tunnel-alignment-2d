#pragma once

#include "types.h"

//////////////////////////////////////////
//Globals
//////////////////////////////////////////

/** Pose graph representation in g2o */
g2o::SparseOptimizer graph;

VertexSE2* addRobotVertex(bool fixed, Sophus::SE2d estimate);
TunnelOrient* addTunnelVertex(bool fixed, Sophus::SE2d estimate);
EdgeSE2* addRobotEdge(Sophus::SE2d meas, Eigen::Matrix3d cov, VertexSE2* v0, VertexSE2* v1);

void buildGraph();
void optimizeGraph();
void drawGraph();

int numRobotVertices;
int numRobotEdges;

std::vector<VertexSE2*> robotPoseVertexList;
std::vector<EdgeSE2*> robotPoseEdgeList;
std::vector<TunnelOrient*> tunnelPoseVertexList;