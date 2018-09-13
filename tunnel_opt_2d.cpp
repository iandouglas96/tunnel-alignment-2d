#include <iostream>
#include "tunnel_opt_2d.h"
#include <SFML/Graphics.hpp>

int main() 
{
    //3 dimensions in both edges and nodes
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>> BlockSolver;
	typedef g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType> LinearSolver;

    //Initialization
    LinearSolver *solver = new LinearSolver();
	BlockSolver *blockSolver = new BlockSolver(solver);
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
	graph.setAlgorithm(algorithm);
	
    graph.setVerbose(false); // printOptimizationInfo
	solver->setWriteDebug(true);
	blockSolver->setWriteDebug(true);
	algorithm->setWriteDebug(true);

	numRobotVertices = 0;
	numRobotEdges = 0;

	buildGraph();
	optimizeGraph();
	drawGraph();

    return 0;
}

void optimizeGraph()
{
	std::cout << "Initial:\n";
	std::cout << "Robots:\n";
	for (auto vertex : robotPoseVertexList) {
		std::cout << vertex->estimate().matrix() << "\n\n";
	}
	std::cout << "Tunnels:\n";
	for (auto vertex : tunnelPoseVertexList) {
		std::cout << vertex->estimate().matrix() << "\n\n";
	}

	graph.initializeOptimization();
	graph.optimize(100, false);

	std::cout << "\nFinal:\n";
	std::cout << "Robots:\n";
	for (auto vertex : robotPoseVertexList) {
		std::cout << vertex->estimate().matrix() << "\n\n";
	}
	std::cout << "Tunnels:\n";
	for (auto vertex : tunnelPoseVertexList) {
		std::cout << vertex->estimate().matrix() << "\n\n";
	}
}

void buildGraph()
{
	//Hardcode construction of a graph for testing
	robotPoseVertexList.push_back(addRobotVertex(false, Sophus::SE2d()));
	robotPoseVertexList.push_back(addRobotVertex(false, Sophus::SE2d()));

	tunnelPoseVertexList.push_back(addTunnelVertex(true, Sophus::SE2d()));
	tunnelPoseVertexList.push_back(addTunnelVertex(false, Sophus::SE2d()));

	Sophus::SE2d edge_trans = Sophus::SE2d();
	Eigen::Vector2d rot;
	/*rot << cos(M_PI/4),sin(M_PI/4);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << 1, 0;
	robotPoseEdgeList.push_back(addRobotEdge(edge_trans, Eigen::Matrix3d::Identity(), robotPoseVertexList[0], robotPoseVertexList[1]));*/

	rot << cos(0),sin(0);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << 0.8, 0;
	robotPoseEdgeList.push_back(addRobotEdge(edge_trans, Eigen::Matrix3d::Identity(), robotPoseVertexList[0], robotPoseVertexList[1]));

	rot << cos(0),sin(0);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << 0, -1;
	robotPoseEdgeList.push_back(addRobotEdge(edge_trans, Eigen::Matrix3d::Identity(), robotPoseVertexList[0], tunnelPoseVertexList[0]));

	rot << cos(M_PI/2),sin(M_PI/2);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << 0, -1;
	robotPoseEdgeList.push_back(addRobotEdge(edge_trans, Eigen::Matrix3d::Identity(), robotPoseVertexList[1], tunnelPoseVertexList[1]));
}

void drawGraph()
{
	sf::RenderWindow window(sf::VideoMode(800, 800), "Pose Graph Render");
    sf::CircleShape shape(100.f);
    shape.setFillColor(sf::Color::Green);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        window.draw(shape);
        window.display();
    }
}

TunnelOrient* addTunnelVertex(bool fixed, Sophus::SE2d estimate) 
{
	TunnelOrient* vertex = new TunnelOrient();
	vertex->setId(numRobotVertices++);

	vertex->setFixed(fixed);

	vertex->setEstimate(estimate);
	vertex->setMarginalized(false);
	graph.addVertex(vertex);

	return vertex;
}

VertexSE2* addRobotVertex(bool fixed, Sophus::SE2d estimate) 
{
	VertexSE2* vertex = new VertexSE2();
	vertex->setId(numRobotVertices++);

	vertex->setFixed(fixed);

	vertex->setEstimate(estimate);
	vertex->setMarginalized(false);
	graph.addVertex(vertex);

	return vertex;
}

EdgeSE2* addRobotEdge(Sophus::SE2d meas, Eigen::Matrix3d cov, VertexSE2* v0, VertexSE2* v1)
{
	EdgeSE2* edge = new EdgeSE2();
	edge->setId(numRobotEdges++);
	edge->setMeasurement(meas);
	edge->setInformation(cov);
	//edge->setRobustKernel(constraint->robustKernel);

	edge->resize(2);

	edge->setVertex(0, v0);
	edge->setVertex(1, v1);
	graph.addEdge(edge);

	return edge;
}