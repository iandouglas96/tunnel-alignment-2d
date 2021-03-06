#include <iostream>
#include "tunnel_opt_2d.h"
#include <SFML/Graphics.hpp>

int main() 
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> BlockSolver;
	typedef g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType> LinearSolver;

    //Initialization
    LinearSolver *solver = new LinearSolver();
	BlockSolver *blockSolver = new BlockSolver(solver);
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
	graph.setAlgorithm(algorithm);
	
    graph.setVerbose(true); // printOptimizationInfo
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
	Sophus::SE2d edge_trans = Sophus::SE2d();
	Eigen::Vector2d rot;

	//Hardcode construction of a graph for testing
	//Create nodes
	robotPoseVertexList.push_back(addRobotVertex(false, Sophus::SE2d()));
	robotPoseVertexList.push_back(addRobotVertex(false, Sophus::SE2d()));
	robotPoseVertexList.push_back(addRobotVertex(false, Sophus::SE2d()));

	tunnelPoseVertexList.push_back(addTunnelVertex(true, Sophus::SE2d()));
	tunnelPoseVertexList.push_back(addTunnelVertex(false, Sophus::SE2d()));
	tunnelPoseVertexList.push_back(addTunnelVertex(false, Sophus::SE2d()));

	rot << cos(0),sin(0);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << -1, 0;
	tunnelPoseVertexList[0]->setPose(-1, edge_trans);
	edge_trans.translation() << 1, 0.2;
	rot << cos(M_PI/15),sin(M_PI/15);
	edge_trans.so2().setComplex(rot);
	tunnelPoseVertexList[0]->setPose(1, edge_trans);

	edge_trans.translation() << -1, 0.2;
	rot << cos(-M_PI/13),sin(-M_PI/13);
	edge_trans.so2().setComplex(rot);
	tunnelPoseVertexList[1]->setPose(-1, edge_trans);
	rot << cos(0),sin(0);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << 1, 0;
	tunnelPoseVertexList[1]->setPose(1, edge_trans);

	edge_trans.translation() << -1, 0.2;
	rot << cos(-M_PI/13),sin(-M_PI/13);
	edge_trans.so2().setComplex(rot);
	tunnelPoseVertexList[2]->setPose(-1, edge_trans);
	rot << cos(0),sin(0);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << 1, 0;
	tunnelPoseVertexList[2]->setPose(1, edge_trans);

	/*rot << cos(M_PI/4),sin(M_PI/4);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << 1, 0;
	robotPoseEdgeList.push_back(addRobotEdge(edge_trans, Eigen::Matrix3d::Identity(), robotPoseVertexList[0], robotPoseVertexList[1]));*/

	//Robot <-> Robot edges
	rot << cos(0),sin(0);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << 0.5, 0;
	robotPoseEdgeList.push_back(addRobotEdge(edge_trans, Eigen::Matrix3d::Identity(), robotPoseVertexList[0], robotPoseVertexList[1]));

	rot << cos(0),sin(0);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << 0.5, 0;
	robotPoseEdgeList.push_back(addRobotEdge(edge_trans, Eigen::Matrix3d::Identity(), robotPoseVertexList[1], robotPoseVertexList[2]));

	//Robot <-> Tunnel edges
	rot << cos(0),sin(0);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << 0, -1;
	robotPoseEdgeList.push_back(addRobotEdge(edge_trans, Eigen::Matrix3d::Identity(), robotPoseVertexList[0], tunnelPoseVertexList[0]));

	rot << cos(M_PI/8),sin(M_PI/8);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << 0, -1;
	robotPoseEdgeList.push_back(addRobotEdge(edge_trans, Eigen::Matrix3d::Identity(), robotPoseVertexList[1], tunnelPoseVertexList[1]));

	rot << cos(M_PI/8),sin(M_PI/8);
	edge_trans.so2().setComplex(rot);
	edge_trans.translation() << 0, -1;
	robotPoseEdgeList.push_back(addRobotEdge(edge_trans, Eigen::Matrix3d::Identity(), robotPoseVertexList[2], tunnelPoseVertexList[2]));

	//Constrant between tunnel segments
	tunnelEdgeList.push_back(addTunnelEdge(tunnelPoseVertexList[0], tunnelPoseVertexList[1]));
	tunnelEdgeList.push_back(addTunnelEdge(tunnelPoseVertexList[1], tunnelPoseVertexList[0]));

	tunnelEdgeList.push_back(addTunnelEdge(tunnelPoseVertexList[1], tunnelPoseVertexList[2]));
	tunnelEdgeList.push_back(addTunnelEdge(tunnelPoseVertexList[2], tunnelPoseVertexList[1]));
}

void drawGraph()
{
	sf::RenderWindow window(sf::VideoMode(800, 800), "Pose Graph Render");

	std::vector<sf::ConvexShape*> shapes;
	std::vector<sf::Vertex*> lines;

    for (VertexSE2 *vertex : robotPoseVertexList) {
		Eigen::Vector2d location = vertex->estimate().translation()*100;
		float angle = atan2(vertex->estimate().rotationMatrix()(0,1), vertex->estimate().rotationMatrix()(0,0));

		sf::ConvexShape *convex = new sf::ConvexShape();
		convex->setPointCount(3);

		convex->setPoint(0, sf::Vector2f(0, 3));
		convex->setPoint(1, sf::Vector2f(0, -3));
		convex->setPoint(2, sf::Vector2f(20, 0));

		convex->setPosition(location[0]+200, location[1]+200);
		convex->rotate(angle*180/M_PI);
		convex->setFillColor(sf::Color(250, 0, 0));

		shapes.push_back(convex);
	}

	for (TunnelOrient *vertex : tunnelPoseVertexList) {
		Eigen::Vector2d center_pose_loc = vertex->getPose(0).translation()*100;
		for (int i=-NUM_TUNNEL_SEGMENTS/2; i<=NUM_TUNNEL_SEGMENTS/2; i++) {
			Sophus::SE2d pose = vertex->getPose(i);

			Eigen::Vector2d location = pose.translation()*100;
			float angle = -atan2(pose.rotationMatrix()(0,1), pose.rotationMatrix()(0,0));

			sf::ConvexShape *convex = new sf::ConvexShape();
			convex->setPointCount(3);

			// define the points
			convex->setPoint(0, sf::Vector2f(0, 3));
			convex->setPoint(1, sf::Vector2f(0, -3));
			convex->setPoint(2, sf::Vector2f(20, 0));

			convex->setPosition(location[0]+200, location[1]+200);
			convex->rotate(angle*180/M_PI);
			if (i==0)
				convex->setFillColor(sf::Color(0, 250, 0));
			else
				convex->setFillColor(sf::Color(0, 100, 0));

			//std::cout << convex->getPosition() << "\n";
			shapes.push_back(convex);


			//Draw lines between segments
			//Have to malloc so we don't go out of scope
			sf::Vertex *line = (sf::Vertex *)malloc(2*sizeof(sf::Vertex));

			line[0] = sf::Vertex(sf::Vector2f(center_pose_loc[0] + 200, center_pose_loc[1]+200));
			line[1] = sf::Vertex(sf::Vector2f(location[0] + 200, location[1]+200));

			lines.push_back(line);
		}
	}
 
	for (EdgeSE2 *edge : robotPoseEdgeList) {
		Eigen::Vector2d from = edge->from()->estimate().translation()*100;
		Eigen::Vector2d to = edge->to()->estimate().translation()*100;
		
		//Have to malloc so we don't go out of scope
		sf::Vertex *line = (sf::Vertex *)malloc(2*sizeof(sf::Vertex));

		line[0] = sf::Vertex(sf::Vector2f(from[0] + 200, from[1]+200));
		line[1] = sf::Vertex(sf::Vector2f(to[0] + 200, to[1]+200));

		lines.push_back(line);
	}

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
		for (auto shape : shapes)
			window.draw(*shape);
		for (auto line : lines)
			window.draw(line, 2, sf::Lines);

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

TunnelAlignEdge* addTunnelEdge(TunnelOrient *t0, TunnelOrient *t1)
{
	TunnelAlignEdge* edge = new TunnelAlignEdge();
	edge->setId(numTunnelEdges++);
	edge->setInformation(Eigen::Matrix2d::Identity());
	//edge->setRobustKernel(constraint->robustKernel);

	edge->resize(2);

	edge->setVertex(0, t0);
	edge->setVertex(1, t1);
	graph.addEdge(edge);

	return edge;
}