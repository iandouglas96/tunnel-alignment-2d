#include "types.h"

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

G2O_USE_TYPE_GROUP(sba);

G2O_REGISTER_TYPE_GROUP(se2sophus);

G2O_REGISTER_TYPE(VERTEX_SE2_SOPHUS:EXPMAP, VertexSE2);
G2O_REGISTER_TYPE(TUNNEL_ORIENT:EXPMAP, TunnelOrient);
G2O_REGISTER_TYPE(EDGE_SE2_SOPHUS:EXPMAP, EdgeSE2);
G2O_REGISTER_TYPE(TUNNEL_ALIGN_EDGE:EXPMAP, TunnelAlignEdge);

TunnelOrient::TunnelOrient() : VertexSE2()
{
	for (int i=0; i<NUM_TUNNEL_SEGMENTS-1; i++)
		crossSectionPoses.push_back(Sophus::SE2d());
}

VertexSE2::VertexSE2() : g2o::BaseVertex<3, Sophus::SE2d>()
{
	_marginalized=false;
	_fix_scale = false;
}

//have to define write and read to make linker happy.  Just empty ones are fine.
bool VertexSE2::write(std::ostream& os) const
{
	// TODO
	assert(false);
	return false;
}

bool VertexSE2::read(std::istream& is)
{
	// TODO
	assert(false);
	return false;
}

EdgeSE2::EdgeSE2() :
	g2o::BaseBinaryEdge<3, Sophus::SE2d, VertexSE2, VertexSE2>()
{
}

bool EdgeSE2::write(std::ostream& os) const
{
	// TODO
	assert(false);
	return false;
}

bool EdgeSE2::read(std::istream& is)
{
	// TODO
	assert(false);
	return false;
}

TunnelAlignEdge::TunnelAlignEdge() :
	g2o::BaseBinaryEdge<2, Eigen::Vector2d, TunnelOrient, TunnelOrient>()
{
}

bool TunnelAlignEdge::write(std::ostream& os) const
{
	// TODO
	assert(false);
	return false;
} 

bool TunnelAlignEdge::read(std::istream& is)
{
	// TODO
	assert(false);
	return false;
}

double TunnelAlignEdge::getError(Eigen::Vector2d& pos, Eigen::Vector4d spline)
{
	//Perform gradient descent
	//Step 1: init with a guess at x
	double min_x = pos(0);
	double new_min_x = min_x;
	double dist_sqr = 100000;
	double new_dist_sqr = dist_sqr;
	double Dprime = 0;
	double step_size = 1;

	//Limit 500 iterations
	for (int i=0; i<500; i++) {
		//Step 2: calculate derivative distance to spline eval at min_x
		Dprime = (spline(0) + spline(1)*pos(0) + spline(2)*pos(0)*pos(0) + spline(3)*pow(pos(0),3) - pos(1));
		Dprime *= (spline(1) + spline(2)*pos(0) + spline(3)*pos(0)*pos(0));
		Dprime -= (min_x-pos(0));

		//Step 3: descend
		new_min_x = min_x + Dprime*step_size;
		//Step 4: calculate distance
		new_dist_sqr = pow(new_min_x - pos(0), 2) + pow(spline(0) + spline(1)*pos(0) + spline(2)*pos(0)*pos(0) + spline(3)*pow(pos(0), 3) - pos(1), 2);
		//Step 5: adjust step size if necessary
		if (new_dist_sqr > dist_sqr)
			step_size *= 0.5;
		else if (abs(Dprime) < 0.00001) {
			std::cout << "iterations: " << i << "\n";
			return sqrt(new_dist_sqr);
		}

		min_x = new_min_x;
		dist_sqr = new_dist_sqr;
	}
	return 0;
}