#include "types.h"

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

G2O_USE_TYPE_GROUP(sba);

G2O_REGISTER_TYPE_GROUP(se2sophus);

G2O_REGISTER_TYPE(VERTEX_SE2_SOPHUS:EXPMAP, VertexSE2);
G2O_REGISTER_TYPE(TUNNEL_ORIENT:EXPMAP, TunnelOrient);
G2O_REGISTER_TYPE(EDGE_SE2_SOPHUS:EXPMAP, EdgeSE2);

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