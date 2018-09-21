#pragma once

#include <sophus/se2.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/estimate_propagator.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

#define NUM_TUNNEL_SEGMENTS 3

//////////////////////////////////////////
//Class Definitions for g2o
//////////////////////////////////////////

class VertexSE2 : public g2o::BaseVertex<3, Sophus::SE2d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	VertexSE2();
	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;

	virtual void setToOriginImpl() {
		_estimate = Sophus::SE2d();
	}

	virtual void oplusImpl(const double* update_)
	{
		Eigen::Map< Eigen::Matrix<double, 3, 1> > update(const_cast<double*>(update_));

		setEstimate(Sophus::SE2d::exp(update) * estimate());
	}

	bool _fix_scale;
};

class TunnelOrient : public VertexSE2
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	TunnelOrient();

	void setPose(int num, Sophus::SE2d &pose)
	{
		if (num == 0)
			setEstimate(pose);
		else if (num < 0)
			crossSectionPoses[-2*num-1] = pose;
		else
			crossSectionPoses[2*(num-1)] = pose;

		calculateSpline();
	}

	Sophus::SE2d getPose(int num) {
		if (num == 0)
			return _estimate;
		else if (num < 0)
			return _estimate*crossSectionPoses[-2*num-1];

		return _estimate*crossSectionPoses[2*(num-1)];
	}

	void calculateSpline() {
		for (int i=-1; i<1; i++) {
			Eigen::Vector2d p1 = getPose(i).translation();
			Eigen::Vector2d p2 = getPose(i+1).translation();

			float slope1 = -getPose(i).rotationMatrix()(0,1)/getPose(i).rotationMatrix()(0,0);
			if (isnan(slope1))
				slope1 = 10000;
			float slope2 = -getPose(i+1).rotationMatrix()(0,1)/getPose(i+1).rotationMatrix()(0,0);
			if (isnan(slope2))
				slope2 = 10000;

			Eigen::Matrix4d A;
			A <<	1,	p1(0),	(p1(0))*(p1(0)),	pow(p1(0),3),
					1,	p2(0),	(p2(0))*(p2(0)),	pow(p2(0),3),
					0,	1,		2*(p1(0)),		3*(p1(0))*(p1(0)),
					0,	1,		2*(p2(0)),		3*(p2(0))*(p2(0));

			Eigen::Vector4d y;
			y << p1(1), p2(1), slope1, slope2;

			//Solve for coefficients for polynomial
			splines.block<1, 4>(i+1, 0) = A.inverse()*y;
		}
		std::cout << "Splines:\n" << splines << "\n";
	}

	Eigen::Vector4d getSpline(int i)
	{
		return splines.block<1, 4>(i+1, 0);
	}

private:
	std::vector<Sophus::SE2d> crossSectionPoses;
	Eigen::Matrix<double, 2, 4> splines; //Keep track of spline constants
};

/**
* \brief 3D edge between two Vertex3
*/
class EdgeSE2 : public g2o::BaseBinaryEdge<3, Sophus::SE2d, VertexSE2, VertexSE2>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeSE2();
	
	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;
	
	void computeError()
	{
		const VertexSE2* _from = static_cast<const VertexSE2*>(_vertices[0]);
		const VertexSE2* _to = static_cast<const VertexSE2*>(_vertices[1]);

		//Compute difference between estimates, then effectively subtract the constraint
		Sophus::SE2d error_= _from->estimate().inverse() * _to->estimate() * _inverseMeasurement;
		_error = error_.log();

	}
	
	void linearizeOplus()
	{
		const VertexSE2* _from = static_cast<const VertexSE2*>(_vertices[0]);

		_jacobianOplusXj = _from->estimate().inverse().Adj();
		_jacobianOplusXi = -_jacobianOplusXj;
	}


	virtual void setMeasurement(const Sophus::SE2d& m)
	{
		_measurement = m;
		_inverseMeasurement = m.inverse();
	}
	
	virtual bool setMeasurementData(const double* m)
	{
		Eigen::Map<const Sophus::Vector3d> v(m);
		setMeasurement(Sophus::SE2d::exp(v));
		return true;
	}
	
	virtual bool setMeasurementFromState()
	{
		const VertexSE2* from = static_cast<const VertexSE2*>(_vertices[0]);
		const VertexSE2* to   = static_cast<const VertexSE2*>(_vertices[1]);
		Sophus::SE2d delta = from->estimate().inverse() * to->estimate();
		setMeasurement(delta);
		return true;
	}

	virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}
	
	virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/)
	{
		VertexSE2 *_from = static_cast<VertexSE2*>(_vertices[0]);
		VertexSE2 *_to   = static_cast<VertexSE2*>(_vertices[1]);

		if (from.count(_from) > 0)
			_to->setEstimate(_from->estimate() * _measurement);
		else
			_from->setEstimate(_to->estimate() * _inverseMeasurement);
	}

	const VertexSE2 *from() 
	{
		VertexSE2 *_from = static_cast<VertexSE2*>(_vertices[0]);
		return _from;
	}

	const VertexSE2 *to() 
	{
		VertexSE2 *_to   = static_cast<VertexSE2*>(_vertices[1]);
		return _to;
	}
	
protected:
	Sophus::SE2d _inverseMeasurement;
};

/**
* \brief 3D edge between two Vertex3
*/
class TunnelAlignEdge : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, TunnelOrient, TunnelOrient>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	TunnelAlignEdge();
	
	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;
	 
	void computeError()
	{
		TunnelOrient* _from = static_cast<TunnelOrient*>(_vertices[0]);
		TunnelOrient* _to = static_cast<TunnelOrient*>(_vertices[1]);

		_to->calculateSpline();
		for (int i=-1; i<1; i++) {
			_error(i+1) = getError(_from->getPose(i).translation(), _to->getSpline(i));
		}

		std::cout << "error :\n" << _error << "\n";
	}

	double getError(Eigen::Vector2d& pos, Eigen::Vector4d spline);
	
protected:
	Sophus::SE2d _inverseMeasurement;
};