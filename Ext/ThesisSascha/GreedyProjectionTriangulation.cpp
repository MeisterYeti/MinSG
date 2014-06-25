/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */


#include "GreedyProjectionTriangulation.h"
#include "Profiler.h"

#include <Geometry/Box.h>
#include <Geometry/Plane.h>
#include <Geometry/VecHelper.h>

#include <Rendering/Mesh/Mesh.h>
#include <Rendering/Mesh/MeshIndexData.h>
#include <Rendering/Mesh/MeshVertexData.h>
#include <Rendering/Mesh/VertexDescription.h>
#include <Rendering/Mesh/VertexAttributeIds.h>
#include <Rendering/Mesh/VertexAttributeAccessors.h>

#include <Util/Macros.h>

#include <cmath>
#include <cstdlib>

namespace MinSG {

using namespace Util;
using namespace Rendering;

/** \brief Returns if a point X is visible from point R (or the origin)
* when taking into account the segment between the points S1 and S2
* \param X 2D coordinate of the point
* \param S1 2D coordinate of the segment's first point
* \param S2 2D coordinate of the segment's secont point
* \param R 2D coorddinate of the reference point (defaults to 0,0)
* \ingroup surface
*/
inline bool isVisible (const Geometry::Vec2 &X, const Geometry::Vec2 &S1, const Geometry::Vec2 &S2, const Geometry::Vec2 &R = Geometry::Vec2()) {
	PROFILE_SCOPE("isVisible");
	double a0 = S1.y() - S2.y();
	double b0 = S2.x() - S1.x();
	double c0 = S1.x()*S2.y() - S2.x()*S1.y();
	double a1 = -X.y();
	double b1 = X.x();
	double c1 = 0;
	if (!R.isZero()) {
		a1 += R.y();
		b1 -= R.x();
		c1 = R.x()*X.y() - X.x()*R.y();
	}
	double div = a0*b1 - b0*a1;
	double x = (b0*c1 - b1*c0) / div;
	double y = (a1*c0 - a0*c1) / div;

	bool intersection_outside_XR;
	if (R.isZero())	{
		if (X.x() > 0)
			intersection_outside_XR = (x <= 0) || (x >= X.x());
		else if (X.x() < 0)
			intersection_outside_XR = (x >= 0) || (x <= X.x());
		else if (X.y() > 0)
			intersection_outside_XR = (y <= 0) || (y >= X.y());
		else if (X.y() < 0)
			intersection_outside_XR = (y >= 0) || (y <= X.y());
		else
			intersection_outside_XR = true;
	} else {
		if (X.x() > R.x())
			intersection_outside_XR = (x <= R.x()) || (x >= X.x());
		else if (X.x() < R.x())
			intersection_outside_XR = (x >= R.x()) || (x <= X.x());
		else if (X.y() > R.y())
			intersection_outside_XR = (y <= R.y()) || (y >= X.y());
		else if (X.y() < R.y())
			intersection_outside_XR = (y >= R.y()) || (y <= X.y());
		else
			intersection_outside_XR = true;
	}
	if (intersection_outside_XR) {
		return true;
	} else {
		if (S1.x() > S2.x())
			return (x <= S2.x()) || (x >= S1.x());
		else if (S1.x() < S2.x())
			return (x >= S2.x()) || (x <= S1.x());
		else if (S1.y() > S2.y())
			return (y <= S2.y()) || (y >= S1.y());
		else if (S1.y() < S2.y())
			return (y >= S2.y()) || (y <= S1.y());
		else
			return false;
	}
}

GreedyProjectionTriangulation::GreedyProjectionTriangulation() :
                mu_ (0),
                search_radius_ (0), // must be set by user
                nnn_ (100),
                minimum_angle_ (M_PI/18), // 10 degrees
                maximum_angle_ (2*M_PI/3), // 120 degrees
                eps_angle_(M_PI/4), //45 degrees,
                consistent_(false),
                consistent_ordering_ (false),
                indices_ (),
#ifdef USE_TREE_OCTREE
                tree_ (Geometry::Box(), 0, 0),
#else
                tree_ (),
#endif
                triangle_ (),
                coords_ (),
                normals_ (),
                angles_ (),
                R_ (),
                state_ (),
                source_ (),
                ffn_ (),
                sfn_ (),
                part_ (),
                fringe_queue_ (),
                is_current_free_ (false),
                current_index_ (),
                prev_is_ffn_ (false),
                prev_is_sfn_ (false),
                next_is_ffn_ (false),
                next_is_sfn_ (false),
                changed_1st_fn_ (false),
                changed_2nd_fn_ (false),
                new2boundary_ (),
                already_connected_ (false),
                proj_qp_ (),
                u_ (),
                v_ (),
                uvn_ffn_ (),
                uvn_sfn_ (),
                uvn_next_ffn_ (),
                uvn_next_sfn_ (),
                tmp_ ()
              {}

bool GreedyProjectionTriangulation::reconstruct(Rendering::Mesh* source) {
	PROFILING_INIT();

	MeshVertexData& vData = source->openVertexData();
	VertexDescription vd = source->getVertexDescription();

	if(!vd.hasAttribute(VertexAttributeIds::POSITION))
		return false;

	Reference<PositionAttributeAccessor> posAcc = PositionAttributeAccessor::create(vData, VertexAttributeIds::POSITION);

	// TODO: estimate normal when not present
	if(!vd.hasAttribute(VertexAttributeIds::NORMAL))
		return false;
	Reference<NormalAttributeAccessor> normalAcc = NormalAttributeAccessor::create(vData, VertexAttributeIds::NORMAL);

	uint32_t index_count = source->getVertexCount();
	indices_.clear();
	indices_.reserve(index_count*2*3); /// NOTE: usually the number of triangles is around twice the number of vertices

	// initialize point octree
	{
		PROFILE_SCOPE("Init Search Tree");
		tree_.clear();
		Geometry::Box bb = source->getBoundingBox();
#ifdef USE_TREE_OCTREE
		tree_ = Geometry::PointOctree<TreeEntry>(bb, bb.getExtentMax()*0.01, 8);
#endif
		for (uint32_t i=0; i<index_count; ++i) {
			tree_.insert(TreeEntry(i, posAcc->getPosition(i)));
		}
	}

	// Saving coordinates and point to index mapping
	coords_.clear ();
	coords_.reserve (index_count);
	normals_.clear();
	normals_.reserve(index_count);
	for (uint32_t cp = 0; cp < index_count; ++cp) {
		coords_.push_back(posAcc->getPosition(cp));
		normals_.push_back(normalAcc->getNormal(cp));
	}

	if(!reconstructMesh(index_count))
		return false;

	// TODO: make user choose between new mesh and inplace surface generation
	source->setDrawMode(Mesh::DRAW_TRIANGLES);
	source->setUseIndexData(true);
	MeshIndexData& iData = source->openIndexData();
	iData.releaseLocalData();
	iData.allocate(indices_.size());
	uint32_t j = 0;
	for(uint32_t index : indices_)
		iData[j++] = index;

	PROFILING_PRINT();
	return true;
}

bool GreedyProjectionTriangulation::reconstructMesh(uint32_t index_count) {
	PROFILE_SCOPE("reconstruct");
	if(search_radius_ <= 0 || mu_ <= 0) {
		return false;
	}
	const double sqr_mu = mu_*mu_;
	const double sqr_max_edge = search_radius_*search_radius_;

	if(nnn_ > index_count)
		nnn_ = index_count;


	// Variables to hold the results of nearest neighbor searches
	std::vector<int> nnIdx (nnn_);
	std::vector<float> sqrDists (nnn_);

	// current number of connected components
	uint32_t part_index = 0;

	// 2D coordinates of points
	const Geometry::Vec2 uvn_nn_qp_zero;
	Geometry::Vec2 uvn_current;
	Geometry::Vec2 uvn_prev;
	Geometry::Vec2 uvn_next;

	// initializing fields
	already_connected_ = false; // see declaration for comments :P

	// initializing states and fringe neighbors
	part_.clear ();
	state_.clear ();
	source_.clear ();
	ffn_.clear ();
	sfn_.clear ();
	part_.resize(index_count, -1); // indices of point's part
	state_.resize(index_count, FREE);
	source_.resize(index_count, NONE);
	ffn_.resize(index_count, NONE);
	sfn_.resize(index_count, NONE);
	fringe_queue_.clear ();
	uint32_t fqIdx = 0; // current fringe's index in the queue to be processed

	// Avoiding NaN coordinates if needed
	//if (!input_->is_dense)
	{
		Geometry::Vec3 point;
		// Skip invalid points from the indices list
		for (uint32_t i=0; i<index_count; ++i) {
			point = coords_[i];
			if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z()))
				state_[i] = NONE;
		}
	}

	// Initializing
	int is_free=0, nr_parts=0, increase_nnn4fn=0, increase_nnn4s=0, increase_dist=0, nr_touched = 0;
	bool is_fringe;
	angles_.resize(nnn_);
	std::vector<Geometry::Vec2> uvn_nn (nnn_);
	Geometry::Vec2 uvn_s;
	Geometry::Plane projPlane;
	auto toPlaneLocal2DCoords = [this](const Geometry::Vec3& p) {
		static Geometry::Vec2 tmp2;
		tmp_ = p - proj_qp_;
		tmp2.setValue(tmp_.dot(u_),tmp_.dot(v_));
		return tmp2;
	};

	// iterating through fringe points and finishing them until everything is done
	while(is_free != NONE) {
		PROFILE_SCOPE("iterating through fringe points");
		R_ = is_free;
		if(state_[R_] == FREE) {
			state_[R_] = NONE;
			part_[R_] = part_index++;

		    // creating starting triangle
			nearestKSearch(coords_[R_], nnn_, nnIdx, sqrDists);
			double sqr_dist_threshold = std::min(sqr_max_edge, sqr_mu * sqrDists[1]);

			// Get the normal estimate at the current point
			const Geometry::Vec3 nc = normals_[R_];

		    // Get a coordinate system that lies on a plane defined by its normal
			v_ = Geometry::Helper::createOrthogonal(nc).normalize();
			u_ = nc.cross(v_);

		    // Projecting point onto the surface
			projPlane = Geometry::Plane(nc,0);
			proj_qp_ = projPlane.getProjection(coords_[R_]);

			// Converting coords, calculating angles and saving the projected near boundary edges
			PROFILE_SCOPE("Converting coords, calculating angles");
			uint32_t nr_edge = 0;
		    std::vector<doubleEdge> doubleEdges;
		    for(uint32_t i=1; i < nnn_; ++i) { // nearest neighbor with index 0 is the query point R_ itself
		    	// Transforming coordinates
		    	uvn_nn[i] = toPlaneLocal2DCoords(coords_[nnIdx[i]]);
		    	// Computing the angle between each neighboring point and the query point itself
		    	angles_[i].angle = std::atan2(uvn_nn[i].y(), uvn_nn[i].x());
		    	// initializing angle descriptors
		    	angles_[i].index = nnIdx[i];
	    		angles_[i].visible = (state_[nnIdx[i]] != COMPLETED) && (state_[nnIdx[i]] != BOUNDARY)
	    				&& (state_[nnIdx[i]] != NONE) && (sqrDists[i] <= sqr_dist_threshold);
	            // Saving the edges between nearby boundary points
	    		if(state_[nnIdx[i]] == FRINGE || state_[nnIdx[i]] == BOUNDARY) {
	    			++nr_edge;
			    	doubleEdges.push_back(doubleEdge{ i,
	    				toPlaneLocal2DCoords(coords_[ffn_[nnIdx[i]]]),
	    				toPlaneLocal2DCoords(coords_[sfn_[nnIdx[i]]])
	    			});
	    		}
		    }
		    angles_[0].visible = false;

		    // Verify the visibility of each potential new vertex
		    {
		    	PROFILE_SCOPE("Verify Visibility");
				for(uint32_t i=1; i < nnn_; ++i) { // nearest neighbor with index 0 is the query point R_ itself
					if(angles_[i].visible && ffn_[R_] != nnIdx[i] && sfn_[R_] != nnIdx[i]) {
						bool visibility = true;
						for(uint32_t j=0; j < nr_edge; ++j) {
							if( ffn_[nnIdx[doubleEdges[j].index]] != nnIdx[i] )
								visibility = isVisible(uvn_nn[i], uvn_nn[doubleEdges[j].index], doubleEdges[j].first);
							if(!visibility)
								break;
							if( sfn_[nnIdx[doubleEdges[j].index]] != nnIdx[i] )
								visibility = isVisible(uvn_nn[i], uvn_nn[doubleEdges[j].index], doubleEdges[j].second);
							if(visibility)
								break;
						}
						angles_[i].visible = visibility;
					}
				}
		    }

		    // Selecting first two visible free neighbors
			bool not_found = true;
			uint32_t left = 1;
			do {
				while(left < nnn_ && (!angles_[left].visible || state_[nnIdx[left]] > FREE)) ++left;
				if(left >= nnn_) {
					break;
				} else {
					uint32_t right = left+1;
					do {
						while(right < nnn_ && (!angles_[right].visible || state_[nnIdx[right]] > FREE)) ++right;
						if(right >= nnn_) {
							break;
						} else if( (coords_[nnIdx[left]] - coords_[nnIdx[right]]).lengthSquared() > sqr_max_edge ){
							++right;
						} else {
							addFringePoint(nnIdx[right], R_);
							addFringePoint(nnIdx[left], nnIdx[right]);
							addFringePoint(R_, nnIdx[left]);
							state_[R_] = state_[nnIdx[left]] = state_[nnIdx[right]] = FRINGE;
							ffn_[R_] = nnIdx[left];
							sfn_[R_] = nnIdx[right];
							ffn_[nnIdx[left]] = nnIdx[right];
							sfn_[nnIdx[left]] = R_;
							ffn_[nnIdx[right]] = R_;
							sfn_[nnIdx[right]] = nnIdx[left];
							addTriangle (R_, nnIdx[left], nnIdx[right]);
							++nr_parts;
							not_found = false;
							break;
						}
					} while(true);
					++left;
				}
			} while(not_found);
		}

		is_free = NONE;
		for (unsigned temp = 0; temp < index_count; ++temp) {
			if (state_[temp] == FREE) {
				is_free = temp;
				break;
			}
		}

		is_fringe = true;
		while(is_fringe) {
			is_fringe = false;

			uint32_t fqSize = fringe_queue_.size();
			while (fqIdx < fqSize && state_[fringe_queue_[fqIdx]] != FRINGE) ++fqIdx;

			// an unfinished fringe point is found
			if (fqIdx >= fqSize)
				continue;

			R_ = fringe_queue_[fqIdx];
			is_fringe = true;

			if (ffn_[R_] == sfn_[R_]) {
				state_[R_] = COMPLETED;
				continue;
			}

			// search for nearest neighbors
			nearestKSearch(coords_[R_], nnn_, nnIdx, sqrDists);

			// Locating FFN and SFN to adapt distance threshold
			double sqr_source_dist = (coords_[R_] - coords_[source_[R_]]).lengthSquared();
			double sqr_ffn_dist = (coords_[R_] - coords_[ffn_[R_]]).lengthSquared();
			double sqr_sfn_dist = (coords_[R_] - coords_[sfn_[R_]]).lengthSquared();
			double max_sqr_fn_dist = std::max(sqr_ffn_dist, sqr_sfn_dist);
			double sqr_dist_threshold = std::min(sqr_max_edge, sqr_mu * sqrDists[1]); //sqr_mu * sqr_avg_conn_dist);
			if (max_sqr_fn_dist > sqrDists[nnn_-1]) {
				if (0 == increase_nnn4fn)
					WARN("Not enough neighbors are considered: ffn or sfn out of range! Consider increasing nnn_... Setting R=" + std::to_string(R_) + " to be BOUNDARY!");
				++increase_nnn4fn;
				state_[R_] = BOUNDARY;
				continue;
			}
			double max_sqr_fns_dist = (std::max)(sqr_source_dist, max_sqr_fn_dist);
			if (max_sqr_fns_dist > sqrDists[nnn_-1]) {
				if (0 == increase_nnn4s)
					WARN("Not enough neighbors are considered: source of R=" + std::to_string(R_) + " is out of range! Consider increasing nnn_...\n");
				++increase_nnn4s;
			}

			// Get the normal estimate at the current point
			const Geometry::Vec3 nc = normals_[R_];

			// Get a coordinate system that lies on a plane defined by its normal
			v_ = Geometry::Helper::createOrthogonal(nc).normalize();
			u_ = nc.cross(v_);

			// Projecting point onto the surface
			projPlane = Geometry::Plane(nc,0);
			proj_qp_ = projPlane.getProjection(coords_[R_]);

			// Converting coords, calculating angles and saving the projected near boundary edges
			uint32_t nr_edge = 0;
			std::vector<doubleEdge> doubleEdges;
			{
				PROFILE_SCOPE("Converting coords, calculating angles");
				for(uint32_t i=1; i < nnn_; ++i) { // nearest neighbor with index 0 is the query point R_ itself
					// Transforming coordinates
					uvn_nn[i] = toPlaneLocal2DCoords(coords_[nnIdx[i]]);
					// Computing the angle between each neighboring point and the query point itself
					angles_[i].angle = std::atan2(uvn_nn[i].y(), uvn_nn[i].x());
					// initializing angle descriptors
					angles_[i].index = nnIdx[i];
					angles_[i].visible = (state_[nnIdx[i]] != COMPLETED) && (state_[nnIdx[i]] != BOUNDARY)
							&& (state_[nnIdx[i]] != NONE) && (sqrDists[i] <= sqr_dist_threshold);
					if (ffn_[R_] == nnIdx[i] || sfn_[R_] == nnIdx[i])
						angles_[i].visible = true;
					bool same_side = true;
					const Geometry::Vec3 neighbor_normal = normals_[nnIdx[i]]; /// NOTE: nnIdx was reset
					double cosine = nc.dot(neighbor_normal);
					if (cosine > 1) cosine = 1;
					if (cosine < -1) cosine = -1;
					double angle = std::acos(cosine);
					if ((!consistent_) && (angle > M_PI/2))
						angle = M_PI - angle;
					if (angle > eps_angle_) {
						angles_[i].visible = false;
						same_side = false;
					}
					// Saving the edges between nearby boundary points
					if(i!=0 && (state_[nnIdx[i]] == FRINGE || state_[nnIdx[i]] == BOUNDARY)) {
						++nr_edge;
						doubleEdge e{ i,
							toPlaneLocal2DCoords(coords_[ffn_[nnIdx[i]]]),
							toPlaneLocal2DCoords(coords_[sfn_[nnIdx[i]]])
						};
						doubleEdges.push_back(e);

						if(state_[nnIdx[i]] == FRINGE && ffn_[R_] != nnIdx[i] && sfn_[R_] != nnIdx[i]) {
							double angle1 = std::atan2(e.first.y() - uvn_nn[i].y(), e.first.x() - uvn_nn[i].x());
							double angle2 = std::atan2(e.second.y() - uvn_nn[i].y(), e.second.x() - uvn_nn[i].x());
							double angleMin = std::min(angle1, angle2);
							double angleMax = std::max(angle1, angle2);
							double angleR = angles_[i].angle + M_PI;
							if(angleR >= 2*M_PI)
								angleR -= 2*M_PI;
							if (source_[nnIdx[i]] == ffn_[nnIdx[i]] || source_[nnIdx[i]] == sfn_[nnIdx[i]]) {
								if ((angleMax - angleMin) < M_PI) {
									if ((angleMin < angleR) && (angleR < angleMax))
										angles_[i].visible = false;
								} else {
									if ((angleR < angleMin) || (angleMax < angleR))
										angles_[i].visible = false;
								}
							} else {
								uvn_s = toPlaneLocal2DCoords(coords_[source_[nnIdx[i]]]);
								double angleS = std::atan2(uvn_s[1] - uvn_nn[i][1], uvn_s[0] - uvn_nn[i][0]);
								if ((angleMin < angleS) && (angleS < angleMax)) {
									if ((angleMin < angleR) && (angleR < angleMax))
										angles_[i].visible = false;
								} else {
									if ((angleR < angleMin) || (angleMax < angleR))
										angles_[i].visible = false;
								}
							}
						}
					}
				}
				angles_[0].visible = false;
			}

			// Verify the visibility of each potential new vertex
			{
				PROFILE_SCOPE("Verify Visibility");
				for(uint32_t i=1; i < nnn_; ++i) { // nearest neighbor with index 0 is the query point R_ itself
					if(angles_[i].visible && ffn_[R_] != nnIdx[i] && sfn_[R_] != nnIdx[i]) {
						bool visibility = true;
						for(uint32_t j=0; j < nr_edge; ++j) {
							if(doubleEdges[j].index != i) {
								int f = ffn_[nnIdx[doubleEdges[j].index]];
								if( f != nnIdx[i] && f != R_)
									visibility = isVisible(uvn_nn[i], uvn_nn[doubleEdges[j].index], doubleEdges[j].first);
								if(!visibility)
									break;
								int s = sfn_[nnIdx[doubleEdges[j].index]];
								if( s != nnIdx[i] && f != R_)
									visibility = isVisible(uvn_nn[i], uvn_nn[doubleEdges[j].index], doubleEdges[j].second);
								if(!visibility)
									break;
							}
						}
						angles_[i].visible = visibility;
					}
				}
			}

			// Sorting angles
			std::sort(angles_.begin(), angles_.end(), GreedyProjectionTriangulation::nnAngleSortAsc);

			// Triangulating
			if (angles_[2].visible == false) {
				if ( !( (angles_[0].index == ffn_[R_] && angles_[1].index == sfn_[R_]) || (angles_[0].index == sfn_[R_] && angles_[1].index == ffn_[R_]) ) ) {
					state_[R_] = BOUNDARY;
				} else {
					if ((source_[R_] == angles_[0].index) || (source_[R_] == angles_[1].index)) {
						state_[R_] = BOUNDARY;
					} else {
						if (sqr_max_edge < (coords_[ffn_[R_]] - coords_[sfn_[R_]]).lengthSquared()) {
							state_[R_] = BOUNDARY;
						} else {
							uvn_s = toPlaneLocal2DCoords(coords_[source_[R_]]);

							double angleS = std::atan2(uvn_s[1], uvn_s[0]);
							double dif = angles_[1].angle - angles_[0].angle;
							if ((angles_[0].angle < angleS) && (angleS < angles_[1].angle)) {
								if (dif < 2*M_PI - maximum_angle_)
									state_[R_] = BOUNDARY;
								else
									closeTriangle();
							} else {
								if (dif >= maximum_angle_)
									state_[R_] = BOUNDARY;
								else
									closeTriangle();
							}
						}
					}
				}
				continue;
			}

			// Finding the FFN and SFN

			int start = -1, end = -1;
			{
				for (int i=0; i<nnn_; i++) {
					if (ffn_[R_] == angles_[i].index) {
						start = i;
						if (sfn_[R_] == angles_[i+1].index) {
							end = i+1;
						} else if (i==0) {
							for (i = i+2; i < nnn_; i++)
								if (sfn_[R_] == angles_[i].index)
									break;
							end = i;
						} else {
							for (i = i+2; i < nnn_; i++)
								if (sfn_[R_] == angles_[i].index)
									break;
							end = i;
						}
						break;
					}
					if (sfn_[R_] == angles_[i].index) {
						start = i;
						if (ffn_[R_] == angles_[i+1].index) {
							end = i+1;
						} else if (i==0) {
							for (i = i+2; i < nnn_; i++)
								if (ffn_[R_] == angles_[i].index)
									break;
							end = i;
						} else {
							for (i = i+2; i < nnn_; i++)
								if (ffn_[R_] == angles_[i].index)
									break;
							end = i;
						}
						break;
					}
				}
			}
			// start and end are always set, as we checked if ffn or sfn are out of range before, but let's check anyways if < 0
			if ((start < 0) || (end < 0) || (end == nnn_) || (!angles_[start].visible) || (!angles_[end].visible)) {
				state_[R_] = BOUNDARY;
				continue;
			}

			// Finding last visible nn
			int last_visible = end;
			while ((last_visible+1<nnn_) && (angles_[last_visible+1].visible)) ++last_visible;

			// Finding visibility region of R
			bool need_invert = false;
			int sourceIdx = nnn_;
			{
				if ((source_[R_] == ffn_[R_]) || (source_[R_] == sfn_[R_])) {
					if ((angles_[end].angle - angles_[start].angle) < M_PI)
						need_invert = true;
				} else {
					for (sourceIdx=0; sourceIdx<nnn_; sourceIdx++)
						if (angles_[sourceIdx].index == source_[R_])
							break;
					if (sourceIdx == nnn_) {
						int vis_free = NONE, nnCB = NONE; // any free visible and nearest completed or boundary neighbor of R
						for (int i = 1; i < nnn_; i++) { // nearest neighbor with index 0 is the query point R_ itself
							// NOTE: nnCB is an index in nnIdx
							if ((state_[nnIdx[i]] == COMPLETED) || (state_[nnIdx[i]] == BOUNDARY)) {
								if (nnCB == NONE) {
									nnCB = i;
									if (vis_free != NONE)
										break;
								}
							}
							// NOTE: vis_free is an index in angles
							if (state_[angles_[i].index] <= FREE) {
								if (i <= last_visible) {
									vis_free = i;
									if (nnCB != NONE)
										break;
								}
							}
						}
						// NOTE: nCB is an index in angles
						int nCB = 0;
						if (nnCB != NONE)
							while (angles_[nCB].index != nnIdx[nnCB]) nCB++;
						else
							nCB = NONE;

						if (vis_free != NONE) {
							if ((vis_free < start) || (vis_free > end))
								need_invert = true;
						} else {
							if (nCB != NONE) {
								if ((nCB == start) || (nCB == end)) {
									bool inside_CB = false;
									bool outside_CB = false;
									for (int i=0; i<nnn_; i++) {
										if ( (state_[angles_[i].index] == COMPLETED || state_[angles_[i].index] == BOUNDARY) && i != start && i != end ) {
											if ((angles_[start].angle <= angles_[i].angle) && (angles_[i].angle <= angles_[end].angle)) {
												inside_CB = true;
												if (outside_CB)
													break;
											} else {
												outside_CB = true;
												if (inside_CB)
													break;
											}
										}
									}
									if (inside_CB && !outside_CB)
										need_invert = true;
									else if (!(!inside_CB && outside_CB)) {
										if ((angles_[end].angle - angles_[start].angle) < M_PI)
											need_invert = true;
									}
								} else {
									if ((angles_[nCB].angle > angles_[start].angle) && (angles_[nCB].angle < angles_[end].angle))
										need_invert = true;
								}
							} else {
								if (start == end-1)
									need_invert = true;
							}
						}
					} else if ((angles_[start].angle < angles_[sourceIdx].angle) && (angles_[sourceIdx].angle < angles_[end].angle))
						need_invert = true;
				}

			}
			// switching start and end if necessary
			if (need_invert) {
				int tmp = start;
				start = end;
				end = tmp;
			}

			// Arranging visible nnAngles in the order they need to be connected and
			// compute the maximal angle difference between two consecutive visible angles
			bool is_boundary = false, is_skinny = false;
			std::vector<bool> gaps (nnn_, false);
			std::vector<bool> skinny (nnn_, false);
			std::vector<double> dif (nnn_);
			std::vector<int> angleIdx; angleIdx.reserve (nnn_);

			if (start > end) {
				for (int j=start; j<last_visible; j++) {
					dif[j] = (angles_[j+1].angle - angles_[j].angle);
					if (dif[j] < minimum_angle_) {
						skinny[j] = is_skinny = true;
					} else if (maximum_angle_ <= dif[j]) {
						gaps[j] = is_boundary = true;
					}
					if ((!gaps[j]) && (sqr_max_edge < (coords_[angles_[j+1].index] - coords_[angles_[j].index]).lengthSquared())) {
						gaps[j] = is_boundary = true;
					}
					angleIdx.push_back(j);
				}

				dif[last_visible] = (2*M_PI + angles_[0].angle - angles_[last_visible].angle);
				if (dif[last_visible] < minimum_angle_) {
					skinny[last_visible] = is_skinny = true;
				} else if (maximum_angle_ <= dif[last_visible]) {
					gaps[last_visible] = is_boundary = true;
				}
				if ((!gaps[last_visible]) && (sqr_max_edge < (coords_[angles_[0].index] - coords_[angles_[last_visible].index]).lengthSquared())) {
					gaps[last_visible] = is_boundary = true;
				}
				angleIdx.push_back(last_visible);

				for (int j=0; j<end; j++) {
					dif[j] = (angles_[j+1].angle - angles_[j].angle);
					if (dif[j] < minimum_angle_) {
						skinny[j] = is_skinny = true;
					} else if (maximum_angle_ <= dif[j]) {
						gaps[j] = is_boundary = true;
					}
					if ((!gaps[j]) && (sqr_max_edge < (coords_[angles_[j+1].index] - coords_[angles_[j].index]).lengthSquared())) {
						gaps[j] = is_boundary = true;
					}
					angleIdx.push_back(j);
				}
				angleIdx.push_back(end);
			} else { // start < end
				for (int j=start; j<end; j++) {
					dif[j] = (angles_[j+1].angle - angles_[j].angle);
					if (dif[j] < minimum_angle_) {
						skinny[j] = is_skinny = true;
					} else if (maximum_angle_ <= dif[j]) {
						gaps[j] = is_boundary = true;
					}
					if ((!gaps[j]) && (sqr_max_edge < (coords_[angles_[j+1].index] - coords_[angles_[j].index]).lengthSquared())) {
						gaps[j] = is_boundary = true;
					}
					angleIdx.push_back(j);
				}
				angleIdx.push_back(end);
			}

			// Set the state of the point
			state_[R_] = is_boundary ? BOUNDARY : COMPLETED;

			std::vector<int>::iterator first_gap_after = angleIdx.end ();
			std::vector<int>::iterator last_gap_before = angleIdx.begin ();
			int nr_gaps = 0;
			for (std::vector<int>::iterator it = angleIdx.begin (); it != angleIdx.end () - 1; it++) {
				if (gaps[*it]) {
					nr_gaps++;
					if (first_gap_after == angleIdx.end())
						first_gap_after = it;
					last_gap_before = it+1;
				}
			}
			if (nr_gaps > 1) {
				angleIdx.erase(first_gap_after+1, last_gap_before);
			}

			// Neglecting points that would form skinny triangles (if possible)
			if (is_skinny) {
				double angle_so_far = 0, angle_would_be;
				double max_combined_angle = (std::min)(maximum_angle_, M_PI - 2 * minimum_angle_);
				Geometry::Vec2 X;
				Geometry::Vec2 S1;
				Geometry::Vec2 S2;
				std::vector<int> to_erase;
				for (std::vector<int>::iterator it = angleIdx.begin() + 1; it != angleIdx.end() - 1; it++) {
					if (gaps[*(it - 1)])
						angle_so_far = 0;
					else
						angle_so_far += dif[*(it - 1)];
					if (gaps[*it])
						angle_would_be = angle_so_far;
					else
						angle_would_be = angle_so_far + dif[*it];
					if ((skinny[*it] || skinny[*(it - 1)])
							&& ((state_[angles_[*it].index] <= FREE) || (state_[angles_[*(it - 1)].index] <= FREE))
							&& ((!gaps[*it]) || (angles_[*it].nnIndex > angles_[*(it - 1)].nnIndex))
							&& ((!gaps[*(it - 1)]) || (angles_[*it].nnIndex > angles_[*(it + 1)].nnIndex))
							&& (angle_would_be < max_combined_angle)) {
						if (gaps[*(it - 1)]) {
							gaps[*it] = true;
							to_erase.push_back(*it);
						} else if (gaps[*it]) {
							gaps[*(it - 1)] = true;
							to_erase.push_back(*it);
						} else {
							std::vector<int>::iterator prev_it;
							int erased_idx = static_cast<int>(to_erase.size()) - 1;
							for (prev_it = it - 1; (erased_idx != -1) && (it != angleIdx.begin()); it--)
								if (*it == to_erase[erased_idx])
									erased_idx--;
								else
									break;
							bool can_delete = true;
							for (std::vector<int>::iterator curr_it = prev_it + 1; curr_it != it + 1; curr_it++) {
								X = toPlaneLocal2DCoords(coords_[angles_[*curr_it].index]);
								S1 = toPlaneLocal2DCoords(coords_[angles_[*prev_it].index]);
								S2 = toPlaneLocal2DCoords(coords_[angles_[*(it + 1)].index]);
								// check for inclusions
								if (isVisible(X, S1, S2)) {
									can_delete = false;
									angle_so_far = 0;
									break;
								}
							}
							if (can_delete) {
								to_erase.push_back(*it);
							}
						}
					} else
						angle_so_far = 0;
				}
				for (std::vector<int>::iterator it = to_erase.begin(); it != to_erase.end(); it++) {
					for (std::vector<int>::iterator iter = angleIdx.begin(); iter != angleIdx.end(); iter++) {
						if (*it == *iter) {
							angleIdx.erase(iter);
							break;
						}
					}
				}
			}

			// Writing edges and updating edge-front
			changed_1st_fn_ = false;
			changed_2nd_fn_ = false;
			new2boundary_ = NONE;
			for (std::vector<int>::iterator it = angleIdx.begin() + 1; it != angleIdx.end() - 1; it++) {
				current_index_ = angles_[*it].index;

				is_current_free_ = false;
				if (state_[current_index_] <= FREE) {
					state_[current_index_] = FRINGE;
					is_current_free_ = true;
				} else if (!already_connected_) {
					prev_is_ffn_ = (ffn_[current_index_] == angles_[*(it - 1)].index) && (!gaps[*(it - 1)]);
					prev_is_sfn_ = (sfn_[current_index_] == angles_[*(it - 1)].index) && (!gaps[*(it - 1)]);
					next_is_ffn_ = (ffn_[current_index_] == angles_[*(it + 1)].index) && (!gaps[*it]);
					next_is_sfn_ = (sfn_[current_index_] == angles_[*(it + 1)].index) && (!gaps[*it]);
					if (!prev_is_ffn_ && !next_is_sfn_ && !prev_is_sfn_ && !next_is_ffn_) {
						nr_touched++;
					}
				}

				if (gaps[*it]) {
					if (gaps[*(it - 1)]) {
						if (is_current_free_)
							state_[current_index_] = NONE; /// TODO: document!
					} else {// (gaps[*it]) && ^(gaps[*(it-1)])
						addTriangle(current_index_, angles_[*(it - 1)].index, R_);
						addFringePoint(current_index_, R_);
						new2boundary_ = current_index_;
						if (!already_connected_) {
							connectPoint(angles_[*(it - 1)].index, R_, angles_[*(it + 1)].index, uvn_nn[angles_[*it].nnIndex],
									uvn_nn[angles_[*(it - 1)].nnIndex], uvn_nn_qp_zero);
						} else {
							already_connected_ = false;
						}
						if (ffn_[R_] == angles_[*(angleIdx.begin())].index) {
							ffn_[R_] = new2boundary_;
						} else if (sfn_[R_] == angles_[*(angleIdx.begin())].index) {
							sfn_[R_] = new2boundary_;
						}
					}
				} else if (gaps[*(it - 1)]) {
					addFringePoint(current_index_, R_);
					new2boundary_ = current_index_;
					if (!already_connected_) {
						connectPoint(R_, angles_[*(it + 1)].index, (it + 2) == angleIdx.end() ? -1 : angles_[*(it + 2)].index, uvn_nn[angles_[*it].nnIndex],
								uvn_nn_qp_zero, uvn_nn[angles_[*(it + 1)].nnIndex]);
					} else {
						already_connected_ = false;
					}
					if (ffn_[R_] == angles_[*(angleIdx.end() - 1)].index) {
						ffn_[R_] = new2boundary_;
					} else if (sfn_[R_] == angles_[*(angleIdx.end() - 1)].index) {
						sfn_[R_] = new2boundary_;
					}

				} else { // ^(gaps[*it]) && ^(gaps[*(it-1)])
					addTriangle(current_index_, angles_[*(it - 1)].index, R_);
					addFringePoint(current_index_, R_);
					if (!already_connected_) {
						connectPoint(angles_[*(it - 1)].index, angles_[*(it + 1)].index,
								(it + 2) == angleIdx.end() ? -1 : gaps[*(it + 1)] ? R_ : angles_[*(it + 2)].index, uvn_nn[angles_[*it].nnIndex],
								uvn_nn[angles_[*(it - 1)].nnIndex], uvn_nn[angles_[*(it + 1)].nnIndex]);
					} else {
						already_connected_ = false;
					}
				}
			}

			// Finishing up R_
			if (ffn_[R_] == sfn_[R_]) {
				state_[R_] = COMPLETED;
			}
			if (!gaps[*(angleIdx.end() - 2)]) {
				addTriangle(angles_[*(angleIdx.end() - 2)].index, angles_[*(angleIdx.end() - 1)].index, R_);
				addFringePoint(angles_[*(angleIdx.end() - 2)].index, R_);
				if (R_ == ffn_[angles_[*(angleIdx.end() - 1)].index]) {
					if (angles_[*(angleIdx.end() - 2)].index == sfn_[angles_[*(angleIdx.end() - 1)].index]) {
						state_[angles_[*(angleIdx.end() - 1)].index] = COMPLETED;
					} else {
						ffn_[angles_[*(angleIdx.end() - 1)].index] = angles_[*(angleIdx.end() - 2)].index;
					}
				} else if (R_ == sfn_[angles_[*(angleIdx.end() - 1)].index]) {
					if (angles_[*(angleIdx.end() - 2)].index == ffn_[angles_[*(angleIdx.end() - 1)].index]) {
						state_[angles_[*(angleIdx.end() - 1)].index] = COMPLETED;
					} else {
						sfn_[angles_[*(angleIdx.end() - 1)].index] = angles_[*(angleIdx.end() - 2)].index;
					}
				}
			}
			if (!gaps[*(angleIdx.begin())]) {
				if (R_ == ffn_[angles_[*(angleIdx.begin())].index]) {
					if (angles_[*(angleIdx.begin() + 1)].index == sfn_[angles_[*(angleIdx.begin())].index]) {
						state_[angles_[*(angleIdx.begin())].index] = COMPLETED;
					} else {
						ffn_[angles_[*(angleIdx.begin())].index] = angles_[*(angleIdx.begin() + 1)].index;
					}
				} else if (R_ == sfn_[angles_[*(angleIdx.begin())].index]) {
					if (angles_[*(angleIdx.begin() + 1)].index == ffn_[angles_[*(angleIdx.begin())].index]) {
						state_[angles_[*(angleIdx.begin())].index] = COMPLETED;
					} else {
						sfn_[angles_[*(angleIdx.begin())].index] = angles_[*(angleIdx.begin() + 1)].index;
					}
				}
			}
		}
	}
	DEBUG ("Number of triangles: " + std::to_string(polygons.size()));
	DEBUG ("Number of unconnected parts: " + std::to_string(nr_parts));
	if (increase_nnn4fn > 0)
		WARN ("Number of neighborhood size increase requests for fringe neighbors: " + std::to_string(increase_nnn4fn));
	if (increase_nnn4s > 0)
		WARN ("Number of neighborhood size increase requests for source: " + std::to_string(increase_nnn4s));
	if (increase_dist > 0)
		WARN ("Number of automatic maximum distance increases: " + std::to_string(increase_dist));

	// sorting and removing doubles from fringe queue
	std::sort (fringe_queue_.begin (), fringe_queue_.end ());
	fringe_queue_.erase (std::unique (fringe_queue_.begin (), fringe_queue_.end ()), fringe_queue_.end ());
	DEBUG ("Number of processed points: " + std::to_string(fringe_queue_.size()) + " / " + std::to_string(index_count));
	return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void GreedyProjectionTriangulation::closeTriangle () {
	state_[R_] = COMPLETED;
	addTriangle (angles_[0].index, angles_[1].index, R_);
	for (int aIdx=0; aIdx<2; ++aIdx) {
		if (ffn_[angles_[aIdx].index] == R_) {
			if (sfn_[angles_[aIdx].index] == angles_[(aIdx+1)%2].index) {
				state_[angles_[aIdx].index] = COMPLETED;
			} else {
				ffn_[angles_[aIdx].index] = angles_[(aIdx+1)%2].index;
			}
		} else if (sfn_[angles_[aIdx].index] == R_) {
			if (ffn_[angles_[aIdx].index] == angles_[(aIdx+1)%2].index) {
				state_[angles_[aIdx].index] = COMPLETED;
			} else {
				sfn_[angles_[aIdx].index] = angles_[(aIdx+1)%2].index;
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////
void GreedyProjectionTriangulation::addTriangle (int a, int b, int c) {
	triangle_.resize (3);
	if (consistent_ordering_) {
		const Geometry::Vec3 pa = coords_[a];
		const Geometry::Vec3 na = normals_[a];
		const Geometry::Vec3 pb = coords_[b];
		const Geometry::Vec3 pc = coords_[c];

		if (na.dot((pa - pb).cross(pa - pc)) > 0) {
			triangle_[0] = a;
			triangle_[1] = b;
			triangle_[2] = c;
		} else {
			triangle_[0] = a;
			triangle_[1] = c;
			triangle_[2] = b;
		}
	} else {
		triangle_[0] = a;
		triangle_[1] = b;
		triangle_[2] = c;
	}
	indices_.push_back(triangle_[0]);
	indices_.push_back(triangle_[1]);
	indices_.push_back(triangle_[2]);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void GreedyProjectionTriangulation::connectPoint (const int prev_index, const int next_index, const int next_next_index,
                   const Geometry::Vec2 &uvn_current, const Geometry::Vec2 &uvn_prev,  const Geometry::Vec2 &uvn_next) {
	PROFILE_SCOPE("Connect Point");
	if (is_current_free_) {
		ffn_[current_index_] = prev_index;
		sfn_[current_index_] = next_index;
	} else {
		if ((prev_is_ffn_ && next_is_sfn_) || (prev_is_sfn_ && next_is_ffn_))
			state_[current_index_] = COMPLETED;
		else if (prev_is_ffn_ && !next_is_sfn_)
			ffn_[current_index_] = next_index;
		else if (next_is_ffn_ && !prev_is_sfn_)
			ffn_[current_index_] = prev_index;
		else if (prev_is_sfn_ && !next_is_ffn_)
			sfn_[current_index_] = next_index;
		else if (next_is_sfn_ && !prev_is_ffn_)
			sfn_[current_index_] = prev_index;
		else {
			bool found_triangle = false;
			if ((prev_index != R_) && ((ffn_[current_index_] == ffn_[prev_index]) || (ffn_[current_index_] == sfn_[prev_index]))) {
				found_triangle = true;
				addTriangle(current_index_, ffn_[current_index_], prev_index);
				state_[prev_index] = COMPLETED;
				state_[ffn_[current_index_]] = COMPLETED;
				ffn_[current_index_] = next_index;
			} else if ((prev_index != R_) && ((sfn_[current_index_] == ffn_[prev_index]) || (sfn_[current_index_] == sfn_[prev_index]))) {
				found_triangle = true;
				addTriangle(current_index_, sfn_[current_index_], prev_index);
				state_[prev_index] = COMPLETED;
				state_[sfn_[current_index_]] = COMPLETED;
				sfn_[current_index_] = next_index;
			} else if (state_[next_index] > FREE) {
				if ((ffn_[current_index_] == ffn_[next_index]) || (ffn_[current_index_] == sfn_[next_index])) {
					found_triangle = true;
					addTriangle(current_index_, ffn_[current_index_], next_index);

					if (ffn_[current_index_] == ffn_[next_index]) {
						ffn_[next_index] = current_index_;
					} else {
						sfn_[next_index] = current_index_;
					}
					state_[ffn_[current_index_]] = COMPLETED;
					ffn_[current_index_] = prev_index;
				} else if ((sfn_[current_index_] == ffn_[next_index]) || (sfn_[current_index_] == sfn_[next_index])) {
					found_triangle = true;
					addTriangle(current_index_, sfn_[current_index_], next_index);

					if (sfn_[current_index_] == ffn_[next_index]) {
						ffn_[next_index] = current_index_;
					} else {
						sfn_[next_index] = current_index_;
					}
					state_[sfn_[current_index_]] = COMPLETED;
					sfn_[current_index_] = prev_index;
				}
			}

			if (found_triangle) {
			} else {
				tmp_ = coords_[ffn_[current_index_]] - proj_qp_;
				uvn_ffn_[0] = tmp_.dot(u_);
				uvn_ffn_[1] = tmp_.dot(v_);
				tmp_ = coords_[sfn_[current_index_]] - proj_qp_;
				uvn_sfn_[0] = tmp_.dot(u_);
				uvn_sfn_[1] = tmp_.dot(v_);
				bool prev_ffn = isVisible(uvn_prev, uvn_next, uvn_current, uvn_ffn_) && isVisible(uvn_prev, uvn_sfn_, uvn_current, uvn_ffn_);
				bool prev_sfn = isVisible(uvn_prev, uvn_next, uvn_current, uvn_sfn_) && isVisible(uvn_prev, uvn_ffn_, uvn_current, uvn_sfn_);
				bool next_ffn = isVisible(uvn_next, uvn_prev, uvn_current, uvn_ffn_) && isVisible(uvn_next, uvn_sfn_, uvn_current, uvn_ffn_);
				bool next_sfn = isVisible(uvn_next, uvn_prev, uvn_current, uvn_sfn_) && isVisible(uvn_next, uvn_ffn_, uvn_current, uvn_sfn_);
				int min_dist = -1;
				if (prev_ffn && next_sfn && prev_sfn && next_ffn) {
					/* should be never the case */
					double prev2f = (coords_[ffn_[current_index_]] - coords_[prev_index]).lengthSquared();
					double next2s = (coords_[sfn_[current_index_]] - coords_[next_index]).lengthSquared();
					double prev2s = (coords_[sfn_[current_index_]] - coords_[prev_index]).lengthSquared();
					double next2f = (coords_[ffn_[current_index_]] - coords_[next_index]).lengthSquared();
					if (prev2f < prev2s) {
						if (prev2f < next2f) {
							if (prev2f < next2s)
								min_dist = 0;
							else
								min_dist = 3;
						} else {
							if (next2f < next2s)
								min_dist = 2;
							else
								min_dist = 3;
						}
					} else {
						if (prev2s < next2f) {
							if (prev2s < next2s)
								min_dist = 1;
							else
								min_dist = 3;
						} else {
							if (next2f < next2s)
								min_dist = 2;
							else
								min_dist = 3;
						}
					}
				} else if (prev_ffn && next_sfn) {
					/* a clear case */
					double prev2f = (coords_[ffn_[current_index_]] - coords_[prev_index]).lengthSquared();
					double next2s = (coords_[sfn_[current_index_]] - coords_[next_index]).lengthSquared();
					if (prev2f < next2s)
						min_dist = 0;
					else
						min_dist = 3;
				} else if (prev_sfn && next_ffn) {
					/* a clear case */
					double prev2s = (coords_[sfn_[current_index_]] - coords_[prev_index]).lengthSquared();
					double next2f = (coords_[ffn_[current_index_]] - coords_[next_index]).lengthSquared();
					if (prev2s < next2f)
						min_dist = 1;
					else
						min_dist = 2;
				}
				/* straightforward cases */
				else if (prev_ffn && !next_sfn && !prev_sfn && !next_ffn)
					min_dist = 0;
				else if (!prev_ffn && !next_sfn && prev_sfn && !next_ffn)
					min_dist = 1;
				else if (!prev_ffn && !next_sfn && !prev_sfn && next_ffn)
					min_dist = 2;
				else if (!prev_ffn && next_sfn && !prev_sfn && !next_ffn)
					min_dist = 3;
				/* messed up cases */
				else if (prev_ffn) {
					double prev2f = (coords_[ffn_[current_index_]] - coords_[prev_index]).lengthSquared();
					if (prev_sfn) {
						double prev2s = (coords_[sfn_[current_index_]] - coords_[prev_index]).lengthSquared();
						if (prev2s < prev2f)
							min_dist = 1;
						else
							min_dist = 0;
					} else if (next_ffn) {
						double next2f = (coords_[ffn_[current_index_]] - coords_[next_index]).lengthSquared();
						if (next2f < prev2f)
							min_dist = 2;
						else
							min_dist = 0;
					}
				} else if (next_sfn) {
					double next2s = (coords_[sfn_[current_index_]] - coords_[next_index]).lengthSquared();
					if (prev_sfn) {
						double prev2s = (coords_[sfn_[current_index_]] - coords_[prev_index]).lengthSquared();
						if (prev2s < next2s)
							min_dist = 1;
						else
							min_dist = 3;
					} else if (next_ffn) {
						double next2f = (coords_[ffn_[current_index_]] - coords_[next_index]).lengthSquared();
						if (next2f < next2s)
							min_dist = 2;
						else
							min_dist = 3;
					}
				}
				switch (min_dist) {
					case 0: //prev2f:
					{
						addTriangle(current_index_, ffn_[current_index_], prev_index);

						/* updating prev_index */
						if (ffn_[prev_index] == current_index_) {
							ffn_[prev_index] = ffn_[current_index_];
						} else if (sfn_[prev_index] == current_index_) {
							sfn_[prev_index] = ffn_[current_index_];
						} else if (ffn_[prev_index] == R_) {
							changed_1st_fn_ = true;
							ffn_[prev_index] = ffn_[current_index_];
						} else if (sfn_[prev_index] == R_) {
							changed_1st_fn_ = true;
							sfn_[prev_index] = ffn_[current_index_];
						} else if (prev_index == R_) {
							new2boundary_ = ffn_[current_index_];
						}

						/* updating ffn */
						if (ffn_[ffn_[current_index_]] == current_index_) {
							ffn_[ffn_[current_index_]] = prev_index;
						} else if (sfn_[ffn_[current_index_]] == current_index_) {
							sfn_[ffn_[current_index_]] = prev_index;
						}

						/* updating current */
						ffn_[current_index_] = next_index;

						break;
					}
					case 1: //prev2s:
					{
						addTriangle(current_index_, sfn_[current_index_], prev_index);

						/* updating prev_index */
						if (ffn_[prev_index] == current_index_) {
							ffn_[prev_index] = sfn_[current_index_];
						} else if (sfn_[prev_index] == current_index_) {
							sfn_[prev_index] = sfn_[current_index_];
						} else if (ffn_[prev_index] == R_) {
							changed_1st_fn_ = true;
							ffn_[prev_index] = sfn_[current_index_];
						} else if (sfn_[prev_index] == R_) {
							changed_1st_fn_ = true;
							sfn_[prev_index] = sfn_[current_index_];
						} else if (prev_index == R_) {
							new2boundary_ = sfn_[current_index_];
						}

						/* updating sfn */
						if (ffn_[sfn_[current_index_]] == current_index_) {
							ffn_[sfn_[current_index_]] = prev_index;
						} else if (sfn_[sfn_[current_index_]] == current_index_) {
							sfn_[sfn_[current_index_]] = prev_index;
						}

						/* updating current */
						sfn_[current_index_] = next_index;

						break;
					}
					case 2: //next2f:
					{
						addTriangle(current_index_, ffn_[current_index_], next_index);
						int neighbor_update = next_index;

						/* updating next_index */
						if (state_[next_index] <= FREE) {
							state_[next_index] = FRINGE;
							ffn_[next_index] = current_index_;
							sfn_[next_index] = ffn_[current_index_];
						} else {
							if (ffn_[next_index] == R_) {
								changed_2nd_fn_ = true;
								ffn_[next_index] = ffn_[current_index_];
							} else if (sfn_[next_index] == R_) {
								changed_2nd_fn_ = true;
								sfn_[next_index] = ffn_[current_index_];
							} else if (next_index == R_) {
								new2boundary_ = ffn_[current_index_];
								if (next_next_index == new2boundary_)
									already_connected_ = true;
							} else if (ffn_[next_index] == next_next_index) {
								already_connected_ = true;
								ffn_[next_index] = ffn_[current_index_];
							} else if (sfn_[next_index] == next_next_index) {
								already_connected_ = true;
								sfn_[next_index] = ffn_[current_index_];
							} else {
								tmp_ = coords_[ffn_[next_index]] - proj_qp_;
								uvn_next_ffn_[0] = tmp_.dot(u_);
								uvn_next_ffn_[1] = tmp_.dot(v_);
								tmp_ = coords_[sfn_[next_index]] - proj_qp_;
								uvn_next_sfn_[0] = tmp_.dot(u_);
								uvn_next_sfn_[1] = tmp_.dot(v_);

								bool ffn_next_ffn = isVisible(uvn_next_ffn_, uvn_next, uvn_current, uvn_ffn_)
										&& isVisible(uvn_next_ffn_, uvn_next, uvn_next_sfn_, uvn_ffn_);
								bool sfn_next_ffn = isVisible(uvn_next_sfn_, uvn_next, uvn_current, uvn_ffn_)
										&& isVisible(uvn_next_sfn_, uvn_next, uvn_next_ffn_, uvn_ffn_);

								int connect2ffn = -1;
								if (ffn_next_ffn && sfn_next_ffn) {
									double fn2f = (coords_[ffn_[current_index_]] - coords_[ffn_[next_index]]).lengthSquared();
									double sn2f = (coords_[ffn_[current_index_]] - coords_[sfn_[next_index]]).lengthSquared();
									if (fn2f < sn2f)
										connect2ffn = 0;
									else
										connect2ffn = 1;
								} else if (ffn_next_ffn)
									connect2ffn = 0;
								else if (sfn_next_ffn)
									connect2ffn = 1;

								switch (connect2ffn) {
									case 0: // ffn[next]
									{
										addTriangle(next_index, ffn_[current_index_], ffn_[next_index]);
										neighbor_update = ffn_[next_index];

										/* ffn[next_index] */
										if ((ffn_[ffn_[next_index]] == ffn_[current_index_]) || (sfn_[ffn_[next_index]] == ffn_[current_index_])) {
											state_[ffn_[next_index]] = COMPLETED;
										} else if (ffn_[ffn_[next_index]] == next_index) {
											ffn_[ffn_[next_index]] = ffn_[current_index_];
										} else if (sfn_[ffn_[next_index]] == next_index) {
											sfn_[ffn_[next_index]] = ffn_[current_index_];
										}

										ffn_[next_index] = current_index_;

										break;
									}
									case 1: // sfn[next]
									{
										addTriangle(next_index, ffn_[current_index_], sfn_[next_index]);
										neighbor_update = sfn_[next_index];

										/* sfn[next_index] */
										if ((ffn_[sfn_[next_index]] = ffn_[current_index_]) || (sfn_[sfn_[next_index]] == ffn_[current_index_])) {
											state_[sfn_[next_index]] = COMPLETED;
										} else if (ffn_[sfn_[next_index]] == next_index) {
											ffn_[sfn_[next_index]] = ffn_[current_index_];
										} else if (sfn_[sfn_[next_index]] == next_index) {
											sfn_[sfn_[next_index]] = ffn_[current_index_];
										}

										sfn_[next_index] = current_index_;

										break;
									}
									default:
										;
								}
							}
						}

						/* updating ffn */
						if ((ffn_[ffn_[current_index_]] == neighbor_update) || (sfn_[ffn_[current_index_]] == neighbor_update)) {
							state_[ffn_[current_index_]] = COMPLETED;
						} else if (ffn_[ffn_[current_index_]] == current_index_) {
							ffn_[ffn_[current_index_]] = neighbor_update;
						} else if (sfn_[ffn_[current_index_]] == current_index_) {
							sfn_[ffn_[current_index_]] = neighbor_update;
						}

						/* updating current */
						ffn_[current_index_] = prev_index;

						break;
					}
					case 3: //next2s:
					{
						addTriangle(current_index_, sfn_[current_index_], next_index);
						int neighbor_update = next_index;

						/* updating next_index */
						if (state_[next_index] <= FREE) {
							state_[next_index] = FRINGE;
							ffn_[next_index] = current_index_;
							sfn_[next_index] = sfn_[current_index_];
						} else {
							if (ffn_[next_index] == R_) {
								changed_2nd_fn_ = true;
								ffn_[next_index] = sfn_[current_index_];
							} else if (sfn_[next_index] == R_) {
								changed_2nd_fn_ = true;
								sfn_[next_index] = sfn_[current_index_];
							} else if (next_index == R_) {
								new2boundary_ = sfn_[current_index_];
								if (next_next_index == new2boundary_)
									already_connected_ = true;
							} else if (ffn_[next_index] == next_next_index) {
								already_connected_ = true;
								ffn_[next_index] = sfn_[current_index_];
							} else if (sfn_[next_index] == next_next_index) {
								already_connected_ = true;
								sfn_[next_index] = sfn_[current_index_];
							} else {
								tmp_ = coords_[ffn_[next_index]] - proj_qp_;
								uvn_next_ffn_[0] = tmp_.dot(u_);
								uvn_next_ffn_[1] = tmp_.dot(v_);
								tmp_ = coords_[sfn_[next_index]] - proj_qp_;
								uvn_next_sfn_[0] = tmp_.dot(u_);
								uvn_next_sfn_[1] = tmp_.dot(v_);

								bool ffn_next_sfn = isVisible(uvn_next_ffn_, uvn_next, uvn_current, uvn_sfn_)
										&& isVisible(uvn_next_ffn_, uvn_next, uvn_next_sfn_, uvn_sfn_);
								bool sfn_next_sfn = isVisible(uvn_next_sfn_, uvn_next, uvn_current, uvn_sfn_)
										&& isVisible(uvn_next_sfn_, uvn_next, uvn_next_ffn_, uvn_sfn_);

								int connect2sfn = -1;
								if (ffn_next_sfn && sfn_next_sfn) {
									double fn2s = (coords_[sfn_[current_index_]] - coords_[ffn_[next_index]]).lengthSquared();
									double sn2s = (coords_[sfn_[current_index_]] - coords_[sfn_[next_index]]).lengthSquared();
									if (fn2s < sn2s)
										connect2sfn = 0;
									else
										connect2sfn = 1;
								} else if (ffn_next_sfn)
									connect2sfn = 0;
								else if (sfn_next_sfn)
									connect2sfn = 1;

								switch (connect2sfn) {
									case 0: // ffn[next]
									{
										addTriangle(next_index, sfn_[current_index_], ffn_[next_index]);
										neighbor_update = ffn_[next_index];

										/* ffn[next_index] */
										if ((ffn_[ffn_[next_index]] == sfn_[current_index_]) || (sfn_[ffn_[next_index]] == sfn_[current_index_])) {
											state_[ffn_[next_index]] = COMPLETED;
										} else if (ffn_[ffn_[next_index]] == next_index) {
											ffn_[ffn_[next_index]] = sfn_[current_index_];
										} else if (sfn_[ffn_[next_index]] == next_index) {
											sfn_[ffn_[next_index]] = sfn_[current_index_];
										}

										ffn_[next_index] = current_index_;

										break;
									}
									case 1: // sfn[next]
									{
										addTriangle(next_index, sfn_[current_index_], sfn_[next_index]);
										neighbor_update = sfn_[next_index];

										/* sfn[next_index] */
										if ((ffn_[sfn_[next_index]] == sfn_[current_index_]) || (sfn_[sfn_[next_index]] == sfn_[current_index_])) {
											state_[sfn_[next_index]] = COMPLETED;
										} else if (ffn_[sfn_[next_index]] == next_index) {
											ffn_[sfn_[next_index]] = sfn_[current_index_];
										} else if (sfn_[sfn_[next_index]] == next_index) {
											sfn_[sfn_[next_index]] = sfn_[current_index_];
										}

										sfn_[next_index] = current_index_;

										break;
									}
									default:
										;
								}
							}
						}

						/* updating sfn */
						if ((ffn_[sfn_[current_index_]] == neighbor_update) || (sfn_[sfn_[current_index_]] == neighbor_update)) {
							state_[sfn_[current_index_]] = COMPLETED;
						} else if (ffn_[sfn_[current_index_]] == current_index_) {
							ffn_[sfn_[current_index_]] = neighbor_update;
						} else if (sfn_[sfn_[current_index_]] == current_index_) {
							sfn_[sfn_[current_index_]] = neighbor_update;
						}

						sfn_[current_index_] = prev_index;

						break;
					}
					default:
						;
				}
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////
void GreedyProjectionTriangulation::nearestKSearch(const Geometry::Vec3 &pos, int k, std::vector<int> &k_indices, std::vector<float> &k_distances) {
	PROFILE_SCOPE("Nearest K Search");
#ifdef USE_TREE_OCTREE
	std::deque<TreeEntry> k_nearest = tree_.getSortedClosestPoints(pos, k);
	for(uint32_t i=0; i<k; ++i) {
		k_distances[i] = pos.distanceSquared(k_nearest[i].getPosition());
		k_indices[i] = k_nearest[i].index;
	}
#endif
#ifdef USE_TREE_KDTREE
	tree_.nearestKSearch(pos, k, k_indices, k_distances);
	/*std::vector<TreeEntry> k_nearest;
	tree_.nearestKSearch(pos, k, k_nearest);
	for(uint32_t i=0; i<nnn_; ++i) {
		k_distances[i] = pos.distanceSquared(k_nearest[i].getPosition());
		k_indices[i] = k_nearest[i].index;
	}*/
#endif

}

} /* namespace MinSG */
