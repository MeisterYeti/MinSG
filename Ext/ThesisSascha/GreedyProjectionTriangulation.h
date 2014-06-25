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

#ifndef GREEDYPROJECTIONTRIANGULATION_H_
#define GREEDYPROJECTIONTRIANGULATION_H_

#ifdef MINSG_HAVE_LIB_FLANN
#define USE_TREE_KDTREE
#else
#define USE_TREE_OCTREE
#endif

#include <Util/References.h>
#include <Geometry/Vec2.h>
#include <Geometry/Vec3.h>
#include <Geometry/Point.h>


#ifdef USE_TREE_KDTREE
#undef USE_TREE_OCTREE
#endif
#ifdef USE_TREE_OCTREE
#include <Geometry/PointOctree.h>
#endif
#ifdef USE_TREE_KDTREE
#include "PointKDTree.h"
#endif

#include <vector>

namespace Rendering {
class Mesh;
}

namespace MinSG {

/** \brief GreedyProjectionTriangulation is an implementation of a greedy triangulation algorithm for 3D points
  * based on local 2D projections. It assumes locally smooth surfaces and relatively smooth transitions between
  * areas with different point densities.
  * \author Zoltan Csaba Marton
  * \author Sascha Brandt (adapted to PADrend framework)
  * \ingroup surface
  */
class GreedyProjectionTriangulation {
public:

    enum GP3Type {
      NONE = -1,    // not-defined
      FREE = 0,
      FRINGE = 1,
      BOUNDARY = 2,
      COMPLETED = 3
    };

	GreedyProjectionTriangulation();
	virtual ~GreedyProjectionTriangulation() = default;
	/** \brief Set the multiplier of the nearest neighbor distance to obtain the final search radius for each point
	*  (this will make the algorithm adapt to different point densities in the cloud).
	* \param[in] mu the multiplier
	*/
	inline void setMu (double mu) { mu_ = mu; }

	/** \brief Get the nearest neighbor distance multiplier. */
	inline double getMu () const { return (mu_); }

	/** \brief Set the maximum number of nearest neighbors to be searched for.
	* \param[in] nnn the maximum number of nearest neighbors
	*/
	inline void setMaximumNearestNeighbors (uint32_t nnn) { nnn_ = nnn; }

	/** \brief Get the maximum number of nearest neighbors to be searched for. */
	inline uint32_t getMaximumNearestNeighbors () const { return (nnn_); }

	/** \brief Set the sphere radius that is to be used for determining the k-nearest neighbors used for triangulating.
	* \param[in] radius the sphere radius that is to contain all k-nearest neighbors
	* \note This distance limits the maximum edge length!
	*/
	inline void setSearchRadius (double radius) { search_radius_ = radius; }

	/** \brief Get the sphere radius used for determining the k-nearest neighbors. */
	inline double getSearchRadius () const { return (search_radius_); }

	/** \brief Set the minimum angle each triangle should have.
	* \param[in] minimum_angle the minimum angle each triangle should have
	* \note As this is a greedy approach, this will have to be violated from time to time
	*/
	inline void setMinimumAngle (double minimum_angle) { minimum_angle_ = minimum_angle; }

	/** \brief Get the parameter for distance based weighting of neighbors. */
	inline double getMinimumAngle () const { return (minimum_angle_); }

	/** \brief Set the maximum angle each triangle can have.
	* \param[in] maximum_angle the maximum angle each triangle can have
	* \note For best results, its value should be around 120 degrees
	*/
	inline void setMaximumAngle (double maximum_angle) { maximum_angle_ = maximum_angle; }

	/** \brief Get the parameter for distance based weighting of neighbors. */
	inline double getMaximumAngle () const { return (maximum_angle_); }

	/** \brief Don't consider points for triangulation if their normal deviates more than this value from the query point's normal.
	* \param[in] eps_angle maximum surface angle
	* \note As normal estimation methods usually give smooth transitions at sharp edges, this ensures correct triangulation
	*       by avoiding connecting points from one side to points from the other through forcing the use of the edge points.
	*/
	inline void setMaximumSurfaceAngle (double eps_angle) { eps_angle_ = eps_angle; }

	/** \brief Get the maximum surface angle. */
	inline double getMaximumSurfaceAngle () const { return (eps_angle_); }

	/** \brief Set the flag if the input normals are oriented consistently.
	* \param[in] consistent set it to true if the normals are consistently oriented
	*/
	inline void setNormalConsistency (bool consistent) { consistent_ = consistent; }

	/** \brief Get the flag for consistently oriented normals. */
	inline bool getNormalConsistency () const { return (consistent_); }

	/** \brief Set the flag to order the resulting triangle vertices consistently (positive direction around normal).
	* @note Assumes consistently oriented normals (towards the viewpoint) -- see setNormalConsistency ()
	* \param[in] consistent_ordering set it to true if triangle vertices should be ordered consistently
	*/
	inline void setConsistentVertexOrdering (bool consistent_ordering) { consistent_ordering_ = consistent_ordering; }

	/** \brief Get the flag signaling consistently ordered triangle vertices. */
	inline bool getConsistentVertexOrdering () const { return (consistent_ordering_); }

	/** \brief Get the state of each point after reconstruction.
	* \note Options are defined as constants: FREE, FRINGE, COMPLETED, BOUNDARY and NONE
	*/
	//inline std::vector<int>  getPointStates () const { return (state_); }

	/** \brief Get the ID of each point after reconstruction.
	* \note parts are numbered from 0, a -1 denotes unconnected points
	*/
	// inline std::vector<int>  getPartIDs () const { return (part_); }


	/** \brief Get the sfn list. */
	// inline std::vector<int> getSFN () const { return (sfn_); }

	/** \brief Get the ffn list. */
	// inline std::vector<int> getFFN () const { return (ffn_); }
protected:
    /** \brief The nearest neighbor distance multiplier to obtain the final search radius. */
    double mu_;
    /** \brief The nearest neighbors search radius for each point and the maximum edge length. */
    double search_radius_;
    /** \brief The maximum number of nearest neighbors accepted by searching. */
    uint32_t nnn_;
    /** \brief The preferred minimum angle for the triangles. */
    double minimum_angle_;
    /** \brief The maximum angle for the triangles. */
    double maximum_angle_;
    /** \brief Maximum surface angle. */
    double eps_angle_;
    /** \brief Set this to true if the normals of the input are consistently oriented. */
    bool consistent_;
    /** \brief Set this to true if the output triangle vertices should be consistently oriented. */
    bool consistent_ordering_;
    /** \brief A pointer to the vector of point indices to use. */
    std::vector<uint32_t> indices_;

private:
	struct TreeEntry : public Geometry::Point<Geometry::Vec3f> {
		uint32_t index;
		TreeEntry(uint32_t i,const Geometry::Vec3 & p) : Geometry::Point<Geometry::Vec3f>(p), index(i) {}
	};
#ifdef USE_TREE_OCTREE
	Geometry::PointOctree<TreeEntry> tree_;
#endif
#ifdef USE_TREE_KDTREE
	PointKDTree<TreeEntry> tree_;
#endif

	/** \brief Struct for storing the angles to nearest neighbors **/
	struct nnAngle {
		double angle;
		int index;
		int nnIndex;
		bool visible;
	};

	/** \brief Struct for storing the edges starting from a fringe point **/
	struct doubleEdge {
		uint32_t index;
		Geometry::Vec2 first;
		Geometry::Vec2 second;
	};

	// Variables made global to decrease the number of parameters to helper functions

	/** \brief Temporary variable to store a triangle (as a set of point indices) **/
	std::vector<uint32_t> triangle_;
	/** \brief Temporary variable to store point coordinates **/
	std::vector<Geometry::Vec3> coords_;
	/** \brief Temporary variable to store point normals **/
	std::vector<Geometry::Vec3> normals_;

	/** \brief A list of angles to neighbors **/
	std::vector<nnAngle> angles_;
	/** \brief Index of the current query point **/
	int R_;
	/** \brief List of point states **/
	std::vector<int> state_;
	/** \brief List of sources **/
	std::vector<int> source_;
	/** \brief List of fringe neighbors in one direction **/
	std::vector<int> ffn_;
	/** \brief List of fringe neighbors in other direction **/
	std::vector<int> sfn_;
	/** \brief Connected component labels for each point **/
	std::vector<int> part_;
	/** \brief Points on the outer edge from which the mesh has to be grown **/
	std::vector<uint32_t> fringe_queue_;

	/** \brief Flag to set if the current point is free **/
	bool is_current_free_;
	/** \brief Current point's index **/
	int current_index_;
	/** \brief Flag to set if the previous point is the first fringe neighbor **/
	bool prev_is_ffn_;
	/** \brief Flag to set if the next point is the second fringe neighbor **/
	bool prev_is_sfn_;
	/** \brief Flag to set if the next point is the first fringe neighbor **/
	bool next_is_ffn_;
	/** \brief Flag to set if the next point is the second fringe neighbor **/
	bool next_is_sfn_;
	/** \brief Flag to set if the first fringe neighbor was changed **/
	bool changed_1st_fn_;
	/** \brief Flag to set if the second fringe neighbor was changed **/
	bool changed_2nd_fn_;
	/** \brief New boundary point **/
	int new2boundary_;

	/** \brief Flag to set if the next neighbor was already connected in the previous step.
	* To avoid inconsistency it should not be connected again.
	*/
	bool already_connected_;

	/** \brief Point coordinates projected onto the plane defined by the point normal **/
	Geometry::Vec3 proj_qp_;
	/** \brief First coordinate vector of the 2D coordinate frame **/
	Geometry::Vec3 u_;
	/** \brief Second coordinate vector of the 2D coordinate frame **/
	Geometry::Vec3 v_;
	/** \brief 2D coordinates of the first fringe neighbor **/
	Geometry::Vec2 uvn_ffn_;
	/** \brief 2D coordinates of the second fringe neighbor **/
	Geometry::Vec2 uvn_sfn_;
	/** \brief 2D coordinates of the first fringe neighbor of the next point **/
	Geometry::Vec2 uvn_next_ffn_;
	/** \brief 2D coordinates of the second fringe neighbor of the next point **/
	Geometry::Vec2 uvn_next_sfn_;

	/** \brief Temporary variable to store 3 coordiantes **/
	Geometry::Vec3 tmp_;
public:

	bool reconstruct(Rendering::Mesh* source);

private:

    /** \brief The actual surface reconstruction method.
      */
	bool reconstructMesh(uint32_t index_count);

    /** \brief Forms a new triangle by connecting the current neighbor to the query point
      * and the previous neighbor
      * \param[in] prev_index index of the previous point
      * \param[in] next_index index of the next point
      * \param[in] next_next_index index of the point after the next one
      * \param[in] uvn_current 2D coordinate of the current point
      * \param[in] uvn_prev 2D coordinates of the previous point
      * \param[in] uvn_next 2D coordinates of the next point
      */
    void connectPoint (const int prev_index,
                  const int next_index,
                  const int next_next_index,
                  const Geometry::Vec2 &uvn_current,
                  const Geometry::Vec2 &uvn_prev,
                  const Geometry::Vec2 &uvn_next);

	/** \brief Add a new triangle to the current polygon mesh
	 * \param[in] a index of the first vertex
	 * \param[in] b index of the second vertex
	 * \param[in] c index of the third vertex
	 */
	void addTriangle (int a, int b, int c);

	/** \brief Whenever a query point is part of a boundary loop containing 3 points, that triangle is created
	* (called if angle constraints make it possible)
	*/
	void closeTriangle ();

    /** \brief Add a new vertex to the advancing edge front and set its source point
      * \param[in] v index of the vertex that was connected
      * \param[in] s index of the source point
      */
    inline void addFringePoint (uint32_t v, uint32_t s) {
      source_[v] = s;
      part_[v] = part_[s];
      fringe_queue_.push_back(v);
    }

    /** \brief Function for ascending sort of nnAngle, taking visibility into account
    * (angles to visible neighbors will be first, to the invisible ones after).
    * \param[in] a1 the first angle
    * \param[in] a2 the second angle
    */
    static inline bool nnAngleSortAsc (const nnAngle& a1, const nnAngle& a2) {
    	if (a1.visible == a2.visible)
    	  return (a1.angle < a2.angle);
    	else
    	  return a1.visible;
    }

    void nearestKSearch(const Geometry::Vec3 &pos, int k, std::vector<int> &k_indices, std::vector<float> &k_distances);
};

} /* namespace MinSG */

#endif /* GREEDYPROJECTIONTRIANGULATION_H_ */
