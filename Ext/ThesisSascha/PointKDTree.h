/*
 * PointKDTree.h
 *
 *  Created on: Jun 23, 2014
 *      Author: meisteryeti
 */

#ifndef PointKDTree_H_
#define PointKDTree_H_

#ifdef MINSG_HAVE_LIB_FLANN

#include <flann/flann.h>

#include <Util/Macros.h>
#include <Geometry/Vec3.h>

#include <deque>
#include <memory>

// Forward declarations
/*namespace flann {
struct SearchParams;
template<typename T> struct L2_Simple;
template<typename T> class Index;
}*/

template<typename Point_t, typename Dist = ::flann::L2_Simple<float>>
class PointKDTree {
	public:
		typedef Point_t point_t;
		typedef Dist dist_t;

		typedef ::flann::Index<dist_t> FLANNIndex;
		typedef ::flann::IndexParams IndexParams;

		PointKDTree(const IndexParams& params = ::flann::KDTreeSingleIndexParams(), bool sorted = true);
		virtual ~PointKDTree() = default;

		/**
		 * inserts the point into the kd-tree		 *
		 * @param point Data item containing the position
		 */
		inline bool insert(const Point_t & point, bool immediate = false) ;

		/** \brief Search for k-nearest neighbors for the given query point.
		 *
		 * \attention This method does not do any bounds checking for the input index
		 * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
		 *
		 * \param[in] point a given \a valid (i.e., finite) query point
		 * \param[in] k the number of neighbors to search for
		 * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
		 * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
		 * a priori!)
		 * \return number of neighbors found
		 *
		 * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
		 */
		int nearestKSearch(const Geometry::Vec3 &pos, int k, std::vector<int> &k_indices, std::vector<float> &k_distances);
		int getClosestPoints(const Geometry::Vec3 &pos, int k, std::deque<Point_t>& points);

		/** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
		 * \param[in] eps precision (error bound) for nearest neighbors searches
		 */
		virtual inline void setEpsilon(float eps) {
			epsilon_ = eps;
		}

		/** \brief Get the search epsilon precision (error bound) for nearest neighbors searches. */
		inline float getEpsilon() const {
			return (epsilon_);
		}

		/** \brief Minimum allowed number of k nearest neighbors points that a viable result must contain.
		 * \param[in] min_pts the minimum number of neighbors in a viable neighborhood
		 */
		inline void setMinPts(int min_pts) {
			min_pts_ = min_pts;
		}

		/** \brief Get the minimum allowed number of k nearest neighbors points that a viable result must contain. */
		inline int getMinPts() const {
			return (min_pts_);
		}

		inline void setRebuildThreshold(float rebuild_threshold) {
			rebuild_threshold_ = rebuild_threshold;
		}

		inline float getRebuildThreshold() const {
			return (rebuild_threshold_);
		}

		inline void clear() {
			points_.clear();
		}

		inline void invalidate() { changed_ = true; };
	private:
		inline void validate();

		/** \brief A FLANN index object. */
		std::shared_ptr<FLANNIndex> flann_index_;

		/** \brief Epsilon precision (error bound) for nearest neighbors searches. */
		float epsilon_;

		/** \brief Minimum allowed number of k nearest neighbors points that a viable result must contain. */
		int min_pts_;

		/** \brief Return the radius search neighbours sorted **/
		bool sorted_;

		/** \brief Points that are stored inside this tree. */
		std::deque<Point_t> points_;

		/** \brief The KdTree search parameters for K-nearest neighbors. */
		::flann::SearchParams param_k_;

		bool changed_;

		IndexParams index_params_;

		float rebuild_threshold_;
};

///////////////////////////////////////////////////////////////////////////////////////////
template<typename Point_t, typename Dist>
PointKDTree<Point_t, Dist>::PointKDTree(const IndexParams& params, bool sorted) :
		flann_index_(),
		epsilon_(0.0f), min_pts_(1), sorted_(sorted), points_(), param_k_(::flann::SearchParams(-1, epsilon_)), changed_(true), index_params_(params), rebuild_threshold_(2) {
	clear();
}

///////////////////////////////////////////////////////////////////////////////////////////
template<typename Point_t, typename Dist>
inline void PointKDTree<Point_t, Dist>::validate() {
	if(changed_ && !points_.empty()) {
		std::vector<float> data;
		data.reserve(points_.size() * 3);
		for(Point_t point : points_) {
			data.push_back(point.getPosition().x());
			data.push_back(point.getPosition().y());
			data.push_back(point.getPosition().z());
		}
		flann_index_.reset(new FLANNIndex(flann::Matrix<float>(&data[0], points_.size(), 3),index_params_));
		flann_index_->buildIndex();
	}
	changed_ = false;
}

///////////////////////////////////////////////////////////////////////////////////////////
template<typename Point_t, typename Dist>
inline bool PointKDTree<Point_t, Dist>::insert(const Point_t & point, bool immediate) {
	if(immediate) {
		static float data[3];
		point.getPosition().toArray(data);
		points_.push_back(point);
		if(changed_)
			validate();
		else
			flann_index_->addPoints(flann::Matrix<float>(data, 1, 3));
	} else {
		points_.push_back(point);
		invalidate();
	}
}
///////////////////////////////////////////////////////////////////////////////////////////
template<typename Point_t, typename Dist>
int PointKDTree<Point_t, Dist>::nearestKSearch(const Geometry::Vec3 &pos, int k, std::vector<int> &k_indices,
																							std::vector<float> &k_distances) {
	//assert(point_representation_->isValid(point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
	validate();

	static float query[3];
	pos.toArray(query);

	if (k > points_.size())
		k = points_.size();

	k_indices.resize(k);
	k_distances.resize(k);

	::flann::Matrix<int> k_indices_mat(&k_indices[0], 1, k);
	::flann::Matrix<float> k_distances_mat(&k_distances[0], 1, k);
	// Wrap the k_indices and k_distances vectors (no data copy)
	flann_index_->knnSearch(::flann::Matrix<float>(query, 1, 3), k_indices_mat, k_distances_mat, k, param_k_);

	return k;
}
///////////////////////////////////////////////////////////////////////////////////////////
template<typename Point_t, typename Dist>
int PointKDTree<Point_t, Dist>::getClosestPoints(const Geometry::Vec3 &pos, int k, std::deque<Point_t>& points) {
	std::vector<int> k_indices;
	std::vector<float> k_distances;
	nearestKSearch(pos, k, k_indices, k_distances);
	for(int index : k_indices) {
		points.push_back(points_[index]);
	}
}

#endif /* MINSG_HAVE_LIB_FLANN */
#endif /* PointKDTree_H_ */
