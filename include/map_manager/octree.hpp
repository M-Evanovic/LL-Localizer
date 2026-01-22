#ifndef _LL_LOCALIZER__OCTREE_HPP_
#define _LL_LOCALIZER__OCTREE_HPP_

#include <stdint.h>

#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace ll_localizer {
namespace common
{
    /**
     * Some traits to access coordinates regardless of the specific implementation of point
     * inspired by boost.geometry, which needs to be implemented by new points.
     *
     */
    namespace traits
    {

        template <typename PointT, int D>
        struct access
        {
        };

        template <class PointT>
        struct access<PointT, 0>
        {
            static float get(const PointT &p)
            {
                return p.x;
            }
        };

        template <class PointT>
        struct access<PointT, 1>
        {
            static float get(const PointT &p)
            {
                return p.y;
            }
        };

        template <class PointT>
        struct access<PointT, 2>
        {
            static float get(const PointT &p)
            {
                return p.z;
            }
        };
    } // namespace traits

    /** convenience function for access of point coordinates **/
    template <int D, typename PointT>
    inline float get(const PointT &p)
    {
        return traits::access<PointT, D>::get(p);
    }

    /**
     * Some generic distances: Manhattan, (squared) Euclidean, and Maximum distance.
     *
     * A Distance has to implement the methods
     * 1. compute of two points p and q to compute and return the distance between two points, and
     * 2. norm of x,y,z coordinates to compute and return the norm of a point p = (x,y,z)
     * 3. sqr and sqrt of value to compute the correct radius if a comparison is performed using squared norms (see
     *L2Distance)...
     */
    template <typename PointT>
    struct L1Distance
    {
        static inline float compute(const PointT &p, const PointT &q)
        {
            float diff1 = get<0>(p) - get<0>(q);
            float diff2 = get<1>(p) - get<1>(q);
            float diff3 = get<2>(p) - get<2>(q);

            return std::abs(diff1) + std::abs(diff2) + std::abs(diff3);
        }

        static inline float norm(float x, float y, float z)
        {
            return std::abs(x) + std::abs(y) + std::abs(z);
        }

        static inline float sqr(float r)
        {
            return r;
        }

        static inline float sqrt(float r)
        {
            return r;
        }
    };

    template <typename PointT>
    struct L2Distance
    {
        static inline float compute(const PointT &p, const PointT &q)
        {
            float diff1 = get<0>(p) - get<0>(q);
            float diff2 = get<1>(p) - get<1>(q);
            float diff3 = get<2>(p) - get<2>(q);

            return std::pow(diff1, 2) + std::pow(diff2, 2) + std::pow(diff3, 2);
        }

        static inline float norm(float x, float y, float z)
        {
            return std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2);
        }

        static inline float sqr(float r)
        {
            return r * r;
        }

        static inline float sqrt(float r)
        {
            return std::sqrt(r);
        }
    };

    template <typename PointT>
    struct MaxDistance
    {
        static inline float compute(const PointT &p, const PointT &q)
        {
            float diff1 = std::abs(get<0>(p) - get<0>(q));
            float diff2 = std::abs(get<1>(p) - get<1>(q));
            float diff3 = std::abs(get<2>(p) - get<2>(q));

            float maximum = diff1;
            if (diff2 > maximum)
                maximum = diff2;
            if (diff3 > maximum)
                maximum = diff3;

            return maximum;
        }

        static inline float norm(float x, float y, float z)
        {
            float maximum = x;
            if (y > maximum)
                maximum = y;
            if (z > maximum)
                maximum = z;
            return maximum;
        }

        static inline float sqr(float r)
        {
            return r;
        }

        static inline float sqrt(float r)
        {
            return r;
        }
    };

    struct OctreeParams
    {
    public:
        OctreeParams(uint32_t bucket_size = 32, float min_extent = 0.0f)
            : bucket_size(bucket_size), min_extent(min_extent)
        {
        }
        uint32_t bucket_size;

        float min_extent;
    };

    template <typename PointT, typename ContainerT = std::vector<PointT>>
    class Octree
    {
    public:
        Octree();
        ~Octree();

        Octree(const Octree &other);
        Octree &operator=(const Octree &other);

        void Initialize(const ContainerT &pts, const OctreeParams &params = OctreeParams());

        void clear();

        template <typename Distance>
        void RadiusNeighbors(const PointT &query, float radius, std::vector<uint32_t> &result_indices) const;

        template <typename Distance>
        int32_t NearestNeighbor(const PointT &query, float min_distance = -1) const;

    protected:
        class Octant
        {
        public:
            Octant();
            ~Octant();

            bool is_leaf;

            float x, y, z; // center
            float extent;  // half of side-length

            uint32_t start, end; // start and end in succ_
            uint32_t size;       // number of points

            Octant *child[8];
        };

        void CopyOctant(Octant *this_octant, const Octant *other_octant);

        Octant *CreateOctant(float x, float y, float z, float extent, uint32_t start_idx, uint32_t end_idx, uint32_t size);

        template <typename Distance>
        bool FindNearestNeighbor(const Octant *octant, const PointT &query, float min_distance, float &max_distance,
                                 int32_t &result_index) const;

        template <typename Distance>
        void FindRadiusNeighbors(const Octant *octant, const PointT &query, float radius, float sq_radius,
                                 std::vector<uint32_t> &result_indices) const;

        template <typename Distance>
        static bool Overlaps(const PointT &query, float radius, float sq_radius, const Octant *o);

        template <typename Distance>
        static bool Contains(const PointT &query, float sq_radius, const Octant *octant);

        template <typename Distance>
        static bool Inside(const PointT &query, float radius, const Octant *octant);

        OctreeParams params_;
        Octant *root_;
        const ContainerT *data_;

        std::vector<uint32_t> successors_; // single connected list of next point indices...
    };

    template <typename PointT, typename ContainerT>
    void Octree<PointT, ContainerT>::CopyOctant(Octant *this_octant, const Octant *other_octant)
    {
        this_octant->is_leaf = other_octant->is_leaf;
        this_octant->x = other_octant->x;
        this_octant->y = other_octant->y;
        this_octant->z = other_octant->z;
        this_octant->extent = other_octant->extent;
        this_octant->start = other_octant->start;
        this_octant->end = other_octant->end;
        this_octant->size = other_octant->size;

        for (uint32_t i = 0; i < 8; ++i)
        {
            if (other_octant->child[i])
            {
                this_octant->child[i] = new Octant(*other_octant->child[i]);
                CopyOctant(this_octant->child[i], other_octant->child[i]);
            }
            else
            {
                this_octant->child[i] = nullptr;
            }
        }
    }

    template <typename PointT, typename ContainerT>
    Octree<PointT, ContainerT>::Octree(const Octree &other)
        : params_(other.params_),
          root_(nullptr),
          data_(other.data_ ? new ContainerT(*other.data_) : nullptr),
          successors_(other.successors_)
    {

        if (other.root_)
        {
            root_ = new Octant(*other.root_);
            CopyOctant(root_, other.root_);
        }
    }

    template <typename PointT, typename ContainerT>
    Octree<PointT, ContainerT> &Octree<PointT, ContainerT>::operator=(const Octree &other)
    {
        if (this == &other)
        {
            return *this;
        }

        // Free the existing resources
        delete root_;
        delete data_;

        // Copy the members
        params_ = other.params_;
        root_ = nullptr;
        data_ = other.data_ ? new ContainerT(*other.data_) : nullptr;
        successors_ = other.successors_;

        if (other.root_)
        {
            root_ = new Octant(*other.root_);
            CopyOctant(root_, other.root_);
        }

        return *this;
    }

    template <typename PointT, typename ContainerT>
    Octree<PointT, ContainerT>::Octant::Octant()
        : is_leaf(true), x(0.0f), y(0.0f), z(0.0f), extent(0.0f), start(0), end(0), size(0)
    {
        memset(&child, 0, 8 * sizeof(Octant *));
    }

    template <typename PointT, typename ContainerT>
    Octree<PointT, ContainerT>::Octant::~Octant()
    {
        for (uint32_t i = 0; i < 8; ++i)
            delete child[i];
    }

    template <typename PointT, typename ContainerT>
    Octree<PointT, ContainerT>::Octree() : root_(0), data_(0)
    {
    }

    template <typename PointT, typename ContainerT>
    Octree<PointT, ContainerT>::~Octree()
    {
        delete root_;
        delete data_;
    }

    template <typename PointT, typename ContainerT>
    void Octree<PointT, ContainerT>::Initialize(const ContainerT &pts, const OctreeParams &params)
    {
        clear();
        params_ = params;

        data_ = new ContainerT(pts);

        const uint32_t N = pts.size();
        successors_ = std::vector<uint32_t>(N);

        // determine axis-aligned bounding box.
        float min[3], max[3];
        min[0] = get<0>(pts[0]);
        min[1] = get<1>(pts[0]);
        min[2] = get<2>(pts[0]);
        max[0] = min[0];
        max[1] = min[1];
        max[2] = min[2];

        for (uint32_t i = 0; i < N; ++i)
        {
            // initially each element links simply to the following element.
            successors_[i] = i + 1;

            const PointT &p = pts[i];

            if (get<0>(p) < min[0])
                min[0] = get<0>(p);
            if (get<1>(p) < min[1])
                min[1] = get<1>(p);
            if (get<2>(p) < min[2])
                min[2] = get<2>(p);
            if (get<0>(p) > max[0])
                max[0] = get<0>(p);
            if (get<1>(p) > max[1])
                max[1] = get<1>(p);
            if (get<2>(p) > max[2])
                max[2] = get<2>(p);
        }

        float ctr[3] = {min[0], min[1], min[2]};

        float maxextent = 0.5f * (max[0] - min[0]);
        ctr[0] += maxextent;
        for (uint32_t i = 1; i < 3; ++i)
        {
            float extent = 0.5f * (max[i] - min[i]);
            ctr[i] += extent;
            if (extent > maxextent)
                maxextent = extent;
        }

        root_ = CreateOctant(ctr[0], ctr[1], ctr[2], maxextent, 0, N - 1, N);
    }

    template <typename PointT, typename ContainerT>
    void Octree<PointT, ContainerT>::clear()
    {
        delete root_;
        delete data_;

        root_ = 0;
        data_ = 0;
        successors_.clear();
    }

    template <typename PointT, typename ContainerT>
    typename Octree<PointT, ContainerT>::Octant *Octree<PointT, ContainerT>::CreateOctant(float x, float y, float z,
                                                                                          float extent, uint32_t start_idx,
                                                                                          uint32_t end_idx, uint32_t size)
    {
        // For a leaf we don't have to change anything; points are already correctly linked or correctly reordered.
        Octant *octant = new Octant;

        octant->is_leaf = true;

        octant->x = x;
        octant->y = y;
        octant->z = z;
        octant->extent = extent;

        octant->start = start_idx;
        octant->end = end_idx;
        octant->size = size;

        static const float factor[] = {-0.5f, 0.5f};

        // subdivide subset of points and re-link points according to Morton codes
        if (size > params_.bucket_size && extent > 2 * params_.min_extent)
        {
            octant->is_leaf = false;

            const ContainerT &points = *data_;
            std::vector<uint32_t> child_starts(8, 0);
            std::vector<uint32_t> child_ends(8, 0);
            std::vector<uint32_t> child_sizes(8, 0);

            // re-link disjoint child subsets...
            uint32_t idx = start_idx;

            for (uint32_t i = 0; i < size; ++i)
            {
                const PointT &p = points[idx];

                // determine Morton code for each point...
                uint32_t morton_code = 0;
                if (get<0>(p) > x)
                    morton_code |= 1;
                if (get<1>(p) > y)
                    morton_code |= 2;
                if (get<2>(p) > z)
                    morton_code |= 4;

                // set child starts and update successors...
                if (child_sizes[morton_code] == 0)
                    child_starts[morton_code] = idx;
                else
                    successors_[child_ends[morton_code]] = idx;
                child_sizes[morton_code] += 1;

                child_ends[morton_code] = idx;
                idx = successors_[idx];
            }

            // now, we can create the child nodes...
            float child_extent = 0.5f * extent;
            bool firsttime = true;
            uint32_t last_child_idx = 0;
            for (uint32_t i = 0; i < 8; ++i)
            {
                if (child_sizes[i] == 0)
                    continue;

                float child_x = x + factor[(i & 1) > 0] * extent;
                float child_y = y + factor[(i & 2) > 0] * extent;
                float child_z = z + factor[(i & 4) > 0] * extent;

                octant->child[i] = CreateOctant(child_x, child_y, child_z, child_extent, child_starts[i], child_ends[i], child_sizes[i]);

                if (firsttime)
                    octant->start = octant->child[i]->start;
                else
                    successors_[octant->child[last_child_idx]->end] =
                        octant->child[i]->start; // we have to ensure that also the child ends link to the next child start.

                last_child_idx = i;
                octant->end = octant->child[i]->end;
                firsttime = false;
            }
        }

        return octant;
    }

    template <typename PointT, typename ContainerT>
    template <typename Distance>
    void Octree<PointT, ContainerT>::FindRadiusNeighbors(const Octant *octant, const PointT &query, float radius,
                                                         float sq_radius, std::vector<uint32_t> &result_indices) const
    {

        const ContainerT &points = *data_;

        // if search ball S(q,r) Contains octant, simply add point indexes.

        if (Contains<Distance>(query, sq_radius, octant))
        {
            uint32_t idx = octant->start;
            for (uint32_t i = 0; i < octant->size; ++i)
            {
                result_indices.push_back(idx);
                idx = successors_[idx];
            }

            return; // early pruning.
        }

        if (octant->is_leaf)
        {
            uint32_t idx = octant->start;
            for (uint32_t i = 0; i < octant->size; ++i)
            {
                const PointT &p = points[idx];
                float dist = Distance::compute(query, p);
                if (dist < sq_radius)
                    result_indices.push_back(idx);
                idx = successors_[idx];
            }

            return;
        }

        // check whether child nodes are in range.
        for (uint32_t c = 0; c < 8; ++c)
        {
            if (octant->child[c] == 0)
            {
                continue;
            }

            if (!Overlaps<Distance>(query, radius, sq_radius, octant->child[c]))
            {
                continue;
            }

            FindRadiusNeighbors<Distance>(octant->child[c], query, radius, sq_radius, result_indices);
        }
    }

    template <typename PointT, typename ContainerT>
    template <typename Distance>
    void Octree<PointT, ContainerT>::RadiusNeighbors(const PointT &query, float radius,
                                                     std::vector<uint32_t> &result_indices) const
    {
        result_indices.clear();
        if (root_ == 0)
            return;

        float sq_radius = Distance::sqr(radius); // "squared" radius
        FindRadiusNeighbors<Distance>(root_, query, radius, sq_radius, result_indices);
    }

    template <typename PointT, typename ContainerT>
    template <typename Distance>
    bool Octree<PointT, ContainerT>::Overlaps(const PointT &query, float radius, float sq_radius, const Octant *o)
    {
        // we exploit the symmetry to reduce the test to testing if its Inside the Minkowski sum around the positive quadrant.
        float x = get<0>(query) - o->x;
        float y = get<1>(query) - o->y;
        float z = get<2>(query) - o->z;

        x = std::abs(x);
        y = std::abs(y);
        z = std::abs(z);

        float maxdist = radius + o->extent;

        // Completely outside, since q' is outside the relevant area.
        if (x > maxdist || y > maxdist || z > maxdist)
            return false;

        int32_t num_less_extent = (x < o->extent) + (y < o->extent) + (z < o->extent);

        // Checking different cases:

        // a. Inside the surface region of the octant.
        if (num_less_extent > 1)
            return true;

        // b. checking the corner region && edge region.
        x = std::max(x - o->extent, 0.0f);
        y = std::max(y - o->extent, 0.0f);
        z = std::max(z - o->extent, 0.0f);

        return (Distance::norm(x, y, z) < sq_radius);
    }

    template <typename PointT, typename ContainerT>
    template <typename Distance>
    bool Octree<PointT, ContainerT>::Contains(const PointT &query, float sq_radius, const Octant *o)
    {
        // we exploit the symmetry to reduce the test to test
        // whether the farthest corner is Inside the search ball.
        float x = get<0>(query) - o->x;
        float y = get<1>(query) - o->y;
        float z = get<2>(query) - o->z;

        x = std::abs(x);
        y = std::abs(y);
        z = std::abs(z);
        // reminder: (x, y, z) - (-e, -e, -e) = (x, y, z) + (e, e, e)
        x += o->extent;
        y += o->extent;
        z += o->extent;

        return (Distance::norm(x, y, z) < sq_radius);
    }

    template <typename PointT, typename ContainerT>
    template <typename Distance>
    int32_t Octree<PointT, ContainerT>::NearestNeighbor(const PointT &query, float min_distance) const
    {
        float max_distance = std::numeric_limits<float>::infinity();
        int32_t result_index = -1;
        if (root_ == 0)
            return result_index;

        FindNearestNeighbor<Distance>(root_, query, min_distance, max_distance, result_index);

        return result_index;
    }

    template <typename PointT, typename ContainerT>
    template <typename Distance>
    bool Octree<PointT, ContainerT>::FindNearestNeighbor(const Octant *octant, const PointT &query, float min_distance,
                                                         float &max_distance, int32_t &result_index) const
    {
        const ContainerT &points = *data_;
        // 1. first descend to leaf and check in leafs points.
        if (octant->is_leaf)
        {
            uint32_t idx = octant->start;
            float sqr_max_distance = Distance::sqr(max_distance);
            float sqr_min_distance = (min_distance < 0) ? min_distance : Distance::sqr(min_distance);

            for (uint32_t i = 0; i < octant->size; ++i)
            {
                const PointT &p = points[idx];
                float dist = Distance::compute(query, p);
                if (dist > sqr_min_distance && dist < sqr_max_distance)
                {
                    result_index = idx;
                    sqr_max_distance = dist;
                }
                idx = successors_[idx];
            }

            max_distance = Distance::sqrt(sqr_max_distance);
            return Inside<Distance>(query, max_distance, octant);
        }

        // determine Morton code for each point...
        uint32_t morton_code = 0;
        if (get<0>(query) > octant->x)
            morton_code |= 1;
        if (get<1>(query) > octant->y)
            morton_code |= 2;
        if (get<2>(query) > octant->z)
            morton_code |= 4;

        if (octant->child[morton_code] != 0)
        {
            if (FindNearestNeighbor<Distance>(octant->child[morton_code], query, min_distance, max_distance, result_index))
                return true;
        }

        // 2. if current best point completely Inside, just return.
        float sqr_max_distance = Distance::sqr(max_distance);

        // 3. check adjacent octants for overlap and check these if necessary.
        for (uint32_t c = 0; c < 8; ++c)
        {
            if (c == morton_code)
                continue;
            if (octant->child[c] == 0)
                continue;
            if (!Overlaps<Distance>(query, max_distance, sqr_max_distance, octant->child[c]))
                continue;
            if (FindNearestNeighbor<Distance>(octant->child[c], query, min_distance, max_distance, result_index))
                return true; // early pruning
        }

        // all children have been checked...check if point is Inside the current octant...
        return Inside<Distance>(query, max_distance, octant);
    }

    template <typename PointT, typename ContainerT>
    template <typename Distance>
    bool Octree<PointT, ContainerT>::Inside(const PointT &query, float radius, const Octant *octant)
    {
        // we exploit the symmetry to reduce the test to test
        // whether the farthest corner is Inside the search ball.
        float x = get<0>(query) - octant->x;
        float y = get<1>(query) - octant->y;
        float z = get<2>(query) - octant->z;

        x = std::abs(x) + radius;
        y = std::abs(y) + radius;
        z = std::abs(z) + radius;

        if (x > octant->extent)
            return false;
        if (y > octant->extent)
            return false;
        if (z > octant->extent)
            return false;

        return true;
    }
} // namespace common

}   // namespace ll_localizer

#endif /* OCTREE_HPP_ */
