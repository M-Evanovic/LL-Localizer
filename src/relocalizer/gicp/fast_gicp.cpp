#include <relocalizer/gicp/fast_gicp.hpp>
#include <relocalizer/gicp/impl/fast_gicp_impl.hpp>

template class fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>;
template class fast_gicp::FastGICP<pcl::PointNormal, pcl::PointNormal>;
template class fast_gicp::FastGICP<pcl::PointXYZRGB, pcl::PointXYZRGB>;