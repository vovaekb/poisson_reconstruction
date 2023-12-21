#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
// #include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/pcl_config.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>

#define THREADS_NUMBER 8

using namespace std;
using namespace pcl;

using PointType = pcl::PointXYZRGB;
using NormalType = pcl::PointXYZRGBNormal;

pcl::PolygonMesh poisson(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, int normalKSearch)
{
    // pcl::PointCloud<>::Ptr cloud_smoothed_normals = common_pcl::computeNormals(point_cloud, normalKSearch, true);

    cout << "begin normal estimation" << endl;
    NormalEstimationOMP<PointXYZRGB, Normal> ne;
    ne.setNumberOfThreads(THREADS_NUMBER);
    ne.setInputCloud(point_cloud);
    ne.setRadiusSearch(0.01);
    Eigen::Vector4f centroid;
    compute3DCentroid(*point_cloud, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

    PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
    ne.compute(*cloud_normals);
    cout << "normal estimation complete" << endl;
    cout << "reverse normals' direction" << endl;

    for (size_t i = 0; i < cloud_normals->size(); ++i)
    {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }

    cout << "combine points and normals" << endl;
    PointCloud<NormalType>::Ptr cloud_smoothed_normals(new PointCloud<NormalType>());
    // PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
    concatenateFields(*point_cloud, *cloud_normals, *cloud_smoothed_normals);

    std::cout << "* Poisson reconstruction... ";
    pcl::StopWatch watch;
    watch.reset();

    // Apply Poisson surface reconstruction
    pcl::PolygonMesh mesh;
    pcl::Poisson<NormalType> poisson;
    poisson.setDepth(9);
    poisson.setThreads(THREADS_NUMBER);
    poisson.setInputCloud(cloud_smoothed_normals);

    poisson.reconstruct(mesh);

    /* Make a copy of the mesh cloud since it will be overwritten later. */
    pcl::PCLPointCloud2 vertexCloud = mesh.cloud;

    /* Convert the input cloud to PCLPointCloud2. */
    pcl::PCLPointCloud2 colorCloud;
    pcl::toPCLPointCloud2(*cloud_smoothed_normals, colorCloud);

    /*
     * Use all the fields of the input cloud (i.e. PointXYZRGBNormal) but override
     * the xyz field with the xyz data from the poisson reconstruction.
     * Note that the data of the 2nd cloud is used in case the two input cloud
     * have the same fields.
     */
    pcl::concatenateFields(colorCloud, vertexCloud, mesh.cloud);

    std::cout << "complete in " << watch.getTimeSeconds() << " sec." << std::endl;
    return mesh;
}

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../a_cloud.pcd", *cloud);
    std::cout << PCL_VERSION << std::endl;

    cout << "Cloud is organized: " << cloud->isOrganized() << endl;
    cout << "Cloud size: " << cloud->size() << endl;

    return 0;
}
