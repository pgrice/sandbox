#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

class FeatureCloud
{
    public:
        //a bit of shorthand
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
        typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
        typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

        FeatureCloud(): search_method_xyz_ (new SearchMethod), normal_radius_(0.02f), feature_radius_(0.02f) {}
        ~FeatureCloud() {}; 

        // Process the given cloud
        void setInputCloud(PointCloud::Ptr xyz)
        {
            xyz_ = xyz;
            processInput();
        }

        // Load and process cloud from given PCD File
        void loadInputCloud(const std::string &pcd_file)
        {
            xyz_ = PointCloud::Ptr (new PointCloud);
            pcl::io::loadPCDFile(pcd_file, *xyz_);
            processInput();
        }

        void loadInputPLY(const std::string &ply_file)
        {
            xyz_ = PointCloud::Ptr (new PointCloud);
            pcl::io::loadPLYFile(ply_file, *xyz_);
            processInput();
        }

        //Get a pointer to the cloud of 3d Points
        PointCloud::Ptr getPointCloud() const 
        { 
            return (xyz_); 
        }

        // Get a pointer to the cloud of 3D surface normals
        SurfaceNormals::Ptr getSurfaceNormals() const
        {
            return(normals_);
        }

        //Get a pointer to the cloud of feature descriptors
        LocalFeatures::Ptr getLocalFeatures() const
        {
            return (features_);
        }

    protected:
        //Compute surface normals and local features
        void processInput()
        {
            computeSurfaceNormals();
            computeLocalFeatures();
        }

        //Compute surface normals
        void computeSurfaceNormals() 
        {
            normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
            norm_est.setInputCloud (xyz_);
            norm_est.setSearchMethod(search_method_xyz_);
            norm_est.setRadiusSearch(normal_radius_);
            norm_est.compute(*normals_);
        }

        //Compute the local feature descriptors
        void computeLocalFeatures() 
        {
            features_ = LocalFeatures::Ptr (new LocalFeatures);

            pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
            fpfh_est.setInputCloud(xyz_);
            fpfh_est.setInputNormals(normals_);
            fpfh_est.setSearchMethod(search_method_xyz_);
            fpfh_est.setRadiusSearch(feature_radius_);
            fpfh_est.compute(*features_);
        }

    private:
        //PointCloud data
        PointCloud::Ptr xyz_;
        SurfaceNormals::Ptr normals_;
        LocalFeatures::Ptr features_;
        SearchMethod::Ptr search_method_xyz_;

        //Parameters
        float normal_radius_;
        float feature_radius_;
};

int main (int argc, char **argv)
{
    if (argc < 3)
    {
        printf("No target PCD file given!\n");
        return (-1);
    }

    // Load makehuman ply mesh
    FeatureCloud head_cloud;
    head_cloud.loadInputPLY(argv[1]);

    // Load target PCD cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(argv[2], *cloud);

    //Preprocess cloud by removing distant points
    //Replace with filter around face-detected center
    const float depth_limit = 1.0;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, depth_limit);
    pass.filter(*cloud);

    //downsample pointcloud
    const float voxel_grid_size = 0.005f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud(cloud);
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
    vox_grid.filter(*tempCloud);
    cloud = tempCloud;

    //Assign to the target FeatureCloud
    FeatureCloud target_cloud;
    target_cloud.setInputCloud(cloud);

    
    float min_sample_distance_ = 0.05f;
    float max_correspondence_distance_ = 0.01f*0.01f;
    int nr_iterations_ = 500;
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;

    sac_ia_.setMinSampleDistance (min_sample_distance_);
    sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
    sac_ia_.setMaximumIterations (nr_iterations_);

    sac_ia_.setInputTarget(target_cloud.getPointCloud());
    sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());


    sac_ia_.setInputCloud(head_cloud.getPointCloud());
    sac_ia_.setSourceFeatures(head_cloud.getLocalFeatures());
    pcl::PointCloud<pcl::PointXYZ> registration_output;
    sac_ia_.align(registration_output);

    float fitness_score = (float) sac_ia_.getFitnessScore(max_correspondence_distance_);
    Eigen::Matrix4f final_transformation = sac_ia_.getFinalTransformation();
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //Print the alignment fitness score (values < 0.00002 are good)
    printf("Fitness Score: %f\n", fitness_score);

    //Print rotation matrix and trans vector
    Eigen::Matrix3f rotation = final_transformation.block<3,3>(0,0);
    Eigen::Vector3f translation = final_transformation.block<3,1>(0,3);

    printf("\n");
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(0,0), rotation(0,1), rotation(0,2));
    printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1,0), rotation(1,1), rotation(1,2));
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(2,0), rotation(2,1), rotation(2,2));
    printf("\n");
    printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

    //Save aligned template for visualization
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*head_cloud.getPointCloud(), transformed_cloud, final_transformation);
    pcl::io::savePCDFileBinary("output.pcd", transformed_cloud);

    return(0);
}
