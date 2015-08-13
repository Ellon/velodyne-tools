/**
 * cmake .; make; ./convert in.000 out.pcd
 */
#include <stdio.h>
#include <array> // C++11
#include <cmath> // std::sqrt
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

static const size_t VELODYNE_HEADER_LINE = 128;

typedef enum VELODYNE_POINT_STATUS {
    VELODYNE_BAD_POINT,
    VELODYNE_GOOD_POINT
} VELODYNE_POINT_STATUS;

typedef struct VELODYNE_CARTESIAN_POINT {
    VELODYNE_POINT_STATUS status;
    float x, y, z;
    unsigned char intensity;
} VELODYNE_CARTESIAN_POINT;

typedef std::array<double, 16> matrix;      // transformation matrix
/**
 * Pose6d(yaw,pitch,roll,x,y,z) -> Matrix[16]
 */
inline matrix pose6d_to_matrix(double yaw, double pitch, double roll,
                               double x, double y, double z) {
    matrix mat;
    double ca, sa, cb, sb, cg, sg;

    ca = cos(yaw);   sa = sin(yaw);
    cb = cos(pitch); sb = sin(pitch);
    cg = cos(roll);  sg = sin(roll);

    mat[0]  = ca*cb;
    mat[1]  = ca*sb*sg - sa*cg;
    mat[2]  = ca*sb*cg + sa*sg;
    mat[3]  = x;

    mat[4]  = sa*cb;
    mat[5]  = sa*sb*sg + ca*cg;
    mat[6]  = sa*sb*cg - ca*sg;
    mat[7]  = y;

    mat[8]  = -sb;
    mat[9]  = cb*sg;
    mat[10] = cb*cg;
    mat[11] = z;

    mat[12] = 0.0;
    mat[13] = 0.0;
    mat[14] = 0.0;
    mat[15] = 1.0;

    return mat;
}

/* ascii file header */
static const char magic[] = "Pom sensor position\n";
/* from pom-genom/libeuler/pomEuler.c:329 (pomReadSensorPos) */
int read_pose(char *file, matrix* sensor_to_main, matrix* main_to_origin) {
    static char tmp[50];
    FILE *f;
    int date;
    double yaw, pitch, roll, x, y, z;

    if ((f = fopen(file, "r")) == NULL) {
        return -1;
    }

    if (fgets(tmp, 50, f) == NULL) goto error;
    if (strcmp(tmp, magic)) goto error;

    /* read matrices */
    if (fscanf(f, "date %d\n", &date) != 1) goto error;
    if (fscanf(f, "sensorToMain = %lf %lf %lf %lf %lf %lf\n",
               &yaw, &pitch, &roll, &x, &y, &z) != 6) goto error;
    *sensor_to_main = pose6d_to_matrix(yaw, pitch, roll, x, y, z);
    if (fscanf(f, "mainToBase = %lf %lf %lf %lf %lf %lf\n",
               &yaw, &pitch, &roll, &x, &y, &z) != 6) goto error;
    if (fscanf(f, "mainToOrigin = %lf %lf %lf %lf %lf %lf\n",
               &yaw, &pitch, &roll, &x, &y, &z) != 6) goto error;
    *main_to_origin = pose6d_to_matrix(yaw, pitch, roll, x, y, z);

    fclose(f);
    return 0;
error:
    fclose(f);
    errno = EINVAL;
    return -1;
}

int main(int argc, char * argv[]) {
    FILE* file;
    char aux[VELODYNE_HEADER_LINE]; /* Auxiliary variable for fscanf. */
    int height, width, maxScanWidth, result, good = 0;
    size_t size;
    VELODYNE_CARTESIAN_POINT point;
    if (argc < 3) {
        std::cout << "usage: " << argv[0] << " in.0000 [in.pos.0000] out.pcd" << std::endl;
        return -1;
    }

    file = fopen(argv[1], "r");
    if (file == NULL)
      return -1;
    result = fscanf(file, "%s", aux);
    /* Reads the header. */
    result = fscanf(file, "%d %d %d\n", &height, &width, &maxScanWidth);
    if (result != 3)
      return -1;

    pcl::PointCloud<pcl::PointXYZI> output;
    output.height = 1;
    output.width = height * width;
    output.is_dense = true;
    output.points.resize( output.width );
    auto it = output.points.begin();
    for (size = 0; size < output.width; size++) {
      result = fread(&point, sizeof(VELODYNE_CARTESIAN_POINT), 1, file);
      if ((result == 1) && (point.status == VELODYNE_GOOD_POINT)) {
        (*it).x = point.x;
        (*it).y = point.y;
        (*it).z = point.z;
        (*it).intensity = point.intensity;
        it++;
      }
    }
    /* Closes the file. */
    fclose(file);
    output.width = std::distance(output.points.begin(), it);
    /* Removes the elements in the range [it,end] */
    output.points.erase(it, output.points.end());
    std::cout << "[info] " << output.width << " good points on a total of "
              << size << std::endl;
    if (argc > 3) {
        matrix sensor_to_main, main_to_origin;
        read_pose(argv[2], &sensor_to_main, &main_to_origin);
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
            eigen_sensor_to_main((double*)sensor_to_main.data());
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
            eigen_main_to_origin((double*)main_to_origin.data());
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> sto =
            eigen_main_to_origin * eigen_sensor_to_main;
        // std::cout << sto << std::endl;
        output.sensor_orientation_ =
            Eigen::Quaternionf( sto.topLeftCorner<3,3>().cast<float>() );
        output.sensor_origin_ =
            Eigen::Vector4f( sto.topRightCorner<4,1>().cast<float>() );
        pcl::io::savePCDFileBinaryCompressed(argv[3], output);
    } else {
        pcl::io::savePCDFileBinaryCompressed(argv[2], output);
    }
    return 0;
}
