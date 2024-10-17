#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <pcl/point_types.h>

#include "FEC.h"

namespace py = pybind11;

std::vector<int> FEC_py(std::vector<float> xyz, int min_component_size, double tolerance, int max_n) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointIndices> cluster_indices;

  int n_points = xyz.size() / 3;
  cloud->points.resize(n_points);
  
  // Iterate over the matrix in column-major order
  for (int i = 0; i < n_points; ++i) {
    cloud->points[i].x = xyz[3*i];
    cloud->points[i].y = xyz[3*i+1];
    cloud->points[i].z = xyz[3*i+2];
  }

  std::vector<int> flattened_indices (n_points, -1);
  cluster_indices = FEC(cloud, min_component_size, tolerance, max_n);
  for (int i_segment = 0; i_segment < cluster_indices.size(); i_segment++) {
    for (int j = 0; j < cluster_indices[i_segment].indices.size(); j++) {
      flattened_indices[cluster_indices[i_segment].indices[j]] = i_segment + 1;
    }
  }

  return flattened_indices;
}


PYBIND11_MODULE(fec, m) {
  m.def("FEC", &FEC_py, "FEC algorithm",
        py::arg("xyz"), py::arg("min_component_size"), py::arg("tolerance"), py::arg("max_n"));
}
