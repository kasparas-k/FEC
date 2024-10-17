# FEC: Fast Euclidean Clustering for Point Cloud Segmentation

This is a **fork** of the official repository for _**FEC**_. The main goal of this repository is to provide Python bindings of the performant C++ code for easy integration with existing Python workflows.

## Prerequisites

You'll need to be able to compile c++ code with `cmake` and `make`:

```bash
sudo apt install cmake make g++
```

This code depends on C++ libraries:

- [Boost](https://www.boost.org/)
- [Eigen3](https://eigen.tuxfamily.org/)
- [PCL](https://pointclouds.org/)
- [pybind11](https://github.com/pybind/pybind11)

It is recommended to install these dependencies with `conda`, as the package versions in `apt` are outdated, and vary between by distribution:
```bash
conda install -c conda-forge boost eigen pcl=1.14.1 pybind11
```

## Installation

This is intended to be added as another package in your existing Point Cloud processing environments that meet the prerequisites (see the section above).

In a pre-existing environment, the package can be installed directly from GitHub:
```bash
pip install git+https://github.com/kasparas-k/FEC.git
```

Or by cloning the repository:
```bash
git clone https://github.com/kasparas-k/FEC.git
cd FEC
pip install .
```

## Examples

Example of a short program using [laspy](https://github.com/laspy/laspy) to read a `.las` file, cluster all points belonging to class `3`, and write the output:

```python
import laspy
import numpy as np

from fec import FEC

las = laspy.read('path/to/pointcloud.las')
las.points = las.points[las.classification == 3]

cluster_indices = FEC(las.xyz, min_component_size=50, tolerance=0.5, max_n=50)

las.add_extra_dim(laspy.ExtraBytesParams(
    name="clusterID",
    type=np.int32,
))
las.clusterID = np.array(cluster_indices)

las.write('path/to/output.las')
```

## Citation

In case you use _**FEC**_ in your research or work, it would be highly appreciated if you include a reference to the original authors [paper](https://arxiv.org/abs/2208.07678) in any kind of publication.

```bibtex
@Article{cao2022fec,
  AUTHOR = {Cao, Yu and Wang, Yancheng and Xue, Yifei and Zhang, Huiqing and Lao, Yizhen},
  TITLE = {FEC: Fast Euclidean Clustering for Point Cloud Segmentation},
  JOURNAL = {Drones},
  VOLUME = {6},
  YEAR = {2022},
  NUMBER = {11},
  ARTICLE-NUMBER = {325},
  URL = {https://www.mdpi.com/2504-446X/6/11/325},
  ISSN = {2504-446X},
  ABSTRACT = {Segmentation from point cloud data is essential in many applications, such as remote sensing, mobile robots, or autonomous cars. However, the point clouds captured by the 3D range sensor are commonly sparse and unstructured, challenging efficient segmentation. A fast solution for point cloud instance segmentation with small computational demands is lacking. To this end, we propose a novel fast Euclidean clustering (FEC) algorithm which applies a point-wise scheme over the cluster-wise scheme used in existing works. The proposed method avoids traversing every point constantly in each nested loop, which is time and memory-consuming. Our approach is conceptually simple, easy to implement (40 lines in C++), and achieves two orders of magnitudes faster against the classical segmentation methods while producing high-quality results.},
  DOI = {10.3390/drones6110325}
}
```

## License

MIT © Yizhen LAO