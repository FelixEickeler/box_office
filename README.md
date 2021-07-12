# BoxOffice - The Boxxing Bim 

###### ....is an evaluator for up and downscaling of point clouds

<img src="/docs/overview_paper.png" height="260"


## TODOS:
* write tests (c++) 
    * typdef tests
    * data loading 
    * isolated parts of the algorithm
    * maybe even integration
    
* write interface definitions to python
* make example in pyhton 
* loader and dataprocessing tests in python ?!?

### Datastructure
Behind the scenes the point cloud is saved in a single std::vector. The bounding boxes are supersed by rearrenging the 
point cloud with std::partition. This leaves the bounding box hierarchy (called FitAndSplitHierachy) completly without any points and just references to the start and end.

[comment]: <> (![]&#40;/docs/bounding_hirachie.png&#41;)
<img src="/docs/bounding_hirachie.png" height="700">

### CGal Conda
https://anaconda.org/conda-forge/cgal