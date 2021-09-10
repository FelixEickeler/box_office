# BoxOffice - The Boxxing Bim 

###### ....is an evaluator for up and downscaling of point clouds
[comment]: <>
<img src="/docs/overview_paper.png" height="260">

The concept was presented on the Forum Bauinformatik 2021. Usage is experimental and cgal is sometime a little unstable.
It will create a box hirachy of a given segmented point cloud.
[comment]: <>
<img src="/docs/4_all_ob.png" height="260">

### Datastructure
Behind the scenes the point cloud is saved in a single std::vector. The bounding boxes are supersed by rearrenging the
point cloud with std::partition. This leaves the bounding box hierarchy (called FitAndSplitHierachy) completly without any points and just references to the start and end.

[comment]: <> (![]&#40;/docs/bounding_hirachie.png&#41;)
<img src="/docs/bounding_hirachie.png" height="700">

### Python Interface
[comment]: <>
<img src="/docs/python_interface.png" height="260">

# How to get started:
- start the docker
- recompile the c++ library with cmake and replace the given *.so file to ensure that the git version is compatible.
- import boxoffice in python
- see datastructure for more information

# :whale2: Docker -> full usage guide
A docker was made available -

```
git clone https://github.com/FelixEickeler/box_office.git
git submodule init
cd box_office/docker
```
Change the docker-compose.yml, volumes if you need to change mounts

To start the docker built  / run:
```
UUID="$(id -u)" GID="$(id -g)" docker-compose run box-office
```
This command might take a significant amount of time. So be sure to grab a coffee.

To start the docker as daemon:
```
UUID="$(id -u)" GID="$(id -g)" docker-compose up -d
docker-compose exec -u boxy box-office bash
```

You now can connect with ssh on port 2242 and the username:boxy, pw: brucelee

Do not forget to shutdown the docker after use with docker-compose down



![img.png](img.png)
## TODOS:
* write tests (c++) 
    * improve data loading     
* make example in pyhton 
* loader and dataprocessing tests in python ?!?
* cgal work around (memory leak)
 

