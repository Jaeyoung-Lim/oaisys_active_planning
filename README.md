# oaisys_active_planning

This repository includes a package which enables demonstrations of using the photorealistic terrain simulator [OAISYS](https://github.com/DLR-RM/oaisys) interactively. The interactive mode support for oaisys is added in https://github.com/DLR-RM/oaisys/pull/11

This project includes two demonstrations:
- **Mapping**: Reconstruction of demonstration with Voxblox with predefined viewpoints
- **Active Exploration**: Using [ethz-asl/mav_active_3d_planning](https://github.com/ethz-asl/mav_active_3d_planning) to demonstrate active viewpoint planning


## Build
The project can be built using the following command.
```
catkin build oaisys_client
```

## Running The project
There are two demonstrations 

```
roslaunch oaisys_client test_mapping
```
![oaisys_mapping](https://user-images.githubusercontent.com/5248102/170708874-4f43381f-7c78-4455-bc52-5591724df448.png)


```
roslaunch oaisys_client test_planning
```
![active_mapping](https://user-images.githubusercontent.com/5248102/170708512-55ae0569-9e6d-4eb1-8231-98fa596f27a5.png)

## Additional Information
This work has been presented  presented as a workshop paper at [ICRA 2022 Workshop on Releasing Robots into the Wild: Simulations, Benchmarks, and Deployment](https://www.dynsyslab.org/releasing-robots-into-the-wild-workshop/). The paper can be found in the workshop website
