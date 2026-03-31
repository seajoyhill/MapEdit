## 编译

``` bash
cmake --build build/
```
## 使用说明
### 命令行

``` bash
./build/align_map
./build/align_routes
```
- align_map将align_map的CornerMap.pcd和SurfMap.pcd合并到base_map的CornerMap.pcd和SurfMap.pcd上，并保存到final_map的CornerMap.pcd和SurfMap.pcd中

- align_routes将align_map的routes.yaml和arm_points.yaml合并到base_map的routes.yaml和arm_points.yaml上，并保存到final_map的routes.yaml和arm_points.yaml中

### yaml文件参数说明
1. `base_map`代表基地图，最后以它的世界坐标系作为新的世界坐标系，所以它的pose.txt、trajectory.pcd、routes.yaml都不会被修改;
2. `align_map`待合并地图，也就是要将`align_map`合并到`base_map`上，它的pose.txt、trajectory.pcd、routes.yaml都要乘以坐标变换转换到base_map上去，然后和base_map一起生成final_map;
3. `base_match_roi_enable=true`时，base_match_roi生效，代表在原始地图上长方体中的地图用来匹配;
4. `base_match_use_ros_coord=true` 长方体的长宽高用ROS坐标系规则，这个在用看图软件比如CloudCompare看地图时，需要自己手动转换成ROS，不是很方便，**可以改成false，在地图上选择点，直接填点的坐标，更便捷**.