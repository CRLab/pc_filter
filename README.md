# pc_filter
ROS node to filter a pointcloud via a pass through filter

## Parameters
```
xpassthrough/filter_limit_min: minimum x, anything below in x is filtered out
xpassthrough/filter_limit_max: max x, anything above in x is filtered out
ypassthrough/filter_limit_min: minimum y, anything below in y is filtered out
ypassthrough/filter_limit_max: max y, anything above in y is filtered out
zpassthrough/filter_limit_min: minimum z, anything below in z is filtered out
zpassthrough/filter_limit_max: max z, anything above in z is filtered out
observed_frame_id: This is the frame of reference for the original unfiltered pointcloud
filtered_frame_id: This is the frame of reference that the filtered pointcloud will be rebroadcast in
input_pc_topic: This is the ROS topic the unfiltered pointcloud is being published on.
output_pc_topic: This is the ROS topic in which the filtered pointcloud will be published. 
```

## Usage
Place the following in a launch file:
```
<launch>
   <group ns="pc_filter">
      <>
      <rosparam command="load" file="$(find pc_filter)/configs/$(arg config_file_name).workspace.yaml" />
      <node name="$(arg node_name)" pkg="pc_filter" type="server" output="screen"/>
   </group>
</launch>

```
