# tros_perception_render

## 功能描述

订阅`ai_msgs::msg::PerceptionTargets`类型的感知结果topic，和`sensor_msgs::msg::CompressedImage`类型的`jpeg`编码图像topic，经过时间对齐后，将感知结果渲染到图像上，并发布渲染后的图像。

TODO (20241010): 完善功能描述

## 运行

### 独立进程启动

```bash
ros2 run tros_perception_render tros_perception_render --ros-args -p perception_topic_name:=/tros_perc_fusion -p img_topic_name:=/image_jpeg -p pub_render_topic_name:=/tros_render_img
```

### composition方式启动

以launch启动方式举例，将节点添加到composition中：

```python
ComposableNode(
    package='tros_perception_render',
    plugin='tros::TrosPerceptionRenderNode',
    name='tros_perc_fusion_node',
    parameters=[
        {'perception_topic_name': '/tros_perc_fusion'},
        {'img_topic_name': '/image_jpeg'},
        {'pub_render_topic_name': '/tros_render_img'},
    ],
    extra_arguments=[{'use_intra_process_comms': True}],
)
```
