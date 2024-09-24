# tros_perception_fusion

## 功能描述

订阅多个`ai_msgs::msg::PerceptionTargets`类型的topic，经过时间对齐后，将多个目标合并为一个目标发布。

支持使用service接口动态修改用于融合的topic。

## 运行

### 独立进程启动

```bash
ros2 run tros_ai_fusion tros_ai_fusion --ros-args -p topic_name_base:=/tros_dnn_detection -p "topic_names_fusion:=['/tros_det', '/tros_seg']" -p pub_fusion_topic_name:=/tros_perc
```

### composition方式启动

以launch启动方式举例，将节点添加到composition中：

```python
ComposableNode(
    package='tros_ai_fusion',
    plugin='tros::TrosAiMsgFusionNode',
    name='tros_perc_fusion_node',
    parameters=[
        {'topic_name_base': '/tros_dnn_detection'},
        {'topic_names_fusion': ['/tros_det', '/tros_seg']},
        {'pub_fusion_topic_name': '/tros_perc'},
    ],
    extra_arguments=[{'use_intra_process_comms': True}],
)
```

## 动态修改订阅topic

通过service请求动态修改用于融合的topic，service使用的消息类型为[TopicManage.srv](./tros_ai_fusion_msgs/srv/TopicManage.srv)。

1. 查询用于融合的topic：

```bash
ros2 service call /tros_topic_manage tros_ai_fusion_msgs/srv/TopicManage "{action: 'get'}"
```

2. 增加用于融合的topic：

```bash
ros2 service call /tros_topic_manage tros_ai_fusion_msgs/srv/TopicManage "{action: 'add', topics: {'/tros_det', '/tros_seg'}}"
```

3. 删除用于融合的topic：

```bash
ros2 service call /tros_topic_manage tros_ai_fusion_msgs/srv/TopicManage "{action: 'delete', topics: {'/tros_det'}}"
```
