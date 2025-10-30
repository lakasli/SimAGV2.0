# 错误代码分类整理

## 电池与电源警告

| 错误代码 | 错误级别 | 描述                         | 报警含义               | 常见报警原因           |
| -------- | -------- | ---------------------------- | ---------------------- | ---------------------- |
| 50401    | Fatal    | battery too low and shutdown | 机器人电量过低自动关机 | 机器人电量过低         |
| 52503    | Fatal    | battery is too low to move   | 机器人电量过低         | 电池电量低于模型配置值 |
| 54211    | Warning  | low battery                  | 电量低                 | 电量低                 |

## 路径规划错误

| 错误代码 | 错误级别 | 描述                         | 报警含义               | 常见报警原因                                           |
| -------- | -------- | ---------------------------- | ---------------------- | ------------------------------------------------------ |
| 52700    | Fatal    | can not find a feasible path | 找不到可通行的路径     | agv在固定路径导航过程中无法找到通行线路                |
| 52701    | Fatal    | can not find target id       | 找不到目标点           | 下发的任务指令的id没有包含在地图中                     |
| 52702    | Fatal    | path plan failed             | 路径规划失败           | 绕行或自由导航路径规划失败                             |
| 52705    | Fatal    | motion planning failed       | 运动规划失败           | 多种运动规划相关问题                                   |
| 52708    | Fatal    | destination has obstacles    | 终点处有障碍物         | 自由导航时，终点处有障碍物                             |
| 52715    | Fatal    | current pos has obstacles    | 机器人当前位置有障碍物 | 自由导航时，agv当前位置有障碍物                        |
| 52201    | Fatal    | robot out of path            | 机器人在线路外         | agv在线路外，距离线路的距离大于参数配置中的OutPathDist |

## 导航相关警告

| 错误代码 | 错误级别 | 描述                      | 报警含义         |
| -------- | -------- | ------------------------- | ---------------- |
| 54231    | Warning  | Caution: robot is blocked | 注意机器人被阻挡 |

## 地图加载与解析
| 错误代码 | 错误级别 | 描述                      | 报警含义       |
| -------- | -------- | ------------------------- | -------------- |
| 50100    | Fatal    | map parse error           | 地图解析错误   |
| 50101    | Fatal    | map load error            | 地图加载错误   |
| 50102    | Fatal    | map is too large          | 地图面积过大   |
| 50103    | Fatal    | map is empty              | 地图数据为空   |
| 50104    | Fatal    | map meta error            | 地图元数据错误 |
| 50105    | Fatal    | map resolution is illegal | 地图分辨率非法 |
| 50106    | Fatal    | map format invalid        | 地图格式非法   |

## 地图站点与点位
| 错误代码 | 错误级别 | 描述                                        | 报警含义                     | 常见报警原因                                         |
| -------- | -------- | ------------------------------------------- | ---------------------------- | ---------------------------------------------------- |
| 52015    | Fatal    | stations with the same id number in the map | 地图站点中有相同的id         | 更新地图时可能会出现                                 |
| 52107    | Fatal    | Switch map error in current station point   | 切换的地图中不存在对应的站点 | 在SM点切换地图，但是当前待切换的地图中没有发送的站点 |

## 通用

| 错误代码 | 错误级别 | 描述                                                         |
| -------- | -------- | ------------------------------------------------------------ |
| 90000    | warning  | any - 任意错误                                               |
| 90001    | warning  | operationMode is not AUTOMATIC - 操作模式不是自动模式        |
| 90002    | warning  | send Order to rbk Failed - 向rbk发送订单失败                 |
| 90003    | warning  | new OrderId But NotLock - 收到新订单ID但未锁定               |
| 90004    | warning  | edge.endNodeId != NodeId - 边的结束节点ID与当前节点ID不匹配  |
| 90005    | warning  | edge.startNodeId != NodeId - 边的起始节点ID与当前节点ID不匹配 |
| 90006    | warning  | newOrderId rec,But Order is Running - 收到新订单ID，但订单正在运行中 |
| 90007    | warning  | node Or Edge is empty - 节点或边为空                         |
| 90008    | warning  | orderUpdateId is lower than last one,msg pass - 订单更新ID低于上一个，消息跳过 |
| 90009    | warning  | orderUpdateId is == than last one,msg pass - 订单更新ID与上一个相同，消息跳过 |
| 90010    | warning  | try to create Order Failed - 尝试创建订单失败                |
| 90011    | warning  | new node base error - 新建节点基础错误                       |
| 90012    | warning  | order's nodePosition not in map - 订单的节点位置不在地图中   |
| 90013    | warning  | actionPackEmpty - 动作包为空                                 |
| 90014    | warning  | nonOrderCancel - 非订单取消                                  |