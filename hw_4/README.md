## hw4 lattice规划
- 根据加速度采样前向推理一段距离
- 求解推理后末状态到目标的obvp
  - 利用ceres求解最优时间ot
  - 根据ot求解最优轨迹方程
  - 求解对应的cost