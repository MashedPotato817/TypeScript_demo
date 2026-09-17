# 第二优先级优化：解析几何雅可比、零空间投影避限位与可操作度分析方案

在第一优先级完成课程实验预设与局部 DH 坐标系可视化的基础上，本阶段针对**算法核心与机器人学理论深度**进行升级。

---

## 用户审查与确认 (User Review Required)

> [!IMPORTANT]
> - **解析几何雅可比替代有限差分**：遵循参考教材 Craig《机器人学导论》第 5 章，通过各连杆瞬时转轴 $z_{i-1}$ 与位置矢量直接解析合成 $J(q)$，彻底消除 $\epsilon=10^{-4}$ 数值微商的截断误差与 8 次 FK 计算开销，求解速度提升 5~10 倍。
> - **8 自由度零空间投影 (Null-Space Projection)**：利用 $8 - 3$ 或 $8 - 6$ 的多余自由度，在不影响末端主任务精度的前提下，引入二次优化目标函数：自发将各关节拉向行程中值区间，有效消除关节死卡限位（Joint Limit Avoidance）。
> - **可操作度 (Manipulability) 实时遥测**：引入 Yoshikawa 可操作度指标 $w = \sqrt{\det(J J^T)}$，在遥测面板实时展示机械臂灵巧度与奇异性预警。

---

## 拟定修改内容 (Proposed Changes)

### 1. 核心运动学与雅可比计算升级

#### [MODIFY] [robot.ts](file:///c:/Users/Mashed%20Potato/Desktop/260916机器人技术基础(王振)/project/TypeScript_demo/src/robot.ts)
- **解析几何雅可比函数 `geometricJacobian(q: number[])`**：
  - 基于一次正向运动学得到的 `transforms` 与 `points`；
  - 转动关节：$J_{vi} = \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1})$，$\mathbf{J}_{\omega i} = \mathbf{z}_{i-1}$；
  - 移动关节（第3关节）：$J_{vi} = \mathbf{z}_{i-1}$，$\mathbf{J}_{\omega i} = \mathbf{0}$；
  - 输出 $3 \times 8$（位置）和 $6 \times 8$（位姿）精确解析雅可比。
- **重构 `solvePositionIK` 与 `solvePoseIK`**：
  - 使用解析几何雅可比替代循环微扰；
  - 引入**零空间投影算子**：
    $$\Delta \mathbf{q} = J^{\dagger} \mathbf{e} + \alpha (I - J^{\dagger} J) \nabla H(\mathbf{q})$$
    其中 $H(\mathbf{q}) = -\sum \left(\frac{q_i - q_{i,\text{mid}}}{q_{i,\text{span}}}\right)^2$；
- **新增可操作度测度函数 `computeManipulability(q: number[])`**：
  - 计算 $w = \sqrt{\det(J_v J_v^T)}$，表征末端运动灵巧度。

---

### 2. 遥测面板与交互体验增强

#### [MODIFY] [main.ts](file:///c:/Users/Mashed%20Potato/Desktop/260916机器人技术基础(王振)/project/TypeScript_demo/src/main.ts)
- 在“末端遥测”区增加：
  - **可操作度灵巧度指标** $w$（带有“灵巧”、“一般”、“接近奇异”状态色块）；
  - 零空间优化状态指示。
- `refresh()` 中同步更新可操作度数值。

#### [MODIFY] [style.css](file:///c:/Users/Mashed%20Potato/Desktop/260916机器人技术基础(王振)/project/TypeScript_demo/src/style.css)
- 增加遥测指标行（可操作度小卡片与奇异警示颜色）。

---

### 3. 自动化测试扩充

#### [MODIFY] [robot.test.ts](file:///c:/Users/Mashed%20Potato/Desktop/260916机器人技术基础(王振)/project/TypeScript_demo/src/robot.test.ts)
- 增加**解析雅可比 vs 数值雅可比精度对比测试**，验证两者在任意随机位形下的最大误差小于 $10^{-5}$；
- 增加**零空间避限位测试**，验证在目标收敛的同时关节角度自适应向中心区间回拢；
- 验证收敛速度与迭代次数对比。

---

## 验证计划 (Verification Plan)

### 自动化测试
```powershell
cd 'C:\Users\Mashed Potato\Desktop\260916机器人技术基础(王振)\project\TypeScript_demo'
npm run test
npm run build
```

### 手工交互验证
1. 打开本地页面，滑动关节到极端位置，输入远端目标，点击“求解当前位置”，观察机械臂是否更平滑、且更少卡在极限死角；
2. 观察右侧“末端遥测”面板中新增的“可操作度”数值，随着机械臂舒展或折叠，数值呈现合理物理变化（伸直或奇异时降至接近 0，适度弯曲时达到峰值）。
