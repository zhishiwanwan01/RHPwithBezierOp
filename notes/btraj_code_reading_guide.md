# Btraj 代码阅读 Git 操作指南

## 开始前的准备

### 1. 确认当前状态
```bash
cd Btraj
git status
git branch
```
**确认**: 你应该在`readcode`分支上

### 2. 如果不在readcode分支
```bash
git checkout readcode
```

## 代码阅读工作流

### 阶段一：浏览和理解代码结构

#### 1. 开始阅读前记录
```bash
git commit --allow-empty -m "开始阅读Btraj代码 - $(date '+%Y-%m-%d')"
```
**作用**: 创建时间点标记，记录学习开始时间

#### 2. 添加文件结构注释
当你理解了整体架构后：
```bash
# 查看修改状态
git status

# 添加修改的文件
git add include/trajectory_generator.h src/trajectory_generator.cpp

# 提交架构理解
git commit -m "添加trajectory_generator架构理解注释"
```

### 阶段二：深入理解算法

#### 3. 分模块提交理解
每理解一个重要模块就提交：
```bash
# 理解Fast Marching算法
git add src/a_star.cpp include/a_star.h
git commit -m "理解A*和Fast Marching路径规划算法原理"

# 理解Bezier曲线优化
git add src/bezier_base.cpp include/bezier_base.h
git commit -m "理解Bezier曲线轨迹优化数学原理"

# 理解主要数据结构
git add include/data_type.h
git commit -m "理解主要数据结构定义和类型"
```

#### 4. 添加学习笔记文件
```bash
# 创建学习笔记
echo "# Btraj 学习笔记" > learning_notes.md

# 添加并提交
git add learning_notes.md
git commit -m "创建Btraj学习笔记文档"
```

### 阶段三：实验和测试理解

#### 5. 保存实验性修改
```bash
# 创建实验分支
git checkout -b readcode-experiments

# 进行实验性修改...
git add .
git commit -m "实验：修改Bezier曲线阶数测试效果"

# 回到主学习分支
git checkout readcode
```

#### 6. 合并有价值的实验结果
```bash
# 只合并特定文件的修改
git checkout readcode-experiments -- learning_notes.md
git add learning_notes.md
git commit -m "合并实验结果到学习笔记"
```

## 推送和同步

### 7. 定期推送学习进度
```bash
# 推送到你的fork
git push origin readcode

# 如果是第一次推送新分支
git push -u origin readcode-experiments
```

### 8. 同步上游更新（可选）
```bash
# 获取原项目最新更新
git fetch upstream

# 查看是否有新更新
git log readcode..upstream/master --oneline

# 如果需要合并更新（小心，可能覆盖你的注释）
git merge upstream/master
```

## 实用技巧

### 9. 查看修改历史
```bash
# 查看学习提交历史
git log --oneline --since="2025-08-19"

# 查看某个文件的修改历史
git log -p include/trajectory_generator.h
```

### 10. 对比版本
```bash
# 对比当前版本与原始版本
git diff HEAD~5 include/trajectory_generator.h

# 对比两个提交
git diff <commit1> <commit2> -- src/trajectory_generator.cpp
```

### 11. 暂存工作进度
```bash
# 临时保存当前工作
git stash -u -m "临时保存trajectory_generator注释工作"

# 恢复工作
git stash pop
```

## 建议的注释添加方式

### 头文件注释模板
```cpp
/**
 * @file trajectory_generator.h
 * @brief Bezier曲线轨迹生成器
 * @details 学习笔记：
 * - 使用Bernstein基函数构建Bezier曲线
 * - 通过QP求解器优化轨迹参数
 * - 支持动力学约束和障碍物约束
 * @date 2025-08-19 学习日期
 */
```

### 函数注释模板
```cpp
/**
 * @brief 生成Bezier轨迹
 * @param waypoints 路径点
 * @param corridor 安全走廊
 * @return 优化后的轨迹
 * 
 * 学习理解：
 * 1. 首先构建Bezier基函数矩阵
 * 2. 设置目标函数（最小化加速度）
 * 3. 添加约束条件（位置、速度、加速度）
 * 4. 调用Mosek求解QP问题
 */
```

## 提交信息规范

### 好的提交信息示例
```bash
git commit -m "理解trajectory_generator核心算法

- 添加Bezier曲线构建过程注释
- 解释QP求解器参数设置
- 记录约束条件的物理意义
- 标注关键数学公式的含义"
```

### 按功能模块提交
```bash
git commit -m "路径规划模块：A*算法实现理解"
git commit -m "轨迹优化模块：Bezier曲线数学基础"
git commit -m "数据结构模块：主要类和接口说明"
git commit -m "配置参数模块：launch文件参数含义"
```

## 学习里程碑检查

### 每天结束时
```bash
# 查看今天的学习成果
git log --since="1 day ago" --oneline

# 推送今天的进度
git push origin readcode

# 更新主项目的submodule引用
cd ..
git add Btraj
git commit -m "更新Btraj学习进度 - $(date '+%Y-%m-%d')"
git push origin master
```

## 常见问题解决

### 如果意外修改了不想改的文件
```bash
# 恢复单个文件
git checkout HEAD -- filename

# 恢复所有修改
git reset --hard HEAD
```

### 如果提交信息写错了
```bash
# 修改最近一次提交信息
git commit --amend -m "新的提交信息"
```

### 如果需要回到某个历史状态
```bash
# 查看提交历史
git log --oneline

# 创建新分支保存当前状态
git branch backup-readcode

# 回到指定提交
git reset --hard <commit-hash>
```

## 学习建议

1. **每天至少一次提交**：记录学习进度
2. **模块化理解**：按功能模块分别提交
3. **实验要谨慎**：使用独立分支进行实验
4. **定期推送**：避免丢失学习成果
5. **写详细注释**：帮助future self理解代码

记住：Git是你的学习助手，不要怕犯错，可以随时回退！