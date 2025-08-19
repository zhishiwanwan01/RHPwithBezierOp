# Git 场景处理卡片

## 场景卡片1：项目初始化
**遇到的场景**: 有一个包含多个子项目的目录，需要初始化为Git项目
**使用的命令**:
```bash
git init                    # 初始化Git仓库
git add README.md CLAUDE.md Dockerfile docker-compose.yml manage.sh myProj/
git commit -m "Initialize main project with documentation, configuration and myProj directory"
```
**处理结果**: 成功创建主项目仓库并提交项目级别文件
**经验总结**: 初始化时选择性添加文件比`git add .`更安全

---

## 场景卡片2：连接远程仓库并处理历史冲突
**遇到的场景**: 本地新建仓库要连接已有的GitHub远程仓库
**使用的命令**:
```bash
git remote add origin git@github.com:zhishiwanwan01/RHPwithBezierOp.git
git pull origin master --allow-unrelated-histories
git checkout HEAD -- .gitignore    # 恢复意外删除的文件
```
**处理结果**: 成功合并本地和远程仓库，获得远程的.gitignore和LICENSE
**经验总结**: `--allow-unrelated-histories`用于合并无关历史的仓库

---

## 场景卡片3：检查项目Git状态
**遇到的场景**: 需要了解现有子项目的Git配置情况
**使用的命令**:
```bash
git status                  # 检查当前仓库状态
git remote -v              # 查看远程仓库配置
git branch -a               # 查看所有分支
pwd && ls -la              # 确认目录结构
```
**处理结果**: 发现每个子项目都有独立的Git仓库，指向原始开源项目
**经验总结**: 分析现状是制定Git策略的第一步

---

## 场景卡片4：同步Fork与原项目
**遇到的场景**: 需要将fork的项目与原项目保持同步
**使用的命令**:
```bash
cd Btraj
git fetch upstream                    # 获取原项目最新信息
git checkout master                   # 切换到主分支
git merge upstream/master            # 合并原项目更新
git push origin master              # 推送到自己的fork
```
**处理结果**: 所有fork项目都同步到最新版本
**经验总结**: upstream用于获取原项目更新，origin用于推送自己的修改

---

## 场景卡片5：管理Submodule
**遇到的场景**: 想要将多个独立项目集成到一个主项目中，保持独立性
**使用的命令**:
```bash
# 先移除现有目录
rm -rf Btraj DecompROS Fast-Planner

# 添加为submodule
git submodule add git@github.com:zhishiwanwan01/Btraj.git Btraj
git submodule add git@github.com:zhishiwanwan01/DecompROS.git DecompROS  
git submodule add git@github.com:zhishiwanwan01/Fast-Planner.git Fast-Planner

# 提交submodule配置
git commit -m "Add submodules for Btraj, DecompROS and Fast-Planner"
git push origin master
```
**处理结果**: 成功将三个项目作为submodule集成，创建了.gitmodules文件
**经验总结**: submodule可以在主项目中引用其他项目，保持各自独立性

---

## 场景卡片6：验证Submodule配置
**遇到的场景**: 需要确认submodule配置是否正确工作
**使用的命令**:
```bash
git submodule status                 # 查看submodule状态
cat .gitmodules                     # 查看配置文件
ls -la                              # 确认目录结构
```
**处理结果**: 确认所有submodule都正常，指向正确的仓库和commit
**经验总结**: submodule status显示commit hash和分支信息，用于验证状态

---

## 场景卡片7：为子项目配置学习环境
**遇到的场景**: 需要为每个submodule项目设置upstream和创建学习分支
**使用的命令**:
```bash
# 为每个项目添加upstream
cd Btraj
git remote add upstream git@github.com:HKUST-Aerial-Robotics/Btraj.git
git checkout -b readcode            # 创建学习分支
git push origin readcode           # 推送到fork

cd ../DecompROS  
git remote add upstream git@github.com:sikang/DecompROS.git
git checkout -b readcode
git push origin readcode

cd ../Fast-Planner
git remote add upstream git@github.com:HKUST-Aerial-Robotics/Fast-Planner.git
git checkout -b readcode
git push origin readcode
```
**处理结果**: 每个项目都有了upstream源和专门的学习分支
**经验总结**: 为学习目的创建专门分支，避免污染主分支

---

## 场景卡片8：解决文件丢失问题
**遇到的场景**: pull操作后发现.gitignore文件被标记为deleted
**使用的命令**:
```bash
git status                          # 查看问题状态
git checkout HEAD -- .gitignore    # 从最新commit恢复文件
cat .gitignore                      # 验证文件内容
```
**处理结果**: 成功恢复.gitignore文件
**经验总结**: `git checkout HEAD -- 文件名`可以从最新提交中恢复文件

---

## 场景卡片9：批量操作多个项目
**遇到的场景**: 需要对三个子项目执行相同的Git操作
**使用的命令**:
```bash
# 方法1：使用cd切换目录
cd Btraj && git remote -v
cd ../DecompROS && git remote -v  
cd ../Fast-Planner && git remote -v

# 方法2：使用绝对路径（当相对路径有问题时）
cd /home/zsww/leosNuc15/02_project/learnPNCProj_2025-08/Btraj && git fetch upstream
```
**处理结果**: 成功对所有项目执行相同操作
**经验总结**: 批量操作时注意路径问题，必要时使用绝对路径

---

## 场景卡片10：推送新分支到远程
**遇到的场景**: 创建了新分支，需要首次推送到远程仓库
**使用的命令**:
```bash
git checkout -b readcode            # 创建并切换到新分支
git push origin readcode           # 首次推送新分支
```
**处理结果**: 在远程仓库创建了新分支，GitHub提示可以创建PR
**经验总结**: 新分支首次推送会在远程创建同名分支

---

## 场景卡片11：项目架构规划
**遇到的场景**: 需要设计合适的Git管理架构来支持学习和开发
**使用的命令**:
```bash
# 分析现状
git remote -v                       # 检查各项目的远程配置
git branch                          # 检查分支情况
git submodule status                # 检查submodule状态

# 实施架构
git init                            # 创建主项目
git submodule add <url> <path>      # 集成子项目
git remote add upstream <url>       # 设置上游源
```
**处理结果**: 建立了Fork + Submodule的混合架构
**经验总结**: 选择合适的Git架构需要考虑项目特点和使用需求

---

## 场景卡片12：处理命令执行错误
**遇到的场景**: 在不同目录执行命令时出现"目录不存在"错误
**使用的命令**:
```bash
pwd                                 # 检查当前目录
ls -la                              # 查看目录内容
cd /full/path/to/target            # 使用绝对路径
```
**处理结果**: 通过确认路径和使用绝对路径解决问题
**经验总结**: 路径问题是Git操作中常见错误，善用pwd和绝对路径

---

## 使用建议

### 如何使用这些场景卡片：
1. **遇到类似场景时**，先查找对应卡片
2. **参考命令序列**，但根据实际情况调整
3. **学习经验总结**，避免重复犯错
4. **定期复习**，加深对Git工作流的理解

### 场景卡片的价值：
- **实用性强**：都是真实遇到的问题和解决方案
- **命令组合**：展示了命令的实际使用组合
- **问题导向**：从解决实际问题的角度学习Git
- **经验沉淀**：每个场景都有经验总结

### 扩展学习：
- 每次遇到新的Git场景，记录成新的卡片
- 对比不同场景下的命令差异
- 理解每个命令在不同场景中的作用