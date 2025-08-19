# 今天使用的Git命令总结

## 基础仓库操作

### 1. 检查Git状态
```bash
git status
```
**作用**: 查看当前仓库状态，包括修改的文件、未跟踪的文件等
**今天用途**: 检查当前目录是否是Git仓库，查看文件状态

### 2. 查看远程仓库
```bash
git remote -v
```
**作用**: 显示所有远程仓库的名称和URL
**今天用途**: 检查子项目的origin和upstream配置是否正确

### 3. 查看分支
```bash
git branch -a
```
**作用**: 显示所有分支（本地和远程）
**今天用途**: 确认当前所在分支和可用的远程分支

## 初始化和远程仓库操作

### 4. 初始化Git仓库
```bash
git init
```
**作用**: 在当前目录创建一个新的Git仓库
**今天用途**: 在主项目根目录初始化Git仓库

### 5. 添加远程仓库
```bash
git remote add origin git@github.com:用户名/仓库名.git
git remote add upstream git@github.com:原作者/仓库名.git
```
**作用**: 添加远程仓库链接
**今天用途**: 为主项目添加origin，为子项目添加upstream原始仓库

### 6. 拉取远程内容
```bash
git pull origin master --allow-unrelated-histories
```
**作用**: 从远程仓库拉取并合并到当前分支
**`--allow-unrelated-histories`**: 允许合并无关联历史的分支
**今天用途**: 拉取远程仓库的.gitignore和LICENSE文件

## 文件管理操作

### 7. 添加文件到暂存区
```bash
git add README.md CLAUDE.md Dockerfile docker-compose.yml manage.sh myProj/
git add .  # 添加所有文件
```
**作用**: 将文件添加到暂存区，准备提交
**今天用途**: 将项目级别的文件添加到Git跟踪

### 8. 提交更改
```bash
git commit -m "提交信息"
```
**作用**: 将暂存区的更改提交到本地仓库
**今天用途**: 提交初始项目文件和submodule配置

### 9. 推送到远程仓库
```bash
git push origin master
git push origin readcode
```
**作用**: 将本地提交推送到远程仓库
**今天用途**: 推送主项目更改和新创建的readcode分支

## Submodule相关操作

### 10. 添加Submodule
```bash
git submodule add git@github.com:用户名/仓库名.git 目录名
```
**作用**: 将外部仓库作为submodule添加到当前项目
**今天用途**: 将Btraj、DecompROS、Fast-Planner作为submodule添加

### 11. 查看Submodule状态
```bash
git submodule status
```
**作用**: 显示所有submodule的当前状态和commit信息
**今天用途**: 确认submodule配置正确

## 分支操作

### 12. 创建并切换分支
```bash
git checkout -b 分支名
```
**作用**: 创建新分支并立即切换到该分支
**今天用途**: 为每个子项目创建readcode学习分支

### 13. 切换分支
```bash
git checkout 分支名
```
**作用**: 切换到指定分支
**今天用途**: 切换到master分支进行操作

### 14. 获取远程分支信息
```bash
git fetch upstream
```
**作用**: 从upstream远程仓库获取最新信息（不自动合并）
**今天用途**: 获取原始项目的最新状态

### 15. 合并分支
```bash
git merge upstream/master
```
**作用**: 将指定分支的更改合并到当前分支
**今天用途**: 将upstream的最新更改合并到本地master分支

## 文件操作

### 16. 恢复文件
```bash
git checkout HEAD -- .gitignore
```
**作用**: 从最近的提交中恢复指定文件
**今天用途**: 恢复被意外删除的.gitignore文件

## 配置文件

### 17. 查看.gitmodules
```bash
cat .gitmodules
```
**作用**: 查看submodule配置文件内容
**今天用途**: 确认submodule配置是否正确

## 总结
今天主要完成了：
1. **主项目初始化**: 创建统一的学习项目仓库
2. **Submodule管理**: 将三个算法项目作为submodule集成
3. **Fork策略**: 保持子项目的独立性，可以自由修改
4. **分支管理**: 为每个项目创建专门的学习分支
5. **远程仓库配置**: 设置origin（你的fork）和upstream（原项目）