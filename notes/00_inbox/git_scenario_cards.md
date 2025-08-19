# Git 场景处理卡片

### [[git]]
- Q: 有一个包含多个子项目的目录，需要初始化为Git项目，如何处理？
- A: 使用`git init`初始化，然后选择性添加项目级别文件：`git add README.md CLAUDE.md Dockerfile docker-compose.yml manage.sh myProj/`，最后提交。选择性添加比`git add .`更安全。

- Q: 本地新建仓库要连接已有的GitHub远程仓库，如何处理历史冲突？
- A: 使用`git remote add origin <url>`添加远程仓库，然后`git pull origin master --allow-unrelated-histories`合并无关历史。如果有文件丢失，用`git checkout HEAD -- <文件名>`恢复。

- Q: 需要了解现有子项目的Git配置情况，使用什么命令？
- A: 使用`git status`检查状态，`git remote -v`查看远程配置，`git branch -a`查看分支，`pwd && ls -la`确认目录结构。分析现状是制定Git策略的第一步。

- Q: 如何将fork的项目与原项目保持同步？
- A: 使用`git fetch upstream`获取原项目更新，`git checkout master`切换主分支，`git merge upstream/master`合并更新，`git push origin master`推送到fork。upstream用于获取更新，origin用于推送修改。

- Q: 如何将多个独立项目集成到主项目中并保持独立性？
- A: 先移除现有目录，然后用`git submodule add <url> <path>`添加为submodule，最后提交配置。submodule可以在主项目中引用其他项目，保持各自独立性。

- Q: 如何确认submodule配置是否正确工作？
- A: 使用`git submodule status`查看状态，`cat .gitmodules`查看配置，`ls -la`确认目录结构。submodule status显示commit hash和分支信息，用于验证状态。

- Q: 如何为submodule项目设置upstream和创建学习分支？
- A: 在每个子项目中使用`git remote add upstream <原项目url>`添加上游，`git checkout -b readcode`创建学习分支，`git push origin readcode`推送到fork。为学习目的创建专门分支，避免污染主分支。

- Q: pull操作后文件被标记为deleted，如何恢复？
- A: 使用`git status`查看状态，`git checkout HEAD -- <文件名>`从最新commit恢复文件，`cat <文件名>`验证内容。此命令可以从最新提交中恢复文件。

- Q: 如何对多个子项目执行相同的Git操作？
- A: 方法1使用`cd`切换目录逐个操作，方法2使用绝对路径避免路径问题。批量操作时注意路径问题，必要时使用绝对路径。

- Q: 创建了新分支，如何首次推送到远程仓库？
- A: 使用`git checkout -b <分支名>`创建并切换，`git push origin <分支名>`首次推送。新分支首次推送会在远程创建同名分支。

- Q: 如何设计合适的Git管理架构来支持学习和开发？
- A: 先用`git remote -v`、`git branch`、`git submodule status`分析现状，然后创建主项目，添加submodule，设置upstream。选择架构需要考虑项目特点和使用需求。

- Q: 在不同目录执行命令时出现"目录不存在"错误，如何解决？
- A: 使用`pwd`检查当前目录，`ls -la`查看内容，使用绝对路径`cd /full/path/to/target`解决。路径问题是Git操作中常见错误，善用pwd和绝对路径。