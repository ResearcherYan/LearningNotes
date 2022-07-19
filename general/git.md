# Git Learning notes
> Git是一种分布式版本控制系统，于2005年由Linus创建

- [Git Learning notes](#git-learning-notes)
- [Git Start-up](#git-start-up)
  - [设置与取消代理](#设置与取消代理)
  - [创建本地版本库，并与远程库同步](#创建本地版本库并与远程库同步)
  - [将远程库克隆到本地](#将远程库克隆到本地)
- [Git Basics](#git-basics)
  - [Basic Commands](#basic-commands)
  - [Basic Operations](#basic-operations)
    - [版本回退](#版本回退)
    - [撤销修改](#撤销修改)
    - [删除文件](#删除文件)
  - [分支管理](#分支管理)
    - [创建与合并分支](#创建与合并分支)
    - [解决冲突](#解决冲突)
    - [储存工作现场](#储存工作现场)
    - [多人协作](#多人协作)
    - [其余命令](#其余命令)
  - [其他操作](#其他操作)
    - [标签管理](#标签管理)
    - [.gitignore](#gitignore)
    - [为git命令配置别名](#为git命令配置别名)

附：git基础命令表 [git-cheat-sheet.pdf](../img/git-cheat-sheet.pdf)

# Git Start-up
## 设置与取消代理

```
# 设置http代理
git config --global http.proxy http://127.0.0.1:port
git config --global https.proxy https://127.0.0.1:port
# 取消http代理
git config --global --unset http.proxy
git config --global --unset https.proxy
```
如果没有设置代理，`git push`到github上面的时候可能会出现如下的error
```
fatal: unable to access 'https://<repo url>: gnutls_handshake() failed: The TLS connection was non-properly terminated.
```

## 创建本地版本库，并与远程库同步
1. 进入到某个空目录，并把该目录变成git可以管理的仓库。此后，该目录就变成了一个git工作区（Working Directory）。
```
cd learngit
git init
```
2. 工作区有一个隐藏目录.git，这是git的版本库（Repository），版本库中有暂存区（stage/index）和一个master分支，以及一个指向master的指针HEAD。

<img src=../img/repository.jpeg>

通过两步把工作区的文件添加到git版本库中
```
git add README.md  # 把文件更改添加到暂存区
git commit -m "add README.md"  # 把暂存区的所有内容提交到当前分支
```
使用`git add .`可直接把当前目录所有文件更改都添加到暂存区。

1. 在github上创建一个空仓库，创建好之后根据github的提示运行相应的指令，把本地库的所有内容推送到远程库上。
```
git remote add origin https://github.com/ReseacherYan/learngit.git
git branch -M master
git push -u origin master
```
由于远程库是空的，我们第一次推送master分支时，加上了-u参数，Git不但会把本地的master分支内容推送的远程新的master分支，还会把本地的master分支和远程的master分支关联起来，在以后的推送或者拉取时就可以简化命令，直接使用`git push origin master`即可。  

如果把远程库的地址写错了，需要删除远程库信息。
```
$ git remote -v  # 首先查看远程库信息
origin https://github.com/ReseacherYan/learngit.git (fetch)
origin https://github.com/ReseacherYan/learngit.git (push)
$ git remote rm origin  # 然后删除orign这个远程库
```

## 将远程库克隆到本地
进入一个本地文件夹，基于https协议将远程库克隆到本地。
```
cd githubProjects
git clone https://github.com/ReseacherYan/learngit.git
```
# Git Basics
## Basic Commands
```
git init  # 初始化本地仓库
git add test.txt  # 将单个文件更改添加到暂存区
git add .  # 将当前目录所有文件更改添加到暂存区
git commit -m "commit message"  # 将暂存区的文件更改提交到当前分支
git push origin master  # 将本地master分支的文件修改push到远程段的master分支

git status  # 查看当前仓库状态
git diff  # 查看不同版本文件的差异
git log  # 查看提交日志的历史记录
git reflog  # 获取所有历史命令
git remote  # 查看远程库信息
git remote -v  # 查看远程库更详细的信息
```
## Basic Operations
### 版本回退
`HEAD`表示当前版本，上一个版本就是`HEAD^`，上上一个版本就是`HEAD^^`，往上100个版本写成`HEAD~100`。
```
git reset --hard HEAD^  # 回退到上个版本
git reset --hard 1094a  # 回到版本1094a，1094a为某个版本ID的前几位数
```
### 撤销修改
```
git checkout -- test.txt  # 撤销文件在工作区的修改，让文件回到最后一次git commit或git add时的状态
git reset HEAD test.txt  # 把暂存区的修改撤销掉（unstage），重新放回工作区
git restore --staged test.txt  # 撤销暂存操作
```
### 删除文件
如果要删除某个文件，在删除完工作区中的该文件后，还需要使用`git rm`删除版本库中的该文件，并且`git commit`。
```
rm test.txt
git rm test.txt
git commit -m "remove test.txt"
```
如果是误删了某个文件，可以使用`git checkout -- test.txt`恢复。

## 分支管理
### 创建与合并分支
```
git branch  # 查看分支
git branch <branch-name>  # 创建分支
git checkout <branch-name>  # 切换分支
git switch <branch-name>  # 切换分支
git checkout -b <branch-name>  # 创建+切换分支
git switch -c <branch-name>  # 创建+切换分支
git merge <branch-name>  # 合并某分支到当前分支(HEAD指向的分支)
git branch -d <branch-name>  # 删除分支
```
合并分支有两种模式。第一种模式为`Fast forward`，命令为`git merge <name>`，在这种模式下，删除分支后，会丢掉分支信息。第二种模式为普通模式，命令为`git merge --no-ff -m <message> <name>`，这样在merge时会生成一个新的commit，如此一来，从分支历史上就可以看出分支信息。

<img src=../img/branches.png>

### 解决冲突
若`git merge`时出现冲突，修改冲突文件的内容，重新`git add`和`git commit`

### 储存工作现场
有时候需要放下当前branch的工作，切换到另一个branch，这时就需要把当前branch的工作现场给储存起来，等另一个branch的工作完成后再恢复。
```
git stash  # 储存当前分支的工作现场
git stash list  # 查看stash内容
git stash apply  # 恢复工作现场
git stash drop  # 删除stash内容
git stash pop  # 恢复工作现场并删除stash内容
```

### 多人协作
常见的多人协作的工作模式通常是：
1. 首先，可以试图用`git push origin <branch-name>`推送自己的修改。
2. 如果推送失败，则因为远程分支比你的本地更新，需要先用`git pull`试图合并。如果`git pull`提示`no tracking information`，则说明本地分支和远程分支的链接关系没有创建，用命令`git branch --set-upstream-to <branch-name> origin/<branch-name>`。
3. 如果合并有冲突，则解决冲突，并在本地提交。
4. 没有冲突或者解决掉冲突后，再用`git push origin <branch-name>`推送就能成功！

### 其余命令
- 复制一个特定的提交到当前分支
```
$ git branch
* dev
  master
$ git cherry-pick 4c805e2  # 4c805e2为dev分支最后一次commit的ID的前几位数
[master 1d4b803] fix bug 101
 1 file changed, 1 insertion(+), 1 deletion(-)
```
- 强行删除分支
```
git branch -D <branch-name>  # 对于还未合并的分支，需要这样删除
```

## 其他操作
### 标签管理
标签总是和某个commit挂钩。如果这个commit既出现在master分支，又出现在dev分支，那么在这两个分支上都可以看到这个标签。
```
# 创建标签
git tag <tag-name>  # 新建一个标签，默认为HEAD，也可以指定一个commit id
git tag -a <tag-name> -m <message>  # 创建标签时指定标签信息
git tag  # 查看所有标签
git show <tag-name>  # 查看标签信息

# 操作标签
git push origin <tag-name>  # 推送一个本地标签
git push origin --tags  # 推送全部未推送过的本地标签
git tag -d <tag-name>  # 删除一个本地标签
git push origin :refs/tags/<tag-name>  # 删除一个远程标签
```

### .gitignore
.gitignore文件用于告诉git忽略哪些文件，让这些这些文件只存在于工作区，无法传到暂存区或远程库。示例如下：
```
# 排除所有.开头的隐藏文件:
.*
# 排除所有.class文件:
*.class

# 不排除.gitignore和App.class:
!.gitignore
!App.class
```

### 为git命令配置别名
配置Git的时候，加上--global是针对当前用户起作用的，如果不加，那只针对当前的仓库起作用。每个本地仓库的Git配置文件都放在.git/config文件中。
```
git config --global alias.unstage 'reset HEAD'  # 用git unstage撤销暂存区的修改并放回工作区
git config --global alias.last 'log -1'  # 用git last显示最后一次提交信息
```
