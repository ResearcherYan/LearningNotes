## 代理
- 代理设置
	- 全局：/etc/profile, ~/.bashrc
	- conda: ~/.condarc
	- pip: ~/.pip/pip.conf, ~/.config/pip/pip.conf
- `sudo pip install` 报错：sudo pip install会报proxy错误，但pip install是ok的，原因是代理软件只设置了已登录这个用户的http_proxy，没有设置超级用户的http_proxy，因此超级用户默认还是用的socks_proxy（而代理用的代理规则是http，因此用sudo pip install会报错），分别执行`sudo env | grep -i proxy`和`env | grep -i proxy`可以看到此差别。
- 在base下能用pip代理安装，但在其他conda环境下无法安装。主要表现为两种报错形式：
	1. ValueError: check_hostname requires server_hostname（直接报错）
  2. ProxyError('Cannot connect to proxy.', OSError(0, 'Error'))（最开始会一直报这个警告，最后才会报一个在规定的channel中找不到相应的包的错误）
	这两种情况都可以通过降低pip版本来解决，目前发现降低到19.3.1是可以的。

## 其他
- conda vs. pip: 安装python包尽量只用conda而不要conda和pip混用。因为conda在安装包之前会检查依赖，pip不会。二者混用可能会造成依赖不一致的问题。
- package channel: 只用conda-forge！去掉default通道，也不要用国内镜像源。混用可能导致因为版本不一致等原因引发依赖问题。使用conda-forge的原因：<br/>
  <img src=img/conda-pip_1.png><br/>
- env: 做不同的任务用不同的环境，不要把所有的包都装在base里面，否则base包太多容易产生依赖冲突。