## 代理
- 代理设置
	- 全局：/etc/profile, ~/.bashrc
	- conda: ~/.condarc
	- pip: ~/.pip/pip.conf, ~/.config/pip/pip.conf
- `sudo pip install` 报错：sudo pip install 会报 proxy 错误，但 pip install 是 ok 的，原因是代理软件只设置了已登录这个用户的 http_proxy，没有设置超级用户的 http_proxy，因此超级用户默认还是用的 socks_proxy（而代理用的代理规则是 http，因此用 sudo pip install 会报错），分别执行 `sudo env | grep -i proxy` 和 `env | grep -i proxy` 可以看到此差别。
- 在 base 下能用 pip 代理安装，但在其他 conda 环境下无法安装。主要表现为两种报错形式：
	1. ValueError: check_hostname requires server_hostname（直接报错）
  2. ProxyError('Cannot connect to proxy.', OSError(0, 'Error'))（最开始会一直报这个警告，最后才会报一个在规定的 channel 中找不到相应的包的错误）
	这两种情况都可以通过降低 pip 版本来解决，目前发现降低到 19.3.1 是可以的。

## 其他
- conda vs. pip: 安装 python 包尽量只用 conda 而不要 conda 和 pip 混用。因为 conda 在安装包之前会检查依赖，pip 不会。二者混用可能会造成依赖不一致的问题。
- package channel: 只用 conda-forge！去掉 default 通道，也不要用国内镜像源。混用可能导致因为版本不一致等原因引发依赖问题。使用 conda-forge 的原因：<br>
  <img src=../img/conda-pip_1.png><br>
- env: 做不同的任务用不同的环境，不要把所有的包都装在 base 里面，否则 base 包太多容易产生依赖冲突。