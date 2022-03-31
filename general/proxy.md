> CONTENT
- [Proxy settings](#proxy-settings)
  - [设置环境变量](#设置环境变量)
  - [下载时指定 proxy](#下载时指定-proxy)
- [Trouble Shooting](#trouble-shooting)
  - [http 可用 https 不可用](#http-可用-https-不可用)

## Proxy settings
Linux 走代理，总的来说有两种方式。<br>
第一种，也是最常用的一种，就是通过**设置环境变量**的方式来设置；<br>
第二种，是在安装或者下载文件的时候通过类似 `--proxy` 这种附加命令来指定下载该文件时使用什么代理。
### 设置环境变量
使用 http 代理的话，环境变量主要有两个：`http_proxy` 和 `https_proxy`。这两个环境变量最好都设置成 `http://127.0.0.1:port`（原因参考 [http 可用 https 不可用](#http-可用-https-不可用) ）
- 修改开机自启文件（推荐）
  - `~/.bashrc` & `/etc/profile`
    ```
    export http_proxy="http://127.0.0.1:port"
    export https_proxy="http://127.0.0.1:port"
    ```
    修改这两个文件之一应该就可以实现当前用户的代理设置，通过命令 `env | grep -i proxy` 可以看到。
  - `/etc/sudoers`（参考 [github gist](https://gist.github.com/hindol/4483374)）
    ```
    Defaults	env_keep+="http_proxy ftp_proxy all_proxy https_proxy no_proxy"
    ```
    先用 `sudo -i gedit /etc/sudoers` 打开该文件，在最后一个 *Defaults* 后加入上面这行。<br>
    配置好后可以实现超级用户的代理设置，通过命令 `sudo env | grep -i proxy` 可以看到。（如果未即时生效可试试 `sudo -i source /etc/sudoers` 使设置生效）

- 命令行
  ```bash
  export http_proxy="http://127.0.0.1:port"
  export https_proxy="http://127.0.0.1:port"
  ```
  命令行是一次性设置，在重启之后环境变量仍会改为默认值。

### 下载时指定 proxy
- `curl` 指定代理
  ```bash
  curl https://reqbin.com/echo -x myproxy.com:8080 -U login:password
  ```
  其中 `-x` 可以换成 `--proxy`，如果没有用户名密码就不用 `-U` 了。<br>
  这里面要注意如果是用 http 代理的话，`-x` 后面不要写成 `https://127.0.0.1:port` 的形式，否则会报 *curl: (35) OpenSSL SSL_connect: SSL_ERROR_SYSCALL in connection* 的错误，应该是用 `http://127.0.0.1:port` 或 直接写成 `127.0.0.1:port`。

## Trouble Shooting
### http 可用 https 不可用
在使用 http 代理时经常出现 **http 可用而 https 不可用** 的情况，因此最好在设置环境变量的时候就都用 http，不要用 https。
- Cases
  - VSCode settings 中的 *Http: Proxy* 如果设置成 `https://127.0.0.1:port` 的话，扩展商店就无法使用，会报 *XHR failed* 的错误。
  - Curl 如果使用 `-x https://127.0.0.1:port` 的话，就会报错 *curl: (35) OpenSSL SSL_connect: SSL_ERROR_SYSCALL in connection*
  - VSCode Remote Container 如果在 *Dockerfile* 中使用了 https 源，在 *devcontainer.json* 中设置了 https 代理，但没有设置 `"http.proxyStrictSSL": false` 的话，就可能会报错 *Certificate verification failed: The certificate is NOT trusted.*
- 原因分析<br>
  参考 [HTTP 与 HTTPS 的区别](https://www.runoob.com/w3cnote/http-vs-https.html)，原因简单来说可能是 https(HyperText Transfer Protocol Secure) 传输协议比 http(HyperText Transfer Protocol) 更严格。https 会通过 SSL/TLS 对数据包进行加密，而 http 则直接是以明文方式发送数据。<br>
  因此，猜测可能是因为走代理没法过 SSL/TLS 数据加密这一关。