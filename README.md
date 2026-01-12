# planning
环境要求:ubuntu20.04及以上,并已安装docker。
## 一、docker环境配置
将自己的账号配置到docker组中,配置docker组方式如下:
```
sudo usermod -a -G docker $USER  #将普通用户加入到docker组中

newgrp docker  #更新docker组

sudo echo '{"insecure-registries": ["10.0.0.0/8"]}' | sudo tee /etc/docker/daemon.json

sudo systemctl reload docker
```
## 二、开发环境
### 方式一：手动获取最小环境(临时)
各平台的对应${ENV_BRANCH}如下：
```
B平台-x2b: dev/orin/b_driving_v3
C平台j6m: dev/master/j6_cnoa
C平台orinn: dev/master/orin_cnoa
C平台orinn-MNOA: dev/master/orin_mnoa

```
获取byd_adas_app, 并切换到目标分支${ENV_BRANCH}：
```
git clone -b ${ENV_BRANCH} ssh://git@10.4.35.39:2222/app_platform/byd_adas_app.git

cd byd_adas_app/
```

获取third_party, 并切换到目标分支${ENV_BRANCH}：
```
git clone -b ${ENV_BRANCH} ssh://git@10.4.35.39:2222/app_platform/third_party.git

```

获取msg, 并切换到目标分支${ENV_BRANCH}：
```
cd modules/

git clone -b ${ENV_BRANCH} ssh://git@10.4.35.39:2222/app_platform/msg.git

```

获取vehicle_configs, 并切换到目标分支${ENV_BRANCH}：
```
cd modules/

git clone -b ${ENV_BRANCH} ssh://git@10.4.35.39:2222/app_platform/vehicle_configs.git

```
### 方式二：通过git-repo方式
以 j6_cnoa 为例, 使用 manifest_dev_cnoa.xml 拉取, 该方式可以不用执行后面拉取 planning 子仓的部分
```
repo init -m manifest_cnoa.xml -b dev/master/j6_cnoa -u ssh://git@10.4.35.39:2222/app_platform/manifest.git --depth=1
repo sync -j 8 --no-clone-bundle -c -v
```

## 二、planning代码获取
### a.planning主仓代码获取
```
cd cnoa_pnc

rm -rf planning/

git clone -b dev ssh://git@10.4.35.39:2222/app_platform/pnc/cnp/cnoa_plan.git planning
```
## 三、编译
### a.进入容器
```
cd ${WORKSPACE}/byd_adas_app/      #回到byd_adas_app/目录

bash docker/scripts/cyber_start.sh -p x2b #拉取镜像并创建容器 ```-p```参数根据不同平台设置

bash docker/scripts/cyber_into.sh  #进入容器，当前目录为/apollo，并已与byd_adas_app/映射
```
### b.编译cyber_release
以下步骤均在docker容器中操作:
```
cd /apollo

git checkout modules/cnoa_pnc/planning/libs/

python3 -m dvc pull

./main.sh build_cyber
```
### c.编译planning
使用 modules/cnoa_pnc/planning/scripts/build.sh 脚本进行planning编译，支持x86、arm版本的编译
该脚本需要在docker环境中执行，使用方法可以查看usage：
```
如果涉及头文件更改或proto更新，需先clean
./main.sh clean

更新依赖
git checkout modules/cnoa_pnc/planning/libs/
python3 -m dvc pull
```
#### x86_64版本
```
./modules/cnoa_pnc/planning/scripts/build.sh -p x86 -m all -j 16
```
#### aarch64版本
```
./modules/cnoa_pnc/planning/scripts/build.sh -p arm -m all -j 16
```
## 四、运行
```
mainboard -d modules/cnoa_pnc/planning/dag/planning.dag  #本地docker中运行x86_64版本

mainboard -d modules/cnoa_pnc/planning/dag/planning_board.dag  #台架运行aarch64版本
```

## 五、提交 planning
默认对齐分支是 dev 分支，所有planning仓库提交 MR 前必须先rebase dev 分支。

