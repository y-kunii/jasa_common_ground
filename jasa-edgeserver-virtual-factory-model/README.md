# EdgeTech+ デモ　エッジサーバー

  本リポジトリでは、EdgeTech+でデジタルツインに環境の構築手順と動作方法を記載する。

## 動作環境

  * Linux PC
    * Ubuntu 22.04.4 LTS
    * CPU: Intel(R) Core(TM) i5XXXXX CPU @ XXXXXXX
    * Memory: 8GB
    * ソフトウェア：
      * RosProxy: Ubuntu(ROS2) humble

## インストール手順
  以下の順序で環境を構築する。
  * [Ubuntuをインストールする]()
  * [ROS2 humbleをインストールする]()
  * [RosProxyの準備 Zenohのインストール]()
  * [RosProxyの準備 RosProxyの作成]()
  * [RosProxyの実行]()
  * [Sample Programによる動作確認　SHMProxyの準備]()
  * [Sample Programによる動作確認　Sampleプログラムの実行]()
  * [カメラアプリの作成と実行]()
  * [ボタンアプリの作成と実行]()

### Ubuntuをインストールする

インストールの際は最小構成ではなく通常構成でインストールすること

### ROS2 humbleをインストールする

ROS2 hubleのインストールマニュアルを参考に[インストール](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)を行ってください。

### RosProxyの準備 Zenohのインストール
箱庭ブリッジが利用する Zenoh は、 [zenoh-c](https://github.com/eclipse-zenoh/zenoh-c) です。
なお Zenoh のインストールには[Rust環境](https://www.rust-lang.org/ja/tools/install)が必要です。

**バーチャル側とエッジ側、両方にインストールする必要があります。**

1. ディレクトリ移動

```
cd third-party 
```

2. Build
```
mkdir -p build && cd build 
cmake ../zenoh-c
cmake --build . --config Release
```

3. Install

Linux環境の場合は `sudo` が必要になることがあります。

```
cmake --build . --target install
```

成功すると、以下のファイルが作成されます。

```
% ls /usr/local/include
zenoh.h                 zenoh_commons.h         zenoh_concrete.h        zenoh_configure.h       zenoh_macros.h          zenoh_memory.h
```

```
% ls /usr/local/lib
libzenohc.dylib
```

Linuxの場合は `libzenohc.so` が生成されます。
`${LD_LIBRARY_PATH}` になければ、上記のパスを追加しておきます。

### RosProxyの準備 RosProxyの作成
RosProxyを作成するためには、最初に、箱庭PDUデータとして利用すROSメッセージ定義ファイルを準備する必要があります。ここでは、簡単のため、以下の標準ROSメッセージを利用する前提で説明をします。

* geometry_msgs/Twist
* std_msgs/Bool

1. ディレクトリ移動

```
cd hakoniwa-bridge/third-party/hakonwia-ros2pdu
```

2. 箱庭PDUデータの作成

今回は、ROS標準のメッセージを利用するため、箱庭PDUデータは既存のものを利用できます。
もし独自のROSメッセージを利用する場合は、[こちら](https://github.com/toppers/hakoniwa-ros2pdu/tree/4c658f62b8aac986f9d6571853407d892e01b5cc?tab=readme-ov-file#%E5%89%8D%E6%BA%96%E5%82%99)の手順にしたって、作成してください。


3. コンフィグファイルの作成

本ファイルは、箱庭のUnityエディタ上で`Generate`すると自動生成されるものです。Unityを利用しない場合や、利用する場合でも全てのPDUデータをエッジ側と共有しない場合等は、手動で編集する必要があります。

コンフィグファイルのファイル配置は以下としてください。

hakoniwa-ros2pdu/config/custom.json


定義例：

https://github.com/toppers/hakoniwa-digital-twin/blob/main/sim/config/custom.json


custom.jsonの定義については、[こちら](https://github.com/toppers/hakoniwa-core-cpp-client?tab=readme-ov-file#%E7%AE%B1%E5%BA%AD%E3%82%A2%E3%82%BB%E3%83%83%E3%83%88%E3%82%B3%E3%83%B3%E3%83%95%E3%82%A3%E3%82%B0)を参照ください。



なお、method_typeには、SHMを必ず指定してください。
また、class_nameとconv_class_nameは設定不要です。

以下のコマンドを実行して RosProxyのコードを生成します。

```
bash create_proxy_ros_zenoh.bash ./config/custom.json 
```

成功すると、以下のファイルが作成されます。

```
# ls workspace/src/hako_ros_proxy/src/gen/
hako_ros_proxy_com_ros2.cpp  hako_ros_proxy_com_zenoh.cpp
```

4. RosProxyのビルド

```
cd workspace
```

```
colcon build --packages-select hako_ros_proxy
```

成功すると以下のログが出力されます。

```
Starting >>> hako_ros_proxy
[Processing: hako_ros_proxy]                             
Finished <<< hako_ros_proxy [35.4s]                       

Summary: 1 package finished [36.0s]
```

### RosProxyの実行


```
source install/setup.bash 
```

```
ros2 run hako_ros_proxy hako_ros_proxy_node 
```

### Sample Programによる動作確認　SHMProxyの準備

箱庭コア機能の[インストール手順](https://github.com/toppers/hakoniwa-core-cpp-client?tab=readme-ov-file#%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB%E6%89%8B%E9%A0%86)に従って、コア機能をインストールします。

1. ディレクトリ移動

```
cd third-party/hakoniwa-core-cpp-client
```

2. Build

```
bash build.bash
```

3. Install

```
bash install.bash
```

```
cd hakoniwa-bridge/virtual
```

```
bash build.bash
```

成功すると、以下のファイルが作成されます。

```
% ls cmake-build/shm-proxy/shm-proxy 
cmake-build/shm-proxy/shm-proxy
```
ShmProxyの仕様：
```
Usage: ./cmake-build/shm-proxy/shm-proxy <asset_name> <config_path> <delta_time_msec> [master]
```

ShmProxyは、箱庭コンダクタを含んでいます。もし、箱庭コンダクタを独自に起動しない場合は、`master`オプションを利用することで、箱庭コンダクタを駆動できます。

実行例：masterオプションを利用しない場合

```
 ./cmake-build/shm-proxy/shm-proxy ShmProxy ../third-party/hakoniwa-ros2pdu/config/custom.json 20
```

実行例：masterオプションを利用する場合

```
 ./cmake-build/shm-proxy/shm-proxy ShmProxy ../third-party/hakoniwa-ros2pdu/config/custom.json 20 master
```

### Sample Programによる動作確認　Sampleプログラムの実行

### カメラアプリの作成と実行
カメラアプリをビルドする。

```
cd /home/jasa/work/hakoniwa-digital-twin/real/camera/workspace
colcon build
```

カメラアプリを実行する

```
source ./install/setup.bash 
ros2 run infra_camera camera_publish
```

**ROSを起動していない場合**
```
source /opt/ros/humble/setup.bash 
```

### ボタンアプリの作成と実行
ボタンアプリをビルドする。

```
cd /home/jasa/work/hakoniwa-digital-twin/real/button/infra_button
colcon build
```
ボタンアプリを実行する
```
source ./install/setup.bash
ros2 run infra_button button_publisher
```
**ROSを起動していない場合**
```
source /opt/ros/humble/setup.bash 
```

