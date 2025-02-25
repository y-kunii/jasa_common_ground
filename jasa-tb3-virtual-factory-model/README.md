# Turtlebot3セットアップ手順書

## はじめに
JASAコモングラウンド委員会デジタルツインデモにおけるTurtlebot3のセットアップ手順書(備忘録)です。各設定値やバージョンは2024年11月のEdge Tech時点でのものとなります。

## 事前準備
- リモートPC ※Ubuntu+ROS2セットアップ済
  - OS：Ubuntu20.04.2 LTS
  - ROS：ROS2 Foxy
- TurtleBot3 burger (Raspberry Pi 3B+) 本体 ※組み立て済
- バッテリー（デモ時は有線接続）
- microSDカード（16GB以上あれば問題無し）
- キーボード
- モニター+HDMIケーブル
- Wi-Fi環境

## セットアップ手順
1. **Turtlebot3用のイメージファイルをダウンロード【リモートPC】**
   - Turtlebot3のラズパイのバージョンを確認
     ```bash
     cat /proc/device-tree/model
     ```
   - 公式サイトからラズパイのバージョンに対応するイメージファイルをダウンロード
     [公式サイト](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)


2. **microSDカードにイメージファイルを書き込み【リモートPC】**
   - Raspberry Pi Imager等でイメージファイルをmicroSDに書き込む。
     - Raspberry Pi Imagerをインストール
       *プライバシー保護のため、外部リンクが削除されました。*
     - microSDに書き込み


3. **microSDカードのパーティションサイズを変更【リモートPC】**
   - パーティションツールをインストール
     [GParted](https://gparted.org/download.php)
   - パーティションサイズを変更


4. **microSDカードをTurtlebot3に差し込み、Turtlebot3を起動【Turtlebot3】**
   - Turtlebot3起動時にHDMIでモニターに接続しておくこと
     - ID：ubuntu
     - PW：turtlebot


5. **キーボード配列を変更【Turtlebot3】**
   ```bash
   sudo dpkg-reconfigure keyboard-configuration
   ```
   - Generic 105-key(Intl) PC → Japanese → Japanese → The default for the keyboard layout → No compose key
   - 再起動


6. **Wi-Fi設定【Turtlebot3】**
   ```bash
   cd /etc/netplan/
   sudo vi 50-cloud-init.yaml
   ```
   - 『access-points:』の下の行に繋げたいWi-FiのSSIDを記入
   - さらにその下の行に『password:<パスワードを記入>』を記入して保存し終了
   ```bash
   reboot
   ```
   ```bash
   ifconfig
   ```
   - 設定したWi-Fiに繋がっている事を確認する
   - ifconfigが入っていない場合はsudo apt install net-toolsを実行

   
7. **OpenCRのセットアップ【Turtlebot3】**
   ```bash
   sudo dpkg --add-architecture armhf
   sudo apt-get update
   sudo apt-get install libc6:armhf
   export OPENCR_PORT=/dev/ttyACM0
   export OPENCR_MODEL=burger_noetic
   rm -rf ./opencr_update.tar.bz2
   wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2
   tar -xvf opencr_update.tar.bz2
   cd ~/opencr_update
   ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
   ```


8. **Lidarの設定【Turtlebot3】**
   ```bash
   echo 'export LDS_MODEL=LDS-01' >> ~./bashrc
   source ~/.bashrc
   ```


9. **ROS_HOSTNAME、ROS_MASTER_URIの設定【リモートPC/Turtlebot3】**
   - リモートPCとTurtlebot3を同一ネットワーク上に配置
   ```bash
   sudo vi ~/.bashrc
   ```
   - 以下の内容を追記
     - ROS_MASTER_URI=http://<マスター側のIPアドレス>:11311
     - ROS_HOSTNAME=<Turtlebot、リモートPCのIPアドレス>


10. **ROSトピックのリマッピング**
[参考リンク](https://github.com/toppers/hakoniwa-digital-twin/blob/main/real/robot/tb3/robot.launch.py)
   - 参考リンクを参照し、リマッピングの設定を行う。
   - robot.launch.pyの格納先は
     - /home/ubuntu/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup/launch
   - ※通常、Turtlebot3を操作するROSトピックは『cmd_vel』だが、箱庭デジタルツインが発行するROSトピックで操作できるようにするため。

11. **Turtlebot3に接続、操作【リモートPC】**
   - リモートPCとTurtlebot3を同一ネットワーク上に配置
   - リモートPCからTurtlebot3に接続
     - ssh ubuntu@<Turtlebot3のIPアドレス>
     - PW：turtlebot
   - Turtlebot3を起動
   ```bash
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
   - ROSトピックで操作(別のプロンプトを立ち上げて改めてSSH接続し実施)
   - Turtlebot3を前進させる場合はlinearのXを、回転させる場合はangularのZを変更。値は0.1～-0.1で様子を見て調整する。
   ```bash
   ros2 topic pub /RobotAvator_cmd_pos geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```
