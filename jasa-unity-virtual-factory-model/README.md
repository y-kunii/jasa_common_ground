### 前提条件
1. Unity 2022.3.8f1をインストール済みであること
2. SHMProxyが実行できる状態である（このURLを参照　https://github.com/toppers/hakoniwa-bridge/tree/main ）

### 実施手順
リポジトリ https://github.com/y-kunii/jasa-unity-virtual-factory-model/tree/Fix-EdgeTech-2024
    1. git clone https://github.com/y-kunii/jasa-unity-virtual-factory-model.git
    2. git switch Fix-EdgeTech-2024
    3. Unity Hubより「plugin-srcs」フォルダでUnityを立ち上げる
![alt text](image/image.png)
![alt text](image/SelectFolder.png)
    4. Hakoniwa.unityシーンを選択し、ダブルクリックでロードする（path：plugin-srcs/Assets/2024EdgeTech/Scene/Hakoniwa.unity）
![alt text](image/SceneLoad.png)
    5. Window/Hakoniwa/GenerateDebugよりcustom.jsonを生成。
    このcustom.jsonは下記リンクの「３．コンフィグファイルの作成」時も必要となります。
    
https://github.com/y-kunii/jasa_common_ground/tree/main/jasa-edgeserver-virtual-factory-model#rosproxy%E3%81%AE%E6%BA%96%E5%82%99-rosproxy%E3%81%AE%E4%BD%9C%E6%88%90

![alt text](image/CustomJsonGenerate.png)
    6. ５で生成したcustom.jsonを用いてShmProxyを立ち上げる
![alt text](image/Setup_hakoniwaUnity.png)
    7. Unityのシミュレーション開始ボタンを押下
![alt text](image/play.png)
    8. GameViewのSTARTボタンを押下し、開始する
![alt text](image/appStart.png)


### 操作方法
- ↑：前進 
- ↓：後進 
- →：右旋回 
- ←：左旋回 
- スペースキー：停止