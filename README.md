# 車輪型倒立振子の PID 制御と ACT 模倣学習

Physical AI 講座「応用1」最終課題。Gazebo と ROS 2 で差動二輪ロボットを車輪型倒立振子に改造し、PID 系の状態フィードバック制御で倒立させたうえで、その記録データから ACT（Action Chunking with Transformers）を模倣学習させて制御を置き換える試みを行いました。

- 動画：<[YouTube の URL](https://youtu.be/ciCFZFjH4Fg)>

## 結果の概要

| 制御 | 結果 |
| --- | --- |
| PID 系の状態フィードバック | 約 10° 傾けた状態から立て直し、元の位置付近で立ち続ける |
| ACT（模倣学習） | 前進して立て直そうとするが、行き過ぎて後ろに倒れ、約 20° 傾いて接地する（3 回とも同じ） |

ACT の失敗は、シミュレーションの実時間比を 1.0、0.1、0.01 と下げて推論の遅れを小さくしても変わらなかったため、推論の遅れは主な原因ではないと判断しました。学習データの範囲外の状態に入ったこと（分布シフト）や、更新周期（PID 200 Hz、ACT 50 Hz）の違いが原因として考えられます。詳しくはレポートを参照してください。

## リポジトリの構成

```
.
├── src/
│   ├── nav_dev/          倒立振子のモデル（models/world.sdf）と launch ファイル
│   ├── pend_control/     PID 系の制御ノード（balance_node）
│   └── key_event/        キー入力のパッケージ（サブモジュール）
├── scripts/
│   ├── bag_to_npz.py     ROSBAG を学習用の配列（npz）に変換
│   └── act_node.py       ACT の推論ノード
├── training/
│   ├── pend_features.py      LeRobot データセットの定義
│   ├── npz2lerobot.py        npz を LeRobot v2.1 形式のデータセットに変換
│   └── pend_train_config.yaml ACT の学習設定
├── bags/                 PID で記録した ROSBAG（pend_run11〜20）と ACT の試行の記録
└── dataset_npz/          bag_to_npz.py で変換したデータ
```

学習済みモデル（約 160 MB）は、GitHub の容量制限のため、[Releases](https://github.com/nick-curiosity151/pai-inverted-pendulum/releases) で配布しています。

## 開発環境

| 項目 | 内容 |
| --- | --- |
| ホスト | Windows 11、Docker Desktop（WSL 2） |
| コンテナイメージ | `airobotbook/ros2-desktop-ai-robot-book-humble`（Ubuntu 22.04） |
| ROS 2 | Humble |
| シミュレータ | Gazebo Fortress（ign gazebo 6.16.0）、ros_gz |
| 学習環境 | [matsuolab/pai_training](https://github.com/matsuolab/pai_training)（コミット `b4d6b54`）、LeRobot（コミット `768e366`、データセット形式 v2.1） |

GPU は使わず、CPU のみで動作を確認しています。

## 環境構築

### 1. リポジトリを取得してコンテナを起動する

サブモジュール（key_event）も一緒に取得するため、`--recursive` を付けて clone します。

```powershell
git clone --recursive https://github.com/nick-curiosity151/pai-inverted-pendulum.git
cd pai-inverted-pendulum
docker run -d --name pai_pend_ros2 -p 6081:80 --shm-size=1g -v "${PWD}:/workspace" airobotbook/ros2-desktop-ai-robot-book-humble:latest
```

ブラウザで http://localhost:6081 を開き、デスクトップのターミナルで以降の作業を行います。

### 2. Gazebo と ROS 2 の連携パッケージを入れる

```bash
sudo apt update
sudo apt install -y ros-humble-ros-gz xterm git
```

`apt update` で ROS 2 のリポジトリの署名鍵の期限切れ（`EXPKEYSIG`）が出る場合は、鍵を更新してから再実行します。

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

### 3. ビルドする

```bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 4. 学習環境を作る（ACT を使う場合）

講義の学習環境を、学習に使ったのと同じコミットで取得し、ロックファイルどおりにインストールします。

```bash
curl -fsSL https://pixi.sh/install.sh | bash
source ~/.bashrc
cd ~
git clone https://github.com/matsuolab/pai_training
cd pai_training
git checkout b4d6b5459066a80433475a83567504988875248a
git submodule update --init --recursive
pixi install --locked
```

**注意**：`pixi install` は必ず `--locked` を付けてください。付けずに依存関係が解き直されると、PyAV がソースからのビルドになり、FFmpeg 7 が必要になって失敗します（Gazebo を入れたコンテナでは、Ubuntu 側の FFmpeg 4.4 とも干渉します）。以降、pixi のコマンドは `pixi run --no-install` で実行します。

## 実行方法

ターミナルを開くたびに、最初に次を実行します。

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

### PID 制御で倒立させる

| ターミナル | コマンド |
| --- | --- |
| 1：シミュレーション | `ros2 launch nav_dev nav_teleop.launch.py` |
| 2：IMU のブリッジ | `ros2 run ros_gz_bridge parameter_bridge /imu@sensor_msgs/msg/Imu[ignition.msgs.IMU` |
| 3：制御ノード | `ros2 run pend_control balance_node --ros-args -p kv:=1.5 -p kx:=0.5 -p pitch_target:=-0.18` |

1. ターミナル 1 で launch を起動したら、**xterm（teleop_node）のウィンドウを閉じる**（キー入力がなくても 10 Hz で停止指令を出し、制御に割り込むため）
2. ターミナル 2、3 を起動する
3. Gazebo の ▶ を押すと、約 10° 傾いた状態から立て直す

止めるときは、各ターミナルで `Ctrl + C` を押します（`Ctrl + Z` は使わない）。

### ROSBAG を記録する

PID で動かす前に、別のターミナルで記録を始め、`Subscribed to topic '/imu'` と `'/cmd_vel'` の両方が表示されてから ▶ を押します。1 本ごとに launch と balance_node を起動し直してください。

```bash
cd /workspace/bags
ros2 bag record -o pend_runXX /imu /cmd_vel /rosout
```

### データセットを作る

```bash
# ROSBAG → npz（ROS 2 の Python で実行）
python3 /workspace/scripts/bag_to_npz.py /workspace/bags /workspace/dataset_npz -0.18 20 3 1.5 0.5

# npz → LeRobot データセット（pixi の環境で実行、50 Hz に間引く）
cd ~/pai_training
pixi run --no-install python /workspace/training/npz2lerobot.py
```

`bag_to_npz.py` の最後の 4 つの数値（kp、kd、kv、kx）は、記録したときのゲインです。再現した観測を PID の式に入れ直して検算し、誤差を表示します。

### ACT を学習する

講義の `train.py` をそのまま使い、設定ファイルだけを差し替えます。

```bash
cd ~/pai_training
cp configs/train_config.yaml configs/train_config_crane.yaml
cp /workspace/training/pend_train_config.yaml configs/train_config.yaml
WANDB_MODE=offline pixi run --no-install train
```

このコミットの wandb（0.19.11）は、現在の W&B にログインできない場合があるため、`WANDB_MODE=offline` で実行します。CPU で約 1 時間かかります。学習済みモデルは `/home/ubuntu/checkpoints/train/pend_balance_act` に保存されます。

### ACT で制御する

学習済みモデルを [Releases](../../releases) からダウンロードし、`models/pend_balance_act/`（`config.json` と `model.safetensors`）に置きます。自分で学習した場合は、そのフォルダを引数で指定します。

PID のときのターミナル 3 の代わりに、次を実行します（balance_node は起動しない）。

```bash
cd ~/pai_training
pixi run --no-install python /workspace/scripts/act_node.py
# 自分で学習したモデルを使う場合
pixi run --no-install python /workspace/scripts/act_node.py /home/ubuntu/checkpoints/train/pend_balance_act
```

`ACT モデルを読み込みました` と表示されてから、Gazebo の ▶ を押します。

## 主な設定

| 項目 | 値 |
| --- | --- |
| 車輪 | 半径 0.08 m、質量 0.3 kg、車体の重心の真下（x = 0.1） |
| マスト | 長さ 0.4 m、先端に 0.5 kg の重り |
| IMU | マストに搭載、200 Hz |
| 初期姿勢 | ピッチ 0.18 rad（約 10°）、z 軸まわりに 90° 回転 |
| PID のゲイン | Kp = 20、Kd = 3、Kv = 1.5、Kx = 0.5 |
| ACT の観測 | ピッチ角、ピッチ角速度、直前の速度指令、移動量（4 次元） |
| ACT の行動 | 前後の速度指令、旋回の速度指令（常に 0）（2 次元） |
| ACT の学習 | 50 Hz、chunk_size 10、n_action_steps 1、batch_size 64、3000 ステップ |

## ライセンスと出典

このリポジトリは [Apache License 2.0](LICENSE) で公開しています。

- `src/nav_dev/` は、[OsawaKousei/2DRoboPrac_ws](https://github.com/OsawaKousei/2DRoboPrac_ws)（Apache License 2.0）の `ros2_ws/src/nav_dev` をもとに改造したものです。主な変更点は、`models/world.sdf` の倒立振子化（車輪の大径化、キャスターの撤去、車輪の移動、マストと IMU の追加、初期姿勢の変更）と、`launch/nav_teleop.launch.py` のシミュレーションの自動再生の無効化です。元のリポジトリから引き継いだ、今回使っていないファイル（地図、ナビゲーション用の launch、別のロボットのモデルなど）も含まれています。
- `src/key_event/` は、[nek009/key_event](https://github.com/nek009/key_event) をサブモジュールとして参照しています。
- 学習には [matsuolab/pai_training](https://github.com/matsuolab/pai_training) と [huggingface/lerobot](https://github.com/huggingface/lerobot) を使用しています。
- ロボットの改造と ROS 2 との連携は、Qiita の記事「[gazeboとROS2を連携してロボットを操作する](https://qiita.com/N622/items/9d89f77d85d9da0af29e)」を参考にしました。
