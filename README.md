# Masumasu Crawler ROS 2

マスマスドリルの足回りを単独で動かすROS 2パッケージです。PS3等のゲームパッドによるラジコン操作と、`geometry_msgs/msg/Twist`によるプログラム操作に対応します。ドリル・昇降・傾斜・Pico制御・自動掘削は含みません。独自パッケージは **`masumasu_crawler` 1個**です。

対象: Ubuntu 22.04 / ROS 2 Humble、既存マスマス機体の左右2台のODrive（各axis0/axis1を駆動）。ODriveはモーター設定・校正済みであることが前提です。USB CDCの旧ASCIIプロトコルと`test_property`による応答照合を使います。任意のODrive機種・ファームウェアにそのまま対応する汎用ドライバではありません。ファームウェアの書き換え・校正は行いません。

## 1. インストール

ROS 2 Humbleをインストール済みの新しい端末で実行します。

```bash
git clone https://github.com/7oei/masumasu_crawler_ros2.git ~/masumasu_crawler_ros2
cd ~/masumasu_crawler_ros2
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pytest
# rosdepが未初期化の場合だけ: sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
sudo usermod -aG dialout "$USER"
```

グループ追加後はログアウト・ログインしてください。他のマスマスドリル／Armadillo制御プロセスを停止してから本パッケージを起動します。同じODriveを複数のプロセスで開かないでください。

## 2. USBの左右を固定する

標準の `right_port:=auto left_port:=auto` は左右ODriveを自動判別します。まず `/dev/odrive-right`（右）と `/dev/odrive-left`（左）を使い、固定名がなければ既知のUSBシリアル、続いてASCIIシリアルで判別します。元実装と同じく、片側が既知で残るODriveが1台だけなら残りを反対側とします。判別できなければ接続を待って再試行します。既にこれらの固定名があればルール追加は不要です。

**元のマスマス機体**には[既存機体用ルール](src/masumasu_crawler/udev/99-masumasu-crawler.rules)を同梱しています。内容を確認してから:

```bash
cd ~/masumasu_crawler_ros2
sudo install -m 644 src/masumasu_crawler/udev/99-masumasu-crawler.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
# ODriveのUSBを抜き差しする
ls -l /dev/odrive-right /dev/odrive-left
```

元機体のUSBシリアルは右`335831653536`、左`365338553030`。両方が`ODrive Robotics`と誤表示される個体では、右USBポート1・左USBポート4（`pci-0000:00:14.0`）に限定した代替ルールがあります。その状態で左右の差込口を入れ替えないでください。

**別の機体**では[テンプレート](src/masumasu_crawler/udev/99-masumasu-crawler.rules.example)の`RIGHT_SERIAL` / `LEFT_SERIAL`を実機の値に置換し、`.example`を外した名前でインストールします。値の確認例:

```bash
udevadm info --query=property --name=/dev/ttyACM0
```

固定名を使わず、起動時に明示することもできます（`ttyACM`番号は再接続で変わります）:

```bash
ros2 launch masumasu_crawler crawler.launch.py \
  right_port:=/dev/serial/by-id/YOUR_RIGHT_DEVICE \
  left_port:=/dev/serial/by-id/YOUR_LEFT_DEVICE
```

## 3. ラジコンとして使う

各端末で次を読み込みます:

```bash
source /opt/ros/humble/setup.bash
source ~/masumasu_crawler_ros2/install/setup.bash
```

```bash
ros2 launch masumasu_crawler crawler.launch.py
```

| PS3の操作 | 標準割当 | 指令 |
| --- | --- | --- |
| 左スティック上下 | `axes[1]` | 前後、最大0.47 m/s |
| 右スティック左右 | `axes[3]` | 旋回、最大0.5 rad/s |
| スティックを中立 | — | 減速して停止 |
| コントローラー通信断 | 0.5秒未受信 | ゼロ指令 |

ボタン操作による掘削・昇降・傾斜や走行モード切替はありません。標準ではボタンを押さずにスティックだけで走行します。起動対象のドライバ・Joy・teleop・odometryノードが異常終了した場合は2秒後に自動再起動します。割当はSDL／ゲームパッドに依存するため、最初は履帯を浮かせて前後・旋回方向を確認してください。

複数のコントローラーがある場合や、指定ボタンを押している間だけ走行させる場合:

```bash
ros2 launch masumasu_crawler crawler.launch.py joy_device_id:=1 enable_button:=4
ros2 topic echo /joy
```

`enable_button`は`/joy.buttons`の0始まりの番号です。`-1`は無効、0以上は押している間だけ有効で、離すと即座にゼロ指令になります。番号4は例です。自分のコントローラーで確認してください。軸番号・速度・加減速は[設定ファイル](src/masumasu_crawler/config/crawler.yaml)の`crawler_teleop`で変更します。

## 4. コマンドレベルで操作する

### モードを選ぶ

ラジコンlaunchをCtrl+Cで終了し、次で起動します:

```bash
ros2 launch masumasu_crawler crawler.launch.py mode:=command
```

このモードはジョイスティック／teleopノードを起動しません。**`/cmd_vel`を発行するのは自作プログラム1個だけ**にしてください。複数publisherの優先順位制御はありません。走行中のモード切替サービスもありません。モード変更は停止してlaunchを起動し直します。

### 端末から前進・旋回・停止

別端末で環境を読み込み、10 Hzで送ります:

```bash
# 前進0.1 m/s。Ctrl+Cで送信終了→指令タイムアウトで停止
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'

# その場で左旋回0.2 rad/s（前のpublisherを終了してから）
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.2}}'

# 前のpublisherを終了した後、ゼロを明示送信
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

`linear.x`は前進が正[m/s]、`angular.z`は左旋回が正[rad/s]。その他の成分は無視します。標準の上限は±0.47 m/s、±0.5 rad/sで、それを超える値は制限します。コマンドモードに加速度制限はありません。なめらかな加減速は発行元で実装してください。`--once`の非ゼロ指令で走り続ける設計ではなく、定期送信が必要です。

### Pythonから操作する

[動作時間を指定するサンプル](src/masumasu_crawler/examples/drive_for.py)を同梱しています:

```bash
python3 ~/masumasu_crawler_ros2/src/masumasu_crawler/examples/drive_for.py \
  --linear 0.1 --angular 0.0 --seconds 2
```

約20 Hzで2秒間指令を送り、終了時・Ctrl+C時にゼロ指令を送ります。2秒は**送信時間**です。初回のODrive有効化に時間がかかるため実走行時間・距離は保証しません。受信先が5秒以内に見つからなければエラーになります。通常のROS 2 publisherなのでC++、独自ノード、経路追従器からも同じAPIで操作できます。`TwistStamped`を使う発行元には`Twist`への変換が必要です。

別PCから使う場合は両PCで同じ`ROS_DOMAIN_ID`に設定し、同じLAN上でDDSのdiscovery・通信が通るようにします。`ROS_LOCALHOST_ONLY=1`では別PCと通信できません。このパッケージにインターネット公開用のHTTP APIや認証機構はありません。

### 外部向けインターフェース

| Topic | 型 | 意味 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 走行入力。推奨10〜20 Hz |
| `/joy` | `sensor_msgs/msg/Joy` | ラジコンモードの入力 |
| `/odom` | `nav_msgs/msg/Odometry` | エンコーダ由来の姿勢・速度。新しい計測時だけ発行 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom` → `base_footprint` |
| `/wheel_travel` | `std_msgs/msg/Float64MultiArray` | `[右累積距離m, 左累積距離m, 起動後ROS秒, 計測ROS秒]`、前進が正 |
| `/battery/voltages` | `std_msgs/msg/Float64MultiArray` | `[右電圧V, 左電圧V]` |
| `/battery/currents` | `std_msgs/msg/Float64MultiArray` | `[右バス電流A, 左バス電流A]`（回生は負） |
| `/odrive/right/feedback`, `/odrive/left/feedback` | `std_msgs/msg/Empty` | 各側の有効なエンコーダ応答の受信通知 |

通常のvolatile / reliable QoSです。`/odom_buf`は内部用で、直接操作しません。

```bash
ros2 topic echo /wheel_travel
ros2 topic echo /battery/voltages
ros2 topic hz /odom
ros2 topic info /cmd_vel --verbose
```

オドメトリは再起動で原点に戻り、履帯の滑りを補正しません。共分散は推定していません。自己位置推定や自律走行への組み込み時は、時刻・鮮度・共分散・再起動を自作側でも扱ってください。新パッケージでは旧実装で逆符号だった前進オドメトリを前進正に統一し、更新のない計測を新しい時刻で再配信しません。

## 5. 設定と停止動作

設定をコピーして調整できます:

```bash
cp ~/masumasu_crawler_ros2/src/masumasu_crawler/config/crawler.yaml ~/my_crawler.yaml
ros2 launch masumasu_crawler crawler.launch.py mode:=command config:=$HOME/my_crawler.yaml
```

| 設定（`crawler_driver.ros__parameters`） | 既定 | 内容 |
| --- | --- | --- |
| `cmd_vel_timeout` | 0.5秒 | 指令未受信で目標速度をゼロにする。単調時計使用 |
| `linear_vel_limit`, `angular_vel_limit` | 0.47, 0.5 | m/s、rad/sの上限 |
| `tread` | 0.8 m | 左右履帯間隔 |
| `reduction` | 26.4 | 減速比 |
| `drive_wheel_teeth`, `crawler_teeth` | 8, 46 | 駆動輪／履帯の歯数 |
| `crawler_perimeter` | 3.312 m | 履帯周長 |
| `yaw_calibration` | 1.875 | オドメトリ旋回補正 |
| `voltage_min`, `current_max` | 22.2 V, 100 A | 元機体から引き継いだ保護閾値 |
| `watchdog_test_enabled`, `watchdog_test_timeout` | true, 0.5秒 | 名前は旧実装由来。ODriveの実watchdogを有効化 |
| `usb_reset_after_failures` | 0（コード既定） | 0はUSB自動リセット無効。通常の再接続は継続 |

`right_port` / `left_port`のlaunch引数は設定ファイルのUSBパスより優先されます。その他の設定は起動時に読み込みます。変更後は再起動してください。

- 指令が0.5秒以上途絶えると制御周期でゼロにします。USBの計測待ち・OSスケジューリングがあり、物理停止までの厳密な時間保証ではありません。
- NaN/Infの速度はゼロ扱いにし、指令の有効期限を更新しません。
- ドライバ停止・通信断時はODrive側のwatchdogも停止を担います。初回のclosed-loop有効化直前と状態変更要求直後に、受信した速度を設定します（元の動作実績のある実装と同じ）。有効化完了後の制御周期では従来どおり指令の鮮度を確認します。
- USB不通では再接続を待ち、通信異常後はプロセスを再起動して接続し直します。**復帰後に新しい非ゼロ指令が届けば走行を再開します**。ラジコンでスティックを倒したまま、または外部publisherが送信中なら再開し得ます。
- 低電圧・過電流保護は元機体の実装を引き継いでいます。閾値は別機体では電池・モーターに合わせて検証してください。
- ゼロ指令はソフトウェア上の停止です。実機で使用する前に履帯を浮かせて方向・通信断時停止を確認し、独立した物理非常停止を使ってください。

## 6. 実機なしで確認・開発する

```bash
# USBドライバを起動しない。シミュレータではなくインターフェース確認用
ros2 launch masumasu_crawler crawler.launch.py start_driver:=false
```

単体テストと、隔離したROSドメインでの結合テスト:

```bash
cd ~/masumasu_crawler_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/masumasu_crawler/test -q
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 \
  python3 src/masumasu_crawler/test/integration_ros.py
```

結合テストは`start_driver:=false`で起動し、ゲームパッドの代わりにJoyメッセージを送ります。ドメイン187を他のロボットに使用しないでください。実機ポートは開きません。

構成:

```text
src/masumasu_crawler/
  masumasu_crawler/  ODrive通信、teleop、オドメトリ
  launch/           teleop / commandの起動
  config/           機体・操作設定
  examples/         外部Python操作例
  udev/             足回りだけのUSBルール
  test/             通信・停止・方向の単体テスト、ROS結合テスト
```

## 検証範囲・出典

ROS 2 Humbleでビルド・単体テスト・実機なしのROS結合テストを行っています。この抽出版での実機走行は未検証です。

[マスマスドリル](https://github.com/7oei/masumasu_drill_ros2)と同じローカル`armadillo_ros2_ws`の足回り実装を元に、2026-10-25時点の作業ツリーから抽出しています。元ワークスペースの基準コミットは`a005f7ec7a8d2134ef2a15f4c70b4d4efca6eb89`で、未コミットのODrive通信修正を含みます。通信処理には`air-water2026/abot04_driver`の`e34215ca8bb3a9655ea59065fba124e692fc6444`由来の応答照合・非同期計測の変更が含まれます。既存パッケージのMIT表記を引き継いでいます。

元機体の軸の駆動符号・2軸同時駆動は保持しています。ドライバの指令タイムアウト、不正値拒否、前進正のオドメトリ、独立teleopと外部操作例をこの抽出版で追加しています。ドリル用コード、ファームウェア、GNSS、Web操作、個体別バックアップ、ログ、認証情報は収録していません。

GitHub Actions用の[CIテンプレート](docs/github-actions-test.yml)も同梱しています。公開に使用した認証に`workflow`権限がないため自動実行は有効化していません。必要なら権限のある環境で`.github/workflows/test.yml`へ配置してください。
