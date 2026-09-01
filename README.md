# motion_tracer_ros2セットアップ

## 1.各種PKGインストール
1. [seed_robot_ros2_pkg](https://github.com/thkrrc1/seed_robot_ros2_pkg)、 [noid_lifter_mover](https://github.com/thkrrc1/noid_lifter_mover.git)のREADMEに従ってプロジェクトをクローンする。

2. 上記リポジトリseed_robot_ros2_pkgと同ワークスペースにて、motion_tracer_ros2をクローンした上でビルドする。

**※テレオペレーションを行うフォロワーデバイス（ex. SEED-Noid-Lifter-Mover）PCとリーダーデバイス（ex. SEED-Tracer）PC、両方で上記のセットアップをしてください。**

## 2.udev設定
リーダーデバイスPCにリーダーデバイスのみを接続した状態で、下記コマンド等でリーダーデバイス情報を取得する。
```
udevadm info /dev/ttyUSB0
```
```
・出力例
E: ID_BUS=usb
E: ID_MODEL=SEED-CM4U-A
E: ID_MODEL_ENC=SEED-CM4U-A
E: ID_MODEL_ID=a1e8
E: ID_SERIAL=THK_SEED-CM4U-A_000000000030
E: ID_SERIAL_SHORT=000000000030
E: ID_VENDOR=THK
E: ID_VENDOR_ENC=THK
E: ID_VENDOR_ID=0483
```
udevファイルを作成します。
```
sudo -E gedit /etc/udev/rules.d/91-tracer.rules
```

下記を記入する。（idVendor、idProduct、serialに上記で取得した情報を記入）
```
SUBSYSTEM=="tty",ATTRS{idVendor}=="0483",ATTRS{idProduct}=="a1e8",ATTRS{serial}=="000000000030",MODE="666",SYMLINK+="tracer_usb", RUN+="/bin/setserial /dev/tracer_usb low_latency"
```

## 3.テレオペレーションの実行
フォロワーデバイスPC、リーダーデバイスPCが同一ネットワーク上に接続された状態で下記を実行する。
1. フォロワーデバイス関連のプログラム実行　**※フォロワーデバイスPCにて実行**  
    ```
    $ ros2 launch motion_tracer_ros2 robot_bringup.launch.py simulation:=false
    ```

2. リーダーデバイス関連のプログラム実行　**※リーダーデバイスPCにて実行**  
    ```
    $ ros2 launch motion_tracer_ros2 tracer_bringup.launch.py
    ```
