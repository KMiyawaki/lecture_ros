# コマンドで動くロボット

[README](./README.md)

---

## 実習

```shell
$ roscd beginner_tutorials
$ mkdir scripts
$ cd scripts
$ pwd
/home/[user name]/catkin_ws/src/beginner_tutorials/scripts
```

- 次のファイルを`scripts`にダウンロードし実行しなさい。
  - [recv_commands.py](https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/recv_commands/recv_commands.py)

```shell
$ wget https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/recv_commands/recv_commands.py
・・・
2020-10-28 12:01:06 (2.69 MB/s) - recv_commands.py’ saved [1785/1785]

$ chmod u+x recv_commands.py
$ ls -l
・・・
-rwxr--r-- 1 [user name] [user name] 1785 Oct 28 12:01 recv_commands.py
```

### 実行順序

- まず、[シミュレータを起動する](../stage_simulator/stage_simulator_01.md)
- しばらくしてから`recv_commands.py`を実行する。

```shell
$ rosrun beginner_tutorials recv_commands.py
[INFO] [1640765754.038343, 0.000000]: C19XXX 工大 太郎
[INFO] [1640765754.298499, 1103.000000]: The server comes up
[INFO] [1640765754.304402, 1103.000000]: Executing check_command
# 一旦ここで出力は止まる。
```

- さらに別ターミナルで次のコマンドを実行する。

```shell
$ rostopic pub -1 /chatter std_msgs/String "data: 'go_forward_1.0'"
publishing and latching message for 3.0 seconds
```

- すると、最初のコマンドを実行したターミナルで次のように表示される。

```shell
・・・
[INFO] [1640765805.493627, 1154.175000]: Recv command go_forward_1.0
[INFO] [1640765805.497791, 1154.200000]: Executing check_command
[INFO] [1640765805.531208, 1154.200000]: Recv command go_forward_1.0
[INFO] [1640765805.535580, 1154.250000]: Executing check_command
[INFO] [1640765805.560089, 1154.275000]: Recv command go_forward_1.0
[INFO] [1640765805.563202, 1154.275000]: Executing check_command
・・・
```

## 問題(1)

実行して分かる通り、`recv_commands.py`はターミナルで実行した`rostopic pub`コマンドの引数`go_forward_1.0`を出力している。

- トピック名とその型は何か？

## 問題(2)

受信したコマンドに従って、実際にロボットを動作させなさい。コマンドは次の通り。

決められた距離の前進・後退、決められた角度の旋回は時間決めの方法でも構わない。

- go_forward_1.0：1.0m進む。
- go_back_1.0：1.0m後退する。
- turn_left_90：90度回転する。
- turn_right_90：-90度回転する。
- p1：座標（7, 4）に自律移動する。到着後の方向は任意。
- p2：座標（4, 4）に自律移動する。到着後の方向は任意。
- p3：座標（3.5, 2）に自律移動する。到着後の方向は任意。

## 問題(3)

さらに以下のコマンドを実装しなさい。

- wait_door：正面の障害物が無くなったら1.0m進む。
- wait_blue：青い物体を認識したら1.0m進む。
- wait_red：赤の物体を認識したら1.0m後退する。
- wait_green：緑の物体を認識したら右に90度回転する。
- wait_yellow：黄色の物体を認識したら左に90度回転する。
