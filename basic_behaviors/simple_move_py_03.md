# 指定した距離・角度だけ動く（自己位置推定） (Python)

[基本的な動作](./Home.md)

---

## 自己位置推定とは

- ロボットがあらかじめ環境の地図を持っている場合、レーザレンジファインダの観測結果と地図を照らし合わせることで自分の位置を把握することができる。
- これを自己位置推定（Localization）という。
- ここでは、この自己位置推定により得られたロボットの位置・姿勢を使ってロボットが進んだ距離を計算し、一定距離だけ進むプログラムを作成する。

## 実習

```shell
$ roscd beginner_tutorials/scripts
$ pwd
/home/[user name]/catkin_ws/src/beginner_tutorials/scripts
```

- 次のファイルを`scripts`に保存しなさい。実行権限の付与を忘れないように。
  - [simple_move_with_localization.py](https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/basic_behaviors/simple_move/simple_move_with_localization.py)

```shell
$ wget https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/basic_behaviors/simple_move/simple_move_with_localization.py
・・・
2020-10-28 12:01:06 (2.69 MB/s) - ‘simple_move_with_localization.py’ saved [1785/1785]

$ chmod u+x simple_move_with_localization.py
$ ls -l
・・・
-rwxr--r-- 1 [user name] [user name] 1785 Oct 28 12:01 simple_move_with_localization.py
```

### 実行順序

- まず、[シミュレータを起動する](../stage_simulator/stage_simulator_01.md)
- しばらくしてから`simple_move_with_localization.py`を実行する。

```shell
$ rosrun beginner_tutorials simple_move_with_localization.py
[INFO] [1635294934.204416, 0.000000]: C19XXX ロボット　太郎
[INFO] [1635294935.242251, 9.875000]: Executing go_straight_by_distance_with_localization
[INFO] [1635294936.231659, 10.875000]: Recv localized pose. (x, y, theta) = (8.03, 0.50, 90.22)
[INFO] [1635294936.353328, 11.000000]: Recv localized pose. (x, y, theta) = (8.03, 0.51, 90.22)
```

- 起動したら、コマンドターミナルの画面をよく観察すること。ロボットが２秒間直進して停止する。その間自己位置推定による座標情報が出力されるはずである。

## 課題

### 課題（１）

- `go_straight_by_distance_with_localization`関数を完成させなさい。
  - 仮引数`distance`で指定した距離だけ進むようにする。手順は次の通り。
    1. `while`文に入る前に、ロボットの自己位置推定による現在地を取得し、局所変数に記憶しておく。つまり、動作の開始地点。
    2. 繰り返しのたびに、`x`、`y`と、動作の開始地点との距離を計算し、それが`distance`以上になったとき、`break`で`while`ループを抜ける。
  - 仮引数`time_limit`は直進の制限時間を表しているが、`main`関数で使用する際は`go_straight_by_distance_with_localization(listener, 1.0, 30.0)`のように長めにしておくこと。
  - `rospy.loginfo`を使った出力では、次のようにどれだけ進んだかも表示すること。

```shell
・・・
[INFO] [1634615851.715154, 1147.450000]: Recv localized pose. (x, y, theta) d = (8.14, 0.92, 89.42) 0.93 # 0.93m進んだ
[INFO] [1634615851.957260, 1147.675000]: Recv localized pose. (x, y, theta) d = (8.15, 1.01, 89.42) 1.02
```

### 課題（２）

- `go_straight_by_distance_with_localization`関数をコピーし、`turn_by_angle_with_localization`という、指定した角度だけ旋回する関数を作りなさい。全問同様、自己位置推定の結果を使います。
  - 引数名は変えた方が良い（`linear_vel`：直進速度、`angular_vel`：回転速度）。
  - 回転速度のデフォルト値は`30度/秒`とする。

```python
def turn_by_angle_with_localization(listener, angle, time_limit=999, angular_vel=???, cmd_vel="/cmd_vel") # 初期値はどうする？
```

- 角度の場合は距離のときと異なり、－180度～＋180度で表現される点に注意が必要です。
- これらの注意点は[オドメトリを使ったとき](./simple_move_py_02.md#問題２)と全く同じです。

### 課題（３）

- `go_straight_by_distance_with_localization`、`turn_by_angle_with_localization`を使って、ロボットを四角形を描くように移動させてみよう。直進->90 度回転->直進・・・。時計回りに動いてから反時計回りに動くなど。
  - シミュレーションのロボットは自分の都合の良い位置に自由に移動させて構わない。
- 作成したプログラムを実機でも試してみよう。オドメトリのときと比べてどうだろうか。
  - 実機の場合は[ナビゲーション起動コマンド](https://github.com/KMiyawaki/oit_navigation_minibot_light_01#%E3%83%8A%E3%83%93%E3%82%B2%E3%83%BC%E3%82%B7%E3%83%A7%E3%83%B3)を実行してから自分のプログラムを実行すること。

---

[基本的な動作](./Home.md)
