# レーザレンジファインダのデータを使う (C++)

[Home](./Home.md)

---

## 実習

```shell
$ roscd beginner_tutorials/scripts
$ pwd
/home/[user name]/catkin_ws/src/beginner_tutorials/src
```

- 次のファイルを`src`にダウンロードし実行しなさい。
  - [check_laser.cpp](https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/sensor_data/laser/check_laser.cpp)

```shell
$ wget https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/sensor_data/laser/check_laser.cpp
・・・
2020-10-28 12:01:06 (2.69 MB/s) - ‘check_laser.cpp’ saved [1785/1785]
```

テキストエディタで`~catkin_ws/src/beginner_tutorials/CMakeLists.txt`を編集し、末尾に以下を貼り付ける。

```text
add_executable(check_laser src/check_laser.cpp)
target_link_libraries(check_laser ${catkin_LIBRARIES})
```

## コンパイル

以下のコマンドでコンパイルする。`C++`の場合は、ファイルを編集後、実行前に必ずコンパイルが必要である。

```shell
$ cd ~/catkin_ws && catkin_make
Base path: /home/[user name]/catkin_ws
Source space: /home/[user name]/catkin_ws/src
...
####
#### Running command: "make cmake_check_build_system" in "/home/[user name]/catkin_ws/build"
...
[100%] Linking CXX executable /home/ubuntu/catkin_ws/devel/lib/beginner_tutorials/check_laser
[100%] Built target check_laser  # 100% まで表示されたら成功
```

### 実行順序

- まず、[シミュレータを起動する](../stage_simulator/stage_simulator_01.md)
- しばらくしてから`check_laser`を実行する。

```shell
$ rosrun beginner_tutorials check_laser
[INFO] [1638258318.313909, 3270.650000]: Executing check_laser
[INFO] [1638258318.322379, 3270.650000]: is_simulation = True
[INFO] [1638258318.366697, 3270.700000]: Recv LaserScan
[INFO] [1638258318.490554, 3270.825000]: Recv LaserScan
[INFO] [1638258318.613752, 3270.950000]: Recv LaserScan
・・・
```

## 問題(1)

- 上記のプログラムはレーザレンジファインダのデータを受信しメッセージを表示している。

- `check_laser.cpp`を編集し`LaserScan`のスキャンデータの個数を画面に出力しなさい。
  - `sensor_msgs/LaserScan`の中身は[ここ](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)
- 次いで、スキャンデータの配列において、中央の要素の値を画面に出力しなさい。単に要素数を２で割り実数部を切り捨てればよい。

シミュレータでの実行例

```shell
$ ./check_laser.cpp 
[INFO] [1638258680.892022, 3633.225000]: Executing check_laser
[INFO] [1638258680.903774, 3633.225000]: is_simulation = True
[INFO] [1638258680.945945, 3633.275000]: Recv LaserScan, num = 720, dist = 0.990009
[INFO] [1638258681.127661, 3633.450000]: Recv LaserScan, num = 720, dist = 1.290012
[INFO] [1638258681.290543, 3633.625000]: Recv LaserScan, num = 720, dist = 1.290012
[INFO] [1638258681.439832, 3633.775000]: Recv LaserScan, num = 720, dist = 1.550015
```

## 問題(2)

以下の直進距離は実習環境に応じて適宜変更します。

- ロボット正面に何か障害物を置いて、レーザでそれを検出し、障害物が除去されたら1m程度前進させなさい。
  - [演習用ロボット](https://github.com/KMiyawaki/oit_navigation_minibot_middle_01)搭載のレーザ[YDLIDAR X4](https://www.ydlidar.com/products/view/5.html)は11メートルまでの障害物しか検出できません。11メートル以内の空間に何もない場合、その場所の距離はゼロとして返ってきます。このことに注意してください。
  - シミュレーションと実機では、「ロボット正面」の距離が格納されている配列の要素番号が異なります。
    - シミュレーションの場合は配列の中央の要素です。
    - 実機の場合は`540`番目です。
    - `is_simulation = check_ros_node()`により変数`is_simulation`に、シミュレーション時であれば`True`そうでない場合は`False`が格納されていますので、この変数により場合分けをしてください。
    - ただし、実機が搭載するレーザレンジファインダは計測結果が安定しておらず、物体があっても距離ゼロを返してくる場合があります。その場合は例えば距離ゼロ、もしくは一定距離以上の値が一定個数観測されたら障害物は存在しないとする、といった工夫が必要です。
  - この課題は、ロボットに「ドアが開いたら部屋に入る」という行動をさせることを想定しています。
  - 実機の場合は[ナビゲーション起動コマンド](https://github.com/KMiyawaki/oit_navigation_minibot_light_01#%E3%83%8A%E3%83%93%E3%82%B2%E3%83%BC%E3%82%B7%E3%83%A7%E3%83%B3)を実行してから自分のプログラムを実行すること。

## 問題(3)

以下の地点（A）等は出題者から指示します。また、直進距離や待機時間は実習環境に応じて適宜変更します。

- 指定された場所（A）まで自律移動しなさい。
- その場で前方に障害物があるかどうかをチェックし、障害物がある場合は5秒待ちなさい。
  - 5秒以内に障害物が除去された場合は0.5m程度直進してから次の地点（B）に向かいなさい。
  - 5秒以内に障害物が除去されなかった場合はスタート地点（S）に戻りなさい。

## 参考文献

- [ROS-Manual-for-triangular-ranging-YDLIDAR.pdf](https://www.generationrobots.com/media/ROS-Manual-for-triangular-ranging-YDLIDAR.pdf)
  - P5に YDLIDAR X4 の座標系詳細が記述されている。

---

[Home](./Home.md)
