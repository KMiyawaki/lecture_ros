# 一定時間動く (C++)

[基本的な動作](./Home.md)

---

## Navigation メタパッケージに頼らないロボットのコントロール

- [Navigation メタパッケージ](http://wiki.ros.org/ja/navigation)は経路計画等を自動的に行ってくれる便利なシステム。
- ただし自己位置推定が完璧でないとデタラメな動作をする。
- 実際のタスクでは最初正確な自己位置を推定することは難しい。
- その場合、とにかく少し前進してみると自己位置推定が正しくなってくることが多い。
- したがってタスク開始から少しの間、`Navigation`を使わず直接`/cmd_vel`にコマンドを送信しロボットを前進させる。

## 実習

```shell
$ roscd beginner_tutorials/src
$ pwd
/home/[user name]/catkin_ws/src/beginner_tutorials/src
```

- 次のファイルを`src`に保存しなさい。
  - [simple_move.cpp](https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/basic_behaviors/simple_move/simple_move.cpp)

```shell
$ wget https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/basic_behaviors/simple_move/simple_move.cpp
・・・
2020-10-28 12:01:06 (2.69 MB/s) - ‘simple_move.cpp’ saved [1785/1785]
```

テキストエディタで`~catkin_ws/src/beginner_tutorials/CMakeLists.txt`を編集し、末尾に以下を貼り付ける。

```text
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
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
[100%] Linking CXX executable /home/ubuntu/catkin_ws/devel/lib/beginner_tutorials/simple_move
[100%] Built target simple_move  # 100% まで表示されたら成功
```

### 実行順序

- まず、[シミュレータを起動する](../stage_simulator/stage_simulator_01.md)
- しばらくしてから`simple_move`を実行する。

```shell
$ rosrun beginner_tutorials simple_move
[INFO] [1632480750.849343, 60.675000]: Executing go_straight_by_time
```

- 起動したら、`Stage simulator`の画面と`rviz`の画面をよく観察すること。ロボットが２秒間直進して停止するはずである。

## 問題

- `simple_move.cpp`を修正し、直進->その場で（大体でよい）１回転-> 直進 という行動をさせなさい。
  - まずは`go_straight_by_time`関数をコピーし、`turn_by_time`という関数を作ってみよう。
  - 引数名は変えた方が良い（`linear_vel`：直進速度、`angular_vel`：回転速度）。
  - 回転速度のデフォルト値は`30度`とする。ただし、`angular_vel`は必ずラジアンの値を渡すようにすること。`angles::from_degrees(deg)`で、ラジアンへの変換ができる。
    - プログラミングの世界で角度を360度表現で扱うことは稀である。`sin`、`cos`といった関数は引数にラジアンを取る。

```c++
void turn_by_time(ros::NodeHandle& n, double time_limit, double angular_vel = ???, const std::string &cmd_vel = "/cmd_vel") // 初期値はどうする？
```

- ロボットを四角形を描くように移動させてみよう。直進->90 度回転->直進・・・。時計回りに動いてから反時計回りに動くなど。
  - シミュレーションのロボットは自分の都合の良い位置に自由に移動させて構わない。
- 作成したプログラムを実機でも試してみよう。

---

[基本的な動作](./Home.md)
