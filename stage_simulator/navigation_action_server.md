# ROS navigation の Action Server を使う

[stage_simulator/Home](Home.md)

---

## シミュレータを起動する

前頁[Stage Simulator (1)](./stage_simulator_01.md)に従って、シミュレータを起動しておく。

## ロボットをコマンドで動かす

- 下記コマンドを入力するが、`Tab`キーを活用するので説明を聞くこと。
- コマンドを実行するとロボットが前進し、いつかは壁にぶつかって停止する。
  - `Ctrl+C`でコマンドの実行を停止する。

```shell
$ rostopic pub /cmd_vel geometry_msgs/Twist "linear:
 x: 0.3
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0"  -r 10
```

終わったら、一旦シミュレータを終了させておくこと。

## move_base にコマンドを送る

- [move_base](http://wiki.ros.org/move_base)は`ROS navigation`メタパッケージの全体 ( SLAM 以外) を束ねるものである。

  - 現状でも`/cmd_vel`に速度をパブリッシュすればロボットを動かせる。しかし今やりたいのは、`move_base`が持つ、大域的・局所的な経路計画と障害物回避機能である。
  - この機能は`ROS`の`Action Server`という仕組みを使って実装されている。簡単にいうと、他の実行プログラムの機能をあたかも関数のように呼び出せる機能で、トピックを使った通信より信頼のおけるものとなっている。

- [move_base にコマンドを送る (Python)](./navigation_action_server_py.md)
- [move_base にコマンドを送る (C++)](./navigation_action_server_cpp.md)

## 課題

- 任意の目標地点を数個（4 点程度）地図上に設定し、それらを順番に回っていくプログラムを作成しなさい。
  - このように最終目的地に至るまでのサブゴールをウェイポイントと呼びます。
  - 本来、ウェイポイントで停止して欲しくはないですが、現状では止まってしまう問題があります。
- 目標地点の座標は`Stage`上の座標軸からではなく、下記の方法で読み取ること。
  - [マップ上の座標の調べ方](../how_to_get_coordinates.md)

## 参考

- [ROS×Python 勉強会：ウェイポイントナビゲーション(ActionLib：Python)](http://demura.net/lecture/12433.html)

### `Stage`のシミュレータ上に障害物を置く方法

- 地図を編集して障害物を作っているのではない点に注意。なお、現在のシミュレータにはもともと障害物を置いてある。

```shell
$ roscd oit_navigation_test/maps
$ emacs HRC.world &
```

- 末尾に追記

```text
# Additional Obstacles
define block model
(
  size [0.5 0.5 0.5]
  gui_nose 0
)
block( pose [ 3 -1 0 0] color "blue")
```

- `navigation.launch`を再起動すると Stage 上の画面に青い四角形の障害物が出ているが、Rviz 上では出ていない。
- しかし`Laser`のデータを見ると、障害物があることが分かる。
- これで、 ROS の`navigation`の未知の障害物に対する回避機能を試すことができる。
- また、ここで定義した障害物は`Stage`のウィンドウ上でマウスドラッグにより移動可能である。
  - したがって、ドアオープンのシミュレーションや移動する人に対する回避シミュレーションもできる。

---

[stage_simulator/Home](Home.md)
