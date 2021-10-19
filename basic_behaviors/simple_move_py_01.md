# 一定時間動く (Python)

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
$ roscd beginner_tutorials/scripts
$ pwd
/home/[user name]/catkin_ws/src/beginner_tutorials/scripts
```

- 次のファイルを`scripts`に保存しなさい。実行権限の付与を忘れないように。
  - [simple_move.py](https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/basic_behaviors/simple_move/simple_move.py)

```shell
$ wget https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/basic_behaviors/simple_move/simple_move.py
・・・
2020-10-28 12:01:06 (2.69 MB/s) - ‘simple_move.py’ saved [1785/1785]

$ chmod u+x simple_move.py
$ ls -l
・・・
-rwxr--r-- 1 [user name] [user name] 1785 Oct 28 12:01 simple_move.py
```

### 実行順序

- まず、[シミュレータを起動する](../stage_simulator/stage_simulator_01.md)
- しばらくしてから`simple_move.py`を実行する。

```shell
$ rosrun beginner_tutorials simple_move.py
[INFO] [1632480750.849343, 60.675000]: Executing go_straight_by_time
```

- 起動したら、`Stage simulator`の画面と`rviz`の画面をよく観察すること。ロボットが２秒間直進して停止するはずである。

## 問題

- `simple_move.py`を修正し、直進->その場で（大体でよい）１回転-> 直進 という行動をさせなさい。
  - まずは`go_straight_by_time`関数をコピーし、`turn_by_time`という関数を作ってみよう。
  - 引数名は変えた方が良い（`linear_vel`：直進速度、`angular_vel`：回転速度）。
  - 回転速度のデフォルト値は`30度`とする。

```python
def turn_by_time(time_limit, angular_vel=???, cmd_vel="/cmd_vel"): # 初期値はどうする？
```

- ロボットを四角形を描くように移動させてみよう。直進->90 度回転->直進・・・。時計回りに動いてから反時計回りに動くなど。
  - シミュレーションのロボットは自分の都合の良い位置に自由に移動させて構わない。
- 作成したプログラムを実機でも試してみよう。

---

[基本的な動作](./Home.md)
