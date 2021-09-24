# roslaunch（ロスランチ）(C++)

[roslaunch（ロスランチ）](./roslaunch.md)

---

## 実習(3)

- 新たな`launch`ファイルを作成する。

```shell
$ roscd beginner_tutorials/launch
$ touch pair.launch # このファイルを編集する。
```

- `pair.launch`の内容は次の通り。

```xml
<launch>
  <node name="talker" type="talker" pkg="beginner_tutorials" />
  <node name="listener" type="listener" pkg="beginner_tutorials" />
</launch>
```

- 次のコマンドで`pair.launch`を実行しなさい。**これまでのように画面上に文字が流れることはない。**

```shell
$ roslaunch beginner_tutorials pair.launch
```

---

- 別の端末を開き`rqt_graph`を実行し、確かに 2 つのノードが起動していることを確認しなさい。

![pair_01.png](./roslaunch/1061250541-pair_01.png)

- 結果を確認したら、`Ctrl+C`で終了させなさい。
- `rostopic echo /chatter`コマンドを実行して`/chatter`トピックにデータが流れていることも確認しなさい。

```shell
data: "Good Bye 46"
---
data: "Good Bye 47"
---
```

- データが流れていることを確認したら、`Ctrl+C`で終了させなさい。

---

- `rqt_console`コマンドを実行し、トピックが流れていることを GUI でも確認しなさい。

![pair_02.png](./roslaunch/2010396211-pair_02.png)

- 最後に全ての端末を`Ctrl+C`で終了させ、端末を閉じなさい。

---

## 実習(4)

- `pair.launch`を編集し、`talker`を起動している`node`タグを次のように変更しなさい。
  - **param タグを追加するために node の終了タグが必要になったことに注意**

```xml
  <node name="talker" type="talker" pkg="beginner_tutorials" />
```

を

```xml
  <node name="talker" type="talker" pkg="beginner_tutorials">
    <param name="text" type="str" value="Hello ROS launch" />
  </node>
```

にする。

---

- `listener`を起動している`node`タグを次のように変更しなさい。

```xml
  <node name="listener" type="listener" pkg="beginner_tutorials" output="screen" />
```

- 次のコマンドで`pair.launch`を実行しなさい。

```shell
$ roslaunch beginner_tutorials pair.launch
# launch の起動メッセージが流れる。
[ INFO] [1632479477.993946800]: I heard: [Hello ROS launch 3]
[ INFO] [1632479478.094340100]: I heard: [Hello ROS launch 4]
[ INFO] [1632479478.194534000]: I heard: [Hello ROS launch 5]
[ INFO] [1632479478.294730000]: I heard: [Hello ROS launch 6]
[ INFO] [1632479478.394069300]: I heard: [Hello ROS launch 7]
[ INFO] [1632479478.494204500]: I heard: [Hello ROS launch 8]
```

- 動作が確認できたら`Ctrl+C`で終了させ、端末を閉じなさい。

---

- `talker`を起動している`node`タグを次のように変更しなさい。

```xml
  <node name="talker" type="talker" pkg="beginner_tutorials">
    <param name="text" type="str" value="$(find beginner_tutorials)" />
  </node>
```

- これまで同様`pair.launch`を実行しなさい。
- 実行結果

```shell
# launch の起動メッセージが流れる。
[ INFO] [1632479522.505597300]: I heard: [/home/[ユーザ名]/catkin_ws/src/beginner_tutorials 3]
[ INFO] [1632479522.604779800]: I heard: [/home/[ユーザ名]/catkin_ws/src/beginner_tutorials 4]
[ INFO] [1632479522.705396200]: I heard: [/home/[ユーザ名]/catkin_ws/src/beginner_tutorials 5]
[ INFO] [1632479522.805491700]: I heard: [/home/[ユーザ名]/catkin_ws/src/beginner_tutorials 6]
[ INFO] [1632479522.904685500]: I heard: [/home/[ユーザ名]/catkin_ws/src/beginner_tutorials 7]
[ INFO] [1632479523.005123700]: I heard: [/home/[ユーザ名]/catkin_ws/src/beginner_tutorials 8]
```

---

[roslaunch（ロスランチ）](./roslaunch.md)
