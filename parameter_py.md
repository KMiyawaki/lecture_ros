# ROS のパラメータ (Python)

[README](./README.md)

---

## 問題(1)（復習）ROS の node の起動

- `beginner_tutorials`パッケージの`talker.py`を起動し、動作が確認できたら終了させなさい。
  - 忘れた人は [ROS(1)](basics_01.md) を参照しなさい。
- `talker.py`や`listener.py`に変更を加えている人は、下記をダウンロードし、上書きして使いなさい。
  - [talker.py](https://raw.githubusercontent.com/ros/ros_tutorials/indigo-devel/rospy_tutorials/001_talker_listener/talker.py)
  - [listener.py](https://raw.githubusercontent.com/ros/ros_tutorials/indigo-devel/rospy_tutorials/001_talker_listener/listener.py)
  - **実行権限の付与を忘れないように。** `chmod u+x ファイル名`

ファイル取得と実行権限付与をまとめると以下のコマンドになる。

```shell
$ roscd beginner_tutorials/scripts
$ wget https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py -O talker.py
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py -O listener.py
$ chmod u+x talker.py listener.py
```

ファイルを取得できたら実行すること。以下は実行時の出力例。

```shell
[INFO] [1534992351.137028]: hello world 1534992351.14
[INFO] [1534992351.237922]: hello world 1534992351.24
[INFO] [1534992351.337966]: hello world 1534992351.34
[INFO] [1534992351.436902]: hello world 1534992351.44
```

---

## rosparam

- `node`を起動する際にパラメータを与えて挙動を変化させる。

---

## 実習(1)（準備運動）

- `talker.py`に下記の変更を加えなさい。
- 関数`talker`に仮引数`text`を追加しなさい。`text`には文字列が与えられることを想定しています。

---

- 関数`main()`において関数`talker`を実引数`"Good Morning"`を与えて呼び出すように変更しなさい。
- `talker.py`実行時に次のような出力が得られるように関数`talker`を変更しなさい。

```shell
[INFO] [1534992351.137028]: Good Morning 1534992351.14 # 数字は気にしない
```

- 関数`main()`において関数`talker`を実引数`"Good Bye"`を与えて呼び出すと、次のよう出力がなされることを確認しなさい。
  - **talker 関数の内容を変更してはいけない。**

```shell
[INFO] [1534992351.137028]: Good Bye 1534992351.14 # 数字は気にしない
```

---

## 実習(2)

- `rospy.init_node('talker', anonymous=False)`の部分で`anonymous=False`となっていなければ修正しなさい。
- その下に次のコードを追加しなさい。

```python
text = rospy.get_param("~text", text)
```

- 次のコマンドで`talker.py`を起動して結果を確認しなさい。

```shell
$ rosrun beginner_tutorials talker.py _text:="Happy Halloween"
```

- 出力例

```shell
[INFO] [1534994829.681879]: Happy Halloween 1534994829.68
[INFO] [1534994829.781600]: Happy Halloween 1534994829.78
[INFO] [1534994829.881448]: Happy Halloween 1534994829.88
```

---

## rospy.get_param(" **~** パラメータ名", デフォルト値)

- `node`起動時に与えられたパラメータをプログラムから取得する。
- 指定された名前のパラメータが存在すればその値を、存在しなければデフォルト値を返却する。

## rosrun でのパラメータ指定

- パラメータ名の前に「\_（アンダースコア）」が必要。

```shell
$ rosrun パッケージ名 スクリプト名 _パラメータ名:=パラメータ
```

## 補足

- `rospy.get_param`の`~（チルダ）`や`rosrun`におけるパラメータ名先頭の`_（アンダースコア）`はそのパラメータが[プライベートパラメータ](http://wiki.ros.org/ja/Parameter%20Server#Private_Parameters.28.2BMNcw6TCkMNkw.2FDDIMAAw0TDpMOEw.2FDC.2F.29)であることを示している。

---

## rosparam list

- ノード起動時に指定したパラメータは ROS マスターが提供するパラメータサーバに記憶されている。
  `rosparam list`コマンドで現在設定されているパラメータの一覧を取得できる。
- 別のターミナルを起動し、次のコマンドを実行しなさい。

```shell
$ rosparam list
/rosdistro
/rosversion
・・・
/talker/text
```

---

## rosparam get パラメータ名

- パラメータサーバに記憶されているパラメータの内容を見る。
- 続けて、次のコマンドを実行しなさい。

```shell
$ rosparam get /talker/text
Happy Halloween
```

---

## どんなものがパラメータになるのか

- 複数接続されたデバイスの識別子。
  - （例）カメラの番号
- `node`を実行するために必要なデータベースのファイル名。
  - （例）画像認識の物体学習データ。音声認識の設定ファイル。
- 台車の最高速度。
  - （例）直進 0.4m/sec、回転 30deg/sec
- **これらの値を本番用のプログラムに直接書いてはいけない（特に`C++`）！！**

---

## 問題(2)

- パラメータを本番用のプログラムに直接書いてはいけない理由は何か？

## 参考文献

- [ROS のサービスとパラメータを理解する](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingServicesParams)

---

[README](./README.md)
