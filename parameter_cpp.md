# ROS のパラメータ (C++)

[README](./README.md)

---

## 問題(1)（復習）ROS の node の起動

- `beginner_tutorials`パッケージの`talker`を起動し、動作が確認できたら終了させなさい。
  - 忘れた人は [ROS(1)](basics_01.md) を参照しなさい。
- `talker.cpp`や`listener.cpp`に変更を加えている人は、下記をダウンロードし、上書きして使いなさい。
  - [talker.cpp](https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/talker/talker.cpp)
  - [listener.cpp](https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/listener/listener.cpp)
  - **実行権限の付与を忘れないように。** `chmod u+x ファイル名`

ファイル取得をまとめると以下のコマンドになる。

```shell
$ roscd beginner_tutorials/src
$ wget https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/talker/talker.cpp -O talker.cpp
$ wget https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/listener/listener.cpp -O listener.cpp
```

ファイルを取得できたら実行すること。以下は実行時の出力例。

```shell
[ INFO] [1632477952.891893100]: hello world 0
[ INFO] [1632477952.992088500]: hello world 1
[ INFO] [1632477953.092486300]: hello world 2
[ INFO] [1632477953.192618500]: hello world 3
```

---

## rosparam

- `node`を起動する際にパラメータを与えて挙動を変化させる。

---

## 実習(1)（準備運動）

- `talker`を実行時に次のような出力が得られるように変更しなさい。

```shell
[INFO] [1534992351.137028]: Good Morning 0 # 数字は気にしない
```

---

## 実習(2)

- `ros::init(argc, argv, "talker");`の下に次のコードを追加しなさい。

```c++
  ros::NodeHandle pnh("~");
  std::string text = "Good Bye";
  pnh.getParam("text", text);
  // さらに、この文字列 text が画面に出力されるようにコードを修正しなさい。
```

- 次のコマンドで`talker`を起動して結果を確認しなさい。

```shell
$ rosrun beginner_tutorials talker _text:="Happy Halloween"
```

- 出力例

```shell
[ INFO] [1632478786.103009700]: Happy Halloween 0
[ INFO] [1632478786.203118400]: Happy Halloween 1
[ INFO] [1632478786.304004300]: Happy Halloween 2
[ INFO] [1632478786.403233600]: Happy Halloween 3
```

---

## ros::NodeHandle::getParam("パラメータ名", デフォルト値)

- `node`起動時に与えられたパラメータをプログラムから取得する。
- 指定された名前のパラメータが存在すればその値を、存在しなければデフォルト値を返却する。

## rosrun でのパラメータ指定

- パラメータ名の前に「\_（アンダースコア）」が必要。

```shell
$ rosrun パッケージ名 スクリプト名 _パラメータ名:=パラメータ
```

## 補足

- `ros::NodeHandle pnh("~")`のように`~（チルダ）`を引数として生成された`ros::NodeHandle`を通じてパラメータを取得すると[プライベートパラメータ](http://wiki.ros.org/ja/Parameter%20Server#Private_Parameters.28.2BMNcw6TCkMNkw.2FDDIMAAw0TDpMOEw.2FDC.2F.29)が取得できる。`rosrun`におけるパラメータ名先頭の`_（アンダースコア）`はそのパラメータがプライベートであることを示している。

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
