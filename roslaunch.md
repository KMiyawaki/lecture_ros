# roslaunch（ロスランチ）

多数の`node`を一度に起動するための便利な仕組み。

[README](./README.md)

---

## 一つのシステムに必要な node 数は膨大

- ROS は複数の`node`を連携させて一つのシステムとして動作させる。
- 一つのロボットを動作させるノード群の構成例。
  - ロボット全体の統合＝状態遷移に基づくタスク遂行
  - 音声対話
  - 台車のモータコントローラの ROS ドライバ
  - ステッピングモータ・サーボモータの ROS ドライバ
  - 台車のナビゲーション（目的地への経路生成など）
  - アームの逆運動学計算
  - 画像認識
    - 物体の認識
    - 人間の姿勢推定と動作認識
    - 人間の顔、性別、年齢認識
- 各パッケージに複数の`node`が含まれる。他の ROS の`node`に依存するものもある。
- **一つのシステムにつき大量の`node`を実行する必要がある。全ての`node`を逐一`rosrun`することは不可能。**

---

## roslaunch コマンド

- 構文

```shell
$ roslaunch [パッケージ名] launchファイル名
```

- `launch`ファイル
  - XML で書かれたバッチファイル（実行プログラムのリストを記述し順番に実行するための小さなプログラム）
  - 拡張子は`.launch`。一般的な XML の拡張子`.xml`とは異なる。

---

## 実習(1)

- **実習に入る前に必ず全ての端末を閉じなさい。**
- 端末を開き`roscore`を起動。以降この端末を「端末(1)」とする。
- 端末を開き`rosrun turtlesim turtlesim_node`で亀のシミュレータを起動。以降この端末を「端末(2)」とする。
- 端末を開き`rosrun turtlesim turtle_teleop_key`を実行。以降この端末を「端末(3)」とする。
- 端末(3)で矢印キーを適当に押して、亀が移動することを確認する。
- 端末(1)~(3)を全て終了し、閉じる。

---

## 実習(2)

- **実習に入る前に必ず全ての端末を閉じなさい。**

- `launch`ファイルを作成する。

```shell
$ roscd beginner_tutorials/
$ mkdir launch
$ cd launch
$ touch turtle.launch # このファイルをVSCode等で編集する。touch はファイル作成のコマンド
```

- `turtle.launch`の内容は次の通り。

```xml
<launch>
  <node name="turtlesim" type="turtlesim_node" pkg="turtlesim" />
  <node name="teleop_key" type="turtle_teleop_key" pkg="turtlesim" />
</launch>
```

---

- 次のコマンドを実行して、実習(1)と同様に矢印キーで亀をコントロールできることを確認しなさい。

```shell
$ roslaunch beginner_tutorials turtle.launch
```

- **roslaunch した端末をクリックしてからでないと矢印キーでのコントロールはできない。**
- 亀の移動ができたら`Ctrl+C`で終了させ、端末を閉じなさい。
- さらに、次のやり方でも`roslaunch`できることを確認しなさい。

```shell
$ roscd beginner_tutorials/launch
$ roslaunch turtle.launch
```

- 亀の移動ができたら`Ctrl+C`で終了させ、端末を閉じなさい。

---

## 問題(1)

- 実習(1)と実習(2)で起動した端末の数や実行手順に着目し`roslaunch`の優位性を述べなさい。
  - 単に起動した端末数が少なくなっただけではなく、一つ非常に重要な違いがある。

---

## launch ファイルのタグ

- XML において`<`と`>`で囲まれる部分を「タグ」と呼ぶ。
- 「タグ」には開始タグと終了タグがある。`</タグ名>`のように`/（スラッシュ）`が入っているのが終了タグ。
- XML では **開始タグと終了タグが必ず対になっている。** 開始タグと終了タグの間に他の要素を記述する。
- ただし、開始タグと終了タグの間に何も書かない場合は`<タグ名 />`とすることで終了タグを省略できる。

```xml
終了タグ省略の例。
<node name="teleop_key" type="turtle_teleop_key" pkg="turtlesim" />
```

---

## launch タグ

- `launch`ファイルの全ての内容はこのタグの間に記述される。

## node タグ

- `name="ノードに付ける名前"`
- `type="ノードの実行形式ファイル名"` Python だとスクリプト名を指定する。（例）`type="talker.py"`
- `pkg="パッケージ名"`

---

- [roslaunch（ロスランチ）(Python)](./roslaunch_py.md)
- [roslaunch（ロスランチ）(C++)](./roslaunch_cpp.md)

---

## node タグ（追加）

- `output="screen"`
  - `roslaunch`で`node`を起動した場合、`rospy.loginfo`や`ROS_INFO`のような ROS 固有の出力関数は画面に文字を出さなくなる。それを画面にも出力するように変更する。

## param タグ

- `name="パラメータ名"`
  - `node`タグ内に記述した場合、自動的にプライベートパラメータ扱いになるので`~（チルダ）`不要。
- `type="パラメータの型"`
  - 省略可能。`str int double bool yaml`のいずれか。省略した場合は自動的に型が推測される。
- `value="パラメータの値"`

---

## 変数 \$(find パッケージ名)

- **超重要** 指定されたパッケージの絶対パスを得る。
- この変数を使うことで`node`が使う設定ファイルのパスなどを簡単に取得できる。
- 例えば`~/catkin_ws/src/beginner_tutorials/config.txt`のような設定ファイルがあったとしたら、`launch`ファイル内では`$(find beginner_tutorials)/config.txt`としてファイルのパスを取得できる。

---

## 参考文献

- [http://wiki.ros.org/ja/roslaunch/XML](http://wiki.ros.org/ja/roslaunch/XML)
- [Roslaunch tips for large projects](http://wiki.ros.org/ROS/Tutorials/Roslaunch%20tips%20for%20larger%20projects)

---

[README](./README.md)
