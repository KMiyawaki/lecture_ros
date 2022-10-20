# カメラの画像を使う

[Home](./Home.md)

---

## ソフトのアップデート

実機、シミュレータともにソフトをアップデートしなさい。下記リンク先の下の方に方法が記載されています。

[ソフトのアップデート](https://github.com/KMiyawaki/oit_navigation_minibot_light_01#%E3%82%BD%E3%83%95%E3%83%88%E3%81%AE%E3%82%A2%E3%83%83%E3%83%97%E3%83%87%E3%83%BC%E3%83%88)

## 実習

```shell
$ roscd beginner_tutorials/src
$ pwd
/home/[user name]/catkin_ws/src/beginner_tutorials/src
```

- 次のファイルを`src`にダウンロードし実行しなさい。
  - [image_processing.cpp](https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/sensor_data/camera/image_processing.cpp)

```shell
$ wget https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/sensor_data/camera/image_processing.cpp
・・・
2020-10-28 12:01:06 (2.69 MB/s) - ‘image_processing.cpp’ saved [1785/1785]
```

テキストエディタで`~catkin_ws/src/beginner_tutorials/CMakeLists.txt`を編集し、以下の一行を挿入する。

```text
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge # 追記
)
```

さらに末尾に以下を貼り付ける。

```text
add_executable(image_processing src/image_processing.cpp)
target_link_libraries(image_processing ${catkin_LIBRARIES})
```

さらに以下を追記する。

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
[ 98%] Linking CXX executable /home/ubuntu/catkin_ws/devel/lib/beginner_tutorials/image_processing
[100%] Built target image_processing # 100% まで表示されたら成功
```

### 実行順序

- まず、[シミュレータを起動する](../stage_simulator/stage_simulator_01.md)
- しばらくしてから`image_processing`を実行する。

```shell
$ rosrun beginner_tutorials image_processing 
[ INFO] [1638606050.494379200]: C19XXX
[ INFO] [1638606050.765701100, 165.450000000]: imageCallback: Recv image (700 x 670)
[ INFO] [1638606050.779243700, 165.475000000]: imageCallback: R 2376 # Stage シミュレータで赤いブロックをロボットの正面に持ってくると表示される。
[ INFO] [1638606050.784195300, 165.475000000]: imageCallback: Recv image (700 x 670)
[ INFO] [1638606050.790355000, 165.475000000]: imageCallback: R 2376 # Stage シミュレータで赤いブロックをロボットの正面に持ってくると表示される。
・・・
```

- さらに別ターミナルで下記コマンドを実行すると`image_processing`ノードが計算した画素数を受信することができる。
  - 今回は演習を簡略化するため文字列で情報を`publish`しているが、実際は整数の配列等を使うのが良い。

```shell
$ rostopic echo /image_processing/result
data: "R 2376"
---
data: "R 2376"
```

## 問題(1)

- 上記のプログラムはカメラ画像を受信し赤い領域を抽出してその画素数を出力している。
- 実機でも赤い物体をカメラの前にかざして実行しなさい。
  - 実機の場合は[ナビゲーション起動コマンド](https://github.com/KMiyawaki/oit_navigation_minibot_light_01#%E3%83%8A%E3%83%93%E3%82%B2%E3%83%BC%E3%82%B7%E3%83%A7%E3%83%B3)を実行してから自分のプログラムを実行すること。
  - ただし、実機側でソフトのアップデートをしていない場合は実施すること。下記リンク先の下の方に方法が記載されています。
  - [ソフトのアップデート](https://github.com/KMiyawaki/oit_navigation_minibot_light_01#%E3%82%BD%E3%83%95%E3%83%88%E3%81%AE%E3%82%A2%E3%83%83%E3%83%97%E3%83%87%E3%83%BC%E3%83%88)
- パラメータ調整をしないと検出されない可能性が高いです。その場合は`image_processing.cpp`のパラメータ調整が必要です。
- まず、以下の方法で`H`の値を調べなさい。

### 最適なHの値を調べる

- 実機カメラから得られる画像を保存します。`roslaunch oit_navigation_minibot_light_01 navigation.launch`で実機を起動しなさい。
- 次のコマンドを実機ターミナルから実行しなさい。`SSH`接続した端末からでも構いません。

```shell
$ roscd beginner_tutorials
$ rosrun oit_navigation_minibot_light_01 image_capture.py 
[INFO] [1638608939.180082]: Saved sample.jpg # 保存に成功 
```

[キャプチャ画像のサンプル](./camera/sample.jpg)

- 保存できた画像を手元のPCにVSCode経由でダウンロードし、赤い物体の`HSV`値を調べなさい。
  - これには[gimp](https://forest.watch.impress.co.jp/library/software/gimp/)のようなペイントソフトでスポイトツールを利用するなどの方法があります。
- このようにして調べた`H`の値を`OpenCV`で利用する際は**値を1/2にする**ことを忘れないようにしてください。

### パラメータを設定する

コードの以下の部分、`cv::Scalar`の二つのコンストラクタの第一引数で`H`の範囲を指定してください。

```c++
      // HSV 画像から特定の H の値を持つ画素を抽出する。この場合は赤色。
      cv::inRange(cv_hsv, cv::Scalar(0, 120, 120), cv::Scalar(40, 255, 255), cv_mask); // ここのパラメータを調整する
```

上手く抽出されると以下のような画像が得られます。

[成功例](./camera/2021-12-04_182257.png)

## 問題(2)

下記のコードを追記し赤い物体に加えて青い物体も検出しなさい。

- `H`だけでなく`SV`の調整も必要な場合がある。

```c++
      // HSV 画像から特定の H の値を持つ画素を抽出する。この場合は赤色。
      cv::inRange(cv_hsv, cv::Scalar(0, 120, 120), cv::Scalar(40, 255, 255), cv_mask); // ここのパラメータを調整する
      cv_result.setTo(cv::Scalar(0, 0, 255), cv_mask);                                 // 抽出された部分を赤く塗りつぶす。
      int red_pixels = cv::countNonZero(cv_mask);                                      // 赤の画素数を数える。
      // 以下を追記する。
      cv::inRange(cv_hsv, cv::Scalar(???, 120, 120), cv::Scalar(???, 255, 255), cv_mask); // ここのパラメータを調整する
      cv_result.setTo(cv::Scalar(255, 0, 0), cv_mask);                                    // 抽出された部分を青く塗りつぶす。
      int blue_pixels = cv::countNonZero(cv_mask);                                        // 青の画素数を数える。
```

- 下記も修正し、青い画素数の情報も`publish`されるようにしなさい。

```c++
      // 画素数情報を文字列にして Publish する
      std_msgs::String msg_result;
      std::stringstream sst;
      sst << "R " << red_pixels << " B " << blue_pixels;
```

---

[Home](./Home.md)
