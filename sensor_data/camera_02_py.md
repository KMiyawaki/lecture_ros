# カメラの画像を使う（２）

[Home](./Home.md)

---

カメラ画像から特定の色を持つ領域を抽出し、その物体の方向に車体を回転させます。

この項目は実機（Jetson NANO）の場合、`Python`でしかできません。

## 実習

```shell
$ roscd beginner_tutorials/scripts
$ pwd
/home/[user name]/catkin_ws/src/beginner_tutorials/scripts
```

- 次のファイルを`scripts`にダウンロードし実行しなさい。
  - [color_tracking.py](https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/sensor_data/camera/color_tracking.py)

```shell
$ wget https://raw.githubusercontent.com/KMiyawaki/lecture_ros/main/sensor_data/camera/color_tracking.py
・・・
2020-10-28 12:01:06 (2.69 MB/s) - ‘color_tracking.py’ saved [1785/1785]
```

### 実行順序

- まず、[シミュレータを起動する](../stage_simulator/stage_simulator_01.md)
- しばらくしてから`color_tracking.py`を実行する。

```shell
$ rosrun beginner_tutorials color_tracking.py 
[INFO] [1639880984.187661, 0.000000]: C19XXX 工大 太郎
[INFO] [1639880985.201792, 111.075000]: Executing color_track
[INFO] [1639880985.263930, 111.125000]: Recv msg. class = Image
[INFO] [1639880985.286733, 111.150000]: image_processing: Recv image (670 x 700)
[INFO] [1639880985.294546, 111.150000]: R 2779 # Stage シミュレータで赤いブロックをロボットの正面に持ってくると表示される。
・・・
```

- さらに別ターミナルで下記コマンドを実行すると`color_tracking.py`ノードが計算した画素数を受信することができる。
  - 今回は演習を簡略化するため文字列で情報を`publish`しているが、実際は整数の配列等を使うのが良い。

```shell
$ rostopic echo /image_processing/result
data: "R 2376"
---
data: "R 2376"
```

## 問題(1)

- 上記のプログラムはカメラ画像を受信し赤い領域を抽出してその画素数を出力している。
- さらに、その領域のノイズ除去した上で輪郭を抽出し、輪郭線を描画している。
- 実機でも赤い物体をカメラの前にかざして実行しなさい。
  - 実機の場合は[ナビゲーション起動コマンド](https://github.com/KMiyawaki/oit_navigation_minibot_light_01#%E3%83%8A%E3%83%93%E3%82%B2%E3%83%BC%E3%82%B7%E3%83%A7%E3%83%B3)を実行してから自分のプログラムを実行すること。
  - ただし、実機側でソフトのアップデートをしていない場合は実施すること。下記リンク先の下の方に方法が記載されています。
  - [ソフトのアップデート](https://github.com/KMiyawaki/oit_navigation_minibot_light_01#%E3%82%BD%E3%83%95%E3%83%88%E3%81%AE%E3%82%A2%E3%83%83%E3%83%97%E3%83%87%E3%83%BC%E3%83%88)
- パラメータ調整をしないと検出されない可能性が高いです。その場合は`color_tracking.py`のパラメータ調整が必要です。
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

コードの以下の部分、二つのタプルの第一要素で`H`を、第２，３要素で`S,V`の範囲を指定してください。

```python
    red = cv2.inRange(hsv, (0, 40, 40), (40, 255, 255))  # ここのパラメータを調整する
```

上手く抽出されると以下のような画像が得られます。

[成功例](./camera/2021-12-19_103735.png)

## 問題(2)

下記のコードを追記し赤い物体の中心に円を描画しなさい。ここで、`(x, y)`が矩形の左上座標、`w,h`が幅と高さである。

```python
            cv2.rectangle(cv_result, (x, y), (x + w, y + h),
                          (255, 255, 255), 5)  # 輪郭を囲む矩形を描画する
            cv2.circle(cv_result, (???, ???), 30, (0, 0, 0), -1) # 半径30pixel、黒い円で塗りつぶす。
```

[成功例](./camera/2021-12-19_104134.png)

**この例では円が２個出ていることに注意。**

## 問題(3)

前問ではすべての矩形の中心に円を描画した。コードを修正し、最も大きい矩形の中心にのみ円を描画しなさい。

[成功例](./camera/2021-12-19_104448.png)

**この例では円が１個だけ出ていることに注意。**

## 問題(4)

最も大きい矩形の中心を向くようにロボットを旋回させなさい。

例えば中心座標`(x, y)`のうち`x`座標を、画像全体のサイズ`width = cv_image_in.shape[0] height = cv_image_in.shape[1]`と比較する方法がある。

## 問題(5)

旋回だけでなく、前進後退もさせてみなさい。

例えば`y`座標の位置で車体をコントロールする方法がある。

---

[Home](./Home.md)
