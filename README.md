# robdesstamp_2022

## パッケージについて
このパッケージは[オリジナル](https://github.com/rt-net/crane_x7_ros)である株式会社アールティ様のパッケージを使用して、千葉工業大学未来ロボティクス学科の2021年度の設計製作論3という講義で[知能コース1班が作成したもの](https://github.com/MakiSakurai/robotdesign3_2021_1)を使用した、2022年度のミラノ風ドドリアさん班が作成したものです。
## 概要
このリポジトリは、株式会社アールティ様が販売されているcrane_x7を制御し手にスタンプを押させるパッケージです。
このパッケージを使用する際カメラが必要になりますが、今回はRealSenseD435の使用を前提としています。
また、手の検出をする際に Google LLC様の[MediaPipe](https://github.com/google/mediapipe)を使用しています。

![RPReplay_Final1671362546](https://user-images.githubusercontent.com/83112617/209423541-41801fcb-1822-4a1c-9854-b118de171054.gif)



### MediaPipeを用いた手の検出

以下の画像のようにカメラ画像から手を検出し、各関節の座標を取得します。この取得した座標をもとに、手の甲の指定の位置（緑の円マーク）にマニピュレータがスタンプを押します。

![スクリーンショット 2022-12-24 150832](https://user-images.githubusercontent.com/83112617/209423853-595c86eb-33ca-43a6-8c83-39eb7b30c53a.png)

## 動作環境
OS : Ubuntu 18.04LTS

ROS Distribution: Melodic Morenia

Gazebo 9.0.0

Rviz 1.13.21

## セットアップ方法

- gitを使用して、robdesstamp_2022をダウロードします
```
cd ~/catkin_ws/src
git clone git@github.com:matsuyamayusaku/robdesstamp_2022.git
```

- 株式会社アールティ様から配布されているcrane_x7_rosをダウロードします
```
cd ~/catkin_ws/src
git clone https://github.com/rt-net/crane_x7_ros.git
```

- catkin buildを使用して本パッケージをビルドします
```
cd ~/catkin
catkin build
source ~/catkin_ws/devel/setup.bash
```

- 今回カメラを使用しますが、[robdesstamp3_2022/scripts/hand.pyの21行目](https://github.com/matsuyamayusaku/robdesstamp_2022/blob/2cc694d6585962ef92bf7c5f3fa2c32a289f5ae8/scripts/hand.py#L21)を使用しているカメラが認識されている/dev/videoの番号に変更してください


## 使用方法

### 実機起動用コマンド

crane_x7をPCに接続し、以下のコマンドを実行してデバイスドライバに実行権限を与えてから起動します
```
sudo chmod 666 /dev/ttyUSB0
roslaunch robdesstamp_2022 main.launch
```
"Enter"を押すことで動き始めます。


## 参考
### 動作概要

1. “Enter”が押されたら2へ進む。
2. 固定の動きによりスタンプを取る。(左右どちらかになるかはrandomを利用)
3. MediaPipeによる手の認識を用いてスタンプを押す。
4. どのようなスタンプが押されたか見せびらかす。
5. スタンプを元の場所へ戻し、Homeポジションへ戻る。
6. 手順1へ戻る


### 動いている様子

![IMG_1999](https://user-images.githubusercontent.com/83112617/209424087-ebd4edfc-2b2c-4182-8ae6-9fcc472e6e62.jpg)
https://youtu.be/u1fWdKh3Oak

## ライセンス

このリポジトリは株式会社アールティ様のライセンスに則って作成しています。詳細は、LICENSEファイルをご参照ください。
mediapipe:[Apache License 2.0](https://github.com/google/mediapipe/blob/master/LICENSE)
