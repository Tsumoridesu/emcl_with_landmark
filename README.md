# emcl_with_landmark: emclが視覚から検出のランドマークを使用

このリポジトリは[emcl](https://github.com/ryuichiueda/emcl)の機能を拡張したものである．
カメラから検出されたランドマークを使用し，視覚重みを計算することで，2D LiDARの重みと一緒に計算する．
ランドマーク未検出の場合，元のemclと同じ動作をする．

## 全体の構造図

![system](https://github.com/Tsumoridesu/emcl_with_landmark/blob/main/system.drawio.png)

## 依存のパッケージ

### theta_simple_stitching  (全天球カメラ用)
branch: melodic-devel  
https://github.com/open-rdc/theta_simple_stitching

### yolov5_pytorch_ros  (ランドマーク検出用)
branch: detect_landmark  
https://github.com/open-rdc/yolov5_pytorch_ros

### orne_navigation(もしくは他のナビゲーション)
branch: refactor/noetic-devel  
https://github.com/open-rdc/orne_navigation

## 重要な式
ランドマークが検出された場合だけ
### 視覚重みの計算式
<p align="center">
<font size="16">$w_{vision} = cos(\varphi_{err})+A$ </font>
</p>
<p align="center">
<font size="16">$w = w_{vision} \cdot w_{LiDAR}$ </font>
</p>

### リセットの％
<p align="center">
<font size="16">$ratio = \frac{N_{detect} \cdot N_{detect in class} \cdot B} {N_{paritcle}}$</font>
</p>

## 追加のパラメータ
```landmark_file_path```: ランドマーク情報ファイル([サンプル](https://github.com/Tsumoridesu/emcl_with_landmark/blob/main/landmark_list.yaml))のパス 

```phi_th```:検出誤差の閾値($\varphi_{err}$がこれを超えたら重みがゼロになる)

```R_th```:検出距離の閾値(この距離を超えるランドマークが重みの計算を参加しない & リセットの半径)

```A```: 視覚重みの計算式の係数(大きい場合，視覚重みが大きくなる)

```B```: リセットの％の計算式の係数(一回のリセットとして取り出すパーティクルの量)

### 追加のsubscribe topic
```detected_objects_in_image```:yoloからpublishされる検出されたオブジェクトの情報

## ランドマーク情報ファイルの書き方
```yaml
landmark:
  landmark_name:                #認識したいランドマークの名前(class name)
    id:                         #ランドマークの番号/特殊の名前
     pose: [x, y, z]            #ランドマークの位置
     enable: null               #ランドマークが使うかどうか
     option: null               #ランドマーク他の色々

    id2:                        #ランドマークの番号/特殊の名前
     pose: [x, y, z]            #ランドマークの位置
     enable: null               #ランドマークが使うかどうか
     option: null               #ランドマーク他の色々
```

## TODO
一部のパラメータはまだテストされていない．