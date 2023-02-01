# emcl_with_landmark: emclが視覚から検出のランドマークを使用

このリポジトリはemcl(https://github.com/ryuichiueda/emcl)の機能を拡張したものです。
カメラから検出されたランドマークを使用し，視覚重みを計算することで，2D LiDARの重みと一緒に計算する．
ランドマーク未検出の場合，元のemclと同じ動作をする．

## 全体の構造図

<img alt="system" height="1080" src="https://github.com/Tsumoridesu/emcl_with_landmark/blob/main/system.drawio.png" width="1920"/>
