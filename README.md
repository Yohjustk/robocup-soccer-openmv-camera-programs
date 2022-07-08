# robocup-soccer-openmv-camera-programs
__ロボカップジュニアサッカーでのopenMV CAMを用いたカメラ制御用サンプルプログラム群__


## 最初に
- これらのプログラムはロボカップジュニアサッカーワールドリーグでの使用を想定して作成されています。競技ルールについては[ルールブック](https://drive.google.com/file/d/1nENPlAM84UK_o8h0x2WjepuD2egbmoI7/view)を参照して下さい。

- これらのプログラムはopenMV IDE上で、micro pythonでopenMV独自のライブラリを用いて書かれたものです。そのため使う場合は[openMV IDE](https://openmv.io/pages/download)を入手することを推奨します。

- 近年、[全方位ミラー](https://yunit.techblog.jp/archives/70016697.html)とカメラを用いた画像認識方法がロボカップサッカーで多く使われるようになっています。このサンプルプログラム群も全方位ミラーとopenMV CAMを組み合わせて使用することを前提として作成されています。

- 基本的なカメラの画面情報を以下に示します。
![screen_info](./screen_info.png)

## 機能・フローチャート
このプログラム群は以下のような特徴を持ちます。

- 今回作成したプログラムは、基本的に「ロボットの四肢」であるモーターなどの制御を行うマイコンに、ロボットの移動方向・向く方向を送信する「ロボットの脳」の役目を果たすように作成されています。そのため、プログラムは**十分な処理速度**を満たせるように作成されています。

- 機能ごとにサンプルプログラムが存在し、調整は必要ですが最終的に**自分が望む機能のみで組み上げる**ことができます。私が組み上げた例も参照できます。

以下にそれぞれのプログラムの機能・フローチャートを示します。
### [ball_tracker.py](https://github.com/Yohjustk/robocup-soccer-openmv-camera-programs/blob/master/ball_tracker.py)
ボールを探索するプログラムです。
基本的な色認識のライブラリでボールを探索し、認識した場合はボールの位置に応じて回り込む移動角度・向く角度を送信します。このライブラリは精度は高いですが、**処理が重い**ことが難点です。そのため基本的に本当に重要な機能以外では使わないことをお薦めします。ボールの位置を推定するには自身でデータを取り、方程式を導出する必要があります。

![ball_tracker.py　フローチャート](./ball_tracker_flowchart.png)

### [circular_goal_scanner.py](https://github.com/Yohjustk/robocup-soccer-openmv-camera-programs/blob/master/circular_goal_scanner.py)
効率的にゴールを探索するプログラムです。
円環状にピクセルの色を読み取り、ゴールの色の閾値と比較・判定してゴールのある位置、幅を推定します。テーブルを使い、また円環状にスキャンする範囲を制限することで軽量化されています。掲載しているプログラムでは円環を3重にしていますが、何重にするかは任意で変更可能です。

![circular_goal_scanner.py　フローチャート](./circular_goal_scanner_flowchart.png)

### [radial_enemy_scanner.py](https://github.com/Yohjustk/robocup-soccer-openmv-camera-programs/blob/master/radial_enemy_scanner.py)
ゴール前の敵ゴールキーパーロボットを検知し、回避するプログラムです。
circulae_goal_scanner.pyと同じプログラムでゴール範囲を特定し、ゴールの範囲を角度で等分します。分割した角度上のピクセルを画面中心からにスキャンし、それぞれの角度ごとにゴールの色を認識する距離を計測します。ゴールの色を認識できなかったか、認識した距離の平均値よりも遠い（マージンの範囲の外に出ている）角度上に敵ロボットがいると判断します。そして最後にその敵ロボットがいる角度がゴール範囲内にもつ左右の隙間を比較し、より広い方に向かいます。

- スキャンするピクセル同士に間隔を設けることで、処理を軽量化しています。間隔幅は`enemy_skip`で定義されています。
- 敵がいると認識する距離のマージンは`enemy_mg`で定義されています。
- ゴール範囲を分割する個数は`split`で定義されています。

これらの変数は使用する環境（ミラー形状・周辺光など）によって調整する必要があります。


![radial_enemy_scanner.py　フローチャート](./radial_enemy_scanner_flowchart.png)

### [wall_checker.py](https://github.com/Yohjustk/robocup-soccer-openmv-camera-programs/blob/master/wall_checker.py)
壁を検知すると、壁から離れるようにするプログラムです。ロボカップサッカーでは壁の近くに白線があり、そこから出てしまうとペナルティが発生してしまいます。そのため多くの場合ロボットは床の色を読む光センサを搭載します。このプログラムはその光センサの機能を代替する可能性があります。
このプログラムは画面上の４つの座標の周辺のピクセルの色を壁の色と比較し、特定のピクセル数以上壁の色と一致しているピクセルがあった場合に、その座標に「壁がある」と判断します。それぞれの座標には数字が割り当てられており、壁があった場合にそれらの値は加算されていき、その加算結果からどの方向に壁があり、どの方向に移動すべきかを決定します。この加算結果の数値を参照するだけで決定する仕組みにより処理の軽量化・コードの短縮化に成功しています。それぞれに割り当てられた数字と、組み合わせを以下に示します。座標の数を増やすことで精度を上昇させることができますが、パターン数も上昇します。

![wall_checker.py 表](./wall_checker_chart.png)

### [own_goal_distance_keeper.py](https://github.com/Yohjustk/robocup-soccer-openmv-camera-programs/blob/master/own_goal_distance_keeper.py)
ゴールキーパーロボット用の自陣ゴールとの距離が遠くなりすぎないように保つプログラムです。
基本的な部分はradial_enemy_scanner.pyと同様に角度ごとに自陣ゴールまでの距離を計測し、ゴールまでの距離が遠くなりすぎると一番ゴールまでの距離が近い角度に移動します。

![own_goal_distance_keeeper.py フローチャート](./own_goal_didtance_keeper_flowchart.png)

### [assembly_forward.py]()
### [assembly_goal_keeper]()