---
title: "006 - 順運動学"
date: 2023-07-28
categories: ["vnoidベースの開発"]
menu: main
---

---

## 今回の内容

本チャレンジでは、初心者の方でも簡単に人形ロボットの運動計画ができるように、  
vnoidというサンプルパッケージが用意されております。

今回はvnoidパッケージの機能の一つ、順運動学ソルバー(fksolver)について解説します。

---

## 順運動学とは
-	**概要**

	順運動学とは、ロボットにある関節変位を与えたときの  
	先端効果器の位置・姿勢を求める問題のことです。

	人型ロボットの腰リンク(以降ベースリンク)中心に、基準座標 $\Sigma_0$ を置きます。  
	基準座標系$\Sigma_0$から見た各リンク座標系 $\Sigma_i$ の位置・姿勢を計算し、  
	最終的に先端効果器の位置・姿勢を求めるのが目標です。

	{{<figure src="./base_to_links.png" class="center" alt="基準座標から見た各リンク座標" width="50%">}}

-	**座標変換の一般論**

	目標達成のため、ある座標系 $\Sigma_a$ から別の座標系 $\Sigma_b$ を定義する、  
	一般的な方法について考えてみましょう。

	$\Sigma_a$ に対する $\Sigma_b$ の位置に関しては、  
	並進方向の位置の差 $\boldsymbol{{}^ap_b}$ によって表現できます。

	次に、 $\Sigma_a$ に対する $\Sigma_b$ の姿勢の表現方法について考えてみましょう。  
	$\Sigma_a$ を基準とした姿勢とはつまり、  
	「 $\Sigma_a$ の関節軸周りに $\Sigma_b$ がどれだけ回転しているか」です。

	$\Sigma_a$ と $\Sigma_b$ をつなぐ関節が、  
	ロール・ピッチ・ヨー方向のいずれかに回転しているとして、  
	回転を回転行列 $\boldsymbol{{}^aR_b}$ で定義します。  
	すると、 $\Sigma_a$ に対する $\Sigma_b$ の姿勢は、回転行列 $\boldsymbol{{}^aR_b}$ によって表現できます。

	$\boldsymbol{{}^ap_b}$ と $\boldsymbol{{}^aR_b}$ によって、  
	$\Sigma_b$ 系の点を $\Sigma_a$ 系から表現できるかを確認してみましょう。

	そこで、$\Sigma_b$ から見て常に同じ場所に位置する点$E$について考えてみます。  
	$\boldsymbol{{}^bp_E}$ は固定ベクトルですが、  
	$\Sigma_a$ から見た点 $E$ $\boldsymbol{{}^ap_E}$ は $\Sigma_b$ の回転に伴って位置が変わります。

	回転行列をかければベクトルは指定したロール・ピッチ・ヨー回転をしますので、  
	以下図のような構図が見えてきます。

	{{<figure src="./a_to_e.png" class="center" alt="a_to_e" width="50%">}}

	よって、$\boldsymbol{{}^ap_E}$は次の式で与えられます。  
	$$ \boldsymbol{{}^ap_E} = \boldsymbol{{}^ap_b} + \boldsymbol{{}^aR_b} \boldsymbol{{}^bp_E} $$  

	以上のことから、$\Sigma_a$ から別の座標系 $\Sigma_b$ は、  
	$\boldsymbol{{}^ap_b}$ と $\boldsymbol{{}^aR_b}$ によって定義できることが分かります。

-	**ロボットの順運動学計算**

	さて、目標は人型ロボットの基準座標系$\Sigma_0$から見た、  
	各リンク座標系 $\Sigma_i$ の位置・姿勢を計算し、  
	最終的に先端効果器の位置・姿勢を求めることでした。

	上記の議論に基づけば、 $\Sigma_{0}$ から見た $\Sigma_{i}$ の位置・姿勢は次式で表せます。  
	$$ \boldsymbol{{}^0p_i} = \boldsymbol{{}^0p_{i-1}} + \boldsymbol{{}^0R_{i-1}} \boldsymbol{{}^{i-1}p_i} $$  
	$$ \boldsymbol{{}^0R_i} = \boldsymbol{{}^0R_{i-1}} \boldsymbol{{}^{i-1}R_i} $$  
	ここで、 $\boldsymbol{{}^{i-1}p_i}$ は各リンク座標間の相対位置であり、  
	$\boldsymbol{{}^{i-1}R_i}$ は各リンク座標間の回転行列です。  
	$\boldsymbol{{}^{i-1}p_i}$ はリンク長さによって決まり、  
	$\boldsymbol{{}^{i-1}R_i}$ は関節変位によって決まります。

	$i = 1$ から先端効果器を意味する番号まで、これらの式を繰り返し用いれば、  
	基準座標系 $\Sigma_0$ から見た先端効果器の位置・姿勢を計算できます。

	以上のように、リンク座標間の相対位置と関節変位が与えられれば、  
	基準座標から見たリンクの絶対位置が計算できることが分かります。  
	この計算を順運動学計算といいます。

---

## vnoidロボットの機構モデル

サンプルコードの解説に入る前に、vnoidロボットの機構モデルについて説明します。  
以下図に示すのが、vnoidロボットの機構モデルです。  
{{<figure src="./biped_robot_model.png" class="center" alt="vnoidロボットの機構モデル" width="50%">}}

vnoidのロボットには、1本の腕につき7つ、1本の脚につき6つの関節がついています。  
ロボットの関節にはそれぞれが識別できるように以下図のようにidが振られています。  
なお、ロボットから見て右側の機構については省略しています。  
{{<figure src="./biped_arm_model.png" class="center" alt="腕の機構モデル" width="50%">}}  
{{<figure src="./biped_leg_model.png" class="center" alt="脚の機構モデル" width="35%">}}


図中の $r$ , $p$ , $y$ はそれぞれロール・ピッチ・ヨーを表しており、  
その刻印が入った関節モーターは、その方向に回転します。  
なお、ロール・ピッチ・ヨー方向の回転はそれぞれ、  
$x$ , $y$ , $z$ 軸周りの回転に対応します。

---

## サンプルコードの解説

`vnoid/src/fksolver.cpp`を、脚の順運動学計算を例に解説します。  
脚の順運動学計算は、FkSolver::Comp関数内の124行目から始まります。

-	**変数の定義**

	まず変数の定義についてです。  
	`leg_pos`、`leg_ori`、`leg_axis`はそれぞれ、  
	ベースリンク座標を基準としたときの、  
	脚関節のローカル座標の位置、回転姿勢、回転軸です。  
	
	なお回転姿勢については回転行列 $\boldsymbol{R}$ の代わりに四元数 $\boldsymbol{Q}$ で表現されます。  
	四元数の詳細な説明に関しましては、ぜひ[こちら](https://eater.net/quaternions)の記事をご参照ください。

-	**関節回転変位の読み込み(125~128行目)**


	```cpp
	for(int i = 0; i < 2; i++){
		for(int j = 0; j < 6; j++){
			q[j] = joint[param.leg_joint_index[i] + j].q;
		}
		...
	```

	最初のfor文は、左右脚のうち一本ずつ順運動学計算をすることを意味します。

	その一つ下の階層のfor文で、脚関節の回転変位を一つずつ読み込みます。  
	ここで`leg_joint_index`とは、脚の付け根関節のidであり、  
	左脚であれば18、右脚であれば24です。

-	**脚の順運動学(130行目、8~31行目)**
	
	次に130行目のCompLegFk関数について説明します。
	
	```cpp
	CompLegFk(param.upper_leg_length, param.lower_leg_length, q, &leg_pos[i][0], &leg_ori[i][0], &leg_axis[i][0]);
	```
	
	この関数は大腿・下腿のリンク長さや、  
	関節の回転変位、脚関節の位置・姿勢・回転軸を引数に持ちます。
	
	この関数では、名前の通り脚に関する順運動学を解き、  
	脚関節の回転変位から、`leg_pos`、`leg_ori`を計算します。
	
	関数の中身を見てみましょう(8~31行目)。
	
	```cpp
	void FkSolver::CompLegFk(double l1, double l2, const double* q, Vector3* pos, Quaternion* ori, Vector3* axis){
	    Vector3  pos_local[6];
	    Vector3  axis_local[6];
	    pos_local[0] = Vector3(0.0, 0.0, 0.0);
	    pos_local[1] = Vector3(0.0, 0.0, 0.0);
	    pos_local[2] = Vector3(0.0, 0.0, 0.0);
	    pos_local[3] = Vector3(0.0, 0.0, -l1);
	    pos_local[4] = Vector3(0.0, 0.0, -l2);
	    pos_local[5] = Vector3(0.0, 0.0, 0.0);
	    axis_local[0] = Vector3::UnitZ();
	    axis_local[1] = Vector3::UnitX();
	    axis_local[2] = Vector3::UnitY();
	    axis_local[3] = Vector3::UnitY();
	    axis_local[4] = Vector3::UnitY();
	    axis_local[5] = Vector3::UnitX();
	
	    Vector3    pbase(0.0, 0.0, 0.0);
	    Quaternion qbase(1.0, 0.0, 0.0, 0.0);
	    for(int i = 0; i < 6; i++){
	        axis[i] = (i == 0 ? qbase : ori[i-1])*axis_local[i];
	        pos [i] = (i == 0 ? pbase : pos[i-1]) + (i == 0 ? qbase : ori[i-1])*pos_local[i];
	        ori [i] = (i == 0 ? qbase : ori[i-1])*AngleAxis(q[i], axis_local[i]);
	    }
	}
	```

	`pos_local[i]`は、第 $i-1$ 関節座標からみた第 $i$ 関節の座標の位置 $\boldsymbol{{}^{i-1}p_i}$ です。  
	ただし、 $i = 0$ のときは $\boldsymbol{{}^{0}p_0} = [0.0, 0.0, 0.0]^T$ です。

	今回の場合、脚の付け根から数えて2番目から3番目の関節の間に大腿があり、  
	3番目から4番目の関節の間に下腿があります。  
	大腿の長さは $\text{upper\_leg\_length} = 0.3$ 、  
	下腿の長さは $\text{lower\_leg\_length} = 0.4$ です。  
	したがって、 $\boldsymbol{{}^{2}p_3} = [0.0, 0.0, -\text{upper\_leg\_length}]^T$ 、  
	$\boldsymbol{{}^{3}p_4} = [0.0, 0.0, -\text{lower\_leg\_length}]^T$ となります。

	`axis_local[i]`は、第$i$関節の回転軸の方向で、  
	ロール・ピッチ・ヨー(UnitX・UnitY・UnitZ)から選択できます。
	
	プログラム26行目から始まるfor文内で順運動学を解きます。  
	ベースリンク基準の $i$ 番目リンクの座標位置 $\boldsymbol{{}^{0}p_i}$ (`pos` = `leg_pos`)は、  
	次のように計算されます。  
	$$\boldsymbol{{}^{0}p_i} = \boldsymbol{{}^{0}p_{i-1}} + \boldsymbol{{}^{0}Q_{i-1}} \cdot \boldsymbol{{}^{i-1}p_i} \cdot \boldsymbol{{}^{0}Q_{i-1}}^{-1}$$
	
	ベースリンク基準の $i$ 番目リンクの座標姿勢 $\boldsymbol{{}^{0}Q_i}$ (`ori` = `leg_ori`)は、  
	次のように計算されます。  
	$$\boldsymbol{{}^{0}Q_{i}} = \boldsymbol{{}^{0}Q_{i-1}} \cdot \boldsymbol{{}^{i-1}Q_{i}}$$  
	なお、四元数 $\boldsymbol{{}^{i-1}Q_{i}}$ はEigenのAngleAxisメソッドを使って、  
	回転量`q[i]`と回転軸`axis_local[i]`から生成されます。
	
-	**足裏中心の位置、姿勢を計算(132~134行目)**
	
	```cpp
	foot[i].ori   = base.ori*leg_ori[i][5];
	foot[i].angle = ToRollPitchYaw(foot[i].ori);
	foot[i].pos   = base.pos + base.ori*(leg_pos[i][5] + param.base_to_hip[i]) + foot[i].ori*param.ankle_to_foot[i];
	```
	
	前節で、ベースリンク座標を基準とした、  
	足の先端関節(くるぶし)までの位置・姿勢が計算できました。  
	ここでは、それをワールド座標系から見た位置・姿勢に変換します。
	
	ワールド座標を基準とした足裏中心座標の位置を $\boldsymbol{{}^Wp_E}$ (`foot[i].pos)`、  
	姿勢を $\boldsymbol{{}^WQ_E}$ (`foot[i].angle`)を計算します。
	
	ベースリンクから腰関節までのベクトルを $\boldsymbol{{}^Bp_0}$ `base_to_hip`、  
	くるぶしから足裏中心までのベクトルを $\boldsymbol{{}^5p_E}$ `ankle_to_foot`とします。  
	$\boldsymbol{{}^Wp_E}$、$\boldsymbol{{}^WQ_E}$はそれぞれ次のように計算されます。
	
	$$\boldsymbol{{}^Wp_E} = \boldsymbol{{}^Wp_0} + \boldsymbol{{}^WQ_0} \cdot (\boldsymbol{{}^0p_5} + \boldsymbol{{}^Bp_0}) \cdot \boldsymbol{{}^WQ_0}^{-1} + \boldsymbol{{}^WQ_E} \cdot \boldsymbol{{}^5p_E} \cdot \boldsymbol{{}^WQ_E}^{-1}$$
	
	$$\boldsymbol{{}^WQ_E} = \boldsymbol{{}^WQ_B} \cdot \boldsymbol{{}^BQ_0} \cdot \boldsymbol{{}^0Q_5}$$
	
---

## 例題1: 右足の先端位置を計算する

---

## 例題2: 重心位置を計算する

---

## まとめ・次回予告

今回はvnoidパッケージの順運動学ソルバーの理論と、  
サンプルコードについて解説しました。

次回：歩行ステップ計画
