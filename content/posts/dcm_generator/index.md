---
title: "009 - 目標ZMP・DCM計画"
date: 2023-11-30
categories: ["vnoidの解説"]
menu: main
---

(この記事は制作途中です)

---

## 今回の内容

本チャレンジでは、初心者の方でも簡単に人形ロボットの運動計画ができるように、  
vnoidというサンプルパッケージが用意されております。

今回はvnoidパッケージの機能の一つ、  
目標 ZMP・DCM 計画器(footstep_planner::GenerateDCM)について解説します。

はじめに、人型ロボットの運動計画を語るうえで欠かせない概念の説明をし、  
そのあとで本題に入ろうと思います。

---

## 人型ロボットの運動計画法

人型ロボットはいくつものリンクからなる剛体リンク系ですが、  
一つ一つのリンクに関して運動を考えるのは非常に難しいです。

そこで、重心と ZMP を用いて人型ロボットを倒立振り子に簡単化してしまいます。  
特に、重心周りの回転運動を無視したものを線形倒立振り子といいます。  
ここで、人型ロボットの重心が線形倒立振り子運動することを、  
Linear Inverted Pendulum Mode (LIPM) で運動するといいます。

LIPM の重心運動方程式は次式で表されます。  
$$ \boldsymbol{\ddot{x}} = \frac{1}{T^2} (\boldsymbol{x} - \boldsymbol{p}) $$
$$ T = \sqrt{\frac{h}{g}} $$
ここで、$\boldsymbol{x}$ は重心位置、$\boldsymbol{p}$ は ZMP です。  
また $T$ は、重心高さ $h$ と重力加速度 $g$ で決まる倒立振り子運動の時定数です。

この運動方程式を満たす重心運動には次のような特徴があります。  
・ロボットが重心の単質点系で表現される  
・接地期間中における ZMP を基準とした重心の高さは一定である  
・下図のように、ZMP に作用する床反力の延長線上に常に重心が存在する

{{<figure src="./lipm.png" class="center" alt="LIPM" width="50%">}}

LIPM に従う重心と ZMP の計画をして、  
それに辻褄が合うようにベースリンクや手足のポーズを計算することで  
人型ロボットの歩行運動を簡単に計画しよう、  
というのが、vnoidがやろうとしていることです。

---

## ZMPとは

ZMP とは、Zero Moment Point の略語であり、  
文字通りモーメントが0になる点です。

ロボットの足裏には床からの分布反力が作用します。  
その分布力の合計を単に床反力と呼びます。  
床反力によって発生するモーメントのうち、  
鉛直軸周り以外の成分が0となる点が ZMP です。

この定義から、ZMP は足裏に作用する分布力の圧力中心  
CoP (Center of Pressure)と一致します。  
このことからも分かるように、  
ZMP は必ず支持多角形の中に存在します。  
したがって目標 ZMP を計画する際は、  
必ず支持多角形の中に入るように計画しましょう。

---

## 支持多角形とは

支持多角形とは、足裏の床面との接触点を  
ゴム紐でひっかけてできる多角形のことです。  
数学的には、ロボットと床面の接触点集合が作る凸包と定義されます。

{{<figure src="./sfp.png" class="center" alt="Support Foot Polygon" width="50%">}}

---

## DCMとは

DCM とは、Divergent Componet of Motion の略語であり、  
LIPMの重心運動の不安定成分です。  
少しイメージしづらいですが、LIPM の重心運動を  
理解するために重要な役割を果たす概念です。  
とにかく DCM は不安定な点と覚えてください。  
この DCM さえ安定化することができれば、  
LIPM の重心運動も安定化させられます。

DCM $\boldsymbol{\xi}$ は、次式で定義されます。  
$$ \boldsymbol{\xi} = \boldsymbol{x} + T\boldsymbol{\dot{x}} $$
DCM には、その点を瞬時に ZMP で踏めば自然に停止できるという性質があります。  
自然に停止するとは、十分な時間が経過したのちに  
ZMP の真上にロボットの重心が遷移するということです。  
このような状態に遷移すると着地点からの垂直床反力で重力を相殺でき、  
ロボットの重心が静的安定な状態になります。  
(この特徴から、DCMは捕まえれば安定する点として、  
Capture Point (CP) とも呼ばれます。)

LIPM の重心運動方程式に、DCM の定義式を代入すると、  
次のように二つの一階微分方程式に分解できます。  
$$ \boldsymbol{\dot{\xi}} = \frac{1}{T}(\boldsymbol{\xi} - \boldsymbol{p}) $$
$$ \boldsymbol{\dot{x}} = -\frac{1}{T}(\boldsymbol{x} - \boldsymbol{\xi}) $$
上の式は、DCM には、ZMP に対して反発するように運動する性質があることを示します。  
特にこの式を、DCM の運動方程式と呼びます。  
下の式は、重心には、DCM に安定追従するように運動する性質があることを示します。  
よって DCM が ZMP に反発して離れすぎてしまうと、重心運動も不安定化します。  
したがって、DCM さえ安定化させてしまえばLIPMの重心運動は安定化できるのです。

これらの性質を踏まえたうえで、我々が普段何気なくやっている歩行のさなか、  
ZMP や DCM がどのように運動しているかを想像してみましょう。  
普通一歩ごとに停止しないので、一歩一歩 DCM を踏むのではなく、  
DCM の安定を保てるように ZMP が DCM を追いかけていく状態がしばらく継続します。  
そして歩くのをやめたいときに、DCM を ZMP でとらえることで安定停止します。

(ZMP,DCM,重心がどう運動するかを示すgifを貼る)

---

## なぜDCMを使うのか？

人型ロボットの運動計画変数に DCM を加える理由は次の二つです。

1. 2階常微分方程式であるLIPMの重心運動を、  
二つの1階常微分方程式に分解し、問題を簡単化できる。

1. その方程式のうちの一つは DCM の運動方程式であり、  
これを用いることで簡単に[歩行安定化制御](hogehoge)を実装できる。

---

## サンプルコードの解説

```cpp {linenos=inline}
void FootstepPlanner::GenerateDCM(const Param& param, Footstep& footstep){
    // generate reference dcm and zmp 
	// dcm of step[0] should be already specified

    int nstep = footstep.steps.size();
		
	// set final step's state
	int i = nstep-1;
	// ZMP is in the middle of feet
	footstep.steps[i].zmp = (footstep.steps[i].foot_pos[0] + footstep.steps[i].foot_pos[1])/2.0;

	// DCM is com_height above ZMP
	footstep.steps[i].dcm = (footstep.steps[i].foot_pos[0] + footstep.steps[i].foot_pos[1])/2.0
		                  + Vector3(0.0, 0.0, param.com_height);
	i--;

	// calc N-1 to 0 step's state
	for( ; i >= 0; i--) {
		Step& st0 = footstep.steps[i+0];
		Step& st1 = footstep.steps[i+1];
			
		int sup =  st0.side;
		int swg = !st0.side;
		
		double a = exp(-st0.duration/param.T);
		// for initial step, the dcm is specified from outside. determine zmp accordingly
		if(i == 0){
			st0.zmp = (st0.dcm - a*st1.dcm)/(1.0 - a) - Vector3(0.0, 0.0, param.com_height);
		}
		// for other steps
		else{
			const double eps = 1.0e-3;
			// if swing foot position is not changing, treated as double support and set zmp to the middle of feet
			if( (st0.foot_pos  [swg] - st1.foot_pos  [swg]).norm() < eps &&
				(st0.foot_angle[swg] - st1.foot_angle[swg]).norm() < eps ){
				st0.zmp = (st0.foot_pos[sup] + st0.foot_pos[swg])/2.0;
			}
			// otherwise, set zmp to support foot
			else{
				st0.zmp = st0.foot_pos[sup];
			}

			// determine dcm from zmp
			st0.dcm = (1.0 - a)*(st0.zmp + Vector3(0.0, 0.0, param.com_height)) + a*st1.dcm;
		}

		// set stepping flag based on whether swing foot position is chanching or not
		if( (st0.foot_pos  [swg] - st1.foot_pos  [swg]).norm() < eps &&
			(st0.foot_angle[swg] - st1.foot_angle[swg]).norm() < eps ){
			st0.stepping = false;
		}
		else{
			st0.stepping = true;
		}

	}
}
```

簡単のために、片足支持中の目標ZMPを固定点とします。  
この仮定の下では、ZMP と DCM の間に次の漸化式が成立します。  
この漸化式は、任意の歩行ステップの初期時刻から終了時刻までの区間 $[t, t+\tau]$  
において、DCM の運動方程式を離散化することで得られます。  
$$ \boldsymbol{\xi_{i+1}} = e^{\frac{\tau}{T}}\boldsymbol{\xi_i} + (1 - e^{\frac{\tau}{T}})\boldsymbol{p_i} $$
ここで，$\boldsymbol{p_i}$, $\boldsymbol{\xi_i}$ はそれぞれ、  
$i$ステップ目の初期時刻における ZMP と DCM です。  
また、$\tau$ は一歩にかける時間、すなわち歩行期間です。

この漸化式に歩行の開始時と終了時の目標 ZMP および DCM を設定することで、  
歩行中の目標 ZMP と DCM を求めます(31~45行目)。  
歩行中、目標 ZMP は支持多角形の中に入ってほしいので、  
支持側の片足の目標着地位置と一致させます(39~41行目)。  
なお、着地位置がほぼ変化しない場合は両足支持とみなし、  
両足の着地位置の中間点を目標 ZMP とします(32~37行目)。  
得られた目標 ZMP を漸化式に代入することで、  
歩行中の目標 DCM を決定します(44行目)。

歩行開始時の目標 ZMP、DCM について考えます。  
歩行は重心が静止した状態から始まるので、  
歩行開始時の目標 DCM を目標重心位置と一致させなければなりません。  
目標重心位置は外部で決められるので、本プログラム実行前に  
歩行開始時の目標 DCM は既に決められています。

歩行開始時の目標 ZMP は、この目標 DCM に合わせるように、  
上記の漸化式に基づいて決められます(27~29行目)。  
この方法では、進行方向と逆向きに少しずれた位置に目標 ZMP が生成されます。  
その結果、重心を進行方向に加速させる効果をもたらします。

歩行終了時の目標 ZMP、DCM について考えます。  
歩行を終了するときには、重心を静止させなければならないので、  
目標DCMを目標重心位置と一致させる必要があります(13~14行目)。  
また、安定静止するには、目標 ZMP を 目標 DCM と一致させる必要があります。  
ここで、歩行終了時の目標重心位置は、  
両足の目標着地位置の中間点から、目標重心高さ分だけ高い位置です。

(計画されるZMP・DCMの図をここに貼る)


**注意書き**  
本プログラムにおける DCM は、  
階段や凹凸面などの三次元空間における歩行計画を実現するために、  
水平な床の二次元平面における点としてではなく、  
三次元空間上の点として概念を拡張させています。  
目標 DCM のz成分が、目標重心高さに設定されているのはそのためです。

また、それに対応させるように、ZMP の概念も三次元に拡張させています。  
プログラム上で ZMP 変数自体は水平面上の点という扱いですが、  
ZMP を用いて DCM の計算をするときには、  
元の ZMP に重心高さ分だけオフセットした点を入力しています。  
ZMP を三次元に拡張するために、ZMP に重心高さ分だけオフセットした点を、  
VRP(Virtual Repellent Point) といいます。  
日本語訳すると、仮想反発点といったところでしょうか。  
おそらく、ZMP が DCM を寄せ付けないという性質から、  
三次元に拡張したときにこう名付けられたのだと思われます(個人的な解釈です)。

---

## まとめ・次回予告

今回はvnoidパッケージの目標ZMP・DCM計画  
(footstep_planner::generateDCM) について解説しました。

次回は歩行制御(stepping_controller)について解説しようと思います。

次回： [012 - 歩行制御](https://koomiy.github.io/posts/stepping_controller/)
