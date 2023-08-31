---
title: "007 - 逆運動学"
date: 2023-08-24
categories: ["vnoidベースの開発"]
menu: main
---

(こちらは制作中の記事です)

---

## 今回の内容

本チャレンジでは、初心者の方でも簡単に人形ロボットの運動計画ができるように、  
vnoidというサンプルパッケージが用意されております。

今回はvnoidパッケージの機能の一つ、逆運動学ソルバー(iksolver)について解説します。  
[fksolver](https://koomiy.github.io/posts/fk_solver/)の記事をまだ見ていない方は、  
まずそちらから見ることをおすすめします。

---

## 逆運動学とは

逆運動学とは、先端効果器の位置・姿勢を与えたときの  
ロボットの関節変位を求める問題のことです。  
つまり、順運動学と逆のことをします。
	
逆運動学を解くことで、目標とする手足動作を実現できるような  
関節指令を送れるようになります。

---

## サンプルコードの解説

`vnoid/src/iksolver.cpp`を、脚の逆運動学計算を例に解説します(プログラム161行目~)。

-	**変数の定義**

	まず変数の定義についてです。
	
	`joint_tmp`、`hand_tmp`、`foot_tmp`は、
	
-	**逆運動学のおおまかな流れ(127行目~)**
	
	事前に目標重心位置が与えられているとします。
	
	はじめは、ベースリンクの目標位置と重心の目標位置を一致させて逆運動学を解きます。  
	そうして得られた関節角をもとに、fksolverを用いて重心位置を計算します。  
	そうすると、重心と目標重心の位置に大なり小なり誤差が生じます。  
	その誤差をベースリンクの目標位置にフィードバックした上で、再度逆運動学を解きます。  
	この操作を繰り返し、誤差が無視できるほど小さくなる場合の解を採用します。

-	**脚の逆運動学(148~158行目、11~66行目)**
	
	150行目で、脚の付け根関節基準のくるぶしの目標位置を計算します。  
	$$ \boldsymbol{{}^0p_5^{ref}} = \boldsymbol{\overline{{}^WQ_B}} \cdot (\boldsymbol{{}^Wp_E^{ref}} - \boldsymbol{{}^WQ_E^{ref}} \cdot \boldsymbol{{}^5p_E} \cdot \boldsymbol{\overline{{}^WQ_E^{ref}}} - \boldsymbol{{}^Wp_B^{ref}}) \cdot \boldsymbol{{}^WQ_B} - \boldsymbol{{}^Bp_0} $$
	
	この式が成り立つ証明をしておきます。  
	右辺の $\boldsymbol{{}^Bp_0}$ を左辺へ移行すると、  
	左辺はベースリンク基準のくるぶしの目標位置となります。  
	$$ \boldsymbol{{}^Bp_5^{ref}} = \boldsymbol{\overline{{}^WQ_B}} \cdot (\boldsymbol{{}^Wp_E^{ref}} - \boldsymbol{{}^WQ_E^{ref}} \cdot \boldsymbol{{}^5p_E} \cdot \boldsymbol{\overline{{}^WQ_E^{ref}}} - \boldsymbol{{}^Wp_B^{ref}}) \cdot \boldsymbol{{}^WQ_B} $$
	
	続いて、右辺の $ \boldsymbol{{}^Wp_E^{ref}} - \boldsymbol{{}^WQ_E^{ref}} \cdot \boldsymbol{{}^5p_E} \cdot \boldsymbol{\overline{{}^WQ_E^{ref}}} $ についてですが、  
	これはワールド座標基準のくるぶしの目標位置と等価なので  
	(∵足はくるぶしに対して姿勢変化しないので、$ \boldsymbol{{}^WQ_E^{ref}} = \boldsymbol{{}^WQ_5^{ref}} $)、次のようにできます。  
	$$ \boldsymbol{{}^Bp_5^{ref}} = \boldsymbol{\overline{{}^WQ_B}} \cdot (\boldsymbol{{}^Wp_5^{ref}} - \boldsymbol{{}^Wp_B^{ref}}) \cdot \boldsymbol{{}^WQ_B} $$
	
	最後に、右辺の $ \boldsymbol{{}^Wp_5^{ref}} - \boldsymbol{{}^Wp_B^{ref}} $ について、  
	これはワールド座標基準のベースリンクからくるぶしまでの目標相対位置です。  
	よって、 $ \boldsymbol{{}^WQ_B} $ を逆からかけることにより、ワールド座標からベースリンクに基準を変換すると、次のようにできます。  
	$$ \boldsymbol{{}^Bp_5^{ref}} = \boldsymbol{{}^Bp_5^{ref}} - \boldsymbol{{}^Bp_B^{ref}} = \boldsymbol{{}^Bp_5^{ref}} \Box $$
	
	続く151行目で、脚の付け根関節基準のくるぶしの目標姿勢を計算します。  
	$$ \boldsymbol{{}^0Q_5^{ref}} = \boldsymbol{\overline{{}^WQ_B^{ref}}} \cdot \boldsymbol{{}^WQ_E^{ref}}

-	**脚の関節トルクを計算する(200~229行目)**
	
	

---

## 例題: ヨガのポーズ



---

## まとめ・次回予告

今回はvnoidパッケージのiksolverについて解説しました。

次回は歩行パターン計画について解説しようと思います。

次回： [008 - 歩行パターン計画](https://koomiy.github.io/posts/footstep_planning/)
