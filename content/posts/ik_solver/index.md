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

-	**逆運動学のおおまかな流れ(127行目~)**
	
	事前に目標重心位置が与えられているとします。

	はじめは、ベースリンクの目標位置と重心の目標位置を一致させて逆運動学を解きます。  
	そうして得られた関節角をもとに、fksolverを用いて重心位置を計算します。  
	そうすると、重心と目標重心の位置に大なり小なり誤差が生じます。  
	その誤差をベースリンクの目標位置にフィードバックした上で、再度逆運動学を解きます。  
	この操作を繰り返し、誤差が無視できるほど小さくなる場合の解を採用します。

-	**腕の逆運動学(136~146行目、68~129行目)**

-	**脚の逆運動学(148~158行目、11~66行目)**

	~ STEP 1 「股関節基準の足首の目標位置・姿勢を計算する」 ~

	```cpp {.line-numbers .startFrom="132"}
	void IkSolver::Comp(const Param& param, const Base& base, const vector<Hand>& hand, const vector<Foot>& foot, vector<Joint>& joint){
	    Vector3 pos_local;		// 腕や脚の付け根関節を基準とした手首・足首の目標位置
	    Quaternion ori_local;	// 腕や脚の付け根関節を基準とした手首・足首の目標姿勢
	```

	```cpp 

	    ...

	```

	```{r, attr.output=".numberLines startFrom='150'", result='hold'}
	    for(int i = 0; i < 2; i++){
                pos_local = base.ori_ref.conjugate()*(foot[i].pos_ref - foot[i].ori_ref*param.ankle_to_foot[i] - base.pos_ref) - param.base_to_hip[i];
                ori_local = base.ori_ref.conjugate()* foot[i].ori_ref;
                
                CompLegIk(pos_local, ori_local, param.upper_leg_length, param.lower_leg_length, q);
                
                for(int j = 0; j < 6; j++){
                    joint[param.leg_joint_index[i] + j].q_ref = q[j];
                }
            }
	}
	```
	
	150行目で、脚の付け根関節基準の足首の目標位置を計算します。  
	$$ \boldsymbol{{}^0p_5^{ref}} = \boldsymbol{\overline{{}^WQ_B}} \cdot (\boldsymbol{{}^Wp_E^{ref}} - \boldsymbol{{}^WQ_E^{ref}} \cdot \boldsymbol{{}^5p_E} \cdot \boldsymbol{\overline{{}^WQ_E^{ref}}} - \boldsymbol{{}^Wp_B^{ref}}) \cdot \boldsymbol{{}^WQ_B} - \boldsymbol{{}^Bp_0} $$

	> 150行目の式が成り立つ証明をしておきます。  
	右辺の $\boldsymbol{{}^Bp_0}$ を左辺へ移行すると、  
	左辺はベースリンク基準のくるぶしの目標位置となります。  
	$$ \boldsymbol{{}^Bp_5^{ref}} = \boldsymbol{\overline{{}^WQ_B}} \cdot (\boldsymbol{{}^Wp_E^{ref}} - \boldsymbol{{}^WQ_E^{ref}} \cdot \boldsymbol{{}^5p_E} \cdot \boldsymbol{\overline{{}^WQ_E^{ref}}} - \boldsymbol{{}^Wp_B^{ref}}) \cdot \boldsymbol{{}^WQ_B} $$
	> 
	> 続いて、右辺の $ \boldsymbol{{}^Wp_E^{ref}} - \boldsymbol{{}^WQ_E^{ref}} \cdot \boldsymbol{{}^5p_E} \cdot \boldsymbol{\overline{{}^WQ_E^{ref}}} $ についてですが、  
	これはワールド座標基準の足首の目標位置と等価なので  
	(∵足は足首に対して姿勢変化しないので、$ \boldsymbol{{}^WQ_E^{ref}} = \boldsymbol{{}^WQ_5^{ref}} $)、  
	次のようにできます。  
	$$ \boldsymbol{{}^Bp_5^{ref}} = \boldsymbol{\overline{{}^WQ_B}} \cdot (\boldsymbol{{}^Wp_5^{ref}} - \boldsymbol{{}^Wp_B^{ref}}) \cdot \boldsymbol{{}^WQ_B} $$
	> 
	> 最後に、右辺の $ \boldsymbol{{}^Wp_5^{ref}} - \boldsymbol{{}^Wp_B^{ref}} $ について、  
	これはワールド座標基準のベースリンクから足首までの目標相対位置です。  
	よって、 $ \boldsymbol{{}^WQ_B} $ を逆からかけ、  
	ワールド座標からベースリンクに基準を変換することにより、次のようにできます。  
	$$ \boldsymbol{{}^Bp_5^{ref}} = \boldsymbol{{}^Bp_5^{ref}} - \boldsymbol{{}^Bp_B^{ref}} = \boldsymbol{{}^Bp_5^{ref}} \qquad \Box $$
	
	続く151行目で、脚の付け根関節基準の足首の目標姿勢を計算します。  
	$$ \boldsymbol{{}^0Q_5^{ref}} = \boldsymbol{\overline{{}^WQ_B^{ref}}} \cdot \boldsymbol{{}^WQ_E^{ref}} $$
	
	> 151行目の式が成り立つことを証明しておきます。  
	両辺に $ \boldsymbol{{}^BQ_0} = [1.0, 0.0, 0.0, 0.0]^T $ をかけます。  
	すると右辺には変化がありませんが、左辺はベースリンク基準の足首の目標姿勢という意味になります。  
	$$ \boldsymbol{{}^BQ_5^{ref}} = \boldsymbol{\overline{{}^WQ_B^{ref}}} \cdot \boldsymbol{{}^WQ_E^{ref}} $$
	> 
	> 続いて、右辺についてですが、 $ \boldsymbol{\overline{{}^WQ_B^{ref}}} $ を逆からかけることにより、  
	ワールド座標からベースリンク基準に変換します。  
	また、上述したように $ \boldsymbol{{}^WQ_E^{ref}} = \boldsymbol{{}^WQ_5^{ref}} $ が成り立つので、  
	右辺の意味はベースリンク基準の足首の目標姿勢となり、左辺と一致します。
	
	```cpp
	void IkSolver::CompLegIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double* q){
        Vector3 angle = ToRollPitchYaw(ori);

        // hip yaw is directly determined from foot yaw
        q[0] = angle.z();

        // ankle pos expressed in hip-yaw local
        Vector3 pos_local = AngleAxis(-q[0], Vector3::UnitZ())*pos;

        // hip roll
        q[1] = atan2(pos_local.y(), -pos_local.z());

        // ankle pos expressed in hip yaw and hip roll local
        Vector3 pos_local2 = AngleAxis(-q[1], Vector3::UnitX())*pos_local;

        double  alpha = -atan2(pos_local2.x(), -pos_local2.z());
    
        // hip pitch and knee pitch from trigonometrics
        double d   = pos.norm();
        double tmp = (l1*l1 + l2*l2 - d*d)/(2*l1*l2);
        //  singularity: too close
        if(tmp > 1.0){
            q[3] = pi;
            q[2] = alpha;
        }
        //  singularity: too far
        else if(tmp < -1.0){
            q[3] = 0.0;
            q[2] = alpha;
        }
        //  nonsingular
        else{
            q[3] = pi - acos(tmp);
            q[2] = alpha - asin((l2/d)*sin(q[3]));
        }

        Quaternion qzxyyy = AngleAxis(q[0],      Vector3::UnitZ())
                           *AngleAxis(q[1],      Vector3::UnitX())
                           *AngleAxis(q[2]+q[3], Vector3::UnitY());
        Quaternion qrel = qzxyyy.conjugate()*ori;
        Vector3 angle_rel = ToRollPitchYaw(qrel);

        q[4] = angle_rel.y();
        q[5] = angle_rel.x();

        /*
        // easy way, but not correct

        // ankle pitch
        q[4] = angle.y() - q[2] - q[3];

        // ankle roll
        q[5] = angle.x() - q[1];
        */
    
	}
	```
	
	150,151行目で脚の付け根関節基準の足首の目標位置・姿勢が計算できました。  
	この情報をCompLegIK関数に渡すことで逆運動学を解き、各関節の角度を計算します。
	
	脚のヨー・ロール角を決定するのが股関節しかないので、  
	15、22行目のように股関節のヨー角 $\theta_0$ 、 ロール角 $\theta_1$ を決定できます。
	
	次に25行目で、上記の股関節のヨー・ロール角の回転を戻した場合の  
	足首の位置 $\boldsymbol{p''}$ を計算します。  
	なお、この操作により脚はxz平面上に存在します。  
	よって次の図ように、平面幾何として逆運動学を考えることができます。  

	
	図中の $\alpha$ 、 $\beta$ 、 $\gamma$ の角度を次のように求めます。  
	$$ \alpha = - \mathrm{atan2}(p''_x, -p''_z) $$  
	$$ \beta = \mathrm{cos}\bigg(\frac{L_1^2 + L_2^2 - d^2}{2L_1L_2}\bigg) $$  
	$$ \gamma = \mathrm{asin}\bigg(\frac{L_2\mathrm{sin}(\beta)}{d}\bigg) $$
	
	股関節のピッチ角 $\theta_2$ は、 $\theta_2 = \alpha - \gamma$ と求まります。  
	膝関節の角度 $\theta_3$ は、 $\theta_3 = \pi - \beta$ と求まります。
	
	残りの足首関節のロール角 $\theta_4$ 、ピッチ角 $\theta_5$ は次のように計算します(47行目~)。  
	まず、$\theta_0$ 〜  $\theta_3$ 関節までの、股関節からの姿勢変化 $\boldsymbol{Q_{zxyy}}$ を計算します。  
	そして、姿勢$\boldsymbol{Q_{zxyy}}$ から足首の目標姿勢となるまでに  
	必要な姿勢変化 $\boldsymbol{Q_{rel}}$ を計算します。  
	$$ \boldsymbol{Q_{rel}} = \overline{\boldsymbol{Q_{zxyy}}} \cdot \boldsymbol{{}^0Q_5^{ref}} $$  
	姿勢 $\boldsymbol{Q_{rel}}$ のピッチ成分が $\theta_4$ となり、  
	ロール成分が $\theta_5$ となります。

-	**脚の関節トルクを計算する(200~229行目)**
	
	

---

## 例題: ヨガのポーズ



---

## まとめ・次回予告

今回はvnoidパッケージのiksolverについて解説しました。

次回は歩行パターン計画について解説しようと思います。

次回： [008 - 歩行パターン計画](https://koomiy.github.io/posts/footstep_planning/)
