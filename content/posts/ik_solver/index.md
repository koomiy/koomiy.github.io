---
title: "007 - 逆運動学"
date: 2023-08-24
categories: ["vnoidの解説"]
menu: main
---

---

## 今回の内容

本チャレンジでは、初心者の方でも簡単に人形ロボットの運動計画ができるように、  
vnoidというサンプルパッケージが用意されております。

今回はvnoidパッケージの機能の一つ、逆運動学ソルバー(iksolver)について解説します。  
[fksolver](https://koomiy.github.io/posts/fk_solver/)の記事をまだ見ていない方は、まずそちらから見ることをおすすめします。

---

## 逆運動学とは

逆運動学とは、先端効果器の位置・姿勢を与えたときの  
ロボットの関節変位を求める問題のことです。  
つまり、順運動学と逆のことをします。
	
逆運動学を解くことで、目標とする手足動作を実現できるような  
関節指令を送れるようになります。

---

## サンプルコードの解説

`vnoid/src/iksolver.cpp`を、脚の逆運動学計算を例に解説します。

-	**股関節基準の足首の目標位置・姿勢を計算する(~/iksolver.cpp`148~158行目)**

	```cpp {linenos=inline}
	void IkSolver::Comp(const Param& param, const Base& base, const vector<Hand>& hand, const vector<Foot>& foot, vector<Joint>& joint){
	    Vector3 pos_local;		// 腕や脚の付け根関節を基準とした手首・足首の目標位置
	    Quaternion ori_local;	// 腕や脚の付け根関節を基準とした手首・足首の目標姿勢

	    ...

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
	ワールド座標からベースリンクに基準を変換することにより、  
	次のようにできます。  
	$$ \boldsymbol{{}^Bp_5^{ref}} = \boldsymbol{{}^Bp_5^{ref}} - \boldsymbol{{}^Bp_B^{ref}} = \boldsymbol{{}^Bp_5^{ref}} \qquad \Box $$
	
	続く151行目で、脚の付け根関節基準の足首の目標姿勢を計算します。  
	$$ \boldsymbol{{}^0Q_5^{ref}} = \boldsymbol{\overline{{}^WQ_B^{ref}}} \cdot \boldsymbol{{}^WQ_E^{ref}} $$
	
	> 151行目の式が成り立つことを証明しておきます。  
	両辺に $ \boldsymbol{{}^BQ_0} = [1.0, 0.0, 0.0, 0.0]^T $ をかけます。  
	すると右辺には変化がありませんが、  
	左辺はベースリンク基準の足首の目標姿勢という意味になります。  
	$$ \boldsymbol{{}^BQ_5^{ref}} = \boldsymbol{\overline{{}^WQ_B^{ref}}} \cdot \boldsymbol{{}^WQ_E^{ref}} $$
	> 
	> 続いて、右辺についてですが、 $ \boldsymbol{\overline{{}^WQ_B^{ref}}} $ を逆からかけることにより、  
	ワールド座標からベースリンク基準に変換します。  
	また、上述したように $ \boldsymbol{{}^WQ_E^{ref}} = \boldsymbol{{}^WQ_5^{ref}} $ が成り立つので、  
	右辺の意味はベースリンク基準の足首の目標姿勢となり、左辺と一致します。

-	**脚の逆運動学(~/iksolver.cpp`11~66行目)**
	
	```cpp {linenos=inline}
	void IkSolver::CompLegIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double* q){
    	// hip pitch and knee pitch from trigonometrics
    	double d = pos.norm();
    	double c = (l1*l1 + l2*l2 - d*d)/(2*l1*l2);
    	double beta;
    
    	//  singularity: too close
    	if(c > 1.0){
            beta     = 0.0;
    	}
    	//  singularity: too far
    	else if(c < -1.0){
            beta     = pi;
    	}
    	//  nonsingular
    	else{
            beta  = acos(c);
    	}

    	q[3] = pi - beta;

    	Quaternion qinv =   ori.conjugate();
    	Vector3    phat = -(qinv*pos);
    	Quaternion qhat =   qinv;

    	// ankle pitch
    	Vector3 phatd(-l1*sin(q[3]), 0.0, l1*cos(q[3]) + l2);
    	double  c2 = phat.x()/sqrt(phatd.x()*phatd.x() + phatd.z()*phatd.z());
    	double  gamma;
    	if(c2 > 1.0){
            gamma = 0.0;
    	}
    	else if(c2 < -1.0){
            gamma = pi;
    	}
    	else{
            gamma = acos(c2);
    	}

    	double alpha = atan2(phatd.z(), phatd.x());
    	q[4] = -alpha + gamma;
    
    	// hip pos expressed in ankle pitch local
    	Vector3 phatdd = AngleAxis(-q[4], Vector3::UnitY())*phatd;

    	// ankle roll
    	q[5] = -atan2(phat.z(), phat.y())
              + atan2(phatdd.z(), phatdd.y());
    	if(q[5] >  pi) q[5] -= 2.0*pi;
    	if(q[5] < -pi) q[5] += 2.0*pi;

    	// desired hip rotation
    	Quaternion qyy(AngleAxis(q[3] + q[4], Vector3::UnitY()));
    	Quaternion qyyx   = qyy*AngleAxis(q[5], Vector3::UnitX());
    	Quaternion qhip   = ori*qyyx.conjugate();
    	Quaternion qzquad(AngleAxis(pi/2.0, Vector3::UnitZ()));

    	// convert it to roll-pitch-yaw
    	Vector3 angle_hip = ToRollPitchYaw(qzquad*qhip*qzquad.conjugate());

    	// then wrist angles are determined
    	q[0] =  angle_hip.z();
    	q[1] =  angle_hip.y();
    	q[2] = -angle_hip.x();
	}
	```
	
	　上記で求めた股関節基準の足首の目標位置 $\boldsymbol{p}$ ・姿勢 $\boldsymbol{Q}$ の情報を  
	CompLegIK関数に渡すことで逆運動学を解き、各関節の角度を計算します。
	
	　まず、ひざ関節の角度 $\theta_3$ を計算します。  
	{{<figure src="./beta.png" class="center" alt="beta" width="50%">}}  
	股関節から足首の目標位置までを直線で結んだ線分を $L$ とします。  
	線分 $L$ の長さを $d$ とします。このとき、 $d = ||\boldsymbol{p}||$ です。  
	線分 $L$ と大腿、下腿からなる三角形に注目します。  
	この三角形の $\beta$ の角度は第二余弦定理より、次のように計算できます。  
	$$ \beta = \mathrm{acos}(\frac{l_1^2 + l_2^2 - d^2}{2l_1l_2}) $$  
	よって、膝関節の角度 $\theta_3$ は次のように求まります。  
	$$ \theta_3 = \pi - \beta $$
	
	　次に、足首のピッチ角 $\theta_4$を計算します。  
	そのために、足首を基準とした股関節の位置・姿勢について考えます。  
	いま、足首を基準とした股関節は実際には $\boldsymbol{\hat{p}}$ に位置します。  
	$\boldsymbol{\hat{p}}$ は、 $\boldsymbol{p}$ や $\boldsymbol{Q}$ と次のような関係があります。  
	$$ \boldsymbol{\hat{p}} = \boldsymbol{\overline{Q}} \cdot \boldsymbol{p} \cdot \boldsymbol{Q} $$  
	足首がピッチ、ロール方向ともに回転しない場合において、  
	足首基準の股関節は以下図のように $\boldsymbol{\hat{p}'}$ に位置します。  
	{{<figure src="./phatd.png" class="center" alt="phatd" width="50%">}}  
	この状態から、足首をピッチ方向に $\theta_4$ だけ回転させると、  
	股関節は以下図のように位置 $\boldsymbol{\hat{p}''}$ に来ます。  
	{{<figure src="./phatdd.png" class="center" alt="phatdd" width="60%">}}  
	この図から分かるように、 $\theta_4$ は $\alpha$ と $\gamma$ の角度が分かれば求めることができます。  
	$$ \theta_4 = -\alpha + \gamma $$  
	ここで、図中の $\alpha$ の角度は次のように求められます。  
	$$ \alpha = \mathrm{atan2}(\hat{p}_z, \hat{p}_x) $$  
	さらに、 $\gamma$ の角度を求めていきます。  
	先程の状態から更に足首をロール方向に $\theta_5$ だけ回転させると、  
	股関節の位置は $\boldsymbol{\hat{p}}$ と一致します。  
	{{<figure src="./phat.png" class="center" alt="phat" width="45%">}}  
	xz平面でこれを見ると、以下図のようになり、  
	$\boldsymbol{\hat{p}}$ と $\boldsymbol{\hat{p}''}$ のx座標が一致することがわかります。
	{{<figure src="./gamma.png" class="center" alt="gamma" width="40%">}}  
	このことを利用して、図中の $\gamma$ の角度を求めることができます。  
	$$ \gamma = \mathrm{acos}(\frac{\hat{p}_x}{d}) $$  
	以上により $\alpha$ と $\gamma$ が求まったので、 $\theta_4$ が決定します。
	
	　次に、足首のロール角 $\theta_5$ を計算します。  
	以下図のように脚をyz平面に投影すると、ある角度だけ傾いた線分が得られます。  
	{{<figure src="./theta5.png" class="center" alt="theta5" width="40%">}}  
	このときの傾きが、足首のロール角 $\theta_5$ です。  
	よって $\theta_5$ は次のように求まります。  
	$$ \theta_5 = -\mathrm{atan2}(\hat{p}_z, \hat{p}_y) $$
	
	　最後に、股関節のヨー角 $\theta_0$ 、ピッチ角 $\theta_1$ 、ロール角 $\theta_2$ を計算します。  
	これらの回転角度をまとめて $\boldsymbol{\phi} = [\theta_0, \theta_1, \theta_2]^T$ とします。  
	股関節を基準とした足首の姿勢 $\boldsymbol{Q}$ は、  
	各関節の回転 $\boldsymbol{Q_i}$ によってもたらされるので、  
	次のような関係が成立します。  
	$$ \boldsymbol{Q} = \boldsymbol{Q_0} \cdot \boldsymbol{Q_1} \cdot \boldsymbol{Q_2} \cdot \boldsymbol{Q_3} \cdot \boldsymbol{Q_4} \cdot \boldsymbol{Q_5} $$  
	よって、股関節における姿勢変化 $\boldsymbol{Q_{hip}}$ は次のように計算できます。  
	$$ \boldsymbol{Q_{hip}} = \boldsymbol{Q_0} \cdot \boldsymbol{Q_1} \cdot \boldsymbol{Q_2} = \boldsymbol{Q} \cdot \overline{(\boldsymbol{Q_3} \cdot \boldsymbol{Q_4} \cdot \boldsymbol{Q_5})} $$  
	姿勢の回転を表現する四元数から、  
	回転角度に変換する関数`ToRollPitchYaw`を用いれば、  
	$\boldsymbol{Q_{hip}}$ から $\boldsymbol{\phi}$ を求めることができます。  
	ただし、このロボットの股関節はyaw, roll, pitchの順に並んでおり、  
	`ToRollPitchYaw`関数の仕様に合わせるためには、  
	yaw, pitch, rollの順に並べ替える必要があります。  
	並べ替える前は次のような関係が成り立っていますが、  
	$$ \boldsymbol{Q_z(\theta_0)} \cdot \boldsymbol{Q_x(\theta_1)} \cdot \boldsymbol{Q_y(\theta_2)} = \boldsymbol{Q_{hip}} $$  
	これをyaw, pitch, rollの順に並べ替えるには次のような操作をします。  
	$$ \boldsymbol{Q_z(\theta_0)} \cdot \boldsymbol{Q_y(\theta_1)} \cdot \boldsymbol{Q_x(-\theta_2)} = \boldsymbol{Q_z(\pi/2)} \cdot \boldsymbol{Q_{hip}} \cdot \boldsymbol{Q_z(-\pi/2)} =: \boldsymbol{Q_{hip}'} $$  
	変換後の $\boldsymbol{Q_{hip}'}$ を`ToRollPitchYaw`関数に代入することで、  
	股関節の角度リスト $\boldsymbol{\phi}$ を得ます。  
	$$ \boldsymbol{\phi} = \mathrm{ToRollPitchYaw}(\boldsymbol{Q'_{hip}}) $$  
	これにより、 $\theta_0 = \phi_z$ 、 $\theta_1 = \phi_y$ 、 $\theta_2 = -\phi_x$ と求まります。
	
	以上の計算により、股関節を基準とした足首の位置・姿勢情報から、  
	脚の関節角度を逆算することができました。  
	このような計算を逆運動学を解くといいます。
	
-	**[補足] 逆運動学で得られた重心を目標重心に一致させる(`~/iksolver.cpp`127行目~)**

	人型ロボットの目標重心をせっかく計画しても、  
	正確に追従できなければ意味がありません。  
	しかし困ったことに手や足のモーションは  
	重心を目標に一致させることを考えずに計画するので、  
	逆運動学を解いた結果必ずしも重心は目標と一致しません。
	
	そこで本iksolverでは、  
	目標手足位置・姿勢と目標重心を同時に満たすような  
	関節角の解を求める工夫をしております。
	
	事前に目標重心位置が与えられているとします。

	はじめはベースリンクの目標位置と重心の目標位置を一致させて、  
	逆運動学を解きます。  
	そうして得られた関節角をもとに、fksolverを用いて重心位置を計算します。  
	そうすると、重心と目標重心の位置に大なり小なり誤差が生じます。  
	その誤差をベースリンクの目標位置にフィードバックした上で、  
	再度逆運動学を解きます。
	
	この操作を繰り返し、誤差が無視できるほど小さくなる場合の解を採用します。  
	これにより最終的には重心が目標重心に一致することになります。

---

## 例題: ヨガのポーズ

以前投稿した[005 - 関節を動かしてみる](https://koomiy.github.io/posts/footstep_planning/)  
のヨガのポーズをiksolverを使って再現してみます。

今回の例題は、[005 - 関節を動かしてみる](https://koomiy.github.io/posts/footstep_planning/)の記事を一通り終えたところからの続きです。  
前回は、関節の目標角度を与えてポーズを取らせていましたが、  
今回は、手足の目標位置・姿勢および目標重心を与えてポーズを取らせてみましょう。

そのためにはヨガのポーズをとっているときの  
手足の位置・姿勢および重心を確認する必要があります。  
そこで、fksolverを使ってポーズ中の手足の位置・姿勢および重心を計算し、  
その情報を出力します。  

fksolverのコメントアウトを外し、  
その後ろに以下のプログラムを記述すると、  
ヨガのポーズをとっているときの手足の位置・姿勢および重心を確認できます。

```cpp {linenos=inline}
void MyRobot::Control(){

    ...
    
    // calc FK
    fk_solver.Comp(param, joint, base, centroid, hand, foot);
    
    // check hand
    printf("right hand_pos: %lf, %lf, %lf\n",
           hand[0].pos.x(),
           hand[0].pos.y(),
           hand[0].pos.z());
    printf("right hand_ori: %lf, %lf, %lf, %lf\n",
           hand[0].ori.w(),
           hand[0].ori.x(),
           hand[0].ori.y(),
           hand[0].ori.z());
    printf("left hand_pos: %lf, %lf, %lf\n",
           hand[1].pos.x(),
           hand[1].pos.y(),
           hand[1].pos.z());
    printf("left hand_ori: %lf, %lf, %lf, %lf\n",
           hand[1].ori.w(),
           hand[1].ori.x(),
           hand[1].ori.y(),
           hand[1].ori.z());

    // check foot
    printf("right foot_pos: %lf, %lf, %lf\n",
           foot[0].pos.x(),
           foot[0].pos.y(),
           foot[0].pos.z());
    printf("right foot_ori: %lf, %lf, %lf, %lf\n",
           foot[0].ori.w(),
           foot[0].ori.x(),
           foot[0].ori.y(),
           foot[0].ori.z());
    printf("left foot_pos: %lf, %lf, %lf\n",
           foot[1].pos.x(),
           foot[1].pos.y(),
           foot[1].pos.z());
    printf("left foot_ori: %lf, %lf, %lf, %lf\n",
           foot[1].ori.w(),
           foot[1].ori.x(),
           foot[1].ori.y(),
           foot[1].ori.z());
    
    // check com
    printf("com: %lf, %lf, %lf\n", centroid.com_pos.x(), centroid.com_pos.y(), centroid.com_pos.z());
    
    ...
    
}   
```

choreonoidを実行すると、以下のように出力が返ってくるはずです。  

```
right hand_pos: 0.122560, 0.000257, 1.276572
right hand_ori: 0.002227, -0.000196, -0.999997, 0.001291
left hand_pos: 0.122560, -0.000257, 1.276572
left hand_ori: 0.002227, 0.000196, -0.999997, -0.001291
right foot_pos: -0.000000, 0.036232, 0.547969
right foot_ori: 0.500898, 0.499100, 0.499100, -0.500898
left foot_pos: -0.000000, 0.100000, 0.150000
left foot_ori: 1.000000, 0.000000, -0.000000, 0.000000
com: 0.017037, -0.005541, 1.056449
```

これで、ポーズ中の手足の位置・姿勢および重心が分かりました。  
いよいよ関節目標角の設定部分を消去し、  
以下のように目標手足位置・姿勢および重心を設定してみます。

```cpp {linenos=inline}
void MyRobot::Control(){

    ...
    
    // set hand
    hand[0].pos_ref = Vector3(0.122560, 0.000257, 1.276572);
    hand[0].ori_ref = Quaternion(0.002227, -0.000196, -0.999997, 0.001291);
    hand[1].pos_ref = Vector3(0.122560, -0.000257, 1.276572);
    hand[1].ori_ref = Quaternion(0.002227, 0.000196, -0.999997, -0.001291);

    // set foot
    foot[0].pos_ref = Vector3(-0.000000, 0.036232, 0.547969);
    foot[0].ori_ref = Quaternion(0.500898, 0.499100, 0.499100, -0.500898);
    foot[1].pos_ref = Vector3(-0.000000, 0.100000, 0.150000);
    foot[1].ori_ref = Quaternion(1.000000, 0.000000, -0.000000, 0.000000);
    
    // set com
    centroid.com_pos_ref = Vector3(0.017037, -0.005541, 1.056449);
    
    // calc CoM IK
    ik_solver.Comp(&fk_solver, param, centroid, base, hand, foot, joint);
    
    ...
    
}
```

choreonoidを実行してみると、以下のように動きます。  
{{<figure src="./yoga.gif" class="center" alt="yoga" width="50%">}}  
現状、なぜか腕の関節がありえない角度にねじれていきますが、  
脚の方はいい感じにヨガのポーズを再現できていますね！

(この腕の関節がおかしくなる現象については後日対処する予定です。)

---

## まとめ・次回予告

今回はvnoidパッケージの逆運動学ソルバー、iksolverについて解説しました。

次回はステップ計画について解説しようと思います。

次回： [008 - ステップ計画](https://koomiy.github.io/posts/footstep_planner/)
