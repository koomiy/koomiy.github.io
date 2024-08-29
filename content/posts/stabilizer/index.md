---
title: "013 - 安定化制御"
date: 2024-03-23
categories: ["vnoidの解説"]
menu: main
---

年度替わりに伴い、中の人が交代しています。文体等に違いがあるところがあるかと思いますが、ご了承いただければ幸いです。

---

## 今回の内容

本チャレンジでは、初心者の方でも簡単に人形ロボットの運動計画ができるように、  
vnoidというサンプルパッケージが用意されております。

今回はvnoidパッケージの機能の一つ、  
安定化制御器(Stabilizer)について解説します。

---

## 安定化制御の概要

これまでの記事で、人型ロボットを線形倒立振り子として目標運動を計画しましたが、  
残念ながら完璧にその通りに動作することはありません。  
計画運動からずれて不安定化してしまったときに安定化制御を行い、  
転倒を回避しなければなりません。

安定化制御では、ロボットの回転運動を抑えるための転倒回復モーメントを実現させます。  
そのために、目標着地位置を計画から修正します。  
これは、例えばあなたが電車に乗っていて、  
急ブレーキがかかると後ろに倒れそうになりますが、  
倒れる方向に足を踏み出して踏ん張る現象と同じです。

本記事では、その着地位置修正がどのような仕組みで行われているかを説明します。

---

## サンプルコードの解説

-   **安定化制御全体の流れ**

```cpp {linenos=inline}
void Stabilizer::Update(const Timer& timer, const Param& param, const Footstep& footstep_buffer, Centroid& centroid, Base& base, vector<Foot>& foot){
	const Step& stb0 = footstep_buffer.steps[0];
    const Step& stb1 = footstep_buffer.steps[1];
    int sup =  stb0.side;
    int swg = !stb0.side;

	// calc zmp from forces
    CalcZmp(param, centroid, foot);

	// error between desired and actual base link orientation
	Vector3 theta = base.angle  - base.angle_ref;
	Vector3 omega = base.angvel - base.angvel_ref;

	// calc base link tilt
	CalcBaseTilt(timer, param, base, theta, omega);
	
	// calc dcm and zmp 
	CalcDcmDynamics(timer, param, footstep_buffer, base, theta, omega, centroid);

	// calc desired force applied to CoM
	centroid.force_ref  = param.total_mass*(centroid.com_acc_ref + Vector3(0.0, 0.0, param.gravity));
	centroid.moment_ref = Vector3(0.0, 0.0, 0.0);

	// calculate desired forces from desired zmp
	CalcForceDistribution(param, centroid, foot);

	for(int i = 0; i < 2; i++){
		// ground reaction force control
		if( foot[i].contact ){
			for(int j = 0; j < 3; j++){
				dpos[i][j] += (-force_ctrl_damping*dpos[i][j] + force_ctrl_gain*(foot[i].force_ref[j] - foot[i].force[j]))*timer.dt;
				dpos[i][j] = std::min(std::max(-force_ctrl_limit, dpos[i][j]), force_ctrl_limit);

				drot[i][j] += (-moment_ctrl_damping*drot[i][j] + moment_ctrl_gain*(foot[i].moment_ref[j] - foot[i].moment[j]))*timer.dt;
				drot[i][j] = std::min(std::max(-moment_ctrl_limit, drot[i][j]), moment_ctrl_limit);
			}

			// feedback to desired foot pose
			foot[i].pos_ref   += -dpos[i];
			foot[i].angle_ref += -drot[i];
            foot[i].ori_ref = FromRollPitchYaw(foot[i].angle_ref);
		}
	}

}
```

stabilizerでもfootstep_plannerで行ったのと同じように、参照変数として`st0`、`st1`を定義します。
`st0`や`st1`には、両足分の着地位置・姿勢情報を代入します。
これもfootstep_plannerと同じですね。	  
そのため、ここでも左右のうちどちらが支持足で、  
どちらが振り足かを見分けておく必要があります。  
そこでこれまたfootstep_plannerと同じように`sup`と`swg`変数を用意します。  
`sup`が支持足であることを意味し、  
`swg`が振り足であることを意味します。
次にCalcZmpという関数でZMPの位置を算出します。
ZMPについては[009 - 目標DCM計画器](https://koomiy.github.io/posts/dcm_generator/)で、
詳しく説明していますので、そちらをご覧ください。
関数の中身については、下に続く各項目にて解説しています!!
250行目と251行目ではベースリンクの角度と角速度を目標値との誤差を計算しています。
これらの誤差を用いて、254行目にあるCalcBaseTiltという関数を使ってベースリンクの傾きを計算しています。
257行目のCalcDcmDynamicsではDCMや重心の運動について計算されています。
260行目と261行目ではそれらの結果から重心に作用されるべき床反力とモーメントを計算しています。
デフォルトのままでは、目標モーメントの値が各軸で0になっているため、
現状の重心などの状態は反映された目標値になっていません。
264行目のCalcForceDistributionでは目標ZMPから作用する床反力を計算しています。
そのあとのfor文の中で目標とする床反力を制御するために、歩行計画で予定していた位置などからずれて
「どこに」、「どんな角度で」、「どのような姿勢で」足をつけばいいかを計算しています。
最後に278行目から280行目でその修正量を歩行計画で算出した値に
加えることで着地位置修正を考慮した新たな着地位置等をロボットに入力することになります。

では、次は各関数の中でどのようなことが行われているのかを見ていきましょう!!

---

-   **CalcZmp**

```cpp {linenos=inline}
	void Stabilizer::CalcZmp(const Param& param, Centroid& centroid, vector<Foot>& foot){
    	// get actual force from the sensor
		for(int i = 0; i < 2; i++){
			// set contact state
			foot[i].contact = (foot[i].force.z() >= min_contact_force);

			// measure continuous contact duration
			if(foot[i].contact){
				foot[i].zmp = Vector3(-foot[i].moment.y()/foot[i].force.z(), foot[i].moment.x()/foot[i].force.z(), 0.0);
			}
			else{
				foot[i].zmp = Vector3(0.0, 0.0, 0.0);
			}
		}
			// both feet not in contact
		if(!foot[0].contact && !foot[1].contact){
			foot[0].balance = 0.5;
			foot[1].balance = 0.5;
			centroid.zmp = Vector3(0.0, 0.0, 0.0);
		}
		else{
			double f0 = std::max(0.0, foot[0].force.z());
			double f1 = std::max(0.0, foot[1].force.z());
			foot[0].balance = f0/(f0 + f1);
			foot[1].balance = f1/(f0 + f1);
			centroid.zmp =
			     (foot[0].balance) * (foot[0].pos_ref + foot[0].ori_ref * foot[0].zmp)
	           + (foot[1].balance) * (foot[1].pos_ref + foot[1].ori_ref * foot[1].zmp);
		}
	}
```

まずはCalcZmpからです。
この関数内では全体の流れでも書いたようにZMPの位置を計算しています。
まず、43行目で足が設置しているかどうかをセンサーからの値で確認しています。
床反力のz軸方向が最小接触力$min_contact_force$よりも大きいとき、
$foot[i].contact$にはtrueが代入され、そうでないときはfalseが代入されます。
この$foot[i].contact$がtrueのとき、各軸回りのモーメントと床反力から
$$\boldsymbpl{p} = \frac{\boldsymbol{M}}{\boldsymbol{F}}$$
を用いて、ZMPの位置を推定しています。
ただし、$\boldsymbpl{p}$はZMP、$\boldsymbol{M}$はモーメント、$\boldsymbol{F}$は床反力を表しています。
49行目で0が代入されているのは、足が地面に接触していないときのZMPですね。
ここで、求めたのは各足のZMPですが、知りたいのは支持多角形全体でどの位置にZMPがあるかです。
そこで60行目からの計算が必要になってきます。
55行目から始まるif文は両足とも地面に接触していないとき、
つまり、両足ともが空中に浮いているときの話をしています。
さて、60行目からですが、ZMPは各足に作用している床反力の大きさの比によってその位置が決定されます。
そのため、2点間を内分する点を求めることでZMPを求めることができます。
61行目と62行目は各足が接触していれば、床反力の値が$f_0$と$f_1$にその値を代入します。
63行目と64行目で内分比を導出します。
もし仮に片側の足が接触していなかった場合、$f_0$か$f_1$のどちらかには0が代入されているため、
`foot[0].balance`か`foot[1].balance`のどちらかが0となるため、
もう一方の足で求めたZMPの位置がロボットのZMPの位置となります。
両方ともの足が接触している場合は、床反力の大きさに応じて内分比が求められるため、
最終的に65行目以降でロボットのZMPの位置が導出されます。

---

-   **CalcBaseTilt**

```cpp {linenos=inline}
	void Stabilizer::CalcBaseTilt(const Timer& timer, const Param& param, Base& base, Vector3 theta, Vector3 omega){
		// desired angular acceleration for regulating orientation (in local coordinate)
		Vector3 omegadd_local(
			-(orientation_ctrl_gain_p*theta.x() + orientation_ctrl_gain_d*omega.x()),
			-(orientation_ctrl_gain_p*theta.y() + orientation_ctrl_gain_d*omega.y()),
			0.0
		);

		// 
		Vector3 omegadd_base(
			-base_tilt_rate*omegadd_local.x() - base_tilt_damping_p*base.angle_ref.x() - base_tilt_damping_d*base.angvel_ref.x(),
			-base_tilt_rate*omegadd_local.y() - base_tilt_damping_p*base.angle_ref.y() - base_tilt_damping_d*base.angvel_ref.y(),
			 0.0
		);

		base.angle_ref  += base.angvel_ref*timer.dt;
		base.ori_ref = FromRollPitchYaw(base.angle_ref);
		base.angvel_ref += omegadd_base*timer.dt;
	}
```

次はCalcBaseTiltについてです。
ベースリンクの傾きを計算する部分です。
始めのomegadd_localでは、ローカル座標系におけるベースリンクの目標角加速度を計算しています。


---

-   **CalcDcmDynamics**

```cpp {linenos=inline}
	void Stabilizer::CalcDcmDynamics(const Timer& timer, const Param& param, const Footstep& footstep_buffer, const Base& base, Vector3 theta, Vector3 omega, Centroid& centroid){
		const Step& stb0 = footstep_buffer.steps[0];
	    const Step& stb1 = footstep_buffer.steps[1];
    	int sup =  stb0.side;
    	int swg = !stb0.side;
	
		double T = param.T;
		double m = param.total_mass;
		double h = param.com_height;
		double T_mh  = T/(m*h);
		double T2_mh = T*T_mh;

		Vector3 offset(0.0, 0.0, param.com_height);

		// desired angular acceleration for regulating orientation (in local coordinate)
		Vector3 omegadd_local(
			-(orientation_ctrl_gain_p*theta.x() + orientation_ctrl_gain_d*omega.x()),
			-(orientation_ctrl_gain_p*theta.y() + orientation_ctrl_gain_d*omega.y()),
			//-(orientation_ctrl_gain_p*theta.z() + orientation_ctrl_gain_d*omega.z())
			0.0
		);
		// desired moment (in local coordinate)
		Vector3 Ld_local(
			param.nominal_inertia.x()*omegadd_local.x(),
			param.nominal_inertia.y()*omegadd_local.y(),
			param.nominal_inertia.z()*omegadd_local.z()
		);
		// limit recovery moment for safety
		for(int i = 0; i < 3; i++){
			Ld_local[i] = std::min(std::max(-recovery_moment_limit, Ld_local[i]), recovery_moment_limit);
		}
		// desired moment (in global coordinate);
		Vector3 Ld = base.ori_ref * Ld_local;

		// virtual disturbance applied to DCM dynamics to generate desired recovery moment
		Vector3 delta = Vector3(-T_mh*Ld.y(), T_mh*Ld.x(), 0.0);

		// calc zmp for regulating dcm
		const double rate = 1.0;
		centroid.zmp_ref = stb0.zmp + dcm_ctrl_gain*(centroid.dcm_ref - stb0.dcm) + rate*T*delta;

		// project zmp inside support region
		if(stb0.stepping){
			Vector3 zmp_local = stb0.foot_ori[sup].conjugate()*(centroid.zmp_ref - stb0.foot_pos[sup]);
			for(int j = 0; j < 3; j++){
				zmp_local[j] = std::min(std::max(param.zmp_min[j], zmp_local[j]), param.zmp_max[j]);
			}
			centroid.zmp_ref = stb0.foot_pos[sup] + stb0.foot_ori	[sup]*zmp_local;
		}

		// calc DCM derivative
		Vector3 dcm_d = (1/T)*(centroid.dcm_ref - (centroid.zmp_ref + Vector3(0.0, 0.0, h))) + delta;

		// calc CoM acceleration
		centroid.com_acc_ref = (1/T)*(dcm_d - centroid.com_vel_ref);

		// update DCM
		centroid.dcm_ref += dcm_d*timer.dt;
		// limit deviation from reference dcm
		for(int j = 0; j < 3; j++){
			centroid.dcm_ref[j] = std::min(std::max(stb0.dcm[j] - dcm_deviation_limit, centroid.dcm_ref[j]), stb0.dcm[j] + dcm_deviation_limit);
		}

		// calc CoM velocity from dcm
		centroid.com_vel_ref = (1/T)*(centroid.dcm_ref - centroid.com_pos_ref);

		// update CoM position
		centroid.com_pos_ref += centroid.com_vel_ref*timer.dt;
	}
```

続いて，CalcDcmDynamicsについてです。
始めのomegadd_localはCalcBaseTiltと同じなので、省略しますね。
そのあとのLd_localは発生している角加速度に対して、
それを回復させるために必要なモーメントを計算しています。
しかし、そのモーメントはロボットが発生させることができる大きさに限界があります。
その限界を超えないようにしているのが、198行目からのfor文で書かれているところです。
限界値と算出した目標モーメントの大小を比較して、決定しています。
ここまで、計算していたのはローカル座標系での大きさになりますので、
202行目ではそれをワールド座標系に変換しています。
さて、ここまでの内容で目標となる回復モーメントを求めてきました。
ここからはそのモーメントを実現していく話になります。
205行目では目標となる回復モーメントに応じてDCMの運動を決定します。
この行では、どれだけ調整するかの調整量を計算しています。
そしてその208・209行目では、DCMを調整するためにZMPがどの位置にあればいいかを計算しています。
[009 - 目標ZMP・DCM計画](https://koomiy.github.io/posts/dcm_generator/)の回でやったように、DCMはZMPから直線的に離れていきます。
イメージは下の図のような感じです。

{{<figure src="./dcm_dynamics.png" class="center" alt="dcm_dynamics" width="50%">}}
{{<figure src="./change_dcm_dynamics.png" class="center" alt="change_dcm_dynamics" width="50%">}}

さて、回復モーメントを考えなければ、当初計画していたDCMの運動(黒色)でよかったとしましょう。
しかし、回復モーメントを必要分だけ発生させようと思うと水色のDCMのような運動をさせないといけなくなりました。
先ほども説明したようにDCMはZMPから直線的に離れますので、
黒色のZMPの位置では水色の捕点を実現することはできません。
そこで、それを実現するためにZMPの位置を水色の位置に変えようということです。
208・209行目ではその新たに修正することになったDCMの位置から、ZMPの位置を求めているわけです。

212行目から218行目ではZMPが支持多角形内部に収まるようにしています。
[009 - 目標ZMP・DCM計画](https://koomiy.github.io/posts/dcm_generator/)の回に書いていますが、ZMPは支持多角形の内部にしか存在できません。
いくら回復モーメントを実現するためとはいえ、その外に出ることは許されないのです。
そこで、本来あってほしい位置にできるだけ近い位置にとるようにしているのがこの部分になります。

そこから先は修正したZMPを使ってDCMや重心の運動を考えていきます。
221行目はDCMの速度を
$$ \boldsymbol{\dot{\xi}} = \frac{1}{T}(\boldsymbol{\xi} - \boldsymbol{p}) $$
の式を使って更新します。
さらに224行目では更新したDCM速度を利用して
$$ \boldsymbol{\ddot{x}} = -\frac{1}{T}(\dot{\boldsymbol{\xi}} - \boldsymbol{x}) $$
から重心加速度を求めます。
ここまで求めたらあとはそれぞれを積分してDCMの位置、重心速度と位置を求めていくだけですね(227～237行目)。

---

-   **CalcForceDistribution**

```cpp {linenos=inline}
	void Stabilizer::CalcForceDistribution(const Param& param, Centroid& centroid, vector<Foot>& foot){
		// switch based on contact state
		if(!foot[0].contact_ref && !foot[1].contact_ref){
			foot[0].balance_ref = 0.5;
			foot[1].balance_ref = 0.5;
			foot[0].zmp_ref = Vector3(0.0, 0.0, 0.0);
			foot[1].zmp_ref = Vector3(0.0, 0.0, 0.0);
		}
		if( foot[0].contact_ref && !foot[1].contact_ref){
			foot[0].balance_ref = 1.0;
			foot[1].balance_ref = 0.0;
			foot[0].zmp_ref = foot[0].ori_ref.conjugate() * (centroid.zmp_ref - foot[0].pos_ref);
			foot[1].zmp_ref = Vector3(0.0, 0.0, 0.0);
		}
		if(!foot[0].contact_ref &&  foot[1].contact_ref){
			foot[0].balance_ref = 0.0;
			foot[1].balance_ref = 1.0;
			foot[0].zmp_ref = Vector3(0.0, 0.0, 0.0);
			foot[1].zmp_ref = foot[1].ori_ref.conjugate() * (centroid.zmp_ref - foot[1].pos_ref);
		}
		if( foot[0].contact_ref &&  foot[1].contact_ref){
			//
			Vector2 b;
			Vector3 pdiff  = foot[1].pos_ref - foot[0].pos_ref;
			double  pdiff2 = pdiff.squaredNorm();
			const double eps = 1.0e-10;
			if(pdiff2 < eps){
				b[0] = b[1] = 0.5;
			}
			else{
				b[0] = (pdiff.dot(foot[1].pos_ref - centroid.zmp_ref))/pdiff2;
				b[0] = std::min(std::max(0.0, b[0]), 1.0);
				b[1] = 1.0 - b[0];
			}

			foot[0].balance_ref = b[0];
			foot[1].balance_ref = b[1];

			Vector3 zmp_proj = b[0]*foot[0].pos_ref + b[1]*foot[1].pos_ref;

			double b2 = b.squaredNorm();
			foot[0].zmp_ref = (b[0]/b2) * (foot[0].ori_ref.conjugate() * (centroid.zmp_ref - zmp_proj));
			foot[1].zmp_ref = (b[1]/b2) * (foot[1].ori_ref.conjugate() * (centroid.zmp_ref - zmp_proj));
		}

		// limit zmp
		for(int i = 0; i < 2; i++){
			for(int j = 0; j < 3; j++){
				foot[i].zmp_ref[j] = std::min(std::max(param.zmp_min[j], foot[i].zmp_ref[j]), param.zmp_max[j]);
			}
		}

		for(int i = 0; i < 2; i++){
			// force and moment to realize desired Zmp
			foot[i].force_ref     =  foot[i].ori_ref.conjugate() * (foot[i].balance_ref * centroid.force_ref);
			foot[i].moment_ref[0] =  foot[i].force_ref.z() * foot[i].zmp_ref.y();
			foot[i].moment_ref[1] = -foot[i].force_ref.z() * foot[i].zmp_ref.x();
			foot[i].moment_ref[2] =  foot[i].balance_ref * centroid.moment_ref.z();
		}
	}
```

さあ、この回も最後の解説です。最後はCalcForceDistributionですね。
さて、この関数の中ではif文がたくさん並んでいてややこしそうですが、そんなに気にするほどではありません。
これはどちらの足が設置しているかを条件分けしているだけです。
最初のif文の条件では、両方ともの足が設置していないときを表しているので、
支持多角形なんてありませんから、ZMPはないことになっています。
そのあとの二つのif文は片足支持期の話です。
片足支持期のときは、その支持足だけを考えればいいですから、
82行目や89行目で支持足基準の座標系でZMPの目標位置を相対位置で表しています。
最後のif文は両足支持期のことを考えています。
このときは、片足支持期のときとは異なり、どちらの足にZMPが偏っているかを考える必要があります。
それを行っているのが、97行目から104行目の部分です。
この部分では、偏り具合を計算してそれを割合で表しています。
その割合に基づいて、支持多角形の中心を計算し、112行目と113行目でZMPの相対位置を計算しています。
117行目から121行目ではそのZMPが支持多角形の内部で存在するように計算しているところですが、
CalcDcmDynamicsの項目で説明した内容と同じですので、省略しますね。
ここまで、求めてきたZMPの目標位置から床反力やモーメントが
どのように作用すればいいのかを123行目から129行目で計算しています。

---

## まとめ・次回予告

今回はvnoidパッケージの安定化制御  
(Stabilizer) について解説しました。

ひとまずvnoidの標準機能に関する説明は以上となります！