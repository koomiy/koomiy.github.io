---
title: "008 - ステップ計画"
date: 2023-10-05
categories: ["vnoidベースの開発"]
menu: main
---

(この記事は制作途中です)

---

## 今回の内容

本チャレンジでは、初心者の方でも簡単に人形ロボットの運動計画ができるように、  
vnoidというサンプルパッケージが用意されております。

今回はvnoidパッケージの機能の一つ、ステップ計画器(footstep_planner)について解説します。  

---

## ステップ計画とは

将来歩いたときに残る足跡の組を計画することを、  
本記事ではステップ計画と呼びます。

以下の画像のようなイメージです。
{{<figure src="./footstep_sample.png" class="center" alt="footstep_sample" width="50%">}}  

また、ステップ計画では着地瞬間の目標DCMも計画します。  
DCMについては後ほど説明します。

---

## サンプルコードの解説

-	**歩行パラメータの入力(`myrobot.cpp`192行目~)**
	
	```cpp {linenos=inline}
	if(timer.count % 10 == 0){
	    // read joystick
	    joystick.readCurrentState();
	    
	    ...
	    
	    Step step;
	    step.stride   = 0.1     //-max_stride*joystick.getPosition(Joystick::L_STICK_V_AXIS);
	    step.turn     = 0.0     //-max_turn  *joystick.getPosition(Joystick::L_STICK_H_AXIS);
	    step.spacing  = 0.20;
	    step.climb    = 0.0;
	    step.duration = 0.5;
	    footstep.steps.push_back(step);
	    footstep.steps.push_back(step);
	    footstep.steps.push_back(step);
	    step.stride = 0.0;
	    step.turn   = 0.0;
	    footstep.steps.push_back(step);
	    
	    footstep_planner.Plan(param, footstep);
	    footstep_planner.GenerateDCM(param, footstep);
	}
	```
	
	歩行パラメータとは、歩幅や旋回量、歩行期間などといった、  
	一歩進むのに欠かせない情報のことです。  
	歩行パラメータは、`Step`クラスのオブジェクト  
	`step`のメンバ変数に代入されます。
	
	`footstep.steps`は`step`を末尾から順に追加して  
	歩行に関する情報を保持しておくための待ち行列です。
	
	デフォルトでは`footstep.steps`に、0.2mの左右足間隔で、  
	まっすぐ0.1mだけ0.5sで歩行するパラメータを持つ`step`を3歩分と、  
	その場で停止するパラメータを持つ`step`を1歩分を追加します。

-	**着地位置・姿勢計画(`footstep_plenner`16行目~)**
	
	```cpp {linenos=inline}
	void FootstepPlanner::Plan(const Param& param, Footstep& footstep){
    
    	    // we assume that foot placement, support foot flag, and dcm of step[0] are specified from outside

    	    // determine foot placement and support foot flag of remaining steps
    	    int nstep = footstep.steps.size();
	    for(int i = 0; i < nstep-1; i++){
	        Step& st0 = footstep.steps[i+0];
	        Step& st1 = footstep.steps[i+1];

	        int sup =  st0.side;
	        int swg = !st0.side;

	        double  dtheta = st0.turn;
	        double  l      = st0.stride;
	        double  d      = st0.sway;
	        double  w      = (sup == 0 ? 1.0 : -1.0) * st0.spacing;
	        double  dz     = st0.climb;
	        Vector3 dprel;	// 足の相対位置

	        if(std::abs(dtheta) < eps){
	            dprel = Vector3(l, w + d, dz);
	        }
	        else{
	            double r = l/dtheta;
	            dprel = Vector3(
	                (r - w/2.0 - d)*sin(dtheta),
	                (r + w/2.0) - (r - w/2.0 - d)*cos(dtheta),
	                dz);
	        }
	    
	        // support foot exchange
	        st1.side = !st0.side;

	        // support foot pose does not change
	        st1.foot_pos  [sup] = st0.foot_pos  [sup];
	        st1.foot_angle[sup] = st0.foot_angle[sup];
	        st1.foot_ori  [sup] = st0.foot_ori  [sup];
	    
	        // swing foot pose changes
	        st1.foot_pos  [swg] = st0.foot_pos  [sup] + st0.foot_ori[sup]*dprel;
	        st1.foot_angle[swg] = st0.foot_angle[sup] + Vector3(0.0, 0.0, dtheta);
	        st1.foot_ori  [swg] = FromRollPitchYaw(st1.foot_angle[swg]);
	        }
	}
	```
	
	ここでは入力された`footstep.steps`の情報を用いて、  
	先頭から順に着地位置・姿勢を計画します。  
	for文内の処理につき1歩分の計画をします。
	
	参照変数として`st0`、`st1`を定義します。
	これにより例えば、`st0`の内容を変更すれば、  
	その変更は`footstep.steps[i+0]`にも反映されます。
	
	`st0`や`st1`には、左右分の着地位置・姿勢情報を代入します。  
	したがって、左右のうちどちらが支持足で、  
	どちらが振り足かを見分けておく必要があります。  
	そのために`sup`と`swg`変数を用意します。
	`sup`が支持足であることを意味し、  
	`swg`が振り足であることを意味します。
	
	(以降書きかけです)
	
-	**DCM計画(`footstep_planner`91行目~)**
	
	DCM(Divergent Componet of Motion)は、  
	その位置に足を踏み出せば自然に停止できる点を意味します。
	
	自然に停止するとは、十分な時間が経過したのちに  
	着地位置の真上にロボットの重心が遷移するということです。  
	このような踏み出しにより、  
	最終的に着地点からの反力で重力を相殺できるので、  
	静的安定な状態になります。

---

## 例題: ジグザグ歩行



---

## まとめ・次回予告

今回はvnoidパッケージのステップ計画器footstep_plannerについて解説しました。

次回は歩行制御器stepping_controllerについて解説しようと思います。

次回： [009 - 歩行制御器](https://koomiy.github.io/posts/stepping_controller/)
