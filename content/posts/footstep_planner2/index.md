---
title: "015 - ステップ計画"
date: 2023-10-05
categories: ["vnoidの解説"]
menu: main
---

年度替わりに伴い、中の人が交代しています。文体等に違いがあるところがあるかと思いますが、ご了承いただければ幸いです。

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
{{<figure src="./footstep_sample.png" class="center" alt="footstep_sample" width="80%">}}  

また、ステップ計画では着地瞬間の目標DCMも計画します。  
DCMについては後ほど説明します。

---

## サンプルコードの解説

-	**歩行パラメータの入力(`myrobot.cpp`162行目~)**
	
	```cpp {linenos=inline}
	if(timer.count % 10 == 0){
    if(use_joystick){
	    // read joystick
	    joystick.readCurrentState();

	    /* Xbox controller mapping:
		    L_STICK_H_AXIS -> L stick right
		    L_STICK_V_AXIS -> L stick down
		    R_STICK_H_AXIS -> L trigger - R trigger
		    R_STICK_V_AXIS -> R stick down
		    A_BUTTON -> A
		    B_BUTTON -> B
		    X_BUTTON -> X
		    Y_BUTTON -> Y
		    L_BUTTON -> L
		    R_BUTTON -> R
	        */
	    /*
        cout <<  joystick.getPosition(Joystick::L_STICK_H_AXIS) << " " 
		     << joystick.getPosition(Joystick::L_STICK_V_AXIS) << " " 
		     << joystick.getPosition(Joystick::R_STICK_H_AXIS) << " " 
		     << joystick.getPosition(Joystick::R_STICK_V_AXIS) << " " 
		     << joystick.getButtonState(Joystick::A_BUTTON) << " "
		     << joystick.getButtonState(Joystick::B_BUTTON) << " "
		     << joystick.getButtonState(Joystick::X_BUTTON) << " "
		     << joystick.getButtonState(Joystick::Y_BUTTON) << " "
		     << joystick.getButtonState(Joystick::L_BUTTON) << " "
		     << joystick.getButtonState(Joystick::R_BUTTON) << endl;
         */
    }
	
	// erase current footsteps
	while(footstep.steps.size() > 2)
		footstep.steps.pop_back();

    // generate footsteps
	Step step;
    if(use_joystick){
        // set stride and turn based on joystick input
	    step.stride   = -max_stride*joystick.getPosition(Joystick::L_STICK_V_AXIS);
	    step.turn     = -max_turn  *joystick.getPosition(Joystick::L_STICK_H_AXIS);
        step.sway     =  max_sway  *joystick.getPosition(Joystick::R_STICK_H_AXIS);
    }
    else{
        // just walk forward
        step.stride = 0.1;
        step.turn   = 0.0;
        step.sway   = 0.0;
    }
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
	
	歩行パラメータには、  
	前後方向への歩幅$stride$や  
	横方向への歩幅$sway$、  
	両足間の初期幅$spacing$、  
	支持足を基準とした着地足の旋回量(鉛直軸周り)$turn$、  
	高低差$climb$、  
	歩行期間$duration$、  
	右足か左足かの判定フラグ$side$があります。  
	$side$は、右足なら0の値を持ち、左足なら1の値を持ちます。  
	
	歩行パラメータは、`Step`クラスのオブジェクト  
	`step`のメンバ変数に代入されます。
	
	`footstep.steps`は`step`を末尾から順に追加して  
	歩行に関する情報を保持しておくための待ち行列です。
	
	デフォルトでは`footstep.steps`に、0.2mの左右足間隔で、  
	まっすぐ0.1mだけ0.5sで歩行するパラメータを持つ`step`を3歩分、  
	その場で停止するパラメータを持つ`step`を1歩分追加します。
	
	

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
	この処理により得られる支持と離地、着地について、  
	おおよその関係をまとめた図を次に示します。
	{{<figure src="./footstep_overall.png" class="center" alt="footstep_overall" width="50%">}}
	
	参照変数として`st0`、`st1`を定義します。  
	これにより例えば、`st0`の内容を変更すれば、  
	その変更は`footstep.steps[i+0]`にも反映されます。
	
	`st0`や`st1`には、両足分の着地位置・姿勢情報を代入します。  
	したがって、左右のうちどちらが支持足で、  
	どちらが振り足かを見分けておく必要があります。  
	そのために`sup`と`swg`変数を用意します。  
	`sup`が支持足であることを意味し、  
	`swg`が振り足であることを意味します。
	
	前後方向への歩幅$l$を$l = stride$、  
	横方向への歩幅$d$を$d = sway$、  
	現在と次の着地点との高低差$dz$を$dz = climb$、  
	旋回量$\Delta\theta$を$\Delta\theta = turn$とします。  
	また、両足間の初期幅$w$は、以下のようにします。  
	$$ 
	w = \begin{cases}
		spacing & (右足が支持側のとき)\\\\\\
		-spacing & (左足が支持側のとき) 
	\end{cases}
	$$
	
	現在の支持足から、次の着地点までの相対位置を$\Delta p_{rel}$とします。  
	旋回しない($\Delta\theta = 0$)場合、$\Delta p_{rel}$は次のように計算できます。  
	$$ \Delta p_{rel} = [l, w + d, dz]^T $$  
	このとき、各パラメータは視覚的には次の画像のような意味となります。  
	{{<figure src="./footstep_diag.png" class="center" alt="footstep_diag" width="50%">}}
	
	旋回歩行する場合は次のように計算します。  
	$$  \Delta p_{rel} = \begin{bmatrix}(r - \frac{w}{2} - d)\mathrm{sin}\Delta\theta \\\\\\ (r + \frac{w}{2}) - (r - \frac{w}{2} - d)\mathrm{cos}\Delta\theta \\\\\\ dz\end{bmatrix}  $$
	$$  r = \frac{l}{\Delta\theta}  $$  
	このとき、各パラメータの視覚的な意味を考えるのは難しいですが、  
	次の画像に示すような曲線上に、次の着地位置が決まります。  
	{{<figure src="./footstep_turning.png" class="center" alt="footstep_turning" width="50%">}}  
	この曲線は、固定歩幅$l$、$d$に対して  
	旋回量$\Delta\theta$を$[-\pi, 0) || (0, \pi]$の区間で変化させたプロットです。  
	この曲線の特徴として、以下の性質を持ちます。  
	$l = 0$のとき、曲線は$(0, \frac{w}{2})$を中心とする半径$\frac{w}{2} + d$の円となります。  
	$d$が増えると、円の半径が大きくなります。  
	{{<figure src="./curve_stride0.gif" class="center" alt="curve_stride0" width="50%">}}  
	また、$l$が増えるごとに曲線が前方向に伸びて円が開いていきます。  
	{{<figure src="./curve_strideup.gif" class="center" alt="curve_strideup" width="50%">}}  
	さらに、歩幅$l$、$d$を固定としたとき、旋回量$\Delta\theta$が増えるにつれて  
	支持足、着地足間の距離が次第に短くなります。  
	{{<figure src="./dprel_vs_turn.png" class="center" alt="dprel_vs_turn" width="50%">}}  
	これらの性質から、指定した歩幅の情報を反映しながらも、  
	旋回によって脚の長さを超えて着地しないように  
	曲線が設計されていることが分かります。
	
	ここまでで、現在の支持足から次の着地点までの相対位置$\Delta p_{rel}$が求まりました。  
	以降32行目より`st1`に計画した着地情報を入力します。
	
	33行目で支持足の交換を行います。  
	36~38行目で、前のステップにおける着地足と現在における支持足を一致させます。  
	56~58行目で、現在のステップにおける着地足を更新します。

---

## 例題: ジグザグ歩行

今回の例題のまず始めはジグザグ歩行です。  

```cpp
else{
    // just walk forward
    step.stride = 0.1;
    step.turn   = 0.0;
    step.sway   = 0.0;
    if (timer.time < 3.0) {
    	step.stride = 0.0;
		step.turn   = 0.1745;
    } 
    else {
		if (((int) timer.time - 3) / 6 % 4 == 0 || ((int) timer.time -3) / 6 % 4 == 2) {
			step.stride = 0.1;
        	step.turn   = 0.0;
        } 
        else if(((int) timer.time - 3) / 6 % 4 == 3) {
            step.stride = 0.0;
            step.turn   = 0.1745;
        } else {
            step.stride = 0.0;
            step.turn = -0.1745;
        }
    }
}
```

myrobot.cppの210行目以降に書かれているコントローラー制御なしのときの歩行パラメータが書かれた部分に注目してください。  
この部分に時間ごとに回転、前進が切り替わるようにパラメータを設定してみましょう。  
下の例では、まず最初の3秒間で反時計回りに回転して、その後6秒間前進します。  
前進が終わると次の6秒間で時計回りに回転して、6秒間前進するというように記述しています。  
以降はその繰り返しです。  
回転角や前進するときの歩幅、それぞれにかける時間を色々変えてみてどのような歩行になるか試してみてください。  

{{<figure src="./vnoid_jiguzagu_1.gif" class="center" alt="vnoid_jiguzagu_1" width="80%">}}  

---

## 例題: ジグザグ歩行2

二つ目のジグザグ歩行は先ほどのstrideとturnを用いたジグザグ歩行ではなく、  
strideとswayを利用したジグザグ歩行です。  
(これをジグザグ歩行というかはわかりませんが...)  
注目する部分は一つ目の例題と同じでmyrobot.cppの210行目以降です。  
下の例では前進すると同時に左右にも歩行パラメータに値を入力するプログラムになっています。  
前進や横への踏み出しをするときの歩幅、それぞれにかける時間を色々変えてみてどのような歩行になるか試してみてください。  

```cpp {linenos=inline}
else{
    // just walk forward
	step.stride = 0.1;
    step.turn   = 0.0;
    step.sway   = 0.0;
    if ((int)timer.time/5%2 == 0) {
        step.sway = 0.05;
    } else {
        step.sway = -0.05;
    }
}
```

{{<figure src="./vnoid_jiguzagu_2.gif" class="center" alt="vnoid_jiguzagu_2" width="80%">}}  

---

## 例題: コントローラで歩かせてみよう
コントローラーを使ってロボットを動かせるようにしてみましょう。  
成功するとラジコンを操縦するように動かせますよ!!  

＊注意(1)：コントローラーはプロジェクトを開く前にご使用のパソコンに接続しておかないとchoreonoid側が認識してくれません。  

＊注意(2)：コントローラーがない方はchoreonoidに仮想ジョイスティックというものがあるのでそちらをご利用ください。使用方法はこの項目の最後に書いてあります。  

```cpp {linenos=inline}
MyRobot::MyRobot(){
	base_actuation = false;

    // set use_joystick as true if you want to command robot with joystick
    use_joystick = true;	//when true, you can make the robot walk by using your controller. when false, the robot automatically walk 
    max_stride = 0.2;
    max_turn   = 0.1;
    max_sway   = 0.1;
}
```

まずは、myrobot.cppの9行目から17行目に書かれているMyRobot::MyRobot()に注目してください。  
この中に書かれているuse_joystick(13行目)という変数をtrueにしましょう。  
これをtrueにすることでコントローラーからの入力を受け取ってロボットを歩かせることができます。  

```cpp {linenos=inline}
void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);

	// calc FK
    fk_solver.Comp(param, joint, base, centroid, hand, foot);

	if(timer.count % 10 == 0){
    	if(use_joystick){
			// read joystick
		    joystick.readCurrentState();

			/* Xbox controller mapping:
			L_STICK_H_AXIS -> L stick right
			L_STICK_V_AXIS -> L stick down
		    R_STICK_H_AXIS -> L trigger - R trigger
			R_STICK_V_AXIS -> R stick down
	    	A_BUTTON -> A
			B_BUTTON -> B
		    X_BUTTON -> X
			Y_BUTTON -> Y
		    L_BUTTON -> L
			R_BUTTON -> R
        	*/
			/*
	        cout <<  joystick.getPosition(Joystick::L_STICK_H_AXIS) << " " 
			 << joystick.getPosition(Joystick::L_STICK_V_AXIS) << " " 
	    	 << joystick.getPosition(Joystick::R_STICK_H_AXIS) << " " 
			 << joystick.getPosition(Joystick::R_STICK_V_AXIS) << " " 
    		 << joystick.getButtonState(Joystick::A_BUTTON) << " "
			 << joystick.getButtonState(Joystick::B_BUTTON) << " "
		     << joystick.getButtonState(Joystick::X_BUTTON) << " "
			 << joystick.getButtonState(Joystick::Y_BUTTON) << " "
		     << joystick.getButtonState(Joystick::L_BUTTON) << " "
			 << joystick.getButtonState(Joystick::R_BUTTON) << endl;
			 */
        }
		
		// erase current footsteps
		while(footstep.steps.size() > 2)
			footstep.steps.pop_back();

        // generate footsteps
		Step step;
		if(use_joystick){
            // set stride and turn based on joystick input
			step.stride   = -max_stride*joystick.getPosition(Joystick::L_STICK_V_AXIS);
			step.turn     = -max_turn  *joystick.getPosition	(Joystick::L_STICK_H_AXIS);
        	step.sway     =  max_sway  *joystick.getPosition(Joystick::R_STICK_H_AXIS);
    	}
		else{
    		// just walk forward
     	   	step.stride = 0.1;
            step.turn   = 0.0;
	    	step.sway   = 0.0;
	    }
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

	// stepping controller generates swing foot trajectory 
    // it also performs landing position adaptation
    stepping_controller.Update(timer, param, footstep, footstep_buffer, centroid, base, foot);
    
	// stabilizer performs balance feedback
    stabilizer         .Update(timer, param, footstep_buffer, centroid, base, foot);
    
	// step timing adaptation
    //Centroid centroid_pred = centroid;
   	//stabilizer.Predict(timer, param, footstep_buffer, base, centroid_pred);
	//stepping_controller.AdjustTiming(timer, param, centroid_pred, footstep, footstep_buffer);

	hand[0].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0, -0.25, -0.1);
	hand[0].ori_ref = base.ori_ref;
    hand[1].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0,  0.25, -0.1);
    hand[1].ori_ref = base.ori_ref;

	// calc CoM IK
    ik_solver.Comp(&fk_solver, param, centroid, base, hand, foot, joint);

	Robot::Actuate(timer, base, joint);
	
	timer.Countup();
	}
```

さあ実際に制御している部分を見ていきましょう。  
まず、165行目に書かれているjoystick.readCurrentState()でジョイスティックの入力を読み込んできます。  
ジョイスティックの入力値の値はその下にコメントアウトで書かれている通りですので、そちらを参照して対応関係をとってみてください。  
そこからずーっと下のほうに下がっていくと、201行目にif(use_joystick)と書かれていますね。  
さきほどtrueにしたことによって、この条件分岐の中に入っていくことになります。  
この中身に書かれている３行でロボットに送る指令値を前進(step.stride)、回転(step.turn)、横歩き(step.sway)として計算しています。  
あとは`footstep_planner`において前述した方法を用いて着地位置と姿勢が啓作されることになります。  

**仮想ジョイスティックの使用方法**
{{<figure src="./virtual_joystick.png" class="center" alt="footstep_sample" width="80%">}}  
「ジョイスティックなんか持ってないよ!!」  
「ロボットを歩かせられないよ!!」  
というそんなあなたのために仮想ジョイスティックというものをご紹介します。  
まずは普通にchoreonoidを起動してください。  
そのあとプロジェクトも起動してください。  
写真のように上のツールバーに表示というボタンがあるので、それを選択してください。  
すると、写真のようにプルダウンメニューが出てきます。  
そこのビューの表示にカーソルを合わせると、  
さらにプルダウンメニューが出てくるので、  
その中にある仮想ジョイスティックの項目にチェックを入れてください。  
チェックを入れるとこれまで関節角度などが表示されていたところに  
仮想ジョイスティックが追加されるので、  
ジョイスティックを持っていないあなたも自分の思い通りに動かせるようになりました!!  
ちなみに、前進後退が[E]と[D]ボタン、回転が[S]と[F]ボタン、横歩きが[J]と[L]ボタンになっています。  
キーボードでも同じボタンを押せば操作できるみたいです。  

---

## まとめ・次回予告

今回はvnoidパッケージのステップ計画器footstep_planner::Planについて解説しました。  

次回は目標DCM計画器footstep_planner::GenerateDCMについて解説しようと思います。  

次回： [009 - 目標DCM計画器](https://koomiy.github.io/posts/dcm_generator/)
