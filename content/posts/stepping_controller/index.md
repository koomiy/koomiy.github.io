---
title: "012 - 歩行制御"
date: 2024-03-01
categories: ["vnoidの解説"]
menu: main
---

(この記事は制作途中です)

---

## 今回の内容

本チャレンジでは、初心者の方でも簡単に人形ロボットの運動計画ができるように、  
vnoidというサンプルパッケージが用意されております。

今回はvnoidパッケージの機能の一つ、  
歩行制御器(stepping_controller)について解説します。

---

## 歩行制御の概要

一歩ずつ、支持足や、歩行開始時の離陸足、歩行終了時の着陸足の位置・姿勢を更新します。
また、歩行開始時および終了時の目標 ZMP・DCMについても一歩ずつ更新します。
そのうえで、離陸と着陸の間をつなぐ遊脚側の足の目標軌道を生成し、  
人型ロボットにそれら全ての指令を送るというのが、歩行制御の概要となります。

---

## サンプルコードの解説

```cpp {linenos=inline}
void SteppingController::Update(const Timer& timer, const Param& param, Footstep& footstep, Footstep& footstep_buffer, Centroid& centroid, Base& base, vector<Foot>& foot){
    if(CheckLanding(timer, footstep_buffer.steps[0], foot)){
        if(footstep.steps.size() > 1){
            // pop step just completed from the footsteps
            footstep.steps.pop_front();
            if(footstep.steps.size() == 1){
		        printf("end of footstep reached\n");
                return;
	        }
        }

        footstep_buffer.steps[1].dcm = footstep_buffer.steps[0].dcm;
        footstep_buffer.steps.pop_front();
        footstep_buffer.steps.push_back(Step());
        footstep_buffer.steps[0].tbegin = timer.time;

        buffer_ready = false;
    }

	...

}
```

まず2行目の`CheckLanding`で、着地が完了したかを判定します。  
ここで、`footstep_buffer`は直近1歩分の歩行ステップを格納するためのバッファです。  
バッファを用意するのは、前の歩行ステップの情報が必要になる場合があるからです。  
このバッファには、常に2つの歩行ステップが格納されるようになっていますが、  
この2つを合わせて1歩分の情報を表現します。
バッファの0番目の要素が現在の歩行ステップの情報です。  
現在時刻が、現在の歩行ステップの終了予定時刻を過ぎたとき、  
着地完了のフラグが立ちます。  
着地が完了していない、つまり片足支持中の場合は特に何もせず次のブロックに移ります。

着地が完了した、つまり次の歩行ステップの両足支持期間中の場合は、  
3~17行目でバッファを次の1歩分の歩行ステップに更新します。  
まず、たった今終了した歩行ステップを`footstep`から取り除きます。  
このとき、歩行ステップが残り一つしかないときは歩行を終了します(6~9行目)。  
まだ数歩残っているときには、終了した歩行ステップから DCM を引継いだのちに、  
`footstep_buffer`からも終了した歩行ステップを取りのぞきます。  
こうして、次の歩行ステップが現在の歩行ステップとして更新されます。  
さらに、空の歩行ステップ情報をバッファの末尾に追加します。  
よって、いまバッファには更新された現在の歩行ステップと空の次の歩行ステップの  
二歩分が格納されていることになります。  
次の歩行ステップの開始時刻`tbegin`を現在時刻にセットします。  
バッファの1番目の要素がまだ空で、  
歩行ステップ間の足の制御をする準備が整っていないので、  
`buffer_ready = false`として、次のブロックに移ります。

```cpp {linenos=inline}
void SteppingController::Update(const Timer& timer, const Param& param, Footstep& footstep, Footstep& footstep_buffer, Centroid& centroid, Base& base, vector<Foot>& foot){

	...

    Step& st0  = footstep.steps[0];
    Step& st1  = footstep.steps[1];
    Step& stb0 = footstep_buffer.steps[0];
    Step& stb1 = footstep_buffer.steps[1];
    int sup =  st0.side;
    int swg = !st0.side;
	
    double T  = param.T;
    Vector3 offset(0.0, 0.0, param.com_height);
    
    if(!buffer_ready){
        // update support, lift-off, and landing positions
        stb0.side = st0.side;
        stb1.side = st1.side;

        stb0.stepping = st0.stepping;
        stb0.duration = st0.duration;

        stb0.foot_pos  [sup] = foot[sup].pos_ref;
        stb0.foot_angle[sup] = Vector3(0.0, 0.0, foot[sup].angle_ref.z());
        stb0.foot_ori  [sup] = FromRollPitchYaw(stb0.foot_angle[sup]);
        stb0.foot_pos  [swg] = foot[swg].pos_ref;
        stb0.foot_angle[swg] = Vector3(0.0, 0.0, foot[swg].angle_ref.z());
        stb0.foot_ori  [swg] = FromRollPitchYaw(stb0.foot_angle[swg]);
        stb0.dcm = centroid.dcm_ref;
    
        // landing position relative to support foot, taken from footsteps
        Quaternion ori_rel = st0.foot_ori[sup].conjugate()* st1.foot_ori[swg];
        Vector3    pos_rel = st0.foot_ori[sup].conjugate()*(st1.foot_pos[swg] - st0.foot_pos[sup]);
        Vector3    dcm_rel = st0.foot_ori[sup].conjugate()*(st1.dcm - st0.foot_pos[sup]);

        // calc absolute landing position
        stb1.foot_pos  [sup] = stb0.foot_pos  [sup];
        stb1.foot_ori  [sup] = stb0.foot_ori  [sup];
        stb1.foot_angle[sup] = stb0.foot_angle[sup];
        stb1.foot_pos  [swg] = stb0.foot_pos[sup] + stb0.foot_ori[sup]*pos_rel;
        stb1.foot_ori  [swg] = stb0.foot_ori[sup]*ori_rel;
        stb1.foot_angle[swg] = ToRollPitchYaw(stb1.foot_ori[swg]);
        stb1.dcm = stb0.foot_pos[sup] + stb0.foot_ori[sup]*dcm_rel;

        // calc zmp
        double alpha = exp(-stb0.duration/T);
        stb0.zmp = (1.0/(1.0 - alpha))*(stb0.dcm - alpha*stb1.dcm) - offset;

        buffer_ready = true;
    }

    if(footstep.steps.size() < 2){
		return;
	}

	...

}
```

このブロックでは、空の歩行ステップ情報を埋める処理が行われます。  
この処理は、一歩につき**最初の一度だけ**行われます。

まず、以降のプログラムで用いる変数について説明しておきます。  
`st0`や`st1`は、`footstep`オブジェクトの0番目と1番目の要素です。  
`stb0`や`stb1`は、`footstep_buffer`オブジェクトの0番目と1番目の要素です。  
`sup`はその足が支持側の足であることを示すフラグであり、  
`swg`はその足が遊脚側の足であることを示すフラグです。  
`T`は LIPM の重心運動の時定数です。  
`offset`は、三次元空間上のDCM と水平面上の ZMP の次元を揃えるために用いる、  
z軸方向の目標重心高さ分のオフセットです。

17~29行目では、現在の歩行ステップの情報を最新のものに更新しています。  
はじめに、足の左右判別`side`や、  
そのステップで歩行と言えるだけの距離を歩いているかの判別`stepping`、  
歩行期間`duration`を`st`と`stb`で揃えておきます。  
続いて、現在の歩行ステップの支持足の位置・姿勢を最新の情報に更新します(23~25行目)。  
また、現在の歩行ステップの離陸足の位置・姿勢を最新の情報に更新します(26~28行目)。  
さらに、現在の歩行ステップの目標 DCM を、  
stabilizer側で更新された目標 DCM `centroid.dcm_ref`とします(29行目)。  

31~34行目では、`st1`の情報をもとに、支持足を基準とした  
着地足の位置・姿勢および次の歩行ステップの目標 DCM を相対的に決定します。

36~43行目では、次の歩行ステップの情報を最新のものに更新しています。  
まず、支持足は二つの歩行ステップ間で変化しないので、  
次の歩行ステップの支持足の位置・姿勢を、  
現在の歩行ステップのものと一致させます(37~39行目)。  
続いて、先ほど求めた相対着地位置・姿勢および目標 DCM を絶対座標系に変換し、  
それらを次の歩行ステップの着地足の位置・姿勢、目標 DCM とします(40~43行目)。  





(今一度、歩行ステップとは何か、一歩を歩行ステップでどのように表現するのか、footとかcentroidの中のposと、footstepのposの扱いの違いについて説明した方がいい気がする)

```cpp {linenos=inline}
void SteppingController::Update(const Timer& timer, const Param& param, Footstep& footstep, Footstep& footstep_buffer, Centroid& centroid, Base& base, vector<Foot>& foot){

	...

    // time to landing
    double ttl = stb0.tbegin + stb0.duration - timer.time;
	double alpha = exp(-ttl/T);
    
	stb0.dcm = (1.0-alpha)*(stb0.zmp + offset) + alpha*stb1.dcm;
    
    // landing adjustment based on dcm
    Vector3 land_rel = (st1.foot_pos[swg] - st0.foot_pos[sup]) - (stb0.dcm - stb0.foot_pos[sup]) + 0.0*(stb0.zmp - stb0.foot_pos[sup]);
	stb1.foot_pos[swg].x() = centroid.dcm_ref.x() + land_rel.x();
	stb1.foot_pos[swg].y() = centroid.dcm_ref.y() + land_rel.y();

    // reference base orientation is set as the middle of feet orientation
    double angle_diff = foot[1].angle_ref.z() - foot[0].angle_ref.z();
    while(angle_diff >  pi) angle_diff -= 2.0*pi;
    while(angle_diff < -pi) angle_diff += 2.0*pi;
	base.angle_ref.z() = foot[0].angle_ref.z() + angle_diff/2.0;

    base.ori_ref   = FromRollPitchYaw(base.angle_ref);

    // set support foot position
    foot[sup].pos_ref     = stb0.foot_pos[sup];
    foot[sup].angle_ref   = stb0.foot_angle[sup];
    foot[sup].ori_ref     = FromRollPitchYaw(foot[sup].angle_ref);
    foot[sup].contact_ref = true;

    // set swing foot position
    if(!stb0.stepping || (timer.time - stb0.tbegin) < dsp_duration)
    {
        foot[swg].pos_ref     = stb0.foot_pos  [swg];
        foot[swg].angle_ref   = stb0.foot_angle[swg];
        foot[swg].ori_ref     = stb0.foot_ori  [swg];
        foot[swg].contact_ref = true;
    }
    else{
        double ts   = timer.time - (stb0.tbegin + dsp_duration);            //< time elapsed in ssp
        double tauv = stb0.duration - dsp_duration; //< duration of vertical movement
        double tauh = tauv - descend_duration;     //< duration of horizontal movement

        // cycloid swing profile
        double sv     = ts/tauv;
        double sh     = ts/tauh;
        double thetav = 2.0*pi*sv;
        double thetah = 2.0*pi*sh;
        double ch     = (sh < 1.0 ? (thetah - sin(thetah))/(2.0*pi) : 1.0);
        double cv     = (1.0 - cos(thetav))/2.0;
        double cv2    = (1.0 - cos(thetav/2.0))/2.0;
        double cw     = sin(thetah);

        // foot turning
        Vector3 turn = stb1.foot_angle[swg] - stb0.foot_angle[swg];
        while(turn.z() >  pi) turn.z() -= 2.0*pi;
        while(turn.z() < -pi) turn.z() += 2.0*pi;

        // foot tilting
        Vector3 tilt = stb0.foot_ori[swg]*Vector3(0.0, swing_tilt, 0.0);

        foot[swg].pos_ref      = (1.0 - ch)*stb0.foot_pos[swg] + ch*stb1.foot_pos[swg];
        foot[swg].pos_ref.z() += (cv*(swing_height + 0.5*descend_depth) - cv2*descend_depth);
        foot[swg].angle_ref    = stb0.foot_angle[swg] + ch*turn + cw*tilt;
        foot[swg].ori_ref      = FromRollPitchYaw(foot[swg].angle_ref);
        foot[swg].contact_ref  = false;

        // adjust swing foot considering base link inclination
        Quaternion qrel = 
            FromRollPitchYaw(Vector3(base.angle_ref.x(), base.angle_ref.y(), base.angle_ref.z()))*
            FromRollPitchYaw(Vector3(base.angle    .x(), base.angle    .y(), base.angle_ref.z())).conjugate();
        Vector3 pivot   = centroid.zmp_ref;
        foot[swg].pos_ref   = qrel*(foot[swg].pos_ref - pivot) + pivot;
        foot[swg].ori_ref   = qrel* foot[swg].ori_ref;
        foot[swg].angle_ref = ToRollPitchYaw(foot[swg].ori_ref);

    }	
```

---

## まとめ・次回予告

今回はvnoidパッケージの目標ZMP・DCM計画  
(footstep_planner::generateDCM) について解説しました。

次回は歩行制御(stepping_controller)について解説しようと思います。

次回： [012 - 歩行制御](https://koomiy.github.io/posts/stepping_controller/)
