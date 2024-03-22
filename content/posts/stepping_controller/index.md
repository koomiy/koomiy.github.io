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

一歩ずつ、支持足や、歩行開始時の離地足、歩行終了時の着地足の位置・姿勢を更新します。  
また、歩行開始時および終了時の目標 ZMP・DCMについても一歩ずつ更新します。

そのうえで、離地と着地の間をつなぐ遊脚側の足の目標軌道を生成し、  
人型ロボットにそれら全ての指令を送るというのが、歩行制御の概要です。

---

## サンプルコードの解説

-   **歩行ステップの更新**

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

45~47行目では、現在の歩行ステップにおける目標 ZMP を、  
歩行開始時と終了時の目標 DCM に合わせて決めます。
ここで用いられている漸化式は、  
[目標ZMP・DCM計画](https://koomiy.github.io/posts/dcm_generator/)の記事で説明したものです。

最後に、歩行ステップの情報がすべて埋まり、歩行制御の準備が整ったので、  
`buffer_ready = true`とします。

(今一度、歩行ステップとは何か、一歩を歩行ステップでどのように表現するのか、footとかcentroidの中のposと、footstepのposの扱いの違いについて説明した方がいい気がする)

-   **遊脚軌道の更新**

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

このブロックから、本格的に歩行制御が行われます。

歩行は、両足支持期から片足支持期へ移行する流れを繰り返すものとします。  
浮遊足の離地から着地までの期間が片足支持期です。  
両足支持期の場合は、支持足はそのまま変化しないように(24~28行目)、  
もう一方の支持足は離地の位置・姿勢を保持するように(31~37行目)制御します。  
片足支持期の場合は、支持足はそのまま変化しないように(24~28行目)、  
浮遊足は、サイクロイド曲線をベースに更新されるように(61~62行目)制御します。

以下、片足支持期における浮遊足の位置・姿勢の更新方法について説明します。  
離地足と着地足をつなぐ軌道を、遊脚軌道と言います。

離地、着地瞬間の浮遊足の状態は、それぞれ  
`stb0.foot_pos[swg]`と`stb1.foot_pos[swg]`にあらかじめ計画されています。  
そのうち着地に関しては、歩行間にロボットがバランスを崩した際に、  
安定化のために修正しなければならない場合があります。  
そこで、11~14行目でDCM力学に基づいた着地位置修正を実装しています。  
この安定化方法については、[012 - 安定化制御](https://koomiy.github.io/posts/stepping_controller/)の記事で取り扱います。

安定化制御に伴い目標DCMは修正されます。  
修正前の現在時刻における目標DCM`stb0.dcm`は、  
事前に計画しておいた着地瞬間の目標DCMに安定収束するように計算されます(9行目)。  
ここで、`alpha`は、現在時刻から着地予定時刻までの区間で、  
DCMの運動方程式を離散化して得られる係数です。  
修正後の目標DCMは`centroid.dcm_ref`です。

遊脚軌道$\boldsymbol{p^{swg}}$は、離地位置を$\boldsymbol{p^{lift}}$、着地位置を$\boldsymbol{p^{land}}$として、  
次式のように表される(61~62行目)。

$$ \boldsymbol{p^{swg}} = \boldsymbol{p^{lift}} + \tilde{c}_h (\phi(t_{ssp})) (\boldsymbol{p^{land}} - \boldsymbol{p^{lift}}) + \tilde{c}_v (\phi(t_{ssp})) \boldsymbol{h_{swg}} $$

ここで、$h_{swg}$は足を上げる高さです。  
また、$\tilde{c}_h$、$\tilde{c}_v$はそれぞれ、  
正規化されたサイクロイドの横変位と縦変位で、次式で表されます(39~48行目)。

$$ \tilde{c}_h(\phi) = \frac{c_h(\phi) - c_h(\phi_0)}{c_h(\phi_1) - c_h(\phi_0)} = \frac{\phi - \mathrm{sin}\phi}{2\pi} \\
\tilde{c}_v(\phi) = \frac{c_v(\phi) - c_v(\phi_0)}{c_v(\phi_1) - c_v(\phi_0)} = \frac{1 - \mathrm{cos}\phi}{2} \\
c_h(\phi) = \phi - \mathrm{sin}\phi \\
c_v(\phi) = 1 - \mathrm{cos}\phi \\
\phi = 2\pi s \nonumber \\
\phi_0 = 0 \nonumber \\
\phi_1 = 2\pi \nonumber \\
s = \frac{t_{ssp}}{\tau_{ssp}} \in [0, 1] $$

他にも、歩行制御ではベースリンクのヨー方向姿勢を両足の中間角度に設定したり、  
片足支持期における浮遊足のヨー・ピッチ方向の姿勢を

(これはまだpushしていない)

## 以下だらっと書いたやつ

6行目で、着地予定時刻までの期間`ttl`を決めます。  
ここで、`stb0.tbegin`は現在の歩行ステップの開始時刻、  
`stb0.duration`は現在の歩行ステップの歩行期間、  
`timer.time`は現在時刻です。

7~9行目で、現在時刻における目標DCMを計算します。  
その際、[離地から着地までの区間で離散化したDCMの運動方程式](https://koomiy.github.io/posts/dcm_generator/)を用います。  

11~14行目では、DCM力学に基づいた着地位置修正をします。  
`land_rel`は、現在の支持足を基準とした、相対的な目標着地位置です。  
`stabilizer`では、歩行を安定化させるために目標DCMの修正を行うのですが、  
その修正量を目標着地位置にも反映させることで、安定化を実現します。  
(目標DCMの修正については、[012 - 安定化制御](https://koomiy.github.io/posts/stepping_controller/)の記事で取り扱います。)

16~22行目では、ベースリンクのヨー方向の目標姿勢を設定します。  
これは、両足の角度の中間角度に設定されます。  

24~28行目では、支持足の目標位置・姿勢を設定します。

30~76行目では、遊脚側の足の目標位置・姿勢を計算します。

31~37行目は、両足支持期のときのみ実行されます。  
歩行の流れとしては、先に両足支持期が来たのちに片足支持期に移行します。  
したがって、遊脚側の足の目標位置・姿勢は、計画離地位置・姿勢と一致させます。

38~76行目は、続く片足支持期に実行されます。  
ここで、`ts`は、片足支持期の開始時刻からの経過時間です。  
`tauv`は、片足支持期間で、足の高さ方向の移動にかける期間として使用します。  
`dsp_duration`は、両足支持期間です。  
`tauh`は、足の水平移動にかける期間です。  
`descend_duration`は、足を下ろす期間ですが、標準では0に設定されています。  
したがって、標準では`tauv = tauh`となります。

`sv`、`sh`は、正規化した片足支持期の開始時刻からの経過時間です。  
`thetav`、`thetah`は、サイクロイドの回転角です。  
`cv`、`ch`はそれぞれ、正規化されたサイクロイドの縦変位と横変位です。  
これらは、`tauv`と`tauh`が異なる場合にも対応できるようになっています。

53~59, 63~64行目では、足の目標回転角度を計算します。  
足をヨー・ピッチ方向に旋回するように着地計画をしている場合は、  
離地瞬間からゆるやかに着地瞬間の足の角度になるように計画します。

61~62行目でサイクロイド曲線をベースにした浮遊足の目標位置を計算します。  

67~74行目では、ベースリンク姿勢の目標誤差に合わせて浮遊足の目標位置・姿勢を修正します。  
足の目標位置については、支持足の目標ZMPを中心にロボットが傾いたと想定し、  
その傾きを考慮した着地位置に修正します。  
足の目標姿勢については、そのままベースリンク姿勢の傾きを足します。

---

## まとめ・次回予告

今回はvnoidパッケージの目標ZMP・DCM計画  
(footstep_planner::generateDCM) について解説しました。

次回は歩行制御(stepping_controller)について解説しようと思います。

次回： [013 - 歩行安定化制御](https://koomiy.github.io/posts/stepping_controller/)
