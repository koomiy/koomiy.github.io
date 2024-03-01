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
}
```



---

## まとめ・次回予告

今回はvnoidパッケージの目標ZMP・DCM計画  
(footstep_planner::generateDCM) について解説しました。

次回は歩行制御(stepping_controller)について解説しようと思います。

次回： [012 - 歩行制御](https://koomiy.github.io/posts/stepping_controller/)
