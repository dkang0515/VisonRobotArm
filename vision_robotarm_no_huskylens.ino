#include <Dynamixel2Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#if defined(ARDUINO_OpenRB)
  #define DXL_SERIAL Serial1
  #define PC_SERIAL  Serial
  const int DXL_DIR_PIN = -1;
#else
  #define DXL_SERIAL Serial1
  #define PC_SERIAL  Serial
  const int DXL_DIR_PIN = 2;
#endif

#ifndef BDPIN_DXL_PWR_EN
  #define BDPIN_DXL_PWR_EN 31
#endif

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ====================== IDs ======================
static const uint8_t ID_BASE         = 1;

static const uint8_t ID_SHOULDER_R   = 11;
static const uint8_t ID_SHOULDER_L   = 12;

static const uint8_t ID_ELBOW_L      = 21;
static const uint8_t ID_ELBOW_R      = 22;

static const uint8_t ID_WRIST_PITCH  = 31;
static const uint8_t ID_WRIST_YAW    = 32;

static const uint8_t ID_GRIPPER      = 33;

// ====================== Mirror directions for Dual joints ======================
static const int8_t SHOULDER_DIR_L = +1;
static const int8_t SHOULDER_DIR_R = -1;

static const int8_t ELBOW_DIR_L    = +1;
static const int8_t ELBOW_DIR_R    = -1;

// ====================== Single motor directions (CW/CCW가 반대면 +/- 뒤집기) ======================
static const int8_t BASE_DIR        = +1;
static const int8_t WRIST_PITCH_DIR = +1;
static const int8_t WRIST_YAW_DIR   = +1;
static const int8_t GRIPPER_DIR     = +1;

// ====================== Joint index mapping ======================
// 0 base, 1 shoulder, 2 elbow, 3 wrist pitch, 4 wrist yaw, 5 gripper
static const uint8_t J_BASE = 0;
static const uint8_t J_SH   = 1;
static const uint8_t J_EL   = 2;
static const uint8_t J_WP   = 3;
static const uint8_t J_WY   = 4;
static const uint8_t J_G    = 5;

static inline float clampf(float v, float lo, float hi){
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int32_t degToRaw(float deg){
  return (int32_t)lroundf(deg * (4096.0f / 360.0f));
}

// ====================== Soft limits (deg, relative) ======================
static const float LIM_MIN[6] = { -60, -45, -60, -60, -90, -20 };
static const float LIM_MAX[6] = { +60, +45, +60, +60, +90, +20 };

// ====================== Power-safe: per-joint profile (vel/acc) ======================
static const uint32_t PROF_VEL[6] = { 25, 14, 14, 22, 22, 18 };
static const uint32_t PROF_ACC[6] = {  8,  4,  4,  6,  6,  5 };

// ====================== Power-safe: max step per update (deg) ======================
static const float MAX_STEP_DEG[6] = { 2.0f, 1.0f, 1.0f, 1.5f, 1.5f, 1.0f };

// ====================== Timing ======================
static const unsigned long SEND_PERIOD_MS = 30;
static const unsigned long SEG_DWELL_MS   = 150;

// ====================== Control Table Items ======================
using namespace ControlTableItem;

// ====================== Keep gripper vertical (new) ======================
// wrist pitch = WP_OFFSET_DEG + WP_COMP_SIGN*(shoulder + elbow)
// 보통은 WP_COMP_SIGN = -1이 맞는 경우가 많음.
// 만약 동작이 반대로 나오면 +1로 바꾸면 됨.
static bool  KEEP_VERTICAL = true;
static float WP_OFFSET_DEG = 0.0f;
static const float WP_COMP_SIGN = -1.0f;

static float computeWristPitchForVertical(float sh_deg, float el_deg){
  return WP_OFFSET_DEG + WP_COMP_SIGN * (sh_deg + el_deg);
}

// ====================== DualMotorJoint (syncWrite) ======================
struct DualMotorJoint {
  uint8_t idA, idB;
  int8_t dirA, dirB;
  int32_t zeroA=0, zeroB=0;
  ParamForSyncWriteInst_t sw;

  void init(uint8_t _idA, uint8_t _idB, int8_t _dirA, int8_t _dirB){
    idA=_idA; idB=_idB;
    dirA=_dirA; dirB=_dirB;
    sw.addr = 116;   // Goal Position
    sw.length = 4;
    sw.id_count = 2;
    sw.xel[0].id = idA;
    sw.xel[1].id = idB;
  }

  bool calibrateZero(){
    int32_t pA = (int32_t)dxl.readControlTableItem(PRESENT_POSITION, idA);
    int32_t pB = (int32_t)dxl.readControlTableItem(PRESENT_POSITION, idB);
    if (dxl.getLastLibErrCode() != 0) return false;
    zeroA=pA; zeroB=pB;
    return true;
  }

  bool writeDeg(float deg_rel){
    int32_t d = degToRaw(deg_rel);
    int32_t goalA = zeroA + (int32_t)dirA * d;
    int32_t goalB = zeroB + (int32_t)dirB * d;
    memcpy(sw.xel[0].data, &goalA, 4);
    memcpy(sw.xel[1].data, &goalB, 4);
    return dxl.syncWrite(sw);
  }
};

static DualMotorJoint shoulder;
static DualMotorJoint elbow;

// ====================== Zeros (single motors) ======================
static int32_t z_base=0, z_wp=0, z_wy=0, z_g=0;

// ====================== Current commanded joint angles (deg, relative) ======================
static float q_cur[6] = {0,0,0,0,0,0};

// ====================== Helpers ======================
static void applySoftLimits(float q[6]){
  for (int i=0;i<6;i++){
    q[i] = clampf(q[i], LIM_MIN[i], LIM_MAX[i]);
  }
}

static void setProfileMotor(uint8_t id, uint32_t vel, uint32_t acc){
  dxl.writeControlTableItem(PROFILE_VELOCITY, id, (int32_t)vel);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, id, (int32_t)acc);
}

static void applyProfilesAll(){
  setProfileMotor(ID_BASE, PROF_VEL[J_BASE], PROF_ACC[J_BASE]);

  setProfileMotor(ID_SHOULDER_L, PROF_VEL[J_SH], PROF_ACC[J_SH]);
  setProfileMotor(ID_SHOULDER_R, PROF_VEL[J_SH], PROF_ACC[J_SH]);

  setProfileMotor(ID_ELBOW_L, PROF_VEL[J_EL], PROF_ACC[J_EL]);
  setProfileMotor(ID_ELBOW_R, PROF_VEL[J_EL], PROF_ACC[J_EL]);

  setProfileMotor(ID_WRIST_PITCH, PROF_VEL[J_WP], PROF_ACC[J_WP]);
  setProfileMotor(ID_WRIST_YAW,   PROF_VEL[J_WY], PROF_ACC[J_WY]);

  setProfileMotor(ID_GRIPPER, PROF_VEL[J_G], PROF_ACC[J_G]);
}

static bool readZero(uint8_t id, int32_t &zero_out){
  int32_t p = (int32_t)dxl.readControlTableItem(PRESENT_POSITION, id);
  if (dxl.getLastLibErrCode() != 0) return false;
  zero_out = p;
  return true;
}

static bool writeSingleDeg(uint8_t id, int32_t zero, float deg, int8_t dir){
  int32_t goal = zero + (int32_t)dir * degToRaw(deg);
  return dxl.writeControlTableItem(GOAL_POSITION, id, goal);
}

// power-safe: clamp step per update
static void clampStepTowards(float out_q[6], const float desired_q[6]){
  for (int i=0;i<6;i++){
    float dq = desired_q[i] - out_q[i];
    float m  = MAX_STEP_DEG[i];
    if (dq >  m) dq =  m;
    if (dq < -m) dq = -m;
    out_q[i] += dq;
  }
}

static void writeAllNow(const float q_desired_in[6]){
  // copy desired
  float q_desired[6] = {
    q_desired_in[0], q_desired_in[1], q_desired_in[2],
    q_desired_in[3], q_desired_in[4], q_desired_in[5]
  };

  // keep-vertical override (new)
  if (KEEP_VERTICAL){
    q_desired[J_WP] = computeWristPitchForVertical(q_desired[J_SH], q_desired[J_EL]);
  }

  // step clamp
  float q_next[6] = { q_cur[0], q_cur[1], q_cur[2], q_cur[3], q_cur[4], q_cur[5] };
  clampStepTowards(q_next, q_desired);
  applySoftLimits(q_next);

  // update internal
  for (int i=0;i<6;i++) q_cur[i] = q_next[i];

  // dual joints (sync)
  shoulder.writeDeg(q_cur[J_SH]);
  elbow.writeDeg(q_cur[J_EL]);

  // single joints
  writeSingleDeg(ID_BASE,        z_base, q_cur[J_BASE], BASE_DIR);
  writeSingleDeg(ID_WRIST_PITCH, z_wp,   q_cur[J_WP],   WRIST_PITCH_DIR);
  writeSingleDeg(ID_WRIST_YAW,   z_wy,   q_cur[J_WY],   WRIST_YAW_DIR);
  writeSingleDeg(ID_GRIPPER,     z_g,    q_cur[J_G],    GRIPPER_DIR);
}

// ====================== Motion engine with segmentation ======================
struct MotionSeg {
  bool active=false;
  uint8_t mask=0;               // bit0..bit5
  unsigned long t0=0;
  unsigned long T=800;
  float q_start[6];
  float q_goal[6];
};

static MotionSeg segs[4];
static uint8_t seg_count=0;
static uint8_t seg_idx=0;

static bool motion_active=false;
static unsigned long last_send=0;
static bool dwell_active=false;
static unsigned long dwell_t0=0;

static void pushSeg(uint8_t mask, const float goal[6], unsigned long T){
  if (seg_count >= 4) return;
  MotionSeg &s = segs[seg_count++];
  s.active = true;
  s.mask = mask;
  s.t0 = 0;
  s.T  = (T < 200 ? 200 : T);
  for (int i=0;i<6;i++){
    s.q_start[i] = 0;
    s.q_goal[i]  = goal[i];
  }
}

static float absf(float x){ return x < 0 ? -x : x; }

static void buildSegmentedMove(const float target_in[6], unsigned long total_ms){
  float target[6] = {
    target_in[0],target_in[1],target_in[2],
    target_in[3],target_in[4],target_in[5]
  };

  // keep-vertical override at planning stage too (new)
  if (KEEP_VERTICAL){
    target[J_WP] = computeWristPitchForVertical(target[J_SH], target[J_EL]);
  }

  applySoftLimits(target);

  const float TH = 0.5f;

  bool need_base = absf(target[J_BASE]-q_cur[J_BASE]) > TH;
  bool need_sh   = absf(target[J_SH]  -q_cur[J_SH])   > TH;
  bool need_el   = absf(target[J_EL]  -q_cur[J_EL])   > TH;
  bool need_wp   = absf(target[J_WP]  -q_cur[J_WP])   > TH;
  bool need_wy   = absf(target[J_WY]  -q_cur[J_WY])   > TH;
  bool need_g    = absf(target[J_G]   -q_cur[J_G])    > TH;

  seg_count=0; seg_idx=0;
  dwell_active=false;

  float goal1[6] = {q_cur[0],q_cur[1],q_cur[2],q_cur[3],q_cur[4],q_cur[5]};
  float goal2[6] = {q_cur[0],q_cur[1],q_cur[2],q_cur[3],q_cur[4],q_cur[5]};
  float goal3[6] = {q_cur[0],q_cur[1],q_cur[2],q_cur[3],q_cur[4],q_cur[5]};

  unsigned long t1 = (unsigned long)(total_ms * 0.25f);
  unsigned long t2 = (unsigned long)(total_ms * 0.45f);
  unsigned long t3 = (unsigned long)(total_ms * 0.30f);
  if (t1 < 300) t1=300;
  if (t2 < 500) t2=500;
  if (t3 < 300) t3=300;

  if (need_base){
    goal1[J_BASE]=target[J_BASE];
    pushSeg( (1<<J_BASE), goal1, t1);
  }

  if (need_sh || need_el){
    goal2[J_BASE]=target[J_BASE];
    if (need_sh) goal2[J_SH]=target[J_SH];
    if (need_el) goal2[J_EL]=target[J_EL];

    // keep-vertical so WP should follow SH/EL even if wp isn't explicitly "needed"
    if (KEEP_VERTICAL){
      goal2[J_WP] = computeWristPitchForVertical(goal2[J_SH], goal2[J_EL]);
      need_wp = true;
    }

    uint8_t m = 0;
    if (need_sh) m |= (1<<J_SH);
    if (need_el) m |= (1<<J_EL);
    if (KEEP_VERTICAL) m |= (1<<J_WP);
    pushSeg(m, goal2, t2);
  }

  if (need_wp || need_wy || need_g){
    goal3[J_BASE]=target[J_BASE];
    goal3[J_SH]  =target[J_SH];
    goal3[J_EL]  =target[J_EL];

    // final wp is already computed if KEEP_VERTICAL
    if (need_wp) goal3[J_WP]=target[J_WP];
    if (need_wy) goal3[J_WY]=target[J_WY];
    if (need_g)  goal3[J_G] =target[J_G];

    uint8_t m = 0;
    if (need_wp) m |= (1<<J_WP);
    if (need_wy) m |= (1<<J_WY);
    if (need_g)  m |= (1<<J_G);
    pushSeg(m, goal3, t3);
  }

  motion_active = (seg_count > 0);
  last_send = 0;
}

static void startSegmentsNow(){
  if (!motion_active) return;
  unsigned long now = millis();
  MotionSeg &s = segs[0];
  s.t0 = now;
  for (int i=0;i<6;i++) s.q_start[i] = q_cur[i];
}

static void startNextSegment(){
  if (seg_idx >= seg_count) { motion_active=false; return; }
  unsigned long now = millis();
  MotionSeg &s = segs[seg_idx];
  s.t0 = now;
  for (int i=0;i<6;i++) s.q_start[i] = q_cur[i];
}

static void motionUpdate(){
  if (!motion_active) return;

  unsigned long now = millis();

  if (dwell_active){
    if (now - dwell_t0 >= SEG_DWELL_MS){
      dwell_active=false;
      startNextSegment();
    } else {
      return;
    }
  }

  if (seg_idx >= seg_count){
    motion_active=false;
    return;
  }

  if (last_send != 0 && (now - last_send) < SEND_PERIOD_MS) return;
  last_send = now;

  MotionSeg &s = segs[seg_idx];
  float t = (float)(now - s.t0) / (float)s.T;
  if (t > 1.0f) t = 1.0f;

  float q_des[6] = { q_cur[0],q_cur[1],q_cur[2],q_cur[3],q_cur[4],q_cur[5] };

  for (int i=0;i<6;i++){
    if (s.mask & (1<<i)){
      q_des[i] = s.q_start[i] + (s.q_goal[i] - s.q_start[i]) * t;
    }
  }

  // writeAllNow will re-apply KEEP_VERTICAL anyway
  applySoftLimits(q_des);
  writeAllNow(q_des);

  if (t >= 1.0f){
    seg_idx++;
    if (seg_idx < seg_count){
      dwell_active=true;
      dwell_t0=millis();
    } else {
      motion_active=false;
    }
  }
}

// ====================== Torque / Mode init (power-safe) ======================
static void torqueAll(bool on){
  const uint8_t ids[] = {
    ID_BASE,
    ID_SHOULDER_L, ID_SHOULDER_R,
    ID_ELBOW_L, ID_ELBOW_R,
    ID_WRIST_PITCH, ID_WRIST_YAW,
    ID_GRIPPER
  };
  for (uint8_t i=0;i<sizeof(ids);i++){
    if (on) dxl.torqueOn(ids[i]);
    else    dxl.torqueOff(ids[i]);
  }
}

static void torqueOnSequential(){
  const uint8_t ids[] = {
    ID_GRIPPER,
    ID_WRIST_PITCH, ID_WRIST_YAW,
    ID_BASE,
    ID_SHOULDER_L, ID_SHOULDER_R,
    ID_ELBOW_L, ID_ELBOW_R
  };

  for (uint8_t i=0;i<sizeof(ids);i++){
    dxl.torqueOn(ids[i]);
    delay(250);
  }
}

static void initMotorsPowerSafe(){
  const uint8_t ids[] = {
    ID_BASE,
    ID_SHOULDER_L, ID_SHOULDER_R,
    ID_ELBOW_L, ID_ELBOW_R,
    ID_WRIST_PITCH, ID_WRIST_YAW,
    ID_GRIPPER
  };

  for (uint8_t i=0;i<sizeof(ids);i++){
    dxl.torqueOff(ids[i]);
    dxl.setOperatingMode(ids[i], OP_POSITION);
    delay(20);
  }

  applyProfilesAll();
  delay(50);

  torqueOnSequential();
}

static bool doZeroAll(){
  bool ok=true;

  ok &= shoulder.calibrateZero();
  ok &= elbow.calibrateZero();

  ok &= readZero(ID_BASE, z_base);
  ok &= readZero(ID_WRIST_PITCH, z_wp);
  ok &= readZero(ID_WRIST_YAW, z_wy);
  ok &= readZero(ID_GRIPPER, z_g);

  for (int i=0;i<6;i++) q_cur[i]=0;

  float q0[6]={0,0,0,0,0,0};
  writeAllNow(q0);
  return ok;
}

// ====================== Sequence ======================
static const uint8_t SEQ_NONE=0;
static const uint8_t SEQ_PICK=1;
static uint8_t seq=SEQ_NONE;
static uint8_t seq_step=0;

static void seqStop(){
  seq=SEQ_NONE;
  seq_step=0;
}

static void startMoveSegmented(float base,float sh,float el,float wp,float wy,float g, unsigned long ms){
  float tgt[6] = {base,sh,el,wp,wy,g};
  buildSegmentedMove(tgt, ms);
  seg_idx=0;
  if (motion_active) startSegmentsNow();
}

static void seqStartPick(){
  seq=SEQ_PICK;
  seq_step=0;
}

static void seqUpdate(){
  if (seq==SEQ_NONE) return;
  if (motion_active) return;

  if (seq==SEQ_PICK){
    if (seq_step==0){
      startMoveSegmented(0,0,0,0,0,0, 1500); // HOME
      seq_step++;
    } else if (seq_step==1){
      startMoveSegmented(0,+15,-20,0,0,0, 1800); // HOVER
      seq_step++;
    } else if (seq_step==2){
      startMoveSegmented(0,+25,-35,0,0,0, 2000); // DOWN
      seq_step++;
    } else if (seq_step==3){
      startMoveSegmented(0,+25,-35,0,0,+15, 900); // GRIP close(+)
      seq_step++;
    } else if (seq_step==4){
      startMoveSegmented(0,+15,-20,0,0,+15, 1800); // LIFT
      seq_step++;
    } else if (seq_step==5){
      startMoveSegmented(0,0,0,0,0,+15, 1600); // RETURN
      seq_step++;
    } else {
      seqStop();
    }
  }
}

// ====================== Serial command handling ======================
static char rx[96];
static uint8_t rxlen=0;

static void printHelp(){
  PC_SERIAL.println("Commands:");
  PC_SERIAL.println("  help                      : show help");
  PC_SERIAL.println("  zero                      : set current pose as 0deg for all joints");
  PC_SERIAL.println("  hold / relax              : torque on / off all");
  PC_SERIAL.println("  keep on|off               : keep gripper pitch vertical using wrist pitch");
  PC_SERIAL.println("  cal_wp                    : compute WP_OFFSET_DEG from current pose");
  PC_SERIAL.println("  move <j> <deg> <ms>       : segmented move (j=base|sh|el|wp|wy|g)");
  PC_SERIAL.println("  pose home|hover|down      : preset pose (segmented)");
  PC_SERIAL.println("  run pick                  : conservative pick sequence (segmented)");
  PC_SERIAL.println("  stop                      : stop motion/sequence");
  PC_SERIAL.println("Note: Serial Monitor line ending = Newline");
}

static void cmdStop(){
  seqStop();
  motion_active=false;
  seg_count=0; seg_idx=0;
  dwell_active=false;
}

static void setJointByName(float q[6], const char* j, float val){
  if (!strcmp(j,"base")) q[J_BASE]=val;
  else if (!strcmp(j,"sh")) q[J_SH]=val;
  else if (!strcmp(j,"el")) q[J_EL]=val;
  else if (!strcmp(j,"wp")) q[J_WP]=val;
  else if (!strcmp(j,"wy")) q[J_WY]=val;
  else if (!strcmp(j,"g"))  q[J_G]=val;
}

static void processLine(char* s){
  for (int i=(int)strlen(s)-1;i>=0;i--){
    if (s[i]=='\r'||s[i]=='\n'||s[i]==' '||s[i]=='\t') s[i]=0;
    else break;
  }
  if (s[0]==0) return;

  if (!strcmp(s,"help")) { printHelp(); return; }
  if (!strcmp(s,"stop")) { cmdStop(); PC_SERIAL.println("STOP"); return; }

  if (!strcmp(s,"hold"))  { torqueAll(true);  PC_SERIAL.println("Torque ON"); return; }
  if (!strcmp(s,"relax")) { torqueAll(false); PC_SERIAL.println("Torque OFF"); return; }

  if (!strcmp(s,"zero")){
    bool ok = doZeroAll();
    PC_SERIAL.println(ok ? "Zero OK" : "Zero FAIL");
    return;
  }

  // keep on|off
  if (!strncmp(s,"keep",4)){
    strtok(s," ");
    char* v = strtok(NULL," ");
    if (!v){ PC_SERIAL.println("usage: keep on|off"); return; }
    if (!strcmp(v,"on")){
      KEEP_VERTICAL = true;
      PC_SERIAL.println("KEEP_VERTICAL = ON");
      return;
    }
    if (!strcmp(v,"off")){
      KEEP_VERTICAL = false;
      PC_SERIAL.println("KEEP_VERTICAL = OFF");
      return;
    }
    PC_SERIAL.println("usage: keep on|off");
    return;
  }

  // cal_wp: user sets gripper vertical by eye, then run cal_wp
  if (!strcmp(s,"cal_wp")){
    // offset = wp - WP_COMP_SIGN*(sh+el)
    // (because wp = offset + sign*(sh+el))
    WP_OFFSET_DEG = q_cur[J_WP] - WP_COMP_SIGN * (q_cur[J_SH] + q_cur[J_EL]);
    PC_SERIAL.print("WP_OFFSET_DEG set to: ");
    PC_SERIAL.println(WP_OFFSET_DEG, 3);
    return;
  }

  // pose home|hover|down
  if (!strncmp(s,"pose",4)){
    strtok(s," ");
    char* name = strtok(NULL," ");
    if (!name) { PC_SERIAL.println("pose ?"); return; }

    if (!strcmp(name,"home")){
      startMoveSegmented(0,0,0,0,0,0, 1500);
      return;
    }
    if (!strcmp(name,"hover")){
      startMoveSegmented(0,+15,-20,0,0,0, 1800);
      return;
    }
    if (!strcmp(name,"down")){
      startMoveSegmented(0,+25,-35,0,0,0, 2000);
      return;
    }
    PC_SERIAL.println("unknown pose");
    return;
  }

  // run pick
  if (!strncmp(s,"run",3)){
    strtok(s," ");
    char* name = strtok(NULL," ");
    if (name && !strcmp(name,"pick")){
      seqStartPick();
      PC_SERIAL.println("SEQ pick started");
      return;
    }
    PC_SERIAL.println("run ? (pick)");
    return;
  }

  // move <j> <deg> <ms>
  if (!strncmp(s,"move",4)){
    strtok(s," ");
    char* j  = strtok(NULL," ");
    char* a  = strtok(NULL," ");
    char* ms = strtok(NULL," ");
    if (!j||!a||!ms){ PC_SERIAL.println("usage: move <j> <deg> <ms>"); return; }

    // if keep vertical ON, direct wp command is ignored (policy choice)
    if (KEEP_VERTICAL && !strcmp(j,"wp")){
      PC_SERIAL.println("KEEP_VERTICAL is ON. Turn it off: keep off, then move wp ...");
      return;
    }

    float target = (float)atof(a);
    unsigned long dur = (unsigned long)atoi(ms);

    float q[6] = { q_cur[0],q_cur[1],q_cur[2],q_cur[3],q_cur[4],q_cur[5] };
    setJointByName(q, j, target);

    // keep vertical adjustment will be applied inside build/write anyway
    applySoftLimits(q);
    startMoveSegmented(q[0],q[1],q[2],q[3],q[4],q[5], dur);
    return;
  }

  PC_SERIAL.print("Unknown: ");
  PC_SERIAL.println(s);
}

// ====================== Setup / Loop ======================
void setup(){
  PC_SERIAL.begin(115200);
  while(!PC_SERIAL){}

  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(300);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);

  shoulder.init(ID_SHOULDER_L, ID_SHOULDER_R, SHOULDER_DIR_L, SHOULDER_DIR_R);
  elbow.init(ID_ELBOW_L, ID_ELBOW_R, ELBOW_DIR_L, ELBOW_DIR_R);

  initMotorsPowerSafe();

  bool ok = doZeroAll();
  PC_SERIAL.println(ok ? "Boot OK (zero set)" : "Boot OK (zero fail)");
  printHelp();

  PC_SERIAL.print("KEEP_VERTICAL: ");
  PC_SERIAL.println(KEEP_VERTICAL ? "ON" : "OFF");
  PC_SERIAL.print("WP_OFFSET_DEG: ");
  PC_SERIAL.println(WP_OFFSET_DEG, 3);
}

void loop(){
  while(PC_SERIAL.available()>0){
    char c=(char)PC_SERIAL.read();
    if (c=='\n' || c=='\r'){
      rx[rxlen]=0;
      processLine(rx);
      rxlen=0;
    } else if (rxlen < sizeof(rx)-1){
      rx[rxlen++]=c;
    }
  }

  motionUpdate();
  seqUpdate();
}

