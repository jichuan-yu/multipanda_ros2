// Standalone MuJoCo regression for the viewer's release operation.
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>
#include <mujoco_ros/drag_release.h>

int main(int argc, char** argv) {
  assert(argc == 2);
  char error[1024];
  mjModel* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!m) { std::cerr << error; return 1; }
  mjData* d = mj_makeData(m);
  const int ball = mj_name2id(m, mjOBJ_BODY, "ball");
  const int child = mj_name2id(m, mjOBJ_BODY, "ball_child");
  const int falling = mj_name2id(m, mjOBJ_BODY, "falling");
  const int robot = mj_name2id(m, mjOBJ_BODY, "robot");
  const int bd = m->jnt_dofadr[m->body_jntadr[ball]];
  const int fd = m->jnt_dofadr[m->body_jntadr[falling]];
  const int rd = m->jnt_dofadr[m->body_jntadr[robot]];
  const int bp = m->jnt_qposadr[m->body_jntadr[ball]];
  const int fp = m->jnt_qposadr[m->body_jntadr[falling]];
  for (int i=0; i<m->nv; ++i) d->qvel[i] = .1;
  for (int i=0; i<6; ++i) d->qacc_warmstart[bd+i] = 10;
  const std::vector<mjtNum> pose(d->qpos, d->qpos + m->nq);
  assert(!mujoco_ros::StopReleasedBody(m,d,robot));
  assert(!mujoco_ros::StopReleasedBody(m,d,0));
  assert(!mujoco_ros::StopReleasedBody(m,d,-1));
  assert(!mujoco_ros::StopReleasedBody(m,d,m->nbody));
  assert(mujoco_ros::StopReleasedBody(m,d,child));
  for (int i=0; i<6; ++i) {
    assert(d->qvel[bd+i] == 0);
    assert(d->qacc_warmstart[bd+i] == 0);
  }
  assert(d->qvel[fd] == .1 && d->qvel[rd] == .1);
  for (int i=0; i<m->nq; ++i) assert(d->qpos[i] == pose[i]);
  // Full gravity compensation: release stops drift without a weld or teleport.
  for (int i=0; i<15000; ++i) mj_step(m,d);
  for (int i=0; i<3; ++i) assert(std::abs(d->qpos[bp+i]-pose[bp+i]) < 1e-8);
  // Ordinary objects still experience gravity after their velocity is cleared.
  assert(mujoco_ros::StopReleasedBody(m,d,falling));
  const double z = d->qpos[fp+2];
  for (int i=0; i<50; ++i) mj_step(m,d);
  assert(d->qpos[fp+2] < z - .01);
  mj_deleteData(d); mj_deleteModel(m);
  std::cout << "PASS: free body and rigid child stop, pose unchanged, other joints untouched, "
               "floating body stationary for 30 s, gravity retained\n";
}
