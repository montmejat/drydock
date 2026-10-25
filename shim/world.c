// The few calls drydock makes of MuJoCo, in C: whatever reads or writes
// MuJoCo's structs lives here, compiled against its headers, so the Rust
// side never mirrors a struct layout.
//
// Errors come back as a nonzero return (or NULL) and a message in `err`.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <mujoco/mujoco.h>

typedef struct {
  mjModel* m;
  mjData* d;
} dd_world;

static void on_warning(const char* msg) { fprintf(stderr, "drydock: MuJoCo: %s\n", msg); }

dd_world* dd_load(const char* path, char* err, int len) {
  mju_user_warning = on_warning;
  if (mj_version() != mjVERSION_HEADER) {
    snprintf(err, len, "MuJoCo's library is version %d, its headers %d", mj_version(), mjVERSION_HEADER);
    return NULL;
  }
  mjModel* m = mj_loadXML(path, NULL, err, len);
  if (!m) return NULL;
  dd_world* w = malloc(sizeof *w);
  w->m = m;
  w->d = mj_makeData(m);
  mj_forward(w->m, w->d);
  return w;
}

void dd_free(dd_world* w) {
  mj_deleteData(w->d);
  mj_deleteModel(w->m);
  free(w);
}

// A hinge or slide joint by name: its index, and where its position and
// velocity are.
int dd_joint(const dd_world* w, const char* name, int* id, int* qpos, int* dof, char* err, int len) {
  int j = mj_name2id(w->m, mjOBJ_JOINT, name);
  if (j < 0) {
    snprintf(err, len, "no joint in the scene called `%s`", name);
    return 1;
  }
  int type = w->m->jnt_type[j];
  if (type != mjJNT_HINGE && type != mjJNT_SLIDE) {
    snprintf(err, len, "joint `%s` is neither a hinge nor a slide: a motor can't drive it", name);
    return 1;
  }
  *id = j;
  *qpos = w->m->jnt_qposadr[j];
  *dof = w->m->jnt_dofadr[j];
  return 0;
}

// Silences the scene's own actuators on joint `j` (zero gain, zero bias),
// since its motor drives it now. Returns how many there were.
int dd_release_actuators(dd_world* w, int j) {
  int n = 0;
  for (int i = 0; i < w->m->nu; i++) {
    int trn = w->m->actuator_trntype[i];
    if ((trn == mjTRN_JOINT || trn == mjTRN_JOINTINPARENT) && w->m->actuator_trnid[2 * i] == j) {
      memset(w->m->actuator_gainprm + i * mjNGAIN, 0, mjNGAIN * sizeof(mjtNum));
      memset(w->m->actuator_biasprm + i * mjNBIAS, 0, mjNBIAS * sizeof(mjtNum));
      n++;
    }
  }
  return n;
}

int dd_keyframe(dd_world* w, const char* name, char* err, int len) {
  int k = mj_name2id(w->m, mjOBJ_KEY, name);
  if (k < 0) {
    snprintf(err, len, "no keyframe in the scene called `%s`", name);
    return 1;
  }
  mj_resetDataKeyframe(w->m, w->d, k);
  mj_forward(w->m, w->d);
  return 0;
}

double dd_time(const dd_world* w) { return w->d->time; }
double dd_timestep(const dd_world* w) { return w->m->opt.timestep; }
double dd_q(const dd_world* w, int qpos) { return w->d->qpos[qpos]; }
double dd_v(const dd_world* w, int dof) { return w->d->qvel[dof]; }
void dd_set_force(dd_world* w, int dof, double f) { w->d->qfrc_applied[dof] = f; }
void dd_step(dd_world* w) { mj_step(w->m, w->d); }

// The torque on a joint's degree of freedom from everything but its motor
// and its own friction: gravity and the other bodies' motion (the bias, with
// its sign turned), contacts and limits (the constraints), less the force of
// the dof's own friction constraint. Each friction row of a dof is a unit
// row on it, so its force is its share of `qfrc_constraint`. As of the last
// step.
double dd_load_torque(const dd_world* w, int dof) {
  double friction = 0;
  for (int i = 0; i < w->d->nefc; i++) {
    if (w->d->efc_type[i] == mjCNSTR_FRICTION_DOF && w->d->efc_id[i] == dof) friction += w->d->efc_force[i];
  }
  return -w->d->qfrc_bias[dof] + w->d->qfrc_constraint[dof] - friction;
}

// A dof's dry friction (N m, or N) and viscous damping (N m s/rad, or N s/m),
// from the next step on.
void dd_set_friction(dd_world* w, int dof, double frictionloss, double damping) {
  w->m->dof_frictionloss[dof] = frictionloss;
  w->m->dof_damping[dof] = damping;
}
