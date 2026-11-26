// Drives the simulated EtherCAT arm with SOEM, as a CiA 402 master drives
// real drives: checks over SDO that each drive has CSP and reads its
// encoder's counts a turn, sets CSP, takes the line to Operational, enables
// each drive with the controlword (Shutdown, Switch On, Enable Operation),
// then moves the shoulder 0.5 rad and back to Switched On.
//
//   examples/ethercat-arm/enable ecm0
//
// Exits 1, saying why, if a drive doesn't enable, a cycle is missed, or the
// shoulder doesn't get there. Built against SOEM 2.0.1: see docs/ethercat.md.

#include "soem/soem.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define DRIVES 3
#define CSP 8
#define SHOULDER 1

static const char *joints[DRIVES] = {"base", "shoulder", "elbow"};

// The default PDOs drydock's drives have (0x1600, 0x1A00).
struct outputs {
    uint16 controlword; // 0x6040
    int32 target;       // 0x607A
    int32 velocity;     // 0x60FF
    int16 torque;       // 0x6071
    int8 mode;          // 0x6060
} __attribute__((packed));

struct inputs {
    uint16 statusword; // 0x6041
    int32 position;    // 0x6064
    int32 velocity;    // 0x606C
    int16 torque;      // 0x6077
    int8 mode;         // 0x6061
    uint16 error;      // 0x603F
} __attribute__((packed));

static ecx_contextt ctx;
static uint8 map[4096];
static int expected_wkc;
static struct timespec next;

static const char *state_name(uint16 sw) {
    if ((sw & 0x4F) == 0x40) return "Switch On Disabled";
    if ((sw & 0x6F) == 0x21) return "Ready To Switch On";
    if ((sw & 0x6F) == 0x23) return "Switched On";
    if ((sw & 0x6F) == 0x27) return "Operation Enabled";
    if ((sw & 0x6F) == 0x07) return "Quick Stop Active";
    if ((sw & 0x4F) == 0x0F) return "Fault Reaction Active";
    if ((sw & 0x4F) == 0x08) return "Fault";
    return "Not Ready To Switch On";
}

static struct outputs *out(int d) { return (struct outputs *)ctx.slavelist[d + 1].outputs; }
static struct inputs *in(int d) { return (struct inputs *)ctx.slavelist[d + 1].inputs; }

// One cycle, every millisecond: the outputs out, the inputs back, every
// drive counted in the working counter.
static int cycle(void) {
    ecx_send_processdata(&ctx);
    int wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    next.tv_nsec += 1000000;
    if (next.tv_nsec >= 1000000000) {
        next.tv_nsec -= 1000000000;
        next.tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    if (wkc < expected_wkc) {
        printf("a cycle came back with working counter %d, not %d: a drive missed it\n", wkc, expected_wkc);
        return 0;
    }
    return 1;
}

static int fail(const char *why) {
    printf("%s\n", why);
    ctx.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&ctx, 0);
    ecx_close(&ctx);
    return 1;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        printf("usage: %s IFNAME (the master's end of the veth pair, e.g. ecm0)\n", argv[0]);
        return 1;
    }
    if (!ecx_init(&ctx, argv[1])) {
        printf("can't open %s: it needs raw sockets (run in `unshare -rn`, or with cap_net_raw)\n", argv[1]);
        return 1;
    }
    if (ecx_config_init(&ctx) != DRIVES) {
        printf("found %d drives, not %d: is drydock running examples/ethercat-arm/robot.yml?\n", ctx.slavecount, DRIVES);
        ecx_close(&ctx);
        return 1;
    }

    // In Pre-Op, over SDO, as a master configures drives it doesn't know.
    // SOEM sends no mailbox until it has read each drive's state as Pre-Op.
    if (ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE) != EC_STATE_PRE_OP)
        return fail("the drives didn't reach Pre-Operational");
    ecx_readstate(&ctx);
    double counts[DRIVES];
    for (int d = 0; d < DRIVES; d++) {
        uint32 modes = 0, turn = 0;
        int size = sizeof(modes);
        if (ecx_SDOread(&ctx, d + 1, 0x6502, 0, FALSE, &size, &modes, EC_TIMEOUTRXM) <= 0 || !(modes & (1 << 7)))
            return fail("a drive doesn't say it has CSP (0x6502 bit 7)");
        size = sizeof(turn);
        if (ecx_SDOread(&ctx, d + 1, 0x608F, 1, FALSE, &size, &turn, EC_TIMEOUTRXM) <= 0 || turn == 0)
            return fail("a drive has no encoder resolution (0x608F:01)");
        int8 mode = CSP;
        if (ecx_SDOwrite(&ctx, d + 1, 0x6060, 0, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM) <= 0)
            return fail("a drive refused CSP (0x6060)");
        counts[d] = turn;
        printf("drive %d, %s: CSP, %u counts a turn\n", d + 1, joints[d], turn);
    }

    ecx_config_map_group(&ctx, map, 0);
    ec_groupt *group = &ctx.grouplist[0];
    expected_wkc = group->outputsWKC * 2 + group->inputsWKC;
    if (group->Obytes != DRIVES * sizeof(struct outputs) || group->Ibytes != DRIVES * sizeof(struct inputs))
        return fail("the drives' PDOs aren't drydock's defaults");
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    clock_gettime(CLOCK_MONOTONIC, &next);
    for (int d = 0; d < DRIVES; d++) out(d)->mode = CSP;
    cycle();
    ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx, 0);
    for (int i = 0; i < 100 && ctx.slavelist[0].state != EC_STATE_OPERATIONAL; i++) {
        cycle();
        ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, 1000);
    }
    if (ctx.slavelist[0].state != EC_STATE_OPERATIONAL) return fail("the line didn't reach Operational");
    printf("line: Operational, working counter %d\n", expected_wkc);

    // CiA 402: the target held at the actual position until enabled, so
    // the drive doesn't jump.
    uint16 seen[DRIVES] = {0};
    int enabled = 0;
    for (int i = 0; i < 1000 && enabled < DRIVES; i++) {
        if (!cycle()) return fail("missed a cycle while enabling");
        enabled = 0;
        for (int d = 0; d < DRIVES; d++) {
            uint16 sw = in(d)->statusword;
            if (strcmp(state_name(sw), state_name(seen[d])) || i == 0) {
                printf("drive %d, %s: %s (statusword 0x%04X)\n", d + 1, joints[d], state_name(sw), sw);
                seen[d] = sw;
            }
            uint16 cw;
            if ((sw & 0x4F) == 0x40) cw = 0x06;
            else if ((sw & 0x6F) == 0x21) cw = 0x07;
            else if ((sw & 0x4F) == 0x08) return fail("a drive is in Fault");
            else cw = 0x0F;
            if ((sw & 0x6F) == 0x27) enabled++;
            else out(d)->target = in(d)->position;
            out(d)->controlword = cw;
        }
    }
    if (enabled < DRIVES) return fail("not every drive reached Operation Enabled within 1 s");

    // The shoulder 0.5 rad up, ramped over 1 s, then held for 0.5 s.
    int32 start[DRIVES];
    for (int d = 0; d < DRIVES; d++) start[d] = in(d)->position;
    double goal = 0.5 / (2 * M_PI) * counts[SHOULDER];
    for (int k = 0; k < 1500; k++) {
        out(SHOULDER)->target = start[SHOULDER] + (int32)(goal * fmin(k / 1000.0, 1.0));
        if (!cycle()) return fail("missed a cycle while moving");
        for (int d = 0; d < DRIVES; d++)
            if ((in(d)->statusword & 0x6F) != 0x27) return fail("a drive left Operation Enabled while moving");
    }
    for (int d = 0; d < DRIVES; d++) {
        double moved = (in(d)->position - start[d]) / counts[d] * 2 * M_PI;
        printf("%s: moved %.3f rad\n", joints[d], moved);
    }
    double moved = (in(SHOULDER)->position - start[SHOULDER]) / goal;
    if (moved < 0.9 || moved > 1.1) return fail("the shoulder didn't reach 0.5 rad");

    // Disable Operation: back to Switched On, the torque off.
    for (int d = 0; d < DRIVES; d++) out(d)->controlword = 0x07;
    for (int i = 0; i < 10; i++)
        if (!cycle()) return fail("missed a cycle while disabling");
    for (int d = 0; d < DRIVES; d++) {
        printf("drive %d, %s: %s\n", d + 1, joints[d], state_name(in(d)->statusword));
        if ((in(d)->statusword & 0x6F) != 0x23) return fail("a drive didn't go back to Switched On");
    }

    ctx.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&ctx, 0);
    ecx_close(&ctx);
    return 0;
}
