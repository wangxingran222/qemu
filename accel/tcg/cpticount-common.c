#include "qemu/osdep.h"
#include "sysemu/cpus.h"
#include "sysemu/cpticount.h"

int64_t cpticount = 0;
CptICountMode cpticount_status = CPTICOUNT_DISABLED;

static int64_t cpticount_get_executed(CPUState *cpu)
{
    return (cpu->icount_budget -
            (cpu->neg.icount_decr.u16.low + cpu->icount_extra));
}

void cpticount_update(CPUState *cpu)
{
    /* update cpticount without lock? */
    int64_t executed = cpticount_get_executed(cpu);
    cpu->icount_budget -= executed;
    /* synchronize with global var? */
    cpticount = cpu->icount_budget;
}

void cpticount_prepare_for_run(CPUState *cpu)
{
    int insns_left;

    /*
     * DEBUG
     * cpu->icount_budget should be the same with cpticount
     * DEBUG
     */
    g_assert(cpu->icount_budget == cpticount);

    insns_left = MIN(0xffff, cpu->icount_budget);
    cpu->neg.icount_decr.u16.low = insns_left;
    cpu->icount_extra = cpu->icount_budget - insns_left;

    if (cpu->icount_budget == 0) {
        /*
         * DEBUG
         * icount_budget == 0, time to terminate?
         */
        printf("icount_budget == 0 in prepare for run\n");
        exit(0);
    }
}

void cpticount_process_data(CPUState *cpu)
{
    /* Account for executed instructions */
    cpticount_update(cpu);

    /* Reset the counters */
    cpu->neg.icount_decr.u16.low = 0;
    cpu->icount_extra = 0;

}

void cpticount_init(CPUState *cpu)
{
    /* init icount_budget */
    cpu->icount_budget = cpticount;
}