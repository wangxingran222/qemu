/*
 * CPU CPTICOUNT API
 *
 */
#ifndef SYSEMU_CPTICOUNT_H
#define SYSEMU_CPTICOUNT_H

typedef enum {
    CPTICOUNT_DISABLED = 0,
    CPTICOUNT_ENABLED,
} CptICountMode;

#if defined(CONFIG_TCG) && !defined(CONFIG_USER_ONLY)
extern int64_t cpticount;
extern CptICountMode cpticount_status;
#define cpticount_enabled() (cpticount_status)
#else
#define cpticount_enabled() 0
#endif

void cpticount_update(CPUState *cpu);
void cpticount_prepare_for_run(CPUState *cpu);
void cpticount_process_data(CPUState *cpu);
void cpticount_init(CPUState *cpu);

#endif /* SYSEMU_CPTICOUNT_H */
