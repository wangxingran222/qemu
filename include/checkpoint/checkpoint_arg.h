#ifndef CHECKPOINT_ARG_H
#define CHECKPOINT_ARG_H

#define STRING_LEN 100
#define SIMPOINT_IDX_MAX 100
#include "qemu/osdep.h"

enum CheckpointState{
    NoCheckpoint=0,
    SimpointCheckpointing,
    UniformCheckpointing,
    ManualOneShotCheckpointing,
    ManualUniformCheckpointing,
};
enum ProfilingState{
    NoProfiling =0,
    SimpointProfiling,
};

struct PathManger{
    char statsBaseDir[STRING_LEN];
    char configName[STRING_LEN];
    char workloadName[STRING_LEN];
    int cptID;
    char workloadPath[STRING_LEN];
    char outputPath[STRING_LEN];
    char simpointPath[STRING_LEN];
};
struct Serializer{
    uint64_t intervalSize;
    uint64_t nextUniformPoint;
    uint64_t simpoints[SIMPOINT_IDX_MAX];
    double weights[SIMPOINT_IDX_MAX];
};
typedef struct PathManger PathManger;
typedef struct Serializer Serializer;

extern int checkpoint_state;
extern Serializer serializer;
extern PathManger pathmanger;

#endif