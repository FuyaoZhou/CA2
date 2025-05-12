#ifndef PROCSIM_HPP
#define PROCSIM_HPP

#include <cstdint>
#include <cstdio>

#define DEFAULT_K0 1
#define DEFAULT_K1 2
#define DEFAULT_K2 3
#define DEFAULT_R 8
#define DEFAULT_F 4

// Tomasulo pipeline instruction structure
typedef struct _proc_inst_t
{
    uint32_t instruction_address;
    int32_t op_code;
    int32_t src_reg[2];
    int32_t dest_reg;

    // Tomasulo fields
    uint64_t tag;             // Unique tag for instruction
    bool src_ready[2];        // Whether source operands are ready
    uint64_t src_tag[2];      // If not ready, what tag to wait for
    bool issued;              // Has been issued to FU
    bool executed;            // Has finished execution
    bool retired;             // Has retired
    uint64_t fetch_cycle;
    uint64_t dispatch_cycle;
    uint64_t sched_cycle;
    uint64_t retire_cycle;
    bool safe_to_delete;      // New flag to mark when instruction is safe to delete
} proc_inst_t;
// --- Tomasulo Global Structures and Variables ---
#include <queue>
#include <deque>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <iostream> // Added for debug output

// Processor configuration
extern uint64_t PROC_R; // ROB size
extern uint64_t PROC_K0, PROC_K1, PROC_K2; // FU counts
extern uint64_t PROC_F; // Fetch width

// Cycle counter
extern uint64_t CYCLE;

// Instruction tag counter
extern uint64_t NEXT_TAG;


// ROB: Reorder Buffer (circular buffer)
extern std::deque<proc_inst_t*> ROB;

// Dispatch queue (FIFO)
extern std::deque<proc_inst_t*> DISPATCH_Q;

// Scheduling queue (Reservation Stations)
extern std::deque<proc_inst_t*> SCHED_Q;

// FETCH buffer for fetched but not yet dispatched instructions
extern std::deque<proc_inst_t*> FETCH_BUF;

// Result tags (instructions that have completed execution this cycle)
extern std::unordered_set<uint64_t> RESULT_TAGS;

// Stage tracker: for output
extern std::unordered_map<uint64_t, std::vector<uint64_t>> STAGE_TRACKER; // tag -> [fetch, disp, sched, exec, retire]

// Functional Unit status
extern std::vector<uint64_t> FU_K0; // busy_until times
extern std::vector<uint64_t> FU_K1;
extern std::vector<uint64_t> FU_K2;

// Max/avg dispatch queue size tracking
extern uint64_t DISP_QUEUE_MAX;
extern uint64_t DISP_QUEUE_NUM;
extern uint64_t INSTR_RETIRE_NUM;

// Function prototypes for pipeline stages
void fetch();
void dispatch();
void schedule();
void execute();
void update();

typedef struct _proc_stats_t
{
    float avg_inst_retired;
    float avg_inst_fired;
    float avg_disp_size;
    unsigned long max_disp_size;
    unsigned long retired_instruction;
    unsigned long cycle_count;
} proc_stats_t;

bool read_instruction(proc_inst_t* p_inst);

void setup_proc(uint64_t r, uint64_t k0, uint64_t k1, uint64_t k2, uint64_t f);
void run_proc(proc_stats_t* p_stats);

void complete_proc(proc_stats_t *p_stats);

#endif /* PROCSIM_HPP */
