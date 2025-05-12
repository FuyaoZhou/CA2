#include <iostream>
#include "procsim.hpp"
#include <deque>
#include <queue>
#include <vector>
#include <unordered_map>
#include <set>
#include <fstream>
#include <iomanip>

#define DEBUG_LEVEL 1  // 0 = no debug, 1 = essential debug, 2 = verbose

// --- Tomasulo Global Variables ---

// Persistent dispatch ready flag for one-cycle delay between fetch and dispatch
bool DISPATCH_READY = false;

uint64_t PROC_R = 0, PROC_K0 = 0, PROC_K1 = 0, PROC_K2 = 0, PROC_F = 0;
uint64_t CYCLE = 0;
uint64_t NEXT_TAG = 1;
std::unordered_map<int32_t, proc_inst_t*> RAT;
std::deque<proc_inst_t*> ROB;
std::deque<proc_inst_t*> DISPATCH_Q;
std::deque<proc_inst_t*> SCHED_Q;
std::unordered_set<uint64_t> RESULT_TAGS;
std::unordered_map<uint64_t, std::vector<uint64_t>> STAGE_TRACKER;
std::vector<uint64_t> FU_K0, FU_K1, FU_K2;
uint64_t DISP_QUEUE_MAX = 0;
uint64_t DISP_QUEUE_NUM = 0;
uint64_t INSTR_RETIRE_NUM = 0;

// Helper: get FU vector for op_code
// Treat op == -1 as equivalent to op == 1 (k1), per assignment spec
static std::vector<uint64_t>* get_fu_vec(int32_t op) {
    if (op == -1 || op == 1) return &FU_K1;
    if (op == 0) return &FU_K0;
    if (op == 2) return &FU_K2;
    return nullptr;
}

void setup_proc(uint64_t r, uint64_t k0, uint64_t k1, uint64_t k2, uint64_t f)
{
    PROC_R = r;
    PROC_K0 = k0;
    PROC_K1 = k1;
    PROC_K2 = k2;
    PROC_F = f;
    CYCLE = 1;
    NEXT_TAG = 1;
    RAT.clear();
    for (auto ptr : ROB) {
        delete ptr;
    }
    ROB.clear();
    for (auto ptr : DISPATCH_Q) {
        delete ptr;
    }
    DISPATCH_Q.clear();
    for (auto ptr : SCHED_Q) {
        delete ptr;
    }
    SCHED_Q.clear();
    RESULT_TAGS.clear();
    STAGE_TRACKER.clear();
    FU_K0 = std::vector<uint64_t>(k0, 0);
    FU_K1 = std::vector<uint64_t>(k1, 0);
    FU_K2 = std::vector<uint64_t>(k2, 0);
    DISP_QUEUE_MAX = 0;
    DISP_QUEUE_NUM = 0;
    INSTR_RETIRE_NUM = 0;
    SCHED_Q.clear();
}

// Helper: instruction latency by op_code
static int instr_latency(int32_t op) {
    if (op == 0) return 1; // k0
    if (op == 1) return 1; // k1
    if (op == 2) return 1; // k2
    return 1;
}

void fetch() {
    // std::cerr << "[CYCLE " << CYCLE << "] Attempting to fetch\n";
    // Fetch up to PROC_F instructions into dispatch queue
    for (uint64_t i = 0; i < PROC_F; ++i) {
        // Removed ROB size limit to allow unlimited ROB size
        // if (ROB.size() >= PROC_R) break;
        proc_inst_t* inst = new proc_inst_t();
        if (DEBUG_LEVEL >= 2 && CYCLE < 10) std::cerr << "[DEBUG] Calling read_instruction()\n";
        if (!read_instruction(inst)) {
            if (DEBUG_LEVEL >= 2 && CYCLE < 10) std::cerr << "[DEBUG] read_instruction() returned false\n";
            delete inst;
            break;
        }
        inst->tag = NEXT_TAG++;
        inst->issued = false;
        inst->executed = false;
        inst->retired = false;
        inst->fetch_cycle = CYCLE;
        inst->dispatch_cycle = 0;
        inst->sched_cycle = 0;
        inst->exec_cycle = 0;
        inst->retire_cycle = 0;
        // Set src_ready/src_tag, with range check for src_reg[j]
        for (int j = 0; j < 2; ++j) {
            if (inst->src_reg[j] < 0 || inst->src_reg[j] >= 128) { // added upper bound check
                inst->src_ready[j] = true;
                inst->src_tag[j] = 0;
                continue;
            }
            auto rat_it = RAT.find(inst->src_reg[j]);
            if (rat_it != RAT.end() && rat_it->second != nullptr && !rat_it->second->retired
                && rat_it->second->tag > 0 && rat_it->second->tag < NEXT_TAG) {
                inst->src_ready[j] = false;
                inst->src_tag[j] = rat_it->second->tag;
            } else {
                if (DEBUG_LEVEL >= 2 && CYCLE < 10) {
                    std::cerr << "[DEBUG] src[" << j << "] is ready or no valid RAT mapping for inst " << inst->tag << "\n";
                }
                inst->src_ready[j] = true;
                inst->src_tag[j] = 0;
            }
            // Revised error checking: only flag src_tag out of bounds if not ready
            if (!inst->src_ready[j] && (inst->src_tag[j] <= 0 || inst->src_tag[j] >= NEXT_TAG)) {
                std::cerr << "[ERROR] src_tag[" << j << "] = " << inst->src_tag[j]
                          << " is out of bounds for inst tag=" << inst->tag
                          << " (NEXT_TAG=" << NEXT_TAG << ")\n";
            }
        }
        if (DEBUG_LEVEL >= 2 && CYCLE < 10) {
            for (int j = 0; j < 2; ++j) {
                if (!inst->src_ready[j]) {
                    std::cerr << "[DEBUG] Fetch: inst " << inst->tag
                              << ", src[" << j << "] depends on tag " << inst->src_tag[j]
                              << ", from RAT[" << inst->src_reg[j] << "] = " << RAT[inst->src_reg[j]] << "\n";
                }
            }
        }
        // Update RAT for dest_reg
        if (inst->dest_reg >= 0) {
            if (DEBUG_LEVEL >= 2 && CYCLE < 10) {
                std::cerr << "[DEBUG] Updating RAT[" << inst->dest_reg << "] = inst tag " << inst->tag << "\n";
            }
            RAT[inst->dest_reg] = inst;
        }
        // Add to ROB and dispatch queue
        ROB.push_back(inst);
        DISPATCH_Q.push_back(inst);
        // Stage tracker: 5 stages: FETCH(0), DISP(1), SCHED(2), EXEC(3), RETIRE(4)
        STAGE_TRACKER[inst->tag] = std::vector<uint64_t>(5, 0);
        STAGE_TRACKER[inst->tag][0] = CYCLE; // FETCH
        if (DEBUG_LEVEL >= 1 && CYCLE < 10) std::cerr << "[CYCLE " << CYCLE << "] Fetching instruction " << inst->tag << " @ PC=0x" 
                  << std::hex << inst->instruction_address << std::dec << "\n";
    }
}

void dispatch() {
    // Move from dispatch queue to scheduling queue if possible
    size_t rs_max = 2 * (PROC_K0 + PROC_K1 + PROC_K2);
    uint64_t n = DISPATCH_Q.size();
    for (uint64_t i = 0; i < n && SCHED_Q.size() < rs_max; ++i) {
        proc_inst_t* inst = DISPATCH_Q.front();
        inst->dispatch_cycle = CYCLE;
        STAGE_TRACKER[inst->tag][1] = CYCLE; // DISPATCH
        if (DEBUG_LEVEL >= 1 && CYCLE < 10) {
            std::cerr << "[CYCLE " << CYCLE << "] Dispatched instruction " << inst->tag
                      << " @ PC=0x" << std::hex << inst->instruction_address << std::dec << "\n";
        }
        SCHED_Q.push_back(inst);
        DISPATCH_Q.pop_front();
    }
    // Track max/avg dispatch queue size
    if (DISPATCH_Q.size() > DISP_QUEUE_MAX) DISP_QUEUE_MAX = DISPATCH_Q.size();
    DISP_QUEUE_NUM += DISPATCH_Q.size();
    if (DEBUG_LEVEL >= 2 && CYCLE < 10) std::cerr << "[CYCLE " << CYCLE << "] Dispatch queue size: " << DISPATCH_Q.size() << "\n";
}

void schedule() {
    if (DEBUG_LEVEL >= 2 && CYCLE < 10) {
        std::cerr << "[DEBUG] SCHED_Q status before scheduling:\n";
        for (auto& inst : SCHED_Q) {
            std::cerr << "  Inst " << inst->tag << ", src_ready=[" << inst->src_ready[0] << ", " << inst->src_ready[1]
                      << "], src_tag=[" << inst->src_tag[0] << ", " << inst->src_tag[1] << "], issued=" << inst->issued << "\n";
        }
    }

    std::unordered_map<int32_t, int> issued_this_cycle = {{0, 0}, {1, 0}, {2, 0}, {-1, 0}};

    for (auto& inst : SCHED_Q) {
        if (inst->issued) continue;
        if (!(inst->src_ready[0] && inst->src_ready[1])) continue;

        int op = inst->op_code;
        if (op == -1) op = 1;
        std::vector<uint64_t>* fuvec = get_fu_vec(op);

        bool scheduled = false;
        for (auto& fu : *fuvec) {
            if (fu <= CYCLE) {
                // Respect FU usage limits
                if ((op == 0 && issued_this_cycle[0] >= PROC_K0) ||
                    (op == 1 && issued_this_cycle[1] >= PROC_K1) ||
                    (op == 2 && issued_this_cycle[2] >= PROC_K2)) {
                    continue;
                }

                fu = CYCLE + instr_latency(inst->op_code);
                inst->issued = true;
                inst->sched_cycle = CYCLE;
                inst->exec_cycle = fu;
                STAGE_TRACKER[inst->tag][2] = CYCLE;
                STAGE_TRACKER[inst->tag][3] = fu;
                issued_this_cycle[op]++;
                if (DEBUG_LEVEL >= 1 && CYCLE < 10)
                    std::cerr << "[CYCLE " << CYCLE << "] Scheduling instruction " << inst->tag 
                              << " to FU (exec @ " << inst->exec_cycle << ")\n";
                scheduled = true;
                break;
            }
        }

        if (!scheduled && DEBUG_LEVEL >= 2 && CYCLE < 10) {
            std::cerr << "[CYCLE " << CYCLE << "] No available FU for inst " << inst->tag << " with opcode " << inst->op_code << "\n";
        }
    }
}

void execute() {
    // Move executed instructions to result bus
    RESULT_TAGS.clear();
    // For each FU, if busy_until == CYCLE, instruction completes
    auto try_complete = [&](std::vector<uint64_t>& fuvec) {
        for (size_t i = 0; i < fuvec.size(); ++i) {
            if (fuvec[i] == CYCLE) {
                // Find inst in SCHED_Q with exec_cycle == CYCLE and issued true
                for (auto& inst : SCHED_Q) {
                    if (inst->issued && !inst->executed && inst->exec_cycle == CYCLE) {
                        if (DEBUG_LEVEL >= 1 && CYCLE < 10) std::cerr << "[CYCLE " << CYCLE << "] Executing instruction " << inst->tag
                                  << ", exec_cycle=" << inst->exec_cycle << ", FU done\n";
                        inst->executed = true;
                        // if (DEBUG_LEVEL >= 1 && CYCLE < 10) std::cerr << "[CYCLE " << CYCLE << "] Instruction " << inst->tag << " finished execution\n";
                        RESULT_TAGS.insert(inst->tag);
                        break;
                    }
                }
                fuvec[i] = 0;
            }
        }
    };
    try_complete(FU_K0);
    try_complete(FU_K1);
    try_complete(FU_K2);
}

void retire() {
    int retire_count = 0;
    while (!ROB.empty() && retire_count < PROC_R) {
        proc_inst_t* inst = ROB.front();
        if (!inst->executed || inst->retired) break;
        if (DEBUG_LEVEL >= 1 && CYCLE < 10) std::cerr << "[CYCLE " << CYCLE << "] Retiring instruction " << inst->tag << "\n";
        inst->retired = true;
        inst->retire_cycle = CYCLE;
        STAGE_TRACKER[inst->tag][4] = CYCLE; // RETIRE
        INSTR_RETIRE_NUM++;
        if (inst->dest_reg >= 0 && RAT[inst->dest_reg] == inst) {
            if (DEBUG_LEVEL >= 2 && CYCLE < 10) {
                std::cerr << "[DEBUG] Clearing RAT[" << inst->dest_reg << "] at retire for inst " << inst->tag << "\n";
            }
            RAT[inst->dest_reg] = nullptr;
        }
        ROB.pop_front();
        delete inst;
        retire_count++;
    }
}

void run_proc(proc_stats_t* p_stats)
{
    // std::cerr << "[DEBUG] Entering run_proc()\n";

    if (CYCLE < 10 && DEBUG_LEVEL >= 1) std::cerr << "[DEBUG] NEXT_TAG = " << NEXT_TAG << "\n";

    // Main simulation loop
    while (true) {
        if (DEBUG_LEVEL >= 2 && CYCLE >= 10) break;
        if (DEBUG_LEVEL >= 1 && CYCLE < 10) std::cerr << "[CYCLE " << CYCLE << "] Entering loop: ROB=" << ROB.size()
            << ", DISPATCH_Q=" << DISPATCH_Q.size()
            << ", SCHED_Q=" << SCHED_Q.size()
            << "\n";

        // Retire
        retire();
        // Clean up executed + retired instructions from SCHED_Q
        SCHED_Q.erase(
            std::remove_if(SCHED_Q.begin(), SCHED_Q.end(),
                [](proc_inst_t* inst) {
                    return inst->executed && inst->retired;
                }),
            SCHED_Q.end());
        // Execute
        execute();
        // Schedule
        schedule();

        // Dispatch (only if DISPATCH_READY from previous cycle)
        if (DISPATCH_READY) {
            dispatch();
        }

        // Fetch, then set DISPATCH_READY for next cycle
        fetch();
        DISPATCH_READY = true;

        // Check for simulation end: all queues empty, ROB empty
        bool done = DISPATCH_Q.empty() && SCHED_Q.empty() && ROB.empty();
        if (done) break;
        // Update operand readiness in scheduling queue (wakeup)
        for (auto& inst : SCHED_Q) {
            for (int j = 0; j < 2; ++j) {
                if (!inst->src_ready[j]) {
                    if (inst->src_tag[j] == 0) continue;
                    if (RESULT_TAGS.count(inst->src_tag[j])) {
                        inst->src_ready[j] = true;
                        if (DEBUG_LEVEL >= 2 && CYCLE < 10) std::cerr << "[CYCLE " << CYCLE << "] Woke up src[" << j << "] of inst " << inst->tag
                                  << " from tag " << inst->src_tag[j] << "\n";
                        inst->src_tag[j] = 0;
                    } else {
                        // Only print error if src_tag is not tracked by any current inst in ROB or SCHED_Q
                        bool valid_tag = false;
                        for (auto& rob_inst : ROB) {
                            if (rob_inst->tag == inst->src_tag[j]) valid_tag = true;
                        }
                        for (auto& sched_inst : SCHED_Q) {
                            if (sched_inst->tag == inst->src_tag[j]) valid_tag = true;
                        }
                        if (!valid_tag && inst->src_tag[j] > 0 && inst->src_tag[j] < NEXT_TAG) {
                            if (DEBUG_LEVEL >= 1 && CYCLE < 10)
                                std::cerr << "[ERROR] Invalid src_tag[" << j << "] = " << inst->src_tag[j]
                                          << " for inst tag=" << inst->tag
                                          << " (NEXT_TAG=" << NEXT_TAG << ")\n";
                        }
                        // (Removed the check for inst->src_tag[j] > NEXT_TAG)
                    }
                }
            }
        }
        // Advance cycle
        CYCLE++;

        if (CYCLE < 10 && DEBUG_LEVEL >= 1) std::cerr << "[CYCLE " << CYCLE << "] ROB=" << ROB.size()
                  << " DISP_Q=" << DISPATCH_Q.size()
                  << " SCHED_Q=" << SCHED_Q.size()
                  << " RESULT_TAGS=" << RESULT_TAGS.size()
                  << " RETIRED=" << INSTR_RETIRE_NUM << std::endl;
    }
    // Set stats
    p_stats->cycle_count = CYCLE;
    p_stats->retired_instruction = INSTR_RETIRE_NUM;
}

// Finalizes statistics after simulation ends and writes results to output file.
void complete_proc(proc_stats_t *p_stats) 
{
    p_stats->max_disp_size = DISP_QUEUE_MAX;
    p_stats->avg_disp_size = static_cast<double>(DISP_QUEUE_NUM) / p_stats->cycle_count;
    p_stats->avg_inst_fired = static_cast<double>(INSTR_RETIRE_NUM) / p_stats->cycle_count;
    p_stats->avg_inst_retired = static_cast<double>(p_stats->retired_instruction) / p_stats->cycle_count;

    std::ofstream file("result_test.output", std::ios::out | std::ios::trunc);

    auto out_setting = [&](const char* name, uint64_t val) { file << name << ": " << val << "\n"; };
    auto out_stat = [&](const char* label, uint64_t val) { file << label << val << "\n"; };

    file << "Processor Settings\n";
    out_setting("R", PROC_R);
    out_setting("k0", PROC_K0);
    out_setting("k1", PROC_K1);
    out_setting("k2", PROC_K2);
    out_setting("F", PROC_F);
    file << "\n";

    file << "INST\tFETCH\tDISP\tSCHED\tEXEC\tRETIRE\n";
    for (auto it = STAGE_TRACKER.begin(); it != STAGE_TRACKER.end(); ++it) {
        const auto& tag = it->first;
        const auto& stages = it->second;
        file << tag;
        for (int j = 0; j < 5; ++j) {
            // Add 1 to each stage to reflect cycle counting from 1
            file << "\t" << (stages[j] + 1);
        }
        file << "\n";
    }

    file << "\nProcessor stats:\n";
    out_stat("Total instructions: ", p_stats->retired_instruction);
    file << std::fixed << std::setprecision(6);
    file << "Avg Dispatch queue size: " << p_stats->avg_disp_size << "\n";
    out_stat("Maximum Dispatch queue size: ", p_stats->max_disp_size);
    file << std::fixed << std::setprecision(6);
    file << "Avg inst fired per cycle: " << p_stats->avg_inst_fired << "\n";
    file << "Avg inst retired per cycle: " << p_stats->avg_inst_retired << "\n";
    out_stat("Total run time (cycles): ", p_stats->cycle_count);

    file.close();
}
