#include <algorithm>
#include <deque>
std::deque<uint64_t> RETIRE_BUFFER;
#include <iostream>
#include "procsim.hpp"
#include <deque>
#include <queue>
#include <vector>
#include <unordered_map>
#include <set>
#include <fstream>
#include <iomanip>

#define DEBUG_LEVEL 0  // 0 = no debug, 1 = essential debug, 2 = verbose

// --- Tomasulo Global Variables ---

// Persistent dispatch ready flag for one-cycle delay between fetch and dispatch
bool DISPATCH_READY = false;

uint64_t PROC_R = 0, PROC_K0 = 0, PROC_K1 = 0, PROC_K2 = 0, PROC_F = 0;
uint64_t CYCLE = 0;
uint64_t NEXT_TAG = 1;
std::deque<proc_inst_t*> ROB;
std::deque<proc_inst_t*> DISPATCH_Q;
std::deque<proc_inst_t*> SCHED_Q;
// FETCH buffer for fetched but not yet dispatched instructions
std::deque<proc_inst_t*> FETCH_BUF;
std::unordered_set<uint64_t> RESULT_TAGS;
// Delayed result broadcast for Tomasulo: tags that will be broadcast next cycle
std::unordered_set<uint64_t> BROADCAST_TAGS;
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
    CYCLE = 0;
    NEXT_TAG = 1;
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
    // No exec_cycle to initialize
}

// // Helper: instruction latency by op_code
// static int instr_latency(int32_t op) {
//     if (op == 0) return 1; // k0
//     if (op == 1) return 1; // k1
//     if (op == 2) return 1; // k2
//     return 1;
// }

void fetch() {
    // Only fetch instructions from the trace and store in FETCH_BUF.
    for (uint64_t i = 0; i < PROC_F; ++i) {
        proc_inst_t* inst = new proc_inst_t();
        if (!read_instruction(inst)) {
            delete inst;
            continue;
        }
        // Only assign tag and mark fetch cycle here.
        inst->tag = NEXT_TAG++;
        inst->issued = false;
        inst->executed = false;
        inst->retired = false;
        inst->safe_to_delete = false; // Initialize safe_to_delete to false
        inst->just_retired = false;
        inst->fetch_cycle = CYCLE;
        inst->dispatch_cycle = 0;
        inst->sched_cycle = 0;
        inst->retire_cycle = 0;
        // Stage tracker: 5 stages: FETCH(0), DISP(1), SCHED(2), EXEC(3), RETIRE(4)
        STAGE_TRACKER[inst->tag] = std::vector<uint64_t>(5, 0);
        STAGE_TRACKER[inst->tag][0] = CYCLE; // FETCH
        if (DEBUG_LEVEL >= 1 && CYCLE < 10)
            std::cerr << "[CYCLE " << CYCLE << "] Fetching instruction " << inst->tag << " @ PC=0x"
                      << std::hex << inst->instruction_address << std::dec << "\n";
        FETCH_BUF.push_back(inst);
    }
}

void dispatch() {
    // Move up to PROC_F instructions from FETCH_BUF to DISPATCH_Q.
    uint64_t dispatched = 0;
    while (!FETCH_BUF.empty() && dispatched < PROC_F) {
        proc_inst_t* inst = FETCH_BUF.front();
        FETCH_BUF.pop_front();
        // Assign dependency/tag information here, as we now have access to ROB
        for (int j = 0; j < 2; ++j) {
            if (inst->src_reg[j] < 0 || inst->src_reg[j] >= 128) {
                inst->src_ready[j] = true;
                inst->src_tag[j] = 0;
                continue;
            }
            if (DEBUG_LEVEL >= 2 && CYCLE < 10) {
                std::cerr << "[DEBUG][DISPATCH] Inst tag=" << inst->tag
                          << " src_reg[" << j << "]=" << inst->src_reg[j]
                          << ", checking against ROB entries\n";
            }
            // New logic: find the last (most recent) matching ROB entry (not retired), if any
            uint64_t last_tag = 0;
            bool last_ready = true;
            for (auto rob_iter = ROB.begin(); rob_iter != ROB.end(); ++rob_iter) {
                proc_inst_t* rob_inst = *rob_iter;
                if (DEBUG_LEVEL >= 2 && CYCLE < 10) {
                    // std::cerr << "  [DEBUG][DISPATCH] ROB entry tag=" << rob_inst->tag
                    //           << ", dest_reg=" << rob_inst->dest_reg
                    //           << ", retired=" << rob_inst->retired << "\n";
                }
                if (rob_inst->dest_reg == inst->src_reg[j] && !rob_inst->retired) {
                    last_tag = rob_inst->tag;
                    last_ready = rob_inst->executed;
                }
            }
            if (last_tag != 0) {
                inst->src_ready[j] = last_ready;
                inst->src_tag[j] = last_ready ? 0 : last_tag;
            } else {
                inst->src_ready[j] = true;
                inst->src_tag[j] = 0;
            }
            if (!inst->src_ready[j] && (inst->src_tag[j] <= 0 || inst->src_tag[j] >= NEXT_TAG)) {
                std::cerr << "[ERROR] src_tag[" << j << "] = " << inst->src_tag[j]
                          << " is out of bounds for inst tag=" << inst->tag
                          << " (NEXT_TAG=" << NEXT_TAG << ")\n";
            }
        }
        DISPATCH_Q.push_back(inst);
        ROB.push_back(inst);
        inst->dispatch_cycle = CYCLE;
        STAGE_TRACKER[inst->tag][1] = CYCLE; // DISPATCH
        if (DEBUG_LEVEL >= 1 && CYCLE < 10) {
            std::cerr << "[CYCLE " << CYCLE << "] Dispatched instruction " << inst->tag
                      << " @ PC=0x" << std::hex << inst->instruction_address << std::dec << "\n";
        }
        ++dispatched;
    }
    // Track max/avg dispatch queue size
    if (DISPATCH_Q.size() > DISP_QUEUE_MAX) DISP_QUEUE_MAX = DISPATCH_Q.size();
    DISP_QUEUE_NUM += DISPATCH_Q.size();
    if (DEBUG_LEVEL >= 2 && CYCLE < 10) std::cerr << "[CYCLE " << CYCLE << "] Dispatch queue size: " << DISPATCH_Q.size() << "\n";
    // At the end of dispatch, clear FETCH_BUF
    FETCH_BUF.clear();
}

void schedule() {
    uint64_t max_sched_q_size = 2 * (PROC_K0 + PROC_K1 + PROC_K2);
    uint64_t to_schedule = DISPATCH_Q.size();
    for (uint64_t i = 0; i < to_schedule; ++i) {
        if (SCHED_Q.size() >= max_sched_q_size) {
            if (DEBUG_LEVEL >= 1 && CYCLE < 10) {
                std::cerr << "[CYCLE " << CYCLE << "] SCHED_Q full, stopping schedule\n";
            }
            break;
        }
        proc_inst_t* inst = DISPATCH_Q.front();
        DISPATCH_Q.pop_front();
        inst->sched_cycle = CYCLE;
        STAGE_TRACKER[inst->tag][2] = CYCLE;
        SCHED_Q.push_back(inst);
        if (DEBUG_LEVEL >= 1 && CYCLE < 10) {
            std::cerr << "[CYCLE " << CYCLE << "] Scheduled instruction " << inst->tag
                      << " @ PC=0x" << std::hex << inst->instruction_address << std::dec << "\n";
        }
    }
}

void execute() {
    // New FU scheduling: ordered by FU class (k0, k1, k2), FIFO within each class.
    static std::vector<uint64_t> this_cycle_tags;
    auto try_execute_class = [&](int fu_class, std::vector<uint64_t>& fu_vector) {
        for (auto& inst : SCHED_Q) {
            if (inst->issued) continue;
            int op = inst->op_code;
            if (op == -1) op = 1;
            if (op != fu_class) continue;
            if (!(inst->src_ready[0] && inst->src_ready[1] &&
                  inst->src_tag[0] == 0 && inst->src_tag[1] == 0)) continue;
            for (size_t i = 0; i < fu_vector.size(); ++i) {
                if (fu_vector[i] == 0) {
                    fu_vector[i] = 1;
                    inst->issued = true;
                    inst->executed = true;
                    // Updated logic: collect tags for this cycle
                    this_cycle_tags.push_back(inst->tag);
                    BROADCAST_TAGS.insert(inst->tag);
                    STAGE_TRACKER[inst->tag][3] = CYCLE;
                    if (DEBUG_LEVEL >= 1 && CYCLE < 10)
                        std::cerr << "[CYCLE " << CYCLE << "] Issued and executed instruction " << inst->tag
                                  << " to FU, completed at cycle " << CYCLE << "\n";
                    break;
                }
            }
        }
    };

    // Enforce FU class order: k0, then k1, then k2
    try_execute_class(0, FU_K0);
    try_execute_class(1, FU_K1);
    try_execute_class(2, FU_K2);

    // After FU execution, append sorted tags from this cycle to RETIRE_BUFFER
    if (!this_cycle_tags.empty()) {
        std::sort(this_cycle_tags.begin(), this_cycle_tags.end());
        RETIRE_BUFFER.insert(RETIRE_BUFFER.end(), this_cycle_tags.begin(), this_cycle_tags.end());
        this_cycle_tags.clear();
    }
}

void update() {
    // No need to complete instructions in update, as execution is immediate in execute()
    // Remove FU freeing logic here; FUs are now freed at retire time.

    // (Wake up logic moved below, after retire loop)

    // Debug: track tag==5 in SCHED_Q before candidate check
    if (DEBUG_LEVEL >= 1 && CYCLE < 10) {
        for (auto& inst : SCHED_Q) {
            if (inst->tag == 5) {
                std::cerr << "[DEBUG] Tracking tag 5 in SCHED_Q: "
                          << "src_ready[0]=" << inst->src_ready[0]
                          << ", src_tag[0]=" << inst->src_tag[0]
                          << ", src_ready[1]=" << inst->src_ready[1]
                          << ", src_tag[1]=" << inst->src_tag[1]
                          << ", issued=" << inst->issued
                          << ", executed=" << inst->executed
                          << ", retired=" << inst->retired << "\n";
            }
        }
        // Print ROB contents for debug
        std::cerr << "[DEBUG] ROB Contents:\n";
        for (size_t i = 0; i < ROB.size(); ++i) {
            proc_inst_t* inst = ROB[i];
            std::cerr << "  [ROB " << i << "] "
                      << "Tag=" << inst->tag
                      << " Addr=0x" << std::hex << inst->instruction_address << std::dec
                      << " Dest=" << inst->dest_reg
                      << " Done=" << inst->executed
                      << " Issued=" << inst->issued
                      << " Retired=" << inst->retired
                      << " JustRetired=" << inst->just_retired
                      << "\n";
        }
        // Print RETIRE_BUFFER contents for debug
        std::cerr << "[DEBUG] RETIRE_BUFFER Contents:\n";
        for (size_t i = 0; i < RETIRE_BUFFER.size(); ++i) {
            std::cerr << "  [RB " << i << "] Tag=" << RETIRE_BUFFER[i] << "\n";
        }
        // Print SCHED_Q contents for debug
        std::cerr << "[DEBUG] SCHED_Q Contents:\n";
        for (size_t i = 0; i < SCHED_Q.size(); ++i) {
            proc_inst_t* inst = SCHED_Q[i];
            std::cerr << "  [SCHED_Q " << i << "] "
                      << "Tag=" << inst->tag
                      << " Addr=0x" << std::hex << inst->instruction_address << std::dec
                      << " Src0=" << inst->src_reg[0]
                      << " Src1=" << inst->src_reg[1]
                      << " Dest=" << inst->dest_reg
                      << " Ready[0]=" << inst->src_ready[0]
                      << " Ready[1]=" << inst->src_ready[1]
                      << " Issued=" << inst->issued
                      << " Executed=" << inst->executed
                      << " Retired=" << inst->retired
                      << " SafeDelete=" << inst->safe_to_delete
                      << "\n";
        }
    }
    // Debug: check which instructions are ready but not issued
    if (DEBUG_LEVEL >= 1 && CYCLE < 10) {
        for (auto& inst : SCHED_Q) {
            if (!inst->issued && inst->src_ready[0] && inst->src_ready[1]) {
                std::cerr << "[DEBUG] Candidate for execution: inst tag=" << inst->tag
                          << ", src_ready[0]=" << inst->src_ready[0]
                          << ", src_tag[0]=" << inst->src_tag[0]
                          << ", src_ready[1]=" << inst->src_ready[1]
                          << ", src_tag[1]=" << inst->src_tag[1]
                          << ", issued=" << inst->issued
                          << ", executed=" << inst->executed << "\n";
            }
        }
    }

    // Retire in-order using RETIRE_BUFFER (new logic)
    // Take first PROC_R tags from RETIRE_BUFFER
    std::vector<uint64_t> retire_candidates;
    for (size_t i = 0; i < RETIRE_BUFFER.size() && i < PROC_R; ++i) {
        retire_candidates.push_back(RETIRE_BUFFER[i]);
    }
    // Sort candidates by tag value
    std::sort(retire_candidates.begin(), retire_candidates.end());
    int retire_count = 0;
    for (uint64_t tag_to_retire : retire_candidates) {
        auto rob_it = std::find_if(ROB.begin(), ROB.end(), [&](proc_inst_t* inst) {
            return inst->tag == tag_to_retire;
        });
        if (rob_it == ROB.end()) continue;
        proc_inst_t* inst = *rob_it;
        if (!inst->executed || inst->retired) continue;
        if (DEBUG_LEVEL >= 1 && CYCLE < 10)
            std::cerr << "[CYCLE " << CYCLE << "] Retiring instruction " << inst->tag << "\n";
        inst->retired = true;
        inst->just_retired = true;
        inst->safe_to_delete = true;
        inst->retire_cycle = CYCLE;
        STAGE_TRACKER[inst->tag][4] = CYCLE;
        INSTR_RETIRE_NUM++;
        // Free FU
        int op = inst->op_code;
        if (op == -1) op = 1;
        std::vector<uint64_t>* fuvec = get_fu_vec(op);
        for (size_t i = 0; i < fuvec->size(); ++i) {
            if ((*fuvec)[i] > 0) {
                (*fuvec)[i] = 0;
                break;
            }
        }
        // Remove this tag from RETIRE_BUFFER
        auto it = std::find(RETIRE_BUFFER.begin(), RETIRE_BUFFER.end(), tag_to_retire);
        if (it != RETIRE_BUFFER.end()) {
            RETIRE_BUFFER.erase(it);
        }
        retire_count++;
        if (retire_count >= PROC_R) break;
    }

    // Wake up dependent instructions in SCHED_Q (NO in-cycle wakeup from retired instructions)
    for (auto& inst : SCHED_Q) {
        for (int j = 0; j < 2; ++j) {
            if (!inst->src_ready[j]) {
                if (inst->src_tag[j] == 0) continue;
                if (BROADCAST_TAGS.count(inst->src_tag[j])) {
                    inst->src_ready[j] = true;
                    if (DEBUG_LEVEL >= 2 && CYCLE < 10) std::cerr << "[CYCLE " << CYCLE << "] Woke up src[" << j << "] of inst " << inst->tag
                              << " from tag " << inst->src_tag[j] << "\n";
                    inst->src_tag[j] = 0;
                }
                else {
                    // Fallback: If tag is not found in ROB or SCHED_Q and in range, consider it implicitly ready
                    bool valid_tag = false;
                    for (auto& rob_inst : ROB) {
                        if (rob_inst->tag == inst->src_tag[j]) {
                            valid_tag = true;
                            break;
                        }
                    }
                    for (auto& sched_inst : SCHED_Q) {
                        if (sched_inst->tag == inst->src_tag[j]) {
                            valid_tag = true;
                            break;
                        }
                    }
                    // If the tag is not valid anymore and in range, consider it implicitly ready
                    if (!valid_tag && inst->src_tag[j] > 0 && inst->src_tag[j] < NEXT_TAG) {
                        inst->src_ready[j] = true;
                        // Save for debug before clearing
                        uint64_t old_tag = inst->src_tag[j];
                        inst->src_tag[j] = 0;
                        if (DEBUG_LEVEL >= 1 && CYCLE < 10)
                            std::cerr << "[INFO] src[" << j << "] of inst " << inst->tag << " assumed ready: tag "
                                      << old_tag << " not found in ROB or SCHED_Q\n";
                    }
                }
            }
        }
    }

    // Wake up based on just-retired instructions (delayed by one cycle)
    static std::unordered_set<uint64_t> PREV_CYCLE_RETIRED_TAGS;
    std::unordered_set<uint64_t> JUST_RETIRED_TAGS = PREV_CYCLE_RETIRED_TAGS;

    // Wake up based on tags in JUST_RETIRED_TAGS (delayed effect - tags retired in previous cycle)
    for (auto& inst : SCHED_Q) {
        for (int j = 0; j < 2; ++j) {
            if (!inst->src_ready[j] && inst->src_tag[j] != 0 && JUST_RETIRED_TAGS.count(inst->src_tag[j])) {
                inst->src_ready[j] = true;
                // Save for debug before clearing
                uint64_t old_tag = inst->src_tag[j];
                inst->src_tag[j] = 0;
                if (DEBUG_LEVEL >= 1 && CYCLE < 10)
                    std::cerr << "[WAKEUP][JUST_RETIRED_DELAYED] src[" << j << "] of inst " << inst->tag
                              << " woken by delayed just-retired tag=" << old_tag << "\n";
            }
        }
    }

    // After retirement loop, update PREV_CYCLE_RETIRED_TAGS with tags of just-retired instructions (for next cycle)
    PREV_CYCLE_RETIRED_TAGS.clear();
    for (auto& inst : SCHED_Q) {
        if (inst->just_retired) {
            PREV_CYCLE_RETIRED_TAGS.insert(inst->tag);
        }
    }

    // After retiring from ROB, check all instructions (ROB and SCHED_Q) for safe deletion
    // An instruction is safe to delete if retired and no other instruction references its tag in src_tag[0/1]
    auto safe_to_delete_check = [&](proc_inst_t* inst) {
        if (!inst->retired || inst->safe_to_delete) return false;
        uint64_t tag = inst->tag;
        // Check if any instruction in ROB or SCHED_Q references this tag
        for (auto& other_inst : ROB) {
            if (other_inst->src_tag[0] == tag || other_inst->src_tag[1] == tag) return false;
        }
        for (auto& other_inst : SCHED_Q) {
            if (other_inst->src_tag[0] == tag || other_inst->src_tag[1] == tag) return false;
        }
        return true;
    };


    // Check SCHED_Q instructions for safe deletion
    for (auto it = SCHED_Q.begin(); it != SCHED_Q.end(); ) {
        proc_inst_t* inst = *it;
        if (safe_to_delete_check(inst)) {
            inst->safe_to_delete = true;
            // Do not delete or erase here; deletion deferred to run_proc()
            ++it;
        } else {
            ++it;
        }
    }

    // At the end of update, reset just_retired flags
    for (auto& inst : SCHED_Q) {
        inst->just_retired = false;
    }
}


void run_proc(proc_stats_t* p_stats)
{
    if (CYCLE < 10 && DEBUG_LEVEL >= 1) std::cerr << "[DEBUG] NEXT_TAG = " << NEXT_TAG << "\n";

    // Main simulation loop
    // Static buffers to hold instructions to be deleted from SCHED_Q, double-buffered
    static std::deque<proc_inst_t*> SCHED_Q_DELETE_BUFFER[2];

    while (true) {
        RESULT_TAGS = BROADCAST_TAGS;
        BROADCAST_TAGS.clear();

        // if (DEBUG_LEVEL >= 2 && CYCLE >= 10) break;
        if (DEBUG_LEVEL >= 1 && CYCLE < 10) std::cerr << "[CYCLE " << CYCLE << "] Entering loop: ROB=" << ROB.size()
            << ", DISPATCH_Q=" << DISPATCH_Q.size()
            << ", SCHED_Q=" << SCHED_Q.size()
            << "\n";

        // Update (retire, wakeup, FU reclaim, broadcast results)
        update();

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
        bool done = DISPATCH_Q.empty() && SCHED_Q.empty() && ROB.empty() && FETCH_BUF.empty();
        if (done) break;

        // Mark instructions for delayed deletion from SCHED_Q
        for (auto& inst : SCHED_Q) {
            if (inst->executed && inst->retired && inst->safe_to_delete) {
                SCHED_Q_DELETE_BUFFER[1].push_back(inst);
            }
        }

        // Perform actual deletion from SCHED_Q for instructions marked in previous cycle
        for (auto* inst : SCHED_Q_DELETE_BUFFER[0]) {
            auto sched_it = std::find(SCHED_Q.begin(), SCHED_Q.end(), inst);
            if (sched_it != SCHED_Q.end()) {
                SCHED_Q.erase(sched_it);
                // 在 SCHED_Q 删除后再从 ROB 删除
                auto rob_it = std::find(ROB.begin(), ROB.end(), inst);
                if (rob_it != ROB.end()) {
                    delete *rob_it;
                    ROB.erase(rob_it);
                }
            }
        }
        SCHED_Q_DELETE_BUFFER[0].clear();
        std::swap(SCHED_Q_DELETE_BUFFER[0], SCHED_Q_DELETE_BUFFER[1]);

        // Advance cycle
        CYCLE++;

        // Print lightweight progress info every 1000 cycles, even when DEBUG_LEVEL == 0
        if (CYCLE % 1000 == 0) {
            std::cerr << "[INFO] Cycle " << CYCLE << ": ROB=" << ROB.size()
                      << ", DISP_Q=" << DISPATCH_Q.size()
                      << ", SCHED_Q=" << SCHED_Q.size()
                      << ", RETIRED=" << INSTR_RETIRE_NUM << "\n";
        }

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
    p_stats->avg_disp_size = static_cast<double>(DISP_QUEUE_NUM) / (p_stats->cycle_count-1);
    p_stats->avg_inst_fired = static_cast<double>(INSTR_RETIRE_NUM) / (p_stats->cycle_count-1);
    p_stats->avg_inst_retired = static_cast<double>(p_stats->retired_instruction) / (p_stats->cycle_count-1);

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

    file << "INST\tFETCH\tDISP\tSCHED\tEXEC\tSTATE\n";
    // Convert map to vector for sorting
    std::vector<std::pair<uint64_t, std::vector<uint64_t>>> ordered_tracker(STAGE_TRACKER.begin(), STAGE_TRACKER.end());
    // Sort in ascending order by tag (C++11 compatible)
    auto cmp_asc = [](const std::pair<uint64_t, std::vector<uint64_t>>& a,
                      const std::pair<uint64_t, std::vector<uint64_t>>& b) {
        return a.first < b.first;
    };
    std::sort(ordered_tracker.begin(), ordered_tracker.end(), cmp_asc);

    for (const auto& entry : ordered_tracker) {
        const auto& tag = entry.first;
        const auto& stages = entry.second;
        file << tag;
        for (int j = 0; j < 5; ++j) {
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
    out_stat("Total run time (cycles): ", p_stats->cycle_count-1);

    file.close();
}
