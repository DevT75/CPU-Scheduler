#ifndef PROCESSSCHEDULING_H
#define PROCESSSCHEDULING_H

#include <vector>
#include <queue>

class Scheduler {
    public:
        virtual void init() = 0;
        virtual void schedule() = 0;
        virtual void addProcess(int at, int bt, int pri = 0, int pid = -1) = 0;
        virtual ~Scheduler() = default;
};

class ProcessScheduler {

    public:
      class Process {
            public:
                int arrival_time;
                int burst_time;
                int priority;
                int p_id;
                Process(int, int, int, int);
      };
      void run();

    private:
      class FCFS: public Scheduler {
          public:
                std::vector<Process> processes;
                void init() override;
                void addProcess(int at, int bt, int pri = 0, int pid = -1) override;
                void schedule() override;
      };
      class SJF : public Scheduler {
          public:
                std::vector<Process> processes;
                struct cmp {
                    bool operator()(const Process& p1, const Process& p2) {
                        if (p1.burst_time == p2.burst_time) return p1.p_id > p2.p_id;
                        return p1.burst_time > p2.burst_time;
                    }
                };
                std::priority_queue<Process, std::vector<Process>, cmp> queue;
                void init() override;
                void addProcess(int at, int bt, int pri = 0, int pid = -1) override;
                void schedule() override;
      };
      class SRTF : public Scheduler {
          public:
              std::vector<Process> processes;
              struct ProcessState {
                  int p_id, arrival_time, end_time, burst_time, remaining_time;
                  double tat, wt;
                  ProcessState(int p_id, int arrival_time, int burst_time) : p_id(p_id), arrival_time(arrival_time), burst_time(burst_time), remaining_time(burst_time), end_time(-1), tat(0), wt(0) {}
                  bool operator<(const ProcessState& p) const {
                      if (remaining_time == p.remaining_time) return p_id > p.p_id;
                      return remaining_time > p.remaining_time;
                  }
              };
              std::priority_queue<ProcessState> queue;
              void init() override;
              void addProcess(int at, int bt, int pri = 0, int pid = -1) override;
              void schedule() override;
      };
      class HRRN : public Scheduler {
          public:
              struct ProcessState {
                  int p_id, arrival_time, end_time, burst_time;
                  ProcessState(int p_id, int at, int bt): p_id(p_id), arrival_time(at), burst_time(bt), end_time(-1) {}
              };
              std::vector<Process> processes;
              void init() override;
              void addProcess(int at, int bt, int pri = 0, int pid = -1) override;
              void schedule() override;
      };
      class RR : public Scheduler {
          public:
              struct ProcessState {
                  int p_id, arrival_time, end_time, burst_time, remaining_time;
                  ProcessState(int p_id, int at, int bt): p_id(p_id), arrival_time(at), burst_time(bt), remaining_time(bt), end_time(-1) {}
              };
              void init() override;
              int time_quantum;
              std::vector<Process> processes;
              std::queue<ProcessState> queue;
              void addProcess(int at, int bt, int pri = 0, int pid = -1) override;
              void schedule() override;
      };
      class RRP : public Scheduler {
          public:
              void init() override;
              void addProcess(int at, int bt, int pri = 0, int pid = -1) override;
              void schedule() override;
      };
      class RRA : public Scheduler {
          public:
              void init() override;
              void addProcess(int at, int bt, int pri = 0, int pid = -1) override;
              void schedule() override;
      };
      class MFQS : public Scheduler {
          public:
              struct ProcessState {
                  int p_id, priority, arrival_time, end_time, burst_time, remaining_time, wait_time, curr_q_level;
                  bool io_ops, in_Q;
                  ProcessState(int p_id, int at, int bt, int p): p_id(p_id), arrival_time(at), burst_time(bt), remaining_time(bt), end_time(-1), priority(p), wait_time(0), curr_q_level(0), io_ops(false), in_Q(false) {}
              };
              std::vector<Process> processes;
              void init() override;
              void addProcess(int at, int bt, int pri = 0, int pid = -1) override;
              void schedule() override;
          private:
              std::vector<int> time_quantum = { 2, 4 ,8 };
              int max_wait_time = 10;
      };
      class MLQS : public Scheduler {
          public:
              struct ProcessState {
                  int p_id, priority, arrival_time, end_time, burst_time, remaining_time;
                  ProcessState(int p_id, int at, int bt, int p): p_id(p_id), arrival_time(at), burst_time(bt), remaining_time(bt), end_time(-1), priority(p) {}
              };
              std::vector<Process> processes;
              int time_quantum;
              void init() override;
              void addProcess(int at, int bt, int pri = 0, int pid = -1) override;
              void schedule() override;
      };
      class LS : public Scheduler {
          public:
              void init() override;
              void addProcess(int at, int bt, int pri = 0, int pid = -1) override;
              void schedule() override;
      };
      class PS : public Scheduler {
          public:
              struct ProcessState {
                  int p_id, priority, arrival_time, end_time, burst_time, remaining_time;
                  ProcessState(int p_id, int at, int bt, int p): p_id(p_id), arrival_time(at), burst_time(bt), remaining_time(bt), end_time(-1), priority(p) {}
              };
              void init() override;
              std::vector<Process> processes;
              void addProcess(int at, int bt, int pri = 0, int pid = -1) override;
              void schedule() override;
      };

      FCFS fcfs_;
      SJF sjf_;
      SRTF srtf_;
      HRRN hrrn_;
      RR rr_;
      RRP rrp_;
      RRA rra_;
      MFQS mfqs_;
      MLQS mlqs_;
      LS ls_;
      PS ps_;
};

#endif //PROCESSSCHEDULING_H