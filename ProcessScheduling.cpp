#include "ProcessScheduling.h"

#include <algorithm>
#include <set>
#include <iostream>
#include <limits>
#include <ios>
#include <unordered_map>

void ProcessScheduler::run(){
    while(true){
      std::cout << "Welcome to CPU-Process-Scheduler" << std::endl;
      std::cout << "Which CPU Scheduling algorithm you want to use?" << std::endl;
      std::cout << "1.) First Come First Serve (FCFS)" << std::endl;
      std::cout << "2.) Shortest Job First (SJF)" << std::endl;
      std::cout << "3.) Shortest Remaining Time First (SRTF)" << std::endl;
      std::cout << "4.) Highest Response Ratio Next (HRRN)" << std::endl;
      std::cout << "5.) Round Robin Scheduling (RR)" << std::endl;
      std::cout << "6.) Priority Scheduling" << std::endl;
      std::cout << "7.) Lottery Scheduling" << std::endl;
      std::cout << "8.) Multiple Queue Scheduling" << std::endl;
      std::cout << "9.) Multilevel Feedback Queue Scheduling" << std::endl;
      std::cout << "10.) Round Robin with Aging" << std::endl;
      std::cout << "11.) Round Robin with Priority" << std::endl;
      int choice;
      std::cin >> choice;
      Scheduler* scheduler = nullptr;
      switch(choice){
          case 1: scheduler = &fcfs_; break;
          case 2: scheduler = &sjf_; break;
          case 3: scheduler = &srtf_; break;
          case 4: scheduler = &hrrn_; break;
          case 5: scheduler = &rr_; break;
          case 6: scheduler = &ps_; break;
          case 7: scheduler = &ls_; break;
          case 8: scheduler = &mqs_; break;
          case 9: scheduler = &mfqs_; break;
          case 10: scheduler = &rra_; break;
          case 11: scheduler = &rrp_; break;
          default:
            std::cout << "Invalid Choice" << std::endl;
            return;
      }
      scheduler->init();
      scheduler->schedule();
      char op;
      std::cout << std::endl << "Return to menu(y/n)? :";
      std::cin >> op;
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      if(op != 'Y' && op != 'y'){
        std::cout << "Thank you for using me.\n" << std::endl;
        std::cout << "Exiting..." << std::endl;
        return;
      }
    }
}


void ProcessScheduler::FCFS::init(){
    processes.clear();
    std::cout << "First Come First Serve (Non-preemptive algorithm)" << std::endl;
    std::cout << "Enter number of Processes: ";
    int n;
    while (!(std::cin >> n) || n <= 0) {
      std::cout << "Invalid input. Enter a positive number: ";
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    std::cout << "Enter process details (Arrival_Time Burst_Time)" << std::endl;
    for (int i = 0; i < n; i++) {
      int at,bt;
      std::cin >> at >> bt;
      addProcess(at, bt, 0, i + 1);
    }
}
void ProcessScheduler::FCFS::addProcess(int at, int bt, int pri, int pid) {
  Process p(at, bt, pri, pid);
  processes.emplace_back(p);
}
void ProcessScheduler::FCFS::schedule() {
  sort(processes.begin(), processes.end(), [&](const Process& p1, const Process& p2) {
    if (p1.arrival_time == p2.arrival_time) return p1.p_id < p2.p_id;
    return p1.arrival_time < p2.arrival_time;
  });
  int curr = 0;
  double totol_tat = 0.0, total_wt = 0.0;
  std::cout << std::endl << "Process Execution Order" << std::endl;
  std::cout << "PID\tStart\tEnd\tTAT\tWT" << std::endl;

  for (auto const& p : processes) {
    if (p.arrival_time > curr) {
      curr = p.arrival_time;
    }
    int waiting_time = curr - p.arrival_time; // waiting time = curr time - arrival time
    int tat = waiting_time + p.burst_time; // tat = waiting time + burst time
    total_wt += waiting_time;
    totol_tat += tat;
    std::cout << "P" << p.p_id << "\t" << curr << "\t" << curr + p.burst_time << "\t" <<
       tat << "\t" << waiting_time << std::endl;
    curr += p.burst_time;
  }

  std::cout << "Average Turn Around Time : " << totol_tat / processes.size() << std::endl;
  std::cout << "Average Waiting Time : " << total_wt / processes.size() << std::endl;
}

void ProcessScheduler::SJF::init() {
  processes.clear();
  std::cout << "Shortest Job First (Non-preemptive algorithm)" << std::endl;
  std::cout << "Enter number of Processes: ";
  int n;
  while (!(std::cin >> n) || n <= 0) {
    std::cout << "Invalid input. Enter a positive number: ";
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cout << "Enter process details (Arrival_Time Burst_Time)" << std::endl;
  for (int i = 0; i < n; i++) {
    int at,bt;
    std::cin >> at >> bt;
    addProcess(at, bt, 0, i + 1);
  }
}
void ProcessScheduler::SJF::addProcess(int at, int bt, int pri, int pid) {
  Process p(at, bt, pri, pid);
  processes.emplace_back(p);
}
void ProcessScheduler::SJF::schedule() {
  sort(processes.begin(), processes.end(), [&](const Process& p1, const Process& p2) {
    if (p1.arrival_time == p2.arrival_time) return p1.burst_time < p2.burst_time;
    return p1.arrival_time < p2.arrival_time;
  });
  int curr = 0;
  double totol_tat = 0.0, total_wt = 0.0;
  std::set<int> completed;
  int process_idx = 0;
  std::cout << std::endl << "Process Execution Order (Non-preemptive SJF)" << std::endl;
  std::cout << "PID\tStart\tEnd\tTAT\tWT" << std::endl;
  while (completed.size() < processes.size()) {
    while (process_idx < processes.size() && processes[process_idx].arrival_time <= curr) {
      queue.push(processes[process_idx]);
      process_idx++;
    }
    if (queue.empty()) {
      if (process_idx < processes.size()) {
        curr = processes[process_idx].arrival_time;
        continue;
      }
    }
    Process p = queue.top();
    queue.pop();
    int waiting_time = curr - p.arrival_time; // waiting time = curr time - arrival time
    int tat = waiting_time + p.burst_time; // tat = waiting time + burst time
    total_wt += waiting_time;
    totol_tat += tat;
    std::cout << "P" << p.p_id << "\t" << curr << "\t" << curr + p.burst_time << "\t" << tat << "\t" << waiting_time << std::endl;
    completed.insert(p.p_id);
    curr += p.burst_time;
  }
  std::cout << "Average Turn Around Time : " << totol_tat / processes.size() << std::endl;
  std::cout << "Average Waiting Time : " << total_wt / processes.size() << std::endl;
}

void ProcessScheduler::SRTF::init() {
  processes.clear();
  std::cout << "Shortest Remaining Time First (Preemptive algorithm)" << std::endl;
  std::cout << "Enter number of Processes: ";
  int n;
  while (!(std::cin >> n) || n <= 0) {
    std::cout << "Invalid input. Enter a positive number: ";
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cout << "Enter process details (Arrival_Time Burst_Time)" << std::endl;
  for (int i = 0; i < n; i++) {
    int at,bt;
    std::cin >> at >> bt;
    addProcess(at, bt, 0, i + 1);
  }
}
void ProcessScheduler::SRTF::addProcess(int at, int bt, int pri, int pid) {
  Process p(at, bt, pri, pid);
  processes.emplace_back(p);
}
void ProcessScheduler::SRTF::schedule() {
  std::vector<ProcessState> states;
  for (auto const &p : processes) {
    states.emplace_back(p.p_id, p.arrival_time, p.burst_time);
  }
  std::sort(states.begin(), states.end(), [&](const ProcessState& p1, const ProcessState& p2) {
    return p1.arrival_time < p2.arrival_time;
  });
  int curr_time = 0;
  double total_tat = 0.0, total_wt = 0.0;
  int counter = 0;
  ProcessState* curr = nullptr;
  int process_idx = 0;
  std::unordered_map<int, int> pid_idx;
  for (int i = 0;i < states.size(); i++) pid_idx[states[i].p_id] = i;
  std::cout << "Process Execution Order (SRTF)" << std::endl;
  std::cout << "Time\tPID" << std::endl;

  while (counter < processes.size()) {
    while (process_idx < states.size() && states[process_idx].arrival_time <= curr_time) {
      queue.push(states[process_idx]);
      process_idx++;
    }

    if (queue.empty()) {
      if (process_idx < states.size()) curr_time = states[process_idx].arrival_time;
      else break;
      continue;
    }

    if (!curr && !queue.empty()) {
      curr = new ProcessState(queue.top());
      queue.pop();
      std::cout << curr_time << "\t" << curr->p_id << std::endl;
    }

    if (!queue.empty() && curr && queue.top().remaining_time < curr->remaining_time) {
      states[pid_idx[curr->p_id]] = *curr;
      delete curr;
      curr = new ProcessState(queue.top());
      queue.pop();
      std::cout << curr_time << "\t" << curr->p_id << std::endl;
    }

    if (curr) {
      curr->remaining_time--;
      states[pid_idx[curr->p_id]] = *curr;
      curr_time++;
      if (curr->remaining_time == 0) {
        int id = pid_idx[curr->p_id];
        states[id].end_time = curr_time;
        states[id].remaining_time = 0;;
        int tat = curr_time - states[id].arrival_time;
        int wt = tat - states[id].burst_time;
        total_wt += wt;
        total_tat += tat;
        delete curr;
        curr = nullptr;
        counter++;
      }
      else {
        queue.push(states[pid_idx[curr->p_id]]);
        delete curr;
        curr = nullptr;
      }
    }
  }
  if (curr) delete curr;
  std::cout << "Process Details:" << std::endl;
  std::cout << "PID\tStart\tEnd\tRT\tTAT\tWT" << std::endl;
  for (auto const&p: states) {
    int tat = p.end_time - p.arrival_time;
    int wt = tat - p.burst_time;
    std::cout << p.p_id << "\t" << p.arrival_time << "\t" << p.end_time << "\t" << p.remaining_time << "\t" << tat << "\t" << wt << std::endl;
  }
  std::cout << "Average Turn Around Time: " << total_tat / processes.size() << std::endl;
  std::cout << "Average Waiting Time: " << total_wt / processes.size() << std::endl;
}

void ProcessScheduler::HRRN::init() {
  processes.clear();
  std::cout << "Highest Response Ratio Next (Non-Preemptive)" << std::endl;
  std::cout << "Enter number of Processes: ";
  int n;
  while (!(std::cin >> n) || n <= 0) {
    std::cout << "Invalid input. Enter a positive number: ";
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  std::cout << "Enter process details (Arrival_Time Burst_Time)" << std::endl;
  for (int i = 0; i < n; i++) {
    int at,bt;
    std::cin >> at >> bt;
    addProcess(at, bt, 0, i + 1);
  }
}
void ProcessScheduler::HRRN::addProcess(int at, int bt, int pri, int pid) {
  Process p(at, bt, pri, pid);
  processes.emplace_back(p);
}
void ProcessScheduler::HRRN::schedule() {
  std::vector<ProcessState> states;
  for (auto const &p : processes) {
    states.emplace_back(p.p_id, p.arrival_time, p.burst_time);
  }
  std::sort(states.begin(), states.end(), [&](const ProcessState& p1, const ProcessState& p2) {
    return p1.arrival_time < p2.arrival_time;
  });

  int counter = 0, curr_time = 0, process_idx = 0;
  double total_tat = 0.0, total_wt = 0.0;
  std::unordered_map<int, int> pid_idx;
  for (int i = 0;i < states.size();i++) pid_idx[states[i].p_id] = i;

  auto cmp = [&](const ProcessState& p1, const ProcessState& p2) {
    double rr1 = (curr_time - p1.arrival_time + p1.burst_time) / static_cast<double> (p1.burst_time);
    double rr2 = (curr_time - p2.arrival_time + p2.burst_time) / static_cast<double> (p2.burst_time);
    if (rr1 == rr2) return p1.p_id < p2.p_id;
    return rr1 > rr2;
  };
  std::set<ProcessState, decltype(cmp)> queue(cmp);


  std::cout << "Process Execution Order:" << std::endl;
  std::cout << "Time\tPID" << std::endl;
  while (counter < processes.size()) {
    while (process_idx < states.size() && states[process_idx].arrival_time <= curr_time) {
      queue.insert(states[process_idx]);
      process_idx++;
    }

    if (queue.empty()) {
      if (process_idx < states.size()) {curr_time = states[process_idx].arrival_time; continue;}
      break;
    }

    ProcessState curr = *queue.begin();
    queue.erase(queue.begin());
    std::cout << curr_time << "\t" << curr.p_id << std::endl;

    curr_time += curr.burst_time;
    curr.end_time = curr_time;

    int tat = curr.end_time - curr.arrival_time;
    int wt = tat - curr.burst_time;

    total_tat += tat;
    total_wt += wt;

    states[pid_idx[curr.p_id]] = curr;
    counter++;

    std::vector<ProcessState> temp;
    while (!queue.empty()) {
      temp.push_back(*queue.begin());
      queue.erase(queue.begin());
    }
    for (auto &p : temp) {
      queue.insert(p);
    }
  }

  std::cout << "Process Details:" << std::endl;
  std::cout << "PID\tStart\tEnd\tTAT\tWT" << std::endl;
  for (auto const&p: states) {
    int tat = p.end_time - p.arrival_time;
    int wt = tat - p.burst_time;
    std::cout << p.p_id << "\t" << p.arrival_time << "\t" << p.end_time << "\t" << tat << "\t" << wt << std::endl;
  }

  std::cout << "Average Turn Around Time: " << total_tat / processes.size() << std::endl;
  std::cout << "Average Waiting Time: " << total_wt / processes.size() << std::endl;
}

void ProcessScheduler::RR::init() { std::cout << "RR not implemented yet.\n"; }
void ProcessScheduler::RR::addProcess(int at, int bt, int pri, int pid) {}
void ProcessScheduler::RR::schedule() {}

void ProcessScheduler::RRP::init() { std::cout << "RRP not implemented yet.\n"; }
void ProcessScheduler::RRP::addProcess(int at, int bt, int pri, int pid) {}
void ProcessScheduler::RRP::schedule() {}

void ProcessScheduler::RRA::init() { std::cout << "RRA not implemented yet.\n"; }
void ProcessScheduler::RRA::addProcess(int at, int bt, int pri, int pid) {}
void ProcessScheduler::RRA::schedule() {}

void ProcessScheduler::MFQS::init() { std::cout << "MFQS not implemented yet.\n"; }
void ProcessScheduler::MFQS::addProcess(int at, int bt, int pri, int pid) {}
void ProcessScheduler::MFQS::schedule() {}

void ProcessScheduler::MQS::init() { std::cout << "MQS not implemented yet.\n"; }
void ProcessScheduler::MQS::addProcess(int at, int bt, int pri, int pid) {}
void ProcessScheduler::MQS::schedule() {}

void ProcessScheduler::LS::init() { std::cout << "LS not implemented yet.\n"; }
void ProcessScheduler::LS::addProcess(int at, int bt, int pri, int pid) {}
void ProcessScheduler::LS::schedule() {}

void ProcessScheduler::PS::init() { std::cout << "PS not implemented yet.\n"; }
void ProcessScheduler::PS::addProcess(int at, int bt, int pri, int pid) {}
void ProcessScheduler::PS::schedule() {}

ProcessScheduler::Process::Process(int at, int bt, int priority, int p_id) : arrival_time(at), burst_time(bt), priority(priority), p_id(p_id) {
 if (at < 0 || bt <= 0)
   throw std::invalid_argument("Invalid Arrival or Burst time");
}