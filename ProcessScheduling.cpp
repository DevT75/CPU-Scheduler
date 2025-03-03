#include "ProcessScheduling.h"

#include <algorithm>
#include <set>
#include <iostream>
#include <limits>
#include <ios>

void ProcessScheduler::run(){
    while(true){
      std::cout << "Welcome to CPU-Process-Scheduler" << std::endl;
      std::cout << "Which CPU Scheduling algorithm you want to use?" << std::endl;
      std::cout << "1.) First Come First Serve (FCFS)" << std::endl;
      std::cout << "2.) Shortest Job First (SJF)" << std::endl;
      std::cout << "3.) Shortest Remaining Time First (SRTF)" << std::endl;
      std::cout << "4.) Round Robin Scheduling (RR)" << std::endl;
      std::cout << "5.) Priority Scheduling" << std::endl;
      std::cout << "6.) Lottery Scheduling" << std::endl;
      std::cout << "7.) Highest Response Ratio Next (HRRN)" << std::endl;
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
          case 4: scheduler = &rr_; break;
          case 5: scheduler = &ps_; break;
          case 6: scheduler = &ls_; break;
          case 7: scheduler = &hrrn_; break;
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

void ProcessScheduler::SRTF::init() { std::cout << "SRTF not implemented yet.\n"; }
void ProcessScheduler::SRTF::addProcess(int at, int bt, int pri, int pid) {}
void ProcessScheduler::SRTF::schedule() {}

void ProcessScheduler::HRRN::init() { std::cout << "HRRN not implemented yet.\n"; }
void ProcessScheduler::HRRN::addProcess(int at, int bt, int pri, int pid) {}
void ProcessScheduler::HRRN::schedule() {}

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