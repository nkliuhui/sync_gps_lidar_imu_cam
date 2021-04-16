#pragma once

#include <cstdlib>
#include <chrono>
#include <iostream>

class TicToc {
 public:
  TicToc() {
    Tic();
  }

  void Tic() {
    start_ = std::chrono::system_clock::now();
  }

  double Toc() {
    end_ = std::chrono::system_clock::now();
    elapsed_seconds_ = end_ - start_;
    return elapsed_seconds_.count() * 1000;
  }

  double GetLastStop() {
    return elapsed_seconds_.count() * 1000;
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start_, end_;
  std::chrono::duration<double> elapsed_seconds_;
};

