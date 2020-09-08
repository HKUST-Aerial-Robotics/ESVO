#ifndef esvo_tictoc_H_
#define esvo_tictoc_H_

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

namespace esvo_time_surface
{
class TicToc
{
  public:
  TicToc()
  {
    tic();
  }

  void tic()
  {
    start = std::chrono::system_clock::now();
  }

  double toc()
  {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

  private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};
}

#endif // esvo_tictoc_H_
