// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <string>

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

class TicTocV2
{
  public:
    TicTocV2()
    {
        tic();
    }

    explicit TicTocV2(bool display)
        : display_(display)
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    void toc(const std::string &task)
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        if (display_)
        {
            std::cout.precision(3);
            std::cout << task << ": " << elapsed_seconds.count() * 1000 << " msec." << std::endl;
        }
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
    bool display_ = false;
};
