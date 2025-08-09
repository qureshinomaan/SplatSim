#pragma once
#ifndef RTDE_RTDE_UTILITY_H
#define RTDE_RTDE_UTILITY_H

#include <ur_rtde/rtde_export.h>
#include <string>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <thread>
#include <cmath>
#include <chrono>
#include <ratio>
#include <ctime>
#include <cerrno>
#include <cassert>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <pthread.h>
#include <fstream>
#endif

#define SLACK_TIME_IN_MICROS 300UL

namespace ur_rtde
{
struct RTDEControlHeader
{
  uint16_t msg_size;
  uint8_t msg_cmd;
};

#if defined(__linux__) || defined(__APPLE__)
class PriorityInheritanceMutex
{
 public:
  PriorityInheritanceMutex()
  {
    pthread_mutexattr_t mattr;
    pthread_mutexattr_init(&mattr);
    pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&mutex_, &mattr);
  }

  PriorityInheritanceMutex(const PriorityInheritanceMutex &other) = delete;
  PriorityInheritanceMutex(PriorityInheritanceMutex &&other) = default;

  PriorityInheritanceMutex &operator=(const PriorityInheritanceMutex &other) = delete;
  PriorityInheritanceMutex &operator=(PriorityInheritanceMutex &&other) = default;

  ~PriorityInheritanceMutex()
  {
    pthread_mutex_destroy(&mutex_);
  }

  void lock()
  {
    pthread_mutex_lock(&mutex_);
  }

  void unlock()
  {
    pthread_mutex_unlock(&mutex_);
  }

  bool try_lock()
  {
    return 0 == pthread_mutex_trylock(&mutex_);
  }

 private:
  pthread_mutex_t mutex_;
};
#endif

class RTDEUtility
{
 public:
  static inline RTDEControlHeader readRTDEHeader(const std::vector<char> &data, uint32_t &message_offset)
  {
    RTDEControlHeader rtde_control_header;
    rtde_control_header.msg_size = RTDEUtility::getUInt16(data, message_offset);
    rtde_control_header.msg_cmd = RTDEUtility::getUInt8(data, message_offset);
    return rtde_control_header;
  }

  static inline std::vector<char> packUInt32(uint32_t uint32)
  {
    std::vector<char> result;
    result.push_back(uint32 >> 24);
    result.push_back(uint32 >> 16);
    result.push_back(uint32 >> 8);
    result.push_back(uint32);
    return result;
  }

  static inline std::vector<char> packInt32(int32_t int32)
  {
    std::vector<char> result;
    result.push_back(int32 >> 24);
    result.push_back(int32 >> 16);
    result.push_back(int32 >> 8);
    result.push_back(int32);
    return result;
  }

  static inline std::vector<char> packVectorNInt32(std::vector<int32_t> vector_n_int32)
  {
    std::vector<char> result;
    for (auto i : vector_n_int32)
    {
      result.push_back(i >> 24);
      result.push_back(i >> 16);
      result.push_back(i >> 8);
      result.push_back(i);
    }

    return result;
  }

  static inline std::vector<char> packVectorNd(std::vector<double> vector_nd)
  {
    std::vector<char> output;

    for (auto d : vector_nd)
    {
      union temp
      {
        double value;
        char c[8];
      } in, out;

      in.value = d;
      out.c[0] = in.c[7];
      out.c[1] = in.c[6];
      out.c[2] = in.c[5];
      out.c[3] = in.c[4];
      out.c[4] = in.c[3];
      out.c[5] = in.c[2];
      out.c[6] = in.c[1];
      out.c[7] = in.c[0];

      for (auto const &character : out.c)
        output.push_back(character);
    }

    return output;
  }

  static inline std::vector<char> packDouble(double d)
  {
    std::vector<char> output;
    union temp
    {
      double value;
      char c[8];
    } in, out;

    in.value = d;
    out.c[0] = in.c[7];
    out.c[1] = in.c[6];
    out.c[2] = in.c[5];
    out.c[3] = in.c[4];
    out.c[4] = in.c[3];
    out.c[5] = in.c[2];
    out.c[6] = in.c[1];
    out.c[7] = in.c[0];

    for (auto const &character : out.c)
      output.push_back(character);

    return output;
  }

  static inline std::vector<double> unpackVector3d(const std::vector<char> &data, uint32_t &message_offset)
  {
    std::vector<double> vector_3d;
    for (unsigned int i = 0; i < 3; i++)
    {
      double d = getDouble(data, message_offset);
      vector_3d.push_back(d);
    }
    return vector_3d;
  }

  static inline std::vector<double> unpackVector6d(const std::vector<char> &data, uint32_t &message_offset)
  {
    std::vector<double> vector_6d;
    for (unsigned int i = 0; i < 6; i++)
    {
      double d = getDouble(data, message_offset);
      vector_6d.push_back(d);
    }
    return vector_6d;
  }

  static inline std::vector<int32_t> unpackVector6Int32(const std::vector<char> &data, uint32_t &message_offset)
  {
    std::vector<int32_t> vector_6_int32;
    for (unsigned int i = 0; i < 6; i++)
    {
      int32_t int32_value = getInt32(data, message_offset);
      vector_6_int32.push_back(int32_value);
    }
    return vector_6_int32;
  }

  static inline double getDouble(const std::vector<char> &data, uint32_t &message_offset)
  {
    double output;

    ((char *)(&output))[7] = data[message_offset];
    ((char *)(&output))[6] = data[message_offset + 1];
    ((char *)(&output))[5] = data[message_offset + 2];
    ((char *)(&output))[4] = data[message_offset + 3];
    ((char *)(&output))[3] = data[message_offset + 4];
    ((char *)(&output))[2] = data[message_offset + 5];
    ((char *)(&output))[1] = data[message_offset + 6];
    ((char *)(&output))[0] = data[message_offset + 7];

    message_offset += 8;
    return output;
  }

  static inline uint32_t getUInt32(const std::vector<char> &data, uint32_t &message_offset)
  {
    uint32_t output = 0;
    ((char *)(&output))[3] = data[message_offset];
    ((char *)(&output))[2] = data[message_offset + 1];
    ((char *)(&output))[1] = data[message_offset + 2];
    ((char *)(&output))[0] = data[message_offset + 3];
    message_offset += 4;

    return output;
  }

  static inline uint16_t getUInt16(const std::vector<char> &data, uint32_t &message_offset)
  {
    uint16_t output = 0;
    ((char *)(&output))[1] = data[message_offset + 0];
    ((char *)(&output))[0] = data[message_offset + 1];
    message_offset += 2;

    return output;
  }

  static inline int32_t getInt32(const std::vector<char> &data, uint32_t &message_offset)
  {
    int32_t output = 0;
    ((char *)(&output))[3] = data[message_offset];
    ((char *)(&output))[2] = data[message_offset + 1];
    ((char *)(&output))[1] = data[message_offset + 2];
    ((char *)(&output))[0] = data[message_offset + 3];
    message_offset += 4;
    return output;
  }

  static inline uint64_t getUInt64(const std::vector<char> &data, uint32_t &message_offset)
  {
    uint64_t output;

    ((char *)(&output))[7] = data[message_offset];
    ((char *)(&output))[6] = data[message_offset + 1];
    ((char *)(&output))[5] = data[message_offset + 2];
    ((char *)(&output))[4] = data[message_offset + 3];
    ((char *)(&output))[3] = data[message_offset + 4];
    ((char *)(&output))[2] = data[message_offset + 5];
    ((char *)(&output))[1] = data[message_offset + 6];
    ((char *)(&output))[0] = data[message_offset + 7];

    message_offset += 8;
    return output;
  }

  static inline unsigned char getUChar(const std::vector<char> &data, uint32_t &message_offset)
  {
    unsigned char output = data[message_offset];
    message_offset += 1;
    return output;
  }

  static inline uint8_t getUInt8(const std::vector<char> &data, uint32_t &message_offset)
  {
    uint8_t output = data[message_offset];
    message_offset += 1;
    return output;
  }

  static inline std::string double2hexstr(double x)
  {
    union
    {
      long long i;
      double d;
    } value;

    value.d = x;

    std::ostringstream buf;
    buf << std::hex << std::setw(6) << value.i;
    return buf.str();
  }

  static inline std::vector<char> hexToBytes(const std::string &hex)
  {
    std::vector<char> bytes;

    for (unsigned int i = 0; i < hex.length(); i += 2)
    {
      std::string byteString = hex.substr(i, 2);
      char byte = (char)strtol(byteString.c_str(), nullptr, 16);
      bytes.push_back(byte);
    }

    return bytes;
  }

  static inline std::ostream &hexDump(std::ostream &o, char const *p, std::size_t size)
  {
    o << std::hex << std::setw(2) << std::setfill('0');
    while (size--)
      o << (static_cast<unsigned int>(*p++) & 0xff) << ' ';
    return o;
  }

  static inline std::vector<std::string> split(const std::string &s, char delimiter)
  {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
      tokens.push_back(token);
    }
    return tokens;
  }

  template <typename T>
  bool isWithinBounds(const T &value, const T &low, const T &high)
  {
    return (low <= value && value <= high);
  }

  static bool isNumber(const std::string &s)
  {
    return !s.empty() && std::find_if(s.begin(), s.end(), [](unsigned char c) { return !std::isdigit(c); }) == s.end();
  }

  static void preciseSleep(double seconds)
  {
    using namespace std;
    using namespace std::chrono;

    static double estimate = 5e-3;
    static double mean = 5e-3;
    static double m2 = 0;
    static int64_t count = 1;

    while (seconds > estimate)
    {
      auto start = high_resolution_clock::now();
      std::this_thread::sleep_for(milliseconds(1));
      auto end = high_resolution_clock::now();

      double observed = duration<double>(end - start).count();
      seconds -= observed;

      ++count;
      double delta = observed - mean;
      mean += delta / static_cast<double>(count);
      m2 += delta * (observed - mean);
      double stddev = sqrt(m2 / (static_cast<double>(count - 1)));
      estimate = mean + stddev;
    }

    // spin lock
    auto start = high_resolution_clock::now();
    while (duration<double>((high_resolution_clock::now() - start)).count() < seconds)
      ;
  }

#if defined(__linux__) || defined(__APPLE__)
  static timespec timepointToTimespec(std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> tp)
  {
    auto secs = std::chrono::time_point_cast<std::chrono::seconds>(tp);
    auto ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(tp) -
              std::chrono::time_point_cast<std::chrono::nanoseconds>(secs);

    return timespec{secs.time_since_epoch().count(), ns.count()};
  }
#endif

  static void waitPeriod(const std::chrono::steady_clock::time_point &t_cycle_start, double dt)
  {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
    using namespace std::chrono;
    auto t_app_stop = steady_clock::now();
    auto t_app_duration = duration<double>(t_app_stop - t_cycle_start);
    if (t_app_duration.count() < dt)
    {
      preciseSleep(dt - t_app_duration.count());
    }
#elif defined(__APPLE__)
    using namespace std::chrono;
    auto t_app_stop = steady_clock::now();
    auto t_app_duration = duration<double>(t_app_stop - t_cycle_start);
    if (t_app_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_app_duration.count()));
    }
#else
    using namespace std::chrono;
    auto t_app_stop = steady_clock::now();
    auto t_app_duration = duration<double>(t_app_stop - t_cycle_start);
    if (t_app_duration.count() < dt)
    {
      auto cycle_time_in_ms = static_cast<int64_t>(dt * 1000);
      auto t_cycle_ideal_end = t_cycle_start + milliseconds(cycle_time_in_ms);
      auto t_cycle_end_with_slack = t_cycle_ideal_end - (microseconds(SLACK_TIME_IN_MICROS) +
                                                         t_app_duration);

      struct timespec tv_cycle_end_with_slack{}, tv_cycle_end{}, curr{};
      tv_cycle_end = timepointToTimespec(time_point_cast<nanoseconds>(t_cycle_ideal_end));
      tv_cycle_end_with_slack = timepointToTimespec(time_point_cast<nanoseconds>(t_cycle_end_with_slack));
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tv_cycle_end_with_slack, NULL);

      clock_gettime(CLOCK_MONOTONIC, &curr);
      for (; curr.tv_nsec < tv_cycle_end.tv_nsec; clock_gettime(CLOCK_MONOTONIC, &curr))
        ;
    }
#endif
  }

  static bool isRealtimeKernelAvailable()
  {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
    return true;
#else
    std::ifstream realtime("/sys/kernel/realtime", std::ios_base::in);
    if (realtime.good())
    {
      bool is_realtime;
      realtime >> is_realtime;
      return is_realtime;
    }
    else
    {
      return false;
    }
#endif
  }

  static bool setRealtimePriority(int priority)
  {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
    auto get_last_windows_error = []() -> std::string {
      DWORD error_id = GetLastError();
      LPSTR buffer = nullptr;
        size_t size = FormatMessageA(
          FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
          nullptr, error_id, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)(&buffer), 0, nullptr);
      return std::string(buffer, size);
    };

    if (priority == 0)
    {
      // priority not set explicitly by user, assume that max. priority is desired.
      priority = THREAD_PRIORITY_TIME_CRITICAL;
    }

    if (!SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS))
    {
      std::cerr << "ur_rtde: unable to set priority for the process: " << get_last_windows_error() << std::endl;
      return false;
    }

    if (!SetThreadPriority(GetCurrentThread(), priority))
    {
      std::cerr << "ur_rtde: unable to set priority for the thread: " << get_last_windows_error() << std::endl;
      return false;
    }
    return true;
#else
    if (priority < 0)
    {
      std::cout << "ur_rtde: realtime priority less than 0 specified, realtime priority will not be set on purpose!" <<
          std::endl;
      return false;
    }

    if (priority == 0)
    {
      // priority not set explicitly by user, assume that a fair max. priority is desired.
      const int thread_priority = sched_get_priority_max(SCHED_FIFO);
      if (thread_priority == -1)
      {
        std::cerr << "ur_rtde: unable to get maximum possible thread priority: " << strerror(errno) <<
            std::endl;
        return false;
      }
      // the priority is capped at 90, since any higher value would make the OS too unstable.
      priority = std::min(90, std::max(0, thread_priority));
    }

    sched_param thread_param{};
    thread_param.sched_priority = priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0)
    {
      std::cerr << "ur_rtde: unable to set realtime scheduling: " << strerror(errno) << std::endl;
      return false;
    }
    return true;
#endif
  }

};

}  // namespace ur_rtde

#endif  // RTDE_RTDE_UTILITY_H
