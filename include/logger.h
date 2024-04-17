/*************************************************************************************************************************
 * Copyright 2023 Xidian114
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************************************************************/

#pragma once

#include <cstdlib>
#include <map>
#include <memory>   // std::make_unique
#include <sstream>  // std::stringstream
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

// make_unique
template <typename T>
using Ele = typename std::enable_if<!std::is_array<T>::value, std::unique_ptr<T>>::type;

template <typename T>
using Slice = typename std::enable_if<std::is_array<T>::value && std::extent<T>::value == 0, std::unique_ptr<T>>::type;

template <typename T>
using Arr = typename std::enable_if<std::extent<T>::value != 0, void>::type;

template <typename T, typename... Args>
inline Ele<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template <typename T>
inline Slice<T> make_unique(size_t size) {
  using U = typename std::remove_extent<T>::type;
  return std::unique_ptr<T>(new U[size]);
}

template <typename T, typename... Args>
Arr<T> make_unique(Args&&...) = delete;

// string_utilss
template <typename... Args>
inline std::string fmtstr(const std::string& format, Args... args) {
  // This function came from a code snippet in stackoverflow under cc-by-1.0
  //   https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf

  // Disable format-security warning in this function.
#if defined(_MSC_VER)  // for visual studio
#pragma warning(push)
#pragma warning(warning(disable : 4996))
#elif defined(__GNUC__) || defined(__clang__)  // for gcc or clang
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
#endif
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;  // Extra space for '\0'
  if (size_s <= 0) {
    throw std::runtime_error("Error during formatting.");
  }
  auto size = static_cast<size_t>(size_s);
  auto buf = make_unique<char[]>(size);
  std::snprintf(buf.get(), size, format.c_str(), args...);
#if defined(_MSC_VER)
#pragma warning(pop)
#elif defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
  return std::string(buf.get(), buf.get() + size - 1);  // We don't want the '\0' inside
}

template <typename T>
inline std::string vec2str(std::vector<T> vec) {
  std::stringstream ss;
  ss << "(";
  if (!vec.empty()) {
    for (size_t i = 0; i < vec.size() - 1; ++i) {
      ss << vec[i] << ", ";
    }
    ss << vec.back();
  }
  ss << ")";
  return ss.str();
}

class MLogger {
 public:
  enum Level { LEVEL_TRACE = 0, LEVEL_DEBUG = 10, LEVEL_INFO = 20, LEVEL_WARNING = 30, LEVEL_ERROR = 40 };

  static MLogger& getLogger() {
    static MLogger instance;
    return instance;
  }
  MLogger(MLogger const&) = delete;
  void operator=(MLogger const&) = delete;

  template <typename... Args>
  void log(const Level level, const std::string format, const Args&... args) {
    if (level_ <= level) {
      std::string fmt = getPrefix(level) + format + "\n";
      FILE* out = level_ < LEVEL_WARNING ? stdout : stderr;
      std::string logstr = fmtstr(fmt, args...);
      fprintf(out, "%s", logstr.c_str());
    }
  }

  template <typename... Args>
  void log(const Level level, const int rank, const std::string format, const Args&... args) {
    if (level_ <= level) {
      std::string fmt = getPrefix(level, rank) + format + "\n";
      FILE* out = level_ < LEVEL_WARNING ? stdout : stderr;
      std::string logstr = fmtstr(fmt, args...);
      fprintf(out, "%s", logstr.c_str());
    }
  }

  void setLevel(const Level level) {
    level_ = level;
    log(LEVEL_INFO, "Set logger level by %s", getLevelName(level).c_str());
  }

 private:
  const std::string PREFIX = "[XDU]";
  std::map<Level, std::string> level_name_ = {{LEVEL_TRACE, "LEVEL_TRACE"},
                                              {LEVEL_DEBUG, "LEVEL_DEBUG"},
                                              {LEVEL_INFO, "LEVEL_INFO"},
                                              {LEVEL_WARNING, "LEVEL_WARNING"},
                                              {LEVEL_ERROR, "LEVEL_ERROR"}};

#ifndef NDEBUG
  const Level DEFAULT_LOG_LEVEL = LEVEL_DEBUG;
#else
  const Level DEFAULT_LOG_LEVEL = LEVEL_INFO;
#endif
  Level level_ = DEFAULT_LOG_LEVEL;

  MLogger() {
    char* level_name = std::getenv("LOG_LEVEL");
    if (level_name != nullptr) {
      std::map<std::string, Level> name_to_level = {
          {"LEVEL_TRACE", LEVEL_TRACE},     {"LEVEL_DEBUG", LEVEL_DEBUG}, {"LEVEL_INFO", LEVEL_INFO},
          {"LEVEL_WARNING", LEVEL_WARNING}, {"LEVEL_ERROR", LEVEL_ERROR},
      };
      auto level = name_to_level.find(level_name);
      if (level != name_to_level.end()) {
        setLevel(level->second);
      } else {
        fprintf(stderr,
                "[XDU][LEVEL_WARNING] Invalid logger level LOG_LEVEL=%s. "
                "Ignore the environment variable and use a default "
                "logging level.\n",
                level_name);
        level_name = nullptr;
      }
    }
  }

  inline std::string getLevelName(const Level level) { return level_name_[level]; }

  inline std::string getPrefix(const Level level) { return PREFIX + "[" + getLevelName(level) + "] "; }

  inline std::string getPrefix(const Level level, const int rank) {
    return PREFIX + "[" + getLevelName(level) + "][" + std::to_string(rank) + "] ";
  }
};

#define MLOG(level, ...) MLogger::getLogger().log(level, __VA_ARGS__)
#define MLOG_TRACE(...) MLOG(MLogger::LEVEL_TRACE, __VA_ARGS__)
#define MLOG_DEBUG(...) MLOG(MLogger::LEVEL_DEBUG, __VA_ARGS__)
#define MLOG_INFO(...) MLOG(MLogger::LEVEL_INFO, __VA_ARGS__)
#define MLOG_WARNING(...) MLOG(MLogger::LEVEL_WARNING, __VA_ARGS__)
#define MLOG_ERROR(...) MLOG(MLogger::LEVEL_ERROR, __VA_ARGS__)
