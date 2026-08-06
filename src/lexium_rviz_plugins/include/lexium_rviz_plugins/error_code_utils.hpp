#ifndef LEXIUM_RVIZ_PLUGINS__ERROR_CODE_UTILS_HPP_
#define LEXIUM_RVIZ_PLUGINS__ERROR_CODE_UTILS_HPP_

#include <algorithm>
#include <cctype>
#include <string>

namespace lexium_rviz_plugins
{

inline std::string trimCopy(const std::string & value)
{
  const auto start = value.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(start, end - start + 1);
}

inline std::string lowerCopy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return value;
}

inline bool isNoErrorCode(const std::string & code)
{
  const std::string trimmed = trimCopy(code);
  if (trimmed.empty()) {
    return true;
  }

  const std::string normalized = lowerCopy(trimmed);
  if (normalized == "0" || normalized == "ok" || normalized == "none") {
    return true;
  }

  if (normalized.rfind("0x", 0) == 0) {
    const std::string digits = normalized.substr(2);
    return !digits.empty() &&
           std::all_of(digits.begin(), digits.end(), [](char c) { return c == '0'; });
  }

  return false;
}

}  // namespace lexium_rviz_plugins

#endif  // LEXIUM_RVIZ_PLUGINS__ERROR_CODE_UTILS_HPP_
