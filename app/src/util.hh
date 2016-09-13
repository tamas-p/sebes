#ifndef APP_SRC_UTIL_H_
#define APP_SRC_UTIL_H_

//------------------------------------------------------------------------------

#include <string>

//------------------------------------------------------------------------------

bool parse_str(char separator,
               const std::string& str,
               std::string* first,
               std::string* second);

bool parse_dicom_host(const std::string& str,
                      std::string* aet,
                      std::string* host);

bool parse_host(const std::string& str,
                std::string* host,
                std::string* port);

bool parse_port(const std::string& str,
                std::string* port);

void set_tcp_buffer_length(const std::string& length);

//------------------------------------------------------------------------------

#endif  // APP_SRC_UTIL_H_
