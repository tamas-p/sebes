// Copyright 2016 Tamas Palagyi

//------------------------------------------------------------------------------

#include <string>
#include <iostream>

#include "util.hh"

//------------------------------------------------------------------------------

bool parse_str(char separator,
               const std::string& str,
               std::string* first,
               std::string* second) {
  size_t pos = str.find_first_of(separator);
  if (pos == std::string::npos) {
    return false;
  }

  *first = str.substr(0, pos);
  *second = str.substr(pos + 1, std::string::npos);
  return true;
}

//------------------------------------------------------------------------------

bool parse_dicom_host(const std::string& str,
                      std::string* aet,
                      std::string* host) {
  return parse_str('@', str, aet, host);
}

//------------------------------------------------------------------------------

bool parse_host(const std::string& str,
                std::string* host,
                std::string* port) {
  return parse_str(':', str, host, port);
}

//------------------------------------------------------------------------------

bool parse_port(const std::string& str,
                std::string* port) {
  std::string host;
  return parse_str(':', str, &host, port);
}

//------------------------------------------------------------------------------
