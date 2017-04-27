// Copyright 2016 Tamas Palagyi

#include <getopt.h>

#include "imagestore.hh"
#include "dicom.hh"
#include "dcmtk.hh"
#include "util.hh"

#include <easylogging++.h>
INITIALIZE_EASYLOGGINGPP

//------------------------------------------------------------------------------

void print_usage_and_exit(const std::string& name) {
  std::cout << "Usage: "
            << name
            << " --host=aet@host:port"
            << " --rhost=raet@rhost:rport"
            << " --xfer=transfer_syntax_uid"
            << " --level=study|series|image"
            << " [--study=study_uid] [--series=series_uid] [--instance=sop_instance_uid]"
            << " [--tcpbuffer=length]"
            << " [--norsp]" << std::endl;

  exit(-1);
}

//------------------------------------------------------------------------------

struct Arguments {
  std::string aet_;
  std::string host_;
  std::string port_;

  std::string raet_;
  std::string rhost_;

  std::string xfer_;

  std::string level_;
  std::string instance_uid_;
  std::string series_uid_;
  std::string study_uid_;
  std::string tcp_buffer_length_;
  bool need_response_;
  Arguments()
    : tcp_buffer_length_("512000"),
      need_response_(true)
  {}
};

//------------------------------------------------------------------------------

void cmdparse(int argc,
              char** argv,
              Arguments* arguments) {
  std::string name = basename(argv[0]);
  while (1) {
    const char* options = "v:h:r:x:l:i:s:e:n";
    static struct option long_options[] = {
      {"v", optional_argument, 0, 'v'},
      {"host", required_argument, 0, 'h'},
      {"rhost", required_argument, 0, 'r'},
      {"xfer", required_argument, 0, 'x'},
      {"level", required_argument, 0, 'l'},
      {"instance", required_argument, 0, 'i'},
      {"series", required_argument, 0, 's'},
      {"study", required_argument, 0, 'e'},
      {"tcpbuffer", required_argument, 0, 'b'},
      {"norsp", no_argument, 0, 'n'},
      {0, 0, 0, 0}
    };

    // getopt_long stores the option index here.
    int option_index = 0;

    int c = getopt_long(argc, argv, options, long_options, &option_index);

    // Detect the end of the options.
    if (c == -1)
      break;

    switch (c) {
    case 'h':
      parse_dicom_host(optarg,
                       &arguments->aet_,
                       &arguments->host_);

      parse_port(arguments->host_,
                 &arguments->port_);
      break;

    case 'r':
      parse_dicom_host(optarg,
                       &arguments->raet_,
                       &arguments->rhost_);
      break;

    case 'x':
      arguments->xfer_ = optarg;
      break;

    case 'l':
      arguments->level_ = optarg;
      break;

    case 'i':
      arguments->instance_uid_ = optarg;
      break;

    case 's':
      arguments->series_uid_ = optarg;
      break;

    case 'e':
      arguments->study_uid_ = optarg;
      break;

    case 'b':
      arguments->tcp_buffer_length_ = optarg;
      break;

    case 'n':
      arguments->need_response_ = false;
      break;

    case 'v':
      break;

    case ':':
      fprintf(stderr, "%s: option '-%c' requires an argument\n", name.c_str(), optopt);
      print_usage_and_exit(name);

    case '?':
      fprintf(stderr, "%s: option '-%c' is unknown\n", name.c_str(), optopt);
      print_usage_and_exit(name);

    default:
      std::cerr << "Unexpected character from getopt_long, exiting." << std::endl;
      print_usage_and_exit(name);
    }
  }

  // int num_uids = 0;
  // if (!arguments->instance_uid_.empty()) num_uids++;
  // if (!arguments->series_uid_.empty()) num_uids++;
  // if (!arguments->study_uid_.empty()) num_uids++;

  // Check that everything we need is set.
  if (arguments->aet_.empty() ||
      arguments->host_.empty() ||
      arguments->port_.empty() ||
      arguments->raet_.empty() ||
      arguments->rhost_.empty() ||
      arguments->xfer_.empty() ||
      arguments->level_.empty())
    print_usage_and_exit(name);
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
  START_EASYLOGGINGPP(argc, argv);
  TIMED_FUNC(timerObj);

  /* el::Configurations conf("easylogging.conf");
  el::Loggers::reconfigureAllLoggers(conf);
  el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format,
                                     "%datetime %level %file:%line : %msg"); */

  DicomDcmtk::init_codec();

  Arguments arguments;
  cmdparse(argc, argv, &arguments);

  Dicom::QueryLevel query_level = Dicom::STUDY;
  if (arguments.level_.compare("image") == 0) {
    query_level = Dicom::IMAGE;
  } else if (arguments.level_.compare("series") == 0) {
    query_level = Dicom::SERIES;
  } else if (arguments.level_.compare("study") == 0) {
    query_level = Dicom::STUDY;
  } else {
    print_usage_and_exit(basename(argv[0]));
  }

  set_tcp_buffer_length(arguments.tcp_buffer_length_);
  LOG(INFO) << basename(argv[0]) << " has been started...";

  Dicom* dicom = new DicomDcmtk(arguments.aet_,
                                arguments.host_,
                                std::stoi(arguments.port_),
                                arguments.need_response_);

  dicom->movescu_execute(arguments.raet_,
                         arguments.rhost_,
                         query_level,
                         arguments.study_uid_,
                         arguments.series_uid_,
                         arguments.instance_uid_,
                         arguments.xfer_);

  return 0;
}

//------------------------------------------------------------------------------
