// Copyright 2016 Tamas Palagyi
//------------------------------------------------------------------------------

#include <getopt.h>

#include <easylogging++.h>
INITIALIZE_EASYLOGGINGPP

#include "dcmtk.hh"
#include "imagestore.hh"
#include "util.hh"

//------------------------------------------------------------------------------

void print_usage_and_exit(const std::string& name) {
  std::cout << "Usage: "
            << name
            << " --host=aet@host:port"
            << " --rhosts=aet1@host1:port1,aet2@host2:port2,..."
            << " [--xfers=transfer_syntax_uid1,transfer_syntax_ui2,...]"
            << " --imagedir=/path/to/dicom/images"
            << " [--tcpbuffer=length]"
            << " [--norsp]"
            << " [--multi]" << std::endl;
  exit(-1);
}

//------------------------------------------------------------------------------

struct Arguments {
  std::string aet_;
  std::string host_;
  std::string port_;

  AetMap rhosts_;

  Xfers xfers_;

  std::string imagedir_;
  bool multi_;
  std::string tcp_buffer_length_;
  bool need_response_;
  Arguments() : multi_(false), need_response_(true) {}
};

//------------------------------------------------------------------------------

void parse_rhosts(AetMap* rhosts, const std::string& optarg) {
  std::istringstream ss(optarg);
  std::string rhost;
  while (std::getline(ss, rhost, ',')) {
    std::string aet;
    std::string host;
    if (parse_dicom_host(rhost,
                         &aet,
                         &host)) {
      (*rhosts)[aet] = host;
    }
  }
}

//------------------------------------------------------------------------------

void parse_xfers(Xfers* xfers, const std::string& optarg) {
  std::istringstream ss(optarg);
  std::string xfer;
  while (std::getline(ss, xfer, ',')) {
    xfers->insert(xfer);
  }
}

//------------------------------------------------------------------------------

void cmdparse(int argc,
              char** argv,
              Arguments* arguments) {
  std::string name = basename(argv[0]);
  while (1) {
    const char* options = "v:h:r:d:nm";
    static struct option long_options[] = {
      {"v", optional_argument, 0, 'v'},
      {"host", required_argument, 0, 'h'},
      {"rhosts", required_argument, 0, 'r'},
      {"xfers", required_argument, 0, 'x'},
      {"imagedir", required_argument, 0, 'd'},
      {"tcpbuffer", required_argument, 0, 'b'},
      {"norsp", no_argument, 0, 'n'},
      {"multi", no_argument, 0, 'm'},
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
      parse_rhosts(&arguments->rhosts_, optarg);
      break;

    case 'x':
      parse_xfers(&arguments->xfers_, optarg);
      break;

    case 'd':
      arguments->imagedir_ = optarg;
      break;

    case 'm':
      arguments->multi_ = false;
      break;

    case 'b':
      arguments->tcp_buffer_length_ = optarg;
      break;

    case 'n':
      arguments->need_response_ = false;
      break;

    case 'v':
      std::cout << "HELLO" << std::endl;
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

  // Check that everything we need is set.
  if (arguments->port_.empty() ||
      arguments->imagedir_.empty() ||
      arguments->aet_.empty() ||
      arguments->rhosts_.empty() ) {
    print_usage_and_exit(name);
  }
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
  /* el::Configurations conf("easylogging.conf");
  el::Loggers::reconfigureAllLoggers(conf);
  el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format,
  "%datetime %level %file:%line : %msg"); */

  START_EASYLOGGINGPP(argc, argv);

  Arguments arguments;
  cmdparse(argc, argv, &arguments);
  set_tcp_buffer_length(arguments.tcp_buffer_length_);
  LOG(INFO) << basename(argv[0]) << " has been started...";

  if (arguments.xfers_.empty()) {
    LOG(INFO) << "Adding default list of Xfers.";
    arguments.xfers_.insert(UID_LittleEndianExplicitTransferSyntax);
    arguments.xfers_.insert(UID_BigEndianExplicitTransferSyntax);
    arguments.xfers_.insert(UID_LittleEndianImplicitTransferSyntax);
    arguments.xfers_.insert(UID_JPEGLSLosslessTransferSyntax);
    arguments.xfers_.insert(UID_JPEGProcess14SV1TransferSyntax);
    arguments.xfers_.insert(UID_RLELosslessTransferSyntax);
    arguments.xfers_.insert(UID_JPEG2000TransferSyntax);
    arguments.xfers_.insert(UID_JPEG2000LosslessOnlyTransferSyntax);
  }

  const ImageStore* imagestore = new ImageStore(arguments.imagedir_, arguments.xfers_);
  imagestore->print();

  Dicom* dicom = new DicomDcmtk(std::stoi(arguments.port_),
                                arguments.aet_,
                                imagestore,
                                &arguments.rhosts_,
                                arguments.multi_,
                                arguments.need_response_);
  dicom->init();

  signal(SIGCHLD, SIG_IGN);
  while (true) {
    dicom->waitfor_association();
    pid_t pid = fork();
    if (pid < 0) {
      LOG(ERROR) << "Could not create child process.";
      dicom->cleanup();
    } else {
      if (pid == 0) {
        LOG(INFO) << "Executing work in child";
        dicom->process_request();
        return 0;
      } else {
        LOG(INFO) << "Cleaning up the parent";
        dicom->cleanup();
      }
    }
  }

  return 0;
}

//------------------------------------------------------------------------------
